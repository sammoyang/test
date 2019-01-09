/****************************************************************************/
/* MODULE:      I2C.c                                                       */
/****************************************************************************/

#define     DRIVER
#include    "global.h"
#include    "pgmspace.h"
#include    "I2C.h"
#include    "I2C_hw.h"
#include    "timer2.h"
#include    "schedule.h"
#include    "MTV048.h"

// Attempt to recover from lost START bits - usually due to a missing I2C
// device - like a projector.
#define RECOVER_LOST_START              1
#if ((8 * I2C_TIMER2_SLOW_OCR) > 255)
#define     I2C_START_TIMEOUT           255
#else
#define     I2C_START_TIMEOUT           (8 * I2C_TIMER2_SLOW_OCR)
#endif

#define I2CReqSize( data_size )                 \
        union  I2CReqSize {                     \
                I2CReqStruct    req;            \
                uint8_t         filler[sizeof(I2CReqStruct) + data_size - 2]; \
              }

enum { I2C_IDLE, I2C_START_SENT, I2C_RESTART, I2C_ADDRESS, I2C_READING,
       I2C_WRITING, I2C_STOP, I2C_COMPLETE, I2C_RETRY, I2C_SLEEP, I2C_FAIL };

volatile uint8_t    i2c_state;     // ISR state machine 
volatile uint8_t    i2c_nattempts; //Counts down retries from I2C address phase
volatile uint8_t    i2c_nbytes;    // Counts down number of bytes to transfer
volatile uint8_t    i2c_speed_bus;      // 
volatile uint8_t    i2c_indirect;
volatile uint8_t   *i2c_dataptr;
volatile I2CReqPtr  i2c_queue;

// Enable I2C interface (TWEN), Enable I2C interrupt (TWIE),
// and Enable I2C interface ack (TWEA), and clear the I2C int (TWINT)
inline void I2C_Enable()
{
    outb(TWCR, _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA));
}

// Disable I2C interface (TWEN), disable I2C interrupt (TWIE),
// and clear the I2C int (TWINT) just in case
inline void I2C_Disable()
{
    outb(TWCR, _BV(TWINT));
}

inline void I2C_Clear_Int()
{
    outb(TWCR, (inb(TWCR) & I2C_COMMAND_MASK) | _BV(TWINT));
}

inline void I2C_Recv_Ack()
{
    outb(TWCR, (inb(TWCR) & I2C_COMMAND_MASK) | _BV(TWINT) | _BV(TWEA));
}

inline void I2C_Recv_Nack()
{
    outb(TWCR, (inb(TWCR) & I2C_COMMAND_MASK) | _BV(TWINT));
}

inline void I2C_Send_Stop()
{
    outb(TWCR, (inb(TWCR) & I2C_COMMAND_MASK) | _BV(TWINT) | _BV(TWEA) | _BV(TWSTO));
}

inline void I2C_Send_Start()
{
    outb(TWCR, (inb(TWCR) & I2C_COMMAND_MASK) | _BV(TWINT) | _BV(TWEA) | _BV(TWSTA));
#if RECOVER_LOST_START
    Timer2_Set( I2C_START_TIMEOUT, I2C_TIMER2_SLOW_SCALE );
#endif  // RECOVER_LOST_START
}

inline void I2C_Select_Bus( uint8_t bus_num )
{
    if (bus_num & 0x01)
        sbi(I2C_BUS_SEL_PORT, I2C_BUS_SEL_LSB );
    else
        cbi(I2C_BUS_SEL_PORT, I2C_BUS_SEL_LSB );
    if (bus_num & 0x02)
        sbi(I2C_BUS_SEL_PORT, I2C_BUS_SEL_LSB + 1);
    else
        cbi(I2C_BUS_SEL_PORT, I2C_BUS_SEL_LSB + 1);
    i2c_speed_bus = (i2c_speed_bus & 0xF0) | bus_num;
}


inline void I2C_Select_Speed( uint8_t bus_speed )
{
    if (bus_speed == I2C_SPEED_FAST) {
        // Set the prescaler to 4^0 = 1 (mega128 & newer)
        TWSR = I2C_SCALER_FAST;
        // Set the bit rate to fast (100K bps)
        TWBR = I2C_BR_FAST;
        i2c_speed_bus = (i2c_speed_bus & 0x0F) | (I2C_SPEED_FAST << 4);
    } else {    // (bus_speed == I2C_SPEED_SLOW)
        // Set the prescaler to 4^3 = 64
        TWSR = I2C_SCALER_SLOW;
        // Set the bit rate to slow (225 bps)
        TWBR = I2C_BR_SLOW;
        i2c_speed_bus = (i2c_speed_bus & 0x0F) | (I2C_SPEED_SLOW << 4);
    }
}


// This must ONLY be called while interrupts are disabled, like during
// an ISR or after explicitly disabling interrupts.
void Check_And_Restart_Queue( void )
{
    if ((i2c_queue) && (i2c_state == I2C_IDLE)) {
        if (i2c_speed_bus != (i2c_queue->Control & I2C_SPEED_BUS_MASK)) {
            I2C_Disable();
            I2C_Select_Bus( i2c_queue->Control & 0x0F );
            I2C_Select_Speed(( i2c_queue->Control & ~I2C_INDIRECT_DATA) >> 4 );
            // Wait for timer to wake us
            i2c_state = I2C_SLEEP;
            // Delay to settle the bus
            Timer2_Set( I2C_TIMER2_SLOW_OCR, I2C_TIMER2_SLOW_SCALE );
        } else {
            // We may be disabled from switching bus and/or speed
            I2C_Enable();
            I2C_Send_Start();
            i2c_nattempts = I2C_MAX_ATTEMPTS;
            i2c_state = I2C_START_SENT;
        }
    }
}


void I2C_Timer( void )
{
    Timer2_Reset();
    if ((i2c_state == I2C_STOP) ||  // Assume that stop is completed
        (i2c_state == I2C_SLEEP))   // Time to wake up
        i2c_state = I2C_IDLE;

    if (i2c_state == I2C_RETRY) {
        I2C_Send_Start();
        i2c_state = I2C_START_SENT;
#if  RECOVER_LOST_START
    } else if ((i2c_state == I2C_START_SENT) ||
               (i2c_state == I2C_RESTART)) {
        if (i2c_queue) {
            i2c_queue->Status = I2C_STATUS_INCOMPLETE;
            i2c_queue   = (volatile I2CReqPtr) i2c_queue->Link;
            i2c_nbytes  = 0;
            i2c_dataptr = NULL;
        }
        I2C_Send_Stop();
        i2c_state = I2C_STOP;
        Timer2_Set( I2C_TIMER2_SLOW_OCR, I2C_TIMER2_SLOW_SCALE );
#endif  // RECOVER_LOST_START
    } else {
        Check_And_Restart_Queue();
    }
}


void    I2C_Init( void ) {

//    I2C_Select_Speed( I2C_SPEED_FAST );
    I2C_Select_Speed( I2C_SPEED_SLOW );


#if DEBUG_I2C
    // Enable pullup resistors on the pins if these is nothing else
    sbi(PORTD, 0);    // SCL
    sbi(PORTD, 1);    // SDA
#else   // DEBUG_I2C
    // Disable pullup resistors on the pins
    cbi(PORTD, 0);    // SCL
    cbi(PORTD, 1);    // SDA
#endif  // DEBUG_I2C

    // Initialize the I2C bus selection bits
    I2C_Select_Bus( 0x00 );   // Select bus 0
    sbi(I2C_BUS_SEL_DDR, I2C_BUS_SEL_LSB);        // LSB set to output
    sbi(I2C_BUS_SEL_DDR, I2C_BUS_SEL_LSB + 1);    // MSB set to output

    // MSB LSB
    //  0   0   MOD
    //  0   1   PROG
    //  1   0   HDCP_MS
    //  1   1   PLAIN/5V  SD_IN (SAA7118), LD (FL12200)
    // Plus, everybody's favorite: RAW SDA/SCL, always connected

    // Enable I2C interface
    I2C_Enable();

    // Initialize the ISR state machine
    i2c_state     = I2C_IDLE;
    i2c_nattempts = 0;
    i2c_nbytes    = 0;
    i2c_dataptr   = NULL;
    i2c_queue     = NULL;
    i2c_indirect  = 0;

    Timer2_Init( &I2C_Timer );

  // Remember to enable interrupts
}


// This is the I2C Interrupt Service Routine for Master Mode only; 
// the remaining question is whether an interrupt is generated after
// a STOP bit is sent.


#ifdef  __GNUC__
SIGNAL (SIG_2WIRE_SERIAL)
#else
#pragma vector= TWI_vect
__interrupt void TWI_vect_interrupt( void );
#endif // __GNUC__
{
    uint8_t     status;

    status = inb(TWSR) & I2C_STATUS_MASK;

    do {
        switch (status) {
        /* Master */
        case I2C_REP_START:
            // Assert( i2c_state == I2C_RESTART )
        case I2C_START:
            // Assert( i2c_state == I2C_START_SENT )
#if RECOVER_LOST_START
            Timer2_Reset();
#endif  // RECOVER_LOST_START
            if (i2c_queue) {    // Make sure we have something to send
                outb(TWDR, i2c_queue->Address);
                I2C_Clear_Int();
                i2c_nbytes  = i2c_queue->DataLen;
                if (i2c_queue->Control & I2C_INDIRECT_DATA)
                    i2c_dataptr = i2c_queue->DataPtr;
                else
                    i2c_dataptr = i2c_queue->Data;
                i2c_state   = I2C_ADDRESS;
            } else {            // This is naughty - send a stop
                I2C_Send_Stop();
                i2c_state   = I2C_STOP;
                status      = I2C_STATUS_FAILED;
            }
            break;

        case I2C_BUS_ERROR:
            I2C_Send_Stop();
            i2c_state = I2C_STOP;
            status = I2C_STATUS_FAILED;
            break;

        case I2C_ARB_LOST:
        case I2C_NO_INFO:
            // Not sure how to handle these
//            I2C_Clear_Int();
//            i2c_state = I2C_STOP;
//            status = I2C_STATUS_FAILED;
            i2c_nbytes  = i2c_queue->DataLen;
            if (i2c_queue->Control & I2C_INDIRECT_DATA)
                i2c_dataptr = i2c_queue->DataPtr;
            else
                i2c_dataptr = i2c_queue->Data;
            i2c_state   = I2C_ADDRESS;
            I2C_Send_Start();
            break;

        /* Master Transmitter */
        case I2C_MT_SLA_ACK:
            // Assert( i2c_state == I2C_ADDRESS )
            i2c_state = I2C_WRITING;
        case I2C_MT_DATA_ACK:
            // Assert( i2c_state == I2C_WRITING )
            if (i2c_nbytes--) {
                // Assume that all FLASH (ROM) strings are in memory above
                // RAMEND.  This allows us to distinguish between RAM & ROM
                // data, since they are in different memory spaces and
                // accessed differently.
                if ((uint16_t) i2c_dataptr < RAMEND)
                    outb(TWDR, *i2c_dataptr++);
                else
                    outb(TWDR, LPMX(i2c_dataptr++));
                I2C_Clear_Int();
            } else {
                i2c_state = I2C_COMPLETE;
                status    = I2C_STATUS_SUCCESS;
            }
            break;

        case I2C_MT_SLA_NACK:
        case I2C_MR_SLA_NACK:
            I2C_Send_Stop();
            if ( i2c_nattempts ) {
                i2c_state = I2C_RETRY;
                i2c_nattempts--;
            } else {
                i2c_state = I2C_STOP;
                status    = I2C_STATUS_NO_ACK;
            }
            break;

        case I2C_MT_DATA_NACK:
            I2C_Send_Stop();
            i2c_state = I2C_STOP;
            status    = I2C_STATUS_INCOMPLETE;
            break;

        /* Master Receiver */
        case I2C_MR_DATA_ACK:
            if (i2c_nbytes) {
                *i2c_dataptr++ = inb(TWDR);
                i2c_nbytes--;
            }
            // Fall Through
        case I2C_MR_SLA_ACK:
            i2c_state = I2C_READING;
            if (i2c_nbytes > 1) {
                I2C_Recv_Ack();
            } else {
                I2C_Recv_Nack();
            }
            break;

        case I2C_MR_DATA_NACK:  // End of data
            if (i2c_nbytes) {
                *i2c_dataptr++ = inb(TWDR);
                i2c_nbytes--;
            }
            i2c_state = I2C_COMPLETE;
            status    = I2C_STATUS_SUCCESS;
            break;

//        case I2C_MR_SLA_NACK: see case I2C_MT_SLA_NACK:

        default:
            // Not sure how to handle this - maybe PANIC!
            I2C_Clear_Int();
            I2C_Send_Stop();
            i2c_state = I2C_STOP;
            status    = I2C_STATUS_FAILED;
            break;
        }
    } while (0);

    // Clean up
    if (i2c_state == I2C_COMPLETE) {
        // Check for another packet queued up for the same bus & speed
        if ((i2c_queue) && (i2c_queue->Link) &&
            (i2c_speed_bus == (i2c_queue->Link->Control & I2C_SPEED_BUS_MASK))&&
            ((i2c_queue->Address|I2C_READ_BIT)==(i2c_queue->Link->Address|I2C_READ_BIT))){
            i2c_state = I2C_RESTART;
            I2C_Send_Start();
        } else {
            I2C_Send_Stop();
            i2c_state = I2C_STOP;
        }
        i2c_nbytes  = 0;
        i2c_dataptr = NULL;
    }

    // Return status and unlink the old data - done or incomplete.
    if ((i2c_state == I2C_STOP) || (i2c_state == I2C_RESTART)) {
        if (i2c_queue) {
// Try stopping for huge periods of time for the junk MTV048
//            if (i2c_queue->Address == MTV_I2C_ADDR) {
//                i2c_speed_bus &= 0x0F;
//                i2c_speed_bus |= (I2C_SPEED_SLOW << 4);
//            }
            i2c_queue->Status = status;
            i2c_queue   = (volatile I2CReqPtr) i2c_queue->Link;
            i2c_nbytes  = 0;
            i2c_dataptr = NULL;
        }
    }

    if ((i2c_state == I2C_STOP) || (i2c_state == I2C_RETRY)) {
        Timer2_Set( I2C_TIMER2_SLOW_OCR, I2C_TIMER2_SLOW_SCALE );
    } else
        Check_And_Restart_Queue();
}


// This is the lowest level API to enqueue a request to the I2C
int8_t I2C_Add_Req( I2CReqPtr new_req )
{
    uint8_t     sreg;
    I2CReqPtr   list_ptr;

    // Handle bogus calls early
    if (new_req == NULL)
        return I2C_STATUS_ERROR;

#ifdef __GNUC__    
    (volatile I2CReqPtr) new_req->Link = NULL;
    (volatile uint8_t) new_req->Status = I2C_STATUS_WAIT;
#else
    new_req->Link = NULL;
    new_req->Status = I2C_STATUS_WAIT;
#endif

    sreg = SREG;
    cli();      // Disable interrupts while we monkey around

    // Insert new request into the list
    if (i2c_queue) {
        list_ptr = i2c_queue;
        while (list_ptr->Link)
            list_ptr = (volatile I2CReqPtr) list_ptr->Link;
        list_ptr->Link = new_req;
    } else {
        i2c_queue = new_req;
    }

    Check_And_Restart_Queue();

    SREG = sreg;            // Re-enable interrupts - very important!

    return I2C_STATUS_WAIT;
}


int8_t I2C_Add_Req_Wait( I2CReqPtr new_req )
{
    uint8_t         ret_val;

    if ((ret_val = I2C_Add_Req( new_req )) != I2C_STATUS_WAIT)
        return ret_val;

    while (new_req->Status == I2C_STATUS_WAIT)
        Task_Suspend();

    return new_req->Status;
}


int8_t I2C_Write_Byte( uint8_t speed_bus,
                       uint8_t address,
                       uint8_t data )
{
    I2CReqSize(1)   new_req;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_WRITE( address );
    new_req.req.DataLen = 1;
    new_req.req.Data[0] = data;

    return I2C_Add_Req_Wait((I2CReqPtr) &new_req );
}


int8_t I2C_Write_Word( uint8_t  speed_bus,
                       uint8_t  address,
                       uint16_t data )
{
    I2CReqSize(2)   new_req;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_WRITE( address );
    new_req.req.DataLen = 2;
    new_req.req.Data[0] = (uint8_t) data;
    new_req.req.Data[1] = (uint8_t) (data >> 8);

    return I2C_Add_Req_Wait((I2CReqPtr) &new_req );
}

int8_t I2C_Write_2Bytes( uint8_t speed_bus,
                         uint8_t address,
                         uint8_t data0,
                         uint8_t data1 )
{
    I2CReqSize(2)   new_req;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_WRITE( address );
    new_req.req.DataLen = 2;
    new_req.req.Data[0] = data0;
    new_req.req.Data[1] = data1;

    return I2C_Add_Req_Wait((I2CReqPtr) &new_req );
}

int8_t I2C_Write_3Bytes( uint8_t speed_bus,
                         uint8_t address,
                         uint8_t data0,
                         uint8_t data1,
                         uint8_t data2 )
{
    I2CReqSize(3)   new_req;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_WRITE( address );
    new_req.req.DataLen = 3;
    new_req.req.Data[0] = data0;
    new_req.req.Data[1] = data1;
    new_req.req.Data[2] = data2;

    return I2C_Add_Req_Wait((I2CReqPtr) &new_req );
}


int8_t I2C_Write_Array( uint8_t  speed_bus,
                        uint8_t  address,
                        uint8_t  len,
                        uint8_t *data_ptr )
{
    I2CReqSize(0)   new_req;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus | I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_WRITE( address );
    new_req.req.DataLen = len;
    new_req.req.DataPtr = data_ptr;

    return I2C_Add_Req_Wait((I2CReqPtr) &new_req );
}


int8_t I2C_Read_Byte( uint8_t  speed_bus,
                      uint8_t  address,
                      uint8_t *data_ptr )
{
    I2CReqSize(1)   new_req;
    int8_t          ret_val;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_READ( address );
    new_req.req.DataLen = 1;

    if ((ret_val = I2C_Add_Req_Wait((I2CReqPtr) &new_req )) == I2C_STATUS_SUCCESS)
        *data_ptr = new_req.req.Data[0];

    return ret_val;
}


int8_t I2C_Read_Word( uint8_t   speed_bus,
                      uint8_t   address,
                      uint16_t *data_ptr )
{
    I2CReqSize(2)   new_req;
    int8_t          ret_val;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_READ( address );
    new_req.req.DataLen = 2;

    if ((ret_val = I2C_Add_Req_Wait((I2CReqPtr) &new_req )) == I2C_STATUS_SUCCESS)
        *data_ptr = new_req.req.Data[0] | (new_req.req.Data[1] << 8);

    return ret_val;
}


// These routines will generate a two phase I2C transaction.  First, the
// write phase will write a 8 bit register address(es) to the target I2C
// device.  Then, the driver will generate a Restart read phase from the
// same device and read a single byte.
//
// Key assumptions embodied in this routine:
//     1. Sequential I2C requests are performed in order. (by design)
//     2. Two requests queued by a single thread will always be adjacent
//        because no other foreground task can preempt the running task.
//     3. No background task can be allowed to queue up on the I2C list.
//     4. The driver will seperate multiple requests to the same bus &
//        speed with a restart (by design).
//     5. This is very important: the first I2C phase _can_ _not_ fail
//        because the second phase will be started before the foreground
//        task can ever know. (may need to revisit this one)

int8_t I2C_Read_Addresed_Byte( uint8_t  speed_bus,
                               uint8_t  address,
                               uint8_t  reg_addr,
                               uint8_t *data_ptr )
{
    I2CReqSize(1)   reg_req, data_req;
    int8_t          ret_val;

    reg_req.req.Status  = I2C_STATUS_WAIT;
    reg_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    reg_req.req.Address = I2C_WRITE( address );
    reg_req.req.DataLen = 1;
    reg_req.req.Data[0] = reg_addr;

    ret_val = I2C_Add_Req((I2CReqPtr) &reg_req );
    if (ret_val != I2C_STATUS_WAIT)
        return ret_val; // Could not enqueue the write phase so quit now

    data_req.req.Status  = I2C_STATUS_WAIT;
    data_req.req.Control = speed_bus & ~I2C_INDIRECT_DATA;
    data_req.req.Address = I2C_READ( address );
    data_req.req.DataLen = 1;
    data_req.req.Data[0] = 0;

    ret_val = I2C_Add_Req((I2CReqPtr) &data_req );
    if (ret_val != I2C_STATUS_WAIT) {
        // Could not enqueue the read phase so quit after write
        while (reg_req.req.Status == I2C_STATUS_WAIT)
            Task_Suspend();
        return ret_val;
    } else {            // At least show failure if write phase fails
        while (data_req.req.Status == I2C_STATUS_WAIT)
            Task_Suspend();
        ret_val = data_req.req.Status;
        if (ret_val == I2C_STATUS_SUCCESS)
            *data_ptr = data_req.req.Data[0];
        return reg_req.req.Status ? reg_req.req.Status : ret_val;
    }
}


int8_t I2C_Write_Array_Read_Array( uint8_t  speed_bus,
                                   uint8_t  address,
                                   uint8_t  wr_len,
                                   uint8_t *wr_ptr,
                                   uint8_t  rd_len,
                                   uint8_t *rd_ptr )
{
    I2CReqSize(1)   reg_req, data_req;
    int8_t          ret_val;

    reg_req.req.Status  = I2C_STATUS_WAIT;
    reg_req.req.Control = speed_bus | I2C_INDIRECT_DATA;
    reg_req.req.Address = I2C_WRITE( address );
    reg_req.req.DataLen = wr_len;
    reg_req.req.DataPtr = wr_ptr;

    ret_val = I2C_Add_Req((I2CReqPtr) &reg_req );
    if (ret_val != I2C_STATUS_WAIT)
        return ret_val; // Could not enqueue the write phase so quit now

    data_req.req.Status  = I2C_STATUS_WAIT;
    data_req.req.Control = speed_bus | I2C_INDIRECT_DATA;
    data_req.req.Address = I2C_READ( address );
    data_req.req.DataLen = rd_len;
    data_req.req.DataPtr = rd_ptr;

    ret_val = I2C_Add_Req((I2CReqPtr) &data_req );
    if (ret_val != I2C_STATUS_WAIT) {
        // Could not enqueue the read phase so quit after write
        while (reg_req.req.Status == I2C_STATUS_WAIT)
            Task_Suspend();
        return ret_val;
    } else {            // At least show failure if write phase fails
        while (data_req.req.Status == I2C_STATUS_WAIT)
            Task_Suspend();
        ret_val = data_req.req.Status;
        return reg_req.req.Status ? reg_req.req.Status : ret_val;
    }
}



int8_t I2C_Read_Array( uint8_t  speed_bus,
                       uint8_t  address,
                       uint8_t  len,
                       uint8_t *data_ptr )
{
    I2CReqSize(0)   new_req;

    new_req.req.Status  = I2C_STATUS_WAIT;
    new_req.req.Control = speed_bus | I2C_INDIRECT_DATA;
    new_req.req.Address = I2C_READ( address );
    new_req.req.DataLen = len;
    new_req.req.DataPtr = data_ptr;

    return I2C_Add_Req_Wait((I2CReqPtr) &new_req );
}
