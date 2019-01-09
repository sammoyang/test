#ifndef _I2C_I2C_H_
#define _I2C_I2C_H_ 1

// This is the number of attempts to get a slave to ACK it's address
#define I2C_MAX_ATTEMPTS        3

//#include <avr/io.h>

/* TWSR values (not bits) */
/* Master */
#define I2C_START               0x08
#define I2C_REP_START           0x10
#define I2C_ARB_LOST            0x38
/* Master Transmitter */
#define I2C_MT_SLA_ACK          0x18
#define I2C_MT_SLA_NACK         0x20
#define I2C_MT_DATA_ACK         0x28
#define I2C_MT_DATA_NACK        0x30
/* Master Receiver */
#define I2C_MR_SLA_ACK          0x40
#define I2C_MR_SLA_NACK         0x48
#define I2C_MR_DATA_ACK         0x50
#define I2C_MR_DATA_NACK        0x58
/* Slave Transmitter */
#define I2C_ST_SLA_ACK          0xA8
#define I2C_ST_ARB_LOST_SLA_ACK 0xB0
#define I2C_ST_DATA_ACK         0xB8
#define I2C_ST_DATA_NACK        0xC0
#define I2C_ST_LAST_DATA        0xC8
/* Slave Receiver */
#define I2C_SR_SLA_ACK          0x60
#define I2C_SR_ARB_LOST_SLA_ACK 0x68
#define I2C_SR_GCALL_ACK        0x70
#define I2C_SR_ARB_LOST_GCALL_ACK 0x78
#define I2C_SR_DATA_ACK         0x80
#define I2C_SR_DATA_NACK        0x88
#define I2C_SR_GCALL_DATA_ACK   0x90
#define I2C_SR_GCALL_DATA_NACK  0x98
#define I2C_SR_STOP             0xA0
/* Misc */
#define I2C_NO_INFO             0xF8
#define I2C_BUS_ERROR           0x00

/*
 * The lower 3 bits of TWSR are reserved on the ATmega163.
 * The 2 LSB carry the prescaler bits on the newer ATmegas.
 */
#define I2C_STATUS_MASK         (_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)| _BV(TWS3))
#define I2C_STATUS              (TWSR & I2C_STATUS_MASK)

#define I2C_COMMAND_MASK        (_BV(TWEN)|_BV(TWIE))

/*
 * R/~W bit in SLA+R/W address field.
 */
#define I2C_READ_BIT    1
#define I2C_READ( X )   (X | I2C_READ_BIT)
#define I2C_WRITE( X )  (X & ~I2C_READ_BIT)

#define I2C_SCALE_1             0x00
#define I2C_SCALE_4             0x01
#define I2C_SCALE_16            0x02
#define I2C_SCALE_64            0x03

#define TIMER2_SCALE_1          0x01
#define TIMER2_SCALE_8          0x02
#define TIMER2_SCALE_64         0x03
#define TIMER2_SCALE_256        0x04
#define TIMER2_SCALE_1024       0x05

#define I2C_BR_MIN              10
#define I2C_BR_MAX              255

// The I2C bus select lines are PortD4 & PortD5 (LSB, MSB)
// If this changes, then these values must be changed
#define I2C_BUS_SEL_PORT        PORTD
#define I2C_BUS_SEL_DDR         DDRD
#define I2C_BUS_SEL_LSB         4

// Processor speed based defines

#define I2C_FAST_BIT            100000UL    // Normal I2C bus speed 100Kbps
#define I2C_SLOW_BIT            10000UL     // Slooooow I2c bus speed for VFD

#if (((SYSCLK) / (I2C_FAST_BIT)) < 36)

#   warning     "I2C Clock speed is too fast for this processor, using max."
#   define      I2C_SCALER_FAST         I2C_SCALE_1
#   define      I2C_BR_FAST             I2C_BR_MIN
#   define      I2C_TIMER2_FAST_SCALE   TIMER2_SCALE_1
#   define      I2C_TIMER2_FAST_OCR     144

#elif (((SYSCLK) / (I2C_FAST_BIT)) < 500)   // Use 1x prescaler

#   define      I2C_SCALER_FAST         I2C_SCALE_1
#   define      I2C_BR_FAST             ((((SYSCLK) / (I2C_FAST_BIT)) - 16) / 2)
#   define      I2C_TIMER2_FAST_SCALE   TIMER2_SCALE_8
#   define      I2C_TIMER2_FAST_OCR     I2C_BR_FAST

#elif (((SYSCLK) / (I2C_FAST_BIT)) < 2000)  // Use 4x prescaler

#   define      I2C_SCALER_FAST         I2C_SCALE_4
#   define      I2C_BR_FAST             ((((SYSCLK) / (I2C_FAST_BIT)) - 16) / 8)
#   define      I2C_TIMER2_FAST_SCALE   TIMER2_SCALE_64
#   define      I2C_TIMER2_FAST_OCR     (I2C_BR_FAST >> 1)

#elif (((SYSCLK) / (I2C_FAST_BIT)) < 8000)  // Use 16x prescaler

#   define      I2C_SCALER_FAST         I2C_SCALE_16
#   define      I2C_BR_FAST             ((((SYSCLK) / (I2C_FAST_BIT)) - 16) / 32)
#   define      I2C_TIMER2_FAST_SCALE   TIMER2_SCALE_256
#   define      I2C_TIMER2_FAST_OCR     (I2C_BR_FAST >> 2)

#elif (((SYSCLK) / (I2C_FAST_BIT)) < 32656) // Use 64x prescaler

#   define      I2C_SCALER_FAST         I2C_SCALE_64
#   define      I2C_BR_FAST             ((((SYSCLK) / (I2C_FAST_BIT)) - 16) / 128)
#   define      I2C_TIMER2_FAST_SCALE   TIMER2_SCALE_256
#   define      I2C_TIMER2_FAST_OCR     I2C_BR_FAST

#else

#   warning     "I2C Clock speed is too slow for this processor, using min."
#   define      I2C_SCALER_FAST         64
#   define      I2C_BR_FAST             I2C_BR_MAX
#   define      I2C_TIMER2_FAST_SCALE   TIMER2_SCALE_256
#   define      I2C_TIMER2_FAST_OCR     I2C_BR_FAST

#endif

#if (((SYSCLK) / (I2C_SLOW_BIT)) < 36)

#   warning     "I2C Clock speed is too fast for this processor, using max."
#   define      I2C_SCALER_SLOW         I2C_SCALE_1
#   define      I2C_BR_SLOW             I2C_BR_MIN
#   define      I2C_TIMER2_SLOW_SCALE   TIMER2_SCALE_1
#   define      I2C_TIMER2_SLOW_OCR     144

#elif (((SYSCLK) / (I2C_SLOW_BIT)) < 500)   // Use 1x prescaler

#   define      I2C_SCALER_SLOW         I2C_SCALE_1
#   define      I2C_BR_SLOW             ((((SYSCLK) / (I2C_SLOW_BIT)) - 16) / 2)
#   define      I2C_TIMER2_SLOW_SCALE   TIMER2_SCALE_8
#   define      I2C_TIMER2_SLOW_OCR     I2C_BR_SLOW

#elif (((SYSCLK) / (I2C_SLOW_BIT)) < 2000)  // Use 4x prescaler

#   define      I2C_SCALER_SLOW         I2C_SCALE_4
#   define      I2C_BR_SLOW             ((((SYSCLK) / (I2C_SLOW_BIT)) - 16) / 8)
#   define      I2C_TIMER2_SLOW_SCALE   TIMER2_SCALE_64
#   define      I2C_TIMER2_SLOW_OCR     (I2C_BR_SLOW >> 1)

#elif (((SYSCLK) / (I2C_SLOW_BIT)) < 8000)  // Use 16x prescaler

#   define      I2C_SCALER_SLOW         I2C_SCALE_16
#   define      I2C_BR_SLOW             ((((SYSCLK) / (I2C_SLOW_BIT)) - 16) / 32)
#   define      I2C_TIMER2_SLOW_SCALE   TIMER2_SCALE_256
#   define      I2C_TIMER2_SLOW_OCR     (I2C_BR_SLOW >> 2)

#elif (((SYSCLK) / (I2C_SLOW_BIT)) < 32656) // Use 64x prescaler

#   define      I2C_SCALER_SLOW         I2C_SCALE_64
#   define      I2C_BR_SLOW             ((((SYSCLK) / (I2C_SLOW_BIT)) - 16) / 128)
#   define      I2C_TIMER2_SLOW_SCALE   TIMER2_SCALE_256
#   define      I2C_TIMER2_SLOW_OCR     I2C_BR_SLOW

#else

#   warning     "I2C Clock speed is too slow for this processor, using min."
#   define      I2C_SCALER_SLOW         I2C_SCALE_64
#   define      I2C_BR_SLOW             I2C_BR_MAX
#   define      I2C_TIMER2_SLOW_SCALE   TIMER2_SCALE_256
#   define      I2C_TIMER2_SLOW_OCR     I2C_BR_SLOW

#endif


#endif  /* _I2C_I2C_H_ */
