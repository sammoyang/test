OBJ			= I2C.o
ROOT_DIR	= ../..
INCDIR		= -I$(TIMERS_INC) -I$(SYSTEM_INC) -I$(OSD_INC)
include		$(ROOT_DIR)/Makefile.inc
