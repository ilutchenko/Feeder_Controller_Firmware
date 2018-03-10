APPL_SRC =      sources

BINARY = feedercontroller

INCLUDE_DIRS =  -I$(APPL_SRC)


SOURCES =       $(APPL_SRC)/gpio.c              \
                $(APPL_SRC)/rcc.c               \
                $(APPL_SRC)/usart.c             \
                $(APPL_SRC)/timers.c            
                

OBJS = $(SOURCES:%.c=%.o)
+OPENCM3_DIR=./libopencm3
#CFLAGS = -Wall $(INCLUDE_DIRS)

LDSCRIPT = $(OPENCM3_DIR)/lib/stm32/f1/stm32f103x8.ld

include ./libopencm3.target.mk
