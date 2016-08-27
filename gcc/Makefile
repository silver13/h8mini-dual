TARGET=h8mini
EXECUTABLE=h8mini.elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump


DEFS = -DUSE_STDPERIPH_DRIVER
STARTUP = startup.s

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork -ffast-math -fdata-sections -ffunction-sections

topdir = ..
INCLUDES = -I$(topdir)/H8mini-test/src/ \
	-I$(topdir)/H8mini-acro/src/ \
	-I$(topdir)/CMSIS/ \
	-I$(topdir)/Peripherals/inc/

OPTIMIZE = -Os

CFLAGS = $(MCFLAGS) $(OPTIMIZE) $(DEFS) $(INCLUDES) -std=gnu99
AFLAGS = $(MCFLAGS)
LDFLAGS = -nostdlib -lm -lnosys -Wl,-T,flash.ld,-Map,$(TARGET).map,--gc-sections,-Os

SRC = $(wildcard $(topdir)/H8mini_*/src/*.c) \
	$(wildcard $(topdir)/H8mini_*/src/config.h) \
	$(wildcard $(topdir)/H8mini_*/src/*.cpp) \
	$(wildcard $(topdir)/CMSIS/*.c) \
	$(wildcard $(topdir)/Peripherals/src/*.c)
HEADERS = $(wildcard $(topdir)/H8mini_*/src/*.h)

OBJDIR = .
OBJ = $(patsubst %.c,$(OBJDIR)/%.o,$(filter %.c,$(SRC)))
OBJ += $(patsubst %.cpp,$(OBJDIR)/%.o,$(filter %.cpp,$(SRC)))
OBJ += Startup.o

all: $(TARGET)

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@
$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@

$(EXECUTABLE): $(SRC) $(STARTUP) $(HEADERS)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

clean:
	rm -f Startup.lst $(TARGET) $(TARGET).lst $(AUTOGEN) \
		$(TARGET).out $(TARGET).hex  $(TARGET).map \
		$(TARGET).dmp $(EXECUTABLE)
