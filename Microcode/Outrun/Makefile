# Makefile for Sega Outrun JAMMA Converter Firmware (8MHz Internal Clock)
# ATMEGA328P @ 8MHz INTERNAL CLOCK (no external crystal required)

# Project name (without extension)
PROJECT = outrun_jamma

# MCU type
MCU = atmega328p

# Target frequency (8MHz internal clock)
F_CPU = 8000000

# Compiler settings
CC = avr-gcc
OBJCOPY = avr-objcopy
SIZE = avr-size

# Optimization level (use -O2 for production, -Og for debugging)
OPT = -Os

# Standard flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL $(OPT) -Wall -Wextra
CFLAGS += -ffunction-sections -fdata-sections

# B2 shifter output mode (see outrun_jamma.c for details):
#   SHIFT_MODE_TOGGLE      latching hi/lo toggle (default, original Outrun)
#   SHIFT_MODE_MOMENTARY   momentary: HIGH while held, for a turbo button
# Override on the command line, e.g.: make SHIFT_MODE=SHIFT_MODE_MOMENTARY
SHIFT_MODE ?= SHIFT_MODE_TOGGLE
CFLAGS += -DSHIFT_MODE=$(SHIFT_MODE)

# Linker flags
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

# Programmer settings (adjust for your programmer)
# Examples:
# - usbtiny (USBtinyISP)
# - avrisp2 (Atmel AVRISP mkII)
# - arduino (Arduino as ISP)
PROGRAMMER = usbtiny

# Fuse bits for ATMEGA328P (8MHz Internal Clock)
# Low: 0xC2 (internal 8MHz RC oscillator)
# High: 0xDE (boot reset vector enabled, SPIEN=0, WDTON=0)
# Extended: 0xFD (brown-out detect at 2.7V)
LFUSE = 0xC2
HFUSE = 0xDE
EFUSE = 0xFD

# Source files
SRC = $(PROJECT).c

# Object files
OBJ = $(SRC:.c=.o)

# Default target
all: hex size

# Compile C source files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link object files
$(PROJECT).elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

# Create hex file from ELF
hex: $(PROJECT).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $(PROJECT).hex

# Create eeprom hex file (if needed)
eeprom: $(PROJECT).elf
	$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom=alloc,load \
		--change-section-lma .eeprom=0 -O ihex $< $(PROJECT)eep.hex

# Display program size
size: $(PROJECT).elf
	$(SIZE) --format=avr --mcu=$(MCU) $<

# Upload firmware using avrdude
flash: hex
	avrdude -c $(PROGRAMMER) -p $(MCU) -U flash:w:$(PROJECT).hex:i

# Upload eeprom
flash_eep: eeprom
	avrdude -c $(PROGRAMMER) -p $(MCU) -U eeprom:w:$(PROJECT)eep.hex:i

# Set fuses (WARNING: Incorrect fuse settings can brick your chip!)
fuses:
	avrdude -c $(PROGRAMMER) -p $(MCU) -u \
		-U lfuse:w:$(LFUSE):m \
		-U hfuse:w:$(HFUSE):m \
		-U efuse:w:$(EFUSE):m

# Read fuses
read_fuses:
	avrdude -c $(PROGRAMMER) -p $(MCU) -u \
		-U lfuse:r:lfuse.txt:h \
		-U hfuse:r:hfuse.txt:h \
		-U efuse:r:efuse.txt:h

# Full build with fuses and flash
full_build: fuses flash

# Clean build artifacts
clean:
	rm -f $(OBJ) $(PROJECT).d $(PROJECT).elf $(PROJECT).hex $(PROJECT)eep.hex

# Clean all including hex files
distclean: clean
	rm -f *.txt

# Read signature (verify chip connection)
signature:
	avrdude -c $(PROGRAMMER) -p $(MCU) -n

# Interactive terminal (if using serial programmer)
terminal:
	minicom -b 115200 -o

# Debug build (with debug symbols)
debug: OPT = -Og -g
debug: all

# Phony targets
.PHONY: all hex eeprom size flash flash_eep fuses read_fuses full_build clean distclean signature terminal debug

# Dependencies (auto-generated)
-include $(SRC:.c=.d)

# Generate dependency files
%.d: %.c
	@set -e; rm -f $@; \
	$(CC) $(CFLAGS) -MM $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,$@ $@.$$$$ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
