# Name: Makefile
# Author: <insert your name here>
# Copyright: <insert your copyright message here>
# License: <insert your license reference here>

# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = attiny2313
CLOCK      = 7372800
PROGRAMMER = -c usbtiny
OBJECTS    = main.o
FUSES	   = -U lfuse:w:0xcc:m -U hfuse:w:0xdb:m -U efuse:w:0xff:m

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) -B 10 -F
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	digitnet.hex	size

main.o:

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:digitnet.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

size:	digitnet.elf
	avr-size --mcu=$(DEVICE) --format=avr digitnet.elf
	
ident:
	$(AVRDUDE)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID digitnet.hex

clean:
	rm -f digitnet.hex digitnet.elf $(OBJECTS)

# file targets:
digitnet.elf: $(OBJECTS)
	$(COMPILE) -o digitnet.elf $(OBJECTS)

digitnet.hex: digitnet.elf
	rm -f digitnet.hex
	avr-objcopy -j .text -j .data -O ihex digitnet.elf digitnet.hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	digitnet.elf
	avr-objdump -d digitnet.elf

cpp:
	$(COMPILE) -E main.c