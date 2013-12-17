############################################
## $Id$ 
## 
## Jan-Willem Smaal <jws@gispen.org>
############################################
PRG	= main
OBJ = main.o

# Xcode needs the full path....
AVRTOOLS_PATH = /opt/local/bin


MCU_TARGET     = atmega328p
OPTIMIZE       = -O2

DEFS           =
LIBS           =


# You should not have to change anything below here.
CC             = $(AVRTOOLS_PATH)/avr-gcc

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = $(AVRTOOLS_PATH)/avr-objcopy
OBJDUMP        = $(AVRTOOLS_PATH)/avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
demo.o: demo.c 

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

install: 
	$(AVRTOOLS_PATH)/avrdude -patmega328p -c buspirate -P /dev/tty.usbserial-A901LOEI -b 115200 -U flash:w:$(PRG).hex:i

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@



############################################
# Rules for building the .text rom images
############################################
text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@


############################################
# Rules for building the .eeprom rom images
############################################
eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }


# EOF
