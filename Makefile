all: build

build: kk.raw

kk.raw: kk.o
	avr-objcopy -j .text -j .data -O binary $< $@

kk.hex: kk.o
	avr-objcopy -j .text -j .data -O ihex $< $@

kk.o: kk.c io_cfg.h typedefs.h
	avr-gcc -mmcu=atmega88a -Wall -g3 -gdwarf-2 -std=gnu99 -DF_CPU=8000000UL -Os -fsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -o $@ $<
	size $@

kk.s: kk.c io_cfg.h typedefs.h
	avr-gcc -mmcu=atmega88a -Wall -g3 -gdwarf-2 -std=gnu99 -DF_CPU=8000000UL -Os -fsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -S -o $@ $<

program: kk.raw
	avrdude -c dragon_isp -p m88 -P usb -U flash:w:$<:r

shell:
	avrdude -c dragon_isp -p m88 -P usb -t
