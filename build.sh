#!/usr/bin/sh

# Provided linker scripts can easily be used
# for compilation against RAM. Do not forget
# to change address in your flashing program.

FLASH_BASE=0x08000000
SRAM_BASE=0x20000200

ORIGIN=$FLASH_BASE
# ORIGIN=$SRAM_BASE

arm-none-eabi-gcc \
    -c -O2 usb_audio.c -I cmsis -mcpu=cortex-m3 -mthumb -Wall -Wno-unused \
    -fno-hosted -ffunction-sections -fdata-sections -fno-inline -fno-align-functions &&

arm-none-eabi-ld \
    usb_audio.o -o usb_audio.elf -Ttext=$ORIGIN \
    --gc-sections -L ldscripts -T blue_pill.ld --strip-debug &&

arm-none-eabi-objcopy \
    -O binary usb_audio.elf usb_audio.bin &&

( echo Origin: $ORIGIN
  echo Binary size: `wc -c usb_audio.bin` )
