#!/bin/sh
cargo objcopy --release -- -O binary ./target/thumbv7m-none-eabi/release/ross-programmer-firmware.bin
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init; reset halt; flash erase_address 0x08008000 0x00018000; flash write_bank 0 ./target/thumbv7m-none-eabi/release/ross-programmer-firmware.bin 0x00008000; reset run; shutdown;"
