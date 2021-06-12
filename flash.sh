#!/bin/sh
cargo objcopy --release -- -O binary ./target/thumbv7m-none-eabi/release/ross-firmware.bin
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init; reset halt; flash erase_address 0x08004000 0x0001c000; flash write_bank 0 ./target/thumbv7m-none-eabi/release/ross-firmware.bin 0x00004000; reset run; shutdown;"
