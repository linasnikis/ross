#!/bin/sh
cargo objcopy --release -- -O binary ./target/thumbv7m-none-eabi/release/ross-firmware.bin
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init; reset halt; stm32f1x mass_erase 0; flash write_bank 0 ./target/thumbv7m-none-eabi/release/ross-firmware.bin ; reset run; shutdown;"
