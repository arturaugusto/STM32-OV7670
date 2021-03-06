#!/bin/bash
cargo build --release

#cargo flash --chip stm32f103C8 --release

xterm -geometry -10-10 -e 'openocd -x openocd.gdb -f interface/stlink-v2.cfg -f target/stm32f1x.cfg' &

ocdpid=$!

sleep 1

xterm -geometry -10-550 -e 'gdb-multiarch -x openocd.gdb target/thumbv7m-none-eabi/release/diapabot' &

gdbpid=$!

#https://docs.rust-embedded.org/book/start/hardware.html
read -n 1 -p "On (gdb) term, type 'load' and them 'continue' to get started. Press any key to terminate." 

kill $ocdpid

kill $gdbpid
