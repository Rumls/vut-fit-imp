pegdbserver_console -startserver -device=Freescale_K6x_K60DN512M10 &

arm-none-eabi-gdb -ex 'target remote localhost:7224' -ex 'monitor _reset' \
-ex 'load ../Debug/FITkit3-demo.elf' -ex 'disconnect' -ex 'quit'

pkill pegdbserver_con
