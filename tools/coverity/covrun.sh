#!/bin/bash

#cov-configure.exe --comptype gcc --template  --compiler arm-none-eabi-gcc.exe

#st fsdev
#example=hid_composite
#board=stm32f070rbnucleo

#st syn
#board=stm32f407disco
#st fsdev
board=stm32f303disco

example=all_classes

#example=webusb_serial
#board=lpcxpresso54114
rm -rf cov-int
rm -rf tusb-${example}-${board}.xz

make -j4 -C examples/device/$example BOARD=$board clean
cov-build --dir cov-int --  make -j4 -C examples/device/$example BOARD=$board all

tar caf tusb-${example}-${board}.xz  cov-int
