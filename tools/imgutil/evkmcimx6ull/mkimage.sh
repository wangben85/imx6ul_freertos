#!/bin/sh

if [ "$#" -ne 1 ] || ! [ "$1" == "ram" -o "$1" == "flash" ] ; then
  echo "Usage: $0 target"
  echo "       target: ram -- the image will be loaded to RAM and run, the application must be built with ram link file"
  echo "       target: flash -- the image will be run on flash directly, the application must be build with flash link file"
  echo "Example: $0 ram"
  exit 1
fi

../dcdgen dcd.config dcd.bin
if [ "$1" == "ram" ]; then
    ../imgutil --combine base_addr=0x80000000 ivt_offset=0x1000 app_offset=0x2000 dcd_file=dcd.bin app_file=sdk20-app.bin ofile=sdk20-app.img image_entry_point=0x80002000
elif [ "$1" == "flash" ]; then
    ../imgutil --combine base_addr=0x60000000 ivt_offset=0x1000 app_offset=0x2000 dcd_file=dcd.bin app_file=sdk20-app.bin ofile=sdk20-app.img image_entry_point=0x60002000
fi
