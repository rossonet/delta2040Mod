#!/bin/bash

cp -f bin/variant.cpp ./arduino-1.5.8/hardware/arduino/sam/variants/arduino_due_x/
cp -f bin/variant.h ./arduino-1.5.8/hardware/arduino/sam/variants/arduino_due_x/
export PATH_ARDUINO=$(pwd)/arduino-1.5.8

echo "Uso Arduino 1.5.8 in $PATH_ARDUINO"

ris334=$($PATH_ARDUINO/arduino --verify --board arduino:sam:arduino_due_x DW2040DUEF40SP1_REV1.3.3.4/DW2040DUEF40SP1_REV1.3.3.4.ino -v | grep DW2040DUEF40SP1_REV1.3.3.4.cpp.bin | cut -d\  -f 5)
ris338=$($PATH_ARDUINO/arduino --verify  --board arduino:sam:arduino_due_x DW2040DUEF40SP1_REV1.3.3.8/DW2040DUEF40SP1_REV1.3.3.8.ino -v | grep DW2040DUEF40SP1_REV1.3.3.8.cpp.bin | cut -d\  -f 5)

rm -f DW2040DUEF40SP1_REV1.3.3.4.cpp.bin
cp $ris334 .
ls -lh DW2040DUEF40SP1_REV1.3.3.4.cpp.bin

rm -f DW2040DUEF40SP1_REV1.3.3.8.cpp.bin
cp $ris338 .
ls -lh DW2040DUEF40SP1_REV1.3.3.8.cpp.bin

