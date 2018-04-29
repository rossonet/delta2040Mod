#!/bin/bash

export PATH_ARDUINO=$(pwd)/arduino-1.6.0

echo "Uso Arduino 1.6.0 in $PATH_ARDUINO"

risultato=$($PATH_ARDUINO/arduino --board arduino:avr:mega --verify DW2040F21P1_Rev6.5/DW2040F21P1_Rev6.5.ino -v | grep DW2040F21P1_Rev6.5.cpp.hex | cut -d\  -f 7)

rm -f DW2040F21P1_Rev6.5.cpp.hex
cp $risultato .
ls -lh DW2040F21P1_Rev6.5.cpp.hex

