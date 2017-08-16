#!/bin/bash

echo "Cerco Arduino 1.6.0 in $PATH_ARDUINO"
echo "settare il percorso in PATH_ARDUINO"

$PATH_ARDUINO/arduino --verify DW2040F21P1_Rev6.5/DW2040F21P1_Rev6.5.ino -v

