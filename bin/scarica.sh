#!/bin/bash

MACHINE_TYPE=`uname -m`
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
  selezione=linux64
else
  selezione=linux32
fi

if [ -e arduino-1.6.0 ]
then
	echo "IDE Arduino 1.6.0 $selezione OK"
else
	if [ -e arduino-1.6.0-$selezione.tar.xz ]
	then
		echo "Tar xz di IDE Arduino 1.6.0 $selezione OK"
	else
		echo "Scarico IDE Arduino 1.6.0 $selezione"
		wget https://downloads.arduino.cc/arduino-1.6.0-$selezione.tar.xz
	fi
	tar -xJf arduino-1.6.0-linux64.tar.xz
fi


if [ -e arduino-1.5.8 ]
then
	echo "IDE Arduino 1.5.8 $selezione OK"
else
	if [ -e arduino-1.5.8-$selezione.tgz ]
	then
		echo "Tar gz di IDE Arduino 1.5.8 $selezione OK"
	else
		echo "Scarico IDE Arduino 1.5.8 $selezione"
		wget https://downloads.arduino.cc/arduino-1.5.8-$selezione.tgz
	fi
	tar -xzf arduino-1.5.8-linux64.tgz
fi
