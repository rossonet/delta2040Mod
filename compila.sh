#!/bin/bash

echo "Controlla la presenza delle IDE Arduino necessarie, se non presenti le scarica e installa in locale"
bin/scarica.sh
echo "Compila la versione per Arduino Mega"
mega_bin/compilaArduino.sh
echo "Compila le due versione per Arduino Due"
bin/compila.sh

echo "Compilazione completata"
