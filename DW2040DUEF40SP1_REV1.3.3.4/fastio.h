/*
 Contributors:
    Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
	Modified by Dennis Patella 2015 www.wasproject.it
*/
/* **************************************************************************
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

// **************************************************************************
//
// Description: Fast IO functions for Arduino Due
//
// ARDUINO_ARCH_SAM
// **************************************************************************

#ifndef	_FASTIO_H
#define	_FASTIO_H


#include <inttypes.h>
// --------------------------------------------------------------------------
/*
#define F_CPU       21000000        // should be factor of F_CPU_TRUE
#define F_CPU_TRUE  84000000        // actual CPU clock frequency
#define SPR0    0
#define SPR1    1
#undef  SOFTWARE_SPI
#define TIMER0_PRESCALE 128*/
// --------------------------------------------------------------------------

/// Read a pin

#define	READ(pin) PIO_Get(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin)


/// Write to a pin

#define	WRITE(pin, v) PIO_SetOutput(g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin, v, 0, PIO_PULLUP) 
/// toggle a pin

#define TOGGLE(pin) WRITE(pin,!READ(pin))
/// set pin as input

//#define PULLUP WRITE
#define PULLUP(IO,v) {pinMode(IO, (v!=LOW ? INPUT_PULLUP : INPUT)); }

#define	SET_INPUT(pin) pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId); \
    PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0) 
/// set pin as output

#define	SET_OUTPUT(pin) PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, \
    g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)
/// check if pin is an input
#define GET_INPUT(IO)
/// check if pin is an output
#define GET_OUTPUT(IO)

/// check if pin is an timer
#define GET_TIMER(IO)

#endif	/* _FASTIO_H */
