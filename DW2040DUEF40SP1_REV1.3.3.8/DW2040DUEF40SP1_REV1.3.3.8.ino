  /* -*- c++ -*- */

/*
   This firmware is UNOFFICIAL version of the original Marlin
Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm.
Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
Copyright (c) 2014 Dennis Patella dennis@wasproject.it

This is a modified version partially rewrited by Dennis Patella (Ðen) WASProject on 2014/2015 
http://www.wasproject.it
and adapted for WASP DELTA PRINTERS and WASP HARDWARE 
The use with different hardware is absolutely not tested and not recommended
- adapted for working whith SAM S3 as Arduino DUE
Some improvements: 
- full compatibility with DELTA WASP MACHINES (jerk and some improvement for delta)
- Integrated recovery print
- Emergency stop with print rescue
- Manual levelling delta
- improvement in changing filament (M600)
- pause M25 with extruder lifting
- some additional lcd menu
- dual language selection from menu
- ...and much more
- V 3.1  with Free Zeta System (thanks to Marcello) that can start gcode from given Z
 WARNING, gcode MUST CONTAIN MAX 1 Z movement before the init print (eg. start gcode in Slic3r or Cura)
 Otherwise FZS can be auto-excluded and the print begins from the beginning
- Rev1 enhanced extruder temp saving
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
- Rev2 NEW RESURRECTION SYSTEM 
- 4.0S Rev 1.3 full dual extruder compatibility

 Most of features are in CREATIVE COMMONS LICENSE
 Please do not distribuite this firmware and do not modify without WASP permission
 This program is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 */



#include "Configuration.h"
#include "pins.h"

#ifdef ULTRA_LCD
  #if defined(LCD_I2C_TYPE_PCF8575)
    #include <Wire.h>
    #include <LiquidCrystal_I2C.h>
  #elif defined(LCD_I2C_TYPE_MCP23017) || defined(LCD_I2C_TYPE_MCP23008)
    #include <Wire.h>
    #include <LiquidTWI2.h>
  #elif defined(DOGLCD)||defined(WASPLCD)
    #include <U8glib.h> // library for graphics LCD by Oli Kraus (https://code.google.com/p/u8glib/)
  #else
    #include <LiquidCrystal.h> // library for character LCD
  #endif
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#if defined(DIGIPOT_I2C)
  #include <Wire.h>
#endif
