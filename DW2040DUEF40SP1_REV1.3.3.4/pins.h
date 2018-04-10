/*
Modified by Dennis Patella WASPROJECT TEAM on 11/2014 
http://www.wasproject.it
 */
#ifndef	_PINS_H
#define	_PINS_H

// --------------------------------------------------------------------------
// Arduino Due version
// --------------------------------------------------------------------------

/* Arduino Due supports the following shields:
 *
 * RADDS
 * RAMPS-FD v1
 * RAMPS-FD v2
 *
 * Extras
 *  - LCDs
 *  - SDRAMPS
 *
 */


#if MOTHERBOARD != 88
#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1
#endif



/****************************************************************************************
* Arduino Due pin assignment
*
* for RADDS
****************************************************************************************/
#if MOTHERBOARD == 402
#define KNOWN_BOARD 1
//
#ifndef __SAM3X8E__
 #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif
//
#define RADDS


#define X_STEP_PIN         24
#define X_DIR_PIN          23
#define X_ENABLE_PIN       26


#define Y_STEP_PIN         17
#define Y_DIR_PIN          16
#define Y_ENABLE_PIN       22

#define Z_STEP_PIN         2
#define Z_DIR_PIN          3
#define Z_ENABLE_PIN       15

#define X_MIN_PIN          28
#define X_MAX_PIN          -1  // 34   //Max endstops default to disabled "-1", set to commented value to enable.
#define Y_MIN_PIN          30
#define Y_MAX_PIN          -1  // 36
#define Z_MIN_PIN          32
#define Z_MAX_PIN          -1  // 38

#define E0_STEP_PIN        61
#define E0_DIR_PIN         60
#define E0_ENABLE_PIN      62

#define E1_STEP_PIN        64
#define E1_DIR_PIN         63
#define E1_ENABLE_PIN      65

#define E2_STEP_PIN        51
#define E2_DIR_PIN         53
#define E2_ENABLE_PIN      49

#define SDPOWER            -1
#define SDSS               4 //10 Display
#define LED_PIN            -1

#define BEEPER             41

#define FAN_PIN            9

#define CONTROLLERFAN_PIN  8 //Pin used for the fan to cool controller

#define PS_ON_PIN          40

#define KILL_PIN           -1

#define HEATER_BED_PIN     7    // BED
#define HEATER_0_PIN       13
#define HEATER_1_PIN       12
#define HEATER_2_PIN       11

#define TEMP_BED_PIN       4   // ANALOG NUMBERING
#define TEMP_0_PIN         0   // ANALOG NUMBERING
#define TEMP_1_PIN         -1  // 1   // ANALOG NUMBERING
#define TEMP_2_PIN         -1  // 2   // ANALOG NUMBERING
#define TEMP_3_PIN         -1  // 3   // ANALOG NUMBERING



  #ifdef NUM_SERVOS
    #define SERVO0_PIN         5

    #if NUM_SERVOS > 1
      #define SERVO1_PIN         6
    #endif

    #if NUM_SERVOS > 2
      #define SERVO2_PIN         39
    #endif

    #if NUM_SERVOS > 3
      #define SERVO3_PIN         40
    #endif
  #endif


#ifdef ULTRA_LCD

	// RADDS LCD panel
	#ifdef NEWPANEL
	  #define LCD_PINS_RS 		42
	  #define LCD_PINS_ENABLE 	43
	  #define LCD_PINS_D4 		44
	  #define LCD_PINS_D5 		45
	  #define LCD_PINS_D6 		46
	  #define LCD_PINS_D7 		47

	  #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER
			#define BEEPER 41

			#define BTN_EN1 52
			#define BTN_EN2 50
			#define BTN_ENC 48

			#define SDCARDDETECT 14
	  #endif
	#endif

#endif //ULTRA_LCD


// SPI for Max6675 Thermocouple

//works with radds??? #ifndef SDSUPPORT
//// these pins are defined in the SD library if building with SD support
//  #define MAX_SCK_PIN          52
//  #define MAX_MISO_PIN         50
//  #define MAX_MOSI_PIN         51
//  #define MAX6675_SS       53
//#else
//  #define MAX6675_SS       49
//#endif

#endif //MOTHERBOARD == 402



/****************************************************************************************
* Arduino Due pin assignment
*
* for WASP ELECTRONICS
****************************************************************************************
analog pin as digital 
Port         Pin     Function
PF 0       54       A0
PF 1       55       A1
PF 2       56       A2
PF 3       57       A3
PF 4       58       A4
PF 5       59       A5
PF 6       60       A6
PF 7       61       A7  <<
PK 0       62       A8
PK 1       63       A9
PK 2       64       A10
PK 3       65       A11  << 
PK 4       66       A12
PK 5       67       A13
PK 6       68       A14
PK 7       69       A15  */
#if MOTHERBOARD == 405
#define KNOWN_BOARD 1
//
#ifndef __SAM3X8E__
 #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif
//
#define WASP
#define STEPPER_RESET      61   //A7
#define X_STEP_PIN         25
#define X_DIR_PIN          23
#define X_ENABLE_PIN       27
#define X_FAULT       		55

#define Y_STEP_PIN         31
#define Y_DIR_PIN          33
#define Y_ENABLE_PIN       29
#define Y_FAULT 			56

#define Z_STEP_PIN         37
#define Z_DIR_PIN          39
#define Z_ENABLE_PIN       35
#define Z_FAULT				57

#define X_MIN_PIN          -1
#define X_MAX_PIN          24  
#define Y_MIN_PIN          -1
#define Y_MAX_PIN          28  
#define Z_MIN_PIN          -1  //30
#define Z_MAX_PIN          32  

#define E0_STEP_PIN        43
#define E0_DIR_PIN         45
#define E0_ENABLE_PIN      41
#define E0_FAULT 			58

#define E2_STEP_PIN        44
#define E2_DIR_PIN         46
#define E2_ENABLE_PIN      23
#define E2_FAULT 		   59 

#define E1_STEP_PIN        49
#define E1_DIR_PIN         47
#define E1_ENABLE_PIN      48
#define E1_FAULT 		   60 //ad6

#define SDPOWER            -1
#define SDSS               53 // Display
#define LED_PIN            13

//#define BEEPER             18

#define FAN_PIN            7   //7
#define FAN_PIN2		   67
//#define HARDRESET	9
#define EXT_PAUSE			30 //Z MIN
#define WIFI_ON	92
#define WIFI_RUN 36
#define CONTROLLERFAN_PIN  -1 // Pin used for the fan to cool controller ON WASP THE FAN WAS ALWAYS POWERED

#define CURR_CONTROL_DRIVER		8   // control the current on drivers

#define CHECK_EXTR		50   //pin ausiliario connettore a pettine vicino cpu (Gnd rx tx 50 51 52)

#define PS_ON_PIN          12

#define KILL_PIN           -1

#define HEATER_BED_PIN     4   // BED
#define HEATER_0_PIN       3   //2
#define HEATER_1_PIN       2   //3
#define HEATER_2_PIN       -1

#define TEMP_BED_PIN       10  // ANALOG NUMBERING
#define TEMP_0_PIN         8   // ANALOG NUMBERING
#define TEMP_1_PIN         9   // ANALOG NUMBERING
#define TEMP_2_PIN         -1  // ANALOG NUMBERING
#define TEMP_3_PIN         -1  // ANALOG NUMBERING

#define SENS_BED			54 //A0

#define SUICIDE_PIN        -1
#define EMERGENCY_STOP	   65  // A11 
#define EXT_FILAMENT	   26  //26
#define EXT_FILAMENT2	   22  
#ifdef ULTRA_LCD

	// WASP LCD panel
	#ifdef NEWPANEL
	#define LCD_PINS_RS 		13    //14 20   50
	#define LCD_PINS_ENABLE 	12
	#define LCD_PINS_D4 		11
	#define LCD_PINS_D5 		10   //15 21   51
	#define LCD_PINS_D6 		9
	#define LCD_PINS_D7 		6
	#define BEEPER 5
	#define BTN_EN1 40
	#define BTN_EN2 42
	#define BTN_ENC 34
	#define SDCARDDETECT 38
	#endif

#endif //WASP LCD panel



#endif //MOTHERBOARD == 405

/****************************************************************************************
* Arduino Due pin assignment
*
* for RAMPS-FD
****************************************************************************************/
#if MOTHERBOARD == 403 || MOTHERBOARD == 404
#define KNOWN_BOARD 1

//
#ifndef __SAM3X8E__
 #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif


//
#if MOTHERBOARD == 403
  #define RAMPS_FD_V1
  #define INVERTED_HEATER_PINS
  // No EEPROM
  // Use 4k7 thermistor tables
#else
  #define RAMPS_FD_V2
  // EEPROM supported
  // Use 1k thermistor tables
#endif

#define X_STEP_PIN         63
#define X_DIR_PIN          62
#define X_ENABLE_PIN       48
#define X_MIN_PIN          22
#define X_MAX_PIN           -1   //2 //Max endstops default to disabled "-1", set to commented value to enable.

#define Y_STEP_PIN         65
#define Y_DIR_PIN          64
#define Y_ENABLE_PIN       46
#define Y_MIN_PIN          24
#define Y_MAX_PIN          -1   //15

#define Z_STEP_PIN         67
#define Z_DIR_PIN          66
#define Z_ENABLE_PIN       44
#define Z_MIN_PIN          26
#define Z_MAX_PIN          -1

#define E0_STEP_PIN        36
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      42

#define E1_STEP_PIN        43
#define E1_DIR_PIN         41
#define E1_ENABLE_PIN      39

#define E2_STEP_PIN        32
#define E2_DIR_PIN         47
#define E2_ENABLE_PIN      45

#define SDPOWER            -1
#define SDSS               4
#define LED_PIN            13

#define BEEPER             -1

#define FAN_PIN            -1

#define CONTROLLERFAN_PIN  -1 //Pin used for the fan to cool controller

#define PS_ON_PIN          -1

#define KILL_PIN           -1


#define HEATER_BED_PIN     8    // BED

#define HEATER_0_PIN       9
#define HEATER_1_PIN       10
#define HEATER_2_PIN       11

#define TEMP_BED_PIN       0   // ANALOG NUMBERING

#define TEMP_0_PIN         1   // ANALOG NUMBERING
#define TEMP_1_PIN         -1  // 2    // ANALOG NUMBERING
#define TEMP_2_PIN         -1  // 3     // ANALOG NUMBERING

#define TEMP_3_PIN         -1   // ANALOG NUMBERING
#define TEMP_4_PIN         -1   // ANALOG NUMBERING



  #ifdef NUM_SERVOS
    #define SERVO0_PIN         11

    #if NUM_SERVOS > 1
      #define SERVO1_PIN         6
    #endif

    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
    #endif

    #if NUM_SERVOS > 3
      #define SERVO3_PIN         4
    #endif
  #endif


  #ifdef ULTRA_LCD

    #ifdef NEWPANEL
      // ramps-fd lcd adaptor
      #define LCD_PINS_RS 16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4 23
      #define LCD_PINS_D5 25
      #define LCD_PINS_D6 27
      #define LCD_PINS_D7 29

      #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER
        #define BEEPER 37

        #define BTN_EN1 33
        #define BTN_EN2 31
        #define BTN_ENC 35

        #define SDCARDDETECT 49
        #endif

      #endif

  #endif //ULTRA_LCD


// SPI for Max6675 Thermocouple

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define MAX_SCK_PIN          52
  #define MAX_MISO_PIN         50
  #define MAX_MOSI_PIN         51
  #define MAX6675_SS       53
#else
  #define MAX6675_SS       49
#endif

#endif //MOTHERBOARD == 403, 404

// --------------------------------------------------------------------------
// 
// --------------------------------------------------------------------------


#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifdef DISABLE_MAX_ENDSTOPS
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
#define X_MIN_PIN          -1
#define Y_MIN_PIN          -1
#define Z_MIN_PIN          -1
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS _E2_PINS             \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_BED_PIN) }





#endif	/* _PINS_H */
