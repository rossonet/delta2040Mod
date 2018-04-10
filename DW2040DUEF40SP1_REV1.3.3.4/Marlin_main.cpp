/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

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
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

Modified by Dennis Patella WASPROJECT TEAM on 10/2015 
http://www.wasproject.it
 */
#include "Marlin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
#include "vector_3.h"
  #ifdef AUTO_BED_LEVELING_GRID
    #include "qr_solve.h"
  #endif
#endif // ENABLE_AUTO_BED_LEVELING

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "languagew.h"
#include "pins_arduino.h"
#include "math.h"
#include <avr/dtostrf.h>
#ifdef BLINKM
#include "BlinkM.h"
#include "Wire.h"
#endif

#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#define VERSION_STRING  "2.0"
//Per G667 e G668 by Rossonet
#define CONFIG_ECHO_START do{ if (true) SERIAL_ECHO_START; }while(0)
// look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G29 - automatic Z home set.
// G30 - Single Z Probe, probes bed at current XY location.
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given
// G667 - Auto calibration by Rossonet


// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M2   - Stop and save
// M3	- Resurrect
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
//        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
//        The '#' is necessary when calling from within sd files, as it stops buffer prereading
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
//        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
// M112 - Emergency stop
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homing offset
// M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y> E<0 1>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beep sound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D or T extruder type
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M401 - Lower z-probe if present
// M402 - Raise z-probe if present
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from EEPROM)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M665 - set delta configurations Ldiagonal rod Radius Segment A B C Zmax Maxiumradius Door
// M666 - set delta endstop adjustment
// M777 - set current for drivers (es. M777 C0 set current to 0   values must be from 0 to 254)
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;

#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
bool Estopped=false;
bool extFilCgn=false;
bool outOfPlate=false;
bool firstMove = false;
bool dontUpdateLcd = false;
bool wifisupported=false;
bool filSensor = true;
bool doorExcluded=true;
bool continuousPrint=false;
bool autolevel_enabled=false;
static bool check_autocalib=true;
uint8_t extr_type = 1;
int currentControlDriver;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100
  #if EXTRUDERS > 1
    , 100
    #if EXTRUDERS > 2
      , 100
    #endif
  #endif
};
float volumetric_multiplier[EXTRUDERS] = {1.0
  #if EXTRUDERS > 1
    , 1.0
    #if EXTRUDERS > 2
      , 1.0
    #endif
  #endif
};
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
#ifdef DELTA
float endstop_adj[3]={0,0,0};
#endif
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
float Z_home_pos = max_pos[2];
bool axis_known_position[3] = {false, false, false};
bool actualClaymode;
float zprobe_zoffset;
float Zcorrect;
float ZcorNoClay;
float resurrAtZ=0.0;
float tmpZcorrect;
bool makeResurrection=true;
bool testing=false;
bool DualExtMode=true;
bool extruderCheck=false;
bool DualSensorFilament=true;
bool canUpdateLcd=false;
bool donTchange=true;
uint8_t Lang;
bool clayMode;
bool wifi_off=true;
bool wifi_run=true;
int savingTempEX1=0;
int savingTempEX2=0;
DataStruct resurrectionData;
char  nameDir[13];
char tmpnameDir[13];
char  nameFile[13];
uint16_t resurrectionData_bed=0;

// Extruder offset


 

float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] ;

float extruder_offset_X[];
float extruder_offset_Y[];
float extruder_offset_Z[];

uint8_t active_extruder = 0;
float changeT[2][EXTRUDERS];
int defaultfan=DEFAULTFAN;
int fanSpeed=defaultfan;
#ifdef SERVO_ENDSTOPS
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=false;
  bool retracted=false;
  float retract_length = RETRACT_LENGTH;
  float retract_feedrate = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;
#endif

#ifdef ULTIPANEL
  #ifdef PS_DEFAULT_OFF
    bool powersupply = false;
  #else
	  bool powersupply = true;
  #endif
#endif

#ifdef DELTA
  float delta[3] = {0.0, 0.0, 0.0};
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  float dA,dB,dC;
  float dX,dY,dZ;
  bool overrideBed = false;
  
  // these are the default values, can be overriden with M665
  float delta_radius= DELTA_RADIUS;
  float delta_tower1_x= -SIN_60*(delta_radius+dA); // front left tower
  float delta_tower1_y= -COS_60*(delta_radius+dA);	   
  float delta_tower2_x=  SIN_60*(delta_radius+dB); // front right tower
  float delta_tower2_y= -COS_60*(delta_radius+dB);	   
  float delta_tower3_x= 0.0+dC;                  // back middle tower
  float delta_tower3_y= delta_radius+dC;
  float delta_diagonal_rod= DELTA_DIAGONAL_ROD;
  float delta_diagonal_rod_2X= sq(delta_diagonal_rod);
  float delta_diagonal_rod_2Y=delta_diagonal_rod_2X;
  float delta_diagonal_rod_2Z=delta_diagonal_rod_2X;
  float delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;
  float max_plater_radius = MAX_PLATER;
#endif					
uint32_t start_timeout_preheat = 0;

bool cancel_heatup = false ;
bool resurState = false;
bool ext0=true;
bool ext1=true;
bool firstCheck=true;
bool dontcheck=true;

//===========================================================================
//=============================Private Variables=============================
//===========================================================================
static bool firstCommand=true;
static bool showMsg=true;
static bool clearMsg=false;
static bool Xfault=false;
static bool Yfault=false;
static bool Zfault=false;
static bool initfault=true;
static unsigned long timeout_preheat = (TIMEOUT_PREHEAT * 1000) * 60;
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
static bool first=true; 
static bool clearStatus=false; 
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static char rawbuffer[RAWBUFSIZE][RAW_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
static int rawlen = 0;
static int rawindr =0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;
static bool prev_ext_fil = true;
static unsigned long previousMillis=0;
static int interval = 1000;
static void resurrecting();

bool Stopped=false;

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

bool CooldownNoWait = true;
bool target_direction;
bool fanInit=false;
//Insert variables if CHDK is defined
#ifdef CHDK
unsigned long chdkHigh = 0;
boolean chdkActive = false;
#endif

//===========================================================================
//=============================Routines======================================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }


//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}
void setup_ext_pin()
{
#if defined(CHECK_EXTR) && CHECK_EXTR > -1
	SET_INPUT(CHECK_EXTR);
	digitalWrite(CHECK_EXTR,HIGH);
#endif
  #if defined(EMERGENCY_STOP) && EMERGENCY_STOP > -1
    SET_INPUT(EMERGENCY_STOP);
	
    //WRITE(EMERGENCY_STOP,HIGH);
  #endif
  #if defined(EXT_FILAMENT) && EXT_FILAMENT > -1
    SET_INPUT(EXT_FILAMENT);
    
  #endif
  #if defined(EXT_FILAMENT2) && EXT_FILAMENT2 > -1
    SET_INPUT(EXT_FILAMENT2);
    
  #endif
  #if defined(WIFI_ON) && WIFI_ON > -1
    SET_OUTPUT(WIFI_ON);
	WRITE(WIFI_ON,HIGH);
    
  #endif
  #if defined(WIFI_RUN) && WIFI_RUN > -1
    SET_OUTPUT(WIFI_RUN);
	WRITE(WIFI_RUN,HIGH);
	
	//SET_INPUT(18);
	//SET_INPUT(19);
	
    
  #endif
  #if defined(EXT_PAUSE) && EXT_PAUSE > -1
  SET_INPUT(EXT_PAUSE);
  #endif
}
void setup_killpin()
{
#if defined(HARDRESET) && HARDRESET > -1
	WRITE(HARDRESET,HIGH);
	SET_OUTPUT(HARDRESET);
	WRITE(HARDRESET,HIGH);
#endif
  #if defined(KILL_PIN) && KILL_PIN > -1
    SET_INPUT(KILL_PIN);
    PULLUP(KILL_PIN,HIGH);
  #endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    PULLUP(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
	#if defined(PS_DEFAULT_OFF)
	  WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
	  WRITE(PS_ON_PIN, PS_ON_AWAKE);
	#endif
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif

  // Set position of Servo Endstops that are defined
  #ifdef SERVO_ENDSTOPS
  for(int8_t i = 0; i < 3; i++)
  {
    if(servo_endstops[i] > -1) {
      servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
    }
  }
  #endif

  #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
  delay(PROBE_SERVO_DEACTIVATION_DELAY);
  servos[servo_endstops[Z_AXIS]].detach();
  #endif
}

void setup()
{

  setup_killpin();
  setup_ext_pin();
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  WIFI.begin(WIFIBAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;
	
  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  
  MCUSR=0;
  #ifdef ARDUINO_ARCH_SAM
  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;   // faster ADC conversion on analog read on DUE 
  //PWMC_ConfigureClocks(10000 * 255 , 0, VARIANT_MCK);
  #endif
  SERIAL_ECHOPGM(MSG_MARLIN);
  
  SERIAL_ECHOLNPGM(VERSION_STRING);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif
  #endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
 
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
   if (check_eeprom()==false) {
	SERIAL_ERROR_START;
    SERIAL_ERRORPGM("EEPROM Failure!"); 
	LCD_ALERTMESSAGEPGM("EEPROM Failure!");
	SERIAL_ECHOLN("");
	
	Config_ResetDefault();
	} else {
  Config_RetrieveSettings();
  delay(500);
  recalc_delta_settings(delta_radius, delta_diagonal_rod);
  fanSpeed=defaultfan;
  
  
  for (uint8_t a=0;a<NUM_EXTRUDER_OFFSETS;a++) {
	  for (uint8_t b=0;b<EXTRUDERS;b++){
		if (a==0)extruder_offset[a][b] = extruder_offset_X[b];
		if (a==1)extruder_offset[a][b] = extruder_offset_Y[b];
		if (a==2)extruder_offset[a][b] = extruder_offset_Z[b];
	  }
	  
  }
  
  
  float tmpMov1[]=CHANGE_EXTRUDER1;
  float tmpMov2[]=CHANGE_EXTRUDER2;
  for (uint8_t e=0;e<EXTRUDERS;e++) {
	  for (uint8_t c=0;c<2;c++)
  (e==0)?changeT[c][e]=tmpMov1[c]:changeT[c][e]=tmpMov2[c];
  
  
  
  }
  /*
  changeT[0][0]=-125.0;
  changeT[1][0]=-60.0;
  changeT[0][1]=125.0;
  changeT[1][1]=60.0;*/
	Retrieve_ResurData();
	
  SERIAL_ECHO("Dual Extruder Mode: ");
  SERIAL_ECHOLN(DualExtMode);
  SERIAL_ECHOLN("-----------Resurrection data: ------------");
  SERIAL_ECHO("Active: ");
  SERIAL_ECHOLN(resurrectionData.resurrActive);
  SERIAL_ECHO("Autoresurr: ");
  SERIAL_ECHOLN(resurrectionData.autoResurr);
  SERIAL_ECHO("File: ");
  SERIAL_ECHO(nameDir);
  SERIAL_ECHO("/");
  SERIAL_ECHOLN(nameFile);
  SERIAL_ECHO("Pos: Z: ");
  SERIAL_ECHO(resurrectionData.high);
  SERIAL_ECHO(" X: ");
  SERIAL_ECHO(resurrectionData.X);
  SERIAL_ECHO(" Y: ");
  SERIAL_ECHO(resurrectionData.Y);
  SERIAL_ECHO(" E: ");
  SERIAL_ECHO(resurrectionData.E);
  SERIAL_ECHO("Feedrate: ");
  SERIAL_ECHO(resurrectionData.feedrate);
  SERIAL_ECHO(" ");
  SERIAL_ECHO(resurrectionData.feedmultiply);
  SERIAL_ECHOLN("%");
  SERIAL_ECHO("Fan: ");
  SERIAL_ECHO(resurrectionData.fan);
  SERIAL_ECHO(" Bed: ");
  SERIAL_ECHO(resurrectionData_bed);
  SERIAL_ECHO(" Extruder1: ");
  SERIAL_ECHO(resurrectionData.extruder);
  SERIAL_ECHO(" Extruder2: ");
  SERIAL_ECHOLN(resurrectionData.extruder2);
  SERIAL_ECHO("SDPOS: ");
  SERIAL_ECHOLN(resurrectionData.sdpos);
  SERIAL_ECHO("Ext Sel: ");
  SERIAL_ECHOLN(resurrectionData.ext_selected);
  SERIAL_ECHOLN("--------------------------------------------");
  
  }
  
  SERIAL_ECHO("Door sensor ");
  SERIAL_ECHOLN(!doorExcluded);
  if (clayMode) 
	{
	add_homeing[2] = Zcorrect;
	tmpZcorrect = Zcorrect;
 }
  else 
{
	add_homeing[2]=ZcorNoClay;
	tmpZcorrect=ZcorNoClay;
}
  if (!clayMode) {
	  tp_init();    // Initialize temperature loop
	  
	  }
  else {
	  filSensor = false;
	  set_extrude_min_temp(.0);  
  }
  actualClaymode=clayMode;
  if (DualExtMode) defaultfan=0;
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();

  lcd_init();
  _delay_ms(1000);	// wait 1sec to display the splash screen

  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #ifdef DIGIPOT_I2C
    digipot_i2c_init();
  #endif
  
  fanSpeed=200;
  fanInit=true;
  
	for (uint8_t i=0; i<sizeof(tmpnameDir); i++) 
		tmpnameDir[i]=0;  
}


void loop()
{
	
	if (card.saving) {
		if(rawlen < (RAWBUFSIZE-1))
			get_raw();
		if (rawlen) {
		
				card.write_command(rawbuffer[rawindr]);
				
				rawlen = (rawlen-1);
				rawindr = (rawindr + 1)%RAWBUFSIZE;
			
			
		}
		
		lcd_update();
		
		
		
		
	} else {
		if(buflen < (BUFSIZE-1))
			get_command();

		card.checkautostart(false);
 
		if(buflen)
		{
			process_commands();
			buflen = (buflen-1);
			bufindr = (bufindr + 1)%BUFSIZE;
		}
		//check heater every n milliseconds
		if (!clayMode) manage_heater();
		manage_inactivity();
		checkHitEndstops();
		lcd_update();
		if (fanInit) {
			fanInit=false;
			delay(1000);
			fanSpeed=defaultfan;
		}
		/*
		if (millis()<15000) {
			if (check_autocalib && (READ(SENS_BED)==LOW)) {
				check_autocalib=false;
				autolevel_enabled=true;
				lcd_setstatus("AUTOCALIB ENABLED  ");
			}
			
			
		}*/
				
	}
	/*
  if(buflen < (BUFSIZE-1))
    get_command();

  card.checkautostart(false);
 
  if(buflen)
  {
    
      if(card.saving)
      {
        if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
        {
          card.write_command(cmdbuffer[bufindr]);
          if(card.logging)
          {
            process_commands();
          }
		*/  /*
          else
          {
            (wifi_off)?SERIAL_PROTOCOLLNPGM(MSG_OK):WIFI_PROTOCOLLNPGM(MSG_OK); 
          }
		  *//*
        }
        else
        {
          card.closefile();
          (wifi_off)?SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED):WIFI_PROTOCOLLNPGM(MSG_FILE_SAVED);
		  lcd_setstatus(MSG_FILE_SAVED);
        }
      }
      else
      {
        process_commands();
      }
    
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
  //check heater every n milliseconds
  if (!clayMode) manage_heater();
  manage_inactivity();
  checkHitEndstops();
  lcd_update();
  if (fanInit) {
		fanInit=false;
		delay(1000);
		fanSpeed=defaultfan;
		
	} */
}
void get_raw() 
{
while( (wifi_off)?MYSERIAL.available():WIFI.available() > 0  && rawlen < RAWBUFSIZE) {
		serial_char = (wifi_off)?MYSERIAL.read():WIFI.read();
		if(serial_char == '\n' ||
		serial_char == '\r' ||
		serial_count >= (RAW_SIZE - 1) )
		{
			if (!serial_count) return;
			rawbuffer[rawindr][serial_count] = 0;
			rawlen ++;
			
			
			serial_count=0;
			if(strstr_P(rawbuffer[rawindr], PSTR("M29")) != NULL)
			{
					card.closefile();
					(wifi_off)?SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED):WIFI_PROTOCOLLNPGM(MSG_FILE_SAVED);
					lcd_setstatus(MSG_FILE_SAVED);
					rawindr=0;
					rawlen=0;
					break;
			}
			
		} else
			rawbuffer[rawindr][serial_count++] = serial_char;
	
	
	
	
	
	
}	
	
	
	
	
	
}
void get_command()
{
	
  while( (wifi_off)?MYSERIAL.available():WIFI.available() > 0  && buflen < BUFSIZE) {
    serial_char = (wifi_off)?MYSERIAL.read():WIFI.read();
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr(cmdbuffer[bufindw], 'N') != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
            if (wifi_off){
			SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
			} else {
			WIFI_ERROR_START;
            WIFI_ERRORPGM(MSG_ERR_LINE_NO);
            WIFI_ERRORLN(gcode_LastN);	
			}
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              if (wifi_off){
			  SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              SERIAL_ERRORLN(gcode_LastN);
			  }else{
			  WIFI_ERROR_START;
              WIFI_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              WIFI_ERRORLN(gcode_LastN);  
			  }
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else
          {
			  if (wifi_off){
				SERIAL_ERROR_START;
				SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
				SERIAL_ERRORLN(gcode_LastN);
			  }else{
				WIFI_ERROR_START;
				WIFI_ERRORPGM(MSG_ERR_NO_CHECKSUM);
				WIFI_ERRORLN(gcode_LastN);  
			  }
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
			  if (wifi_off){
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
			  }else {
			WIFI_ERROR_START;
            WIFI_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            WIFI_ERRORLN(gcode_LastN);  
			  }
            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
           
	if (Stopped == true) {
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM((!Lang)?MSG_STOPPED:MSG_1_STOPPED);
            }
            break;
          default:
            break;
          }

        }
		

        //If command was e-stop process now
        if(strcmp(cmdbuffer[bufindw], "M112") == 0)
          kill();
        
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  #ifdef SDSUPPORT
  if(!card.sdprinting || serial_count!=0){
    return;
  }

  //'#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
  // if it occurs, stop_buffering is triggered and the buffer is ran dry.
  // this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing

  static bool stop_buffering=false;
  if(buflen==0) stop_buffering=false;

  while( !card.eof()  && buflen < BUFSIZE && !stop_buffering) {
    int16_t n=card.get();
    serial_char = (char)n;
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == '#' && comment_mode == false) ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
    {
      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        lcd_setstatus(time);
        card.printingHasFinished();
        card.checkautostart(true);
		
		
		resurrectionData.resurrActive=false;
		Store_ResurData();
		
      }
      if(serial_char=='#')
        stop_buffering=true;

      if(!serial_count)
      {
        comment_mode = false; //for new command
        return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
//      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

  #endif //SDSUPPORT

}


float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#ifdef DUAL_X_CARRIAGE
  #if EXTRUDERS == 1 || defined(COREXY) \
      || !defined(X2_ENABLE_PIN) || !defined(X2_STEP_PIN) || !defined(X2_DIR_PIN) \
      || !defined(X2_HOME_POS) || !defined(X2_MIN_POS) || !defined(X2_MAX_POS) \
      || !defined(X_MAX_PIN) || X_MAX_PIN < 0
    #error "Missing or invalid definitions for DUAL_X_CARRIAGE mode."
  #endif
  #if X_HOME_DIR != -1 || X2_HOME_DIR != 1
    #error "Please use canonical x-carriage assignment" // the x-carriages are defined by their homing directions
  #endif

#define DXC_FULL_CONTROL_MODE 0
#define DXC_AUTO_PARK_MODE    1
#define DXC_DUPLICATION_MODE  2
static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

static float x_home_pos(int extruder) {
  if (extruder == 0)
    return base_home_pos(X_AXIS) + add_homeing[X_AXIS];
  else
    // In dual carriage mode the extruder offset provides an override of the
    // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
    // This allow soft recalibration of the second extruder offset position without firmware reflash
    // (through the M218 command).
    return (extruder_offset[X_AXIS][1] > 0) ? extruder_offset[X_AXIS][1] : X2_HOME_POS;
}

static int x_home_dir(int extruder) {
  return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
}

static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
static bool active_extruder_parked = false; // used in mode 1 & 2
static float raised_parked_position[NUM_AXIS]; // used in mode 1
static unsigned long delayed_move_time = 0; // used in mode 1
static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
static float duplicate_extruder_temp_offset = 0; // used in mode 2
bool extruder_duplication_enabled = false; // used in mode 2
#endif //DUAL_X_CARRIAGE

static void axis_is_at_home(int axis) {
#ifdef DUAL_X_CARRIAGE
  if (axis == X_AXIS) {
    if (active_extruder != 0) {
      current_position[X_AXIS] = x_home_pos(active_extruder);
      min_pos[X_AXIS] =          X2_MIN_POS;
      max_pos[X_AXIS] =          max(extruder_offset[X_AXIS][1], X2_MAX_POS);
      return;
    }
    else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
      current_position[X_AXIS] = base_home_pos(X_AXIS) + add_homeing[X_AXIS];
      min_pos[X_AXIS] =          base_min_pos(X_AXIS) + add_homeing[X_AXIS];
      max_pos[X_AXIS] =          min(base_max_pos(X_AXIS) + add_homeing[X_AXIS],
                                  max(extruder_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
      return;
    }
  }
#endif
  current_position[axis] = base_home_pos(axis) + add_homeing[axis];
  min_pos[axis] =          base_min_pos(axis) + add_homeing[axis];
  max_pos[axis] =          base_max_pos(axis) + add_homeing[axis];
}


static float probeZ() {
	float TP,a,b;
    float saved[NUM_AXIS];
	
	a = st_get_position_mm(Z_AXIS);
	for (int a=0; a<NUM_AXIS;a++){
		saved[a]=current_position[a];
		destination[a]=current_position[a];
	}	
	destination[Z_AXIS] = -10.0;
	calculate_delta(destination);
	plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], feedrate/300, active_extruder);
	st_synchronize();
			
	b = st_get_position_mm(Z_AXIS);
	
	for (int a=0; a<NUM_AXIS;a++){
		current_position[a]=destination[a];
	}	
	TP=saved[Z_AXIS]-(a-b);
	current_position[Z_AXIS] = TP;
	destination[Z_AXIS]=TP;
	calculate_delta(current_position);
	plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
	st_synchronize();
		
		for (int a=0; a<NUM_AXIS;a++){
		destination[a]=saved[a];
	}	
				
	prepare_move();
	return TP;
	
}














#ifdef ENABLE_AUTO_BED_LEVELING
#ifdef AUTO_BED_LEVELING_GRID
static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
    vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
    planeNormal.debug("planeNormal");
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    //bedLevel.debug("bedLevel");

    //plan_bed_level_matrix.debug("bed level before");
    //vector_3 uncorrected_position = plan_get_position_mm();
    //uncorrected_position.debug("position before");

    vector_3 corrected_position = plan_get_position();
//    corrected_position.debug("position after");
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // but the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

#else // not AUTO_BED_LEVELING_GRID

static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

    plan_bed_level_matrix.set_to_identity();

    vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
    vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
    vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);

    vector_3 from_2_to_1 = (pt1 - pt2).get_normal();
    vector_3 from_2_to_3 = (pt3 - pt2).get_normal();
    vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3).get_normal();
    planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));

    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

    vector_3 corrected_position = plan_get_position();
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // put the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset;

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

}

#endif // AUTO_BED_LEVELING_GRID

#endif
static void run_z_probe() {
      
    float start_z = current_position[Z_AXIS];
    long start_steps = st_get_position(Z_AXIS);

    // move down slowly until you find the bed
    feedrate = homing_feedrate[Z_AXIS] / 4;
    destination[Z_AXIS] = -10;
    //float zPosition = -10;
    //plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    prepare_move_raw(); // this will also set_current_to_destination
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // we have to let the planner know where we are right now as it is not where we said to go.
    long stop_steps = st_get_position(Z_AXIS);
    float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
    current_position[Z_AXIS] = mm;

    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
   
}

static void do_blocking_move_to(float x, float y, float z) {
   float oldFeedRate = feedrate;

   

      feedrate = XY_TRAVEL_SPEED;

      destination[X_AXIS] = x;
      destination[Y_AXIS] = y;
      destination[Z_AXIS] = z;
      prepare_move_raw(); // this will also set_current_to_destination
      st_synchronize();

  

    feedrate = oldFeedRate;
}

static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
    do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
}

static void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    previous_millis_cmd = millis();

    enable_endstops(true);
}

static void clean_up_after_endstop_move() {
#ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
#endif

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    previous_millis_cmd = millis();
}


/// Probe bed height at position (x,y), returns the measured z value
static float probe_pt(float x, float y, float z_before) {
  
  clean_up_after_endstop_move();
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before); // this also updates current_position
  testing=true;
  do_blocking_move_to(x, y, current_position[Z_AXIS]); // this also updates current_position
  setup_for_endstop_move();

  run_z_probe();
  float measured_z = current_position[Z_AXIS];


  SERIAL_PROTOCOLPGM(MSG_BED);
  SERIAL_PROTOCOLPGM(" x: ");
  SERIAL_PROTOCOL(x);
  SERIAL_PROTOCOLPGM(" y: ");
  SERIAL_PROTOCOL(y);
  SERIAL_PROTOCOLPGM(" z: ");
  SERIAL_PROTOCOL(measured_z);
  SERIAL_PROTOCOLPGM("\n");
  testing=false;
  return measured_z;
}



static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {
    int axis_home_dir = home_dir(axis);
#ifdef DUAL_X_CARRIAGE
    if (axis == X_AXIS)
      axis_home_dir = x_home_dir(active_extruder);
#endif

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);


    // Engage Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        if (axis==Z_AXIS) {
          ;
        }
	    else
      #endif
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
      }
    #endif

    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
#ifdef DELTA
    feedrate = homing_feedrate[axis]/10;
#else
    feedrate = homing_feedrate[axis]/2 ;
#endif
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
#ifdef DELTA
    // retrace by the amount specified in endstop_adj
    if (endstop_adj[axis] * axis_home_dir < 0) {
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      destination[axis] = endstop_adj[axis];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
    }
#endif
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      }
    #endif
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
    if (axis==Z_AXIS) retract_z_probe();
#endif

  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
void refresh_cmd_timeout(void)
{
  previous_millis_cmd = millis();
}

#ifdef FWRETRACT
  void retract(bool retracting) {
    if(retracting && !retracted) {
      destination[X_AXIS]=current_position[X_AXIS];
      destination[Y_AXIS]=current_position[Y_AXIS];
      destination[Z_AXIS]=current_position[Z_AXIS];
      destination[E_AXIS]=current_position[E_AXIS];
      current_position[E_AXIS]+=retract_length/volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      float oldFeedrate = feedrate;
      feedrate=retract_feedrate*60;
      retracted=true;
      prepare_move();
      current_position[Z_AXIS]-=retract_zlift;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      prepare_move();
      feedrate = oldFeedrate;
    } else if(!retracting && retracted) {
      destination[X_AXIS]=current_position[X_AXIS];
      destination[Y_AXIS]=current_position[Y_AXIS];
      destination[Z_AXIS]=current_position[Z_AXIS];
      destination[E_AXIS]=current_position[E_AXIS];
      current_position[Z_AXIS]+=retract_zlift;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      //prepare_move();
      current_position[E_AXIS]-=(retract_length+retract_recover_length)/volumetric_multiplier[active_extruder]; 
      plan_set_e_position(current_position[E_AXIS]);
      float oldFeedrate = feedrate;
      feedrate=retract_recover_feedrate*60;
      retracted=false;
      prepare_move();
      feedrate = oldFeedrate;
    }
  } //retract
#endif //FWRETRACT

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif
  if(code_seen('G'))
  {
	 
	
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
	if (Stopped == false) {
    		if (resurrAtZ==0.0) {
      
		  
		  
		  
        get_coordinates(); // For X Y Z E F
          #ifdef FWRETRACT
            if(autoretract_enabled)
            if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
              float echange=destination[E_AXIS]-current_position[E_AXIS];
              if((echange<-MIN_RETRACT && !retracted) || (echange>MIN_RETRACT && retracted)) { //move appears to be an attempt to retract or recover
                  current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
                  plan_set_e_position(current_position[E_AXIS]); //AND from the planner
                  retract(!retracted);
                  return;
              }
            }
          #endif //FWRETRACT
		  if (DualExtMode) {  
			 if(code_seen('X')) destination[X_AXIS]-=extruder_offset[X_AXIS][active_extruder];
			 if(code_seen('Y')) destination[Y_AXIS]-=extruder_offset[Y_AXIS][active_extruder];	
			 if(code_seen('Z')) destination[Z_AXIS]-=extruder_offset[Z_AXIS][active_extruder];	
		  }
        prepare_move();
        //ClearToSend();
        
		
		
	  
      		} else {
			get_coordinates();  
		 if (code_seen('Z'))
		 {
			
			if (((float)code_value() >= resurrAtZ+0.1))
		  	{
			if (firstMove) firstMove=false;
			else {
			resurrAtZ=0.0;
		  	prepare_move();
			}
			}
			
		  }
		 
		if (code_seen('E')) {
			current_position[E_AXIS]=(float)code_value();
			plan_set_e_position(current_position[E_AXIS]);
			}
		 
		  
		  
		  
		  
	  }
	}
      break;
    case 2: // G2  - CW ARC
      if ((Stopped == false) && (resurrAtZ==0.0)) {
        get_arc_coordinates();
        prepare_arc_move(true);
        
      }
      break;
    case 3: // G3  - CCW ARC
      if ((Stopped == false) && (resurrAtZ==0.0)) {
        get_arc_coordinates();
        prepare_arc_move(false);
        
      }
      break;
    case 4: // G4 dwell
      LCD_MESSAGEPGM((!Lang)?MSG_DWELL:MSG_1_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis()  < codenum ){
        if (!clayMode) manage_heater();
        manage_inactivity();
        lcd_update();
      }
      break;
      #ifdef FWRETRACT
      case 10: // G10 retract
        retract(true);
      break;
      case 11: // G11 retract_recover
        retract(false);
      break;
      #endif //FWRETRACT
    case 28: //G28 Home all Axis one at a time
	if(code_seen('H')) make_homing(false);
	else make_homing(true);
    break;
	  
	  
	  
	case 29: //G29 auto Test Zmax
	if (autolevel_enabled) gcode_G29(true);
	break;
	
#ifdef ENABLE_AUTO_BED_LEVELING
    case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
        {
            #if Z_MIN_PIN == -1
            #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
            #endif

            // Prevent user from running a G29 without first homing in X and Y
            if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
            {
                LCD_MESSAGEPGM((!Lang)?MSG_POSITION_UNKNOWN:MSG_1_POSITION_UNKNOWN);
                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM((!Lang)?MSG_POSITION_UNKNOWN:MSG_1_POSITION_UNKNOWN);
                break; // abort G29, since we don't know where we are
            }

            st_synchronize();
            // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
            //vector_3 corrected_position = plan_get_position_mm();
            //corrected_position.debug("position before G29");
            plan_bed_level_matrix.set_to_identity();
            vector_3 uncorrected_position = plan_get_position();
            //uncorrected_position.debug("position durring G29");
            current_position[X_AXIS] = uncorrected_position.x;
            current_position[Y_AXIS] = uncorrected_position.y;
            current_position[Z_AXIS] = uncorrected_position.z;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            setup_for_endstop_move();

            feedrate = homing_feedrate[Z_AXIS];
#ifdef AUTO_BED_LEVELING_GRID
            // probe at the points of a lattice grid

            int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
            int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);


            // solve the plane equation ax + by + d = z
            // A is the matrix with rows [x y 1] for all the probed points
            // B is the vector of the Z positions
            // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
            // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

            // "A" matrix of the linear system of equations
            double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
            // "B" vector of Z points
            double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];


            int probePointCounter = 0;
            bool zig = true;

            for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing)
            {
              int xProbe, xInc;
              if (zig)
              {
                xProbe = LEFT_PROBE_BED_POSITION;
                //xEnd = RIGHT_PROBE_BED_POSITION;
                xInc = xGridSpacing;
                zig = false;
              } else // zag
              {
                xProbe = RIGHT_PROBE_BED_POSITION;
                //xEnd = LEFT_PROBE_BED_POSITION;
                xInc = -xGridSpacing;
                zig = true;
              }

              for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++)
              {
                float z_before;
                if (probePointCounter == 0)
                {
                  // raise before probing
                  z_before = Z_RAISE_BEFORE_PROBING;
                } else
                {
                  // raise extruder
                  z_before = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
                }

                float measured_z = probe_pt(xProbe, yProbe, z_before);

                eqnBVector[probePointCounter] = measured_z;

                eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
                eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
                eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
                probePointCounter++;
                xProbe += xInc;
              }
            }
            clean_up_after_endstop_move();

            // solve lsq problem
            double *plane_equation_coefficients = qr_solve(AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);

            SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
            SERIAL_PROTOCOL(plane_equation_coefficients[0]);
            SERIAL_PROTOCOLPGM(" b: ");
            SERIAL_PROTOCOL(plane_equation_coefficients[1]);
            SERIAL_PROTOCOLPGM(" d: ");
            SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);


            set_bed_level_equation_lsq(plane_equation_coefficients);

            free(plane_equation_coefficients);

#else // AUTO_BED_LEVELING_GRID not defined

            // Probe at 3 arbitrary points
            // probe 1
            float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);

            // probe 2
            float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

            // probe 3
            float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

            clean_up_after_endstop_move();

            set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);


#endif // AUTO_BED_LEVELING_GRID
            st_synchronize();

            // The following code correct the Z height difference from z-probe position and hotend tip position.
            // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
            // When the bed is uneven, this height must be corrected.
            real_z = float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
            x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
            z_tmp = current_position[Z_AXIS];

            apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
            current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        }
        break;

    case 30: // G30 Single Z Probe
        {
            engage_z_probe(); // Engage Z Servo endstop if available

            st_synchronize();
            // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
            setup_for_endstop_move();

            feedrate = homing_feedrate[Z_AXIS]/4;

            run_z_probe();
            SERIAL_PROTOCOLPGM(MSG_BED);
            SERIAL_PROTOCOLPGM(" X: ");
            SERIAL_PROTOCOL(current_position[X_AXIS]);
            SERIAL_PROTOCOLPGM(" Y: ");
            SERIAL_PROTOCOL(current_position[Y_AXIS]);
            SERIAL_PROTOCOLPGM(" Z: ");
            SERIAL_PROTOCOL(current_position[Z_AXIS]);
            SERIAL_PROTOCOLPGM("\n");

            clean_up_after_endstop_move();

            retract_z_probe(); // Retract Z Servo endstop if available
        }
        break;
#endif // ENABLE_AUTO_BED_LEVELING
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           if(i == E_AXIS) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
             
             current_position[i] = code_value();
			#ifndef DELTA
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
			#else
			 calculate_delta(current_position);
			 plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
			#endif
		   }
        }
      }
	  
      break;
	case 666:
	if (autolevel_enabled) {
		autolevel_enabled=false;
		lcd_setstatus("AUTOCALIB DISABLED ");
	} else {
		
		autolevel_enabled=true;
		lcd_setstatus("AUTOCALIB ENABLED  ");
		
	}
				
	break;
	case 667: // implementazione tuning automatico
	if (autolevel_enabled) gcode_G667();
	break;
	
    }
	if (!resurState) {
		dontcheck=false;
	}
	else dontcheck=true; 
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
#ifdef ULTIPANEL
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {
      LCD_MESSAGEPGM((!Lang)?MSG_USERWAIT:MSG_1_USERWAIT);
      codenum = 0;
      char *src = strchr_pointer + 2;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
      starpos = strchr(src, '*');
      if (starpos != NULL) *(starpos) = '\0';
      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0){
        codenum += millis();  // keep track of when we started waiting
        while(millis()  < codenum && !lcd_clicked()){
          if (!clayMode) manage_heater();
          manage_inactivity();
          lcd_update();
        }
      }else{
        while(!lcd_clicked()){
          if (!clayMode) manage_heater();
          manage_inactivity();
          lcd_update();
        }
      }
      LCD_MESSAGEPGM((!Lang)?MSG_RESUMING:MSG_1_RESUMING);
    }
    break;
#endif
	case 2:  //M2 
		Stopped=true;
		resurState=true;
		dontcheck=true;
		card.sdprinting = false;
		quickStop();
		resurrectionData.high = current_position[Z_AXIS];
		resurrectionData.X = current_position[X_AXIS];
		resurrectionData.Y = current_position[Y_AXIS];
		resurrectionData.E = current_position[E_AXIS];
		resurrectionData.feedmultiply = feedmultiply;
		resurrectionData.feedrate = feedrate;
		resurrectionData.extruder = degTargetHotend(0);
		if (DualExtMode)
		resurrectionData.extruder2 = degTargetHotend(1);
		//resurrectionData.bed = degTargetBed();
		resurrectionData.fan=fanSpeed;
		resurrectionData.sdpos = card.sdPosition();
		resurrectionData.resurrActive=true;
		resurrectionData.ext_selected=active_extruder;
		if (!clayMode)	autotempShutdown();
		card.closefile(); 
		Store_ResurData();		
		LCD_ALERTMESSAGEPGM(MSG_EMERGENCY_SAVE);
		
		//bufindr = 0;
		//bufindw = 0;
		//buflen = 0;
		serial_count = 0;
		if (!clayMode) { 
    	 		setTargetHotend0(0);
       			setTargetHotend1(0);
         		setTargetHotend2(0);
         		setTargetBed(0);
        	}
        	fanSpeed=defaultfan;
		enquecommand("G28");
		lcd_reset_alert_level();

		//while(!lcd_clicked()){}
	break;
	case 3:  //M3
		if (resurrectionData.resurrActive) {
			resurrectionData.resurrActive=false;
			resurrecting();

		}
	break;
	case 4:  //M4 Free z system
		if (code_seen('Z')) {
			resurrAtZ=code_value();
		}
		char message[20];
		sprintf(message,"Wait Z:");
		strcat(message,ftostr32(resurrAtZ));
		strcat(message,0);
		lcd_setstatus(message);
	break;
	case 5:  //M5 Stop
		Stop();
	break;
    case 17:
        LCD_MESSAGEPGM((!Lang)?MSG_NO_MOVE:MSG_1_NO_MOVE);
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
        enable_e1();
        enable_e2();
      break;

#ifdef SDSUPPORT
    case 20: // M20 - list SD card
      (wifi_off)?SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST):WIFI_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      (wifi_off)?SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST):WIFI_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      break;
    case 21: // M21 - init SD card

      card.initsd();

      break;
    case 22: //M22 - release SD card
      card.release();

      break;
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos)='\0';
		char filePrinting[13];
		sprintf(filePrinting,(strchr_pointer + 4));
		card.openFile(strchr_pointer + 4,true);
		memcpy(nameFile,filePrinting,strlen(filePrinting)+1);
		nameFile[strlen(filePrinting)]=0;
		
		
      break;
    case 24: //M24 - Start SD print
	if (resurrectionData.resurrActive) {
		resurrectionData.resurrActive=false;
		resurrectionData.high = 0.0;
		resurrectionData.X = 0.0;
		resurrectionData.Y = 0.0;
		resurrectionData.E = 0.0;
		resurrectionData.fan=0;
		resurrectionData_bed=0;
		resurrectionData.extruder = 0.0;
		
		}
      card.startFileprint();
      starttime=millis();
	  resurState=false;
	  sprintf(nameDir,tmpnameDir);
	  Store_ResurFile();
		dontcheck=true;
      break;
    case 25: //M25 - Pause SD print
      
    
    
    
      //card.pauseSDPrint();   //original line
      {
        float target[4];
        float lastpos[4];
        target[X_AXIS]=current_position[X_AXIS];
        target[Y_AXIS]=current_position[Y_AXIS];
        target[Z_AXIS]=current_position[Z_AXIS];
        target[E_AXIS]=current_position[E_AXIS];
        lastpos[X_AXIS]=current_position[X_AXIS];
        lastpos[Y_AXIS]=current_position[Y_AXIS];
        lastpos[Z_AXIS]=current_position[Z_AXIS];
        lastpos[E_AXIS]=current_position[E_AXIS];
        

        //lift Z
        
          #ifdef FILAMENTCHANGE_ZADD
            target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
          #endif
#ifdef DELTA       
	calculate_delta(target);
        //plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
        //move xy
#else        
        
	plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
#endif	
        
        
       
        

        
        //finish moves
        st_synchronize();
        //disable extruder steppers so filament can be removed
        disable_e0();
        disable_e1();
        disable_e2();
        delay(50);
        LCD_ALERTMESSAGEPGM((!Lang)?MSG_PAUSE_PRINT:MSG_1_PAUSE_PRINT);
        uint8_t cnt=0;
        while(!lcd_clicked()){
          cnt++;
          if (!clayMode) manage_heater();
          manage_inactivity();
          lcd_update();
          if(cnt==0)
          {
          #if BEEPER > 0
            SET_OUTPUT(BEEPER);

            WRITE(BEEPER,HIGH);
            delay(300);
            WRITE(BEEPER,LOW);
            delay(100);
          #else
			#if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
              lcd_buzz(1000/6,100);
			#else
			  lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS,LCD_FEEDBACK_FREQUENCY_HZ);
			#endif
          #endif
          }
        }

        //return to normal
        lcd_reset_alert_level();
	
#ifdef DELTA  	
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //should do nothing
	calculate_delta(lastpos);        
	plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //move xy back
#else
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //should do nothing
	        
		plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); 
#endif        
    }
      
      break;
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
	if (card.cardOK) {
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
	  lcd_setstatus("Write to file");
	  } else {
		  if (wifi_off) 
				SERIAL_ERRORLNPGM("SD not ok.");
		   else 
				WIFI_ERRORLNPGM("SD not ok.");
	  }
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //in loop()
	  
      break;
    case 30: //M30 <filename> Delete File
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 32: //M32 - Select file and start SD print
    {
      if(card.sdprinting) {
        st_synchronize();

      }
      starpos = (strchr(strchr_pointer + 4,'*'));

      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
      if(namestartpos==NULL)
      {
        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
      }
      else
        namestartpos++; //to skip the '!'

      if(starpos!=NULL)
        *(starpos)='\0';

      bool call_procedure=(code_seen('P'));

      if(strchr_pointer>namestartpos)
        call_procedure=false;  //false alert, 'P' found within filename

      if( card.cardOK )
      {
        
        card.openFile(namestartpos,true,!call_procedure);
        if(code_seen('S'))
          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
            card.setIndex(code_value_long());
            feedrate = 3600.0;   // 60mm/sec
            feedmultiply = 100;	 // 100% feedrate
        card.startFileprint();
        if(!call_procedure)
          starttime=millis(); //procedure calls count as normal print time.
      }
    } break;
    case 928: //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

#endif //SDSUPPORT

    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      lcd_setstatus(time);
      autotempShutdown();
      }
      break;
    case 42: //M42 -Change pin status via gcode
      if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          SET_OUTPUT(pin_number);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
     break;
    case 104: // M104
    if (!clayMode) {
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
        setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
      setWatch();
}
      break;
    case 112: //  M112 -Emergency Stop
		lcd_setstatus(" STOP ");
		DISABLE_STEPPER_DRIVER_INTERRUPT();
		disable_pwm();	
		WRITE(STEPPER_RESET,LOW);
		WRITE(HEATER_BED_PIN,LOW);
		SET_INPUT(X_STEP_PIN);
		SET_INPUT(X_DIR_PIN);
		SET_INPUT(X_ENABLE_PIN);
		SET_INPUT(Y_STEP_PIN);
		SET_INPUT(Y_DIR_PIN);
		SET_INPUT(Y_ENABLE_PIN);
		SET_INPUT(Z_STEP_PIN);
		SET_INPUT(Z_DIR_PIN);
		SET_INPUT(Z_ENABLE_PIN);
		SET_INPUT(E0_STEP_PIN);
		SET_INPUT(E0_DIR_PIN);
		SET_INPUT(E0_ENABLE_PIN);
		SET_INPUT(LCD_PINS_ENABLE);
		SET_INPUT(LCD_PINS_D4);
		SET_INPUT(LCD_PINS_D5);
		SET_INPUT(LCD_PINS_D6);
		SET_INPUT(LCD_PINS_D7);
		SET_INPUT(LCD_PINS_RS);
      kill();
	  
      break;
    case 140: // M140 set bed temp
if (!clayMode) {
      if (code_seen('S')) setTargetBed(code_value());
	  resurrectionData_bed=code_value();
	  Store_tempbed(RESURR_OFFSET+sizeof(nameDir)+sizeof(nameFile)+sizeof(resurrectionData));
}
      break;
    case 105 : // M105
if (!clayMode) {
      if(setTargetedHotend(105)){
        break;
        }
		if(wifi_off) {
      #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
        SERIAL_PROTOCOLPGM("ok T:");
        SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
          SERIAL_PROTOCOLPGM(" B:");
          SERIAL_PROTOCOL_F(degBed(),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetBed(),1);
        #endif //TEMP_BED_PIN
		if (DualExtMode) {
        for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
          SERIAL_PROTOCOLPGM(" T");
          SERIAL_PROTOCOL(cur_extruder);
          SERIAL_PROTOCOLPGM(":");
          SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetHotend(cur_extruder),1);
        }
		} else {
			
			
		int8_t cur_extruder=0;	
		SERIAL_PROTOCOLPGM(" T");
          SERIAL_PROTOCOL(cur_extruder);
          SERIAL_PROTOCOLPGM(":");
          SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetHotend(cur_extruder),1);	
			
			
		}
      #else
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
      #endif

        SERIAL_PROTOCOLPGM(" @:");
      #ifdef EXTRUDER_WATTS
        SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(tmp_extruder))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));
      #endif

        SERIAL_PROTOCOLPGM(" B@:");
      #ifdef BED_WATTS
        SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(-1));
      #endif

        #ifdef SHOW_TEMP_ADC_VALUES
          #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
            SERIAL_PROTOCOLPGM("    ADC B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
          #endif
          for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
            SERIAL_PROTOCOLPGM("  T");
            SERIAL_PROTOCOL(cur_extruder);
            SERIAL_PROTOCOLPGM(":");
            SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
          }
        #endif

        SERIAL_PROTOCOLLN("");
		} else {
		      #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
        WIFI_PROTOCOLPGM("ok T:");
        WIFI_PROTOCOL_F(degHotend(tmp_extruder),1);
        WIFI_PROTOCOLPGM(" /");
        WIFI_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
          WIFI_PROTOCOLPGM(" B:");
          WIFI_PROTOCOL_F(degBed(),1);
          WIFI_PROTOCOLPGM(" /");
          WIFI_PROTOCOL_F(degTargetBed(),1);
        #endif //TEMP_BED_PIN
		if (DualExtMode) {
        for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
			WIFI_PROTOCOLPGM(" T");
			WIFI_PROTOCOL(cur_extruder);
			WIFI_PROTOCOLPGM(":");
			WIFI_PROTOCOL_F(degHotend(cur_extruder),1);
			WIFI_PROTOCOLPGM(" /");
			WIFI_PROTOCOL_F(degTargetHotend(cur_extruder),1);
				
				
			}
		} else {
		int8_t cur_extruder = 0;
          WIFI_PROTOCOLPGM(" T");
          WIFI_PROTOCOL(cur_extruder);
          WIFI_PROTOCOLPGM(":");
          WIFI_PROTOCOL_F(degHotend(cur_extruder),1);
          WIFI_PROTOCOLPGM(" /");
          WIFI_PROTOCOL_F(degTargetHotend(cur_extruder),1);
        }
      #else
        WIFI_ERROR_START;
        WIFI_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
      #endif

        WIFI_PROTOCOLPGM(" @:");
      #ifdef EXTRUDER_WATTS
        WIFI_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(tmp_extruder))/127);
        WIFI_PROTOCOLPGM("W");
      #else
        WIFI_PROTOCOL(getHeaterPower(tmp_extruder));
      #endif

        WIFI_PROTOCOLPGM(" B@:");
      #ifdef BED_WATTS
        WIFI_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
        WIFI_PROTOCOLPGM("W");
      #else
        WIFI_PROTOCOL(getHeaterPower(-1));
      #endif

        #ifdef SHOW_TEMP_ADC_VALUES
          #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
            WIFI_PROTOCOLPGM("    ADC B:");
            WIFI_PROTOCOL_F(degBed(),1);
            WIFI_PROTOCOLPGM("C->");
            WIFI_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
          #endif
		  if (DualExtMode) {
          for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
            WIFI_PROTOCOLPGM("  T");
            WIFI_PROTOCOL(cur_extruder);
            WIFI_PROTOCOLPGM(":");
            WIFI_PROTOCOL_F(degHotend(cur_extruder),1);
            WIFI_PROTOCOLPGM("C->");
            WIFI_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
          }
		  } else {
			  
			int8_t cur_extruder = 0;
			WIFI_PROTOCOLPGM("  T");
            WIFI_PROTOCOL(cur_extruder);
            WIFI_PROTOCOLPGM(":");
            WIFI_PROTOCOL_F(degHotend(cur_extruder),1);
            WIFI_PROTOCOLPGM("C->");
            WIFI_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);	
			  
			  
			  
			  
		  }
        #endif

        WIFI_PROTOCOLLN("");	
		}
}
      return;
      break;
    case 109:
    {// M109 - Wait for extruder heater to reach target.
if (!clayMode) {
      if(setTargetedHotend(109)){
        break;
      }
      LCD_MESSAGEPGM((!Lang)?MSG_HEATING:MSG_1_HEATING);
      #ifdef AUTOTEMP
        autotemp_enabled=false;
      #endif
      if (code_seen('S')) {
	savingTempEX1=code_value(); //TODO 2 EXTR
	target_temperature[tmp_extruder]=code_value();
	if (degHotend(tmp_extruder) > code_value()) break;
        setTargetHotend(code_value(), tmp_extruder);
        
#ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
        CooldownNoWait = true;
      } else if (code_seen('R')) {
		  savingTempEX1=code_value(); //TODO 2 EXTR
        setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
        CooldownNoWait = false;
      }
      #ifdef AUTOTEMP
        if (code_seen('S')) autotemp_min=code_value();
        if (code_seen('B')) autotemp_max=code_value();
        if (code_seen('F'))
        {
          autotemp_factor=code_value();
          autotemp_enabled=true;
        }
      #endif

      setWatch();
      codenum = millis();

      /* See if we are heating up or cooling down */
      target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling
      
      cancel_heatup = false;

      #ifdef TEMP_RESIDENCY_TIME
        long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp
          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
        while((!cancel_heatup)&&((residencyStart == -1) ||
              (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL)))) ) {
      #else
        while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
      #endif //TEMP_RESIDENCY_TIME
          if( (millis() - codenum) > 1000UL )
          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)tmp_extruder);
            #ifdef TEMP_RESIDENCY_TIME
              SERIAL_PROTOCOLPGM(" W:");
              if(residencyStart > -1)
              {
                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
                 SERIAL_PROTOCOLLN( codenum );
              }
              else
              {
                 SERIAL_PROTOCOLLN( "?" );
              }
            #else
              SERIAL_PROTOCOLLN("");
            #endif
            codenum = millis();
          }
          if (!clayMode) manage_heater();
          manage_inactivity();
          lcd_update();
        #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
              or when current temp falls outside the hysteresis after target temp was reached */
          if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
          {
            residencyStart = millis();
          }
        #endif //TEMP_RESIDENCY_TIME
        }
        LCD_MESSAGEPGM((!Lang)?MSG_HEATING_COMPLETE:MSG_1_HEATING_COMPLETE);
        starttime=millis();
        previous_millis_cmd = millis();
      }
}
      break;
    case 190: // M190 - Wait for bed heater to reach target.
if (!clayMode) {
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        LCD_MESSAGEPGM((!Lang)?MSG_BED_HEATING:MSG_1_BED_HEATING);
        if (code_seen('S')) {
          setTargetBed(code_value());
		  resurrectionData_bed=code_value();
		  Store_tempbed(RESURR_OFFSET+sizeof(nameDir)+sizeof(nameFile)+sizeof(resurrectionData));

          CooldownNoWait = true;
        } else if (code_seen('R')) {
          setTargetBed(code_value());
		  resurrectionData_bed=code_value();
		  Store_tempbed(RESURR_OFFSET+sizeof(nameDir)+sizeof(nameFile)+sizeof(resurrectionData));

          CooldownNoWait = false;
        }
        codenum = millis();
        
        cancel_heatup = false;
        target_direction = isHeatingBed(); // true if heating, false if cooling

        while ( (target_direction)&&(!cancel_heatup) ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false)) )
        {
          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            float tt=degHotend(active_extruder);
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL(tt);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)active_extruder);
            SERIAL_PROTOCOLPGM(" B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLLN("");
            codenum = millis();
          }
          if (!clayMode) manage_heater();
          manage_inactivity();
          lcd_update();
        }
        LCD_MESSAGEPGM((!Lang)?MSG_BED_DONE:MSG_1_BED_DONE);
        previous_millis_cmd = millis();
    #endif
}
        break;

    #if defined(FAN_PIN) && FAN_PIN > -1
      case 106: //M106 Fan On
        if (code_seen('S')){
			if (DualExtMode) {
				(code_value()!=0)?fanSpeed=constrain(code_value(),50,MAXFAN):fanSpeed=DEFAULTFAN; 
				
				
			} else {
				
				(code_value()!=0)?fanSpeed=constrain(code_value(),defaultfan,MAXFAN):fanSpeed=defaultfan; 
				
				
			}
			/*
        	if (code_value()<defaultfan) {
        		fanSpeed=defaultfan;
        	} else {
        		fanSpeed=code_value();
        		}
           fanSpeed=constrain(fanSpeed,defaultfan,MAXFAN);
		   */
        }
        else 
        if (code_seen('L')) {
			defaultfan=code_value();
			defaultfan=constrain(defaultfan,DEFAULTFAN,MAXFAN);
			delay(500);
			Config_StoreSettings();
			}
			else
				fanSpeed=MAXFAN;
        break;
      case 107: //M107 Fan Off
        (DualExtMode)?fanSpeed = DEFAULTFAN:fanSpeed = defaultfan;
        break;
    #endif //FAN_PIN
    #ifdef BARICUDA
      // PWM for HEATER_1_PIN
      #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
        case 126: //M126 valve open
          if (code_seen('S')){
             ValvePressure=constrain(code_value(),0,255);
          }
          else {
            ValvePressure=255;
          }
          break;
        case 127: //M127 valve closed
          ValvePressure = 0;
          break;
      #endif //HEATER_1_PIN

      // PWM for HEATER_2_PIN
      #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
        case 128: //M128 valve open
          if (code_seen('S')){
             EtoPPressure=constrain(code_value(),0,255);
          }
          else {
            EtoPPressure=255;
          }
          break;
        case 129: //M129 valve closed
          EtoPPressure = 0;
          break;
      #endif //HEATER_2_PIN
    #endif

    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
      case 80: // M80 - Turn on Power Supply
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);

        // If you have a switch on suicide pin, this is useful
        // if you want to start another print with suicide feature after
        // a print without suicide...
        #if defined SUICIDE_PIN && SUICIDE_PIN > -1
            SET_OUTPUT(SUICIDE_PIN);
            WRITE(SUICIDE_PIN, HIGH);
        #endif

        #ifdef ULTIPANEL
          powersupply = true;
          
          LCD_MESSAGEPGM(WELCOME_MSG);
          lcd_update();
        #endif
        break;
      #endif

      case 81: // M81 - Turn off Power Supply
        disable_heater();
        st_synchronize();
        disable_e0();
        disable_e1();
        disable_e2();
        finishAndDisableSteppers();
        fanSpeed = defaultfan;
        delay(1000); // Wait a little before to switch off
      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        st_synchronize();
        suicide();
      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
      #ifdef ULTIPANEL
        powersupply = false;
        LCD_MESSAGEPGM(MACHINE_NAME" "MSG_OFF".");
        lcd_update();
      #endif
	  break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
          #endif
        }
      }
      break;
    case 85: // M85
      code_seen('S');
      max_inactive_time = code_value() * 1000;
      break;
    case 92: // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;
    case 117: // M117 display message
	  dontUpdateLcd=true;
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;
    case 114: // M114
	
	if (wifi_off) {
		
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM(" E:");
      SERIAL_PROTOCOL(current_position[E_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_PROTOCOLLN("");
	  } else {
	  WIFI_PROTOCOLPGM("X:");
      WIFI_PROTOCOL(current_position[X_AXIS]);
      WIFI_PROTOCOLPGM(" Y:");
      WIFI_PROTOCOL(current_position[Y_AXIS]);
      WIFI_PROTOCOLPGM(" Z:");
      WIFI_PROTOCOL(current_position[Z_AXIS]);
      WIFI_PROTOCOLPGM(" E:");
      WIFI_PROTOCOL(current_position[E_AXIS]);

      WIFI_PROTOCOLPGM(MSG_COUNT_X);
      WIFI_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      WIFI_PROTOCOLPGM(" Y:");
      WIFI_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      WIFI_PROTOCOLPGM(" Z:");
      WIFI_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      WIFI_PROTOCOLLN("");  
	  }
      break;
    case 120: // M120
      enable_endstops(false) ;
      break;
    case 121: // M121
      enable_endstops(true) ;
      break;
    case 119: // M119
    SERIAL_PROTOCOLLN(MSG_M119_REPORT);
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MIN);
        SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
        SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
        SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      break;
      //TODO: update for all axis, use for loop
    #ifdef BLINKM
    case 150: // M150
      {
        byte red;
        byte grn;
        byte blu;

        if(code_seen('R')) red = code_value();
        if(code_seen('U')) grn = code_value();
        if(code_seen('B')) blu = code_value();

        SendColors(red,grn,blu);
      }
      break;
    #endif //BLINKM
    case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
      {
        float area = .0;
        float radius = .0;
        if(code_seen('D')) {
          radius = (float)code_value() * .5;
          if(radius == 0) {
            area = 1;
          } else {
            area = M_PI * pow(radius, 2);
          }
        } else {
          //reserved for setting filament diameter via UFID or filament measuring device
          break;
        }
        tmp_extruder = active_extruder;
        if(code_seen('T')) {
          tmp_extruder = code_value();
          if(tmp_extruder >= EXTRUDERS) {
            SERIAL_ECHO_START;
            SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
          }
        }
        volumetric_multiplier[tmp_extruder] = 1 / area;
      }
      break;
    case 201: // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
    #endif
    case 203: // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 acclereration S normal moves T filmanent only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 additional homeing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
      }
      break;
    #ifdef DELTA
	case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
		if(code_seen('L')) {
			delta_diagonal_rod= code_value();
		}
		if(code_seen('R')) {
			delta_radius= code_value();
		}
		if(code_seen('S')) {
			delta_segments_per_second= code_value();
		}
		if(code_seen('A')) {
			dA= code_value();
		}
		if(code_seen('B')) {
			dB= code_value();
		}
		if(code_seen('C')) {
			dC= code_value();
		}
		if(code_seen('E')) {
			dX= code_value();
		}
		if(code_seen('F')) {
			dY= code_value();
		}
		if(code_seen('G')) {
			dZ= code_value();
		}
		if (code_seen('Z')) {
			float Ztemp = code_value();
			if (clayMode) {
				Zcorrect = (MANUAL_Z_HOME_POS - Ztemp) * -1;
				add_homeing[2]=Zcorrect;
				tmpZcorrect=Zcorrect;
				}
			else 
			{
				ZcorNoClay = (MANUAL_Z_HOME_POS - Ztemp) * -1;
				add_homeing[2]=ZcorNoClay;
				tmpZcorrect=ZcorNoClay;
				}
		}
		if (code_seen('M')) {
			max_plater_radius = code_value();
		}
		recalc_delta_settings(delta_radius, delta_diagonal_rod);
		if (code_seen('D')) {
			doorExcluded = !(bool)code_value();
		}
		break;
    case 666: // M666 set delta endstop adjustemnt
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) endstop_adj[i] = code_value();
      }
      break;
    #endif
	#ifdef WASP
	case 777: // M777 set current for drivers
      
        if(code_seen('C')) currentControlDriver = code_value();
		currentControlDriver=constrain(currentControlDriver,0,254);
      
      break;
	#endif
    #ifdef FWRETRACT
    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
    {
      if(code_seen('S'))
      {
        retract_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_feedrate = code_value()/60 ;
      }
      if(code_seen('Z'))
      {
        retract_zlift = code_value() ;
      }
    }break;
    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
    {
      if(code_seen('S'))
      {
        retract_recover_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_recover_feedrate = code_value()/60 ;
      }
    }break;
    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S'))
      {
        int t= code_value() ;
        switch(t)
        {
          case 0: autoretract_enabled=false;retracted=false;break;
          case 1: autoretract_enabled=true;retracted=false;break;
          default:
            SERIAL_ECHO_START;
            SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
            SERIAL_ECHO(cmdbuffer[bufindr]);
            SERIAL_ECHOLNPGM("\"");
        }
      }

    }break;
    #endif // FWRETRACT
    
    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
		if (DualExtMode) {
      uint8_t tmpExt=0;
	  if(code_seen('T')) tmpExt=code_value();
      if(code_seen('X'))
      {
        extruder_offset[X_AXIS][tmpExt] = code_value();
		extruder_offset_X[tmpExt] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][tmpExt] = code_value();
		extruder_offset_Y[tmpExt] = code_value();
      }
      if(code_seen('Z'))
      {
        extruder_offset[Z_AXIS][tmpExt] = code_value();
		extruder_offset_Z[tmpExt] = code_value();
      }
	  if(code_seen('E')) {
		  DualSensorFilament = (bool)code_value();
		  Config_StoreSettings();  
	  }
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
      for(tmpExt = 0; tmpExt < EXTRUDERS; tmpExt++)
      {
         SERIAL_ECHO(" ");
         SERIAL_ECHO(extruder_offset[X_AXIS][tmpExt]);
         SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[Y_AXIS][tmpExt]);
		 SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[Z_AXIS][tmpExt]);
      
	  
      }
	  SERIAL_ECHO("      Dual Sensor=");
      SERIAL_ECHO(DualSensorFilament);
      SERIAL_ECHOLN("");
	  }
    }break;
   
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        int tmp_code = code_value();
        if (code_seen('T'))
        {
          if(setTargetedHotend(221)){
            break;
          }
          extruder_multiply[tmp_extruder] = tmp_code;
        }
        else
        {
          extrudemultiply = tmp_code ;
        }
      }
    }
    break;

	case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
	{
      if(code_seen('P')){
        int pin_number = code_value(); // pin number
        int pin_state = -1; // required pin state - default is inverted

        if(code_seen('S')) pin_state = code_value(); // required pin state

        if(pin_state >= -1 && pin_state <= 1){

          for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
          {
            if (sensitive_pins[i] == pin_number)
            {
              pin_number = -1;
              break;
            }
          }

          if (pin_number > -1)
          {
            st_synchronize();

            pinMode(pin_number, INPUT);

            int target;
            switch(pin_state){
            case 1:
              target = HIGH;
              break;

            case 0:
              target = LOW;
              break;

            case -1:
              target = !digitalRead(pin_number);
              break;
            }

            while(digitalRead(pin_number) != target){
              if (!clayMode) manage_heater();
              manage_inactivity();
              lcd_update();
            }
          }
        }
      }
    }
    break;

    #if NUM_SERVOS > 0
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index = code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		      servos[servo_index].attach(0);
#endif
            servos[servo_index].write(servo_position);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
              delay(PROBE_SERVO_DEACTIVATION_DELAY);
              servos[servo_index].detach();
#endif
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOL(servos[servo_index].read());
          SERIAL_PROTOCOLLN("");
        }
      }
      break;
    #endif // NUM_SERVOS > 0

    #if (LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)))
    case 300: // M300
    {
      int beepS = code_seen('S') ? code_value() : 110;
      int beepP = code_seen('P') ? code_value() : 1000;
      if (beepS > 0)
      {
        #if BEEPER > 0
          tone(BEEPER, beepS);
          delay(beepP);
          noTone(BEEPER);
        #elif defined(ULTRALCD)
		  lcd_buzz(beepS, beepP);
		#elif defined(LCD_USE_I2C_BUZZER)
		  lcd_buzz(beepP, beepS);
        #endif
      }
      else
      {
        delay(beepP);
      }
    }
    break;
    #endif // M300

    #ifdef PIDTEMP
    case 301: // M301
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = scalePID_i(code_value());
        if(code_seen('D')) Kd = scalePID_d(code_value());

        #ifdef PID_ADD_EXTRUSION_RATE
        if(code_seen('C')) Kc = code_value();
        #endif

        updatePID();
		if(code_seen('T')) extr_type = code_value();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(Kp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(Ki));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(Kd));
        #ifdef PID_ADD_EXTRUSION_RATE
        SERIAL_PROTOCOL(" c:");
        //Kc does not have scaling applied above, or in resetting defaults
        SERIAL_PROTOCOL(Kc);
        #endif
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED
    case 304: // M304
      {
        if(code_seen('P')) bedKp = code_value();
        if(code_seen('I')) bedKi = scalePID_i(code_value());
        if(code_seen('D')) bedKd = scalePID_d(code_value());

        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(bedKp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(bedKi));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(bedKd));
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
     {
     	#ifdef CHDK
       
         SET_OUTPUT(CHDK);
         WRITE(CHDK, HIGH);
         chdkHigh = millis();
         chdkActive = true;
       
       #else
     	
      	#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
	const uint8_t NUM_PULSES=16;
	const float PULSE_LENGTH=0.01524;
	for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
        delay(7.33);
        for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
      	#endif
      #endif //chdk end if
     }
    break;
#ifdef DOGLCD
    case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
     {
	  if (code_seen('C')) {
	   lcd_setcontrast( ((int)code_value())&63 );
          }
          SERIAL_PROTOCOLPGM("lcd contrast value: ");
          SERIAL_PROTOCOL(lcd_contrast);
          SERIAL_PROTOCOLLN("");
     }
    break;
#endif
    #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: // allow cold extrudes, or set the minimum extrude temperature
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
	#endif
    case 303: // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;
#if defined(ENABLE_AUTO_BED_LEVELING) && defined(SERVO_ENDSTOPS)
    case 401:
    {
        engage_z_probe();    // Engage Z Servo endstop if available
    }
    break;

    case 402:
    {
        retract_z_probe();    // Retract Z Servo endstop if enabled
    }
    break;
#endif
    case 500: // M500 Store settings in EEPROM
    {
        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif

    #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
    case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
    {
      float value;
      if (code_seen('Z'))
      {
        value = code_value();
        if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
        {
          zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " " MSG_OK);
          SERIAL_PROTOCOLLN("");
        }
        else
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
          SERIAL_ECHOPGM(MSG_Z_MIN);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
          SERIAL_ECHOPGM(MSG_Z_MAX);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
          SERIAL_PROTOCOLLN("");
        }
      }
      else
      {
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " : ");
          SERIAL_ECHO(-zprobe_zoffset);
          SERIAL_PROTOCOLLN("");
      }
      break;
    }
    #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

    #ifdef FILAMENTCHANGEENABLE
    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
    {		// modified Dennis 08-2016
#ifdef DELTA
        
        float saved[4];
	    for (int a=0; a < NUM_AXIS; a++) {
			saved[a]=current_position[a];
			destination[a]=current_position[a];
		}
        //lift Z
        if(code_seen('Z'))
        {
          destination[Z_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_ZADD
            destination[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
          #endif
        }
	
		prepare_move();
		
        //move xy
        if(code_seen('X'))
        {
          destination[X_AXIS]= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_XPOS
            destination[X_AXIS] = FILAMENTCHANGE_XPOS ;
          #endif
        }
	
        if(code_seen('Y'))
        {
          destination[Y_AXIS] = code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_YPOS
            destination[Y_AXIS] = FILAMENTCHANGE_YPOS ;
          #endif
        }
		prepare_move();
		destination[E_AXIS]+=3;
		prepare_move();
	
	//retract by E
        if(code_seen('E'))
        {
          destination[E_AXIS] += code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_RETRACT
            destination[E_AXIS]+= FILAMENTCHANGE_RETRACT ;
          #endif
        }
	
		prepare_move();
        //finish moves
        
        //disable extruder steppers so filament can be removed
        disable_e0();
        disable_e1();
        disable_e2();
        delay(100);
        LCD_ALERTMESSAGEPGM((!Lang)?MSG_FILAMENTCHANGE:MSG_1_FILAMENTCHANGE);
        
        uint8_t cnt=0;
		start_timeout_preheat = millis();
		float tmp_temp = degHotend(active_extruder)+0.5;
        while(!lcd_clicked()){
          cnt++;
		if (!clayMode) {
		if ((millis() - start_timeout_preheat) > timeout_preheat) {
		setTargetHotend(tmp_temp-30,active_extruder);		
		}		 
        manage_heater();
		}
        
          lcd_update();
          if(cnt==0)
          {
            SET_OUTPUT(BEEPER);

            WRITE(BEEPER,HIGH);
            delay(5);
            WRITE(BEEPER,LOW);
            delay(50);
          }
        }

        //return to normal
        setTargetHotend(tmp_temp,active_extruder);
		while(isHeatingHotend(active_extruder)) {
		manage_heater();
		lcd_update();
		}
		
		
		lcd_reset_alert_level();
        extFilCgn=false;
		ext0=true;
		ext1=true;
		firstCheck=true;
		lcd_update();
        current_position[E_AXIS]=saved[E_AXIS]; 
        plan_set_e_position(current_position[E_AXIS]);
		for (int a=0; a < NUM_AXIS; a++) {
			destination[a]=saved[a];
		}
        prepare_move();
#endif
    }
    break;
    #endif //FILAMENTCHANGEENABLE
    
    #ifdef DUAL_X_CARRIAGE
    case 605: // Set dual x-carriage movement mode:
              //    M605 S0: Full control mode. The slicer has full control over x-carriage movement
              //    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
              //    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
              //                         millimeters x-offset and an optional differential hotend temperature of
              //                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
              //                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
              //
              //    Note: the X axis should be homed after changing dual x-carriage mode.
    {
        st_synchronize();

        if (code_seen('S'))
          dual_x_carriage_mode = code_value();

        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
        {
          if (code_seen('X'))
            duplicate_extruder_x_offset = max(code_value(),X2_MIN_POS - x_home_pos(0));

          if (code_seen('R'))
            duplicate_extruder_temp_offset = code_value();

          SERIAL_ECHO_START;
          SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
          SERIAL_ECHO(" ");
          SERIAL_ECHO(extruder_offset[X_AXIS][0]);
          SERIAL_ECHO(",");
          SERIAL_ECHO(extruder_offset[Y_AXIS][0]);
          SERIAL_ECHO(" ");
          SERIAL_ECHO(duplicate_extruder_x_offset);
          SERIAL_ECHO(",");
          SERIAL_ECHOLN(extruder_offset[Y_AXIS][1]);
        }
        else if (dual_x_carriage_mode != DXC_FULL_CONTROL_MODE && dual_x_carriage_mode != DXC_AUTO_PARK_MODE)
        {
          dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        }

        active_extruder_parked = false;
        extruder_duplication_enabled = false;
        delayed_move_time = 0;
    }
    break;
    #endif //DUAL_X_CARRIAGE

    case 907: // M907 Set digital trimpot motor current using axis codes.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
        if(code_seen('B')) digipot_current(4,code_value());
        if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_XY_PIN
        if(code_seen('X')) digipot_current(0, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_Z_PIN
        if(code_seen('Z')) digipot_current(1, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_E_PIN
        if(code_seen('E')) digipot_current(2, code_value());
      #endif
      #ifdef DIGIPOT_I2C
        // this one uses actual amps in floating point
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
        // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
        for(int i=NUM_AXIS;i<DIGIPOT_I2C_NUM_CHANNELS;i++) if(code_seen('B'+i-NUM_AXIS)) digipot_i2c_set_current(i, code_value());
      #endif
    }
    break;
    case 908: // M908 Control digital trimpot directly.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
      #endif
    }
    break;
    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
      #endif
    }
    break;
    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
      if(code_seen('S')) switch((int)code_value())
      {
        case 1:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
      }
      microstep_readings();
      #endif
    }
    break;
    case 999: // M999: Restart after being stopped
      Stopped = false;
      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
    }
  }
  /*

  T0 T1 switch extruder


*/
  else if(code_seen('T') && DualExtMode)
  {
	  float fd = 0;
	  uint8_t xtr = code_value();
	  if(code_seen('F')) 
        fd = code_value();
		
	gcode_change_extruder(xtr,fd);
	
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }
	
  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  if (wifi_off) {
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  } else {
	WIFI.flush();
  WIFI_PROTOCOLPGM(MSG_RESEND);
  WIFI_PROTOCOLLN(gcode_LastN + 1);  
  }
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
  (wifi_off)?SERIAL_PROTOCOLLNPGM(MSG_OK):WIFI_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
   // if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
   // if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
  //  if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
  //  if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

#ifdef DELTA
void recalc_delta_settings(float radius, float diagonal_rod)
{
	 delta_tower1_x= -SIN_60*(radius+dA); // front left tower
	 delta_tower1_y= -COS_60*(radius+dA);	   
	 delta_tower2_x=  SIN_60*(radius+dB); // front right tower
	 delta_tower2_y= -COS_60*(radius+dB);	   
	 delta_tower3_x= 0.0+dC;                  // back middle tower
	 delta_tower3_y= radius+dC;
	 delta_diagonal_rod_2X= sq(diagonal_rod+dX);
	 delta_diagonal_rod_2Y= sq(diagonal_rod+dY);
	 delta_diagonal_rod_2Z= sq(diagonal_rod+dZ);
	 
}

void calculate_delta(float cartesian[3])
{
	if ((max_plater_radius<sqrt(sq(cartesian[X_AXIS])+sq(cartesian[Y_AXIS]))) && !overrideBed)
		out_of_plate();
	else {
  delta[X_AXIS] = sqrt(delta_diagonal_rod_2X
                       - sq(delta_tower1_x-cartesian[X_AXIS])
                       - sq(delta_tower1_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Y_AXIS] = sqrt(delta_diagonal_rod_2Y
                       - sq(delta_tower2_x-cartesian[X_AXIS])
                       - sq(delta_tower2_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Z_AXIS] = sqrt(delta_diagonal_rod_2Z
                       - sq(delta_tower3_x-cartesian[X_AXIS])
                       - sq(delta_tower3_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  /*
  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  */
  }
}
#endif

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis();
#ifdef DELTA
  float difference[NUM_AXIS];
  for (int8_t i=0; i < NUM_AXIS; i++) {
    difference[i] = destination[i] - current_position[i];
  }
  float cartesian_mm = sqrt(sq(difference[X_AXIS]) +
                            sq(difference[Y_AXIS]) +
                            sq(difference[Z_AXIS]));
  if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
  if (cartesian_mm < 0.000001) { return; }
  float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
  int steps = max(1, int(delta_segments_per_second * seconds));
  // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
  // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
  // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);
  for (int s = 1; s <= steps; s++) {
    float fraction = float(s) / float(steps);
    for(int8_t i=0; i < NUM_AXIS; i++) {
      destination[i] = current_position[i] + difference[i] * fraction;
    }
    calculate_delta(destination);
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
                     destination[E_AXIS], feedrate*feedmultiply/60/100.0,
                     active_extruder);
  }
#else

#ifdef DUAL_X_CARRIAGE
  if (active_extruder_parked)
  {
    if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0)
    {
      // move duplicate extruder into correct duplication position.
      plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset, current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS], max_feedrate[X_AXIS], 1);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      st_synchronize();
      extruder_duplication_enabled = true;
      active_extruder_parked = false;
    }
    else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) // handle unparking of head
    {
      if (current_position[E_AXIS] == destination[E_AXIS])
      {
        // this is a travel move - skit it but keep track of current position (so that it can later
        // be used as start of first non-travel move)
        if (delayed_move_time != 0xFFFFFFFFUL)
        {
          memcpy(current_position, destination, sizeof(current_position));
          if (destination[Z_AXIS] > raised_parked_position[Z_AXIS])
            raised_parked_position[Z_AXIS] = destination[Z_AXIS];
          delayed_move_time = millis();
          return;
        }
      }
      delayed_move_time = 0;
      // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
      plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS],    current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS],
          current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]), active_extruder);
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
      active_extruder_parked = false;
    }
  }
#endif //DUAL_X_CARRIAGE

  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
#endif //else DELTA
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN) || (soft_pwm_bed > 0)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
      #if defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
       || !READ(X2_ENABLE_PIN)
      #endif
       || !READ(E1_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }

    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
        digitalWrite(CONTROLLERFAN_PIN, 0);
        analogWrite(CONTROLLERFAN_PIN, 0);
    }
    else
    {
        // allows digital or PWM fan output to be used (see M42 handling)
        digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
        analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
    }
  }
}
#endif

#ifdef TEMP_STAT_LEDS
static bool blue_led = false;
static bool red_led = false;
static uint32_t stat_update = 0;

void handle_status_leds(void) {
  float max_temp = 0.0;
  if(millis() > stat_update) {
    stat_update += 500; // Update every 0.5s
    for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
       max_temp = max(max_temp, degHotend(cur_extruder));
       max_temp = max(max_temp, degTargetHotend(cur_extruder));
    }
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
      max_temp = max(max_temp, degTargetBed());
      max_temp = max(max_temp, degBed());
    #endif
    if((max_temp > 55.0) && (red_led == false)) {
      digitalWrite(STAT_LED_RED, 1);
      digitalWrite(STAT_LED_BLUE, 0);
      red_led = true;
      blue_led = false;
    }
    if((max_temp < 54.0) && (blue_led == false)) {
      digitalWrite(STAT_LED_RED, 0);
      digitalWrite(STAT_LED_BLUE, 1);
      red_led = false;
      blue_led = true;
    }
  }
}
#endif

void manage_inactivity()
{

if (outOfPlate) {
SERIAL_ERROR_START;
SERIAL_ERRORLNPGM(MSG_OUTOFPLATE);
LCD_ALERTMESSAGEPGM((!Lang)?MSG_OUTOFPLATE:MSG_1_OUTOFPLATE);
outOfPlate=false;

}
if ((READ(EMERGENCY_STOP)) && (resurrectionData.resurrActive) && resurrectionData.autoResurr and (!resurState)) resurrecting();
if ((!clayMode) && start_timeout_preheat > 0 ) {
if (((millis() - start_timeout_preheat) > timeout_preheat) && !card.sdprinting ) {
		setTargetHotend(EXTRUDE_MINTEMP,active_extruder);
        LCD_MESSAGEPGM("Timeout Heater");
        SERIAL_ECHO_START;
        SERIAL_ECHOLN("Timeout Heater");	
		start_timeout_preheat = 0;		
}
if (card.sdprinting) start_timeout_preheat = 0;
}
  if(buflen < (BUFSIZE-1))
    get_command();

  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        //disable_x();
        //disable_y();
        //disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }
  
  #ifdef CHDK //Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive)
    {
      chdkActive = false;
      if (millis()-chdkHigh < CHDK_DELAY) return;
      WRITE(CHDK, LOW);
    }
  #endif
  
  #if defined(KILL_PIN) && KILL_PIN > -1
    if( 0 == READ(KILL_PIN) )
      kill();
  #endif
 
 
#if defined(EMERGENCY_STOP) && EMERGENCY_STOP > -1
	if (0 == READ(EMERGENCY_STOP)) {
		
		DISABLE_STEPPER_DRIVER_INTERRUPT();
		disable_pwm();	
		WRITE(STEPPER_RESET,LOW);
		WRITE(HEATER_BED_PIN,LOW);
		WRITE(HEATER_0_PIN,LOW);
		WRITE(HEATER_1_PIN,LOW);
		SET_INPUT(X_STEP_PIN);
		SET_INPUT(X_DIR_PIN);
		SET_INPUT(X_ENABLE_PIN);
		SET_INPUT(Y_STEP_PIN);
		SET_INPUT(Y_DIR_PIN);
		SET_INPUT(Y_ENABLE_PIN);
		SET_INPUT(Z_STEP_PIN);
		SET_INPUT(Z_DIR_PIN);
		SET_INPUT(Z_ENABLE_PIN);
		SET_INPUT(E0_STEP_PIN);
		SET_INPUT(E0_DIR_PIN);
		SET_INPUT(E0_ENABLE_PIN);
		SET_INPUT(E1_STEP_PIN);
		SET_INPUT(E1_DIR_PIN);
		SET_INPUT(E1_ENABLE_PIN);
		SET_INPUT(LCD_PINS_ENABLE);
		SET_INPUT(LCD_PINS_D4);
		SET_INPUT(LCD_PINS_D5);
		SET_INPUT(LCD_PINS_D6);
		SET_INPUT(LCD_PINS_D7);
		SET_INPUT(LCD_PINS_RS);
	}
    if( (0 == READ(EMERGENCY_STOP)) and (!resurState)) {
		
		Stopped=true;
		
    	if ((!Estopped) and (card.sdprinting) and (resurrAtZ == 0.0) ) {
			resurrectionData.high = current_position[Z_AXIS];
			resurrectionData.X = current_position[X_AXIS];
			resurrectionData.Y = current_position[Y_AXIS];
			resurrectionData.E = current_position[E_AXIS];
			resurrectionData.feedmultiply = feedmultiply;
			resurrectionData.feedrate = feedrate;
			resurrectionData.extruder = degTargetHotend(0);
			if (DualExtMode)
			resurrectionData.extruder2 = degTargetHotend(1);
			//resurrectionData.bed = degTargetBed();
			resurrectionData.sdpos = card.sdPosition();
			resurrectionData.ext_selected=active_extruder;
			resurrectionData.resurrActive=true;
			disable_heater();
			HAL_timer_disable_interrupt (TEMP_TIMER_NUM);
			disable_e0();
			disable_e1();
			disable_e2();
			Store_ResurData();
			
			card.closefile();
			Estopped=true;
			card.sdprinting = false;
			
			quickStop();
			if (!clayMode) autotempShutdown();
			
			disable_x();
			disable_y();
			disable_z();
			disable_e0();
			disable_e1();
			disable_e2();
			#if defined(CURR_CONTROL_DRIVER) && CURR_CONTROL_DRIVER > -1
			disable_pwm();		
			#endif
      			 
		
    	}	
		
		
		
}     
#endif

Xfault=READ(X_FAULT);
Yfault=READ(Y_FAULT);
Zfault=READ(Z_FAULT);

if ((!Xfault || !Yfault || !Zfault) && initfault) {
if (!Xfault) LCD_ALERTMESSAGEPGM("FAULT DRV X");	
if (!Yfault) LCD_ALERTMESSAGEPGM("FAULT DRV Y");	
if (!Zfault) LCD_ALERTMESSAGEPGM("FAULT DRV Z");		
initfault=false;	
	
}



#if defined(EXT_FILAMENT) && EXT_FILAMENT > -1
if (filSensor && !clayMode) {
bool state_ext_fil;
if (DualExtMode && DualSensorFilament) {
	
	
	
	
	
	
	if (active_extruder==0) {
	if (ext0)	state_ext_fil = READ(EXT_FILAMENT);	
	} else {
	if (ext1)	state_ext_fil = READ(EXT_FILAMENT2);	
	}
	
	
	
} else  {
	state_ext_fil = READ(EXT_FILAMENT);
	continuousPrint=false;
	}
if ((!state_ext_fil) && (prev_ext_fil) ) previousMillis = millis();
if (!state_ext_fil)  { 
	if (millis() - previousMillis > interval) {		
		prev_ext_fil = true;
		
		if ((!extFilCgn) and (card.sdprinting) ) {		
      			
			
			if (firstCheck && continuousPrint) {
			(!active_extruder)?ext0=false:ext1=false;
			SERIAL_ECHOPGM("Continuous print!");	
			enquecommand(itoextr((int)!active_extruder));
			firstCheck=false;
			
			} else {
			extFilCgn=true;
			SERIAL_ECHOPGM("EXT_FILAMENT Triggered!");
    		enquecommand_P(PSTR("M600")); 
			
			}    		

			}
	}
  }   
  prev_ext_fil = state_ext_fil;
  }
#endif		
 #if defined(EXT_PAUSE) && EXT_PAUSE > -1
	
  if (card.sdprinting && !doorExcluded) {
	
	while(READ(EXT_PAUSE)==1) {
		if (first) {
			(!Lang)?lcd_setstatus("Porta aperta"):lcd_setstatus("Door is open");
			first=false;
			clearStatus=true;
		}
		manage_heater();
		lcd_update();
		}  
	  	if (clearStatus) {
			lcd_setstatus("                    ");
			clearStatus=false;
		}
  }
#endif  

#if defined(CHECK_EXTR) && CHECK_EXTR > -1
if (DualExtMode and extruderCheck){
		if	(!dontcheck) {
showMsg=true;
clearMsg=false;

	if ((active_extruder==0) && (READ(CHECK_EXTR)==0)) {
			
		
	 
	while	(READ(CHECK_EXTR)==0) {
	if (showMsg){
		lcd_setstatus("!!!Extruder 2!!!");
		showMsg=false;
		clearMsg=true;
		if (firstCommand) {
		active_extruder=1;
		if (int(degHotend(1) + 0.5)<PREHEAT_TEMP) {
			lcd_setstatus("Preheating...");
			setTargetHotend(PREHEAT_TEMP,1);
			while(isHeatingHotend(1)) {
				manage_heater();
				lcd_update();
			}	
		}
		
		
		gcode_change_extruder(0,0);
		st_synchronize();
		firstCommand=false;
		}
	}
	manage_heater();
	lcd_update();
	}
		
	
		
		
		
		
	} else if ((active_extruder==1) && (READ(CHECK_EXTR)==1)){
		
	while	(READ(CHECK_EXTR)==1) {
		if (showMsg){
		lcd_setstatus("!!!Extruder 1!!!");
		showMsg=false;
		clearMsg=true;
	}
	manage_heater();
	lcd_update();
			}
		
		
		
		
		
	}	else 
		clearMsg=false;
	
	
	if (clearMsg){
		lcd_setstatus("                  ");
		clearMsg=false;
	}
	
}
}
#endif  
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
    if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
     bool oldstatus=READ(E0_ENABLE_PIN);
     enable_e0();
     float oldepos=current_position[E_AXIS];
     float oldedes=destination[E_AXIS];
     plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                      destination[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
     current_position[E_AXIS]=oldepos;
     destination[E_AXIS]=oldedes;
     plan_set_e_position(oldepos);
     previous_millis_cmd=millis();
     st_synchronize();
     WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  #if defined(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time != 0 && (millis() - delayed_move_time) > 1000 && Stopped == false)
    {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      memcpy(destination,current_position,sizeof(destination));
      prepare_move();
    }
  #endif
  #ifdef TEMP_STAT_LEDS
      handle_status_leds();
  #endif
  check_axes_activity();

}
void gcode_change_extruder(uint8_t exttochange, float feedratechange) {
	if (!resurState) dontcheck=true;  
	
	bool saved_overrideBed=overrideBed;
    overrideBed=true;
	tmp_extruder = exttochange;
	int savedFeedrate=feedrate;
	int savedFeedmultiply=feedmultiply;
	feedmultiply=100;
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO("T");
      SERIAL_ECHO(tmp_extruder);
      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
    }
    else {
      if(feedratechange>0) {
        next_feedrate = feedratechange;
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      } else {
		  
		next_feedrate=9000;
		feedrate=next_feedrate;  
		  
	  }
      if(Stopped == false) {
		  float savedPosition[NUM_AXIS];
		  memcpy(savedPosition, current_position, sizeof(savedPosition));
		  memcpy(destination, current_position, sizeof(destination));
		  if(tmp_extruder != active_extruder)
	  {
		if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] ) )
            {
                make_homing(false);
            }
		
		bool nochange=false;
		bool isHome=false;
		float safe_z = MANUAL_Z_HOME_POS+ZcorNoClay+SAFE_Z;
		if ((current_position[Z_AXIS]>safe_z) && card.sdprinting && donTchange)  {
				nochange=true;
		} else if (current_position[Z_AXIS]>=(MANUAL_Z_HOME_POS+ZcorNoClay-5))
			{	
		//destination[Z_AXIS]=MANUAL_Z_HOME_POS+ZcorNoClay-(DELTA_DIAGONAL_ROD/3);
				destination[Z_AXIS]=safe_z;
				savedPosition[Z_AXIS]=(MANUAL_Z_HOME_POS+ZcorNoClay);
				prepare_move();
				st_synchronize();
				isHome=true;
			}
		
		
		
		
		
		if (!nochange) {
		
		destination[Z_AXIS]+=1; //Z hop
		prepare_move();
		st_synchronize();
		
		destination[X_AXIS]=0;
		destination[Y_AXIS]=changeT[1][1];
        
        prepare_move();
		st_synchronize();
      
		for (uint8_t a=0;a<2;a++)
			destination[a]=changeT[a][tmp_extruder];//move for change extruder
		
		prepare_move();
		st_synchronize();
		}
		if (isHome) {
			destination[X_AXIS]=0;
			destination[Y_AXIS]=0;
			prepare_move();
			st_synchronize();
			make_homing(false);
			
			
		} else {
		
		
			memcpy(destination, savedPosition, sizeof(destination));
			prepare_move();
			st_synchronize();
		}
	  }
       
		
        
		
        // Set the new active extruder and position
    active_extruder = tmp_extruder;
		
		
		
     

  
   
  
    }    
      
	  
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
      SERIAL_PROTOCOLLN((int)active_extruder);
    }
	feedmultiply=savedFeedmultiply;
	next_feedrate=savedFeedrate;
	feedrate=next_feedrate;
	overrideBed=saved_overrideBed;
	if (!resurState) dontcheck=false;
}
void make_homing(bool changeExtr) {
	
	if (Stopped==true) Stopped=false;
#ifdef ENABLE_AUTO_BED_LEVELING
      plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
#endif //ENABLE_AUTO_BED_LEVELING


      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;


          

          // Move all carriages up together until the first endstop is hit.
          current_position[X_AXIS] = 0;
          current_position[Y_AXIS] = 0;
          current_position[Z_AXIS] = 0;
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

          destination[X_AXIS] = 3 * Z_MAX_LENGTH;
          destination[Y_AXIS] = 3 * Z_MAX_LENGTH;
          destination[Z_AXIS] = 3 * Z_MAX_LENGTH;
          feedrate = 1.732 * homing_feedrate[X_AXIS];
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          endstops_hit_on_purpose();

          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];
          current_position[Z_AXIS] = destination[Z_AXIS];

          // take care of back off and rehome now we are all at the top
          HOMEAXIS(X);
          HOMEAXIS(Y);
          HOMEAXIS(Z);

          calculate_delta(current_position);
          plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);



      

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
	  if (DualExtMode && changeExtr) {
		  donTchange=false;  
			int tmpTemp1=degTargetHotend(0);
			int tmpTemp2=degTargetHotend(1);
			if ((int(tmpTemp1 + 0.5)<PREHEAT_TEMP )||(int(tmpTemp2 + 0.5)<PREHEAT_TEMP )) {
				lcd_setstatus("...Please wait...   ");
	
				if (int(degHotend(0) + 0.5)<PREHEAT_TEMP ) setTargetHotend(PREHEAT_TEMP,0);
				if (int(degHotend(1) + 0.5)<PREHEAT_TEMP ) setTargetHotend(PREHEAT_TEMP,1);
				while(isHeatingHotend(0)) {
					manage_heater();
					lcd_update();
				}
				while(isHeatingHotend(1)) {
					manage_heater();
					lcd_update();
				}
				lcd_setstatus("                    ");
				
			}	 
		  
		  
		  
		  active_extruder=1;
		  gcode_change_extruder(0,0); 
		  st_synchronize();
		setTargetHotend(tmpTemp1+0.5,0);
		setTargetHotend(tmpTemp2+0.5,1); 
		  
		  
		  
		  
		  
		  
		donTchange=true;  
	  }
}
void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  SET_INPUT(PS_ON_PIN);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM((!Lang)?MSG_KILLED:MSG_1_KILLED);
  suicide();
  DISABLE_STEPPER_DRIVER_INTERRUPT();
		disable_pwm();	
		WRITE(STEPPER_RESET,LOW);
		WRITE(HEATER_BED_PIN,LOW);
		SET_INPUT(X_STEP_PIN);
		SET_INPUT(X_DIR_PIN);
		SET_INPUT(X_ENABLE_PIN);
		SET_INPUT(Y_STEP_PIN);
		SET_INPUT(Y_DIR_PIN);
		SET_INPUT(Y_ENABLE_PIN);
		SET_INPUT(Z_STEP_PIN);
		SET_INPUT(Z_DIR_PIN);
		SET_INPUT(Z_ENABLE_PIN);
		SET_INPUT(E0_STEP_PIN);
		SET_INPUT(E0_DIR_PIN);
		SET_INPUT(E0_ENABLE_PIN);
		SET_INPUT(LCD_PINS_ENABLE);
		SET_INPUT(LCD_PINS_D4);
		SET_INPUT(LCD_PINS_D5);
		SET_INPUT(LCD_PINS_D6);
		SET_INPUT(LCD_PINS_D7);
		SET_INPUT(LCD_PINS_RS);
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void out_of_plate() {
	outOfPlate=true;
	if (card.sdprinting){
	card.sdprinting = false;
    	card.closefile();
		}
		Stopped = true;
    	quickStop();
	if (!clayMode) {
		setTargetHotend0(0);
		setTargetHotend1(0);
		setTargetHotend2(0);
	}
	
	

}

void Stop()
{
  if (!clayMode) disable_heater();
  if(Stopped == false) {
    Stopped = true;
	firstMove = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM((!Lang)?MSG_STOPPED:MSG_1_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN
void resurrecting() {
	dontcheck=true;
	resurState=true;
	card.initsd();
	if (!card.cardOK) {
		lcd_setstatus("No SDCARD");
		return;
	}
	lcd_setstatus(" Resurrecting...");
	
	Estopped=false;
	firstMove=true;
	if (!clayMode) {
	setTargetBed((int)resurrectionData_bed);
	
	if (int(degHotend(resurrectionData.ext_selected) + 0.5)<PREHEAT_TEMP ) {
		setTargetHotend(PREHEAT_TEMP,resurrectionData.ext_selected);
		while(isHeatingHotend(resurrectionData.ext_selected)) {
			manage_heater();
			lcd_update();
		}	
	}
	}
	make_homing(false);	
	

	delay(1000);
	
	
	if (!clayMode) {
	if (int(degHotend(0) + 0.5)<(int)resurrectionData.extruder ) {
		setTargetHotend((int)resurrectionData.extruder,0);
	if (DualExtMode)	setTargetHotend((int)resurrectionData.extruder2,1);
		while(isHeatingHotend(resurrectionData.ext_selected)) {
			manage_heater();
			lcd_update();
		}	
	}
	}
	
	
	
	current_position[E_AXIS] = resurrectionData.E-5.0;
	plan_set_e_position(current_position[E_AXIS]);
	destination[X_AXIS] = resurrectionData.X;
	destination[Y_AXIS] = resurrectionData.Y;
	destination[Z_AXIS] = resurrectionData.high;
	destination[E_AXIS] = resurrectionData.E;
	feedrate = resurrectionData.feedrate;   
	feedmultiply = resurrectionData.feedmultiply;	 // 100% feedrate
	next_feedrate=resurrectionData.feedrate;
	prepare_move();
	st_synchronize();
	if (DualExtMode) {
		
		//enquecommand_P(itoextr(resurrectionData.ext_selected));
		gcode_change_extruder(resurrectionData.ext_selected,0);
		
	}
	
	fanSpeed=constrain(resurrectionData.fan,defaultfan,MAXFAN);
	if (wifi_off) {
		SERIAL_ECHO_START;
		SERIAL_ECHO("Open dir ");
		SERIAL_ECHOLN(nameDir);
	} else{
		WIFI_PROTOCOL("echo: Open dir ");
		WIFI_PROTOCOLLN(nameDir);
	}
	if (strlen(nameDir) >0) {
		if (!card.chdir(nameDir)) {
			lcd_setstatus("Resurrrection failed.");
			return;
		
		}
	}
	if (wifi_off) {
		SERIAL_ECHO_START;
		SERIAL_ECHO("Printing file ");
		SERIAL_ECHOLN(nameFile);
	} else{
		WIFI_PROTOCOL("echo: Printing file ");
		WIFI_PROTOCOLLN(nameFile);
	}
	card.openFile(nameFile,true,false);
	card.setIndex(resurrectionData.sdpos);
	
	card.startFileprint(); 
	resurState=false;
	Stopped=false;
	dontcheck=false;
	starttime=millis(); //procedure calls count as normal print time.
		lcd_setstatus(" Printing      ");

}
bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code){
        case 104:
          SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
        case 221:
          SERIAL_ECHO(MSG_M221_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}

// by Rossonet
/**
 * Propone un setting ottimale per il diametro della delta
 * e il setting dei tre end stop
 *
 * il risultato finale sarà stampanto, ma non salvato in flash
 * Modificato da Dennis Patella dennis@wasproject.it
 * Inserito preriscaldamento, modificato test altezza e calcolo offset z doppio estrusore
 */

void gcode_G667() {
	lcd_setstatus("...Please wait...   ");
	if (int(degHotend(0) + 0.5)<220 ) setTargetHotend(220,0);
	if (DualExtMode) {
		extruder_offset_Z[1]=0;
		extruder_offset[Z_AXIS][1]=0;
	if (int(degHotend(1) + 0.5)<220 ) setTargetHotend(220,1);
		while(isHeatingHotend(1)) {
				manage_heater();
				lcd_update();
			}
	}
	while(isHeatingHotend(0)) {
				manage_heater();
				lcd_update();
			}
	
	lcd_setstatus("                    ");
	make_homing(true);
	destination[E_AXIS]= -10;
	prepare_move();
	st_synchronize();
	// Aggiunta ricalcolo Z correction per altezza massima HOME
	gcode_G29(false);
	
	
    feedrate = homing_feedrate[Z_AXIS] / 6;

	float centroX = 0;
	float centroY = 0;
	float torreXX = -86.602540;
	float torreXY = -50;
	float torreYX = 86.602540;
	float torreYY = -50;
	float torreZX = 0;
	float torreZY = 100;
	
	float baseRilevamentiZ = 30.0;

	int cicliTorri = 3;
	int cicliRaggio = 2;

	int scartoAccettato = 0.1;

	float passoRaggio = 0.9;

	float rilevamentoCentro = 0;
	float misuraZ = 0;
	float rilevamentoX = 0;
	float rilevamentoY = 0;
	float rilevamentoZ = 0;
	float correctionZ[2];
	lcd_setstatus("Auto calibration");
	SERIAL_ECHOLNPGM("Configurazioni automatiche fine corsa e diametro");
	SERIAL_ECHOLNPGM("by Rossonet");
	SERIAL_ECHOLNPGM("ATTENZIONE! Versione beta, monitorare la macchina durante l'esecuzione...");
	CONFIG_ECHO_START; 
	SERIAL_ECHOLNPGM("Attuale situazione endstops (mm):"); 
	CONFIG_ECHO_START; 
	SERIAL_ECHOPAIR(" X", endstop_adj[X_AXIS]); 
	SERIAL_ECHOPAIR(" Y", endstop_adj[Y_AXIS]); 
	SERIAL_ECHOPAIR(" Z", endstop_adj[Z_AXIS]); 
	SERIAL_EOL; 
	
	CONFIG_ECHO_START; 
	SERIAL_ECHOPAIR("Lunghezza braccia (mm):", delta_diagonal_rod); 
	SERIAL_ECHOPAIR(" Diametro stampante (mm):", delta_radius); 
	SERIAL_EOL; 

	SERIAL_ECHOLNPGM("Inizio i rilevamenti..."); 
	float correzioneX = endstop_adj[X_AXIS];
	float correzioneY = endstop_adj[Y_AXIS];
	float correzioneZ = endstop_adj[Z_AXIS];
	correctionZ[0]=ZcorNoClay;
	
	
	for (int contatoreCicli = 1; contatoreCicli <= (cicliTorri+cicliRaggio); contatoreCicli++){
		make_homing(false);
		st_synchronize();
		if (contatoreCicli>cicliTorri) {
			//rilevamentoCentro = probe_pt(centroX, centroY, baseRilevamentiZ, ProbeDeploy, 1);
			//engage_z_probe();
			rilevamentoCentro = probe_pt(centroX, centroY, baseRilevamentiZ);
		}
		//if (contatoreCicli>cicliTorri) rilevamentoX = probe_pt(torreXX, torreXY, baseRilevamentiZ, ProbeStay, 1);
		if (contatoreCicli>cicliTorri) rilevamentoX = probe_pt(torreXX, torreXY, baseRilevamentiZ);
		if (!(contatoreCicli>cicliTorri)) {
			//rilevamentoX = probe_pt(torreXX, torreXY, baseRilevamentiZ, ProbeDeploy, 1);
			//;
			rilevamentoX = probe_pt(torreXX, torreXY, baseRilevamentiZ);
		}
		//rilevamentoY = probe_pt(torreYX, torreYY, baseRilevamentiZ, ProbeStay, 1);
		rilevamentoY = probe_pt(torreYX, torreYY, baseRilevamentiZ);
		//rilevamentoZ = probe_pt(torreZX, torreZY, baseRilevamentiZ, ProbeStow, 1);
		rilevamentoZ = probe_pt(torreZX, torreZY, baseRilevamentiZ);
		//retract_z_probe();
		SERIAL_ECHOLNPGM("Rilevamento (mm):"); 
		CONFIG_ECHO_START; 
		if (contatoreCicli>cicliTorri) SERIAL_ECHOPAIR("Centro:", rilevamentoCentro); 
		SERIAL_ECHOPAIR(" X:", rilevamentoX); 
		SERIAL_ECHOPAIR(" Y:", rilevamentoY); 
		SERIAL_ECHOPAIR(" Z:", rilevamentoZ); 
		SERIAL_EOL; 

		// aggiusta anche il diametro
		if (contatoreCicli>cicliTorri) {
			if (rilevamentoCentro>rilevamentoX && rilevamentoCentro>rilevamentoY && rilevamentoCentro>rilevamentoZ){
				delta_radius=delta_radius-(passoRaggio*(rilevamentoCentro-((rilevamentoX+rilevamentoY+rilevamentoZ)/3))); 
				recalc_delta_settings(delta_radius, delta_diagonal_rod);
				SERIAL_ECHOLNPGM("Correzione diametro per centro alto:");
				CONFIG_ECHO_START;
				SERIAL_ECHOPAIR(" M665 R", delta_radius);
				SERIAL_ECHOPAIR(" L", delta_diagonal_rod);
				SERIAL_ECHOPAIR(" S", delta_segments_per_second);
				SERIAL_EOL;	
			}
			if (rilevamentoCentro<rilevamentoX && rilevamentoCentro<rilevamentoY && rilevamentoCentro<rilevamentoZ){
				delta_radius=delta_radius+(passoRaggio*(((rilevamentoX+rilevamentoY+rilevamentoZ)/3)-rilevamentoCentro)); 
				recalc_delta_settings(delta_radius, delta_diagonal_rod);
				SERIAL_ECHOLNPGM("Correzione diametro per centro basso:");
				CONFIG_ECHO_START;
				SERIAL_ECHOPAIR(" M665 R", delta_radius);
				SERIAL_ECHOPAIR(" L", delta_diagonal_rod);
				SERIAL_ECHOPAIR(" S", delta_segments_per_second);
				SERIAL_EOL;	
			}
		}

		//aggiustamenti
		float deltaX = 0;
		float deltaY = 0;
		float deltaZ = 0;

		//X è il più grande
		if (rilevamentoX>rilevamentoY && rilevamentoX>rilevamentoZ ){
			//Z è il più piccolo
			if(rilevamentoY>rilevamentoZ){
				deltaY = 0;
				deltaX=rilevamentoY-rilevamentoX;
				deltaZ=rilevamentoY-rilevamentoZ;
			//Y è il più piccolo
			} else {
				deltaZ = 0;
				deltaX=rilevamentoZ-rilevamentoX;
				deltaY=rilevamentoZ-rilevamentoY;
			}

		}
		//Y è il più grande
		if (rilevamentoY>rilevamentoX && rilevamentoY>rilevamentoZ ){
			//Z è il più piccolo
                        if(rilevamentoX>rilevamentoZ){
				deltaX = 0;
                                deltaY=rilevamentoX-rilevamentoY;
                                deltaZ=rilevamentoX-rilevamentoZ;
			//X è il più piccolo
                        } else {
				deltaZ = 0;
                                deltaY=rilevamentoZ-rilevamentoY;
                                deltaX=rilevamentoZ-rilevamentoX;
                        }

		}
		//Z è il più grande
		if (rilevamentoZ>rilevamentoX && rilevamentoZ>rilevamentoY ){
			//X è il più piccolo
                        if(rilevamentoY>rilevamentoX){
				deltaY = 0;
                                deltaZ=rilevamentoY-rilevamentoZ;
                                deltaX=rilevamentoY-rilevamentoX;
			//Y è il più piccolo
                        } else {
				deltaX = 0;
                                deltaZ=rilevamentoX-rilevamentoZ;
                                deltaY=rilevamentoX-rilevamentoY;
                        }
		}

		correzioneX=correzioneX-deltaX;
		correzioneY=correzioneY-deltaY;
		correzioneZ=correzioneZ-deltaZ;

		//elimina i valori positivi abbassando
		if (correzioneX>0){
			correzioneX=correzioneX-correzioneX;
			correzioneY=correzioneY-correzioneX;
			correzioneZ=correzioneZ-correzioneX;
		}
		if (correzioneY>0){
			correzioneX=correzioneX-correzioneY;
			correzioneY=correzioneY-correzioneY;
			correzioneZ=correzioneZ-correzioneY;
		}
		if (correzioneZ>0){
			correzioneX=correzioneX-correzioneZ;
			correzioneY=correzioneY-correzioneZ;
			correzioneZ=correzioneZ-correzioneZ;
		}
        	
		endstop_adj[X_AXIS]=correzioneX;
	    endstop_adj[Y_AXIS]=correzioneY;
		endstop_adj[Z_AXIS]=correzioneZ;

		SERIAL_ECHOLNPGM("Configurazione applicata dopo questo ciclo:");
		CONFIG_ECHO_START;
		SERIAL_ECHOPAIR("  M666 X", correzioneX);
		SERIAL_ECHOPAIR(" Y", correzioneY);
		SERIAL_ECHOPAIR(" Z", correzioneZ);
		SERIAL_EOL;	
		
	}
	SERIAL_ECHOLNPGM("Fine rilevamenti G667 (by Rossonet)");
	
	lcd_setstatus("     END Calib      ");
	if (DualExtMode) {
		make_homing(false);
		gcode_change_extruder(1,0);
		gcode_G29(false);
		correctionZ[1]=ZcorNoClay;
		ZcorNoClay=correctionZ[0];
		extruder_offset[Z_AXIS][1] = (correctionZ[0]*-1) - (correctionZ[1]*-1);
		extruder_offset_Z[1]=extruder_offset[Z_AXIS][1];
		gcode_change_extruder(0,0);
		lcd_setstatus("     END Offset    ");
	}
	ZcorNoClay-=scartoAccettato;
	add_homeing[2]=ZcorNoClay;
	make_homing(false);
	destination[E_AXIS] = 0;
	prepare_move();
	st_synchronize();
	Config_StoreSettings(); //Salvataggio nuovi valori
}
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
/**
* Calculate delta, start a line, and set current_position to destination
*/
void prepare_move_raw() {
	refresh_cmd_timeout();
	calculate_delta(destination);
	plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], (feedrate/60), active_extruder);
	set_current_to_destination();
}
void gcode_G29(bool saving) {
	
	
	if (!clayMode) {
	SERIAL_ECHOLNPGM("Start autoset Zmax"); 
	float misuraZ=0;
	float testZ[3];
	lcd_setstatus("Zmax autoset");
	ZcorNoClay=0;
	add_homeing[2]=ZcorNoClay;
	
	for (uint8_t t=0; t<3; t++) {
		make_homing(false);
		st_synchronize();
		testZ[t] = (probe_pt(0, 0, 30) + 0.15) * -1;
		}
	if (testZ[0]<testZ[1])
		if (testZ[0]<testZ[2])
			ZcorNoClay = testZ[0];
		else
			ZcorNoClay = testZ[2];	
	else
		if (testZ[1]<testZ[2])
			ZcorNoClay = testZ[1];
		else
			ZcorNoClay = testZ[2];
		
	add_homeing[2]=ZcorNoClay;
	misuraZ = MANUAL_Z_HOME_POS + ZcorNoClay;
	SERIAL_ECHOPAIR("Nuova misura Z = ", misuraZ);
	SERIAL_ECHOLNPGM(" ok");
	make_homing(false);
	st_synchronize();
	if (saving) Config_StoreSettings();
	}
}
