/*
Modified by Dennis Patella WASPROJECT TEAM on 2015
http://www.wasproject.it
 */
#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"
#include <avr/dtostrf.h>

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
	
	
}

#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
//#define EEPROM_WRITE_VAR_PAGE(pos, value) writeEEPROM(pos,  (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

//======================================================================================


#define EEPROM_OFFSET 0


#define EEPROM_VERSION "V6S"


#ifdef EEPROM_SETTINGS


void Config_StoreSettings() 
{
  char ver[4]= EEPROM_VERSION;
 
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit); 
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,extruder_offset_X);
  EEPROM_WRITE_VAR(i,extruder_offset_Y);
  EEPROM_WRITE_VAR(i,extruder_offset_Z);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homeing);
  EEPROM_WRITE_VAR(i,defaultfan);
  EEPROM_WRITE_VAR(i,extr_type);
  EEPROM_WRITE_VAR(i,DualExtMode);
  EEPROM_WRITE_VAR(i,DualSensorFilament);
  #ifdef DELTA
  EEPROM_WRITE_VAR(i,endstop_adj);
  EEPROM_WRITE_VAR(i,delta_radius);
  EEPROM_WRITE_VAR(i,delta_diagonal_rod);
  EEPROM_WRITE_VAR(i,delta_segments_per_second);
  EEPROM_WRITE_VAR(i,dA);
  EEPROM_WRITE_VAR(i,dB);
  EEPROM_WRITE_VAR(i,dC);
  EEPROM_WRITE_VAR(i,max_plater_radius);
  #endif
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,zprobe_zoffset);
  #ifdef PIDTEMP
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);
  #else
		float dummy = 3000.0f;
    EEPROM_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    EEPROM_WRITE_VAR(i,dummy);
    EEPROM_WRITE_VAR(i,dummy);
  #endif
  #ifndef DOGLCD
    int lcd_contrast = 32;
  #endif
  EEPROM_WRITE_VAR(i,lcd_contrast);
  EEPROM_WRITE_VAR(i,clayMode);
  EEPROM_WRITE_VAR(i,Zcorrect);
  EEPROM_WRITE_VAR(i,ZcorNoClay);
  EEPROM_WRITE_VAR(i,Lang);
  EEPROM_WRITE_VAR(i,currentControlDriver);
  EEPROM_WRITE_VAR(i,tmp_e_step_per_unit);
  EEPROM_WRITE_VAR(i,filSensor);
  EEPROM_WRITE_VAR(i,DualExtMode);
  EEPROM_WRITE_VAR(i,DualSensorFilament);
  EEPROM_WRITE_VAR(i,doorExcluded);
  EEPROM_WRITE_VAR(i,continuousPrint);
  EEPROM_WRITE_VAR(i,extruderCheck);
  char ver2[4]=EEPROM_VERSION;

  
  EEPROM_WRITE_VAR(i,ver2); // validate data
  
  
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
    SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
    SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
    SERIAL_ECHOPAIR(" Y",max_feedrate[1] ); 
    SERIAL_ECHOPAIR(" Z", max_feedrate[2] ); 
    SERIAL_ECHOPAIR(" E", max_feedrate[3]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] ); 
    SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] ); 
    SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
    SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration ); 
    SERIAL_ECHOPAIR(" T" ,retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
    SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
    SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk);
    SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Home offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X",add_homeing[0] );
    SERIAL_ECHOPAIR(" Y" ,add_homeing[1] );
    SERIAL_ECHOPAIR(" Z" ,add_homeing[2] );
    SERIAL_ECHOLN("");
#if EXTRUDERS > 1
    SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Extruders offset (mm):");
	SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M218 T0 X",extruder_offset_X[0] );
	SERIAL_ECHOPAIR(" Y" ,extruder_offset_Y[0] );
	SERIAL_ECHOPAIR(" Z" ,extruder_offset_Z[0] );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M218 T1 X",extruder_offset_X[1] );
	SERIAL_ECHOPAIR(" Y" ,extruder_offset_Y[1] );
	SERIAL_ECHOPAIR(" Z" ,extruder_offset_Z[1] );
	SERIAL_ECHOLN("");
#endif
#ifdef DELTA
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop adjustement (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M666 X",endstop_adj[0] );
    SERIAL_ECHOPAIR(" Y" ,endstop_adj[1] );
    SERIAL_ECHOPAIR(" Z" ,endstop_adj[2] );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Delta settings: L=delta_diagonal_rod, R=delta_radius, S=delta_segments_per_second, A, B, C = mm adj for X,Y,Z column");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M665 L",delta_diagonal_rod );
	SERIAL_ECHOPAIR(" R" ,delta_radius );
	SERIAL_ECHOPAIR(" S" ,delta_segments_per_second );
	SERIAL_ECHOPAIR(" A" ,dA );
	SERIAL_ECHOPAIR(" B" ,dB );
	SERIAL_ECHOPAIR(" C" ,dC );
	if (!clayMode) SERIAL_ECHOPAIR(" Z" ,MANUAL_Z_HOME_POS+ZcorNoClay );
	else SERIAL_ECHOPAIR(" Z" ,MANUAL_Z_HOME_POS+Zcorrect );
	SERIAL_ECHOPAIR(" M" ,max_plater_radius );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHO("Clay Mode ");
	SERIAL_ECHOLN(clayMode);
	SERIAL_ECHO("Door exclusion ");
	SERIAL_ECHOLN(doorExcluded);
	SERIAL_ECHO_START;
	SERIAL_ECHO("Primary language sel. (0=IT 1=EN 2=FR) ");
	SERIAL_ECHOLN(Lang);
#endif
#ifdef WASP
	SERIAL_ECHO_START;
	SERIAL_ECHO("Control the drivers current: M777 C");
	SERIAL_ECHOLN(currentControlDriver);
	if (resurrAtZ>0) {
	SERIAL_ECHO_START;
	SERIAL_ECHO("Resurrection from Z=");
	SERIAL_ECHOLN(resurrAtZ);
	}
	/*float drivCurrent = (((int)currentControlDriver*3.3)/254)/0.5;
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("Actual calculated current is " ,drivCurrent );
	SERIAL_ECHOLN("A");
	*/
#endif
#ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M301 P",Kp); 
    SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki)); 
    SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd));
	SERIAL_ECHO(" EXTR TYPE:");
	SERIAL_ECHO(extr_type);
    SERIAL_ECHOLN(""); 
#endif
	SERIAL_ECHO_START;
	SERIAL_ECHO("  M106 L");
	SERIAL_ECHOLN(defaultfan ); 
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
	 
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;

    EEPROM_READ_VAR(i,stored_ver); //read stored version

#ifdef VERBOSE
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    SERIAL_ECHO_START;
    SERIAL_ECHO(" Version: ");
    SERIAL_ECHO(ver);
    SERIAL_ECHO(" Stored: ");
    SERIAL_ECHO(stored_ver);
    SERIAL_ECHOLN("");
#endif


    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);  
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
        EEPROM_READ_VAR(i,extruder_offset_X);
		EEPROM_READ_VAR(i,extruder_offset_Y);
		EEPROM_READ_VAR(i,extruder_offset_Z);
		
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
        
        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homeing);
		EEPROM_READ_VAR(i,defaultfan);
		EEPROM_READ_VAR(i,extr_type);
		EEPROM_READ_VAR(i,DualExtMode);
		EEPROM_READ_VAR(i,DualSensorFilament);
        #ifdef DELTA
		EEPROM_READ_VAR(i,endstop_adj);
		EEPROM_READ_VAR(i,delta_radius);
		EEPROM_READ_VAR(i,delta_diagonal_rod);
		EEPROM_READ_VAR(i,delta_segments_per_second);
		EEPROM_READ_VAR(i,dA);
		EEPROM_READ_VAR(i,dB);
		EEPROM_READ_VAR(i,dC);
		EEPROM_READ_VAR(i,max_plater_radius);
        #endif
        #ifndef ULTIPANEL
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
        #endif
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
        EEPROM_READ_VAR(i,zprobe_zoffset);
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);
        #ifndef DOGLCD
        int lcd_contrast;
        #endif
        EEPROM_READ_VAR(i,lcd_contrast);
        EEPROM_READ_VAR(i,clayMode);
        EEPROM_READ_VAR(i,Zcorrect);
		EEPROM_READ_VAR(i,ZcorNoClay);
		EEPROM_READ_VAR(i,Lang);
		EEPROM_READ_VAR(i,currentControlDriver);
        EEPROM_READ_VAR(i,tmp_e_step_per_unit); 
		EEPROM_READ_VAR(i,filSensor);
		EEPROM_READ_VAR(i,DualExtMode);
		EEPROM_READ_VAR(i,DualSensorFilament);
		EEPROM_READ_VAR(i,doorExcluded);
		EEPROM_READ_VAR(i,continuousPrint);
		EEPROM_READ_VAR(i,extruderCheck);
        currentControlDriver=constrain(currentControlDriver,0,254);
		// Call updatePID (similar to when we have processed M301)
		updatePID();
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    }
    else
    {
        Config_ResetDefault();
    }
	
    #ifdef EEPROM_CHITCHAT
      Config_PrintSettings();
    #endif
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
	float tmpEX[]=EXTRUDER_OFFSET_X;
	float tmpEY[]=EXTRUDER_OFFSET_Y;
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }
    tmp_e_step_per_unit=CLAYMODE_E_AXIS_STEPS_PER_UNIT;
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    for (short i=0;i<EXTRUDERS;i++) 
    {
    extruder_offset_X[i]=tmpEX[i];
	extruder_offset_Y[i]=tmpEY[i];
	extruder_offset_Z[i]=0;
	}
	for (short i=0;i<EXTRUDERS;i++) 
    {
	extruder_offset[X_AXIS][i]=tmpEX[i];
	extruder_offset[Y_AXIS][i]=tmpEY[i];
	extruder_offset[Z_AXIS][i]=0;
	}
    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
	defaultfan=30;
	DualExtMode=true;
	DualSensorFilament=true;
#ifdef DELTA
	endstop_adj[0] = endstop_adj[1] = endstop_adj[2] = 0;
	delta_radius= DELTA_RADIUS;
	dA=0.0;
	dB=0.0;
	dC=0.0;
	delta_diagonal_rod= DELTA_DIAGONAL_ROD;
	delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;
	max_plater_radius=MAX_PLATER;
	recalc_delta_settings(delta_radius, delta_diagonal_rod);
	doorExcluded=true;
	
#endif
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef ENABLE_AUTO_BED_LEVELING
    zprobe_zoffset = -Z_PROBE_OFFSET_FROM_EXTRUDER;
#endif
#ifdef DOGLCD
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP
    Lang=0;
    clayMode = false;  //Default mode Delta//
	Zcorrect = 0.00;
	ZcorNoClay = Zcorrect;
	tmpZcorrect = Zcorrect; 
	currentControlDriver=170;
	filSensor=true;
	resurrectionData.resurrActive=false;
	resurrectionData.autoResurr=false;
	char buf[13]="\0";
	memcpy(nameFile,buf,strlen(buf)+1);
	memcpy(nameDir,buf,strlen(buf)+1);
	resurrectionData.high = 0.0;
	resurrectionData.X = 0.0;
	resurrectionData.Y=0.0;
	resurrectionData.E = 0.0;
	resurrectionData.fan = defaultfan;
	resurrectionData.ext_selected=0;
	resurrectionData_bed = 0;
	resurrectionData.extruder=0;
	resurrectionData.feedrate= 3600.0;
	resurrectionData.feedmultiply=100;
	resurrectionData.sdpos=0;
	Store_ResurData();
SERIAL_ECHO_START;
SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");

}
void Store_tempbed(int indexEEPROM) {
	
	EEPROM_WRITE_VAR(indexEEPROM,resurrectionData_bed);
	
}
void Store_ResurFile() {
	int i=RESURR_OFFSET;
	EEPROM_WRITE_VAR(i,nameDir);
	EEPROM_WRITE_VAR(i,nameFile);
	
}
void Store_ResurData() {
	
	
	uint32_t start=millis();
	int i=RESURR_OFFSET+sizeof(nameDir)+sizeof(nameFile);
	/*char buf[128];
	char bufSDpos[resurrectionData.sdpos];
	char bufX[sizeof(resurrectionData.X)];
	char bufY[sizeof(resurrectionData.Y)];
	char bufZ[sizeof(resurrectionData.high)];
	char bufE[sizeof(resurrectionData.E)];
	char bufFan[sizeof(resurrectionData.fan)];
	char bufBed[sizeof(resurrectionData.bed)];
	char bufEX[sizeof(resurrectionData.extruder)];
	char bufFE[sizeof(resurrectionData.feedrate)];
	char bufMU[sizeof(resurrectionData.feedmultiply)];
	char bufAC[sizeof(resurrectionData.resurrActive)];
	char bufAU[sizeof(resurrectionData.autoResurr)];
	snprintf(bufSDpos, sizeof bufSDpos, "%lu", (unsigned long)resurrectionData.sdpos); 
	dtostrf(resurrectionData.X,1,3,bufX);
	dtostrf(resurrectionData.Y,1,3,bufY);
	dtostrf(resurrectionData.high,1,3,bufZ);
	dtostrf(resurrectionData.E,1,3,bufE);
	
	strcpy(buf,resurrectionData.nameDir);
	strcat(buf,";");
	strcat(buf,resurrectionData.nameFile);
	strcat(buf,";");
	strcat(buf,bufSDpos);
	strcat(buf,";");
	strcat(buf,bufX);
	strcat(buf,";");
	strcat(buf,bufY);
	strcat(buf,";");
	strcat(buf,bufZ);
	strcat(buf,";");
	strcat(buf,bufE);
	strcat(buf,";");
	*/
	//writeEEPROM(i+1024,buf,128);
	EEPROM_WRITE_VAR(i,resurrectionData);
	//writeEEPROM(i,  (char*) &resurrectionData, sizeof(resurrectionData));
	//eeprom_write_page(i,(uint8_t *)&resurrectionData,sizeof(resurrectionData));
	start=millis()-start;
	/*
	SERIAL_ECHO_START;
	SERIAL_ECHO("Time for saving resurrection data: ");
	SERIAL_ECHO(start);
	SERIAL_ECHOLN("msec");
*/
}
void Retrieve_ResurData() {
	int i=RESURR_OFFSET;
	EEPROM_READ_VAR(i,nameDir);
	EEPROM_READ_VAR(i,nameFile);
	//char buf[sizeof(resurrectionData)];
EEPROM_READ_VAR(i,resurrectionData);
EEPROM_READ_VAR(i,resurrectionData_bed);
/*EEPROM_READ_VAR(i+1024,buf);
for (uint32_t a=0;a<sizeof(resurrectionData);a++)
	SERIAL_ECHO(buf[a]);
//memcpy(&resurrectionData,buf,sizeof(resurrectionData));
*/
}
