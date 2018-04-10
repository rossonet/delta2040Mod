/*
Modified by Dennis Patella WASPROJECT TEAM on 02/2015
http://www.wasproject.it
 */
#ifndef LANGUAGEW_H
#define LANGUAGEW_H


// Languages
//   Italian		default language
// 1  English		alternative language !!!Work ok!!!
// 2  French		alternative language !!!Work ok!!!
#define ALTERNATIVE_LANGUAGE_CHOICE 1  // Pick your alternative language from the list above



#define PROTOCOL_VERSION "1.0"

#if MOTHERBOARD == 7 || MOTHERBOARD == 71 || MOTHERBOARD == 72
	#define MACHINE_NAME "Ultimaker"
	#define FIRMWARE_URL "http://firmware.ultimaker.com"
#elif MOTHERBOARD == 80
	#define MACHINE_NAME "Rumba"
	#define FIRMWARE_URL "https://github.com/ErikZalm/Marlin/"
#elif MOTHERBOARD == 77
	#define MACHINE_NAME "3Drag"
	#define FIRMWARE_URL "http://3dprint.elettronicain.it/"
#elif MOTHERBOARD == 88
	#define MACHINE_NAME "Makibox"
	#define FIRMWARE_URL "https://github.com/ErikZalm/Marlin/"
#elif MOTHERBOARD == 405
	#define MACHINE_NAME "Delta 2040turbo2"
	#define FIRMWARE_URL "http://www.wasproject.it/"
	#define MSG_INFO_1L "Delta Wasp DUE 4.0S"
	#define MSG_INFO_2L "WASProject"
	#define MSG_INFO_3L "REV1.3.3.8 03/2018"

#else
	#ifdef CUSTOM_MENDEL_NAME
		#define MACHINE_NAME CUSTOM_MENDEL_NAME
	#else
		#define MACHINE_NAME "Mendel"
	#endif


	#define FIRMWARE_URL "http://www.wasproject.it/"
#endif


#ifndef MACHINE_UUID
   #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#endif




#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)
#define WELCOME_MSG MACHINE_NAME " OK!"


#define MSG_MARLIN "Marlin Wasp due "
// Serial Console Messages

	#define MSG_Enqueing "enqueing \""
	#define MSG_POWERUP "PowerUp"
	#define MSG_EXTERNAL_RESET " External Reset"
	#define MSG_BROWNOUT_RESET " Brown out Reset"
	#define MSG_WATCHDOG_RESET " Watchdog Reset"
	#define MSG_SOFTWARE_RESET " Software Reset"
	#define MSG_AUTHOR " | Author: "
	#define MSG_CONFIGURATION_VER " Last Updated: "
	#define MSG_FREE_MEMORY " Free Memory: "
	#define MSG_PLANNER_BUFFER_BYTES "  PlannerBufferBytes: "
	#define MSG_OK "ok"
	#define MSG_FILE_SAVED "Done saving file."
	#define MSG_ERR_LINE_NO "Line Number is not Last Line Number+1, Last Line: "
	#define MSG_ERR_CHECKSUM_MISMATCH "checksum mismatch, Last Line: "
	#define MSG_ERR_NO_CHECKSUM "No Checksum with line number, Last Line: "
	#define MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM "No Line Number with checksum, Last Line: "
	#define MSG_FILE_PRINTED "Done printing file"
	#define MSG_BEGIN_FILE_LIST "Begin file list"
	#define MSG_END_FILE_LIST "End file list"
	#define MSG_M104_INVALID_EXTRUDER "M104 Invalid extruder "
	#define MSG_M105_INVALID_EXTRUDER "M105 Invalid extruder "
	#define MSG_M200_INVALID_EXTRUDER "M200 Invalid extruder "
	#define MSG_M218_INVALID_EXTRUDER "M218 Invalid extruder "
	#define MSG_M221_INVALID_EXTRUDER "M221 Invalid extruder "
	#define MSG_ERR_NO_THERMISTORS "No thermistors - no temperature"
	#define MSG_M109_INVALID_EXTRUDER "M109 Invalid extruder "
	
	
	#define MSG_M115_REPORT "FIRMWARE_NAME:Marlin V1; Sprinter/grbl mashup for gen6 FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:" MACHINE_NAME " EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
	#define MSG_COUNT_X " Count X: "
	#define MSG_ERR_KILLED "Printer halted. kill() called!"
	#define MSG_ERR_STOPPED "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
	#define MSG_RESEND "Resend: "
	#define MSG_UNKNOWN_COMMAND "Unknown command: \""
	#define MSG_ACTIVE_EXTRUDER "Active Extruder: "
	#define MSG_INVALID_EXTRUDER "Invalid extruder"
	#define MSG_X_MIN "x_min: "
	#define MSG_X_MAX "x_max: "
	#define MSG_Y_MIN "y_min: "
	#define MSG_Y_MAX "y_max: "
	#define MSG_Z_MIN "z_min: "
	#define MSG_Z_MAX "z_max: "
	#define MSG_M119_REPORT "Reporting endstop status"
	#define MSG_ENDSTOP_HIT "TRIGGERED"
	#define MSG_ENDSTOP_OPEN "open"
	#define MSG_HOTEND_OFFSET "Hotend offsets:"

	#define MSG_SD_CANT_OPEN_SUBDIR "Cannot open subdir"
	#define MSG_SD_INIT_FAIL "SD init fail"
	#define MSG_SD_VOL_INIT_FAIL "volume.init failed"
	#define MSG_SD_OPENROOT_FAIL "openRoot failed"
	#define MSG_SD_CARD_OK "SD card ok"
	#define MSG_SD_WORKDIR_FAIL "workDir open failed"
	#define MSG_SD_OPEN_FILE_FAIL "open failed, File: "
	#define MSG_SD_FILE_OPENED "File opened: "
	#define MSG_SD_SIZE " Size: "
	
	#define MSG_SD_FILE_SELECTED "File selected"
	#define MSG_SD_WRITE_TO_FILE "Writing to file: "
	#define MSG_SD_PRINTING_BYTE "SD printing byte "
	#define MSG_SD_NOT_PRINTING "Not SD printing"
	#define MSG_SD_ERR_WRITE_TO_FILE "error writing to file"
	#define MSG_SD_CANT_ENTER_SUBDIR "Cannot enter subdir: "

	#define MSG_STEPPER_TOO_HIGH "Steprate too high: "
	#define MSG_ENDSTOPS_HIT "endstops hit: "
	#define MSG_ERR_COLD_EXTRUDE_STOP " cold extrusion prevented"
	#define MSG_ERR_LONG_EXTRUDE_STOP " too long extrusion prevented"
	#define MSG_BABYSTEPPING_X "Babystepping X"
	#define MSG_BABYSTEPPING_Y "Babystepping Y"
	#define MSG_BABYSTEPPING_Z "Babystepping Z"
	#define MSG_SERIAL_ERROR_MENU_STRUCTURE "Error in menu structure"


	//Italiano
	#define MSG_DEFAULT_LANG_SEL "Italiano selezionato"
	#define MSG_REBOOT_DEF 			"Riavviare    "
	#define MSG_SD_INSERTED        "SD Card inserita"
	#define MSG_SD_REMOVED         "SD Card rimossa"
	#define MSG_MAIN                 "Menu principale"
	#define MSG_MENU_LANG_CHG		"Lingua" 
	#define MSG_AUTOSTART            "Autostart"
	#define MSG_DISABLE_STEPPERS     "Disabilita Motori"
	#define MSG_AUTO_HOME            "Auto Home"
	#define MSG_SET_ORIGIN           "Imposta Origine"
	#define MSG_PREHEAT_PLA          "Preriscalda PLA"
	#define MSG_PREHEAT_PLA0         "Preriscalda PLA 1"
	#define MSG_PREHEAT_PLA1         "Preriscalda PLA 2"
	#define MSG_PREHEAT_PLA2         "Preriscalda PLA 3"
	#define MSG_PREHEAT_PLA012       "Preris. PLA Tutto"
	#define MSG_PREHEAT_PLA_BEDONLY  "Preri. PLA Piatto"
	#define MSG_PREHEAT_PLA_SETTINGS "Preris. PLA Conf"
	#define MSG_PREHEAT_ABS          "Preriscalda ABS"
	#define MSG_PREHEAT_ABS0         "Preriscalda ABS 1"
	#define MSG_PREHEAT_ABS1         "Preriscalda ABS 2"
	#define MSG_PREHEAT_ABS2         "Preriscalda ABS 3"
	#define MSG_PREHEAT_ABS012       "Preris. ABS Tutto"
	#define MSG_PREHEAT_ABS_BEDONLY  "Preri. ABS Piatto"
	#define MSG_PREHEAT_ABS_SETTINGS "Preris. ABS Conf"
	#define MSG_COOLDOWN             "Raffredda"
	#define MSG_SWITCH_PS_ON         "Switch Power On"
	#define MSG_SWITCH_PS_OFF        "Switch Power Off"
	#define MSG_EXTRUDE              "Estrudi"
	#define MSG_RETRACT              "Ritrai"
	#define MSG_MOVE_AXIS            "Muovi Asse"
	#define MSG_MOVE_X               "Move X"
	#define MSG_MOVE_Y               "Move Y"
	#define MSG_MOVE_Z               "Move Z"
	#define MSG_MOVE_E               "Extruder"
	#define MSG_MOVE_E1 			 "Extruder2"
	#define MSG_MOVE_E2 			 "Extruder3"
	#define MSG_MOVE_01MM            "Move 0.1mm"
	#define MSG_MOVE_1MM             "Move 1mm"
	#define MSG_MOVE_10MM            "Move 10mm"
	#define MSG_SPEED                "Velocita"
	#define MSG_NOZZLE               "Ugello"
	#define MSG_NOZZLE1              "Ugello2"
	#define MSG_NOZZLE2              "Ugello3"
	#define MSG_BED                  "Piatto"
	#define MSG_FAN_SPEED            "Ventola"
	#define MSG_FLOW                 "Flusso"
	#define MSG_FLOW0                "Flusso 0"
	#define MSG_FLOW1                "Flusso 1"
	#define MSG_FLOW2                "Flusso 2"
	#define MSG_CONTROL              "Avanzate"
	#define MSG_MIN                  " \002 Min:"
	#define MSG_MAX                  " \002 Max:"
	#define MSG_FACTOR               " \002 Fact:"
	#define MSG_AUTOTEMP             "Autotemp"
	#define MSG_ON                   "On "
	#define MSG_OFF                  "Off"
	#define MSG_PID_P                "PID-P"
	#define MSG_PID_I                "PID-I"
	#define MSG_PID_D                "PID-D"
	#define MSG_PID_C                "PID-C"
	#define MSG_ACC                  "Accel"
	#define MSG_VXY_JERK             "Vxy-jerk"
	#define MSG_VZ_JERK              "Vz-jerk"
	#define MSG_VE_JERK              "Ve-jerk"
	#define MSG_VMAX                 "Vmax"
	#define MSG_X                    "x"
	#define MSG_Y                    "y"
	#define MSG_Z                    "z"
	#define MSG_E                    "e"
	#define MSG_VMIN                 "Vmin"
	#define MSG_VTRAV_MIN            "VTrav min"
	#define MSG_AMAX                 "Amax"
	#define MSG_A_RETRACT            "A-retract"
	#define MSG_XSTEPS               "Xpassi/mm"
	#define MSG_YSTEPS               "Ypassi/mm"
	#define MSG_ZSTEPS               "Zpassi/mm"
	#define MSG_ESTEPS               "Epassi/mm"
	#define MSG_RECTRACT             "Ritrai"
	#define MSG_TEMPERATURE          "Temperatura"
	#define MSG_MOTION               "Movimento"
	#define MSG_CONTRAST "LCD contrast"
	#define MSG_STORE_EPROM          "Salva in EEPROM"
	#define MSG_LOAD_EPROM           "Carica da EEPROM"
	#define MSG_RESTORE_FAILSAFE     "Impostaz. default"
	#define MSG_REFRESH              "Aggiorna"
	#define MSG_WATCH                "Guarda"
	#define MSG_PREPARE              "Prepara"
	#define MSG_TUNE                 "Adatta"
	#define MSG_PAUSE_PRINT          "Pausa"
	#define MSG_PAUSE_PRINT_WASP	 "Cambia il filo"
	#define MSG_FIL_CRG				"Carica filamento"
	#define MSG_MODIFY_Z_MAX  	 "Modifica altezza"
	#define MSG_DRIVER_ADJ  	 "Corrente mot."
	#define MSG_FORCE_RESTART1 "NECESSARIO RIAVVIO"
	#define MSG_FORCE_RESTART2 "Assicurarsi di "
	#define MSG_FORCE_RESTART3 "aver inserito il"
	#define MSG_FORCE_RESTART4 "corretto estrusore"
	#define MSG_AUTOCALIB1	" ATTENZIONE"
	#define MSG_AUTOCALIB2	"Assicurarsi che le"
	#define MSG_AUTOCALIB3	"zone di contatto"
	#define MSG_AUTOCALIB4	"siano pulite"
	#define MSG_CLAYMODE      "Modo LDM (clay)"
	#define MSG_LCD_CLAY		"Modalita Clay "
	#define MSG_NUMEXTRUDERS	"Num. estrusori"
	#define MSG_CNG_EXT			"Cambia estrusore"
	#define	MSG_EMERGENCY_SAVE	 "Salvataggio"
	#define MSG_WASP_FILAMENTMOVE	 "Muovi estrusore"
	#define	MSG_WASP_MAN_LEVEL	 "Livellamento Man."
	#define MSG_LCD_MAN_CAL		"Posizione ->"
	#define MSG_RESUME_PRINT         "Riprendi Stampa"
	#define MSG_STOP_PRINT           "Arresta Stampa"
	#define MSG_STOP_SAVE_PRINT 	"Stop e Salva"
	#define MSG_CARD_MENU            "SD Card Menu"
	#define MSG_NO_CARD              "No SD Card"
	#define MSG_DWELL                "Sospensione..."
	#define MSG_USERWAIT             "Attendi Utente..."
	#define MSG_RESUMING             "Riprendi Stampa"
	#define MSG_NO_MOVE              "Nessun Movimento."
	#define MSG_KILLED               "Emergenza. "
	#define MSG_OUTOFPLATE 			"Fuori dal piatto."
	#define MSG_STOPPED              "ARRESTATO. "
	#define MSG_CONTROL_RETRACT      "Ritrai mm"
	#define MSG_CONTROL_RETRACTF     "Ritrai  V"
	#define MSG_CONTROL_RETRACT_ZLIFT "Salta mm"
	#define MSG_CONTROL_RETRACT_RECOVER "UnRet +mm"
	#define MSG_CONTROL_RETRACT_RECOVERF "UnRet  V"
	#define MSG_AUTORETRACT          "AutoArretramento"
	#define MSG_FILAMENTCHANGE       "Cambio filamento"
	#define MSG_INIT_SDCARD          "Iniz. SD-Card"
	#define MSG_CNG_SDCARD           "Cambia SD-Card"
	#define MSG_ZPROBE_OUT "Z probe out. bed"
	#define MSG_POSITION_UNKNOWN "Home X/Y before Z"
	#define MSG_ZPROBE_ZOFFSET "Z Offset"
	#define MSG_BABYSTEP_X "Babystep X"
	#define MSG_BABYSTEP_Y "Babystep Y"
	#define MSG_BABYSTEP_Z "Babystep Z"
	#define MSG_ENDSTOP_ABORT "Endstop abort"
	#define MSG_CONTRAST "Contrast"
	#define MSG_BED_HEATING "Riscaldamento piatto."
	#define MSG_BED_DONE "Piatto caldo."
	#define MSG_HEATING_COMPLETE "Stampante calda."
	#define MSG_HEATING "Riscaldamento..."
	#define MSG_SD_ERROR	"File corrotto."
	#define MSG_FILSENSOR "Sensore filo"
		
	//English as alternative
#if ALTERNATIVE_LANGUAGE_CHOICE == 1	

	#define MSG_ALTERNA_LANG_SEL "English  selected   "
	#define MSG_REBOOT_ALT "Reboot       "
	#define MSG_1_SD_INSERTED "Card inserted"
	#define MSG_1_SD_REMOVED "Card removed"
	#define MSG_1_MAIN "Main"
	#define MSG_1_MENU_LANG_CHG		"Language"
	#define MSG_1_AUTOSTART "Autostart"
	#define MSG_1_DISABLE_STEPPERS "Disable steppers"
	#define MSG_1_AUTO_HOME "Auto home"
	#define MSG_1_SET_ORIGIN "Set origin"
	#define MSG_1_PREHEAT_PLA "Preheat PLA"
	#define MSG_1_PREHEAT_PLA0 "Preheat PLA 1"
	#define MSG_1_PREHEAT_PLA1 "Preheat PLA 2"
	#define MSG_1_PREHEAT_PLA2 "Preheat PLA 3"
	#define MSG_1_PREHEAT_PLA012 "Preheat PLA All"
	#define MSG_1_PREHEAT_PLA_BEDONLY "Preheat PLA Bed"
	#define MSG_1_PREHEAT_PLA_SETTINGS "Preheat PLA conf"
	#define MSG_1_PREHEAT_ABS "Preheat ABS"
	#define MSG_1_PREHEAT_ABS0 "Preheat ABS 1"
	#define MSG_1_PREHEAT_ABS1 "Preheat ABS 2"
	#define MSG_1_PREHEAT_ABS2 "Preheat ABS 3"
	#define MSG_1_PREHEAT_ABS012 "Preheat ABS All"
	#define MSG_1_PREHEAT_ABS_BEDONLY "Preheat ABS Bed"
	#define MSG_1_PREHEAT_ABS_SETTINGS "Preheat ABS conf"
	#define MSG_1_COOLDOWN "Cooldown"
	#define MSG_1_SWITCH_PS_ON "Switch power on"
	#define MSG_1_SWITCH_PS_OFF "Switch power off"
	#define MSG_1_EXTRUDE "Extrude"
	#define MSG_1_RETRACT "Retract"
	#define MSG_1_MOVE_AXIS "Move axis"
	#define MSG_1_MOVE_X "Move X"
	#define MSG_1_MOVE_Y "Move Y"
	#define MSG_1_MOVE_Z "Move Z"
	#define MSG_1_MOVE_E "Extruder"
	#define MSG_1_MOVE_E1 "Extruder2"
	#define MSG_1_MOVE_E2 "Extruder3"
	#define MSG_1_MOVE_01MM "Move 0.1mm"
	#define MSG_1_MOVE_1MM "Move 1mm"
	#define MSG_1_MOVE_10MM "Move 10mm"
	#define MSG_1_SPEED "Speed"
	#define MSG_1_NOZZLE "Nozzle"
	#define MSG_1_NOZZLE1 "Nozzle2"
	#define MSG_1_NOZZLE2 "Nozzle3"
	#define MSG_1_BED "Bed"
	#define MSG_1_FAN_SPEED "Fan speed"
	#define MSG_1_FLOW "Flow"
	#define MSG_1_FLOW0 "Flow 0"
	#define MSG_1_FLOW1 "Flow 1"
	#define MSG_1_FLOW2 "Flow 2"
	#define MSG_1_CONTROL "Advanced"
	#define MSG_1_MIN " \002 Min"
	#define MSG_1_MAX " \002 Max"
	#define MSG_1_FACTOR " \002 Fact"
	#define MSG_1_AUTOTEMP "Autotemp"
	#define MSG_1_ON "On "
	#define MSG_1_OFF "Off"
	#define MSG_1_PID_P "PID-P"
	#define MSG_1_PID_I "PID-I"
	#define MSG_1_PID_D "PID-D"
	#define MSG_1_PID_C "PID-C"
	#define MSG_1_ACC  "Accel"
	#define MSG_1_VXY_JERK "Vxy-jerk"
	#define MSG_1_VZ_JERK "Vz-jerk"
	#define MSG_1_VE_JERK "Ve-jerk"
	#define MSG_1_VMAX "Vmax "
	#define MSG_1_X "x"
	#define MSG_1_Y "y"
	#define MSG_1_Z "z"
	#define MSG_1_E "e"
	#define MSG_1_VMIN "Vmin"
	#define MSG_1_VTRAV_MIN "VTrav min"
	#define MSG_1_AMAX "Amax "
	#define MSG_1_A_RETRACT "A-retract"
	#define MSG_1_XSTEPS "Xsteps/mm"
	#define MSG_1_YSTEPS "Ysteps/mm"
	#define MSG_1_ZSTEPS "Zsteps/mm"
	#define MSG_1_ESTEPS "Esteps/mm"
	#define MSG_1_RECTRACT "Rectract"
	#define MSG_1_TEMPERATURE "Temperature"
	#define MSG_1_MOTION "Motion"
	#define MSG_1_CONTRAST "LCD contrast"
	#define MSG_1_STORE_EPROM "Store memory"
	#define MSG_1_LOAD_EPROM "Load memory"
	#define MSG_1_RESTORE_FAILSAFE "Restore failsafe"
	#define MSG_1_REFRESH "Refresh"
	#define MSG_1_WATCH "Info screen"
	#define MSG_1_PREPARE "Prepare"
	#define MSG_1_TUNE "Tune"
	#define MSG_1_PAUSE_PRINT "Pause print"
	#define MSG_1_PAUSE_PRINT_WASP "Pause for filament"
	#define MSG_1_FORCE_RESTART1 "PLEASE RESTART"
	#define MSG_1_FORCE_RESTART2 "Ensure "
	#define MSG_1_FORCE_RESTART3 "that the right"
	#define MSG_1_FORCE_RESTART4 "extruder is mounted"
	#define MSG_1_AUTOCALIB1	" ATTENTION"
	#define MSG_1_AUTOCALIB2	"Ensure that"
	#define MSG_1_AUTOCALIB3	"contact areas"
	#define MSG_1_AUTOCALIB4	"are clean"
	#define MSG_1_MODIFY_Z_MAX  "Modify Z MAX"
	#define MSG_1_DRIVER_ADJ  	 "Driver current adj"
	#define MSG_1_CLAYMODE "LDM mode (clay)"
	#define MSG_1_LCD_CLAY		"Clay Mode "
	#define MSG_1_NUMEXTRUDERS	"Num. extruders"
	#define MSG_1_CNG_EXT		"Change extruder"
	#define	MSG_1_WASP_MAN_LEVEL	 "Manual Leveling"
	#define MSG_1_LCD_MAN_CAL		"Position ->"
	#define	MSG_1_EMERGENCY_SAVE	 "Saving"
	#define MSG_1_RESUME_PRINT "Resume print"
	#define MSG_1_STOP_PRINT "Stop print"
	#define MSG_1_STOP_SAVE_PRINT "Stop and Save"
	#define MSG_1_CARD_MENU "Print from SD"
	#define MSG_1_NO_CARD "No SD card"
	#define MSG_1_DWELL "Sleep..."
	#define MSG_1_USERWAIT "Wait for user..."
	#define MSG_1_RESUMING "Resuming print"
	#define MSG_1_NO_MOVE "No move."
	#define MSG_1_KILLED "Emergency stop. "
	#define MSG_1_OUTOFPLATE "Out of Plate"
	#define MSG_1_STOPPED "STOPPED. "
	#define MSG_1_CONTROL_RETRACT  "Retract mm"
	#define MSG_1_CONTROL_RETRACTF "Retract  V"
	#define MSG_1_CONTROL_RETRACT_ZLIFT "Hop mm"
	#define MSG_1_CONTROL_RETRACT_RECOVER "UnRet +mm"
	#define MSG_1_CONTROL_RETRACT_RECOVERF "UnRet  V"
	#define MSG_1_AUTORETRACT "AutoRetr."
	#define MSG_1_FILAMENTCHANGE "Change filament"
	#define MSG_1_FIL_CRG		"Load Filament"
	#define MSG_1_INIT_SDCARD "Init. SD card"
	#define MSG_1_CNG_SDCARD "Change SD card"
	#define MSG_1_ZPROBE_OUT "Z probe out. bed"
	#define MSG_1_POSITION_UNKNOWN "Home X/Y before Z"
	#define MSG_1_ZPROBE_ZOFFSET "Z Offset"
	#define MSG_1_BABYSTEP_X "Babystep X"
	#define MSG_1_BABYSTEP_Y "Babystep Y"
	#define MSG_1_BABYSTEP_Z "Babystep Z"
	#define MSG_1_ENDSTOP_ABORT "Endstop abort"
	#define MSG_1_BED_HEATING "Bed Heating."
	#define MSG_1_BED_DONE "Bed done."
	#define MSG_1_HEATING_COMPLETE "Heating done."
	#define MSG_1_HEATING "Heating..."
	#define MSG_1_SD_ERROR	"File corrupted."
	#define MSG_1_FILSENSOR "Filament sensor"
#endif	


#if ALTERNATIVE_LANGUAGE_CHOICE == 2	

// Francaise as alternative		
	#define MSG_ALTERNA_LANG_SEL "Francaise  sel.     "
	#define MSG_REBOOT_ALT "Redemarrer      "
	#define MSG_1_SD_INSERTED "Carte inseree"
	#define MSG_1_SD_REMOVED "Carte retiree"
	#define MSG_1_MAIN "Menu principal"
	#define MSG_1_MENU_LANG_CHG		"Langue "
	#define MSG_1_AUTOSTART "Demarrage auto"
	#define MSG_1_DISABLE_STEPPERS "Arreter moteurs"
	#define MSG_1_AUTO_HOME "Home auto."
	#define MSG_1_SET_ORIGIN "Regler origine"
	#define MSG_1_PREHEAT_PLA " Prechauffage PLA"
	#define MSG_1_PREHEAT_PLA0 "Prechauff. PLA 1"
    #define MSG_1_PREHEAT_PLA1 "Prechauff. PLA 2"
	#define MSG_1_PREHEAT_PLA2 "Prechauff. PLA 3"
	#define MSG_1_PREHEAT_PLA012 "Prech. PLA Tout"
	#define MSG_1_PREHEAT_PLA_BEDONLY "Prech. PLA Plateau"
	#define MSG_1_PREHEAT_PLA_SETTINGS "Regl. prech. PLA"
	#define MSG_1_PREHEAT_ABS "Prechauffage ABS"
	#define MSG_1_PREHEAT_ABS0 "Prechauff. ABS 1"
	#define MSG_1_PREHEAT_ABS1 "Prechauff. ABS 2"
	#define MSG_1_PREHEAT_ABS2 "Prechauff. ABS 3"
	#define MSG_1_PREHEAT_ABS012 "Prech. ABS Tout"
	#define MSG_1_PREHEAT_ABS_BEDONLY "Prech. ABS Plateau"
	#define MSG_1_PREHEAT_ABS_SETTINGS "Regl. prech. ABS"
	#define MSG_1_COOLDOWN "Refroidir"
	#define MSG_1_SWITCH_PS_ON "Allumer alim."
	#define MSG_1_SWITCH_PS_OFF "Eteindre alim."
	#define MSG_1_EXTRUDE "Extrusion"
	#define MSG_1_RETRACT "Retraction"
	#define MSG_1_PREHEAT_PLA "Prechauffage PLA"
	#define MSG_1_PREHEAT_ABS "Prechauffage ABS"
	#define MSG_1_MOVE_AXIS "Deplacer un axe"
	#define MSG_1_MOVE_X "Move X"
	#define MSG_1_MOVE_Y "Move Y"
	#define MSG_1_MOVE_Z "Move Z"
	#define MSG_1_MOVE_E "Extruder"
	#define MSG_1_MOVE_E1 "Extruder2"
	#define MSG_1_MOVE_E2 "Extruder3"
	#define MSG_1_MOVE_01MM "Move 0.1mm"
	#define MSG_1_MOVE_1MM "Move 1mm"
	#define MSG_1_MOVE_10MM "Move 10mm"
	#define MSG_1_SPEED " Vitesse"
	#define MSG_1_NOZZLE "Buse"
	#define MSG_1_NOZZLE1 "Buse2"
	#define MSG_1_NOZZLE2 "Buse3"
	#define MSG_1_BED "Plateau"
	#define MSG_1_FAN_SPEED "Vite. ventilateur"
	#define MSG_1_FLOW "Flux"
	#define MSG_1_FLOW0 "Flux 0"
	#define MSG_1_FLOW1 "Flux 1"
	#define MSG_1_FLOW2 "Flux 2"
	#define MSG_1_CONTROL "Controler"
	#define MSG_1_MIN " \002 Min"
	#define MSG_1_MAX " \002 Max"
	#define MSG_1_FACTOR " \002 Facteur"
	#define MSG_1_AUTOTEMP "Temp. Auto."
	#define MSG_1_ON "Marche "
	#define MSG_1_OFF "Arret"
	#define MSG_1_PID_P "PID-P"
	#define MSG_1_PID_I "PID-I"
	#define MSG_1_PID_D "PID-D"
	#define MSG_1_PID_C "PID-C"
	#define MSG_1_ACC "Accel"
	#define MSG_1_VXY_JERK "Vxy-jerk"
	#define MSG_1_VZ_JERK "Vz-jerk"
	#define MSG_1_VE_JERK "Ve-jerk"
	#define MSG_1_VMAX "Vmax"
	#define MSG_1_X "x"
	#define MSG_1_Y "y"
	#define MSG_1_Z "z"
	#define MSG_1_E "e"
	#define MSG_1_VMIN "Vmin"
	#define MSG_1_VTRAV_MIN "Vdepl min"
	#define MSG_1_AMAX "Amax "
	#define MSG_1_A_RETRACT "A-retract"
	#define MSG_1_XSTEPS "Xpas/mm"
	#define MSG_1_YSTEPS "Ypas/mm"
	#define MSG_1_ZSTEPS "Zpas/mm"
	#define MSG_1_ESTEPS "Epas/mm"
	#define MSG_1_TEMPERATURE "Temperature"
	#define MSG_1_MOTION "Mouvement"
	#define MSG_1_CONTRAST "Contraste LCD"
	#define MSG_1_STORE_EPROM "Sauver config"
	#define MSG_1_LOAD_EPROM "Lire config"
	#define MSG_1_RESTORE_FAILSAFE "Restaurer defauts"
	#define MSG_1_REFRESH "Actualiser"
	#define MSG_1_WATCH "Surveiller"
	#define MSG_1_PREPARE "Preparer"
	#define MSG_1_TUNE "Regler"
	#define MSG_1_PAUSE_PRINT "Interrompre impr."
	#define MSG_1_PAUSE_PRINT_WASP "Interrompre impr."
	#define MSG_1_FORCE_RESTART1 "Effectuez un redemar"
	#define MSG_1_FORCE_RESTART2 "rage. Assurez-vous"
	#define MSG_1_FORCE_RESTART3 "les extrudeur"
	#define MSG_1_FORCE_RESTART4 "est appropriee"
	#define MSG_1_MODIFY_Z_MAX  "Changement Z MAX"
	#define MSG_1_DRIVER_ADJ  	 "Courant au moteurs"
	#define MSG_1_LCD_MAN_CAL		"Emplacement ->"
	#define MSG_1_CLAYMODE 	"LDM mode (clay)"
	#define MSG_1_LCD_CLAY		"Clay Mode "
	#define	MSG_1_WASP_MAN_LEVEL	 "Nivellement manu."
	#define	MSG_1_EMERGENCY_SAVE	 "Sauve"
	#define MSG_1_RESUME_PRINT "Reprendre impr."
	#define MSG_1_STOP_PRINT "Arreter impr."
	#define MSG_1_STOP_SAVE_PRINT 	"Arreter et sauver"
	#define MSG_1_CARD_MENU "Impr. depuis SD"
	#define MSG_1_NO_CARD "Pas de carte"
	#define MSG_1_DWELL "Repos..."
	#define MSG_1_USERWAIT "Atten. de l'util."
	#define MSG_1_RESUMING "Repri. de l'impr."
	#define MSG_1_NO_MOVE "Aucun mouvement."
	#define MSG_1_KILLED "MORT."
	#define MSG_1_STOPPED "STOPPE."
	#define MSG_1_STEPPER_RELEASED "RELACHE."
	#define MSG_1_CONTROL_RETRACT "Retraction mm"
	#define MSG_1_CONTROL_RETRACTF "Retraction V"
	#define MSG_1_CONTROL_RETRACT_ZLIFT "Hop mm"
	#define MSG_1_CONTROL_RETRACT_RECOVER "UnRet +mm"
	#define MSG_1_CONTROL_RETRACT_RECOVERF "UnRet V"
	#define MSG_1_AUTORETRACT "Retract. Auto."
	#define MSG_1_FILAMENTCHANGE "Changer filament"
	#define MSG_1_INIT_SDCARD "Init. la carte SD"
	#define MSG_1_CNG_SDCARD "Changer de carte"
	#define MSG_1_ZPROBE_OUT "Z sonde exte. lit"
	#define MSG_1_POSITION_UNKNOWN "Rev. dans XY av.Z"
	#define MSG_1_ZPROBE_ZOFFSET "Offset Z"
	#define MSG_1_BABYSTEP_X "Babystep X"
	#define MSG_1_BABYSTEP_Y "Babystep Y"
	#define MSG_1_BABYSTEP_Z "Babystep Z"
	#define MSG_1_ENDSTOP_ABORT "Butee abandon"
	#define MSG_1_CONTRAST "Contrast"
	#define MSG_1_BED_HEATING "Bed Heating."
	#define MSG_1_BED_DONE "Bed done."
	#define MSG_1_HEATING_COMPLETE "Heating done."
	#define MSG_1_HEATING "Heating..."
	#define MSG_1_SD_ERROR	"File corrupted."

#endif
	
	
	#endif
