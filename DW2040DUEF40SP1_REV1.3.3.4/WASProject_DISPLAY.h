


#ifndef WASP_LCD_IMPLEMENTATION_H
#define WASP_LCD_IMPLEMENTATION_H

/**
* Implementation of the LCD display routines for a 128x64 graphic display.
**/

#ifdef ULTIPANEL
#define BLEN_A 0
#define BLEN_B 1
#define BLEN_C 2
#define EN_A (1<<BLEN_A)
#define EN_B (1<<BLEN_B)
#define EN_C (1<<BLEN_C)
#define encrot0 0
#define encrot1 2
#define encrot2 3
#define encrot3 1
#define LCD_CLICKED (buttons&EN_C)
#endif

#include <U8glib.h>
#include "WASPbmp.h"
#include "dogm_font_data_marlin.h"
#include "ultralcd.h"
#include "ultralcd_st7920_u8glib_rrd.h"




// DOGM parameters (size in pixels)
#define DOG_CHAR_WIDTH			6
#define DOG_CHAR_HEIGHT			12
#define DOG_CHAR_WIDTH_LARGE	9
#define DOG_CHAR_HEIGHT_LARGE	18


#define START_ROW				0


/* Custom characters defined in font font_6x10_marlin.c */
#define LCD_STR_BEDTEMP     "\xFE"
#define LCD_STR_DEGREE      "\xB0"
#define LCD_STR_THERMOMETER "\xFF"
#define LCD_STR_UPLEVEL     "\xFB"
#define LCD_STR_REFRESH     "\xF8"
#define LCD_STR_FOLDER      "\xF9"
#define LCD_STR_FEEDRATE    "\xFD"
#define LCD_STR_CLOCK       "\xFC"
#define LCD_STR_ARROW_RIGHT "\xFA"

#define FONT_STATUSMENU	u8g_font_6x9

int lcd_contrast=1;

// LCD selection

//U8GLIB_ST7920_128X64_4X u8g(16, 5, 15);
U8GLIB_ST7920_128X64_RRD u8g(0);

bool init_screen=true;
/*
char lcd_print(char c) {
  if ((c > 0) && (c <= LCD_STR_SPECIAL_MAX)) {
    u8g.setFont(FONT_SPECIAL_NAME);
    u8g.print(c);
    lcd_setFont(currentfont);
    return 1;
  } else {
    return charset_mapper(c);
  }
}

char lcd_print(char* str) {
  char c;
  int i = 0;
  char n = 0;
  while ((c = str[i++])) {
    n += lcd_print(c);
  }
  return n;
}
*/
static void lcd_implementation_init()
{
#ifdef LCD_PIN_BL
	pinMode(LCD_PIN_BL, OUTPUT);	// Enable LCD backlight
	digitalWrite(LCD_PIN_BL, HIGH);
#endif

    u8g.setContrast(lcd_contrast);	
	
	
	//	u8g.setRot180();	// Rotate screen by 180°
	u8g.firstPage();

	do {
			if (init_screen) {
			u8g.drawBitmapP(0,0,START_BMPBYTEWIDTH,START_BMPHEIGHT,logo_wasp);
			// Welcome message
			u8g.setFont(u8g_font_5x8);
			u8g.drawStr(58,10,"Wasp Turbo 2.0"); 
			u8g.setFont(u8g_font_5x8);
			u8g.drawStr(62,19,"MARLIN");
			u8g.drawStr(72,28,"Compatible");
			u8g.setFont(u8g_font_5x8);
			u8g.drawStr(62,48,"By Dennis");
			u8g.drawStr(72,55,"WASProject");
			card.initsd();
			}
	   } while(u8g.nextPage());
	   if (init_screen)
		   delay(5000);
	   init_screen=false;
	   
	   }
static void lcd_splash_screen() {
/*	
	u8g.firstPage();

	do {
			
			u8g.drawBitmapP(0,0,START_RESBMP,START_RESBMPHEIGHT,resurrection);
			// Welcome message
			u8g.setFont(u8g_font_5x8);
			u8g.drawStr(58,10,"FW 3.1Rev2.1"); 
			u8g.setFont(u8g_font_5x8);
			u8g.drawStr(62,19,"DELTAWASP");
			u8g.drawStr(72,28,"Turbo");
			u8g.setFont(u8g_font_5x8);
			u8g.drawStr(62,48,"RESURRECTION");
			u8g.drawStr(72,55,"Free Zeta System");
			
			
	   } while(u8g.nextPage());
*/	
}
static void lcd_implementation_clear(){}

/* Arduino < 1.0.0 is missing a function to print PROGMEM strings, so we need to implement our own */
static void lcd_printPGM(const char* str)
{
    char c;
    while((c = pgm_read_byte(str++)) != '\0')
    {
			u8g.print(c);
    }
}


static void lcd_implementation_status_screen()
{
	if (!dontUpdateLcd) {
 static unsigned char fan_rot = 0;
 
 u8g.setColorIndex(1);	// black on white
 
 // Symbols menu graphics, animated fan
 if ((blink % 2) &&  fanSpeed )	u8g.drawXBMP(9,1,STATUS_SCREENWIDTH,STATUS_SCREENHEIGHT,status_screen0);
 	else u8g.drawXBMP(9,1,STATUS_SCREENWIDTH,STATUS_SCREENHEIGHT,status_screen1);
 //u8g.drawXBMP(9,1,STATUS_SCREENWIDTH,STATUS_SCREENHEIGHT,status_screen1);
 
 //SD Card Symbol
 u8g.drawBox(42,42,8,7);
 u8g.drawBox(50,44,2,5);
 u8g.drawFrame(42,49,10,4);
 u8g.drawPixel(50,43);
 // Progress bar
 u8g.drawFrame(54,49,73,4);
 
 // SD Card Progress bar and clock
 u8g.setFont(FONT_STATUSMENU);
 
 if (IS_SD_PRINTING)
   {
	// Progress bar
	u8g.drawBox(55,50, (unsigned int)( (71 * card.percentDone())/100) ,2);
   }
    
 
 u8g.setPrintPos(82,47);
 if(starttime != 0)
    {
        uint16_t time = millis()/60000 - starttime/60000;

		u8g.print(itostr2(time/60));
		u8g.print(':');
		u8g.print(itostr2(time%60));
    }else{
			lcd_printPGM(PSTR("--:--"));
		 }
 
 
 // Extruder 1
 u8g.setFont(FONT_STATUSMENU);
 u8g.setPrintPos(8,6);
 u8g.print(itostr3(int(degTargetHotend(0) + 0.5)));
 lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
 u8g.setPrintPos(8,27);
 u8g.print(itostr3(int(degHotend(0) + 0.5)));
 lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
 if (!isHeatingHotend(0)) u8g.drawBox(13,17,2,2);
	else
		{
		 u8g.setColorIndex(0);	// white on black
		 u8g.drawLine(22,16,22,18);
		 u8g.setColorIndex(1);	// black on white
		}
 
 // Extruder 2
 u8g.setFont(FONT_STATUSMENU);
 #if EXTRUDERS > 1
 u8g.setPrintPos(44,6);
 u8g.print(itostr3(int(degTargetHotend(1) + 0.5)));
 lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
 u8g.setPrintPos(44,27);
 u8g.print(itostr3(int(degHotend(1) + 0.5)));
 lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
 if (!isHeatingHotend(1)) {
	u8g.setColorIndex(0);
	
	u8g.setColorIndex(1);
	}
	
 #else
 u8g.setPrintPos(44,27);
 u8g.print("---");
 #endif
 
 
 
 // Heatbed
 u8g.setFont(FONT_STATUSMENU);
 u8g.setPrintPos(70,6);
 u8g.print(itostr3(int(degTargetBed() + 0.5)));
 lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
 u8g.setPrintPos(70,27);
 u8g.print(itostr3(int(degBed() + 0.5)));
 lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
 if (!isHeatingBed()) {
	u8g.setColorIndex(0);
	u8g.drawBox(77,8,13,9);
	u8g.setColorIndex(1);
	}
 
 // Fan
 u8g.setFont(FONT_STATUSMENU);
 u8g.setPrintPos(104,27);
 #if defined(FAN_PIN) && FAN_PIN > -1
int percFan=int((fanSpeed*100)/MAXFAN+1);
if (percFan>100) percFan=100;
 u8g.print(itostr3(percFan));
 u8g.print("%");
 #else
 u8g.print("---");
 #endif
 
 u8g.setFont(FONT_STATUSMENU);
 u8g.setColorIndex(1);
 u8g.setPrintPos(2,37);
 if (clayMode) {
 u8g.print("Clay mode");
 
 } else {
	 
 u8g.print("FFF mode");	 
	 
 }
 u8g.drawBox(82,29,128,10);
 u8g.setColorIndex(0);	// white on black
 u8g.setPrintPos(83,37);
 u8g.print("Z");
 u8g.drawPixel(89,33);
 u8g.drawPixel(89,35);
 u8g.setPrintPos(91,37);
 u8g.print(ftostr31(current_position[Z_AXIS]));
 u8g.setColorIndex(1);	// black on white
 
 // Feedrate
 u8g.setFont(u8g_font_6x10_marlin);
 u8g.setPrintPos(3,49);
 u8g.print(LCD_STR_FEEDRATE[0]);
 u8g.setFont(FONT_STATUSMENU);
 u8g.setPrintPos(12,48);
 u8g.print(itostr3(feedmultiply));
 u8g.print('%');

 // Status line
 u8g.setFont(FONT_STATUSMENU);
 u8g.setPrintPos(0,61);
 u8g.print(lcd_status_message);
} else {
	
u8g.drawBitmapP(0,0,START_BMPBYTEWIDTH,START_BMPHEIGHT,logo_wasp);
u8g.setPrintPos(58,10);			
u8g.print(card.filename);				
u8g.setPrintPos(58,30);	
u8g.print(lcd_status_message);	
}
}

static void lcd_implementation_drawmenu_generic(uint8_t row, const char* pstr, char pre_char, char post_char)
{
    char c;
    
    uint8_t n = LCD_WIDTH - 1 - 2;
		
		if ((pre_char == '>') || (pre_char == LCD_STR_UPLEVEL[0] ))
		   {
			u8g.setColorIndex(1);		// black on white
			u8g.drawBox (0, row*DOG_CHAR_HEIGHT + 3, 128, DOG_CHAR_HEIGHT);
			u8g.setColorIndex(0);		// following text must be white on black
		   } else u8g.setColorIndex(1); // unmarked text is black on white
		
		u8g.setPrintPos(0 * DOG_CHAR_WIDTH, (row + 1) * DOG_CHAR_HEIGHT);
		if (pre_char != '>') u8g.print(pre_char); else u8g.print(' ');	// Row selector is obsolete


    while( (c = pgm_read_byte(pstr)) != '\0' )
    {
		u8g.print(c);
        pstr++;
        n--;
    }
    while(n--){
					u8g.print(' ');
		}
	   
		u8g.print(post_char);
		u8g.print(' ');
		u8g.setColorIndex(1);		// restore settings to black on white
}

static void lcd_implementation_drawmenu_setting_edit_generic(uint8_t row, const char* pstr, char pre_char, char* data)
{
    static unsigned int fkt_cnt = 0;
	char c;
    uint8_t n = LCD_WIDTH - 1 - 2 - strlen(data);
		
		u8g.setPrintPos(0 * DOG_CHAR_WIDTH, (row + 1) * DOG_CHAR_HEIGHT);
		u8g.print(pre_char);
	
    while( (c = pgm_read_byte(pstr)) != '\0' )
    {
			u8g.print(c);
		
        pstr++;
        n--;
    }
	
		u8g.print(':');

    while(n--){
					u8g.print(' ');
			  }

		u8g.print(data);
}

static void lcd_implementation_drawmenu_setting_edit_generic_P(uint8_t row, const char* pstr, char pre_char, const char* data)
{
    char c;
    uint8_t n= LCD_WIDTH - 1 - 2 - strlen_P(data);

		u8g.setPrintPos(0 * DOG_CHAR_WIDTH, (row + 1) * DOG_CHAR_HEIGHT);
		u8g.print(pre_char);
	
    while( (c = pgm_read_byte(pstr)) != '\0' )
    {
			u8g.print(c);
		
        pstr++;
        n--;
    }

		u8g.print(':');
	
    while(n--){
					u8g.print(' ');
			  }

		lcd_printPGM(data);
}

#define lcd_implementation_drawmenu_setting_edit_int3_selected(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_int3(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float3_selected(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float3(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float32_selected(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float32(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float5_selected(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float5(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float52_selected(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr52(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float52(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr52(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float51_selected(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr51(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float51(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr51(*(data)))
#define lcd_implementation_drawmenu_setting_edit_long5_selected(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_long5(row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_bool_selected(row, pstr, pstr2, data) lcd_implementation_drawmenu_setting_edit_generic_P(row, pstr, '>', (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))
#define lcd_implementation_drawmenu_setting_edit_bool(row, pstr, pstr2, data) lcd_implementation_drawmenu_setting_edit_generic_P(row, pstr, ' ', (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

//Add version for callback functions
#define lcd_implementation_drawmenu_setting_edit_callback_int3_selected(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_int3(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float3_selected(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float3(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float32_selected(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float32(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float5_selected(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float5(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float52_selected(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr52(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float52(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr52(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float51_selected(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr51(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float51(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr51(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_long5_selected(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, '>', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_long5(row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(row, pstr, ' ', ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_bool_selected(row, pstr, pstr2, data, callback) lcd_implementation_drawmenu_setting_edit_generic_P(row, pstr, '>', (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))
#define lcd_implementation_drawmenu_setting_edit_callback_bool(row, pstr, pstr2, data, callback) lcd_implementation_drawmenu_setting_edit_generic_P(row, pstr, ' ', (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

void lcd_implementation_drawedit(const char* pstr, char* value)
{
		u8g.setPrintPos(0 * DOG_CHAR_WIDTH_LARGE, (u8g.getHeight() - 1 - DOG_CHAR_HEIGHT_LARGE) - (1 * DOG_CHAR_HEIGHT_LARGE) - START_ROW );
		//u8g.setFont(u8g_font_9x18);
		lcd_printPGM(pstr);
		
		u8g.setPrintPos((14 - strlen(value)) * DOG_CHAR_WIDTH_LARGE, (u8g.getHeight() - 1 - DOG_CHAR_HEIGHT_LARGE) - (1 * DOG_CHAR_HEIGHT_LARGE) - START_ROW );
		u8g.print(value);
}

static void lcd_implementation_drawmenu_sdfile_selected(uint8_t row, const char* pstr, const char* filename, char* longFilename)
{
    char c;
    uint8_t n = LCD_WIDTH - 1;

    if (longFilename[0] != '\0')
    {
        filename = longFilename;
        longFilename[LCD_WIDTH-1] = '\0';
    }

		u8g.setColorIndex(1);		// black on white
		u8g.drawBox (0, row*DOG_CHAR_HEIGHT + 3, 128, DOG_CHAR_HEIGHT);
		u8g.setColorIndex(0);		// following text must be white on black
		u8g.setPrintPos(0 * DOG_CHAR_WIDTH, (row + 1) * DOG_CHAR_HEIGHT);
		u8g.print(' ');	// Indent by 1 char
	   
    while((c = *filename) != '\0')
    {
		u8g.print(c);
        filename++;
        n--;
    }
    while(n--){
					u8g.print(' ');
			   }
	u8g.setColorIndex(1);		// black on white
}

static void lcd_implementation_drawmenu_sdfile(uint8_t row, const char* pstr, const char* filename, char* longFilename)
{
    char c;
    uint8_t n = LCD_WIDTH - 1;

    if (longFilename[0] != '\0')
    {
        filename = longFilename;
        longFilename[LCD_WIDTH-1] = '\0';
    }

		u8g.setPrintPos(0 * DOG_CHAR_WIDTH, (row + 1) * DOG_CHAR_HEIGHT);
		u8g.print(' ');
		
while((c = *filename) != '\0')
    {
			u8g.print(c);
		
        filename++;
        n--;
    }
    while(n--){
					u8g.print(' ');
			   }

}

static void lcd_implementation_drawmenu_sddirectory_selected(uint8_t row, const char* pstr, const char* filename, char* longFilename)
{
    char c;
    uint8_t n = LCD_WIDTH - 2;
		
    if (longFilename[0] != '\0')
    {
        filename = longFilename;
        longFilename[LCD_WIDTH-2] = '\0';
    }
		u8g.setColorIndex(1);		// black on white
		u8g.drawBox (0, row*DOG_CHAR_HEIGHT + 3, 128, DOG_CHAR_HEIGHT);
		u8g.setColorIndex(0);		// following text must be white on black
		u8g.setPrintPos(0 * DOG_CHAR_WIDTH, (row + 1) * DOG_CHAR_HEIGHT);
		u8g.print(' ');	// Indent by 1 char
		u8g.print(LCD_STR_FOLDER[0]);		
	   
    while((c = *filename) != '\0')
    {
			u8g.print(c);
		
        filename++;
        n--;
    }
    while(n--){
					u8g.print(' ');
			   }
	u8g.setColorIndex(1);		// black on white
}

static void lcd_implementation_drawmenu_sddirectory(uint8_t row, const char* pstr, const char* filename, char* longFilename)
{
    char c;
    uint8_t n = LCD_WIDTH - 2;

    if (longFilename[0] != '\0')
    {
        filename = longFilename;
        longFilename[LCD_WIDTH-2] = '\0';
    }

		u8g.setPrintPos(0 * DOG_CHAR_WIDTH, (row + 1) * DOG_CHAR_HEIGHT);
		u8g.print(' ');
		u8g.print(LCD_STR_FOLDER[0]);

    while((c = *filename) != '\0')
    {
			u8g.print(c);
		
        filename++;
        n--;
    }
    while(n--){
					u8g.print(' ');
			   }
}

#define lcd_implementation_drawmenu_back_selected(row, pstr, data) lcd_implementation_drawmenu_generic(row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0])
#define lcd_implementation_drawmenu_back(row, pstr, data) lcd_implementation_drawmenu_generic(row, pstr, ' ', LCD_STR_UPLEVEL[0])
#define lcd_implementation_drawmenu_submenu_selected(row, pstr, data) lcd_implementation_drawmenu_generic(row, pstr, '>', LCD_STR_ARROW_RIGHT[0])
#define lcd_implementation_drawmenu_submenu(row, pstr, data) lcd_implementation_drawmenu_generic(row, pstr, ' ', LCD_STR_ARROW_RIGHT[0])
#define lcd_implementation_drawmenu_gcode_selected(row, pstr, gcode) lcd_implementation_drawmenu_generic(row, pstr, '>', ' ')
#define lcd_implementation_drawmenu_gcode(row, pstr, gcode) lcd_implementation_drawmenu_generic(row, pstr, ' ', ' ')
#define lcd_implementation_drawmenu_function_selected(row, pstr, data) lcd_implementation_drawmenu_generic(row, pstr, '>', ' ')
#define lcd_implementation_drawmenu_function(row, pstr, data) lcd_implementation_drawmenu_generic(row, pstr, ' ', ' ')

static void lcd_implementation_quick_feedback()
{


    SET_OUTPUT(BEEPER);
    for(int8_t i=0;i<15;i++)
    {
		WRITE(BEEPER,HIGH);
		delay(2);
		WRITE(BEEPER,LOW);
		delay(2);
    }

}
#endif//ULTRA_LCD_IMPLEMENTATION_DOGM_H


