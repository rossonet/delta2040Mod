#define START_BMPWIDTH 	60	//Width in pixels
#define START_BMPHEIGHT 	64	//Height in pixels
#define START_BMPBYTEWIDTH 	8	//Width in bytes
const unsigned char logo_wasp [] PROGMEM= {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x1F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xEF, 0xFC, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xE0, 0x00,
0x00, 0x00, 0x00, 0x01, 0xF1, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x3F, 0xF0, 0x00,
0x00, 0x00, 0x00, 0xC7, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x0F, 0xF8, 0x00,
0x00, 0x00, 0x18, 0x4C, 0x00, 0x07, 0xF8, 0x00, 0x00, 0x00, 0xF0, 0x88, 0x00, 0x03, 0xF8, 0x00,
0x00, 0x01, 0x41, 0x18, 0x78, 0x03, 0xFC, 0x00, 0x00, 0x02, 0x82, 0x11, 0xFE, 0x03, 0xFC, 0x00,
0x00, 0x03, 0x02, 0x33, 0xFF, 0x03, 0xFC, 0x00, 0x00, 0x03, 0x04, 0x27, 0xFF, 0x01, 0xF8, 0x00,
0x00, 0x07, 0x04, 0x27, 0xFF, 0x81, 0xF8, 0x00, 0x00, 0x1F, 0x88, 0x2F, 0xFF, 0x83, 0xF8, 0x00,
0x00, 0x3D, 0xD8, 0x0F, 0xFF, 0x83, 0xF8, 0x00, 0x00, 0x58, 0x70, 0x4F, 0xFF, 0x83, 0xF0, 0x00,
0x00, 0xD0, 0x38, 0x0F, 0xFF, 0x07, 0xE0, 0x00, 0x00, 0xB0, 0x3E, 0x2F, 0xFF, 0x0F, 0xE0, 0x00,
0x00, 0xB0, 0x27, 0xAF, 0xFE, 0x1F, 0xC0, 0x00, 0x00, 0xE0, 0x61, 0x93, 0xF8, 0x3F, 0x80, 0x00,
0x00, 0xE0, 0x40, 0xD8, 0x00, 0xFE, 0x07, 0x00, 0x00, 0xE0, 0xC1, 0x07, 0xFF, 0xF0, 0x1F, 0x00,
0x01, 0xF0, 0xC1, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x01, 0xF8, 0x81, 0x00, 0x00, 0x0F, 0xFF, 0x00,
0x02, 0xDC, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x02, 0x8F, 0x82, 0x07, 0xFF, 0xFF, 0xFF, 0x80,
0x02, 0x83, 0x82, 0x11, 0xFF, 0xFF, 0xFF, 0x80, 0x03, 0x81, 0xE2, 0x18, 0x7F, 0xFF, 0xFF, 0x80,
0x03, 0x81, 0x7C, 0x0E, 0x1F, 0xFF, 0xFF, 0x80, 0x03, 0x83, 0x1E, 0x0F, 0x87, 0xFF, 0xFF, 0x80,
0x01, 0x83, 0x07, 0xA7, 0xC0, 0xFF, 0xFF, 0x80, 0x01, 0xC3, 0x04, 0xB7, 0xF0, 0x3F, 0xFF, 0x00,
0x01, 0xE3, 0x00, 0x33, 0xFC, 0x03, 0xFF, 0x00, 0x01, 0xF2, 0x08, 0x39, 0xFF, 0x80, 0x3F, 0x00,
0x00, 0x9E, 0x08, 0x39, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x8F, 0x08, 0x3C, 0xFF, 0xFE, 0x00, 0x00,
0x00, 0x03, 0xE8, 0x3C, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x06, 0x7C, 0x3E, 0x7F, 0xFF, 0xFE, 0x00,
0x00, 0x46, 0x1F, 0xBE, 0x3F, 0xFF, 0xFE, 0x00, 0x00, 0x70, 0x10, 0xBF, 0x1F, 0xFF, 0xFC, 0x00,
0x00, 0x3E, 0x00, 0x3F, 0x9F, 0xFF, 0xFC, 0x00, 0x00, 0x3F, 0xF8, 0x3F, 0x8F, 0xFF, 0xF8, 0x00,
0x00, 0x1F, 0xFF, 0xFF, 0xC7, 0xFF, 0xF0, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE3, 0xFF, 0xF0, 0x00,
0x00, 0x07, 0xFF, 0xFF, 0xF1, 0xFF, 0xE0, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xF0, 0xFF, 0xC0, 0x00,
0x00, 0x03, 0xFF, 0xFF, 0xF8, 0x7F, 0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFC, 0x3F, 0x00, 0x00,
0x00, 0x00, 0x7F, 0xFF, 0xFE, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xC0, 0x00, 0x00,
0x00, 0x00, 0x00, 0x7F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#define STATUS_SCREENWIDTH 	115	//Width in pixels
#define STATUS_SCREENHEIGHT 	19	//Height in pixels
#define STATUS_SCREENBYTEWIDTH 	15	//Width in bytes

const unsigned char status_screen0[] U8G_PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00,
   0x00, 0x20, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x0f, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0xfe, 0x03,
   0x00, 0x80, 0xff, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0x1c, 0xc0, 0x01,
   0x00, 0xde, 0x03, 0x00, 0x80, 0xe3, 0x00, 0x00, 0x40, 0x08, 0x01, 0x00,
   0x3e, 0xe7, 0x03, 0x00, 0xce, 0x03, 0x00, 0x80, 0xef, 0x00, 0x00, 0x40,
   0x08, 0x01, 0x00, 0x7e, 0xf5, 0x03, 0x00, 0xde, 0x03, 0x00, 0x80, 0xe3,
   0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0x3e, 0xe7, 0x03, 0x00, 0xde, 0x03,
   0x00, 0x80, 0xfb, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00, 0x1c, 0xc0, 0x01,
   0x00, 0x8e, 0x03, 0x00, 0x80, 0xe3, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00,
   0x00, 0x02, 0x00, 0x00, 0xfe, 0x03, 0x00, 0x80, 0xff, 0x00, 0x00, 0x20,
   0x84, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x40, 0x08, 0x01, 0x00, 0x80, 0x0f, 0x00, 0x00, 0xf8, 0x00,
   0x00, 0x00, 0x3e, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0x80, 0x0f, 0x00,
   0x00, 0xf8, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x0f, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc,
   0xff, 0x0f, 0x00, 0x00, 0x07, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08,
   0x00, 0x00, 0xfc, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00 
/*
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x02, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x05, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x05, 0x40, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x71, 0x1c, 0x05, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf9, 0x3e, 0x05, 0x00, 0x02, 0x02,
   0x00, 0x80, 0x80, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0xf9, 0x3e, 0x05,
   0x00, 0x32, 0x02, 0x00, 0x80, 0x9c, 0x00, 0x00, 0x40, 0x08, 0x01, 0x00,
   0xf9, 0x3e, 0x05, 0x00, 0x22, 0x02, 0x00, 0x80, 0x90, 0x00, 0x00, 0x40,
   0x08, 0x01, 0x00, 0x71, 0x1d, 0x05, 0x00, 0x22, 0x02, 0x00, 0x80, 0x9c,
   0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0x81, 0x02, 0x05, 0x00, 0x22, 0x02,
   0x00, 0x80, 0x84, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00, 0x71, 0x1d, 0x05,
   0x00, 0x72, 0x02, 0x00, 0x80, 0x9c, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00,
   0xf9, 0x3e, 0x05, 0x00, 0x02, 0x02, 0x00, 0x80, 0x80, 0x00, 0x00, 0x20,
   0x84, 0x00, 0x00, 0xf9, 0x3e, 0x05, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x40, 0x08, 0x01, 0x00, 0xf9, 0x3e, 0x05, 0x00, 0xf8, 0x00,
   0x00, 0x00, 0x3e, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0x71, 0x1c, 0x05,
   0x00, 0xf8, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x05, 0x40, 0x05, 0x00, 0x70, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc,
   0xff, 0x0f, 0x00, 0x01, 0x00, 0x03, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08,
   0x00, 0x00, 0xfc, 0xff, 0x0f, 0x00, 0xff, 0xff, 0x01*/
};


const unsigned char status_screen1[] U8G_PROGMEM = { //AVR-GCC, WinAVR
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xf0, 0x00,
   0x00, 0x20, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xf8, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfc, 0xf8, 0x01, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xf8, 0x00, 0x00, 0xfe, 0x03,
   0x00, 0x80, 0xff, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0xf0, 0x78, 0x00,
   0x00, 0xde, 0x03, 0x00, 0x80, 0xe3, 0x00, 0x00, 0x40, 0x08, 0x01, 0x00,
   0x00, 0x02, 0x00, 0x00, 0xce, 0x03, 0x00, 0x80, 0xef, 0x00, 0x00, 0x40,
   0x08, 0x01, 0x00, 0x00, 0x05, 0x00, 0x00, 0xde, 0x03, 0x00, 0x80, 0xe3,
   0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0xde, 0x03,
   0x00, 0x80, 0xfb, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00, 0xf0, 0x78, 0x00,
   0x00, 0x8e, 0x03, 0x00, 0x80, 0xe3, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00,
   0xf8, 0xf8, 0x00, 0x00, 0xfe, 0x03, 0x00, 0x80, 0xff, 0x00, 0x00, 0x20,
   0x84, 0x00, 0x00, 0xfc, 0xf8, 0x01, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x40, 0x08, 0x01, 0x00, 0xfc, 0xf8, 0x01, 0x00, 0xf8, 0x00,
   0x00, 0x00, 0x3e, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0x78, 0xf0, 0x00,
   0x00, 0xf8, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x30, 0x60, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc,
   0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08,
   0x00, 0x00, 0xfc, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00
   /*
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x02, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x05, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x85, 0x43, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xc1, 0x07, 0x05, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc1, 0x07, 0x05, 0x00, 0x02, 0x02,
   0x00, 0x80, 0x80, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0xc1, 0x07, 0x05,
   0x00, 0x32, 0x02, 0x00, 0x80, 0x9c, 0x00, 0x00, 0x40, 0x08, 0x01, 0x00,
   0xb9, 0x3b, 0x05, 0x00, 0x22, 0x02, 0x00, 0x80, 0x90, 0x00, 0x00, 0x40,
   0x08, 0x01, 0x00, 0x7d, 0x7d, 0x05, 0x00, 0x22, 0x02, 0x00, 0x80, 0x9c,
   0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0xfd, 0x7e, 0x05, 0x00, 0x22, 0x02,
   0x00, 0x80, 0x84, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00, 0x7d, 0x7d, 0x05,
   0x00, 0x72, 0x02, 0x00, 0x80, 0x9c, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00,
   0xb9, 0x3b, 0x05, 0x00, 0x02, 0x02, 0x00, 0x80, 0x80, 0x00, 0x00, 0x20,
   0x84, 0x00, 0x00, 0xc1, 0x07, 0x05, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x40, 0x08, 0x01, 0x00, 0xc1, 0x07, 0x05, 0x00, 0xf8, 0x00,
   0x00, 0x00, 0x3e, 0x00, 0x00, 0x20, 0x84, 0x00, 0x00, 0xc1, 0x07, 0x05,
   0x00, 0xf8, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x85, 0x43, 0x05, 0x00, 0x70, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc,
   0xff, 0x0f, 0x00, 0x01, 0x00, 0x03, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08,
   0x00, 0x00, 0xfc, 0xff, 0x0f, 0x00, 0xff, 0xff, 0x01*/
};
#define START_RESWIDTH 	40	//Width in pixels
#define START_RESBMPHEIGHT 	64	//Height in pixels
#define START_RESBMPBYTEWIDTH 	5	//Width in bytes

const unsigned char resurrection [] U8G_PROGMEM= {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00,
   0x00, 0x00, 0x09, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x07, 0x00, 0x00,
   0xc0, 0xff, 0x0f, 0x00, 0x00, 0x60, 0x00, 0x18, 0x00, 0x00, 0x30, 0x00,
   0x30, 0x00, 0x00, 0x18, 0x18, 0x60, 0x00, 0x00, 0x18, 0x3c, 0x60, 0x00,
   0x00, 0x18, 0x18, 0x60, 0x00, 0x00, 0x18, 0x18, 0x60, 0x00, 0x00, 0x18,
   0x00, 0x60, 0x00, 0x00, 0x18, 0x00, 0x60, 0x00, 0x00, 0x18, 0x00, 0x60,
   0x00, 0x00, 0x18, 0x00, 0x60, 0x00, 0x16, 0x18, 0x00, 0x60, 0x00, 0xa8,
   0x18, 0x00, 0x60, 0x00, 0x53, 0x19, 0x00, 0x60, 0x00, 0x54, 0xd9, 0x5d,
   0x61, 0x80, 0x54, 0x19, 0x55, 0x61, 0x40, 0x54, 0xd9, 0x55, 0x65, 0xc0,
   0xfe, 0x59, 0x54, 0x67, 0x80, 0xff, 0xd9, 0x5d, 0x64, 0x80, 0xff, 0x18,
   0x00, 0x60, 0x00, 0x7f, 0x18, 0x00, 0x60, 0x00, 0x3e, 0x18, 0x00, 0x60,
   0x00, 0x3e, 0x18, 0x00, 0x60, 0x00, 0x3e, 0x18, 0x00, 0x60, 0x00, 0x3e,
   0x18, 0x00, 0x60, 0x00, 0x3e, 0x18, 0x00, 0x60, 0x70, 0x3e, 0xfe, 0xff,
   0x7f, 0xfc, 0x3e, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff,
   0x0f, 0x00, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#define ICO_WIDTH 32
#define ICO_HEIGHT 32
static unsigned char sd_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x03, 0x00, 0xbc, 0x6d, 0x03,
   0x00, 0xbe, 0x6d, 0x03, 0x00, 0xbf, 0x6d, 0x03, 0x80, 0xbf, 0x6d, 0x03,
   0xc0, 0xbf, 0x6d, 0x03, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03,
   0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03,
   0xc0, 0xff, 0xff, 0x03, 0xc0, 0x87, 0xe3, 0x03, 0xc0, 0xf7, 0xdb, 0x03,
   0xc0, 0x87, 0xdb, 0x03, 0xc0, 0xbf, 0xdb, 0x03, 0xc0, 0x87, 0xe3, 0x03,
   0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03,
   0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03,
   0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
static unsigned char sd_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0xfc, 0xff, 0x43, 0x92, 0xfc,
   0xff, 0x41, 0x92, 0xfc, 0xff, 0x40, 0x92, 0xfc, 0x7f, 0x40, 0x92, 0xfc,
   0x3f, 0x40, 0x92, 0xfc, 0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc,
   0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc,
   0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x78, 0x1c, 0xfc, 0x3f, 0x08, 0x24, 0xfc,
   0x3f, 0x78, 0x24, 0xfc, 0x3f, 0x40, 0x24, 0xfc, 0x3f, 0x78, 0x1c, 0xfc,
   0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc,
   0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc,
   0x3f, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f  };
static unsigned char return_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
   0x1c, 0x00, 0x00, 0x00, 0xfe, 0x3f, 0x00, 0x00, 0xff, 0x3f, 0x00, 0x00,
   0xfe, 0x3f, 0x00, 0x00, 0x1c, 0x30, 0x00, 0x00, 0x18, 0x30, 0x00, 0x00,
   0x10, 0x30, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00,
   0x00, 0x30, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x21, 0x4e, 0x01, 0x80, 0x52, 0x42, 0x01, 0x80, 0x51, 0xc2, 0x00,
   0x80, 0x72, 0x42, 0x01, 0x80, 0x51, 0x4e, 0x01, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char return_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xef, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xff,
   0xe3, 0xff, 0xff, 0xff, 0x01, 0xc0, 0xff, 0xff, 0x00, 0xc0, 0xff, 0xff,
   0x01, 0xc0, 0xff, 0xff, 0xe3, 0xcf, 0xff, 0xff, 0xe7, 0xcf, 0xff, 0xff,
   0xef, 0xcf, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff,
   0xff, 0xcf, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x7f, 0xde, 0xb1, 0xfe, 0x7f, 0xad, 0xbd, 0xfe, 0x7f, 0xae, 0x3d, 0xff,
   0x7f, 0x8d, 0xbd, 0xfe, 0x7f, 0xae, 0xb1, 0xfe, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f };
static unsigned char prepare_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x18, 0xc0, 0x3f, 0x00, 0x78, 0xc0, 0xfe, 0x00, 0xf0, 0x00, 0xf8, 0x03,
   0xf0, 0x00, 0xe0, 0x07, 0xc0, 0x01, 0xe0, 0x0f, 0x80, 0x03, 0xe0, 0x0f,
   0x00, 0x07, 0xf0, 0x0f, 0x00, 0x0e, 0x78, 0x7c, 0x00, 0x1c, 0x3c, 0xf8,
   0x00, 0x38, 0x1e, 0xf0, 0x00, 0x70, 0x02, 0x70, 0x00, 0xe0, 0x06, 0x00,
   0x00, 0xc0, 0x0f, 0x00, 0x00, 0xb0, 0x1f, 0x00, 0x00, 0xb8, 0x3f, 0x00,
   0x00, 0xde, 0x7f, 0x00, 0x00, 0xbf, 0xff, 0x00, 0x80, 0x3f, 0xff, 0x01,
   0xc0, 0x1f, 0xfe, 0x03, 0xe0, 0x1f, 0xfc, 0x07, 0xf0, 0x0f, 0xf8, 0x0f,
   0xf8, 0x07, 0xf0, 0x1f, 0xfc, 0x03, 0xe0, 0x3f, 0xfe, 0x01, 0xc0, 0x3f,
   0xfe, 0x00, 0x80, 0x3f, 0x7c, 0x00, 0x00, 0x1f, 0x38, 0x00, 0x00, 0x0e,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char prepare_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xe7, 0x3f, 0xc0, 0xff, 0x87, 0x3f, 0x01, 0xff, 0x0f, 0xff, 0x07, 0xfc,
   0x0f, 0xff, 0x1f, 0xf8, 0x3f, 0xfe, 0x1f, 0xf0, 0x7f, 0xfc, 0x1f, 0xf0,
   0xff, 0xf8, 0x0f, 0xf0, 0xff, 0xf1, 0x87, 0x83, 0xff, 0xe3, 0xc3, 0x07,
   0xff, 0xc7, 0xe1, 0x0f, 0xff, 0x8f, 0xfd, 0x8f, 0xff, 0x1f, 0xf9, 0xff,
   0xff, 0x3f, 0xf0, 0xff, 0xff, 0x4f, 0xe0, 0xff, 0xff, 0x47, 0xc0, 0xff,
   0xff, 0x21, 0x80, 0xff, 0xff, 0x40, 0x00, 0xff, 0x7f, 0xc0, 0x00, 0xfe,
   0x3f, 0xe0, 0x01, 0xfc, 0x1f, 0xe0, 0x03, 0xf8, 0x0f, 0xf0, 0x07, 0xf0,
   0x07, 0xf8, 0x0f, 0xe0, 0x03, 0xfc, 0x1f, 0xc0, 0x01, 0xfe, 0x3f, 0xc0,
   0x01, 0xff, 0x7f, 0xc0, 0x83, 0xff, 0xff, 0xe0, 0xc7, 0xff, 0xff, 0xf1,
   0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f };

static unsigned char level_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xfe, 0x3e, 0x7e, 0x7f, 0xfe, 0xfe, 0x7f, 0x7f,
   0xfe, 0x00, 0x00, 0x7f, 0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f,
   0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f, 0xde, 0xb6, 0x6d, 0x7b,
   0xde, 0xb6, 0x6d, 0x7b, 0xfe, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x40, 0x5c, 0x5d, 0x00, 0x40, 0x44, 0x45, 0x00,
   0x40, 0x4c, 0x4d, 0x00, 0xc0, 0x9d, 0xdc, 0x01, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static unsigned char level_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x01, 0xc1, 0x81, 0x80, 0x01, 0x01, 0x80, 0x80,
   0x01, 0xff, 0xff, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x21, 0x49, 0x92, 0x84,
   0x21, 0x49, 0x92, 0x84, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xbf, 0xa3, 0xa2, 0xff, 0xbf, 0xbb, 0xba, 0xff,
   0xbf, 0xb3, 0xb2, 0xff, 0x3f, 0x62, 0x23, 0xfe, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f };
static unsigned char filchrg_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1f, 0x00, 0x00, 0x40, 0x0e, 0x00,
   0x04, 0x40, 0x04, 0x04, 0x08, 0x40, 0x00, 0x02, 0x10, 0x40, 0x1f, 0x21,
   0x20, 0x40, 0x8e, 0x10, 0x42, 0x40, 0x44, 0x08, 0x84, 0x40, 0x20, 0x04,
   0x08, 0x41, 0x10, 0x02, 0x10, 0x42, 0x08, 0x01, 0x20, 0x44, 0x84, 0x00,
   0x40, 0xe8, 0x42, 0x00, 0x80, 0xe4, 0x22, 0x00, 0x00, 0xe3, 0x14, 0x00,
   0x00, 0xfe, 0x0f, 0x00, 0x00, 0xf8, 0x03, 0x00, 0x00, 0xf8, 0x03, 0x00,
   0x00, 0xf8, 0x03, 0x00, 0x00, 0xf8, 0x03, 0x00, 0x00, 0xf8, 0x03, 0x00,
   0x00, 0xf8, 0x03, 0x00, 0x00, 0xf0, 0x01, 0x00, 0x00, 0xe0, 0x00, 0x00,
   0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00,
   0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0xc0, 0xc7, 0x03, 0x00,
   0x70, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char filchrg_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xbf, 0xe0, 0xff, 0xff, 0xbf, 0xf1, 0xff,
   0xfb, 0xbf, 0xfb, 0xfb, 0xf7, 0xbf, 0xff, 0xfd, 0xef, 0xbf, 0xe0, 0xde,
   0xdf, 0xbf, 0x71, 0xef, 0xbd, 0xbf, 0xbb, 0xf7, 0x7b, 0xbf, 0xdf, 0xfb,
   0xf7, 0xbe, 0xef, 0xfd, 0xef, 0xbd, 0xf7, 0xfe, 0xdf, 0xbb, 0x7b, 0xff,
   0xbf, 0x17, 0xbd, 0xff, 0x7f, 0x1b, 0xdd, 0xff, 0xff, 0x1c, 0xeb, 0xff,
   0xff, 0x01, 0xf0, 0xff, 0xff, 0x07, 0xfc, 0xff, 0xff, 0x07, 0xfc, 0xff,
   0xff, 0x07, 0xfc, 0xff, 0xff, 0x07, 0xfc, 0xff, 0xff, 0x07, 0xfc, 0xff,
   0xff, 0x07, 0xfc, 0xff, 0xff, 0x0f, 0xfe, 0xff, 0xff, 0x1f, 0xff, 0xff,
   0xff, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xff,
   0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfd, 0xff, 0x3f, 0x38, 0xfc, 0xff,
   0x8f, 0x83, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f };

static unsigned char advanced_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x13, 0x00, 0x00, 0xf8, 0x3f,
   0x00, 0x00, 0xfc, 0x7f, 0x00, 0x00, 0x78, 0x3c, 0x00, 0x00, 0x38, 0x38,
   0x00, 0x07, 0x1c, 0x70, 0x00, 0x07, 0x1c, 0x70, 0x98, 0xdf, 0x1d, 0x70,
   0xfc, 0xff, 0x3b, 0x38, 0xfc, 0xff, 0x7b, 0x3c, 0x78, 0xe0, 0xfd, 0x7f,
   0x38, 0xc0, 0xfb, 0x3f, 0x18, 0x80, 0x93, 0x13, 0x1c, 0x80, 0x03, 0x00,
   0x1f, 0x80, 0x0f, 0x00, 0x1f, 0x80, 0x0f, 0x00, 0x1f, 0x80, 0x0f, 0x00,
   0x1c, 0x80, 0x03, 0x00, 0x38, 0xc0, 0x03, 0x00, 0x78, 0xe0, 0x01, 0x00,
   0xfc, 0xff, 0x03, 0x00, 0xfc, 0xff, 0x03, 0x00, 0x98, 0xcf, 0x01, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char advanced_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x6f, 0xec, 0xff, 0xff, 0x07, 0xc0,
   0xff, 0xff, 0x03, 0x80, 0xff, 0xff, 0x87, 0xc3, 0xff, 0xff, 0xc7, 0xc7,
   0xff, 0xf8, 0xe3, 0x8f, 0xff, 0xf8, 0xe3, 0x8f, 0x67, 0x20, 0xe2, 0x8f,
   0x03, 0x00, 0xc4, 0xc7, 0x03, 0x00, 0x84, 0xc3, 0x87, 0x1f, 0x02, 0x80,
   0xc7, 0x3f, 0x04, 0xc0, 0xe7, 0x7f, 0x6c, 0xec, 0xe3, 0x7f, 0xfc, 0xff,
   0xe0, 0x7f, 0xf0, 0xff, 0xe0, 0x7f, 0xf0, 0xff, 0xe0, 0x7f, 0xf0, 0xff,
   0xe3, 0x7f, 0xfc, 0xff, 0xc7, 0x3f, 0xfc, 0xff, 0x87, 0x1f, 0xfe, 0xff,
   0x03, 0x00, 0xfc, 0xff, 0x03, 0x00, 0xfc, 0xff, 0x67, 0x30, 0xfe, 0xff,
   0xff, 0xf8, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f };
static unsigned char info_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0xe0, 0x1f, 0x00,
   0x00, 0xe0, 0x3f, 0x00, 0x00, 0x20, 0x3c, 0x00, 0x00, 0x10, 0x38, 0x00,
   0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x0e, 0x00,
   0x00, 0x00, 0x07, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xc0, 0x00, 0x00,
   0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00,
   0x00, 0x70, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char info_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xf0, 0xff, 0xff, 0x1f, 0xe0, 0xff,
   0xff, 0x1f, 0xc0, 0xff, 0xff, 0xdf, 0xc3, 0xff, 0xff, 0xef, 0xc7, 0xff,
   0xff, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xf1, 0xff,
   0xff, 0xff, 0xf8, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x3f, 0xff, 0xff,
   0xff, 0xdf, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xff,
   0xff, 0x8f, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f };
static unsigned char resurr_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x0f, 0x00, 0x00, 0x10, 0x18, 0x00, 0x00, 0x08, 0x30,
   0x00, 0x00, 0x84, 0x60, 0x00, 0x01, 0xc4, 0x61, 0x80, 0x0a, 0x84, 0x60,
   0x00, 0x55, 0x84, 0x60, 0x00, 0xaa, 0x04, 0x60, 0x00, 0xaa, 0x04, 0x60,
   0x40, 0xfe, 0x04, 0x60, 0xc0, 0xfe, 0x04, 0x60, 0x80, 0x7f, 0x04, 0x60,
   0x00, 0x3f, 0x04, 0x60, 0x00, 0x3e, 0x04, 0x60, 0x00, 0x3e, 0x04, 0x60,
   0x00, 0x3e, 0x04, 0x60, 0x78, 0x3e, 0xff, 0x7f, 0xfe, 0xfe, 0xff, 0x7f,
   0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x30, 0x77, 0x35, 0x13, 0x50, 0x11, 0x55, 0x15,
   0x30, 0x63, 0x35, 0x03, 0x50, 0x77, 0x57, 0x15, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char resurr_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0x1f, 0xf0, 0xff, 0xff, 0xef, 0xe7, 0xff, 0xff, 0xf7, 0xcf,
   0xff, 0xff, 0x7b, 0x9f, 0xff, 0xfe, 0x3b, 0x9e, 0x7f, 0xf5, 0x7b, 0x9f,
   0xff, 0xaa, 0x7b, 0x9f, 0xff, 0x55, 0xfb, 0x9f, 0xff, 0x55, 0xfb, 0x9f,
   0xbf, 0x01, 0xfb, 0x9f, 0x3f, 0x01, 0xfb, 0x9f, 0x7f, 0x80, 0xfb, 0x9f,
   0xff, 0xc0, 0xfb, 0x9f, 0xff, 0xc1, 0xfb, 0x9f, 0xff, 0xc1, 0xfb, 0x9f,
   0xff, 0xc1, 0xfb, 0x9f, 0x87, 0xc1, 0x00, 0x80, 0x01, 0x01, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xcf, 0x88, 0xca, 0xec, 0xaf, 0xee, 0xaa, 0xea,
   0xcf, 0x9c, 0xca, 0xfc, 0xaf, 0x88, 0xa8, 0xea, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x7f };
static unsigned char home_bits[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0xe0, 0x1b, 0x00,
   0x00, 0xf0, 0x1f, 0x00, 0x00, 0xf8, 0x1f, 0x00, 0x00, 0xfc, 0x1f, 0x00,
   0x00, 0xfe, 0x3f, 0x00, 0x00, 0xff, 0x7f, 0x00, 0x80, 0xff, 0xff, 0x00,
   0x80, 0xff, 0xff, 0x00, 0x00, 0xff, 0x7f, 0x00, 0x00, 0xff, 0x7f, 0x00,
   0x00, 0xe7, 0x73, 0x00, 0x00, 0xe7, 0x73, 0x00, 0x00, 0xff, 0x7f, 0x00,
   0x00, 0xff, 0x7f, 0x00, 0x00, 0xff, 0x7f, 0x00, 0x00, 0x7f, 0x7f, 0x00,
   0x00, 0x3f, 0x7e, 0x00, 0x00, 0x3f, 0x7e, 0x00, 0x00, 0x3f, 0x7e, 0x00,
   0x00, 0x80, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0xf0, 0x00, 0x00,
   0x00, 0x78, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00,
   0x00, 0xe0, 0x01, 0x00, 0x00, 0xc0, 0x03, 0x00 };
static unsigned char home_sel_bits[] U8G_PROGMEM= {
   0xfe, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0x7f, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x1f, 0xe4, 0xff,
   0xff, 0x0f, 0xe0, 0xff, 0xff, 0x07, 0xe0, 0xff, 0xff, 0x03, 0xe0, 0xff,
   0xff, 0x01, 0xc0, 0xff, 0xff, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0xff,
   0x7f, 0x00, 0x00, 0xff, 0xff, 0x00, 0x80, 0xff, 0xff, 0x00, 0x80, 0xff,
   0xff, 0x18, 0x8c, 0xff, 0xff, 0x18, 0x8c, 0xff, 0xff, 0x00, 0x80, 0xff,
   0xff, 0x00, 0x80, 0xff, 0xff, 0x00, 0x80, 0xff, 0xff, 0x80, 0x80, 0xff,
   0xff, 0xc0, 0x81, 0xff, 0xff, 0xc0, 0x81, 0xff, 0xff, 0xc0, 0x81, 0xff,
   0xff, 0x7f, 0xff, 0xff, 0xff, 0x1f, 0xfe, 0xff, 0xff, 0x0f, 0xff, 0xff,
   0xff, 0x87, 0xff, 0xff, 0xff, 0x87, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff,
   0xff, 0x1f, 0xfe, 0xff, 0xfe, 0x3f, 0xfc, 0x7f };
static unsigned char no_bits[] U8G_PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x03, 0x00, 0xbc, 0x6d, 0x03,
   0x00, 0xbe, 0x6d, 0x03, 0x00, 0xbf, 0x6d, 0x03, 0x80, 0xbf, 0x6d, 0x03,
   0xc0, 0xbf, 0x6d, 0x03, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xfe, 0x03,
   0xc0, 0x7f, 0xfc, 0x03, 0xc0, 0x3f, 0xf8, 0x03, 0xc0, 0x3f, 0xf8, 0x03,
   0xc0, 0x3f, 0xf8, 0x03, 0xc0, 0x3f, 0xf8, 0x03, 0xc0, 0x3f, 0xf8, 0x03,
   0xc0, 0x7f, 0xfc, 0x03, 0xc0, 0x7f, 0xfc, 0x03, 0xc0, 0x7f, 0xfc, 0x03,
   0xc0, 0xff, 0xfe, 0x03, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03,
   0xc0, 0xff, 0xfe, 0x03, 0xc0, 0x7f, 0xfc, 0x03, 0xc0, 0x7f, 0xfc, 0x03,
   0xc0, 0xff, 0xfe, 0x03, 0xc0, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char fzs_bits[] U8G_PROGMEM = {
   0xfe, 0xff, 0xff, 0x7f, 0x03, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x81, 0x07, 0x00, 0x80,
   0xc1, 0xff, 0x1f, 0x80, 0xc1, 0xf8, 0x1f, 0x80, 0x61, 0x00, 0x0c, 0x80,
   0xf9, 0x03, 0x86, 0x9f, 0xf9, 0x03, 0xc3, 0x9f, 0x61, 0xc0, 0x61, 0x80,
   0x71, 0xe0, 0x60, 0x80, 0x31, 0x70, 0xe0, 0x83, 0x31, 0x18, 0x80, 0x8f,
   0x31, 0x0c, 0x00, 0x8c, 0x31, 0x06, 0x10, 0x8c, 0x19, 0xff, 0xf7, 0x87,
   0x19, 0xff, 0xe7, 0x83, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
   0x03, 0x00, 0x00, 0xc0, 0xfe, 0xff, 0xff, 0x7f };
static unsigned char fzs_sel_bits[] U8G_PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x7f,
   0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f,
   0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f, 0x7e, 0xf8, 0xff, 0x7f,
   0x3e, 0x00, 0xe0, 0x7f, 0x3e, 0x07, 0xe0, 0x7f, 0x9e, 0xff, 0xf3, 0x7f,
   0x06, 0xfc, 0x79, 0x60, 0x06, 0xfc, 0x3c, 0x60, 0x9e, 0x3f, 0x9e, 0x7f,
   0x8e, 0x1f, 0x9f, 0x7f, 0xce, 0x8f, 0x1f, 0x7c, 0xce, 0xe7, 0x7f, 0x70,
   0xce, 0xf3, 0xff, 0x73, 0xce, 0xf9, 0xef, 0x73, 0xe6, 0x00, 0x08, 0x78,
   0xe6, 0x00, 0x18, 0x7c, 0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f,
   0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f,
   0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0x7f,
   0xfc, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00 };
static unsigned char stopsave_bits[] U8G_PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0xfc, 0x3f, 0x00,
   0x00, 0xff, 0xff, 0x00, 0x80, 0x1f, 0xfc, 0x01, 0xe0, 0x07, 0xe0, 0x03,
   0xe0, 0x01, 0x80, 0x07, 0xf0, 0x00, 0x00, 0x0f, 0x78, 0x00, 0x00, 0x1e,
   0x38, 0x00, 0x00, 0x1c, 0x3c, 0x0c, 0x7f, 0x3c, 0x1c, 0x0c, 0x7f, 0x38,
   0x1c, 0x0c, 0x61, 0x38, 0x0e, 0x0c, 0x61, 0x70, 0x0e, 0x0c, 0x61, 0x70,
   0x0e, 0xcc, 0x67, 0x70, 0x0e, 0x8c, 0x63, 0x70, 0x0e, 0x0c, 0x61, 0x70,
   0x1e, 0x0c, 0x60, 0x78, 0x1c, 0x0c, 0x60, 0x38, 0x1c, 0xfc, 0x7f, 0x38,
   0x3c, 0xfc, 0x7f, 0x3c, 0x38, 0x00, 0x00, 0x1c, 0x78, 0x00, 0x00, 0x1e,
   0xf0, 0x00, 0x00, 0x0f, 0xe0, 0x01, 0x80, 0x07, 0xe0, 0x07, 0xe0, 0x03,
   0x80, 0x3f, 0xf8, 0x03, 0x00, 0xff, 0xff, 0x00, 0x00, 0xfc, 0x3f, 0x00,
   0x00, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char stopsave_sel_bits[] U8G_PROGMEM = {
   0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0xf8, 0xff, 0xff, 0x03, 0xc0, 0xff,
   0xff, 0x00, 0x00, 0xff, 0x7f, 0xe0, 0x03, 0xfe, 0x1f, 0xf8, 0x1f, 0xfc,
   0x1f, 0xfe, 0x7f, 0xf8, 0x0f, 0xff, 0xff, 0xf0, 0x87, 0xff, 0xff, 0xe1,
   0xc7, 0xff, 0xff, 0xe3, 0xc3, 0xf3, 0x80, 0xc3, 0xe3, 0xf3, 0x80, 0xc7,
   0xe3, 0xf3, 0x9e, 0xc7, 0xf1, 0xf3, 0x9e, 0x8f, 0xf1, 0xf3, 0x9e, 0x8f,
   0xf1, 0x33, 0x98, 0x8f, 0xf1, 0x73, 0x9c, 0x8f, 0xf1, 0xf3, 0x9e, 0x8f,
   0xe1, 0xf3, 0x9f, 0x87, 0xe3, 0xf3, 0x9f, 0xc7, 0xe3, 0x03, 0x80, 0xc7,
   0xc3, 0x03, 0x80, 0xc3, 0xc7, 0xff, 0xff, 0xe3, 0x87, 0xff, 0xff, 0xe1,
   0x0f, 0xff, 0xff, 0xf0, 0x1f, 0xfe, 0x7f, 0xf8, 0x1f, 0xf8, 0x1f, 0xfc,
   0x7f, 0xc0, 0x07, 0xfc, 0xff, 0x00, 0x00, 0xff, 0xff, 0x03, 0xc0, 0xff,
   0xff, 0x1f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff };
/*   
#define BACKGROUND_WIDTH 128
#define BACKGROUND_HEIGHT 64
static unsigned char background_menu[] U8G_PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xc0, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0xc0, 0x21,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x19, 0x00, 0x00, 0x80,
   0x01, 0x00, 0xa0, 0x22, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0,
   0x3f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0x20, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xf8, 0x7f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0x20,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x7f, 0x06, 0x00, 0x80,
   0x01, 0x00, 0x80, 0x20, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
   0xff, 0x0f, 0x00, 0x80, 0x01, 0x00, 0x80, 0x20, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0xff, 0xff, 0x0f, 0x00, 0x80, 0x01, 0x00, 0x80, 0x20,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xef, 0x0f, 0x00, 0x80,
   0x01, 0x00, 0x80, 0x20, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff,
   0xf7, 0x07, 0x00, 0x80, 0x01, 0x00, 0x80, 0x20, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf7, 0x67, 0xfe, 0x07, 0x00, 0x80, 0x01, 0x00, 0x80, 0xe0,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0xfb, 0x07, 0xfe, 0x0f, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xe0, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x03,
   0xfe, 0x0f, 0x00, 0x80, 0x01, 0x00, 0x80, 0xe0, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x80, 0xff, 0x01, 0xfc, 0x07, 0x00, 0x80, 0x01, 0x00, 0x80, 0xe0,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xfc, 0x03, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xe0, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00,
   0xff, 0x01, 0x00, 0x80, 0x01, 0x00, 0x80, 0xe0, 0x01, 0x00, 0xe2, 0x01,
   0x00, 0x00, 0x3e, 0x80, 0xff, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xe0,
   0x01, 0x00, 0x15, 0x00, 0x00, 0x00, 0x18, 0xe0, 0xff, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xe0, 0x01, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0xb8,
   0x7f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xe0, 0x01, 0x00, 0x10, 0x00,
   0x00, 0x00, 0x00, 0xdc, 0x3f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xf8,
   0x07, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x1f, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xfc,
   0x0f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0xe0, 0x01,
   0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xfc,
   0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
   0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xf8,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd8, 0x1b, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
   0x1f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xfc, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfd, 0xff, 0x01, 0x00, 0x80, 0x01, 0x00, 0x00, 0xde,
   0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x01, 0x00, 0x80,
   0x01, 0x00, 0x00, 0xdf, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
   0xff, 0x01, 0x00, 0x80, 0x01, 0x00, 0x80, 0xdf, 0x76, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x80, 0xff, 0xff, 0x01, 0x00, 0x80, 0x01, 0x00, 0x80, 0xdf,
   0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x07, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xdf, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff,
   0xff, 0x07, 0x00, 0x80, 0x01, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0x3f, 0xfc, 0x03, 0x00, 0x80, 0x01, 0x00, 0x80, 0xff,
   0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x1f, 0xf8, 0x07, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x1f,
   0xf8, 0x0f, 0x00, 0x80, 0x01, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x1f, 0xf8, 0x0f, 0x00, 0x80, 0x01, 0x00, 0x80, 0xff,
   0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x3f, 0xf8, 0x07, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f,
   0xfc, 0x03, 0x00, 0x80, 0x01, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0xff, 0xff, 0x07, 0x00, 0x80, 0x01, 0x00, 0x80, 0xc3,
   0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x07, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xfb, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff,
   0xff, 0x01, 0x00, 0x80, 0x01, 0x00, 0x80, 0xfb, 0x6e, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x66, 0xff, 0xff, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xc3,
   0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0xff, 0xff, 0x01, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xdf, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfd,
   0xbf, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xdf, 0x76, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0xc7, 0xfb, 0x1f, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xc3,
   0x78, 0x00, 0x00, 0x00, 0x00, 0x80, 0x83, 0x9b, 0x19, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x80, 0x81, 0x81,
   0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x81, 0x01, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xff,
   0x7f, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xc3, 0x03, 0x00, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xe7, 0x03,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00 };
   */
