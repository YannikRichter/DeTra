#define LoRa_Logo_width 20
#define LoRa_Logo_height 20

#define HSPF_LOGO_width 128
#define HSPF_LOGO_height 40

#define BT_width 8
#define BT_height 10

#define BAT_width 20
#define BAT_height 9

#define WIFI_width 14
#define WIFI_height 8

const unsigned char LoRa_Logo_bits[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 
  0xC0, 0x3F, 0x00, 0xE0, 0x7F, 0x00, 0xE0, 0x7F, 0x00, 0xFC, 0xF9, 0x00, 
  0xFE, 0xF0, 0x01, 0x7E, 0xE0, 0x07, 0x3F, 0xC0, 0x07, 0xFF, 0xF0, 0x0F, 
  0xFF, 0xF0, 0x0F, 0xFF, 0xF0, 0x0F, 0xFE, 0xFF, 0x07, 0xFC, 0xFF, 0x07, 
  0xF8, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char HSPF_LOGO_bits[] PROGMEM = {
0x03, 0x83, 0x3F, 0x00, 0xFF, 0xF0, 0x0F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0xC3, 0x7F, 0x00, 0xFF, 0xF1, 0x0F, 0x00, 
  0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0xE3, 0xF0, 0x00, 
  0xC3, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 
  0x03, 0x63, 0xC0, 0x00, 0x83, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x80, 0x0F, 0x03, 0xE3, 0x00, 0x00, 0x83, 0x33, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x03, 0x03, 0xE3, 0x01, 0x00, 
  0x83, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 
  0xFF, 0xC3, 0x1F, 0x00, 0xC3, 0xF1, 0x07, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0x7F, 0x00, 0xFF, 0xF0, 0x07, 0x00, 
  0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x03, 0xFC, 0x00, 
  0x7F, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x3C, 
  0x03, 0x03, 0xE0, 0x00, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x3E, 0x00, 0x0F, 0x03, 0x03, 0xC0, 0x00, 0x03, 0x30, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0xC0, 0x07, 0x03, 0x63, 0xE0, 0x00, 
  0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x01, 0xF8, 0x00, 
  0x03, 0xE3, 0xF1, 0x00, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x7C, 0x00, 0x3C, 0x00, 0x03, 0xC3, 0x7F, 0x00, 0x03, 0x30, 0x00, 0x00, 
  0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x83, 0x3F, 0x00, 
  0x03, 0x30, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 
  0x00, 0x3C, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x1F, 0x80, 0x0F, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x07, 0xE0, 0x03, 0xF0, 0x01, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 
  0xF8, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x3C, 0x00, 0x1E, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0F, 0xC0, 0x07, 0xE0, 0x03, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x03, 0xF0, 
  0x01, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x78, 0x00, 0x3C, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x80, 0x0F, 0xC0, 0x07, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03, 0xE0, 0x01, 
  0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xF0, 0x00, 0x7C, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x1F, 0x80, 0x0F, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03, 0xE0, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xF8, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0xC0, 0x07, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xF0, 0x01, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0F, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xE0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00,
  };

const unsigned char BT_bits[] PROGMEM = {
  0x18, 0x28, 0x4A, 0x2C, 0x18, 0x2C, 0x4A, 0x28, 0x18, 0x00,
  };

unsigned char BAT_bits[] PROGMEM = {
  0xFC, 0xFF, 0x0F, 0x04, 0x00, 0x08, 0xF7, 0xDE, 0x0B, 0xF1, 0xDE, 0x0B, 
  0xF1, 0xDE, 0x0B, 0xF1, 0xDE, 0x0B, 0xF7, 0xDE, 0x0B, 0x04, 0x00, 0x08, 
  0xFC, 0xFF, 0x0F,
  };

const unsigned char BAT_empty[] PROGMEM = {
  0xFC, 0xFF, 0x0F, 0x04, 0x00, 0x08, 0x07, 0x00, 0x08, 0x01, 0x00, 0x08, 
  0x01, 0x00, 0x08, 0x01, 0x00, 0x08, 0x07, 0x00, 0x08, 0x04, 0x00, 0x08, 
  0xFC, 0xFF, 0x0F,
  };

const unsigned char BAT_33[] PROGMEM = {
  0xFC, 0xFF, 0x0F, 0x04, 0x00, 0x08, 0x07, 0xC0, 0x0B, 0x01, 0xC0, 0x0B, 
  0x01, 0xC0, 0x0B, 0x01, 0xC0, 0x0B, 0x07, 0xC0, 0x0B, 0x04, 0x00, 0x08, 
  0xFC, 0xFF, 0x0F,
  };
  
const unsigned char BAT_66[] PROGMEM = {
  0xFC, 0xFF, 0x0F, 0x04, 0x00, 0x08, 0x07, 0xDE, 0x0B, 0x01, 0xDE, 0x0B, 
  0x01, 0xDE, 0x0B, 0x01, 0xDE, 0x0B, 0x07, 0xDE, 0x0B, 0x04, 0x00, 0x08, 
  0xFC, 0xFF, 0x0F,
  };
  
const unsigned char BAT_full[] PROGMEM = {
  0xFC, 0xFF, 0x0F, 0x04, 0x00, 0x08, 0xF7, 0xDE, 0x0B, 0xF1, 0xDE, 0x0B, 
  0xF1, 0xDE, 0x0B, 0xF1, 0xDE, 0x0B, 0xF7, 0xDE, 0x0B, 0x04, 0x00, 0x08, 
  0xFC, 0xFF, 0x0F,
};
  

const unsigned char WIFI_bits[] PROGMEM = {
  0xF0, 0x03, 0x04, 0x08, 0xF2, 0x13, 0x09, 0x24, 0xE4, 0x09, 0x10, 0x02, 
  0xC0, 0x00, 0xC0, 0x00,
  };

const unsigned char activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const unsigned char inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};
