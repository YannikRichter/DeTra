/*
 * HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A
 *
 * Function summary:
 *
 * - use internal RTC(150KHz);
 *
 * - Forward the data measured by the 12-bit ADC to the server.
 *
 * - Include stop mode and deep sleep mode;
 *
 * - 15S data send cycle;
 *
 * - Informations output via serial(115200);
 *
 * - Only ESP32 + LoRa series boards can use this library, need a license
 *   to make the code run(check you license here: http://www.heltec.cn/search/);
 *
 * You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"
 *
 * HelTec AutoMation, Chengdu, China.
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 * support@heltec.cn
 *
 *this project also release in GitHub:
 *https://github.com/HelTecAutomation/ESP32_LoRaWAN
*/

#include <ESP32_LoRaWAN.h>
#include "Arduino.h"
#include "images.h"               // Eigene HSPF-Grafiken

#include <Adafruit_BME280.h>      // Environment Sensor Library
#include <MAX30105.h>             // Pulse Sensor Libaray
#include <heartRate.h>            // Libaray for measuring heartRate
#include "spo2_algorithm.h"       // Libaray for measuring SpO2
#include <Adafruit_Sensor.h>      // Sensor Library
#include "Adafruit_BME680.h"      // VOC Sensor Libaray

#define LEDPin 25                 // LED light
#define Vext 21
#define Touch1 T2                 // Touch Pin 1  
#define Touch2 T5                 // Touch Pin 2
#define thresholdTouch 30       // Touch Sensitivity Level
#define MAX_BRIGHTNESS 255        // SpO2
#define LORAFRAMESIZE 17         // Bytes in LoRa Payload
#define SEALEVELPRESSURE_HPA (1013.25)
#define thresholdAceton 0.75      // Treshold for Acetondetetcion --> breathGas < 0.6 = Aceton detected , --> breathGas > 0.6 = Aceton not detected
#define bodytempValue 3.1        // needs to be add to the calculation of the bodytemperature
#define limitPulse 1700               // Time in milli sec for calculating a stable Pulse
#define limitSpo2 15
#define limitBodyTemp 24


/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
//uint32_t  license[4] = {0x2ABE0128, 0x49EAD768, 0x36610362, 0xBF66CDF3}; // LoRaNG_HSPF_Vitalora_1
uint32_t  license[4] = {0xAABA4109, 0xDB7E744C, 0x623F8356, 0x6F814A4E}; // LoRaNG_HSPF_Vitalora_2

/* OTAA para*/
//uint8_t DevEui[] = { 0x00, 0x4C, 0x8F, 0x10, 0xE5, 0x19, 0x89, 0xF9 }; // vitalora_1
uint8_t DevEui[] = { 0x00, 0xDA, 0xD1, 0x2F, 0xBC, 0xE9, 0xD0, 0x6B }; // vitalora_2

uint8_t AppEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0xDA, 0xEA }; // vitalora

//uint8_t AppKey[] = { 0x82, 0xB8, 0xB3, 0x88, 0x38, 0x15, 0x4D, 0x27, 0x71, 0xC3, 0x8E, 0xFB, 0x34, 0x87, 0x8C, 0x01 }; // vitalora_1
uint8_t AppKey[] = { 0x4F, 0x34, 0x8A, 0x1F, 0x73, 0x95, 0xDE, 0xE3, 0xC6, 0xDF, 0x00, 0x71, 0x64, 0xEF, 0xD2, 0x29 }; // vitalora_2

/* ABP para*/
uint8_t NwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t AppSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t DevAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
* None : print basic info.
* Freq : print Tx and Rx freq, DR info.
* Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
* Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

// Global Setup
bool touchONE = false;            // Touchpin 1 Variable
bool touchTWO = false;            // Touchpin 2 Variable
uint8_t Automatic;                  /// Entscheidung Manuell oder automatisch 0= manuell 1=automatisch

// Battery ADC Setup
float localBattVoltage;
float XS = 0.00127;               // The returned reading is multiplied by this XS to get the battery voltage.
float MUL = 2.26;                 // Voltage-Divider-Factor
uint8_t localBattGFX;

// Skin Sensor
Adafruit_BME280 bme;              // BME280 Instance
float localTemp;                  // Temperature Value
float localHumi;                  // Humidity Value
float localPres;                  // Pressure Value
float BodyTempResult;

// VCO Sensor
Adafruit_BME680 BreathSensor ;    // BME680 Instance
float breathTemp;                 // Temperatur Breath Sensor
float breathHumi;                 // Humidity Breath Sensor
float breathPres;                 // Pressure Breath Sensor
float breathGas;                  // Gas Resistance
float breathAlti;                 // Altitude Breath Sensor
String AcetonDetection;            // Indicates if Aceton is deteceted or not


// Pulse & SpO2 Sensor
MAX30105 particleSensor;          // Pulse Sensor Instance
uint32_t irBuffer[100];           // infrared LED sensor data
uint32_t redBuffer[100];          // red LED sensor data
int32_t heartRate;                // heart rate value
uint8_t HeartBeat;
int8_t validHeartRate;            // indicator to show if the heart rate calculation is valid
int32_t bufferLength = 100;       // buffer length of 100 stores 4 seconds of samples running at 25sps
int32_t spo2;                     // SPO2 value
int8_t validSPO2;                 // indicator to show if the SPO2 calculation is valid
byte readLED = LEDPin;

const byte RATE_SIZE = 15; //Increase this for more averaging. 13 is good
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;


void DATAformatSENDING()          // LoRa-Payload-Function
{
    appDataSize = LORAFRAMESIZE;             // Tranceive-Array-Size
    lora_printf("\n*** LORA PAYLOAD ON AIR ***\n");

    if(Automatic == 0)
    {
      lora_printf("\n*** WakeUp Reason ***\n");
    uint8_t WakeUpH = (int) Automatic;
    lora_printf("WakeUpReason: %d\n", WakeUpH);
    appData[0] = (char)(WakeUpH);         // Decision Decoder TTN
   
    
    lora_printf("\n*** LORA BATTERY VOLTAGE ***\n");
    uint8_t localBattVoltageH = (int) localBattVoltage;
    uint8_t localBattVoltageL = (int)((localBattVoltage - localBattVoltageH) * 100);
    lora_printf("LoRaBatteriespannung: %d,%0*d V\n", localBattVoltageH, 2, localBattVoltageL);
    appData[1] = (char)(localBattVoltageH); // Battery voltage before comma
    appData[2] = (char)(localBattVoltageL); // Battery voltage after comma

    lora_printf("\n*** LORA BME680 READINGS ***\n");
    uint8_t breathTempH = (int) breathTemp;
    uint8_t breathTempL = (int)((breathTemp - breathTempH) * 100);
    lora_printf("LoRaBreathTemperatur: %d,%0*d °C\n", breathTempH, 2, breathTempL);
    appData[3] = (char)(breathTempH);
    appData[4] = (char)(breathTempL);

    uint8_t breathHumiH = (int) breathHumi;
    uint8_t breathHumiL = (int)((breathHumi - breathHumiH) * 100);
    lora_printf("LoRaBreathFeuchtigkeit: %d,%0*d rF\n", breathHumiH, 2, breathHumiL);
    appData[5] = (char)(breathHumiH);
    appData[6] = (char)(breathHumiL);

    uint8_t breathPresH = (int) breathPres;
    uint8_t breathPresL = (int)((breathPres - breathPresH) * 100);
    lora_printf("LoRaBreathDruck: %d,%0*d hPa\n", breathPresH, 2, breathPresL);
    appData[7] = (char)(breathPresH);
    appData[8] = (char)(breathPresL);

    uint8_t breathGasH = (int) breathGas;
    uint8_t breathGasL = (int)((breathGas - breathGasH) * 100);
    lora_printf("LoRaBreathGas: %d,%0*d kOhm\n", breathGasH, 2, breathGasL);
    appData[9] = (char)(breathGasH);
    appData[10] = (char)(breathGasL);

    uint8_t breathAltiH = (int) breathAlti;
    uint8_t breathAltiL = (int)((breathAlti - breathAltiH) * 100);
    lora_printf("LoRaHöhenmeter: %d,%0*d m\n", breathAltiH, 2, breathAltiL);
    appData[11] = (char)(breathAltiH);
    appData[12] = (char)(breathAltiL);
    
    lora_printf("\n*** LORA MAX30105 READINGS ***\n");
    uint8_t HeartBeatH = (int) HeartBeat;
    lora_printf("Puls: %d bpm\n", HeartBeatH);
    appData[13] = (char)(HeartBeatH);
    uint8_t spo2H = (int) spo2;
    lora_printf("Sauerstoffsättigung: %d \n",spo2H);
    appData[14] = (char)(spo2H);
    uint8_t localTempH = BodyTempResult;
    uint8_t localTempL = ((BodyTempResult - localTempH) * 100);
    lora_printf("Körpertemperatur: %d,%0*d °C\n", localTempH, 2, localTempL);
    appData[15] = (char)(localTempH);        // Temperature before comma  
    appData[16] = (char)(localTempL); 
    
    // Combine the Payload Frame for sending
    lora_printf("LoRaPayload in HEX: { 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X }\n", 
          appData[0], appData[1], appData[2], appData[3], appData[4], appData[5], appData[6], appData[7], appData[8], appData[9],
          appData[10], appData[11], appData[12], appData[13], appData[14], appData[15], appData[16]);
    }
    if (Automatic == 1)
    {
    lora_printf("\n*** WakeUp Reason ***\n");
    uint8_t WakeUpH = (int) Automatic;
    lora_printf("WakeUpReason: %d\n", WakeUpH);
    appData[0] = (char)(WakeUpH);         // Decision Decoder TTN
   
  
    lora_printf("\n*** LORA BATTERY VOLTAGE ***\n");
    uint8_t localBattVoltageH = (int) localBattVoltage;
    uint8_t localBattVoltageL = (int)((localBattVoltage - localBattVoltageH) * 100);
    lora_printf("LoRaBatteriespannung: %d,%0*d V\n", localBattVoltageH, 2, localBattVoltageL);
    appData[1] = (char)(localBattVoltageH); // Battery voltage before comma
    appData[2] = (char)(localBattVoltageL); // Battery voltage after comma}
    
    lora_printf("\n*** LORA MAX30105 READINGS ***\n");
    uint8_t HeartBeatH = (int) HeartBeat;
    lora_printf("Puls: %d\n", HeartBeatH);
    appData[3] = (char)(HeartBeatH);
    uint8_t spo2H = (int) spo2;
    lora_printf("Sauerstoffsättigung: %d\n",spo2H);
    appData[4] = (char)(spo2H);
    uint8_t localTempH = BodyTempResult;
    uint8_t localTempL = ((BodyTempResult - localTempH) * 100);
    lora_printf("Körpertemperatur: %d,%0*d °C\n", localTempH, 2, localTempL);
    appData[5] = (char)(localTempH);        // Temperature before comma  
    appData[6] = (char)(localTempL); 
    
    // Combine the Payload Frame for sending
    lora_printf("LoRaPayload in HEX: { 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X}\n", 
          appData[0], appData[1], appData[2], appData[3], appData[4], appData[5], appData[6]);
}
}

void LEDdisplaySTART()        // HSPF Startscreen
{ 
  digitalWrite(Vext,LOW);
  Serial.println("STARTING");
  Display.wakeup();
  Display.init();
  delay(100);
  Display.flipScreenVertically();
  Display.clear();
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.setFont(ArialMT_Plain_16);
  Display.drawString(64, 0, "STARTE");
  Display.drawXbm(0,25,HSPF_LOGO_width,HSPF_LOGO_height,(const unsigned char *)HSPF_LOGO_bits);
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 44, "LoRaWAN");
  Display.drawString(0, 54, "Demostrator");
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.display();
}

void LEDdisplayJOINING()
{
  digitalWrite(Vext,LOW);
  Serial.println("JOINING");
  Display.wakeup();
  Display.clear();
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 44, "LoRaWAN");
  Display.drawString(0, 54, "Demostrator");
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER); 
  Display.drawString(64, 0, "VERBINDE...");
  Display.drawXbm(0,25,HSPF_LOGO_width,HSPF_LOGO_height,(const unsigned char *)HSPF_LOGO_bits);
  Display.display();
}

void LEDdisplayJOINED()
{
  digitalWrite(Vext,LOW);
  Serial.println("JOINED");
  Display.wakeup();
  Display.clear();
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 44, "LoRaWAN");
  Display.drawString(0, 54, "Demostrator");
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 0, "VERBUNDEN!");
  Display.drawXbm(0,25,HSPF_LOGO_width,HSPF_LOGO_height,(const unsigned char *)HSPF_LOGO_bits);
  Display.display();
  delay(1000);
}

void LEDdisplayMenu() 
{
  bool breakvalue = false;
  bool Decision = true;             // Manuel = true; automatich = false;
  bool ClearDisplay = true;         // Clears the diaplay if entering the Function for the first Time 
 
  if(touchONE == true && !touchTWO == true){
    touchONE = false;
  }
  if(touchTWO == true && !touchONE == true){
    touchTWO = false;
  }

  if(touchTWO == true && touchONE == true){
    touchTWO = false;
    touchONE = false;
    Serial.println("Touch 2 detected");
    Serial.println("Touch 1 detected");
  }
  
  if( Automatic == 1 )
  { 
    breakvalue = true;
    Decision = false;}
    
  
  ///BatterieStatus
   BatteryREADING();
   switch(localBattGFX)
    {
      case 100: // from 66 -100
        memcpy(BAT_bits, BAT_full, 27);
        break;
      case 66: // from 33 - 65
        memcpy(BAT_bits, BAT_66, 27);
        break;
      case 33: // from 0 - 32
        memcpy(BAT_bits, BAT_33, 27);
        break;
      case 0: //0
        memcpy(BAT_bits, BAT_empty, 27);
        break;
      default:
        break;
   }

   ///Start Menu
      Display.wakeup();
      Display.init();
      delay(50);
      Display.flipScreenVertically();
      Display.clear();
      // GFXs
      Display.setFont(ArialMT_Plain_16);
      Display.setTextAlignment(TEXT_ALIGN_CENTER);
      Display.drawString(64, 0, "MENÜ");
      Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
      Display.drawString(64, 20,"Manuell");
      Display.drawString(64, 40,"Automatisch");
      Display.setTextAlignment(TEXT_ALIGN_LEFT);
      Display.drawCircle(5,30,3);  
      Display.drawCircle(5,50,3); 
      Display.display();
  
  /// Auswahl so lange bis beide Pins gleichzeitig gedrückt werden
  while (breakvalue == false)
  {   
      delay(500);
      if (touchONE == true && !touchTWO == true)
      {
        touchONE = false;
        Decision = true;
        ClearDisplay = true;
        Serial.print("Touch1");
    
      }
       if (touchTWO == true && !touchONE == true )
      {
        touchTWO = false;
        Decision = false;
        ClearDisplay = true;
        Serial.print("Touch2");
        
      }
   if (ClearDisplay == true) /// Dispaly muss gereinigt werden, wenn pin gedrückt wird 
   { 
      Display.clear();
      ClearDisplay = false;
      Display.setFont(ArialMT_Plain_16);
      Display.setTextAlignment(TEXT_ALIGN_CENTER);
      Display.drawString(64, 0, "MENÜ");
      Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
      Display.drawString(64, 20,"Manuell");
      Display.drawString(64, 40,"Automatisch");
      Display.setTextAlignment(TEXT_ALIGN_LEFT);
      Display.drawCircle(5,30,3);  
      Display.drawCircle(5,50,3); 
      switch (Decision)
        {  
        case true:
        Display.fillCircle(5,30,3);
        Display.display();
        break;
        case false:
        Display.fillCircle(5,50,3);
        Display.display();
        break;
        default:
        break;
        }
    }

     if(touchTWO == true && touchONE == true)
      {
        breakvalue = true;
        touchONE = false;
        touchTWO = false;   
      }
     
}

    if (Decision == true)
    { 
    BatteryREADING();
    LEDdisplayMessungen();
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawProgressBar(23,40,80,10,0);
    Display.display();
    I2Cbme680READING();
    LEDdisplayMessungen();
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawProgressBar(23,40,80,10,50);
    Display.display();
    I2Cmax30105READING();
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawProgressBar(23,40,80,10,100);
    Display.display();
   
    }
     if (Decision == false)
    {
      Automatic = 1;
    BatteryREADING();
    LEDdisplayMessungen();
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawProgressBar(23,40,80,10,0);
    Display.display();
    delay(2000);
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawProgressBar(23,40,80,10,50);
    Display.display();
    I2Cmax30105READING();
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawProgressBar(23,40,80,10,100);
    Display.display();
    Display.sleep(); 
    }
    delay(1000);

}

void LEDdisplaySENDING()      // Payload-Data on OLED
{
  if(ifDisplayJoined && mcuStarted == 0)
    {
      LEDdisplayJOINED();
    }
  if (Automatic == 0)
 {
  Serial.println("SENDING");
  Display.wakeup();
  Display.init();
  delay(50);
  Display.flipScreenVertically();
  Display.clear();

  // GFXs
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(55, 0, "SENDE BME280");
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(5, 20, "KörperTemp: ");
  Display.drawString(5, 30, "Feuchte:");
  Display.drawString(5, 40, "Luftdruck: ");
  Display.drawString(5, 50, "Akkuspannung: ");


  switch(localBattGFX)
    {
      case 100: // from 66 -100
        memcpy(BAT_bits, BAT_full, 27);
        break;
      case 66: // from 33 - 65
        memcpy(BAT_bits, BAT_66, 27);
        break;
      case 33: // from 0 - 32
        memcpy(BAT_bits, BAT_33, 27);
        break;
      case 0: //0
        memcpy(BAT_bits, BAT_empty, 27);
        break;
      default:
        break;
   }
   
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.drawString(123, 20, String(BodyTempResult) + " °C");
  Display.drawString(123, 30, String(localHumi) + " %");
  Display.drawString(123, 40, String(localPres) + " hPa");
  Display.drawString(123, 50, String(localBattVoltage) + " V");
  Display.display();
  delay(4000);
  Display.clear();
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(55, 0, "SENDE BME680");
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(5, 20, "AußenTemp: ");
  Display.drawString(5, 30, "Feuchte:");
  Display.drawString(5, 40, "Aceton: ");
  Display.drawString(5, 50, "Höhe: ");
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.drawString(123, 20, String(breathTemp) + " °C");
  Display.drawString(123, 30, String(breathHumi) + " %");
  //Display.drawString(123, 40, String(breathGas) + " kOhm");
  Display.drawString(123, 40, AcetonDetection);
  Display.drawString(123, 50, String(breathAlti) + "m");
  Display.display();
  delay(4000);
  Display.clear();
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(55, 0, "SENDE MAX301");
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(5, 30, "HeartBeat:");
  Display.drawString(5, 40, "SpO2: ");
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.drawString(123, 30, String(HeartBeat) + " bpm");
  Display.drawString(123, 40, String(spo2) + " %");
  Display.display();
  delay(4000);
  Display.sleep();
 }
}

void LEDdisplayACKED()
{ 
  digitalWrite(Vext, LOW);
  Display.wakeup();
  Display.clear();
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.setFont(ArialMT_Plain_16);
  Display.drawString(64, 0, "LORA");
  Display.drawString(64, 16, "EMPFANG");
  Display.drawString(64, 32, "ERFOLGREICH");
  Display.drawString(64, 48, "BESTAETIGT");
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.display();

  delay(2000);
  Display.sleep();
  digitalWrite(Vext,HIGH);
  Serial.println("SLEEPING");
}
void LEDdisplayMessungen()
{
   switch(localBattGFX)
 {
      case 100: // from 66 -100
        memcpy(BAT_bits, BAT_full, 27);
        break;
      case 66: // from 33 - 65
        memcpy(BAT_bits, BAT_66, 27);
        break;
      case 33: // from 0 - 32
        memcpy(BAT_bits, BAT_33, 27);
        break;
      case 0: //0
        memcpy(BAT_bits, BAT_empty, 27);
        break;
      default:
        break;
   }
    Display.clear();
    Display.setFont(ArialMT_Plain_16);
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawString(55, 0, "MESSEN");
    Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
    Display.drawString(64, 20,"Bitte Warten");
    Display.display();
}
void LEDdisplayBreathCountdown()
{

 Display.clear();
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "MESSEN");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(64,30, "Breath out on\n the Sensor");
 Display.display();    
 delay(3000);
 for (int n = 3;n > 0; n--)                    //Timer
 { 
 Display.clear();
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "MESSEN");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.drawCircle(64,50,13);
 Display.drawString(64,40, String (n));
 Display.display(); 
 delay(1000);   
 }

 Display.clear();
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "MESSEN");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(64,40, "Breath out");
 Display.display();    
 delay(1000);
  
}

void touchCallbackOne(){
 touchONE = true;
 deviceState = DEVICE_STATE_SEND;
}

void touchCallbackTwo(){
 touchTWO = true;
 deviceState = DEVICE_STATE_SEND;
}

void I2Cmax30105READING()                              
{
  lora_printf("\n*** Körpertemperatur ***\n");
  ///////////////////////////////////////////////////////////////////////////////////////////////Setup MAX30105 TEMP
    
    float Temp;
    uint16_t counter = 0;
    particleSensor.begin(Wire, I2C_SPEED_FAST);
    particleSensor.setup(); //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is runnin 

    if ( particleSensor.getIR() < 5000)
     {  
     Display.clear();
     Display.setFont(ArialMT_Plain_16);
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawString(55, 0, "MESSEN");
     Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawString(64,40, "Bitte Anziehen");
     Display.display();
     while( particleSensor.getIR() < 5000)
     {delay(1000);}
     LEDdisplayMessungen();
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawProgressBar(23,40,80,10,50);
     Display.display();   
      }    
   
      uint8_t a = 0;  
      float BodyTemp_array[limitBodyTemp];
      float Result = 0;
     float normal = 0;
       while ( counter < limitBodyTemp)
       {

       normal= particleSensor.readTemperature();
       BodyTempResult = particleSensor.readTemperature()- 0.551;                                     
       BodyTemp_array[a] = BodyTempResult;     
       counter++;
       a++;     
      Serial.print("\nBodyTemp: "); Serial.print(BodyTempResult);
      Serial.println("\nTemp: "); Serial.print(normal);
    
        }
        Serial.print("average ");
       for ( int x = 0; x < limitBodyTemp; x++)
       {     
        Result += BodyTemp_array[x];
        }
        Result /= limitBodyTemp;
        BodyTempResult = Result;
        Serial.print(Result);
        
  lora_printf("\n*** Puls ***\n");
  //////////////////////////////////////////////////////////////////////////////////////////////Setup MAX30105 Heart beat
  counter = 0;
     if ( particleSensor.getIR() < 5000)
     {  
     Display.clear();
     Display.setFont(ArialMT_Plain_16);
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawString(55, 0, "MESSEN");
     Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawString(64,40, "Bitte Anziehen");
     Display.display();
     while( particleSensor.getIR() < 5000)
     {delay(1000);}
     LEDdisplayMessungen();
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawProgressBar(23,40,80,10,50);
     Display.display();   
      }    
  while (counter < limitPulse)
   {
       long irValue = particleSensor.getIR();   // raw Data
    
  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);  //calculation BPM
     

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        {beatAvg += rates[x];}
      beatAvg /= RATE_SIZE;
    }
   }
    HeartBeat= beatAvg;  //localVariable
    counter++;
    Serial.println(counter);
    Serial.println(beatAvg);
    Serial.println(beatsPerMinute);
}
    beatAvg = 0;
    rates[RATE_SIZE]={}; //Array of heart rates
    rateSpot = 0;
    lastBeat = 0; //Time at which the last beat occurred
    beatsPerMinute =0;
    counter = 0;

   lora_printf("\n*** Sauerstoffsättigung ***\n");
   //////////////////////////////////////////////////////////////////////////////////////////MAX301015 Spo2 Setup
    byte ledBrightness = 70; //Options: 0=Off to 255=50mA
    byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange =  4096; //Options: 2048, 4096, 8192, 16384
    uint8_t y = 0;
    int Array[limitSpo2];
    float Resultspo2 = 0;
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
    
    if ( particleSensor.getIR() < 5000)
     {  
     Display.clear();
     Display.setFont(ArialMT_Plain_16);
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawString(55, 0, "MESSEN");
     Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawString(64,40, "Bitte Anziehen");
     Display.display();
     while( particleSensor.getIR() < 5000)
     {delay(1000);}
     LEDdisplayMessungen();
     Display.setTextAlignment(TEXT_ALIGN_CENTER);
     Display.drawProgressBar(23,40,80,10,50);
     Display.display();   
      }    
  
    // read the first 100 samples, and determine the signal range
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
  
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }
    
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    
    Serial.println(spo2);
  
    while( counter < limitSpo2)
    {//dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
      {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }
  
      //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
      {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data
  
        digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
  
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
      }
    
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
   /// Höchsten Wert sortieren
    Serial.println(spo2);
    if(spo2 == -999)
    {spo2=0;}
    Array[y] = spo2;
    if (Array[y] < Array[y - 1] && y != 0)
    {Array[y] = Array[y - 1];}
    Serial.println("sortiert");
    spo2 = Array[y];
    Serial.println(spo2);
    counter++;
    y++;
    }
    Serial.println(spo2);
     Serial.println("average ");
    
       for ( int x = 0; x < limitSpo2; x++)
       {     
        Resultspo2 += Array[x];
        }
        Resultspo2 /= limitSpo2;
        //spo2 = Result;
       Serial.print(Resultspo2);
  lora_printf("\n*** I2C MAX30105 READING ***\n");
  lora_printf("KörperTemperatur: %f\n", BodyTempResult);
  lora_printf("Puls: %d Valid: %d\n", HeartBeat, validHeartRate);
  lora_printf("SpO2: %d Valid: %d \n", spo2, validSPO2);
  particleSensor.shutDown();
  
  }
 

void I2Cbme680READING()                              
{
  
  unsigned status = BreathSensor.begin(0x77);  
  uint16_t Results;
  uint8_t readings = 16;                            // 16 steps
  uint16_t Buffer[readings];
  float MeasuredGas;
  float gas_reference;
  
  // Set up oversampling and filter initialization
  BreathSensor.setTemperatureOversampling(BME680_OS_8X);
  BreathSensor.setHumidityOversampling(BME680_OS_2X);
  BreathSensor.setPressureOversampling(BME680_OS_4X);
  BreathSensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
  BreathSensor.setGasHeater(320, 150); // 320*C for 150 ms
  delay(150);
  BreathSensor.performReading();
  
  breathTemp = (BreathSensor.temperature - 0.5);
  breathHumi = (BreathSensor.humidity + 5.0);
  breathPres = (BreathSensor.pressure / 100);
  breathAlti = (BreathSensor.readAltitude(SEALEVELPRESSURE_HPA));

  ///  GAS Referenz Wert
  for (int i = 0; i <= readings; i++){                                    
    Buffer[i] = BreathSensor.readGas()/1000;
    Results += Buffer[i];
  }
  gas_reference = Results / readings;
  Results = 0;
  Buffer[readings] = {};
  
  LEDdisplayBreathCountdown(); 
  
  /// GAS Wert in der Atemluft
  for (int i = 1; i <= readings; i++) 
    { Buffer[i] = BreathSensor.readGas()/1000;
      Results += Buffer[i];}
   MeasuredGas = Results / readings;
   breathGas = (MeasuredGas / gas_reference);                // Ratio of measured gas and the gas reference

  // Abfrage ob Aceton detektiert ist oder nicht
 if ( breathGas > thresholdAceton) 
 {AcetonDetection = "Not Detected";} 
 if ( breathGas < thresholdAceton)
 {AcetonDetection = "Detected";} 
 
  lora_printf("\n*** I2C BME680 READING ***\n");
  lora_printf("Temperatur: %f °C\n", breathTemp);
  lora_printf("Luftfeuchte: %f %rF\n", breathHumi);
  lora_printf("Luftdruck: %f hPa\n", breathPres);
  lora_printf("Gas Widerstand: %f kOhm\n", breathGas);
  lora_printf("Höhenmeter: %f m\n", breathAlti);
  }
  


void BatteryREADING()                             // AD-Conversion Function 4 Battery-Level
{
     float localBattValue  =  analogRead(13)*XS*MUL;
     uint32_t ADCbuffer = 0;   
     
     for(uint i=0; i<8; i++)
     {
        ADCbuffer += analogRead(13);
     }
     Serial.println("ADC Average Value: "+String(ADCbuffer>>3)+" digs");
     localBattVoltage = (ADCbuffer>>3)*XS*MUL;
     Serial.println("ADC Average Value: "+String(localBattVoltage)+" V");

     if(localBattVoltage > 4.0) localBattGFX = 100;           // battery voltage divided into four steps
     else if (localBattVoltage > 3.5) localBattGFX = 66;
     else if (localBattVoltage > 3.3)  localBattGFX = 33;
     else localBattGFX = 0;    
     
     }

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    
   case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); Automatic = 1;
    break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); Automatic = 0;
    break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

// Add your initialization code here
void setup()
{
  pinMode(Vext, OUTPUT); 
  if(mcuStarted==0)
  {
    LEDdisplaySTART();
  }  

  pinMode(LEDPin, OUTPUT);

  for (byte i = 0 ; i < bufferLength ; i++)
  {
    redBuffer[i] = 0;
    irBuffer[i] = 0;
  }
 
  Serial.begin(115200);
  while (!Serial);
  SPI.begin(SCK,MISO,MOSI,SS);
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  
  adcAttachPin(13);
  analogSetClockDiv(255); // 1338mS

  touchAttachInterrupt(Touch1, touchCallbackOne, thresholdTouch);
  touchAttachInterrupt(Touch2, touchCallbackTwo, thresholdTouch);
  esp_sleep_enable_touchpad_wakeup();
  print_wakeup_reason();
  deviceState = DEVICE_STATE_INIT;
}

// The loop function is called in an endless loop
void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {

      LoRaWAN.init(loraWanClass,loraWanRegion);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LEDdisplayJOINING();
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      digitalWrite(Vext, LOW);
      LEDdisplayMenu();
      LEDdisplaySENDING();
      DATAformatSENDING();
      digitalWrite(Vext, HIGH);
      LoRaWAN.send(loraWanClass);
      LEDdisplayACKED();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);

      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    { 
      LoRaWAN.sleep(loraWanClass,debugLevel);

      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
