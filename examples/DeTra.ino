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
#include <Adafruit_BME280.h>      // Sensor-Library
#include <MAX30105.h>             // Sensor-Libaray
#include <heartRate.h>            // Libaray for measuring heartRate
#include "spo2_algorithm.h"       // Libaray for measuring SpO2
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"      // Sensor Libaray


#define Vext 21                   // digital pin 21 = GPIO21
#define Touch1 T2                 // Touch Pin 1  


/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x2ABE0128, 0x49EAD768, 0x36610362, 0xBF66CDF3}; // LoRaNG_HSPF_Vitalora_1                //License für Proto1  
//uint32_t  license[4] = {0xAABA4109, 0xDB7E744C, 0x623F8356, 0x6F814A4E}; // LoRaNG_HSPF_Vitalora_2                  //Licence für Proto2

/* OTAA para*/                                                                                                       //Over-The-Air Activation (OTAA) Nach jedem Neustart führt das Endgerät eine Join-Prodzedur (mit 3 Identifikatoren) aus 
uint8_t DevEui[] = { 0x00, 0x4C, 0x8F, 0x10, 0xE5, 0x19, 0x89, 0xF9 }; // vitalora_1                              //device EUI Proto1
//uint8_t DevEui[] = { 0x00, 0xDA, 0xD1, 0x2F, 0xBC, 0xE9, 0xD0, 0x6B }; // vitalora_2                                //device EUI Proto2

uint8_t AppEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0xDA, 0xEA }; // vitalora                                  //application identifier 

uint8_t AppKey[] = { 0x82, 0xB8, 0xB3, 0x88, 0x38, 0x15, 0x4D, 0x27, 0x71, 0xC3, 0x8E, 0xFB, 0x34, 0x87, 0x8C, 0x01 }; // vitalora_1   //application key 1
//uint8_t AppKey[] = { 0x4F, 0x34, 0x8A, 0x1F, 0x73, 0x95, 0xDE, 0xE3, 0xC6, 0xDF, 0x00, 0x71, 0x64, 0xEF, 0xD2, 0x29 }; // vitalora_2     //application key 2

/* ABP para*/                                                                                                                            // Activation By Personalization (ABP) Parameter werden direkt im Gerät gespeichert
uint8_t NwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };                   //network session key 
uint8_t AppSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };                   //application session key
uint32_t DevAddr =  ( uint32_t )0x007e6ae1;                                                                                              //device address

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;                                                                                                   //GeräteklasseA                                                                                                 

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 10800000; // 3hours          

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/                                                                                                                          //Adaptive Datenrate Optimierung von Datenraten, Sendezeit und Energieverbrauch im Netzwerk
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

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

//Battery Setup
float localBattVoltage;
float XS = 0.00127;               // The returned reading is multiplied by this XS to get the battery voltage. //Spannungswert pro bit
float MUL = 2.26;                 // Voltage-Divider-Factor // verhältniss spannungsteiler
uint8_t localBattGFX;

//Global Setup
#define thresholdTouch 30
bool touch1detected = false;
bool ClearDisplay = true;       // Clears the diaplay if entering the Function for the first Time 
uint8_t TmpStorage;             // Temporary stores the Decision in the start menu and if the Decision had changed it also clears the display 
uint8_t Decision;               // Identification which parameter is measured
uint8_t SendHeartBeat  = 0;         // Stores the last Heartbeat value after canceling the function
uint8_t SendSpo2 = 0;              // Stores the last Spo2 value after canceling the function
float SendTemp = 0;                // Stores the last BodyTemp value after canceling the function
uint8_t SendDetection;         // Stores 1 for Aceton detected and 0 for not detected
uint8_t CovidDetetcion;       //Stores 1 for Detected detected and 0 for not detected
float breathVOC;                // Ratio of the gas reference value and the detcted gas value
uint8_t WakeUpCycle;             // counts how often the MUC wakes up


//Calculations Setup 
float AverageHeartBeat;        
float AverageSpo2;
float AverageBodyTemp;
uint8_t countHeartBeat;         //counter for the average calculation
uint8_t countSpo2;              //counter for the average calculation
uint8_t countBodyTemp;          //counter for the average calculation
bool criticHeartBeat = false;   // indicates that the average value is not right
bool criticSpo2 = false;        //indicates that the average value is not right
bool criticBodyTemp = false;    //indicates that the average value is not right


//https://github.com/adafruit/Adafruit_BME680/blob/master/examples/bme680oled/bme680oled.ino
//BME680
Adafruit_BME680 BreathSensor ;
float gas_reference;          // Gas reference for calculating the ratio
float MeasuredGas;            



// BME280 Setup
Adafruit_BME280 bme;              
float localTemp;
float localHumi;
float localPres;
float SkinTemp;             //directly measured SkinTemp
float BodyTempResult;       //BodyTemp in realtion to the outside temperature
/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout
  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.
  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.
  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/
//MAX30105 Sensor Setup
MAX30105 particleSensor;
const byte RATE_SIZE = 13; //Increase this for more averaging. 13 is good
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

/*
  Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016
  https://github.com/sparkfun/MAX30105_Breakout
  This demo shows heart rate and SPO2 levels.
  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.
  
  This example is based on MAXREFDES117 and RD117_LILYPAD.ino from Maxim. Their example
  was modified to work with the SparkFun MAX30105 library and to compile under Arduino 1.6.11
  Please see license file for more info.
  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

// SpO2 Definintion
#define MAX_BRIGHTNESS 255        // SpO2
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

  
 

void DATAformatSENDING()          // LoRa-Payload-Function                                                                   
{ 
    appDataSize = 8 ;             // Tranceive-Array-Size
    lora_printf("\n*** LORA PAYLOAD ON AIR ***\n");
    switch (Decision)
  {
   case 1:
    {
//   Serial.print(" Sending Heart Beat");
    appData[0] = (char)(SendHeartBeat);
    appData[1] = (char)(Decision);           // Identification Heart Beat
    lora_printf("LoRaPayload in HEX: {0x%02X 0x%02X}\n", appData[0], appData[1]);
//    Serial.print("Sending Over\n");
    break;}
   case 2:
    {
//    Serial.print("Sending Temperature");
    int localTempH = SendTemp;                                                                                             
    int localTempL = ((SendTemp - localTempH) * 100);                                                                       
    lora_printf("LoRaTemperatur: %d,%0*d °C\n", localTempH, 2, localTempL);                                               
    appData[1] = (char)(Decision);                 // Identification BodyTemp                                                                      
    appData[2] = (char)(localTempL);
    lora_printf("LoRaPayload in HEX: { 0x%02X 0x%02X 0x%02X }\n", appData[0], appData[1], appData[2]);  // %02x char output ad 0´s to 2characters long       
//  Serial.print("Sending Over\n");
    break;}
   case 3:
    {
//  Serial.print("Seding Spo2");
    appData[0] = (char)(SendSpo2);
    appData[1] = (char)(Decision);           // Identification spo2
    lora_printf("LoRaPayload in HEX: {0x%02X 0x%02X}\n", appData[0], appData[1]);      
//    Serial.print("Sending Over\n");
    break;}
   case 4:
    {
//    Serial.print(" Sending VOC");
    appData[0] = (char)(SendDetection);
    appData[1] = (char)(Decision);           // Identification VOC
    lora_printf("LoRaPayload in HEX: {0x%02X 0x%02X}\n", appData[0], appData[1]);
//    Serial.print("Sending Over\n");
    break;}
    case 5:
    {
//    Serial.print(" Sending Periodic Samples");
    appData[0] = (char)(SendHeartBeat);
    appData[1] = (char)(Decision);             // Identification Periodic Samples
    appData[2] = (char)(SendSpo2);
    int localTempH = SendTemp;                                                                                             
    int localTempL = ((SendTemp - localTempH) * 100);                                                                                                                   
    appData[3] = (char)(localTempH);                                                                                                                                                                              
    appData[4] = (char)(localTempL);
    lora_printf("LoRaPayload in HEX: {0x%02X 0x%02X 0x%02X 0x%02X 0x%02X}\n", appData[0], appData[1], appData[2], appData[3], appData[4]);
//    Serial.print("Sending Over\n");
    break;}
    case 6: 
    {
//    Serial.print("Sending Detection");
    appData[0] = (char)(CovidDetetcion);
    appData[1] = (char)(Decision);           // Identification spo2
    lora_printf("LoRaPayload in HEX: {0x%02X 0x%02X}\n", appData[0], appData[1]);      
//    Serial.print("Sending Over\n");
    break;}
    default:
    {
//      Serial.print("Sending Nothing");
    break;}
  } 
    

}
void LEDdisplaySTART()        
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
  delay(3000);
}



void LEDdisplayMAX30105()
{
 Display.wakeup();
 Display.init();
 delay(100);
 Display.flipScreenVertically();
 Display.clear();

      //GFX
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "Calculations");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(64,30, "Place finger\n on the LED");
 Display.display();    
 delay(5000);
// Serial.print(" Place your Finger on the LED");
}

  
  

void LEDdisplayBreath()
{
 uint8_t readings = 32;    // 32 values
 uint16_t Result;         
 uint16_t Buffer[readings];
 Display.wakeup();
 Display.init();
 Display.flipScreenVertically();
 Display.clear();

      //GFX
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "Calculations");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);

Serial.println("Getting a new gas reference value");
Display.setFont(ArialMT_Plain_10);
Display.setTextAlignment(TEXT_ALIGN_CENTER);
Display.drawString(64,30, "Getting a new \ngas reference value");
Display.display();

  for (int i = 0; i <= readings; i++){                                    
    Buffer[i] = BreathSensor.readGas()/1000;
    Result += Buffer[i];
//    Serial.print(Result);
//    Serial.print("\n");
  }
  gas_reference = Result / readings;
//  Serial.print(Result);
//  Serial.print("\n");
//  Serial.print(gas_reference);
  
 Display.clear();
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "Calculations");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(64,30, "Breath out on\n the Sensor");
 Display.display();    
 delay(3000);
 Serial.print(" Breath out on the sensor");  
 
 
 for (int n = 3;n > 0; n--)                    //Timer
 { 
 Display.clear();
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "Calculations");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.drawCircle(64,50,13);
 Display.drawString(64,40, String (n));
 Display.display();    
 delay(1000);

 }
 
 Display.clear();
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "Calculations");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(64,40, "Breath out");
 Display.display();    
 delay(1000);
}

void LEDdisplayDECISION()
{
 if ( ClearDisplay == true || Decision != TmpStorage) // if you start your device it ClearDisplays the display; if you change your function ClearDisplay the display for deleting the Rectangle
 {
  Display.wakeup();
  Display.init();
  Display.flipScreenVertically();
  Display.clear();
  ClearDisplay = false;
  TmpStorage = Decision;
 }

      //GFX
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(64, 0, "Selection");
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
 Display.setFont(ArialMT_Plain_10);
 Display.setTextAlignment(TEXT_ALIGN_LEFT);
 Display.drawString(5, 20, "1: Heartbeat");
 Display.drawString(5, 30, "2: BodyTemp");
 Display.setTextAlignment(TEXT_ALIGN_RIGHT);
 Display.drawString(120, 20, "3: SpO2");
 Display.drawString(120, 30, "4: VOC");
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(64, 50, "5: Exit");

switch (Decision)
  {
    case 1:
   
    Display.drawRect(4, 20, 70, 12);
    Display.display();
    break;
    case 2:
 
    Display.drawRect(4, 30, 70, 12);
    Display.display();
    break;
    case 3:
    Display.drawRect(80, 20, 40, 12);
    Display.display();
    break;
    case 4:
    Display.drawRect(80, 30, 40, 12);
    Display.display();
    break;
    case 5:
    Display.drawRect(45, 50, 40, 12);
    Display.display();
    default:
    break;
  }

 Display.display();   
// delay(2000);
}


void LEDdisplayLoRaSend()
{ 
  digitalWrite(Vext, LOW);
  Display.wakeup();
  Display.clear();
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 0, "LORA");
  Display.drawString(64, 16, "RECEIVING");
  Display.drawString(64, 32, "SUCCESSFULLY"); 
  Display.drawString(64, 48, "CONFIRMED");
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.display();
  delay(4000);
  Display.sleep();
  digitalWrite(Vext,HIGH);                                                 
}

void LEDdisplayShutdown()
{
  digitalWrite(Vext, LOW);
  Display.wakeup();
  Display.clear();
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(50, 0, "SHUT DOWN");
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64,30, "Sleeping Mode");
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.display();
  delay(2000);
  Display.sleep();
  digitalWrite(Vext,HIGH);                                                 
//  Serial.println("SLEEPING");
  }
  
void I2Cbme680READING()                                                   
{
     
  unsigned status = BreathSensor.begin(0x77);  
  uint16_t Results;
  uint8_t readings = 16;                 // 16 steps
  uint16_t Buffer[readings];
     
  // Set up oversampling and filter initialization
   BreathSensor.setTemperatureOversampling(BME680_OS_8X);
   BreathSensor.setHumidityOversampling(BME680_OS_2X);
   BreathSensor.setPressureOversampling(BME680_OS_4X);
   BreathSensor.setGasHeater(320, 150); // 320*C for 150 ms
    
  LEDdisplayBreath();

 for (int i = 1; i <= readings; i++) 
    { Buffer[i] = BreathSensor.readGas()/1000;
      Results += Buffer[i];}
    
 MeasuredGas = Results / readings;
 breathVOC = (MeasuredGas / gas_reference);                // Ratio of measured gas and the gas reference
// Serial.print("breathVOC = "); Serial.print(breathVOC); Serial.println(" KOhms");
// Serial.print("Gas = "); Serial.print(MeasuredGas); Serial.println(" KOhms");
// Serial.print("gas_reference = "); Serial.print(gas_reference); Serial.println(" KOhms");
// Serial.print("breathVOC = "); Serial.print(breathVOC); Serial.println(" KOhms");
 
 Display.wakeup();
 Display.init(); 
 Display.flipScreenVertically();
 Display.clear();

  // GFXs
 Display.setFont(ArialMT_Plain_16);
 Display.setTextAlignment(TEXT_ALIGN_CENTER);
 Display.drawString(55, 0, "Calculations");
 Display.setFont(ArialMT_Plain_10);
 Display.setTextAlignment(TEXT_ALIGN_LEFT);                                                 
 Display.drawString(5, 20, "Ratio: ");   
 Display.drawString(5, 40, "Result: ");                     
 Display.setTextAlignment(TEXT_ALIGN_RIGHT);
 Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
//  Display.drawString(123, 20, String(Gas) + " kOhms");                       
//  Display.drawString(123, 40, String(gas_reference) + " kOhms");   
 Display.drawString(123, 20, String(breathVOC));
 if ( breathVOC < 1.8)
 {Display.drawString(100, 50, "Aceton detected");
 SendDetection = 1;} 
 else {Display.drawString(100,50, "Aceton not detected");
 SendDetection = 0;}          
 Display.display(); 
 DATAformatSENDING();
 delay(8000);
 Display.sleep();// Display shutdown
}



void I2CbmeREADING()                              
{
  
  bme.begin(0x76);             // BME280 Sensor Init
  BreathSensor.begin(0x77);   // BME680 Sensor Init
  uint8_t counter = 0;        // Septs of same Temp 
  uint8_t i = 0;             
  float BodyTemp_array[100];
  bool breakvalue = false;   // stops the whole function
  touch1detected = false;
 
  
   // Set up oversampling and filter initialization
   BreathSensor.setTemperatureOversampling(BME680_OS_8X);
   BreathSensor.setHumidityOversampling(BME680_OS_2X);
   BreathSensor.setPressureOversampling(BME680_OS_4X);
   BreathSensor.setGasHeater(320, 150); // 320*C for 150 ms
   
  localTemp = BreathSensor.readTemperature();                                     // outside Temp
   while ( breakvalue == false && touch1detected == false)
   {
   SkinTemp = (bme.readTemperature() - 0.5);  // vitalora_1  // -0.5 tollerance
   BodyTempResult = SkinTemp + ( SkinTemp - localTemp)+1.5;                       // measured skinTemp + (the Ratio of SkinTemp and LocalTemp)--> if its hot ouside then the Temperatur of the skin is getting also hotter, + 3.3 beacuse the Bodytemperatur is always lower at the wrist
   BodyTemp_array[i] = BodyTempResult;
   SendTemp = BodyTempResult;
   if (BodyTemp_array[i] == BodyTemp_array[i - 1]){counter ++;}      // if the Temp value is same as the last one --> counter+1 
   else {counter = 0;}
   if ( counter > 4){breakvalue = true;}                            // if the Temp value is same for 4 readings --> stop the reading       
  
//  Serial.print("\ncounter: "); Serial.print(counter);
//  Serial.print("\nBodyTemp: "); Serial.print(BodyTempResult);
//  Serial.print("\nBODYTEMP1: "); Serial.print( SkinTemp);
  Display.clear();
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(55, 0, "Calculations");
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(5, 20, "Body Temperature:");                                                   
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(75, 40, String(BodyTempResult) + "°C");                      
  
  Display.display();
  i++;
  countBodyTemp++;
 
   if (i == 100){i = 0;}
    delay(400);
    }
   DATAformatSENDING();
//   Serial.print("OVER");
   delay(3000);
  }


 void I2CMAXSp02Reading()
 {
  bool breakvalue = false;
  uint8_t limit = 20;
  long int irValue;
  long int redValue; 
  uint8_t a = 0;
  uint8_t Saver = 0;
  touch1detected = false;
  
 
  //MAX Setup SpO2
  unsigned status = particleSensor.begin(Wire, I2C_SPEED_FAST);
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these setting
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  LEDdisplayMAX30105();


 //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    
    while (particleSensor.available() == false) //do we have new data?
    particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    redValue=redBuffer[i];
    irBuffer[i] = particleSensor.getIR();
    irValue=irBuffer[i];
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

//    Serial.print(F("red="));
//    Serial.print(redBuffer[i], DEC);
//    Serial.print(F(", ir="));
//    Serial.println(irBuffer[i], DEC);
   
    if (irValue < 5000 && redValue < 5000 )  // standart values without finger
      {
//      Serial.print(" no Finger?");
        a+=1;
    if (a == limit)
    { breakvalue = true;
      spo2=0;
      redBuffer[i]={};//Array of heart rates
      irBuffer[i]={};
      validHeartRate = 0;
      validSPO2 = 0; //Time at which the last beat occurred
      } 
      }
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  Saver = spo2;                                                                                                                           // saves the Spo2 value so it can be send after the function stops
      
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second

  while (breakvalue == false && touch1detected == false)
  {
    
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
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
      redValue=redBuffer[i];
      irBuffer[i] = particleSensor.getIR();
      irValue=irBuffer[i];
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
     
      if (irValue < 5000 && redValue < 5000 )  // standart values without finger
      {
        if (a == 0)
     {SendSpo2 = Saver;}          
//      Serial.print(" No finger?");
      a+=1;
     
      if (a == limit)
      { breakvalue = true;
        spo2=0;
        redBuffer[i]={};//Array of heart rates
        irBuffer[i]={};
        validHeartRate = 0;
        validSPO2 = 0; //Time at which the last beat occurred
      } 
      }
      BatteryREADING();                                                       // read Battery useage

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
  
   if (spo2 < 0)  // if there is no pressure show 0% instead of -999
   {spo2=0;};
   
       //Serial print
      //send samples and calculation result to terminal program through UART
//      Serial.print(F("red="));
//      Serial.print(redBuffer[i], DEC);
//      Serial.print(F(", ir="));
//      Serial.print(irBuffer[i], DEC);
//      Serial.print(F(", SPO2="));
//      Serial.print(spo2, DEC);

      // Display Print
 
      Display.wakeup();
      Display.clear();
      Display.setFont(ArialMT_Plain_16);
      Display.setTextAlignment(TEXT_ALIGN_CENTER);
      Display.drawString(55, 0, "Calculations");
      Display.setFont(ArialMT_Plain_10);
      Display.setTextAlignment(TEXT_ALIGN_LEFT);
      Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
      Display.drawString(5, 30, "SpO2:");                                 
      Display.drawString(5, 40, "Battery voltage: ");                                                  
      Display.setTextAlignment(TEXT_ALIGN_RIGHT);                                                                        
      Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
      Display.drawString(123, 30, String(spo2) + " %");                       //showing humidity value 
      Display.drawString(123, 50, String(localBattVoltage) + " V");                //showing Battery volatge value 
      Display.display();
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    Saver = spo2;                                                                                                                                 // saves the Spo2 value so it can be send after the function stops
  }
  DATAformatSENDING();
  countSpo2++;                        // counts the how often the funnction is performed --> for the Average Calculation
  particleSensor.shutDown();
  Display.sleep();// Display shutdown
 }


void I2CMAXBeatReading()
 {
 uint8_t  limit = 40;                       // limit if finger is to long away function is going to stop
 bool breakvalue = false;                           //  parameter for thetimer
 uint8_t  a=0;
 
 touch1detected = false;
 
 
    
  //Setup MAX30105 Heart beat
    particleSensor.begin(Wire, I2C_SPEED_FAST);
    particleSensor.setup(); //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
    
 LEDdisplayMAX30105();

  while ( breakvalue == false  && touch1detected == false )
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
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
   }
   BatteryREADING();                                                       // read Battery useage

  switch(localBattGFX)                                                     //battery voltage divided into four steps // grafix beenden
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
   SendHeartBeat = beatAvg;                
//  Serial.print(irValue);
//  Serial.print(beatAvg);
  // Drawing Values on the Display
  Display.clear();
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(55, 0, "Calculations");
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.drawString(5, 30, "Heart Beat:");                                  //Heart Rate
  Display.drawString(5, 50, "Battery voltage: ");                       
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);                                                                        // read Temp, pressure and humidiy
  Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
  Display.drawString(123, 30, String(beatAvg) + " bpm");                       //showing humidity value 
  Display.drawString(123, 50, String(localBattVoltage) + " V");
  Display.display();
   
  if (irValue < 5000)
  {
//    Serial.print(" No finger?");
   a+=1;
    if (a == limit)
    { breakvalue = true;
      beatAvg=0;
      rates[RATE_SIZE]={}; //Array of heart rates
      rateSpot = 0;
      lastBeat = 0; //Time at which the last beat occurred
      beatsPerMinute =0;
      } 
  } 
//  Serial.println();
  }
  DATAformatSENDING();
  countHeartBeat++;
  particleSensor.shutDown();
  Display.sleep();

 }
 
 void PeriodicSamples()
{    
    //General Setup
    uint16_t interval = 1000;   // steps of Measurements Heartbeat
    uint8_t interval2 = 10;     // steps of Measurements Spo2
    uint16_t interval3 = 10000; // steps of Measurements BodyTemp
    uint16_t counter = 0;       // counts the steps of Measurements
    long int irValue = 0;
    long int redValue = 0;
    Decision = 5; //Case Periodic Samples DataFORMATSENDING;
    touch1detected = false;
   
    
    
    //Display Setup
    if ( ClearDisplay == true) // if you start your device ClearDisplay the display;
     {
      Display.wakeup();
      Display.init();
      Display.flipScreenVertically();
      Display.clear();
      ClearDisplay = false;
     }
    Display.wakeup();
    Display.clear();
    Display.setFont(ArialMT_Plain_16);
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawString(55, 0, "Calculations");
    Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
    Display.setTextAlignment(TEXT_ALIGN_CENTER);
    Display.drawString(64,30, "Periodic Samples");
    Display.display();
    
   //Setup Sensors
                
    //Heart Beat
    particleSensor.begin(Wire, I2C_SPEED_FAST);
    particleSensor.setup(); //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
    while (counter < interval && touch1detected == false)
    {
       irValue = particleSensor.getIR();   // raw Data
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
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
       }
      counter++;
      SendHeartBeat = beatAvg;
//      Serial.print("Heart Beat: ");
//      Serial.print(beatAvg);
//      Serial.print("\n");
//      Serial.print(counter);
      if (irValue < 5000)
        {
//        Serial.print(" No finger?");
            beatAvg=0;
            rates[RATE_SIZE]={}; //Array of heart rates
            rateSpot = 0;
            lastBeat = 0; //Time at which the last beat occurred
            beatsPerMinute =0;
            }
        
        }
        
    beatAvg=0;
    rates[RATE_SIZE]={}; //Array of heart rates
    rateSpot = 0;
    lastBeat = 0; //Time at which the last beat occurred
    beatsPerMinute =0;
    counter = 0;

    //Spo2 
    particleSensor.begin(Wire, I2C_SPEED_FAST);
    byte ledBrightness = 60; //Options: 0=Off to 255=50mA
    byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these setting
    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
    
    if (touch1detected == false){
    for (byte i = 0 ; i < bufferLength ; i++)
    {
    
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    redValue=redBuffer[i];
    irBuffer[i] = particleSensor.getIR();
    irValue=irBuffer[i];
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

//    Serial.print(F("red="));
//    Serial.print(redBuffer[i], DEC);
//    Serial.print(F(", ir="));
//    Serial.println(irBuffer[i], DEC); 
  }
  
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);}
    while (counter < interval2 && touch1detected == false)
    {
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
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
        redValue = redBuffer[i];
        irBuffer[i] = particleSensor.getIR();
        irValue=irBuffer[i];
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
      if (irValue < 5000 && redValue < 5000  )  // standart values without finger
      {
//        Serial.print(" No finger");
        spo2 = 0;
        redBuffer[i]={};//Array of heart rates
        irBuffer[i]={};
        validHeartRate = 0;
        validSPO2 = 0;}//Time at which the last beat occurred
     
      if(spo2 < 0)
         {spo2 = 0;}
      SendSpo2 = spo2;
//      Serial.print("Spo2: ");
//      Serial.print(spo2);
//      Serial.print("\n");
//      Serial.print(counter);
    
    }
     counter++;
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    }
    spo2=0;
    redBuffer[100]={};//Array of heart rates
    irBuffer[100]={};
    validHeartRate = 0;
    validSPO2 = 0; //Time at which the last beat occur
    counter = 0;
    particleSensor.shutDown();
    
    //Temperature
    bme.begin(0x76);            // BME280 Sensor Init
    BreathSensor.begin(0x77);  
    
    uint8_t i = 0;
    uint8_t interrupt = 0;
    float BodyTemp_array[100];
    bool breakvalue = false;
  
     // Set up oversampling and filter initialization
      BreathSensor.setTemperatureOversampling(BME680_OS_8X);
      BreathSensor.setHumidityOversampling(BME680_OS_2X);
      BreathSensor.setPressureOversampling(BME680_OS_4X);
      BreathSensor.setGasHeater(320, 150); // 320*C for 150 ms
      localTemp = BreathSensor.readTemperature();
     while ( breakvalue == false && touch1detected == false)
     {
     //localTemp = BreathSensor.readTemperature()+ 0.7;
     SkinTemp= (bme.readTemperature() - 0.5);  // vitalora_1  // -0.5 tollerance
     BodyTempResult = SkinTemp + ( SkinTemp - localTemp)+3.3;                           // measured skinTemp + (the Ratio of SkinTemp and LocalTemp)--> if its hot ouside then the Temperatur of the skin is getting also hotter, + 3.3 beacuse the Bodytemperatur is always lower at the wrist
     BodyTemp_array[i] = BodyTempResult;
     SendTemp = BodyTempResult;
     if (BodyTemp_array[i] == BodyTemp_array[i - 1]){counter ++;}
     else {counter = 0;}
     if ( counter > 4 || interrupt > 200 ){breakvalue = true;}                             // if the Temp value is same for 4 readings --> stop the reading or a if no clear temperature can be determined
//     Serial.print("\ncounter: "); Serial.print(counter);
//     Serial.print("\nBodyTemp: "); Serial.print(BodyTempResult);
//     Serial.print("\nBODYTEMP1: "); Serial.print( SkinTemp);
     i++;
     interrupt ++;
     if ( i == 100){ i = 0;}  // ClearDisplay Array after 100 steps
      delay(400);
      }
    
    
    DATAformatSENDING();
    Display.sleep();

}
 
  


void BatteryREADING()                             // AD-Conversion Function 4 Battery-Level
{
     float localBattValue  =  analogRead(13)*XS*MUL;  // calculating localBattValue
     uint32_t ADCbuffer = 0;   
     
     for(uint i=0; i<8; i++)
     {
        ADCbuffer += analogRead(13);   // reading Voaltage values
     }
     //Serial.println("ADC Average Value: "+String(ADCbuffer>>3)+" digs");
     localBattVoltage = (ADCbuffer>>3)*XS*MUL;   //>> right shift operator zahl halbieren
     //Serial.println("ADC Average Value: "+String(localBattVoltage)+" V");

     if(localBattVoltage > 4.0) localBattGFX = 100;           // battery voltage divided into four steps Btterie modus
     else if (localBattVoltage > 3.5) localBattGFX = 66;
     else if (localBattVoltage > 3.3)  localBattGFX = 33;
     else localBattGFX = 0;    
     
   }
   

void startmenu()
{

  uint8_t  limit = 6; // limit of steps
  uint8_t  b = 5;     // possible functions
  uint8_t Zaehler = 0; //steps of the function
  bool breakvalue = false;
  bool Reset = false;
  
  ClearDisplay = true;
  uint8_t limit1 = 10;
  uint8_t Touch;
 
  touch1detected = false;
    
     while (breakvalue == false)
      { 
         if (ClearDisplay == true)
         {
          Decision = 0;
          LEDdisplayDECISION();
          ClearDisplay = false;
          }
        Touch = touchRead(T2);
                  
          if (Zaehler > limit && Decision != 0 )     // if a defined time has passed after a touch and if a Parameter is chosen --> Reset = true;
          {Reset = true; }
          if(Touch <limit1)
           {
          Serial.println("Touch1 detected");
          Decision++; 
          Zaehler = 0;
          if (Decision > b)
          {Decision = 0;}
          }
          Zaehler++;
          if (Zaehler > 7)
          {Zaehler=0;}
       
           LEDdisplayDECISION();
           delay(1000);
         if(Reset == true)
            {
            switch (Decision)
            {
            case 1:
              I2CMAXBeatReading();
              breakvalue = true;
             break;
            case 2:
             I2CbmeREADING();   
             delay(1000);
               breakvalue = true;
              break;
             case 3:
             I2CMAXSp02Reading();      
              breakvalue = true;
              break;
             case 4:
            I2Cbme680READING();
            breakvalue = true;
             break;
              case 5:
              breakvalue = true;
             default:
              Serial.print("Nothing\n");
            break;
          }
     }
     }  
}


void Calculations()
{
  
   uint8_t Spo2treshold = 90;   // under 90% should be treated
   uint8_t HeartBeattreshold1 = 80;// average heartbeat for a adult 60-80/min
   uint8_t HeartBeattreshold2 = 60; // average heartbeat for a adult 60-80/min
   uint8_t BodyTemptreshold1 = 37;  // average Temp for a adult is 36.5 - 37
   uint8_t BodyTemptreshold2 = 36.5;  // average Temp for a adult 36.5 - 37
   ClearDisplay = false;
   bool Reset = false;
 switch (Decision)
  {
   case 1:
    { 
      if(countHeartBeat !=0)
      {AverageHeartBeat=((AverageHeartBeat)+SendHeartBeat);
      AverageHeartBeat= AverageHeartBeat/(countHeartBeat);}
      else 
        {AverageHeartBeat = SendHeartBeat;}
    break;}
   case 2:
    {
      if(countBodyTemp !=0)
      {AverageBodyTemp=((AverageBodyTemp)+SendTemp);
     AverageBodyTemp = AverageBodyTemp/(countBodyTemp);}
      else 
        {AverageBodyTemp = SendTemp;
    break;}
   case 3:
    {
      if(countSpo2 !=0)
      {AverageSpo2=((AverageSpo2)+SendSpo2);
      AverageSpo2= AverageSpo2/(countSpo2);}
      else 
        {AverageSpo2 = SendSpo2;}
    break;}
  
    case 5:
    {
      if(countHeartBeat !=0)
      {AverageHeartBeat=((AverageHeartBeat)+SendHeartBeat);
      AverageHeartBeat = AverageHeartBeat/(countHeartBeat);}
      else 
        {AverageHeartBeat = SendHeartBeat;}
      if(countBodyTemp !=0)
      {AverageBodyTemp=((AverageBodyTemp)+SendTemp);
     AverageBodyTemp = AverageBodyTemp/(countBodyTemp);}
      else 
        {AverageBodyTemp = SendTemp;}
         if(countSpo2 !=0)
      {AverageSpo2=((AverageSpo2)+SendSpo2);
      AverageSpo2= AverageSpo2/(countSpo2);}
      else 
        {AverageSpo2 = SendSpo2;}
    }
    break;}
    default:
    {Serial.print("Nothing");
    break;}
  }
  if ( (AverageHeartBeat > HeartBeattreshold1) || (AverageHeartBeat < HeartBeattreshold2) && AverageHeartBeat != 0) //If the value is not the same as the average and if  it is realy measured--> something could be wrong
      { criticHeartBeat = true;}
  if ( (AverageBodyTemp > BodyTemptreshold1 )|| (AverageBodyTemp < BodyTemptreshold2) && AverageBodyTemp != 0) //If the value is not the same as the average and if  it is realy measured--> something could be wrong
      { criticBodyTemp = true;}
  if ( AverageSpo2 < Spo2treshold && AverageSpo2 != 0)//If the value is not the same as the average and if  it is realy measured--> something could be wrong
      { criticSpo2 = true;}

  if ( criticHeartBeat == true && criticBodyTemp == true && criticSpo2 ==true)
  {
    CovidDetetcion= 1;  // indicates that covid is detetced
    Decision = 6;      // 6 for Covid-19 detected
    DATAformatSENDING();
    CovidDetetcion = 0;
    ClearDisplay = true;
    while (touch1detected == false ) 
    {  
       if (ClearDisplay == true)
       {Display.wakeup();
       Display.init();
       delay(100);
       Display.flipScreenVertically();
       Display.clear();}
      
            //GFX
       Display.setFont(ArialMT_Plain_16);
       Display.setTextAlignment(TEXT_ALIGN_CENTER);
       Display.drawString(55, 0, "Detection");
       Display.drawXbm(105, 4, BAT_width, BAT_height, (const unsigned char*)BAT_bits);
       Display.setFont(ArialMT_Plain_10);
       Display.setTextAlignment(TEXT_ALIGN_CENTER);
       Display.drawString(64,30, "Possible COVID-19\n Infection");
       Display.display();        
       delay (3000);
       ClearDisplay = false;
       Reset = true;
  }
  if (WakeUpCycle > 15)
  {Reset = true;}
  if (Reset == true)
  { criticHeartBeat = false;
    criticBodyTemp = false;
    criticSpo2 = false;
    AverageHeartBeat = 0;
    AverageBodyTemp = 0;
    AverageSpo2 = 0;
    countHeartBeat = 0;
    countBodyTemp = 0;
    countSpo2 = 0;}
  
  }
}
 
void gotTouch1(){
 touch1detected = true;
}




void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    
   case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); touch1detected = false;
    break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); touch1detected = true;
    break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


 
// Add your initialization code here
void setup()
{

  pinMode(Vext, OUTPUT);   // sets Vext pin as output
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  if(mcuStarted==0)
  {
    LEDdisplaySTART();     // LEd display HSPF startscreen
    
  }  
  Serial.begin(115200);
  //Serial.begin(9600);                             // sets the date rate in bits per second for serial data transmission /115200
  while (!Serial);                                   // if there is a not serial data transmission then-->
  SPI.begin(SCK,MISO,MOSI,SS);                       // serial peripheral Interface Master and Slave
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  
  adcAttachPin(13);                                   // Attach a pin to ADC --> reding voltage values
  analogSetClockDiv(255); // 1338mS                   //Set the divider for the ADC clock
  deviceState = DEVICE_STATE_INIT; // defined in ESP32_LoRaWAN.h
  touchAttachInterrupt(Touch1, gotTouch1, thresholdTouch);
  print_wakeup_reason();
  esp_sleep_enable_touchpad_wakeup();
    
}




// The loop function is called in an endless loop
void loop()
{ 
  


  switch( deviceState )                        
  {
    case DEVICE_STATE_INIT:                    
    {
      
     LoRaWAN.init(loraWanClass,loraWanRegion); // (Class _A , EU868) inside the function deviceState = dDevice_State_Join (ESP32_LoRaWAN.cpp);
     deviceState =DEVICE_STATE_SEND;
      break;
     }
    case DEVICE_STATE_JOIN:                  
     {
      
      LEDdisplayJOINING();                    // Display is changing 
      LoRaWAN.join();                          // query if it is overThe AirActivation and the requirements are right --> DEVICE_STATE_SLEEP if not DEVICE_STATE_Send --> Stage 3
      break;
    }
    case DEVICE_STATE_SEND:                  
    {   
      ClearDisplay = true; // for Display.init()
      digitalWrite(Vext, LOW);        //  Vext control writing Low Value --> On
      if(ifDisplayJoined)             // Joined Display
      {LEDdisplayJOINED();}
      
     if (touch1detected == false)           
      {PeriodicSamples();}
     if(touch1detected == true)              // Touch Pin 1 and Touch Pin 2 needs to be touch to wakeup the MCU
      {startmenu();}
     
     
      Calculations();
      WakeUpCycle++;
     digitalWrite(Vext, HIGH);
     LoRaWAN.send(loraWanClass);
     LEDdisplayLoRaSend();
     deviceState = DEVICE_STATE_CYCLE; 
     break;
    } 
    case DEVICE_STATE_CYCLE:                 
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);       //Timer to handle the application data transmission duty cycle  gibt an wielange schlafen
       
      deviceState = DEVICE_STATE_SLEEP;    
      LEDdisplayShutdown();
      break;
    }
    case DEVICE_STATE_SLEEP:             
    {
    
      LoRaWAN.sleep(loraWanClass,debugLevel); 
      break;
    }
    default:
    {
      
      deviceState = DEVICE_STATE_INIT;  // Stage 1
      break;
    }
  }
}
