#include <SPI.h>                 //Adafruit_SD-> ReadWrite/sheild_sdlog
#include <SD.h>                  //Adafruit_SD-> ReadWrite/sheild_sdlog
#include <Wire.h>                //MPL3115A2/BMP180 since I2C
#include <Adafruit_MPL3115A2.h>  //Adafruit_MPL3115aA2-> testmpl3115a2
#include <avr/sleep.h>           //Adafruit_GPS-> shield_sdlog
#include <SFE_BMP180.h>          //Sparkfun_BMP180-> SFE_BMP180
#include "MPL3115A2.h"           //Sparkfun_MPL3115A2 (!!If Using SF Lib!!)
#include <PlayingWithFusion_MAX31865.h>        //PWF-> core library
#include <PlayingWithFusion_MAX31865_STRUCT.h> //PWF-> struct library
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "RTClib.h"
 
/*"voltmeter"*/
#define aref_voltage 5 //or 3.3 optionally- this defines the AREF pins voltage if we choose to use it. MUST wire AREF pin to 5V to work.
#define bandgap_voltage 1.1 //not guaranteed to be 1.1 but not going to be too far off.
#define BANDGAPREF 14  //for measuring bandcap
 
/*logger*/
#define LOG_INTERVAL 5000 //how long do we want between log intervals (in ms)?
File logFile;             //For logger: insert filename here
const int chipSelect = 10; //Pin 10 for logger
RTC_DS1307 RTC;            //define logger RTC object
// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3
/*TMP36*/
#define TMPPin 0     //TMP uses pin A0
//int TMPValue = 0; --might use this in loop instead
/*ASDX Pressure*/
#define ASDXPin 1    //ASDX uses pin A1
//int ASDXValue = 1; --might do this in loop instead
/*SF BMP180*/
SFE_BMP180 BMPress;        //pressure object for BMP180
#define ALTITUDE 1655.0    //for baseline altitude from launch site.
/*P.W.F. MAX RTD*/
const int RTDPin = 9;      //for CS on RTD breakout
PWFusion_MAX31865_RTD rtd_ch0(RTDPin);
/*GPS*/
SoftwareSerial mySerial(3, 2); //can change pin numbers to match wiring
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true          //debug/listen to raw GPS. Set to false for silence.
boolean usingInterrupt = false; // this keeps track of whether we're using interrupt off by default
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
 
//int freeRam () //use a freeRam() anywhere in code to report memory usage
//{
//  extern int __heap_start, *__brkval;
//  int v;
//  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
//}
 
void setup() {
Serial.begin(9600);
 
/*BMP180 STARTUP*/
    if (BMPress.begin())
      Serial.println(F("BMP180 init success"));
    else
    { // Oops, something went wrong, this is usually a connection problem,
     Serial.println(F("BMP180 init fail\n\n"));
    //tone(5,8000,500);
    //delay(500);  //play some noise
    //while(1); // Pause forever.
    }
    delay(100);
/*LOGGER STARTUP*/
    pinMode(SS, OUTPUT); //set pin 10 (SS SPI Pin) to OUTPUT mode
    if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    //tone(5,8000,500);
    //delay(500);  //play some noise
    //while (1) ; // don't do anything more
    }
    // create a NEW file
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        logFile = SD.open(filename, FILE_WRITE);
        break;  // leave the loop!
       }
      }
    Serial.println(F("if no card failed msg, card is initialized."));
     // connect to RTC
      Wire.begin();  
        if (!RTC.begin()) {
          logFile.println("RTC failed");
      #if ECHO_TO_SERIAL
          Serial.println("RTC failed");
      #endif  //ECHO_TO_SERIAL
       }
    // use debugging LEDs
    pinMode(redLEDpin, OUTPUT);
    pinMode(greenLEDpin, OUTPUT);
    //let's throw some header cells on here (edit as needed for each column's intended data)
    logFile.println(F("millis,stamp,datetime,asdx,temp,vcc"));    
        #if ECHO_TO_SERIAL
    Serial.println(F("millis,stamp,datetime,asdx,temp,vcc"));
        #endif //ECHO_TO_SERIAL
    delay(100);
/*MPL PRESS SENSOR STARTUP*/
    //none needed according to example. Uses 9600 baudrate Serial BTW
/*P.W.F. MAX RTD BREAKOUT STARTUP*/
    //Uses 115200 baudrate Serial BTW
    // setup for the the SPI library:
    SPI.begin();                            // begin SPI
    SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
    SPI.setDataMode(SPI_MODE3);             // MAX31865 works in MODE1 or MODE3
    pinMode(RTDPin, OUTPUT);                // initalize the chip select pin
    rtd_ch0.MAX31865_config();
    delay(100);                             // give the sensor time to set up
 
   
/*GPS STARTUP*/
    //Serial.begin(9600);   //@ 115200 to read GPS fast enuf & echo w/o dropping chars, also enuf time to spit it out
    Serial.println(F("Basic Test - GPS"));
    GPS.begin(9600);       // 9600 NMEA is default baud rate for Adafruit MTK GPS's- some use 4800
    // uncomment this line below to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  //ON
    // uncomment this line below to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //For parsing data, don't suggest using anything but either RMC only or RMC+GGA since the parser doesn't care about other sentences at this time
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   //Update Rate: 1 Hz update rate for parsed code to play nice and have time to sort data and print.
    GPS.sendCommand(PGCMD_ANTENNA); //request updates on antenna status, comment out to keep quiet
    useInterrupt(true); //can have a timer0 interrupt go off every 1 millisec, and read data from GPS for you. Makes loop easier
    delay(1000);
 
    tone(5,2000,1000);//setup complete
    delay(1000);      //play some noise
}
 
/*OMG MORE GPS STUFF*/
    SIGNAL(TIMER0_COMPA_vect) { //Interrupt is called once a millisec, looks for new GPS data, & stores it
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    #ifdef UDR0
    if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time.
    #endif
    }
   
    void useInterrupt(boolean v) {
    if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
    } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
    }
    }
 
uint32_t timer = millis();
   
void loop() {
 
  DateTime now;
 
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
 
  //log milliseconds since starting
  uint32_t m = millis();
  logFile.print(m);                  //milliseconds since arduino power-on cell
  logFile.print(F(", "));
  //fetch RTC time
  now = RTC.now();
  logFile.print(now.unixtime());     //seconds since 1/1/1970 cell- 'stamp' unix epoch time (in Excel try =CELL/(60*60*24)+"1/1/1970" to convert)
  logFile.print(F(", "));
  logFile.print(now.hour(), DEC);
  logFile.print(F(":"));
  logFile.print(now.minute(), DEC);  //hour:minute:second time cell
  logFile.print(F(":"));
  logFile.print(now.second(), DEC);
  logFile.print(F(", "));
 
  //read ASDX pressure
  analogRead(ASDXPin);
  delay(10);
  int ASDXReading = analogRead(ASDXPin);
  //read TMP temp
  analogRead(TMPPin);
  delay(10);
  int TMPReading = analogRead(TMPPin);
 
  //convert that TMP reading to voltage
  float voltage = TMPReading * aref_voltage / 1024 ;
  float temperatureC = (voltage - 0.5) * 100 ;        //this can be done in Excel if we run out of space
 
  //log analog sensor readings
  logFile.print(ASDXReading);          //ASDX pressure sensor cell
  logFile.print(F(", "));
  logFile.print(temperatureC);         //TMP36 temperature cell
  logFile.print(F(", "));
 
  //log estimated VCC voltage by measuring against internal 1.1v ref
  analogRead(BANDGAPREF);
  delay(10);
  int refReading = analogRead(BANDGAPREF);
  float supplyvoltage = (bandgap_voltage * 1024) / refReading;
  logFile.print(supplyvoltage);        //reference voltage cell
  logFile.print(F(", "));                
 
 
 
 
 
  //let's go to the next line
  logFile.println();
  //sync data to the card & update the FAT
  digitalWrite(redLEDpin, HIGH);
  logFile.flush();
  tone(5,1000,5); //we loggin' so make some noize syntax: tone(pin5,1000hz,5ms)
  digitalWrite(redLEDpin, LOW);
   
}
