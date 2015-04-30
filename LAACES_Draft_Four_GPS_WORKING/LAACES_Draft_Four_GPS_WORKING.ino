#include <SD.h> //Load SD card library
#include <SPI.h> //Load SPI Library
#include <Adafruit_GPS2.h>    //Install the adafruit GPS library
#include <SoftwareSerial.h> //Load the Software Serial library
#include <Wire.h>                //MPL3115A2/BMP180 since I2C
#include <Adafruit_MPL3115A2.h>  //Adafruit_MPL3115aA2-> testmpl3115a2
#include <avr/sleep.h>           //Adafruit_GPS-> shield_sdlog
#include <SFE_BMP180.h>          //Sparkfun_BMP180-> SFE_BMP180
//#include "MPL3115A2.h"           //Sparkfun_MPL3115A2 (!!If Using SF Lib!!)
#include <PlayingWithFusion_MAX31865.h>        //PWF-> core library
#include <PlayingWithFusion_MAX31865_STRUCT.h> //PWF-> struct library
#include "RTClib.h"

#define chipSelect 10
#define ledPin 3
File logFile; //Data object you will write your sesnor data to
SoftwareSerial mySerial(7,6); //Initialize the Software Serial port
Adafruit_GPS GPS(&mySerial); //Create the GPS Object
boolean usingInterrupt = false;
uint32_t timer = millis();


//BEGIN HELPER FUNCTIONS

//Helper function for memory debugging
//uncomment the line below to enable serial output for memory use debugging
#define MEM_DEBUG true
#ifdef MEM_DEBUG
int freeRam ()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif


//GPS interrupt code
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
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
//END HELPER FUNCTIONS


void setup() {
   Serial.begin(115200); // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);
  
  #ifdef MEM_DEBUG
  Serial.println(freeRam());
  #endif
  
  
  //see if the card is present and can be initialized. Spinlock if it fails.
  if (!SD.begin(chipSelect)) {
    digitalWrite(ledPin, HIGH); 
    while(1){};
  }
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    //create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  //open up the logFile, spinlock if it fails.
  logFile = SD.open(filename, FILE_WRITE);
  if( ! logFile ) {
    digitalWrite(ledPin,HIGH);
    while(1){};
  }

  //enable GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_NOANTENNA);
  useInterrupt(true);

#ifdef MEM_DEBUG
   Serial.println(freeRam());
#endif

  //print logFile header
  logFile.println(F("time_utc,lat,long,altitude_M,speed_knots,kPa_out,tempC_out,tempC_in"));  


  
} //END SETUP

void loop() {
  
    //these lines parse the data
    GPS.parse(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

     // approximately every 5 seconds or so, print out the current stats
     if (millis() - timer > 5000) { 
        timer = millis(); // reset the timer

    char s[10];
    //write all data to logFile
    digitalWrite(ledPin,HIGH);
    logFile.print(GPS.hour);
    logFile.print(F(":"));
    if (GPS.minute < 10) logFile.print(F("0"));
    logFile.print(GPS.minute);
    logFile.print(F(":"));
    if (GPS.seconds < 10) logFile.print(F("0"));
    logFile.print(GPS.seconds);
    logFile.print(F(","));   
    char s1[10];
    logFile.print(dtostrf(GPS.latitude,5,5,s));
    logFile.print(F(","));
    logFile.print(dtostrf(GPS.longitude,5,5,s));
    logFile.print(F(","));
    logFile.print((int)GPS.altitude);
    logFile.print(F(","));
    logFile.print(dtostrf(GPS.speed,2,2,s));
    logFile.print(F(","));
    logFile.println("");
    logFile.flush();
    digitalWrite(ledPin,LOW);
    
#ifdef MEM_DEBUG
    Serial.println(freeRam());
#endif     
     }
}

