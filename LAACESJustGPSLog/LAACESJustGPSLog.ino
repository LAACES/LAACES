#include <SPI.h>                 //Adafruit_SD-> ReadWrite/sheild_sdlog
#include <SD.h>                  //Adafuit_SD-> ReadWrite/sheild_sdlog
#include <Wire.h>                //MPL3115A2/BMP180/Logger since I2C
#include <avr/sleep.h>           //Adafruit_GPS-> shield_sdlog
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "RTClib.h"

/*logger*/
#define LOG_INTERVAL 5000 //how long do we want between log intervals (in ms)?
uint32_t syncTime = 0; // time of last sync()
File logFile;             //For logger: insert filename here
const int chipSelect = 10; //Pin 10 for logger
RTC_DS1307 RTC;            //define logger RTC object
// the digital pins that connect to the LEDs
#define redLEDpin 3
#define greenLEDpin 4

/*GPS*/
SoftwareSerial mySerial(7, 6); //can change pin numbers to match wiring
Adafruit_GPS GPS(&mySerial);   //create the GPS object
#define GPSECHO  false          //debug/listen to raw GPS. Set to false for silence.
//boolean usingInterrupt = false; // this keeps track of whether we're using interrupt off by default
//void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
String NMEA1; //Variable for first NMEA sentence
String NMEA2; //Variable for second NMEA sentence
char c; //to read characters coming from the GPS
float deg; //Will hold positin data in simple degree format
float degWhole; //Variable for the whole part of position 
float degDec;  //Variable for the decimal part of degree


void setup() {

/*LOGGER STARTUP*/
//    pinMode(SS, OUTPUT); //set pin 10 (SS SPI Pin) to OUTPUT mode
//    if (!SD.begin(chipSelect)) {
//    Serial.println(F("Card failed, or not present"));
// //   tone(5,8000,5000); 
// //   delay(5000);  //play some noise
//    //while (1) ; // don't do anything more
//    }
  pinMode(10, OUTPUT); //Must declare 10 an output and reserve it to keep SD card happy
  SD.begin(chipSelect); //Initialize the SD card reader
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
    Serial.println(F(" If no card failed msg, card is initialized."));     
    // use debugging LEDs
    pinMode(redLEDpin, OUTPUT);
    pinMode(greenLEDpin, OUTPUT);
    //let's throw some header cells on here (edit as needed for each column's intended data)
    logFile.println(F("Millis,Stamp,LogTime,InternalVolt,SupplyVolt,ASDX,TMP36,MPLPress,MPLAlt,MPLTemp,GPSTime,GPSFix,FixQual,Lat(N/S),Lat(W/E),LatDeg,LonDeg,Speed,Angle,GPSAlt,Sats,FreeMemory"));    
    delay(100);
    logFile.flush();
    
    
    /*GPS STARTUP*/ 
    //Serial.begin(9600);   //@ 115200 to read GPS fast enuf & echo w/o dropping chars, also enuf time to spit it out
    GPS.begin(9600);       // 9600 NMEA is default baud rate for Adafruit MTK GPS's- some use 4800
    // uncomment this line below to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand("$PGCMD,33,0*6D");  //Turn off antenna update nuisance data
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  //ON
    // uncomment this line below to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //For parsing data, don't suggest using anything but either RMC only or RMC+GGA since the parser doesn't care about other sentences at this time
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   //Update Rate: 1 Hz update rate for parsed code to play nice and have time to sort data and print.
    //GPS.sendCommand(PGCMD_ANTENNA); //request updates on antenna status, comment out to keep quiet
    //useInterrupt(true); //can have a timer0 interrupt go off every 1 millisec, and read data from GPS for you. Makes loop easier
    delay(1000);
    
    
    
}


uint32_t timer = millis();

void loop() {

DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  uint32_t m = millis();
  
    readGPS();

  if(GPS.fix==1) { //Only save data if we have a fix
//  logFile = SD.open(filename, FILE_WRITE); //Open file on SD card for writing
  logFile.println(NMEA1); //Write first NMEA to SD card
  logFile.println(NMEA2); //Write Second NMEA to SD card
//  logFile.close();  //Close the file
  
 
//  logFile = SD.open(filename2, FILE_WRITE);
  
  degWhole=float(int(GPS.longitude/100)); //gives me the whole degree part of Longitude
  degDec = (GPS.longitude - degWhole*100)/60; //give me fractional part of longitude
  deg = degWhole + degDec; //Gives complete correct decimal form of Longitude degrees
  if (GPS.lon=='W') {  //If you are in Western Hemisphere, longitude degrees should be negative
    deg= (-1)*deg;
  }
  logFile.print(deg,4); //writing decimal degree longitude value to SD card
  logFile.print(","); //write comma to SD card
  
  degWhole=float(int(GPS.latitude/100)); //gives me the whole degree part of latitude
  degDec = (GPS.latitude - degWhole*100)/60; //give me fractional part of latitude
  deg = degWhole + degDec; //Gives complete correct decimal form of latitude degrees
  if (GPS.lat=='S') {  //If you are in Southern hemisphere latitude should be negative
    deg= (-1)*deg;
  }
  logFile.print(deg,4); //writing decimal degree longitude value to SD card
  logFile.print(","); //write comma to SD card
  
  logFile.print(GPS.altitude); //write altitude to file
  logFile.print(" ");  //format with one white space to delimit data sets

//  logFile.close();
  }


 //let's go to the next line
  logFile.println();
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
 // if ((millis() - syncTime) < LOG_INTERVAL) return;
 // syncTime = millis();
  //sync data to the card & update the FAT
  digitalWrite(greenLEDpin, HIGH);
  delay(10);
  
  logFile.flush();
  delay(1000);
  tone(5,500,1); //we loggin' so make some noize. syntax: tone(pin5,1000hz,5ms)
  delay(10);
  digitalWrite(greenLEDpin, LOW);

}



void readGPS() {
  
  clearGPS();
  while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  NMEA1=GPS.lastNMEA();
  
   while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  NMEA2=GPS.lastNMEA();
  
  Serial.println(NMEA1);
  Serial.println(NMEA2);
  Serial.println("");
  
}

void clearGPS() {  //Clear old and corrupt data from serial port 
  while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  
  while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
   while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  
}
