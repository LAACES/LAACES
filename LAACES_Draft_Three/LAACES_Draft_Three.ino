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

SoftwareSerial mySerial(7,6); //Initialize the Software Serial port
Adafruit_GPS GPS(&mySerial); //Create the GPS Object
String NMEA1; //Variable for first NMEA sentence
String NMEA2; //Variable for second NMEA sentence
char c; //to read characters coming from the GPS
float deg; //Will hold positin data in simple degree format
float degWhole; //Variable for the whole part of position 
float degDec;  //Variable for the decimal part of degree

int chipSelect = 10; //chipSelect pin for the SD card Reader
File logFile; //Data object you will write your sesnor data to


void setup() {
  
  Serial.begin(115200); //Turn on serial monitor
  GPS.begin(9600); //Turn on GPS at 9600 baud
  GPS.sendCommand("$PGCMD,33,0*6D");  //Turn off antenna update nuisance data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Request RMC and GGA Sentences only
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Set update rate to 1 hz
  delay(1000); 
  
  pinMode(10, OUTPUT); //Must declare 10 an output and reserve it to keep SD card happy
  SD.begin(chipSelect); //Initialize the SD card reader
  
//  if (SD.exists("NMEA.txt")) { //Delete old data files to start fresh
//    SD.remove("NMEA.txt");
//  }
//  if (SD.exists("GPSData.txt")) { //Delete old data files to start fresh
//    SD.remove("GPSData.txt");
//  }

}

void loop() {
  
  readGPS();

  if(GPS.fix==1) { //Only save data if we have a fix
  logFile = SD.open("NMEA.txt", FILE_WRITE); //Open file on SD card for writing
  logFile.println(NMEA1); //Write first NMEA to SD card
  logFile.println(NMEA2); //Write Second NMEA to SD card
  logFile.close();  //Close the file
  
 
  logFile = SD.open("GPSData.txt", FILE_WRITE);
  
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

  logFile.close();
  }
  
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
