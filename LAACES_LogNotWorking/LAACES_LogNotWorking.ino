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
#include <MemoryFree.h>

#define BANDGAPREF 21  //(originally 14)for measuring bandcap voltage
#define aref_voltage 3.28
const int vDividerPin = A2; //voltage divider input from Vbatt

/*logger*/
#define LOG_INTERVAL 5000 //how long do we want between log intervals (in ms)?
//uint32_t syncTime = 0; // time of last sync()
File logFile;             //For logger: insert filename here
const int chipSelect = 10; //Pin 10 for logger
RTC_DS1307 RTC;            //define logger RTC object
// the digital pins that connect to the LEDs
#define redLEDpin 3
#define greenLEDpin 4

/*TMP36*/
#define TMPPin 0     //TMP uses pin A0

/*ASDX Pressure*/
const int ASDXPin = A1;    //ASDX uses pin A1

/*SF BMP180*/
SFE_BMP180 BMPress;        //pressure object for BMP180
#define ALTITUDE 1655.0    //for baseline altitude from launch site.

/*P.W.F. MAX RTD*/
const int RTDPin = 9;      //for CS on RTD breakout
PWFusion_MAX31865_RTD rtd_ch0(RTDPin);

/*MPL3115A2 Press/Alt/Temp sensor*/
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

/*GPS*/
SoftwareSerial mySerial(7, 6); //can change pin numbers to match wiring
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false          //debug/listen to raw GPS. Set to false for silence.
boolean usingInterrupt = false; // this keeps track of whether we're using interrupt off by default
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
//#define LOG_FIXONLY false  



void setup() {
Serial.begin(9600);

/*AREF STARTUP*/
// analogReference(INTERNAL);
  
/*BMP180 STARTUP*/ 
//    BMPress.begin();
//    delay(100);


/*LOGGER STARTUP*/
    pinMode(SS, OUTPUT); //set pin 10 (SS SPI Pin) to OUTPUT mode
    if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
 //   tone(5,8000,5000); 
 //   delay(5000);  //play some noise
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
    Serial.println(F(" If no card failed msg, card is initialized."));    
     // connect to RTC
      Wire.begin();  
        if (!RTC.begin()) {
          logFile.println(F("RTC failed"));          //THIS STUFF SHOULD NOT BE NECESSARY
      #if ECHO_TO_SERIAL
          Serial.println(F("RTC failed"));
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
     baro.begin();    
     
/*P.W.F. MAX RTD BREAKOUT STARTUP*/
    //Uses 115200 baudrate Serial BTW
    // setup for the the SPI library:
    SPI.begin();                            // begin SPI
    SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
    SPI.setDataMode(SPI_MODE3);             // MAX31865 works in MODE1 or MODE3
    pinMode(RTDPin, OUTPUT);                // initalize the chip select pin
//    rtd_ch0.MAX31865_config();
    delay(100);                             // give the sensor time to set up
    
/*GPS STARTUP*/ 
    //Serial.begin(9600);   //@ 115200 to read GPS fast enuf & echo w/o dropping chars, also enuf time to spit it out
    GPS.begin(9600);       // 9600 NMEA is default baud rate for Adafruit MTK GPS's- some use 4800
    // uncomment this line below to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  //ON
    // uncomment this line below to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //For parsing data, don't suggest using anything but either RMC only or RMC+GGA since the parser doesn't care about other sentences at this time
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   //Update Rate: 1 Hz update rate for parsed code to play nice and have time to sort data and print.
    GPS.sendCommand(PGCMD_ANTENNA); //request updates on antenna status, comment out to keep quiet
    useInterrupt(true); //can have a timer0 interrupt go off every 1 millisec, and read data from GPS for you. Makes loop easier
    delay(1000);
    tone(5,2000,100);//setup complete
    delay(100);      //play some noise
      
    
}/*END OF setup();*/


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
  logFile.print(':');
  logFile.print(now.minute(), DEC);  //hour:minute:second time cell
  logFile.print(':');
  logFile.print(now.second(), DEC);
  logFile.print(F(", "));
  
  //read ASDX pressure
  int ASDXReading = analogRead(ASDXPin);
  float ASDXVoltage = ASDXReading * 1.09 / 1024 ;
  
  //read interval voltage
  float refReading = analogRead(BANDGAPREF);
  //calculate power supply voltage from known 1.05V reading
  float supplyvoltage = (1024/refReading); //actual voltage of "1.1V" AREF internal reference.
  
  //read TMP temp
  int TMPReading = analogRead(TMPPin);
  //convert that TMP reading to voltage
  float TMPvoltage = TMPReading * 1.09 / 1024 ;  //alternatively if you figure out your supply voltage put that variable in place of 3.28V or 4.74V
  float temperatureC = (TMPvoltage - 0.5) * 100 ;        //this can be done in Excel if we run out of space

  //log analog sensor readings
  logFile.print(refReading);           //internal voltage
  logFile.print(F(", "));                          //if we cant get these going right, let's put vBattery here.
  logFile.print(supplyvoltage);        //power supply voltage compared to known 1.05V reading
  logFile.print(F(", "));
  logFile.print(ASDXReading);          //ASDX pressure sensor cell
  logFile.print(F(", "));
  logFile.print(ASDXVoltage);
  logFile.print(F(", "));
  logFile.print(temperatureC);         //TMP36 temperature cell
  logFile.print(F(", "));

  //log MPL3115A2 reading
  float pascals = baro.getPressure();
  logFile.print(pascals/3377); //pressure in Hg. If you want pascals erase the /3377
  logFile.print(F(", "));
  delay(10);                                                                           //MPL is causing a hang for some reason. czeck that out...
  float altm = baro.getAltitude();
  logFile.print(altm);        //altimeter reading
  logFile.print(F(", "));   
  delay(10);
  
  float tempC = baro.getTemperature();
  logFile.print(tempC);       //MPL temp in celcius  
  logFile.print(F(", "));   
  delay(10);
  
  //log GPS readings...
 // {
 //   if(!usingInterrupt){     // read data from the GPS in the 'main loop'
 //     char c = GPS.read();   // if you want to debug, this is also a good time to do it!
//      if(GPSECHO)                                                                                //some of this may not be necessary
//         if(c) Serial.print(c);
//      } 
 // if(GPS.newNMEAreceived()){   // if a sentence is received, we can check the checksum, parse it...
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
 //   if(!GPS.parse(GPS.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
    //  return;      // we can fail to parse a sentence in which case we should just wait for another
//    goto SkipGPS;
//      }    
 //  if (timer > millis()) timer = millis();    // if millis() or timer wraps around, we'll just reset it
   // approximately every 2 seconds or so, print out the current stats
//   if (millis() - timer > 2000) { 
//      timer = millis(); // reset the timer
    logFile.print(GPS.hour, DEC); 
    logFile.print(':');
    logFile.print(GPS.minute, DEC); 
    logFile.print(':');                    //GPS time cell H:M:S.ms
    logFile.print(GPS.seconds, DEC); 
    logFile.print('.');
    logFile.print(GPS.milliseconds);
    logFile.print(F(", "));
    logFile.print((int)GPS.fix);          //GPS fix cell
    logFile.print(F(", ")); 
    logFile.print((int)GPS.fixquality); 
    logFile.print(F(", "));               //GPS fix quality cell
 //   if (GPS.fix) {
      logFile.print(GPS.latitude, 4);     //Lat(N/S) cell
      logFile.print(GPS.lat);
      logFile.print(F(", ")); 
      logFile.print(GPS.longitude, 4);    //Lon(W/E) cell
      logFile.print(GPS.lon);
      logFile.print(F(", "));
      logFile.print(GPS.latitudeDegrees, 5);    //LatDeg cell
      logFile.print(F(", ")); 
      logFile.print(GPS.longitudeDegrees, 5); //LonDeg cell 
      logFile.print(F(", "));
      logFile.print(GPS.speed);          //Speed cell
      logFile.print(F(", "));                                   
      logFile.print(GPS.angle);          //Angle cell                      
      logFile.print(F(", "));                            
      logFile.print(GPS.altitude);      //Altitude cell
      logFile.print(F(", ")); 
      logFile.print((int)GPS.satellites);    //Satellites cell
      logFile.print(F(", "));
      tone(5,10000,90);
      delay(100);
 //     goto NoSkipGPS;
 //   }
//   }
//  }
  
//  SkipGPS:
//  tone(5,1000,500);
//  NoSkipGPS:

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
  delay(100);
  tone(5,500,1); //we loggin' so make some noize. syntax: tone(pin5,1000hz,5ms)
  delay(10);
  digitalWrite(greenLEDpin, LOW);
    
}

