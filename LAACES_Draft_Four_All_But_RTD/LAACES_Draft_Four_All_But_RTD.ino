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
#define GrnledPin 3
#define RedledPin 4
#define vDividerPin A2 //voltage divider input from Vbatt
#define BANDGAPREF 21
#define TMPPin 0     //TMP uses pin A0
#define ASDXPin 1    //ASDX uses pin A1
SFE_BMP180 BMPress;        //pressure object for BMP180
#define ALTITUDE 1655.0    //for baseline altitude from launch site.
const int RTDPin = 9;      //for CS on RTD breakout
PWFusion_MAX31865_RTD rtd_ch0(RTDPin);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
RTC_DS1307 RTC;            //define logger RTC object
#define LOG_INTERVAL 5000 //how long do we want between log intervals (in ms)?

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
  pinMode(GrnledPin, OUTPUT);
  pinMode(RedledPin, OUTPUT);
  pinMode(10, OUTPUT);
  
  #ifdef MEM_DEBUG
  Serial.println(freeRam());
  #endif
  
  
  //see if the card is present and can be initialized. Spinlock if it fails.
  if (!SD.begin(chipSelect)) {
    digitalWrite(RedledPin, HIGH); 
    while(1){};
  }
   // create a NEW file
  char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
//        logFile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
       }
//  char filename[15];
//  strcpy(filename, "GPSLOG00.TXT");
//  for (uint8_t i = 0; i < 100; i++) {
//    filename[6] = '0' + i/10;
//    filename[7] = '0' + i%10;
//    //create if does not exist, do not open existing, write, sync after write
//    if (! SD.exists(filename)) {
//      break;
//    }
  }
  //open up the logFile, spinlock if it fails.
  logFile = SD.open(filename, FILE_WRITE);
  if( ! logFile ) {
    digitalWrite(GrnledPin,HIGH);
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

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
     logFile.println(F("RTC failed")); //THIS if MAY NOT BE NECESSARY, replace with j/ RTC.begin()
  }
  
  baro.begin(); //Begin the MPL3115A2
  
  /*P.W.F. MAX RTD BREAKOUT STARTUP*/
    //Uses 115200 baudrate Serial BTW
    // setup for the the SPI library:
    SPI.begin();                            // begin SPI
    SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
    SPI.setDataMode(SPI_MODE3);             // MAX31865 works in MODE1 or MODE3  //logger wont work in mode 3!
    pinMode(RTDPin, OUTPUT);                // initalize the chip select pin    //and pretends to work in mode 1
    rtd_ch0.MAX31865_config();
    delay(100);                             // give the sensor time to set up
    SPI.setDataMode(SPI_MODE0);             // back to MODE0 for the SD logger to work
//NOTES ON RTD/SD MODE MIXING: I took a look at the datasheet for your MAX devices, and they will work with SPI mode 3 or mode 0. The SPI timing on the datasheet shows the clock can be either polarity, and that is the difference between the modes. Page 5 here shows this: http://datasheets.maximintegrated.com/en/ds/MAX31865.pdf The SD card is not so flexible. My tests show they require mode 0.

} //END SETUP

void loop() {
  
    DateTime now;
  
    //these lines parse the data
    GPS.parse(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();
    
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
 
    float pascals = baro.getPressure();
    float altm = baro.getAltitude();   
    float tempC = baro.getTemperature();

    
     // approximately every 5 seconds or so, print out the current stats
     if (millis() - timer > LOG_INTERVAL) { 
        timer = millis(); // reset the timer
      
      digitalWrite(GrnledPin,HIGH);
      
 // char s0[10];
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
 // char s1[10];
  //log MPL3115A2 reading
//  float pascals = baro.getPressure();
  logFile.print(pascals/3377); //pressure in Hg. If you want pascals erase the /3377
  logFile.print(F(", "));
//  delay(10);                                      //MPL is causing a hang for some reason. czeck that out...
//  float altm = baro.getAltitude();
  logFile.print(altm);        //altimeter reading
  logFile.print(F(", "));   
//  delay(10);
  
//  float tempC = baro.getTemperature();
  logFile.print(tempC);       //MPL temp in celcius  
  logFile.print(F(", "));   
//  delay(10);

 //   char s2[10];
    //write all data to logFile

    logFile.print(GPS.hour);
    logFile.print(F(":"));
    if (GPS.minute < 10) logFile.print(F("0"));
    logFile.print(GPS.minute);
    logFile.print(F(":"));
    if (GPS.seconds < 10) logFile.print(F("0"));
    logFile.print(GPS.seconds);
    logFile.print(F(","));   
 //   char s3[10];
    logFile.print(/*dtostrf*/(GPS.latitude/*,5,5,s3*/));
    logFile.print(F(","));
    logFile.print(/*dtostrf*/(GPS.longitude/*,5,5,s3*/));
    logFile.print(F(","));
    logFile.print((int)GPS.altitude);
    logFile.print(F(","));
    logFile.print(/*dtostrf*/(GPS.speed/*,2,2/*,s3*/));
    logFile.print(F(","));
    logFile.println("");
    logFile.flush();
    digitalWrite(GrnledPin,LOW);
    
#ifdef MEM_DEBUG
    Serial.println(freeRam());
#endif     
     }
}

