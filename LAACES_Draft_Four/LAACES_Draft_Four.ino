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
#define BANDGAPREF 21
#define TMPPin 0     //TMP uses pin A0
#define ASDXPin 1    //ASDX uses pin A1
#define vBattPin 2   //Battery Voltage reference
#define v5Pin 3      //5V reference from rail
#define RTDPin  9    //For RTD
SFE_BMP180 BMPress;        //pressure object for BMP180
#define ALTITUDE 1655.0    //for baseline altitude from launch site.
PWFusion_MAX31865_RTD rtd_ch0(RTDPin);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
RTC_DS1307 RTC;            //define logger RTC object
#define LOG_INTERVAL 3000 //how long do we want between log intervals (in ms)?

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
  
  /*AREF STARTUP*/
  analogReference(EXTERNAL);
  
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
    
    //read 5V rail
    int v5Reading = analogRead(v5Pin);
    float v5Percent = (v5Reading) ;
    v5Percent /= 1024.0;
    
    //read ASDX pressure
    int ASDXReading = analogRead(ASDXPin);
    float ASDXVoltage = (ASDXReading * 4.89 ) ;
    ASDXVoltage /= 1024.0;
    
    //read interval voltage
    int refReading = analogRead(BANDGAPREF);
    //calculate power supply voltage from known 1.05V reading
    float supplyvoltage = (1024/refReading); //actual voltage of "1.1V" AREF internal reference.
    
    //read vBatt pin
    int vDivider = analogRead(vBattPin);
    
    //read TMP temp
    int TMPReading = analogRead(TMPPin);
    //convert that TMP reading to voltage
    float TMPvoltage = (TMPReading * 4.89 ) ;  //alternatively if you figure out your supply voltage put that variable in place of 3.28V or 4.74V
    TMPvoltage /= 1024.0;
    float temperatureC = (TMPvoltage - 0.5) * 100 ;//this can be done in Excel if we run out of space
 
    float pascals = baro.getPressure();
    float altm = baro.getAltitude();   
    float tempC = baro.getTemperature();

    //RTD Reading
    SPI.setDataMode(SPI_MODE3);             // MAX31865 works in MODE1 or MODE3  //logger wont work in mode 3!
    static struct var_max31865 RTD_CH0;
    double tmp;    
    RTD_CH0.RTD_type = 2;
    struct var_max31865 *rtd_ptr;
    rtd_ptr = &RTD_CH0;
    rtd_ch0.MAX31865_full_read(rtd_ptr);    // Update MAX31855 readings 
    delay(100);
    SPI.setDataMode(SPI_MODE0);             // back to MODE0 for the SD logger to work
    // calculate RTD temperature (simple calc, +/- 2 deg C from -100C to 100C)
    // more accurate curve can be used outside that range   
    tmp = (double)RTD_CH0.rtd_res_raw * 4000 / 32768; //<--resistance
    
    
     // approximately every 5 seconds or so, print out the current stats
     if (millis() - timer > LOG_INTERVAL) { 
        timer = millis(); // reset the timer
      
    digitalWrite(GrnledPin,HIGH);
    
    logFile.print(vDivider);
    logFile.print(F(", "));
    logFile.print(vBatt(vDivider));
    logFile.print(F(", "));
    logFile.print(v5Reading);          
    logFile.print(F(", "));
    logFile.print(v5Percent);          
    logFile.print(F(", "));
    logFile.print(TMPReading);          
    logFile.print(F(", "));
    logFile.print(refReading);           //internal voltage
    logFile.print(F(", "));              //if we cant get these going right, let's put vBattery here.
    logFile.print(supplyvoltage);        //power supply voltage compared to known 1.05V reading
    logFile.print(F(", "));
    logFile.print(ASDXReading);          //ASDX pressure sensor cell
    logFile.print(F(", "));
    logFile.print(ASDXVoltage);          //ASDX voltage sensor cell
    logFile.print(F(", "));
    logFile.print(temperatureC);         //TMP36 temperature cell
    logFile.print(F(", "));
    logFile.print(tmp);                  //RTD resistance cell
    logFile.print(F(", "));
    tmp = ((double)RTD_CH0.rtd_res_raw / 32) - 256;
    logFile.print(tmp);                  //RTD temperature cell
    logFile.print(F(", "));   
    logFile.print(tempC);                //MPL temp in celcius  
    logFile.print(F(", "));
    logFile.print(pascals/3377);         //MPL pressure cell in Hg. If you want pascals erase the /3377
    logFile.print(F(", "));
    logFile.print(altm);                 //MPL altimeter cell
    logFile.print(F(", "));   
    logFile.print(GPS.hour);
    logFile.print(F(":"));
    if (GPS.minute < 10) logFile.print(F("0"));
    logFile.print(GPS.minute);
    logFile.print(F(":"));
    if (GPS.seconds < 10) logFile.print(F("0"));
    logFile.print(GPS.seconds);
    logFile.print(F(","));   
    logFile.print(GPS.latitude);
    logFile.print(F(","));
    logFile.print(GPS.longitude);
    logFile.print(F(","));
    logFile.print((int)GPS.altitude);
    logFile.print(F(","));
    logFile.print(GPS.speed);
    logFile.print(F(","));
    logFile.println("");
    logFile.flush();
    digitalWrite(GrnledPin,LOW);
    
#ifdef MEM_DEBUG
    Serial.println(freeRam());
#endif     
     }
}

    double vBatt(int vBattIn){
    double vBatt = (vBattIn/1023.)*32.2;
    return vBatt;
    }
