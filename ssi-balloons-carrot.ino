/*  Avionics program for Balloons onboarding team Carrot.
 *   ________  ________  ________  ________  ________  _________   
 *  |\   ____\|\   __  \|\   __  \|\   __  \|\   __  \|\___   ___\ 
 *  \ \  \___|\ \  \|\  \ \  \|\  \ \  \|\  \ \  \|\  \|___ \  \_| 
 *   \ \  \    \ \   __  \ \   _  _\ \   _  _\ \  \\\  \   \ \  \  
 *    \ \  \____\ \  \ \  \ \  \\  \\ \  \\  \\ \  \\\  \   \ \  \ 
 *     \ \_______\ \__\ \__\ \__\\ _\\ \__\\ _\\ \_______\   \ \__\
 *      \|_______|\|__|\|__|\|__|\|__|\|__|\|__|\|_______|    \|__|
 *  
 *  
 *  Data downlink ideal rate: 50 bytes every 5 min
 *  
 *  To send back to ground:
 *  GPS, lat long, temp in out, altitude, ascent rate
 *  To send to balloon:
 *  Commands: cutdown, release secondary payload, 
 */

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MAX31855.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#define DEBUG // comment out to turn off debugging
#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print (x)
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Timing (Internal)
int startTime;
#define SD_CARD_FLUSH_TIME 10000 // 10 Seconds
#define ROCKBLOCK_TRANSMIT_TIME 300000 // 5 Minutes
#define BAROMETER_MEASURMENT_INTERVAL 10000 // 10 Seconds

// SPI Ports
#define SD_READER_CS 10
#define THERMOCOUPLE_CS 4

// Environemental Parameters
#define LAUNCH_SITE_PRESSURE 1014.562

// SD Card Reader (SPI)
File dataFile;
int lastFlush;

// Thermocouple (SPI) | Exterior Temperature
Adafruit_MAX31855 thermocouple(THERMOCOUPLE_CS);

// BMP280 (I2C) | Pressure, Internal Temperature
Adafruit_BMP280 bmp; 
int lastAscentTime;             // Time of last ascent rate calculation
double lastAlt;                 // Last altitude, for calculation
double ascentRate;              // Last calculated rate, to fill forward in logging

// BNO055 (Serial) IMU
Adafruit_BNO055 bno(55,0x29);

// ROCKBlock (Hardware Serial) Radio
int lastTransmit;


void setup() {
  startTime = millis();
  lastFlush = 0;
  lastTransmit = 0;

  #ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {continue;}
  #endif

  // SD Card Reader
  DEBUG_PRINTLN("Initializing SD card...");
  if (!SD.begin(SD_READER_CS)) {
    DEBUG_PRINTLN("Card failed, or not present");
    while (true) {
      // TODO: Flash LED
    }
  }
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  DEBUG_PRINTLN("Card initialized.");
 

  // BMP280
  if (!bmp.begin()) {
    DEBUG_PRINTLN("Could not find a valid BMP280 sensor, check wiring!");
    while (true) {
      // TODO: flash LED
    }
  }
  lastAlt = bmp.readAltitude(LAUNCH_SITE_PRESSURE); // initialize ascent rate variables
  ascentRate = 0;
  lastAscentTime = millis();

  // BNO055
  if (!bno.begin()) {
    DEBUG_PRINTLN("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (true) {
      // TODO: flash LED
    }
  }
  bno.setExtCrystalUse(true); // TODO figure this out

  // Data column headers. Temporary.
  dataFile.print("Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C), TempOut(C), ");
  dataFile.println("GPSLat, GPSLong, GPSAlt, ax(G), ay, az, gx, gy, gz, mx, my, mz, RockBlockStatus");
}

void loop() {
  String dataString = readSensors();
  int loopTime = millis();
  
  if (dataFile) {
    DEBUG_PRINTLN("Writing to datalog.txt");
    dataFile.println(dataString);
  } else {
    DEBUG_PRINTLN("Error opening datalog.txt");
  }
  
  if (loopTime - lastFlush > SD_CARD_FLUSH_TIME){
    DEBUG_PRINTLN("Flushing datalog.txt");
    dataFile.flush(); 
    lastFlush = loopTime;
  }

  if (loopTime - lastTransmit > ROCKBLOCK_TRANSMIT_TIME) {
    DEBUG_PRINTLN("Transmiting to ROCKBlock");
    // TODO: Send dataString to rockblock
    lastTransmit = loopTime;
  }
}

// Reads from all of the sensors and outputs the data string
String readSensors() {
  String dataString = "";

  // Timing 
  int loopTime = millis();
  dataString += loopTime + ", ";

  // BMP280 (Barometer + Thermometer) Input
  double tempIn = bmp.readTemperature();
  double pressure = bmp.readPressure();
  double alt = bmp.readAltitude(LAUNCH_SITE_PRESSURE); // avg sea level pressure for hollister for past month
  if (loopTime - lastAscentTime > BAROMETER_MEASURMENT_INTERVAL) { // calculate ascent rate in m/s every 10s
    ascentRate = (alt - lastAlt) * 1000 / (loopTime - lastAscentTime);
    lastAlt = alt;
    lastAscentTime = loopTime;
  }
  dataString += String(pressure) + ", " + String(alt) + ", ";
  dataString += String(ascentRate) + ", " + String(tempIn) + ", ";

  // BNO055 (IMU) Input
  sensors_event_t event;
  bno.getEvent(&event);
  dataString += (String)event.orientation.x + ", " + (String)event.orientation.y + ", ";
  dataString += (String)event.orientation.z + ", "; // TODO: get accel and gyro from BNO

  // MAX31855 (Thermocouple) Input
  double temperature = thermocouple.readFarenheit();
  dataString += String(temperature) + ", ";
  
  // TODO get GPS data

  // TODO talk to rockblock

  return dataString;
}

