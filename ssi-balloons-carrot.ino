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

int startTime;

File dataFile;
const int chipSelect = 10; // untested. try 4 if not working
int lastFlush; // last time the SD card was flushed

Adafruit_BMP280 bmp; // I2C
const double launchSitePressure = 1014.562;
int lastAscentTime; // time of last ascent rate calculation
double lastAlt; // last altitude, for calculation
double ascentRate; // last calculated rate, to fill forward in logging

//rockblock
int lastTransmit;

void setup() {
  startTime = millis(); // record the start time
  lastFlush = 0;
  lastTransmit = 0;

  // initialize serial if debugging
  #ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial connection to computer to open
  }
  #endif

  // initialize the sd card
  DEBUG_PRINTLN("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    DEBUG_PRINTLN("Card failed, or not present");
    while (true) {
      // TODO: flash LED
    }
  }
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  DEBUG_PRINTLN("Card initialized.");
  // Data column headers. Temporary.
  dataFile.print("Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C), TempOut(C), ");
  dataFile.println("GPSLat, GPSLong, GPSAlt, ax(G), ay, az, gx, gy, gz, mx, my, mz, RockBlockStatus");

  if (!bmp.begin()) {
    DEBUG_PRINTLN("Could not find a valid BMP280 sensor, check wiring!");
    while (true) {
      // TODO: flash LED
    }
  }
  lastAlt = bmp.readAltitude(launchSitePressure); // initialize ascent rate variables
  ascentRate = 0;
  lastAscentTime = millis();
}

void loop() {
  int loopTime = millis(); // time of loop start, used synchronize time calculations within loop
  String dataString = ""; // accumulate data using dataString += ...
  dataString += loopTime + ", ";

  // read data from bmp
  double tempIn = bmp.readTemperature();
  double pressure = bmp.readPressure();
  double alt = bmp.readAltitude(launchSitePressure); // avg sea level pressure for hollister for past month
  if (loopTime - lastAscentTime > 10000) { // calculate ascent rate in m/s every 10s
    ascentRate = (alt - lastAlt) * 1000 / (loopTime - lastAscentTime);
    lastAlt = alt;
    lastAscentTime = loopTime;
  }
  dataString += String(pressure) + ", " + String(alt) + ", ";
  dataString += String(ascentRate) + ", " + String(tempIn) + ", ";

  // write out data to the file if available
  if (dataFile) {
    dataFile.println(dataString);
  } else {
    DEBUG_PRINTLN("Error opening datalog.txt");
  }
  if (loopTime - lastFlush > 10000){
    dataFile.flush(); // flush the SD car buffer every 10s
    lastFlush = loopTime;
  }

  // send back rockblock data every 5 mins
  if (loopTime - lastTransmit > 300000) {
    // TODO: send dataString to rockblock
    lastTransmit = loopTime;
  }
}

