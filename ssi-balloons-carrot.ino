/*  Avionics program for Balloons onboarding team Carrot.
     ________  ________  ________  ________  ________  _________
    |\   ____\|\   __  \|\   __  \|\   __  \|\   __  \|\___   ___\
    \ \  \___|\ \  \|\  \ \  \|\  \ \  \|\  \ \  \|\  \|___ \  \_|
     \ \  \    \ \   __  \ \   _  _\ \   _  _\ \  \\\  \   \ \  \
      \ \  \____\ \  \ \  \ \  \\  \\ \  \\  \\ \  \\\  \   \ \  \
       \ \_______\ \__\ \__\ \__\\ _\\ \__\\ _\\ \_______\   \ \__\
        \|_______|\|__|\|__|\|__|\|__|\|__|\|__|\|_______|    \|__|


    Data downlink ideal rate: 50 bytes every 5 min

    To send back to ground:
    GPS, lat long, temp in out, altitude, ascent rate
    To send to balloon:
    Commands: cutdown, release secondary payload,
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MAX31855.h>
#include <IridiumSBD.h>
#include <TinyGPS++.h>
#include "SD.h"
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
long startTime;
#define SD_CARD_FLUSH_TIME 10000 // 10 Seconds
#define ROCKBLOCK_TRANSMIT_TIME 300000 // 5 Minutes
#define BAROMETER_MEASURMENT_INTERVAL 10000 // 10 Seconds

// SPI Ports
#define SD_READER_CS 10
#define THERMOCOUPLE_CS 9

// Environemental Parameters
#define LAUNCH_SITE_PRESSURE 1014.562

// SD Card Reader (SPI)
File dataFile;
long lastFlush;

// Thermocouple (SPI) | Exterior Temperature
Adafruit_MAX31855 thermocouple(THERMOCOUPLE_CS);

// BMP280 (I2C) | Pressure, Internal Temperature
Adafruit_BMP280 bmp;
long lastAscentTime;             // Time of last ascent rate calculation
double lastAlt;                 // Last altitude, for calculation
double ascentRate;              // Last calculated rate, to fill forward in logging

// BNO055 (Serial) IMU
Adafruit_BNO055 bno(55);

// ROCKBlock (Hardware Serial) Radio
long lastTransmit;
#define IridiumSerial Serial3
IridiumSBD modem(IridiumSerial);

// GPS (Hardware Serial) GPS
TinyGPSPlus gps;
float f_lat = 0, f_long = 0;
int sats = -1;
long unsigned f_age = 0;

// FET pin assignments
const int topCut = 23;
const int bottomCut = 22;
const int heater = 2;
long topCutStart = -100000;
long bottomCutStart = -100000;
bool applyHeat = false;

void setup() {
  startTime = millis();
  lastFlush = 0;
  lastTransmit = 0;

  // set FET gates to LOW
  pinmode(topCut, OUTPUT);
  pinmode(bottomCut, OUTPUT);
  pinmode(heater, OUTPUT);
  digitalWrite(topCut, LOW);
  digitalWrite(bottomCut, LOW);
  digitalWrite(heater, LOW);

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {
    continue;
  }
#endif

  // SD Card Reader
  pinMode(SD_READER_CS, OUTPUT);
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

  // GPS
  Serial1.begin(9600);

  pinMode(THERMOCOUPLE_CS, OUTPUT);

  // Data column headers. Temporary.
  dataFile.print("Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C), OrientationX(deg), y(deg) , z(deg), ");
  dataFile.println("TempOut(C), GPSLat, GPSLong, GPSAge, GPSSats, RBSigalQuality");

  // RockBlock
  IridiumSerial.begin(19200);
  DEBUG_PRINTLN("Starting rockblock serial");
  err = modem.begin();
  if (err != ISBD_SUCCESS) {
    DEBUG_PRINT("Begin failed: error ");
    DEBUG_PRINTLN(err);
    if (err == ISBD_NO_MODEM_DETECTED) DEBUG_PRINTLN("No modem detected: check wiring.");
  }
}

void loop() {
  String dataString = readSensors();
  long loopTime = millis();

  // Receive RockBlock Command
  uint8_t buffer[1];
  err = modem.sendReceiveSBDText(NULL, buffer, bufferSize);

  //cut down from balloon
  if (buffer[0] == 't') {
    topCutStart = loopTime;
  } else if (buffer[0] == 'b') {
    bottomCutStart = loopTime;
  }
  if (loopTime - topCutStart < 10000) {
    digitalWrite(topCut, HIGH);
  } else {
    digitalWrite(topCut, LOW);
  }
  if (loopTime - bottomCutStart < 10000) {
    digitalWrite(bottomCut, HIGH);
  } else {
    digitalWrite(bottomCut, LOW);
  }

  // heat if too cold
  if (applyHeat) {
    digitalWrite(heater, HIGH);
  } else {
    digitalWrite(heater, LOW);
  }

  if (dataFile) {
    DEBUG_PRINTLN("Writing to datalog.txt");
    dataFile.println(dataString);
  } else {
    DEBUG_PRINTLN("Error opening datalog.txt");
  }

  if (loopTime - lastFlush > SD_CARD_FLUSH_TIME) {
    DEBUG_PRINTLN("Flushing datalog.txt");
    dataFile.flush();
    lastFlush = loopTime;
  }

  if (loopTime - lastTransmit > ROCKBLOCK_TRANSMIT_TIME) {
    DEBUG_PRINTLN("Transmiting to ROCKBlock");
    err = modem.sendSBDText(dataString);
    lastTransmit = loopTime;
  }
}

// Reads from all of the sensors and outputs the data string
String readSensors() {
  String dataString = "";

  // Timing
  long loopTime = millis();
  DEBUG_PRINTLN("Loop Time ");
  DEBUG_PRINTLN(loopTime);
  dataString += loopTime + ", ";

  // BMP280 (Barometer + Thermometer) Input
  DEBUG_PRINTLN("BMP280 stuff");
  double tempIn = bmp.readTemperature();
  applyHeat = (tempIn < 0);
  double pressure = bmp.readPressure();
  double alt = bmp.readAltitude(LAUNCH_SITE_PRESSURE); // avg sea level pressure for hollister for past month
  if (loopTime - lastAscentTime > BAROMETER_MEASURMENT_INTERVAL) { // calculate ascent rate in m/s every 10s
    ascentRate = (alt - lastAlt) * 1000 / (loopTime - lastAscentTime);
    lastAlt = alt;
    lastAscentTime = loopTime;
  }
  dataString += String(pressure) + ", " + String(alt) + ", ";
  dataString += String(ascentRate) + ", " + String(tempIn) + ", ";
  DEBUG_PRINTLN(pressure);
  DEBUG_PRINTLN(alt);
  DEBUG_PRINTLN(ascentRate);
  DEBUG_PRINTLN(tempIn);

  // BNO055 (IMU) Input
  DEBUG_PRINTLN("BNO055 Stuff");
  sensors_event_t event;
  bno.getEvent(&event);
  dataString += (String)event.orientation.x + ", " + (String)event.orientation.y + ", ";
  dataString += (String)event.orientation.z + ", ";
  DEBUG_PRINTLN(event.orientation.x);
  DEBUG_PRINTLN(event.orientation.y);
  DEBUG_PRINTLN(event.orientation.z);

  // MAX31855 (Thermocouple) Input
  DEBUG_PRINTLN("MAX31855 Stuff");
  double temperature = thermocouple.readCelsius(); // celsius
  dataString += String(temperature) + ", ";
  DEBUG_PRINTLN(temperature);

  // GPS Input
  DEBUG_PRINTLN("GPS Stuff");
  bool newData = false;
  
  while (Serial1.available()) {
    //DEBUG_PRINTLN("SS AVAILABLE");
    char c = Serial1.read();
    if(gps.encode(c)) {
      newData = true;
      break;
    }
  }
  if(newData) {
    f_lat = gps.location.lat();
    f_long = gps.location.lng();
    f_age = gps.location.age();
    sats = gps.satellites.value();
  }

  dataString += String(f_lat) + ", " + String(f_long) + ", ";
  dataSTring += String(f_age) + ", " + String(sats); + ", ";
  DEBUG_PRINTLN(f_lat);
  DEBUG_PRINTLN(f_long);
  DEBUG_PRINTLN(f_age);
  DEBUG_PRINTLN(sats);
  DEBUG_PRINTLN("");

  int signalQuality = -1;
  err = modem.getSignalQuality(signalQuality);
  dataString += string(signalQuality) + ", ";
  
  return dataString;
}

