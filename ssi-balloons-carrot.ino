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

#include <SPI.h>
#include <SD.h>

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

void setup() {
  startTime = millis(); // record the start time
  lastFlush = 0;

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
  dataFile.print("Time(ms), Pressure(mb), Alt(ft), TempIn(C), TempOut(C), GPSLat, GPSLong, ");
  dataFile.println("GPSAlt, ax(G), ay, az, gx, gy, gz, mx, my, mz, RockBlockStatus");
}

void loop() {
  String dataString = ""; // accumulate data using dataString += ...

  // write out data to the file if available
  if (dataFile) {
    dataFile.println(dataString);
  } else {
    DEBUG_PRINTLN("Error opening datalog.txt");
  }
  if (millis() - lastFlush > 10000){
    dataFile.flush(); // flush the SD car buffer every 10s
    lastFlush = millis();
  }
}

