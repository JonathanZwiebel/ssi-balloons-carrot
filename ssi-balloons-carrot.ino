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
    ;
  }
  #endif

  // initialize the sd card
  DEBUG_PRINTLN("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    DEBUG_PRINTLN("Card failed, or not present");
    return; // don't do anything more and return
  }
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  DEBUG_PRINTLN("Card initialized.");
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
  }
}

