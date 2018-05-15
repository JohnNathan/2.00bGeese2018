#include "SD.h"
#include "SPI.h"
#define SD_ChipSelectPin 4
#include "TMRpcm.h"

TMRpcm tmrpcm;
File root;

void setup() {
//  return;
  tmrpcm.speakerPin = 9;
//  Serial.begin(9600);
  if (!SD.begin(SD_ChipSelectPin)) {
    Serial.println("SD fail");
    return;
  }
  Serial.println("SD initialization done");

//  root = SD.open("/");
//  printDirectory(root, 0);
  
  tmrpcm.setVolume(7);
  tmrpcm.play("HL1_S.WAV");
  
  Serial.println("playing");
}

void loop(){
//  tone(9, 440, 1000);
//  delay(1500);
//  tmrpcm.play("HL1_S.WAV");

}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
