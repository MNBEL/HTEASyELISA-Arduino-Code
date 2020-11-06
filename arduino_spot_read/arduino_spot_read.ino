//#include "Wire.h" // serial communication to ADG
#include "well_addresses.h" // adg switch codes... this could belong to the multiplexer class...
#include "PModIA.h" // interface for PModIA
#include "Multiplexer.h" // made up of three ADG's

IA ia(5); // PModIA, fbr select pin as argument
Multiplexer multiplexer{2, 3, 4}; // Multiplexer, sync pins as argument

unsigned long freq = 10000;
int well = 2;
void setup() {
  Serial.begin(38400);
  multiplexer.initiate_communication();
  int pga = 1; // pga 1 x1, 2 x5
  int fbr = 2; // rfb 1 20, 2 100K
  int vpp = 4; // vpp 1 200, 2 400, 3 1000, 4 2000
  ia.setRangeParameters(pga, fbr, vpp);
  ia.ReadTPC(0, 10, 20, 30, 40);
  ia.setSweepParameters(freq, 0, 1, 10, 1);
  multiplexer.select_electrode(well);
}

String inString = "";    // string to hold input
void loop() {
  // Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      well = inString.toInt();
      Serial.println(well);
      multiplexer.select_electrode(well);
      inString = "";
    }
  }
}
