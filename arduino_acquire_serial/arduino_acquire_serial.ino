//#include "Wire.h" // serial communication to ADG
#include "well_addresses_4_well.h" // adg switch codes... this could belong to the multiplexer class...
#include "PModIA.h" // interface for PModIA
#include "Multiplexer.h" // made up of three ADG's
//#include "SoftwareSerial.h" // communicate with bluetooth

IA ia(5); // PModIA, fbr select pin as argument
Multiplexer multiplexer{2, 3, 4}; // Multiplexer, sync pins as argument

int freq = 10000;
void setup() {
  Serial.begin(38400);
  multiplexer.initiate_communication();
  int pga = 1; // pga 1 x1, 2 x5
  int fbr = 2; // rfb 1 20, 2 100K
  int vpp = 4; // vpp 1 200, 2 400, 3 1000, 4 2000
  ia.setRangeParameters(pga, fbr, vpp);
  ia.ReadTPC(0, 10, 20, 30, 40);
  ia.setSweepParameters(freq, 0, 1, 10, 1);
}

int begin = 0;
void loop(){
  delay(200);
//  while(begin == 0){
//    while (Serial.available()){
//      Serial.readString();
//      begin = 1;
//      break;
//    }
//  }
//  begin = 0;
  for (int i = 1; i <= 4; i++){
    multiplexer.select_electrode(i);
    ia.BeginFrequencySweep();
    ia.ApplyTwoPointCal(freq);
    Serial.print(i);
    Serial.print(" ");
    Serial.print(freq);
    Serial.print(" ");
    Serial.print(ia.getImpedance());
    Serial.print(" ");
    Serial.print(ia.getPhase());
    Serial.println();
  }
}
