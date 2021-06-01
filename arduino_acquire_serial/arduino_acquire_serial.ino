//#include "Wire.h" // serial communication to ADG
#include "well_addresses.h" // adg switch codes... this could belong to the multiplexer class...
#include "PModIA.h" // interface for PModIA
#include "Multiplexer.h" // made up of three ADG's
#include "SoftwareSerial.h" // communicate with bluetooth

IA ia(5); // PModIA, fbr select pin as argument
Multiplexer multiplexer{2, 3, 4}; // Multiplexer, sync pins as argument

int freq = 10000;
SoftwareSerial BT(0, 1); // RX | TX
void setup() {
  BT.begin(9600);
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
  while(begin == 0){
    while (BT.readString()){
      begin = 1;
      break;
    }
  }
  begin = 0;
  String message = "[";
  sendBT(message);
  
  for (int j = 0; j <=7; j++){
    message="";
    int index;
    for (int i = 1; i <=12; i++){
      index = j * 12 + i;
      multiplexer.select_electrode(index);
      ia.BeginFrequencySweep();
      ia.ApplyTwoPointCal(freq);
      message += index;
      message += ",";
      message += freq;
      message += ",";
      message += ia.getImpedance();
      message += ",";
      message += ia.getPhase();
      message += ",";
    }
    sendBT(message); //sends data for 12 samples
  }
    
  message = "]";
  sendBT(message);  
}

//method to send data via bluetooth
void sendBT(String message){
  int len = message.length()+1;
  char sending[len];
  message.toCharArray(sending, len);
  BT.write(sending);
}
