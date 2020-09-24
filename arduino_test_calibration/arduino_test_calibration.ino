//#include "Wire.h" // serial communication to ADG
#include "well_addresses.h" // adg switch codes... this could belong to the multiplexer class...
#include "PModIA.h" // interface for PModIA
#include "Multiplexer.h" // made up of three ADG's

IA ia(5); // PModIA, fbr select pin as argument
Multiplexer multiplexer{2, 3, 4}; // Multiplexer, sync pins as argument

void setup() {
  Serial.begin(38400);
  multiplexer.initiate_communication();
  int pga = 1; // pga 1 x1, 2 x5
  int fbr = 2; // rfb 1 20, 2 100K
  int vpp = 4; // vpp 1 200, 2 400, 3 1000, 4 2000
  ia.setRangeParameters(pga, fbr, vpp);
  ia.ReadTPC(0, 10, 20, 30, 40);
}

void loop(){
    for (long f = 5000; f <= 99900; f += 100){
    ia.setSweepParameters(f, 0, 1, 10, 1); // freq, step, nsteps, settling cyc, settling factor
        ia.BeginFrequencySweep();
        // could really use a quadratic fit...
        ia.ApplyTwoPointCal(f);
        // send to python
        Serial.print(f);
        Serial.print(" ");
        Serial.print(ia.getImpedance());
        Serial.print(" ");
        Serial.println(ia.getPhase());
  }
}
