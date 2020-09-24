#include "Wire.h"
#include "well_addresses.h"
//#include "PModIA.h"
#include "WireIA.h"
//#include "Well.h"
#include "Multiplexer.h" // made up of three ADG's

// PModIA
IA ia(5);
// should set gain, vpp, rfb, 

// sync pins are D2 D3 D4
Multiplexer multiplexer{2, 3, 4};

// variables to store single impedance data
double phase;
double impedance;
int voltage;
// pga 1 x1, 2 x5
// rfb 1 20, 2 100K
// vpp 1 200, 2 400, 3 1000, 4 2000
int pga = 1;
int rfb = 1; 
int vpp = 4;
int freq = 10000;
float calibration_imp = 10000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  multiplexer.initiate_communication();

  // pga, rfb, vpp
//  ia.setRangeParameters(1, 2, 4);

  // PModIA calibration
  int freq = 10000;
  int f_Step = 0;
  int nSteps = 1;
  ia.setRangeParameters(pga, rfb, vpp);
  ia.setSweepParameters(freq, f_Step, nSteps);
  ia.MeasureGainFactor(calibration_imp);
}

void loop() {
    for (long f = 5000; f <= 99900; f += 50000){
    ia.setSweepParameters(freq, 0, 1);
      for (int i = 1; i <= 96; i++){
        // timing on this?
        multiplexer.select_electrode(i);
        ia.BeginFrequencySweep();
//      if (ia.getImpedance() < 1000){
        Serial.print(i);
        Serial.print(" ");
        Serial.print(f);
        Serial.print(" ");
        Serial.print(ia.getImpedance());
        Serial.print(" ");
        Serial.println(ia.getPhase());
//      }
    }
  }
}
