//#include "Wire.h" // serial communication to ADG
#include "well_addresses.h" // adg switch codes... this could belong to the multiplexer class...
#include "PModIA.h" // interface for PModIA
#include "Multiplexer.h" // made up of three ADG's

// PModIA
IA ia(5); // fbr select pin as argument

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
int fbr = 1; 
int vpp = 4;
int freq = 10000;
float calibration_imp = 10000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  multiplexer.initiate_communication();

  // PModIA calibration
  int freq = 10000;
  int f_Step = 0;
  int nSteps = 1;
  int settling_cycles = 10;
  int settling_factor = 1;
  ia.setRangeParameters(pga, fbr, vpp);
  ia.setSweepParameters(freq, f_Step, nSteps, settling_cycles, settling_factor);
  ia.MeasureGainFactor(calibration_imp);
//  Serial.println(gainFactor, 10);
//  EEPROM.put(0, gainFactor);
//  EEPROM.get(0, readGF);
//  Serial.println(readGF, 10);
//  ia.getGF();
}

void loop() {
    for (long f = 5000; f <= 99900; f += 5000){
    ia.setSweepParameters(freq, 0, 1, 10, 1);
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
