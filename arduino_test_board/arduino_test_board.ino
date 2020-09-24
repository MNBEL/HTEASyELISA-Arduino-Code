//#include "Wire.h" // serial communication to ADG
#include "well_addresses.h" // adg switch codes... this could belong to the multiplexer class...
#include "PModIA.h" // interface for PModIA
#include "Multiplexer.h" // made up of three ADG's

IA ia(5); // PModIA, fbr select pin as argument
Multiplexer multiplexer{2, 3, 4}; // Multiplexer, sync pins as argument

int freq = 10000;
void setup() {
  Serial.begin(38400);
  multiplexer.initiate_communication();
  int pga = 1; // pga 1 x1, 2 x5
  int fbr = 2; // rfb 1 20, 2 100K
  int vpp = 4; // vpp 1 200, 2 400, 3 1000, 4 2000
  int f_Step = 0;
  int nSteps = 1;
  int settling_cycles = 10;
  int settling_factor = 1;
  ia.setRangeParameters(pga, fbr, vpp);
  ia.setSweepParameters(freq, f_Step, nSteps, settling_cycles, settling_factor);
  ia.ReadTPC(0, 10, 20, 30, 40);
  Serial.print(ia.IMP_DATA.calibration_freq);
  Serial.print(" ");
  Serial.print(ia.IMP_DATA.calibration_gain_factor, 10);
  Serial.print(" ");
  Serial.print(ia.IMP_DATA.calibration_system_phase);
  Serial.print(" ");
  Serial.print(ia.IMP_DATA.calibration_gf_slope, 20);
  Serial.print(" ");
  Serial.println(ia.IMP_DATA.calibration_phase_slope, 20);
}

void loop(){
  for (int i = 1; i <= 96; i++){
    multiplexer.select_electrode(i);
    ia.BeginFrequencySweep();
    ia.ApplyTwoPointCal(freq);
    if (ia.getImpedance() <= 50000){
      Serial.print(i);
      Serial.print(" ");
      Serial.print(freq);
      Serial.print(" ");
      Serial.print(ia.getImpedance());
      Serial.print(" ");
      Serial.println(ia.getPhase());
    }
  }
}
