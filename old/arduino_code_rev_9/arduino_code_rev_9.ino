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

int begin_Freq = 50000;
int f_Step = 0;
int nSteps = 1;
float calibration_imp = 100000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  multiplexer.initiate_communication();

  ia.setFBR(1);
  ia.setV_pp(3);

  // PModIA calibration
  int freq = 10000;
  int f_Step = 0;
  int nSteps = 1;
  ia.setSweepParameters(freq, f_Step, nSteps);
  float calibration_imp = 100000;
  ia.setRangeParameters(1, 1, 3);
  ia.MeasureGainFactor(calibration_imp);
}

// variables to store single impedance data
double phase;
double impedance;
int voltage;

void loop() {
//  for (int i = 0; i <= 989; i++){
//    ia.setSweepParameters(1000 + 100*i, 0, 1);
  for (long freq = 5000; freq <= 99900; freq += 100){
    ia.setSweepParameters(freq, 0, 1);
//    ia.BeginFrequencySweep();
//    ia.getImpedance();
    ia.getPhase();
    Serial.print(freq);
    Serial.print(" ");
    Serial.print(ia.getImpedance());
    Serial.print(" ");
    Serial.println(ia.getPhase());
  }
}
