#include "Wire.h"
#include "well_addresses.h"
//#include "PModIA.h"
#include "WireIA.h"
//#include "Well.h"
#include "Multiplexer.h" // made up of three ADG's

// PModIA
IA ia(5);

// sync pins are D2 D3 D4
Multiplexer multiplexer{2, 3, 4};


int begin_Freq = 10000;
int f_Step = 0;
int nSamples = 1;
bool cal_Flag = 1;
float calibration_imp = 100000;
bool r_Flag = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  multiplexer.initiate_communication();

// PModIA calibration
  ia.setSweepParameters(begin_Freq, f_Step, nSamples, r_Flag, cal_Flag, calibration_imp);
  ia.PerformFrequencySweep();
  cal_Flag = 0;
  ia.setSweepParameters(begin_Freq, f_Step, nSamples, r_Flag, cal_Flag, calibration_imp);
}

// variables to store single impedance data
double phase;
double impedance;
int voltage;

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 1; i <= 96; i++){
    multiplexer.select_electrode(i);
    ia.PerformFrequencySweep();
    Serial.print(i);
    Serial.print(" ");
    Serial.print(ia.getImpedance());
    Serial.print(" ");
    Serial.println(ia.getPhase());
  }
}
