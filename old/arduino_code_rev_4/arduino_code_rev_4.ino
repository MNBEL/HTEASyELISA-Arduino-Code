#include "Wire.h"
#include "PModIA.h"
#include "WireIA.h"

IA ia;

bool cal_Flag = 1;
int begin_Freq = 10000;
int f_Step = 0;
int nSamples = 1;
bool r_Flag = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

//  check_status();
  ia.setSweepParameters(cal_Flag, begin_Freq, f_Step, nSamples, r_Flag);
  ia.PerformFrequencySweep();
  cal_Flag = 0;
  ia.setSweepParameters(cal_Flag, begin_Freq, f_Step, nSamples, r_Flag);
//  check_status();
}

double phase;
double impedance;

void loop() {
  // put your main code here, to run repeatedly:
  ia.PerformFrequencySweep();

  Serial.print(ia.getImpedance());
  Serial.print(" ");
  Serial.println(ia.getPhase());
}
