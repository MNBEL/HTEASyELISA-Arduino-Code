/**********************************************
* @file:	IA.h
* @Author: 	Wanyika Njuguna(jnjuguna@ni.com)
* @Description: Chipkit IA demo
***********************************************/

#ifndef __PMODIA_H__
#define __PMODIA_H__

#include <inttypes.h>
#include <Wire.h>
#include <EEPROM.h>


/***********************************************/
/****************** Macros *********************/

#define IA_ADDRESS		0x0D  //IA
#define BLOCK_WRITE_CMD 0xA0  //block write command
#define BLOCK_READ_CMD  0xA1  //block read command
#define ADR_PTR_CMD     0xB0  //address pointer command

/*Register Map*/
//      Name				Register Address
#define CONTROL_REG 			0x80
#define START_FREQUENCY_REG  	0x82
#define FREQUENCY_INCREMENT_REG 0x85
#define NUM_INCREMENTS_REG		0x88
#define NUM_SETTLING_CYCLES_REG	0x8A
#define STATUS_REG				0x8F
#define TEMP_DATA_REG			0x92
#define REAL_DATA_REG			0x94
#define IMAG_DATA_REG			0x96

/*CONTROL REGISTER COMMANDS*/
#define INIT_START_FREQ			0x1
#define START_FREQ_SWEEP		0x2
#define INC_FREQ				0x3
#define REPEAT_FREQ				0x4
#define MEASURE_TEMP			0x9
#define PWR_DOWN				0xA
#define STAND_BY				0xB

/*Constants*/
/// THIS ASSSUMES 16 MHz, not 16.667
// #define  FREQUENCY_MULTIPLIER      33.5544
#define  FREQUENCY_MULTIPLIER 32.2116

class IA{
public:

// should this be convereted to a structure?
		/*Register Buffers*/
		uint8_t ControlData[2];
		uint8_t StartFreqData[3];
		uint8_t FrequencyIncData[3];
		uint8_t NumIncData[2];
		uint8_t SettlingTimeData[2];
		uint8_t realData[2];
		uint8_t imagData[2];

// keeps track of data and range of the PModIA
		typedef struct IA_STRUCT{
			int16_t rVAL;
			int16_t iVAL;
			int v_pp_code;
			int pga_gain;
			int fbr_code;
			int FBR_select_pin;
			double FBR_val;
		}IA_STRUCT;
		IA_STRUCT IA_DATA;

// keeps all the sweep parameters for the PModIA
		typedef struct SWEEP_PARAM{
			bool calibration_Flag;
			unsigned long start_Freq;
			int freq_Step;
			int num_sweep_Steps;
			bool repeat_Flag;
			unsigned int settling_cycles;
			unsigned int settling_factor;
		}SWEEP_PARAM;
		SWEEP_PARAM SParam;

// keeps track of the off-device values extracted from the PModIA
		typedef struct MEASUREMENT{
			double impedance;
			double phase;
			double gain_factor;
			double system_phase;
			double calibration_freq;
			double calibration_imp;
			double calibration_gain_factor;
			double calibration_system_phase;
			double calibration_gf_slope;
			double calibration_phase_slope;
		}MEASUREMENT;
		MEASUREMENT IMP_DATA;

	public:
// constructor
		IA(int fbr_select_pin);

// communication
		void setRegisterPointer(uint8_t RegAddress);
		void blockWrite(int numBytes, uint8_t *data);
		void blockRead (int numBytes, uint8_t *buffer, uint8_t RegAddress);
		uint8_t readRegisterValue(uint8_t RegAddress);
// 	range settings
		void setRangeParameters(int, int, int );
		void setV_pp(int );
		void setFBR(int );
		void setPGA_gain(int );
// sweep settings
		void setSweepParameters(unsigned long, unsigned long, unsigned long, unsigned int, unsigned int);
		void setStartFrequency(unsigned long frequency);
		void setFrequencyIncrement(unsigned int deltaFreq);
		void setNumSamples(unsigned int samples);
		void setSettlingTime(unsigned int settlingTime,	unsigned int settlingFactor);
// sweep control
		void setControlRegister(unsigned int function);
		void BeginFrequencySweep();
		void wait_till_DFT_complete();
		void RepeatFreq();
		void IncrementFreq();
		bool check_sweep_complete();
		bool check_v_pp_code_set();
// read and convert real and imaginary data registers to useful data
		void getIA_Data();
		void calcGainFactor();
		void calcImpedance();
		void calcPhase();
		double calcZReal();
		double calcZImaginary();

//////////////////////////////////////////////////////////
// this is what needs to be cleaned up!
// all the helper functions, post real/imag register reads
		void MeasureCalPoint(float, float, float);
		void MeasureTwoCalPoint(float, float, float, float);
		void SaveTPC(long, long, long, long, long);
		void ReadTPC(long, long, long, long, long);
		void ApplyTwoPointCal(float );

// fetching / encapsulation functions
		double getGainFactor();
		double getSystemPhase();
		double getPhase();
		double getImpedance();
		void setGainFactor(float ); // to set from memory
		void setSystemPhase(float ); // to set from memory
};

/****************************** FUNCTION DEFINITIONS *****************************/
// constructor
IA::IA(int fbr_select_pin){
	Wire.begin();
	IA_DATA.FBR_select_pin = fbr_select_pin;
	pinMode(IA_DATA.FBR_select_pin, OUTPUT);

	IA_DATA.rVAL = 0;
	IA_DATA.iVAL = 0;
	IA_DATA.v_pp_code = 1;
	IA_DATA.pga_gain = 1;
	IA_DATA.fbr_code = 0;
	IA_DATA.FBR_select_pin = 5;
	IA_DATA.FBR_val = 10000;

	SParam.calibration_Flag = false;
	SParam.start_Freq = 0;
	SParam.freq_Step = 0;
	SParam.num_sweep_Steps = 0;
	SParam.repeat_Flag = 0;
	SParam.settling_cycles = 1;
	SParam.settling_factor = 1;

	IMP_DATA.impedance = 0.0;
	IMP_DATA.phase = 0.0;
	IMP_DATA.gain_factor = 0;
	IMP_DATA.system_phase = 0.0;
	IMP_DATA.calibration_gain_factor = 0;
	IMP_DATA.calibration_system_phase = 0;
	IMP_DATA.calibration_freq = 1.0;
	IMP_DATA.calibration_imp = 100000;
	IMP_DATA.calibration_gf_slope = 1.0;
	IMP_DATA.calibration_phase_slope = 1.0;
}

// communication
void IA::setRegisterPointer(uint8_t RegAddress){
	Wire.beginTransmission(IA_ADDRESS);
	Wire.write(ADR_PTR_CMD);
	Wire.write(RegAddress);
	Wire.endTransmission();
}

void IA::blockWrite(int numBytes, uint8_t *data){
	Wire.beginTransmission(IA_ADDRESS);
	Wire.write(BLOCK_WRITE_CMD);
	Wire.write(numBytes);
	for(int i = 0; i < numBytes; i++){
		Wire.write(data[i]);
	}
	Wire.endTransmission();
}

void IA::blockRead (int numBytes, uint8_t *buffer, uint8_t RegAddress){
	for(int i = 0; i<numBytes; i++){
		setRegisterPointer(RegAddress);
		Wire.requestFrom(IA_ADDRESS, 1);
		while(Wire.available()){
			buffer[i] = Wire.read();
		}
		Wire.endTransmission();
		RegAddress++;
	}
}

uint8_t IA::readRegisterValue(uint8_t RegAddress){
	unsigned char data;
	setRegisterPointer(RegAddress);
	//Wire.beginTransmission(IA_ADDRESS);
	Wire.requestFrom(IA_ADDRESS, 1);
	while(Wire.available()){
		data = Wire.read();
	}
	Wire.endTransmission();
	return data;
}

// range settings
void IA::setRangeParameters(int pga, int rfb, int vpp){
	IA_DATA.pga_gain = pga; // 1 x1, 2 x5
	IA_DATA.fbr_code = rfb; // 1 20, 2 100K
	IA_DATA.v_pp_code = vpp; // 1 200, 2 400, 3 1000, 4 2000
	setPGA_gain(IA_DATA.pga_gain);
	setFBR(IA_DATA.fbr_code);
	setV_pp(IA_DATA.v_pp_code);
}

void IA::setV_pp(int option){
	IA_DATA.v_pp_code = option;
	switch(option)
	{
		case 1: 	// 200mV
			ControlData[0] |= 0x2;
			break;
		case 2:	// 400mV
			ControlData[0] |= 0x4;
			break;
		case 3:	// 1V
			ControlData[0] |= 0x6;
			break;
		case 4:	// 2V
			ControlData[0] |= 0x0;
			break;
		default:
			break;
	}
}

void IA::setFBR(int option){
	switch(option){
		case 1: // 20 ohm
			digitalWrite(IA_DATA.FBR_select_pin, 1);
			IA_DATA.FBR_val = 20;
			break;
		case 2: // 100K ohm
			digitalWrite(IA_DATA.FBR_select_pin, 0);
			IA_DATA.FBR_val = 100000;
			break;
		default:
			break;
		}
		IA_DATA.fbr_code = option;
}

void IA::setPGA_gain(int option){
	switch(option){
		case 1: // x1
			ControlData[0] |= 0x1;
			break;
		case 2: // x5
			ControlData[0] |= 0x0;
			break;
		default:
			break;
	}
	IA_DATA.pga_gain = option;
}

// sweep settings
void IA::setSweepParameters(unsigned long begin_Freq, unsigned long f_Step, unsigned long nSteps, unsigned int settling_cycles, unsigned int settling_factor){
	SParam.start_Freq = begin_Freq;
	SParam.freq_Step = f_Step;
	SParam.num_sweep_Steps = nSteps;
	SParam.settling_cycles = settling_cycles;
	SParam.settling_factor = settling_factor;
	setStartFrequency(SParam.start_Freq); //Hz
	setFrequencyIncrement(SParam.freq_Step); //Hz
	setNumSamples(SParam.num_sweep_Steps);
	setSettlingTime(SParam.settling_cycles, SParam.settling_factor);
}

void IA::setStartFrequency(unsigned long frequency){
	unsigned long result = 0;
	result = frequency  * FREQUENCY_MULTIPLIER;
	StartFreqData[2] = (uint8_t) (result & 0xFF);
	StartFreqData[1] = (uint8_t) ((result >> 8) & 0xFF);
	StartFreqData[0] = (uint8_t) ((result >> 16) & 0xFF);
	setRegisterPointer(START_FREQUENCY_REG);
  blockWrite(3, StartFreqData);
}

void IA::setFrequencyIncrement(unsigned int deltaFreq){
	unsigned long result = 0;
	result = deltaFreq * FREQUENCY_MULTIPLIER;
	FrequencyIncData[2] = (uint8_t) (result & 0xFF);
	FrequencyIncData[1] = (uint8_t) ((result>>8) & 0xFF);
	FrequencyIncData[0] = (uint8_t) ((result>>16) & 0xFF);
	setRegisterPointer(FREQUENCY_INCREMENT_REG);
	blockWrite(3, FrequencyIncData);
}

void IA::setNumSamples(unsigned int samples){
	if(samples == 0x1FF || samples < 0x1FF){
		NumIncData[1] = (uint8_t)(samples & 0xFF);
		NumIncData[0] = (uint8_t)((samples>>8) & 0xFF);
	}
	else{
		//Serial.println("Num of Samples is too large");
		//Serial.println("Usage: samples < 512");
		return;
	}
	setRegisterPointer(NUM_INCREMENTS_REG);
	blockWrite(2, NumIncData);
}

void IA::setSettlingTime(unsigned int settlingTime, unsigned int settlingFactor){
  uint8_t arr[2] = {0,0};

  if(settlingTime <= 0x01FF) {
   SettlingTimeData[1] = (uint8_t)(settlingTime & 0xFF);
   SettlingTimeData[0] = (uint8_t)((settlingTime >> 8)& 0xFF);
  }
  else{
  //  Serial.println("Num of Settling Time is too large");
  //  Serial.println("Usage: < 512 ");
  }
  switch(settlingFactor){
    case 1:
      SettlingTimeData[0] |= 0x00;
      break;
    case 2:
      SettlingTimeData[0] |= 0x2; //D10, D9 --> 01
      break;
    case 4:
      SettlingTimeData[0] |= 0x6; //D10, D9 --> 11
      break;
		default:
			break;
  }
  setRegisterPointer(NUM_SETTLING_CYCLES_REG);
  blockWrite(2, SettlingTimeData);
}

// sweep control
void IA::setControlRegister(unsigned int function){
	switch(function){
		case 1: //Initialize with start frequency
			ControlData[0] = 0x10;
			break;
		case 2: //start frequency sweep
			ControlData[0] = 0x20;
			break;
		case 3: //Increment frequency
			ControlData[0] = 0x30;
			break;
		case 4: //Repeat Frequency
			ControlData[0] = 0x40;
			break;
		case 9: //Measure Temperature
			ControlData[0] = 0x90;
			break;
		case 10: //Power down mode
			ControlData[0] = 0xA0;
			break;
		case 11: //Standby mode
			ControlData[0] = 0xB0;
			break;
		default:
			break;
	}
	setRangeParameters(IA_DATA.pga_gain, IA_DATA.fbr_code, IA_DATA.v_pp_code);
	setRegisterPointer(CONTROL_REG);
	Wire.beginTransmission(IA_ADDRESS);
	Wire.write(CONTROL_REG);
	Wire.write(ControlData[0]);
	Wire.endTransmission();
}

void IA::BeginFrequencySweep(){
	if(check_v_pp_code_set()){ //Range needs to be set before performing a freq sweep
		//1...............Place the AD5933 into standby mode
		setControlRegister(STAND_BY);
		//2...............Initialize the Start Frequency Command to the Control Register
		setControlRegister(INIT_START_FREQ);
		/// this is where a delay would be used to let a high impedance settle
		//3...............Program Start Frequency Sweep Command in the Control Register
		setControlRegister(START_FREQ_SWEEP);
		wait_till_DFT_complete();
	}
	else{
		Serial.println("v_pp_code not set");
	}
}

void IA::wait_till_DFT_complete(){
	uint8_t BIT;
	do{
		blockRead(1, &BIT, STATUS_REG);
		BIT = BIT & 0b00000010;
	}while(BIT != 0x02);
	getIA_Data();
}

void IA::RepeatFreq(){
	setControlRegister(REPEAT_FREQ);
	wait_till_DFT_complete();
}

void IA::IncrementFreq(){
	setControlRegister(INC_FREQ);
	wait_till_DFT_complete();
}

bool IA::check_sweep_complete(){
	uint8_t BIT;
	bool complete;
	Wire.requestFrom(IA_ADDRESS,1);
	BIT = Wire.read();
	BIT = BIT & 0b00000100;
	if (BIT == 0x04){
		complete = true;
	}
	else{
		complete = false;
	}
	return complete;
}

bool IA::check_v_pp_code_set(){
	if (IA_DATA.v_pp_code){
		return true;
	}
	else{
		return false;
	}
}

// read and convert real and imaginary data registers to useful data
void IA::getIA_Data(){
	//Real Data
	blockRead(2, realData, REAL_DATA_REG);
	IA_DATA.rVAL = realData[0];
	IA_DATA.rVAL = IA_DATA.rVAL << 8;
	IA_DATA.rVAL |= realData[1];
	//Imaginary Data
	blockRead(2, imagData, IMAG_DATA_REG);
	IA_DATA.iVAL = imagData[0];
	IA_DATA.iVAL = IA_DATA.iVAL << 8;
	IA_DATA.iVAL |= imagData[1];
}

// takes values set by IA_DATA, calculates, and sets the gain factor
void IA::calcGainFactor(){
	double gainFactor = 0.0;
	double magnitude = 0.0;
	magnitude = sqrt(((float)IA_DATA.rVAL * (float)IA_DATA.rVAL) + ((float)IA_DATA.iVAL * (float)IA_DATA.iVAL));
	IMP_DATA.gain_factor = 1.0 / (magnitude * IMP_DATA.calibration_imp);
}

// takes values set by IA_DATA, calculates, and sets the impedance attribute
void IA::calcImpedance(){
	double Z = 0.0;
	double magnitude = 0.0;
	magnitude = sqrt(((float)IA_DATA.rVAL * (float)IA_DATA.rVAL) + ((float)IA_DATA.iVAL * (float)IA_DATA.iVAL));
	IMP_DATA.impedance = 1 / (IMP_DATA.gain_factor * magnitude);
}

// takes values set by IA_DATA, calculates, and sets the phase attribute
void IA::calcPhase(){
	double ph;
	IMP_DATA.phase = atan2(IA_DATA.iVAL, IA_DATA.rVAL) - IMP_DATA.system_phase;
	Serial.println(atan2(IA_DATA.iVAL, IA_DATA.rVAL));
}

double IA::calcZReal(){
	double ZR, Zphi = 0;
	Zphi = getPhase() - IMP_DATA.system_phase;
	ZR = getImpedance() * cos(Zphi);
	return ZR;
}

double IA::calcZImaginary(){
	double ZI, Zphi = 0;
	Zphi = getPhase() - IMP_DATA.system_phase;
	ZI = getImpedance() * cos(Zphi);
	return ZI;
}

// helper functions... how to organize these...
void IA::MeasureCalPoint(float cal_impedance, float cal_phase, float cal_freq){
	IMP_DATA.calibration_imp = cal_impedance;
	IMP_DATA.calibration_freq = cal_freq;
	setSweepParameters(IMP_DATA.calibration_freq, 0, 1, 10, 1);
	BeginFrequencySweep();
	getGainFactor();
	getSystemPhase();
}

void IA::MeasureTwoCalPoint(float cal_impedance, float cal_phase, float freq1, float freq2){
	float gf1;
	float gf2;
	float phase1;
	float phase2;
	MeasureCalPoint(cal_impedance, cal_phase, freq1);
	gf1 = IMP_DATA.gain_factor;
	phase1 = IMP_DATA.phase;
	MeasureCalPoint(cal_impedance, cal_phase, freq2);
	gf2 = IMP_DATA.gain_factor;
	phase2 = IMP_DATA.phase;
	IMP_DATA.calibration_freq = freq1;
	IMP_DATA.gain_factor = gf1;
	IMP_DATA.system_phase = phase1;
	IMP_DATA.calibration_gain_factor = gf1;
	IMP_DATA.calibration_gf_slope = (gf2 - gf1) / (freq2 - freq1);
	IMP_DATA.calibration_phase_slope = (phase2 - phase1) / (freq2 - freq1);
}

void IA::SaveTPC(long adr1, long adr2, long adr3, long adr4, long adr5){
	EEPROM.put(adr1, IMP_DATA.calibration_freq);
	EEPROM.put(adr2, IMP_DATA.calibration_gain_factor);
	EEPROM.put(adr3, IMP_DATA.calibration_system_phase);
	EEPROM.put(adr4, IMP_DATA.calibration_gf_slope);
	EEPROM.put(adr5, IMP_DATA.calibration_phase_slope);
}

void IA::ReadTPC(long adr1, long adr2, long adr3, long adr4, long adr5){
	EEPROM.get(adr1, IMP_DATA.calibration_freq);
	EEPROM.put(adr2, IMP_DATA.calibration_gain_factor);
	EEPROM.get(adr3, IMP_DATA.calibration_system_phase);
	EEPROM.get(adr4, IMP_DATA.calibration_gf_slope);
	EEPROM.get(adr5, IMP_DATA.calibration_phase_slope);
}

void IA::ApplyTwoPointCal(float freq){
	float old = IMP_DATA.calibration_gain_factor;
	IMP_DATA.gain_factor = old + (freq - IMP_DATA.calibration_freq) * IMP_DATA.calibration_gf_slope;
	old = IMP_DATA.calibration_system_phase;
	IMP_DATA.system_phase = old + (freq - IMP_DATA.calibration_freq) * IMP_DATA.calibration_phase_slope;
}

// fetching and setting functions
double IA::getGainFactor(){
	calcGainFactor();
	return IMP_DATA.gain_factor;
}

double IA::getSystemPhase(){
	calcPhase();
	IMP_DATA.system_phase = IMP_DATA.phase;
	return IMP_DATA.system_phase;
}

double IA::getPhase(){
	calcPhase();
	return IMP_DATA.phase;
}

double IA::getImpedance(){
	calcImpedance();
	return IMP_DATA.impedance;
}

void IA::setGainFactor(float gain_factor){
	IMP_DATA.gain_factor = gain_factor;
}

void IA::setSystemPhase(float sys_phase){
	IMP_DATA.system_phase = sys_phase;
}

#endif
