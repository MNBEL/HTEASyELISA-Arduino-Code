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

/*AD5933 CONTROL FUNCTIONS */
#define AD5933_INIT_START_FREQ

/*Constants*/
/// THIS ASSSUMES 16 MHz, not 16.667
// #define  FREQUENCY_MULTIPLIER      33.5544
#define  FREQUENCY_MULTIPLIER 32.2116


class IA{
public:

		/*Register Buffers*/
		uint8_t ControlData[2];
		uint8_t StartFreqData[3];
		uint8_t FrequencyIncData[3];
		uint8_t NumIncData[2];
		uint8_t SettlingTimeData[2];
		uint8_t realData[2];
		uint8_t imagData[2];

// keeps track of data and range of the PModIA
		typedef struct IA_STRUCT
		{
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
		typedef struct SWEEP_PARAM
		{
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
		typedef struct MEASUREMENT
		{
			double gf_value;
			double SYSPhase;
			double imp;
			double ph_angle;
			double calibration_imp;
			double calibration_freq;
			double calibration_gf;
			double calibration_gf_slope;
			double calibration_phase;
			double calibration_phase_slope;
		}MEASUREMENT;
		MEASUREMENT IMP_DATA;

	public:
		IA(int fbr_select_pin); // constructor

		/*IA Communication*/
		void setRegisterPointer(uint8_t RegAddress);
		void blockWrite(int numBytes, uint8_t *data);
		void blockRead (int numBytes, uint8_t *buffer, uint8_t RegAddress);
		uint8_t readRegisterValue(uint8_t RegAddress);

		/*IA Initialize*/
		void IA_init(void);

		/*Set Control Register*/
		void setControlRegister(unsigned int function);

//////////////////////////////////////////////
// sweep settings
		void setSweepParameters(unsigned long, unsigned long, unsigned long, unsigned int, unsigned int);
		void setStartFrequency(unsigned long frequency);
		void setFrequencyIncrement(unsigned int deltaFreq);
		void setNumSamples(unsigned int samples);
		void setSettlingTime(unsigned int settlingTime,	unsigned int settlingFactor);
/////////////////////////////////////////////////

/////////////////////////////////////////////
// 	range settings
		void setRangeParameters(int, int, int );
		void setV_pp(int );
		void setFBR(int );
		void setPGA_gain(int );
/////////////////////////////////////////////


///////////////////////////////////////////////
// frequency sweep control
		/*Performing a Frequency Sweep*/
		void BeginFrequencySweep(void);
		void RepeatFreq();
		void IncrementFreq();
		void wait_till_DFT_complete();
		bool check_sweep_complete();
		bool check_v_pp_code_set();
////////////////////////////////////////////////

/*
I want to first be able to get a gain factor for a frequency and impedance
Then I want to do this for two frequencies...
Now... these change for every range... but forget that for now, I'll need to
learn some basic dictionary / memory storage for that
Then I want to save that data at two spots
*/

// all the helper functions, post real/imag register reads
		// imp
		void MeasureGainFactor(float);
		float SetSystemFactors(float, float );
		// imp, freq, freq
		void MeasureTwoPointCal(float, float, float);
		// to use two point, need a frequency it starts at, and the slope
		void SaveTPC(unsigned long, unsigned long, unsigned long, unsigned long);
		void ReadTPC(unsigned long, unsigned long, unsigned long, unsigned long);
		void ApplyTwoPointCal(float );

		float CalculateGainFactor();
		void Impedance();
		void Phase();
		double ZReal();
		double ZImaginary();
		void getIA_Data();

		// fetching / encapsulation functions
		double getGF();
		double getPhase();
		double getImpedance();
};

/************************************************************************/
/*																		*/
/*	IA_DATA.cpp  --	Library for Impedance Analyzer   	            		*/
/*																		*/
/************************************************************************/
/*	Author: 	Jessie W. Njuguna								    	*/
/*	Copyright 2014, Digilent Inc.										*/
/************************************************************************/
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This module contains the implementation of the object class that	*/
/*	forms a chipKIT interface to the Impedance Analyzer functions of    */
/*  the Digilent PmodIA_DATA.						  					*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	06/22/2014(JessieN): Created										*/
/*	11/06/2015(JColvin): Fixed getGF() variable assignment				*/
/*  																	*/
/*	Notes (11/6/2015):													*/
/*		Need to update library to do the following:						*/
/*		-User selection of PGA setting									*/
/*		-User setting of calibration impedance used						*/
/*		-User selection of feedback resistor							*/
/*		-User selection of p2p voltage range							*/
/*		-Library auto-selection of PGA, and p2p voltage during 			*/
/*			calibration sequence										*/
/*  																	*/
/************************************************************************/

/****************************** FUNCTION DEFINITIONS *****************************/
IA::IA(int fbr_select_pin)
{
	Wire.begin();
	IA_DATA.FBR_select_pin = fbr_select_pin;
	pinMode(IA_DATA.FBR_select_pin, OUTPUT);

	SParam.calibration_Flag = false;
	SParam.start_Freq = 0;
	SParam.freq_Step = 0;
	SParam.num_sweep_Steps = 0;
	SParam.repeat_Flag = 0;
	SParam.settling_cycles = 1;
	SParam.settling_factor = 1;

	IA_DATA.rVAL = 0;
	IA_DATA.iVAL = 0;
	IA_DATA.v_pp_code = 1;
	IA_DATA.pga_gain = 1;
	IA_DATA.fbr_code = 0;

	IMP_DATA.gf_value = 0;
	IMP_DATA.SYSPhase = 0.0;
	IMP_DATA.imp = 0.0;
	IMP_DATA.ph_angle = 0.0;
	IMP_DATA.calibration_imp = 100000;
	IMP_DATA.calibration_freq = 1.0;
	IMP_DATA.calibration_gf = 1.0;
	IMP_DATA.calibration_gf_slope = 1.0;
	IMP_DATA.calibration_phase = 1.0;
	IMP_DATA.calibration_phase_slope = 1.0;
}

 /***************************************************
* @function: 	IA_init
* @description: Initialize the IA I2C Communication
* @param:
* @return:
****************************************************/


/***********************************************************
* @function: 	setRegisterPointer
* @description: Set Register Pointer to a register address
* @param RegAddress: Register address to be set
*
* @return: 		None
************************************************************/

/*****************************************************************************************************/

void IA::setRegisterPointer(uint8_t RegAddress)
{
	Wire.beginTransmission(IA_ADDRESS);
	Wire.write(ADR_PTR_CMD);
	Wire.write(RegAddress);
	Wire.endTransmission();
}
/*****************************************************************************
* @function: 		blockWrite
* @description: 	Write a stream of bytes to slave using block write command
* @param: numBytes: number of bytes to be sent to slave
* @param: *data:    data to be sent to slave
*
* @return: 			None
******************************************************************************/
void IA::blockWrite(int numBytes, uint8_t *data)
{
	Wire.beginTransmission(IA_ADDRESS);
	Wire.write(BLOCK_WRITE_CMD);
	Wire.write(numBytes);
	for(int i = 0; i < numBytes; i++)
	{
		//Serial.println(data[i]); // display data being transmitted to slave device
		Wire.write(data[i]);
	}
	Wire.endTransmission();
}

/********************************************************
* @function: 		blockRead
* @description: 	Read a block of data from slave device
* @param numBytes: 	number of bytes to be read from slave
* @param *buffer:  	buffer to store read data
* @param RegAddress:Register to start reading from
* @return 			None
*********************************************************/
void IA::blockRead (int numBytes, uint8_t *buffer, uint8_t RegAddress)
{

	for(int i = 0; i<numBytes; i++)
	{
		setRegisterPointer(RegAddress);
		Wire.requestFrom(IA_ADDRESS, 1);
		while(Wire.available())
		{
			buffer[i] = Wire.read();
		}
		Wire.endTransmission();
		RegAddress++;
	}
}

/*****************************************************
* @function:	readRegisterValue
* @description: reads the value stored in a register
* @param:		Register address to read data
* @return: 		byte of data read
******************************************************/
uint8_t IA::readRegisterValue(uint8_t RegAddress)
{
	unsigned char data;

	setRegisterPointer(RegAddress);
	//Wire.beginTransmission(IA_ADDRESS);
	Wire.requestFrom(IA_ADDRESS, 1);
	while(Wire.available())
	{
		data = Wire.read();
	}
	Wire.endTransmission();

	return data;
}
/***************************************************************************************************
* @function: 	 setControlRegister
* @description:	 this function, sets the AD5933 control modes
* @param function: Function mode of the AD5933
* @param gain:	 amplifies the response signal into the ADC by a
*				 multiplication factor of x5 or x1
* @retrun:		 none
*
* NOTE--> Note that the control register should not be written to as part of a block write command.
****************************************************************************************************/
void IA::setControlRegister(unsigned int function)
{
	switch(function)
	{
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
/*********************************************
* @function: 	setStartFrequency
* @description: set start frequency for sweep
* @param frequency: start frequency
* @return 		None
**********************************************/
void IA::setStartFrequency(unsigned long frequency)
{
	unsigned long result = 0;

	result = frequency  * FREQUENCY_MULTIPLIER;

	StartFreqData[2] = (uint8_t) (result & 0xFF);
	StartFreqData[1] = (uint8_t) ((result >> 8) & 0xFF);
	StartFreqData[0] = (uint8_t) ((result >> 16) & 0xFF);

	setRegisterPointer(START_FREQUENCY_REG);
    blockWrite(3, StartFreqData);
}
/************************************************************************
* @function: 	setFrequencyIncrement
* @description:	sets the frequency delta increment between two consecutive
*				frequency points along the sweep
* @param deltaFreq: Delta Frequency to be added to current excitation
*					frequency.
* @return:		None
************************************************************************/
void IA::setFrequencyIncrement(unsigned int deltaFreq)
{
	unsigned long result = 0;

	result = deltaFreq * FREQUENCY_MULTIPLIER;
	FrequencyIncData[2] = (uint8_t) (result & 0xFF);
	FrequencyIncData[1] = (uint8_t) ((result>>8) & 0xFF);
	FrequencyIncData[0] = (uint8_t) ((result>>16) & 0xFF);

	setRegisterPointer(FREQUENCY_INCREMENT_REG);
	blockWrite(3, FrequencyIncData);
}
/**************************************************************************
* @function:	setNumSamples
* @description:	sets the number of frequency points in the frequency sweep
* @param samples:total number of samples
* @return:		None
****************************************************************************/
void IA::setNumSamples(unsigned int samples)
{
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
/**********************************************************************************
* @function:	setSettlingTime
* @description:	determines the number of output excitation cycles that are
*				allowed to pass through the unknown impedance
* @param settlingTime: 		number of settling time cycle
* @param settlingFactor: 	increases the settlingTime by either 2 or 4 or leaves
*							it as default
* @return: 					None
************************************************************************************/
void IA::setSettlingTime(unsigned int settlingTime, unsigned int settlingFactor)
{
  uint8_t arr[2] = {0,0};

  if(settlingTime <= 0x01FF) {
   SettlingTimeData[1] = (uint8_t)(settlingTime & 0xFF);
   SettlingTimeData[0] = (uint8_t)((settlingTime >> 8)& 0xFF);
  }
  else{
  //  Serial.println("Num of Settling Time is too large");
  //  Serial.println("Usage: < 512 ");
  }
  switch(settlingFactor)  {
    case 1:
      SettlingTimeData[0] |= 0x00;
      break;
    case 2:
      SettlingTimeData[0] |= 0x2; //D10, D9 --> 01
      break;
    case 4:
      SettlingTimeData[0] |= 0x6; //D10, D9 --> 11
      break;
  }

  setRegisterPointer(NUM_SETTLING_CYCLES_REG);
  blockWrite(2, SettlingTimeData);
}

/****************************************************************************************
* @function:		setRange
* @description: 	Function gives user an option to select range to use i.e, pick an
*					output excitation voltage and feed back resistor
* @param option: 	provides information of output voltage and feed back resistor to use
* @return:
*****************************************************************************************/

/// check these actual ranges
void IA::setV_pp(int option)
{
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
	IA_DATA.v_pp_code = option;

}

void IA::setFBR(int option)
{
	switch(option)
	{
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

void IA::setRangeParameters(int pga, int rfb, int vpp){
	IA_DATA.pga_gain = pga; // 1 x1, 2 x5
	IA_DATA.fbr_code = rfb; // 1 20, 2 100K
	IA_DATA.v_pp_code = vpp; // 1 200, 2 400, 3 1000, 4 2000
	setPGA_gain(IA_DATA.pga_gain);
	setFBR(IA_DATA.fbr_code);
	setV_pp(IA_DATA.v_pp_code);
}

/***********************************************************************************************
* @function:		setSweepParameters
* @description: 	This function initializes the Sweep Parameter struct with sweep parameters
* @param cal_Flag: 		flag for calibration sweep to set system phase and gain factor
* @param begin_Flag: 	begin frequency sweep flag
* @param f_step:		frequency margin step
* @param nSamples:      number of samples points
* @param r_Flag: 		repeat flag for a sweep point repeat
* @return: 		    None
*************************************************************************************************/
/// should be able to set the number of repeats
void IA::setSweepParameters(unsigned long begin_Freq, unsigned long f_Step, unsigned long nSteps, unsigned int settling_cycles, unsigned int settling_factor)
{
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
/*********************************************************************
* @function: 	PerformFrequencySweep
* @description: programs frequency sweep parameters into relevant
*				registers i.e.
*				1.Start Frequency Register
*				2.Number of Increments Register
*				3.Frequency Increment Register
*				and then performs a frequency sweep through commands
*				to the control register.
* @param :		None
* @retrun: 	    None
**********************************************************************/
/// this doesn't actually allow reading of values from frequency sweep!
void IA::BeginFrequencySweep()
{
	if(check_v_pp_code_set()){ //Range needs to be set before performing a freq sweep
		//1...............Place the AD5933 into standby mode
		setControlRegister(STAND_BY);
		// delay(50);
		//2...............Initialize the Start Frequency Command to the Control Register
		setControlRegister(INIT_START_FREQ);
		/// this is where a delay would be used to let a high impedance settle
		//3...............Program Start Frequency Sweep Command in the Control Register
		setControlRegister(START_FREQ_SWEEP);
		wait_till_DFT_complete();
		// delay(10);
	}
	else{
		// Range needs to be set
	}
}

bool IA::check_v_pp_code_set(){
	if (IA_DATA.v_pp_code){
		return true;
	}
	else{
		return false;
	}
}

// void IA::blockRead (int numBytes, uint8_t *buffer, uint8_t RegAddress)

void IA::wait_till_DFT_complete(){
	uint8_t BIT;
	do{
		blockRead(1, &BIT, STATUS_REG);
		BIT = BIT & 0b00000010;
	}while(BIT != 0x02);
	getIA_Data();
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

void IA::MeasureGainFactor(float calibration_imp){
	float gainFactor;
	if(check_v_pp_code_set()){
		IMP_DATA.calibration_imp = calibration_imp;
		BeginFrequencySweep();
		IMP_DATA.gf_value = CalculateGainFactor();
		IMP_DATA.SYSPhase = 0.0; /// need to 0!, is this the right place to do it?
		Phase();
		IMP_DATA.SYSPhase = IMP_DATA.ph_angle;
	}
	else{
		// v_pp not set
	}
}

void IA::RepeatFreq(){
	setControlRegister(REPEAT_FREQ);
	wait_till_DFT_complete();
	// delay(10);
}

void IA::IncrementFreq(){
	setControlRegister(INC_FREQ);
	wait_till_DFT_complete();
	// delay(10);
}

/*****************************************************************
* @function: 	 CalculateGainFactor
* @description:	 Calculate gain factor of the AD5933
* @param:        Known impedance to calibrate the system
* @return:		 none
******************************************************************/

float IA::CalculateGainFactor()
{
	double gainFactor;
	double magnitude = 0.0;
	magnitude = sqrt(((float)IA_DATA.rVAL * (float)IA_DATA.rVAL) + ((float)IA_DATA.iVAL * (float)IA_DATA.iVAL));
	gainFactor = 1 / (magnitude * IMP_DATA.calibration_imp);

	return gainFactor;
}
/*****************************************************************
* @function: 	 Impedance
* @description:	 Calculates impedance using a gain factor
* @param:
* @return:
******************************************************************/
void IA::Impedance()
{
	double Z = 0.0;
	double magnitude = 0.0;
	magnitude = sqrt(((float)IA_DATA.rVAL * (float)IA_DATA.rVAL) + ((float)IA_DATA.iVAL * (float)IA_DATA.iVAL));
	IMP_DATA.imp = 1 / (IMP_DATA.gf_value * magnitude);
}

/*****************************************************************
* @function: 	 Phase
* @description:	 Calculates phase across an impedance
* @param:
* @return:
******************************************************************/
void IA::Phase()
{
	double ph;
	IMP_DATA.ph_angle = atan2(IA_DATA.iVAL, IA_DATA.rVAL) - IMP_DATA.SYSPhase;
}

/*****************************************************************
* @function: 	 ZReal
* @description:	 Calculates real component of the Impedance
*				 |Z_real| = |Z| * cos(Zphi)
* @param:
* @return:		 |Z_real|
******************************************************************/
double IA::ZReal()
{
	double ZR, Zphi = 0;
	Zphi = getPhase() - IMP_DATA.SYSPhase;
	ZR = getImpedance() * cos(Zphi);
	return ZR;
}

/*****************************************************************
* @function: 	 ZImaginary
* @description:	 Calculates imaginary component of the Impedance
*				 |Z_imaginary| = |Z| * sin(Zphi)
* @param:
* @return:		 |Z_imaginary|
******************************************************************/
double IA::ZImaginary()
{
	double ZI, Zphi = 0;
	Zphi = getPhase() - IMP_DATA.SYSPhase;
	ZI = getImpedance() * cos(Zphi);
	return ZI;
}

/************************************************************
* @function: 		getIAData
* @description: 	reads value from AD5933 registers and
*					then populates the IA Data struct. This
*					function requires a frequency sweep to be
*					conducted
* @param:
* @retrun:
*************************************************************/
void IA::getIA_Data()
{
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


/****************************************************************
* @function:	getPhase
* @description: determines the phase angle of the impedance load
* @param:
* @return: 	    phase angle of last measured impedance
*****************************************************************/
double IA::getPhase()
{
	Phase();
	return IMP_DATA.ph_angle;
}
/*****************************************************************************
* @function:	getImpedance
* @description: gets the last measured impedance
* @param:
* @return: 		impedance value
********************************************************************************/
double IA::getImpedance()
{
	Impedance();
	return IMP_DATA.imp;
}

void IA::MeasureTwoPointCal(float impedance, float freq1, float freq2){
	float gf1;
	float gf2;
	float gf_slope;
	float phase1;
	float phase2;
	float phase_slope;
	setStartFrequency(freq1);
	BeginFrequencySweep();
	gf1 = IMP_DATA.gf_value;
	phase1 = IMP_DATA.ph_angle;
	setStartFrequency(freq2);
	BeginFrequencySweep();
	gf2 = IMP_DATA.gf_value;
	phase2 = IMP_DATA.ph_angle;
	IMP_DATA.calibration_gf = gf1;
	IMP_DATA.calibration_gf_slope = (gf2 - gf1) / (freq2 - freq1);
	IMP_DATA.calibration_phase = phase1;
	IMP_DATA.calibration_phase_slope = (phase2 - phase1) / (freq2 - freq1);
}

void IA::ApplyTwoPointCal(float freq){
	float old = IMP_DATA.gf_value;
	IMP_DATA.gf_value = old + (freq - IMP_DATA.calibration_freq) * IMP_DATA.calibration_gf_slope;

	old = IMP_DATA.ph_angle;
	IMP_DATA.ph_angle = old + (freq - IMP_DATA.calibration_freq) * IMP_DATA.calibration_phase_slope;
}

void IA::SaveTPC(unsigned long adr1, unsigned long adr2, unsigned long adr3, unsigned long adr4){
	EEPROM.put(adr1, IMP_DATA.calibration_gf);
	EEPROM.put(adr2, IMP_DATA.calibration_gf_slope);
	EEPROM.put(adr3, IMP_DATA.calibration_phase);
	EEPROM.put(adr4, IMP_DATA.calibration_phase_slope);
}

void IA::ReadTPC(unsigned long adr1, unsigned long adr2, unsigned long adr3, unsigned long adr4){
	EEPROM.get(adr1, IMP_DATA.calibration_gf);
	EEPROM.get(adr2, IMP_DATA.calibration_gf_slope);
	EEPROM.get(adr3, IMP_DATA.calibration_phase);
	EEPROM.get(adr4, IMP_DATA.calibration_phase_slope);
}

#endif
