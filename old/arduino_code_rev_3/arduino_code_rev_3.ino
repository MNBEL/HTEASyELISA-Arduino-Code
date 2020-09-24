#include "Wire.h" // for I2C to the PModIA // A4 SDA, A5 SCL
#include "PModIA.h"

PModIA pmodIA;

float MCLK = 16776000;
// this should be in class eventually, and split into swtichcase for registers
// actually, should return the entire code and then parse it on the receiving end!
long get_freq_inc_code(float freq){
  // 24 bits
  long code = long((freq / (MCLK / 4)) * pow(2, 27));
  return code;
}

uint8_t real_data_7_0, real_data_15_8, imaginary_data_7_0, imaginary_data_15_8;
long real_data;
long imaginary_data;
float magnitude;
//float gain_factor = 0.000000392156887054443359375;
float gain_factor;
float impedance;

long freq_code = get_freq_inc_code(10000);

void setup() {
  // put your setup code here, to run once:
  Wire.setClock(1600000);
  Wire.begin();
  Serial.begin(38400);
  delay(10);

  // reset and use internal clock
  pmodIA.writeData_AD5933(pmodIA.control_register_7_0, 0x10); // reset and use internal clock
}



void loop() {
  // put your main code here, to run repeatedly:
      pmodIA.writeData_AD5933(pmodIA.control_register_15_8, 0xB0); // standby, 2Vpp, x5 gain
//

      pmodIA.writeData_AD5933(pmodIA.start_frequency_register_7_0, freq_code);
      pmodIA.writeData_AD5933(pmodIA.start_frequency_register_15_8, freq_code >> 8);
      pmodIA.writeData_AD5933(pmodIA.start_frequency_register_23_16, freq_code >> 16);
///
    
      // set increment value
      pmodIA.writeData_AD5933(pmodIA.frequency_increment_register_7_0, 0);
      pmodIA.writeData_AD5933(pmodIA.frequency_increment_register_15_8, 0);
      pmodIA.writeData_AD5933(pmodIA.frequency_increment_register_23_16, 0);
///
    
      // set number of increments
      pmodIA.writeData_AD5933(pmodIA.number_of_increments_register_7_0, 0);
      pmodIA.writeData_AD5933(pmodIA.number_of_increments_register_15_8, 0);
///
    
      // set settling cycles, relative to the probe frequency... not sure why it's designed like that
      pmodIA.writeData_AD5933(pmodIA.number_of_settling_time_cycles_register_7_0, 0);
      pmodIA.writeData_AD5933(pmodIA.number_of_settling_time_cycles_register_15_8, 0);
///
    
      // intiate probe frequency and settle
      pmodIA.writeData_AD5933(pmodIA.control_register_15_8, 0x10); // initialize with start frequency, 2V, x5 gain
    
      // begin frequency sweep
      pmodIA.writeData_AD5933(pmodIA.control_register_15_8, 0x20); // start sweep/measure, 2V, x5 gain
      
      // read values
      real_data_7_0 = pmodIA.readData_AD5933(pmodIA.real_data_register_7_0);
      real_data_15_8 = pmodIA.readData_AD5933(pmodIA.real_data_register_15_8);
      imaginary_data_7_0 = pmodIA.readData_AD5933(pmodIA.imaginary_data_register_7_0);
      imaginary_data_15_8 = pmodIA.readData_AD5933(pmodIA.imaginary_data_register_15_8);

      real_data = (((long)real_data_15_8) << 8) + (int)real_data_7_0;
      imaginary_data = (((long)imaginary_data_15_8) << 8) + (int)imaginary_data_7_0;
      magnitude = pow(pow(real_data,2) + pow(imaginary_data,2), 0.5);
    //  gain_factor = (1.0 / 10000) /  magnitude; 
      impedance = 1 / (gain_factor * magnitude);

      Serial.print(real_data);
      Serial.print(" ");
      Serial.print(imaginary_data);
      Serial.print(" ");
      Serial.println(magnitude);
//      Serial.println(gain_factor, 40);
//      Serial.println(impedance);

      delay(100);
}
