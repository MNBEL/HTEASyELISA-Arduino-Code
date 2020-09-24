int readData_AD5933(int addr){
  int data;
  Wire.beginTransmission(0x0D);
  Wire.write(0xB0);
  Wire.write(addr);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(0x0D, 1);
  if (Wire.available() >= 1){
    data = Wire.read();
  }
  else {
    data = -1;
  }
  delay(1);
  return data;
}

void writeData_AD5933(int addr, int data){
  Wire.beginTransmission(0x0D);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
  delay(1);
}

void check_status(){
  Serial.println("STATUS CHECK");
  Serial.println("control register");
  Serial.println(readData_AD5933(0x80), HEX);
  Serial.println(readData_AD5933(0x81), HEX);
  Serial.println("start frequency");
  Serial.println(readData_AD5933(0x82), HEX);
  Serial.println(readData_AD5933(0x83), HEX);
  Serial.println(readData_AD5933(0x84), HEX);
  Serial.println("frequency increment");
  Serial.println(readData_AD5933(0x85), HEX);
  Serial.println(readData_AD5933(0x86), HEX);
  Serial.println(readData_AD5933(0x87), HEX);
  Serial.println("number increments");
  Serial.println(readData_AD5933(0x88), HEX);
  Serial.println(readData_AD5933(0x8A), HEX);
  Serial.println("number settling cycles");
  Serial.println(readData_AD5933(0x8B), HEX);
  Serial.println(readData_AD5933(0x8F), HEX);
  Serial.println("status register");
  Serial.println(readData_AD5933(0x92), HEX);
  Serial.println("temperature register");
  Serial.println(readData_AD5933(0x93), HEX);
  Serial.println("real data");
  Serial.println(readData_AD5933(0x94), HEX);
  Serial.println(readData_AD5933(0x95), HEX);
  Serial.println("imaginary data");
  Serial.println(readData_AD5933(0x96), HEX);
  Serial.println(readData_AD5933(0x97), HEX);
}

void printIA_DATA(){
  Serial.println("IA_DATA");
  Serial.println(ia.IA_DATA.gf_addr);
  Serial.println(ia.IA_DATA.gf_value_20);
  Serial.println(ia.IA_DATA.gf_value_100K);
  Serial.println(ia.IA_DATA.rVAL);
  Serial.println(ia.IA_DATA.iVAL);
  Serial.println(ia.IA_DATA.range); // default 7
  Serial.println(ia.IA_DATA.pga_gain); // default 1
  Serial.println(ia.IA_DATA.SYSPhase);
  Serial.println(ia.IA_DATA.imp);
  Serial.println(ia.IA_DATA.ph_angle);
}

void printSParam(){
  Serial.println("SParam");
  Serial.println(ia.SParam.calibration_Flag);
  Serial.println(ia.SParam.start_Freq);
  Serial.println(ia.SParam.freq_Step);
  Serial.println(ia.SParam.sweep_Samples);
  Serial.println(ia.SParam.repeat_Flag);
}

int control_register_15_8 = 0x80; // 16 bits ;
int control_register_7_0 = 0x81;
int start_frequency_register_23_16 = 0x82; // 24 bits
int start_frequency_register_15_8 = 0x83;
int start_frequency_register_7_0 = 0x84;
int frequency_increment_register_23_16 = 0x85; // 24 bits
int frequency_increment_register_15_8 = 0x86;
int frequency_increment_register_7_0 = 0x87;
int number_of_increments_register_15_8 = 0x88; // 16 bits
int number_of_increments_register_7_0 = 0x89;
int number_of_settling_time_cycles_register_15_8 = 0x8A; // 16 bits
int number_of_settling_time_cycles_register_7_0 = 0x8B;
int status_register = 0x8F; // 8 bits
int temperature_data_register_15_8 = 0x92; // 16 bits
int temperature_data_register_7_0 = 0x93;
int real_data_register_15_8 = 0x94; // 16 bits
int real_data_register_7_0 = 0x95;
int imaginary_data_register_15_8 = 0x96; // 16 bits
int imaginary_data_register_7_0 = 0x97;
