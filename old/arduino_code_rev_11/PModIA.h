class PModIA{
  public:
    // register addresses
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
    float MCLK = 16.776 * pow(10, 6);
  
    // PModIA address
    int AD5933_address = 0x0D;
  
    // create variables to store commands, or is hex fine?

  public:
    void initiate_communication(){
      // this will initiate communication to the PModIA
      // or perhaps communication should be made outside of this object?
      // I think I'd like it better inside
    }

    void writeData_AD5933(int addr, int data){
      Wire.beginTransmission(AD5933_address);
      Wire.write(addr);
      Wire.write(data);
      Wire.endTransmission();
      delay(1);
    }

    int readData_AD5933(int addr){
      int data;
      Wire.beginTransmission(AD5933_address);
      Wire.write(0xB0);
      Wire.write(addr);
      Wire.endTransmission();
      delay(1);
      Wire.requestFrom(AD5933_address, 1);
      if (Wire.available() >= 1){
        data = Wire.read();
      }
      else {
        data = -1;
      }
      delay(1);
      return data;
    }

    long get_freq_inc_code_byte(float freq, int n){
      // this is a 24 bit word
      // what's n do with the bit shift?
      long val = long((freq / (MCLK / 4)) * pow(2, 27));
      // byte code;
      // code = (val >> (24 - 8 * n));
      // return code
    }

    void reset_and_set_clock(){
      // not sure where this will be used
      // reset and set clock
      writeData_AD5933(control_register_7_0, 0x10);
      // enter standby
      writeData_AD5933(control_register_15_8, 0xB0);
    }

//    void perform_frequency_sweep(float start_frequency, long freq_code){
//      pmodIA.writeData_AD5933(pmodIA.start_frequency_register_7_0, freq_code);
//      pmodIA.writeData_AD5933(pmodIA.start_frequency_register_15_8, freq_code >> 8);
//      pmodIA.writeData_AD5933(pmodIA.start_frequency_register_23_16, freq_code >> 16);
//    
//      // set increment value
//      pmodIA.writeData_AD5933(pmodIA.frequency_increment_register_7_0, 0);
//      pmodIA.writeData_AD5933(pmodIA.frequency_increment_register_15_8, 0);
//      pmodIA.writeData_AD5933(pmodIA.frequency_increment_register_23_16, 0);
//    
//      // set number of increments
//      pmodIA.writeData_AD5933(pmodIA.number_of_increments_register_7_0, 0);
//      pmodIA.writeData_AD5933(pmodIA.number_of_increments_register_15_8, 0);
//    
//      // set settling cycles, relative to the probe frequency... not sure why it's designed like that
//      pmodIA.writeData_AD5933(pmodIA.number_of_settling_time_cycles_register_7_0, 0);
//      pmodIA.writeData_AD5933(pmodIA.number_of_settling_time_cycles_register_15_8, 0);
//    
//      // intiate probe frequency and settle
//      pmodIA.writeData_AD5933(pmodIA.control_register_15_8, 0x10); // initialize with start frequency, 2V, x5 gain
//      delay(10);
//    
//      // begin frequency sweep
//      pmodIA.writeData_AD5933(pmodIA.control_register_15_8, 0x20); // start sweep/measure, 2V, x5 gain
//      delay(10);
//      
//      // read values
//      real_data_7_0 = pmodIA.readData_AD5933(pmodIA.real_data_register_7_0);
//      real_data_15_8 = pmodIA.readData_AD5933(pmodIA.real_data_register_15_8);
//      imaginary_data_7_0 = pmodIA.readData_AD5933(pmodIA.imaginary_data_register_7_0);
//      imaginary_data_15_8 = pmodIA.readData_AD5933(pmodIA.imaginary_data_register_15_8);
//
//      real_data = (long)real_data_15_8 << 8 + (int)real_data_7_0;
//      imaginary_data = (long)imaginary_data_15_8 << 8 + (int)imaginary_data_7_0;
//      magnitude = pow(pow(real_data,2) + pow(imaginary_data,2), 0.5);
//    //  gain_factor = (1.0 / 10000) /  magnitude; 
//      impedance = 1 / (gain_factor * magnitude);
//      Serial.println(real_data);
//      Serial.println(imaginary_data);
//      Serial.println(magnitude);
//      Serial.println(gain_factor, 40);
//      Serial.println(impedance);
//    }

    void start_frequency_sweep(float start_frequency){
      writeData_AD5933(control_register_15_8, 0x25);
    }

    void impedance_calculation(){
      // this will take whatever data is available and return a single(?)
      // impedance
    }
};
