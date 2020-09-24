class PModIA{
  // register addresses
  public:
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
    int real_data_register_7_0 = 0x93;
    int imaginary_data_register = 0x96; // 16 bits
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

    void readData_AD5933(int addr){
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

    void initialize_frequency_sweep(float start_frequency){
      // set the start frequency register
      long frequency_code;
//      frequency_code = this->get_freq_inc_code_byte(start_frequency); // not sure how to do this, big problem // need to get back to the basics of c++ unfortunately
      byte frequency_code_byte;
      frequency_code_byte = frequency_code >> 0; // writes from LSB first?
      writeData_AD5933(start_frequency_register_7_0, frequency_code_byte);
      frequency_code_byte = frequency_code >> 8; // does this shift persist?
      writeData_AD5933(start_frequency_register_15_8, frequency_code_byte);
      frequency_code_byte = frequency_code >> 8; // does this shift persist?
      writeData_AD5933(start_frequency_register_23_16, frequency_code_byte);

      // set the frequency increment register
      writeData_AD5933(frequency_increment_register_7_0, 0x00);
      writeData_AD5933(frequency_increment_register_15_8, 0x00);
      writeData_AD5933(frequency_increment_register_23_16, 0x00);

      // set number of frequency increments
      // set to 0 and ask about this
      writeData_AD5933(number_of_increments_register_7_0, 0x00);
      writeData_AD5933(number_of_increments_register_15_8, 0x00);

      // set setting cycles
      writeData_AD5933(number_of_increments_register_7_0, 0x0C);
      writeData_AD5933(number_of_increments_register_15_8, 0x00);

      // initialize start frequency (for setting time)
      // what output voltage range?... rev_1 says 200mV but programs 400mV
      // also sets gain to 1, what is PGA gain?
      writeData_AD5933(control_register_15_8, 0x15);
    }

    void start_frequency_sweep(float start_frequency){
      writeData_AD5933(control_register_15_8, 0x25);
    }

    void impedance_calculation(){
      // this will take whatever data is available and return a single(?)
      // impedance
    }
};
