
#include "SoftwareSerial.h" // to computer
#include "SPI.h" // for SPI //
#include "Wire.h" // for I2C // PModIA // A4 SDA, A5 SCL 
#include "well_addresses.h"
#include "PModIA.h"
#include "Well.h"
#include "Multiplexer.h"

// this uses a lot of memory, this could be cut down by not saving impedance measurements, the ADG731 sync bits could be done dynamically by ADG numer, actually, yeah I like that... this will reduce well_addresses.h file too!
Well wells[96];

// this could go to the Well class, but it would be a bit hard-coded
void initiate_wells(){
  for (int i = 0; i <= 95; i++){
    wells[i].well_number = well_numbers[i];
    wells[i].ADG731_number = ADG731_numbers[i];
    // this should be able to be cleaned up, but I'm not sure how, haven't done c++ in awhile
    if (wells[i].ADG731_number == 1){
      wells[i].ADG731_select_bits[0] = 1;
      wells[i].ADG731_select_bits[1] = 0;
      wells[i].ADG731_select_bits[2] = 0;
    };
    if (wells[i].ADG731_number == 1){
      wells[i].ADG731_select_bits[0] = 0;
      wells[i].ADG731_select_bits[1] = 1;
      wells[i].ADG731_select_bits[2] = 0;
    };
    if (wells[i].ADG731_number == 1){
      wells[i].ADG731_select_bits[0] = 0;
      wells[i].ADG731_select_bits[1] = 0;
      wells[i].ADG731_select_bits[2] = 1;
    };
    wells[i].ADG731_shift_register = well_address_input_shift_registers[i];
  };
};

void setup() {
  // put your setup code here, to run once:
  initiate_wells();
  Serial.begin(115200);
}

int i = 0;
void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(wells[i].well_number);
  Serial.println(wells[i].ADG731_number);
  Serial.println(wells[i].ADG731_select_bits[0]);
  Serial.println(wells[i].ADG731_shift_register);
  delay(1000);

  // so it should wait till it receives a read command from the serial terminal.
  // for now let's just initiate a block read of all 96, that can switch to individual
  // wells later if need be

  // Then it should
  // use the multiplex class so it reads the right electrode
  // do the PModIA stuff, recieve the real and imaginary parts

  // update the value of the real and imaginary parts of the well type
  // write the well type for computer to receive to a file over serial.
  // ^^^ this will be a file running python that will then write the data to a csv. text file
  // with each line being a well object

  i++;
}
