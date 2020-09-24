#include "SPI.h"

class Multiplexer{
  // this needs a constructor to specify what the sync pins are! in order
  // of convention
public:
  int sync_pin_1;
  int sync_pin_2;
  int sync_pin_3;
  // anything else?

public:

  void initiate_communication(){
    // this needs to establish communication with the multiplexers
    // this is comprised of SPI and defining the appropiate pins for
    // that to happen, maybe this will take the place of the constructor
    SPI.begin();
    SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE1));
    pinMode(sync_pin_1, OUTPUT);
    pinMode(sync_pin_2, OUTPUT);
    pinMode(sync_pin_3, OUTPUT);
    close_all();
  }

  void select_electrode(int well){
    close_all();
      
    if (well <= 32){
  //    Serial.println("well 1");
      digitalWrite(sync_pin_1, LOW);
      digitalWrite(sync_pin_2, HIGH);
      digitalWrite(sync_pin_3, HIGH);
    }
    else if (well <= 64){
  //    Serial.println("well 2");
      digitalWrite(sync_pin_1, HIGH);
      digitalWrite(sync_pin_2, LOW);
      digitalWrite(sync_pin_3, HIGH);
    }
    else{
  //    Serial.println("well 3");
      digitalWrite(sync_pin_1, HIGH);
      digitalWrite(sync_pin_2, HIGH);
      digitalWrite(sync_pin_3, LOW);
    }
    SPI.transfer(well_address_input_shift_registers[well - 1]);
  }

  void close_all(){
    digitalWrite(sync_pin_1, HIGH);
    digitalWrite(sync_pin_2, HIGH);
    digitalWrite(sync_pin_3, HIGH);
    digitalWrite(sync_pin_1, LOW);
    digitalWrite(sync_pin_2, LOW);
    digitalWrite(sync_pin_3, LOW);
    SPI.transfer(0b10000000);
    digitalWrite(sync_pin_1, HIGH);
    digitalWrite(sync_pin_2, HIGH);
    digitalWrite(sync_pin_3, HIGH);
  }
};
