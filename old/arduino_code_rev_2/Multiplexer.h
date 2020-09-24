class Multiplexer{
  // this needs a constructor to specify what the sync pins are! in order
  // of convention
  int sync_pin_1;
  int sync_pin_2;
  int sync_pin_3;
  int MOSI_pin;
  // anything else?

public:
  void initiate_communication(){
    // this needs to establish communication with the multiplexers
    // this is comprised of SPI and defining the appropiate pins for
    // that to happen, maybe this will take the place of the constructor

  }

  void pick_electrode(Well well){
    // write pins based on the Well ADG731 number to isolate the specific
    // ADG731

    // write bit 1 to the 1 adg731 sync pin
    // write bit 2 to the 2 adg731 sync pin
    // write bit 3 to the 3 adg731 sync pin

    // write to the

    // write the appropriate shift register of the well

    // check if it's all good and return a no error bit
  }
};
