
// these convert to a byte with the first 3 all being zero in our case, and the
// other 5 bits being the switch number to activate, but with 00000 being 1 etc.

// the number represenst the byte to send to the multiplexer, the commented
// number is the switch it actually corresponds to. For the four well version,
// this is simply the first 4 switches nominally.
uint8_t well_address_input_shift_registers[] = {
  0, // 1
  1, // 2
  2, // 3
  3, // 4
};
