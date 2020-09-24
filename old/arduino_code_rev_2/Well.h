class Well{
  public:
    uint8_t well_number;
    uint8_t ADG731_number;
    uint8_t ADG731_select_bits[3];
    uint8_t ADG731_shift_register;
    uint8_t real_data_15_8;
    uint8_t real_data_7_0;
    uint8_t imaginary_data_15_8;
    uint8_t imaginary_data_7_0;
};
