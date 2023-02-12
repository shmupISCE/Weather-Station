#include "OPT3001.h"

#define OPT3001_LL_ADDR     0x02
#define OPT3001_HL_ADDR     0x03
#define FULL_SCALE_MODE     0x0C

static void set_fs_mode(void){
    /*0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0*/
    /*1100000000000000*/
    int16_t current_reg_val = I2C1_Read2ByteRegister(OPT3001_DEV_ID, OPT3001_CONFIG);
    int16_t fs_reg = current_reg_val | 0xC000;
    I2C1_Write2ByteRegister(OPT3001_DEV_ID, OPT3001_CONFIG, fs_reg);
}

int16_t get_data(void){
    int16_t lumen = I2C1_Read2ByteRegister(OPT3001_DEV_ID, OPT3001_RESULT);
    return lumen;
}
/*  Returns the calculated value in lux */
double calculate_lux(void){
    double lux = 0;
    int16_t value = get_data();

    int8_t exp = value >> 12;
    int16_t frac = value & 0x0FFF;

    lux = 0.01 * (1 << exp) * frac;
    return lux;  
}