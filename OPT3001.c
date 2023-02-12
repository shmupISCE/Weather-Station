#include "OPT3001.h"

#define OPT3001_LL_ADDR     0x02
#define OPT3001_HL_ADDR     0x03
#define FULL_SCALE_MODE     0x0C

static void set_fs_mode(void){
    /*0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0*/
    /*1100000000000000*/
    int16_t current_reg_val = I2C1_Read2ByteRegister(OPT3001_DEV_ID, OPT3001_CONFIG_REG);
    int16_t fs_reg = current_reg_val | 0xC000;
    I2C1_Write2ByteRegister(OPT3001_DEV_ID, OPT3001_CONFIG_REG, fs_reg);
}

int16_t get_data(void){
    int16_t lumen = I2C1_Read2ByteRegister(OPT3001_DEV_ID, OPT3001_RESULT_REG);
    return lumen;
}
/*  Returns the calculated value in lux */
float calculate_lux(void){
    float lux = 0.0;
    int16_t register_value = get_data();
    
    uint16_t exponent = register_value >> 12;
    exponent = (1 << exponent);     // 1 << N is same as 2^N

    float lsb_size = 0.01 * exponent;

    uint16_t remain = register_value << 4;   //WHAT?!
    remain = remain >> 4;

    float lux = lsb_size * remain;
    return lux;
}