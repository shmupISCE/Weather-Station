#include "iAQ-core.h"

void read_prediction_data(aq_info *data){
    //  Test if it reads LSB or MSB first [Maybe needed to read all 8 bytes]
    uint8_t reg_val[2] = I2C1_ReadDataBlock(AQ_DEV_ADDR, AQ_READ_DATA, 2);
    data->prediction = (reg_val[0] * (1 << 8)) + reg_val[1];
}

void full_burst_read(aq_info *data){
    uint8_t reg_val[8] = I2C1_ReadDataBlock(AQ_DEV_ADDR, AQ_READ_DATA, 8);

    data->prediction = (reg_val[0] * (1 << 8)) + reg_val[1];
    data->status = reg_val[2];
    data->resistance = (reg_val[3] * (1<<16)) + (reg_val[4] * (1 << 8)) + reg_val[5];
    data->Tvoc_prediction = (reg_val[6] * (1<<8)) + reg_val[7];
}