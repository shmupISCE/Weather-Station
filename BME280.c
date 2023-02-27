#include "BME280.h"


void read_temperature(bme280_uncomp_data *data){
    uint8_t temp[3];
    I2C1_ReadDataBlock(BME280_I2C_ADDR, TEMP_MSB, temp, 3);
    data->temperature = (((temp[0] << 8) | temp[1]) << 8 | temp[2]);
    data->temperature = data->temperature >> 4;
}

// Read pressure into BME280 data type structure
void read_pressure(bme280_uncomp_data *data){
    uint8_t temp[3];
    I2C1_ReadDataBlock(BME280_I2C_ADDR, PRESS_MSB, temp, 3);
    data->pressure = (((temp[0] << 8) | temp[1]) << 8 | temp[2]);
    data->pressure = data->pressure >> 4;
}

// Read humidity into BME280 data type structure
void read_humidity(bme280_uncomp_data *data){   
    uint8_t temp[2];
    I2C1_ReadDataBlock(BME280_I2C_ADDR, HUM_MSB, temp, 2);

    data->humidity = ((temp[0] << 8) | temp[1]);
}

/*
*@brief Reads 8 bit register
*@param reg_addr: register address
*@return uint8_t value of register
*/
uint8_t BME280_Read1ByteRegister(uint8_t reg_addr){
    uint8_t buffer = I2C1_Read1ByteRegister(BME280_I2C_ADDR, reg_addr);
    return buffer;
}

/*
*@brief Reads 16 bit register (or two 8bit registers).
Can be used to read one 16bit register or two stacked 8bit registers.
*@param reg_addr: register address
*@return uint16_t value of registers
*/
uint16_t BME280_Read2ByteRegister(uint8_t reg_addr){
    uint16_t buffer = I2C1_Read2ByteRegister(BME280_I2C_ADDR, reg_addr);
    return buffer;
}

/*
*@brief Writes 8bit value to 8 bit register
*@param reg_addr: register address
*@param data_tw: data to write to register
*/
void BME280_Write1ByteRegister(uint8_t reg_addr, uint8_t data_tw){
    I2C1_Write1ByteRegister(BME280_I2C_ADDR, reg_addr, data_tw);
}

/*
*@brief Write 16 bit value to 16 bit register (or two 8bit registers).
*@param reg_addr: register address
*@param data_tw: data to write to register
*/
void BME280_Write2ByteRegister(uint8_t reg_addr, uint16_t data_tw){
    I2C1_Write2ByteRegister(BME280_I2C_ADDR, reg_addr, data_tw);
}

/*
*@brief ReadnReplace function first reads the value from 8 bit register into a buffer. 
*Then it changes only the values of specified bits.
*Can be used to write to a register that has reserved or read only bits.
*Also can be used to change 1 bit.
*@param reg_addr: register address
*@param value: mask to changes specified bits
*/
void BME280_ReadnReplace(uint8_t reg_addr, uint8_t value){
    uint8_t buffer = BME280_Read1ByteRegister(reg_addr);
    uint8_t new_reg = (buffer | value);
    BME280_Write1ByteRegister(reg_addr, new_reg);
}
/*
*@brief Initializes the BME280 into forced mode
*/
void BME280_init(){
    BME280_ReadnReplace(CTRL_HUM, 0x01);
    BME280_ReadnReplace(CONFIG_REG, 0x80);
    BME280_Write1ByteRegister(CTRL_MEAS, BME280_FORCED_MODE);
    
}

/*
*@brief Set operation mode of the device
*@param BME280_MODE: SLEEP MODE - sets device to sleep mode
*@param BME280_MODE: FORCED MODE - sets device to forced mode
*@param BME280_MODE: NORMAL MODE - sets device to normal mode
*/
void BME280_set_mode(uint8_t BME280_MODE){
    uint8_t ctrl_meas_reg = BME280_Read1ByteRegister(CTRL_MEAS);
    ctrl_meas_reg = (ctrl_meas_reg & ~BME280_SENSOR_MODE_MSK) | BME280_MODE;
    BME280_Write1ByteRegister(CTRL_MEAS, ctrl_meas_reg);
}

//Test 
void BME280_change_settings(uint8_t reg, uint8_t setting_mask, uint8_t new_setting){
    uint8_t reg_val = BME280_Read1ByteRegister(reg);
    reg_val = (reg_val & ~setting_mask) | new_setting;
    BME280_Write1ByteRegister(reg, reg_val);
}

/*
*@brief Burst read registers for device configuration 
*/
static void bme280_read_settings_reg(bme280_settings_reg* settings){
    uint8_t register_buffer[4];
    I2C1_ReadDataBlock(BME280_I2C_ADDR, CTRL_HUM, register_buffer, 4);

    settings->ctrl_hum_reg= register_buffer[0];
    settings->ctrl_meas_reg = register_buffer[2];
    settings->config_reg= register_buffer[3];
}

void _bme280_get_settings(bme280_num_settings *settings){
    bme280_settings_reg reg_settings;
    bme280_read_settings_reg(&reg_settings)
    
    settings-> mode = reg_settings.ctrl_meas_reg | BME280_SENSOR_MODE_MSK;
    settings-> osrs_p = ((reg_settings.ctrl_meas_reg | BME280_CTRL_PRESS_MSK) << 3) >> 6;
    settings-> osrs_t = (reg_settings.ctrl_meas_reg | BME280_CTRL_TEMP_MSK) >> 6;
    
    settings-> osrs_h = reg_settings.ctrl_hum_reg | BME280_CTRL_HUM_MSK;
    
    settings-> spi3w_en = reg_settings.config_reg | BME280_SPI3W_E_MSK;
    settings-> filter = ((reg_settings.config_reg | BME280_FILTER_MSK) << 3) >> 6;
    settings-> standby_time = (reg_settings.config_reg | BME280_STANDBY_MSK) >> 6;
}

static void parse_temp_press_calib_data(bme280_calib_data *calib_data){
    uint8_t reg_data[26];
    
    I2C1_ReadDataBlock(BME280_I2C_ADDR, BME280_TEMP_PRESS_CALIB_DATA_ADDR, reg_data, 26);

    calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    
    calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data->dig_h1 = reg_data[25];
}

static void parse_humidity_calib_data(bme280_calib_data *calib_data){
    uint8_t reg_data[7];
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;
    I2C1_ReadDataBlock(BME280_I2C_ADDR, BME280_HUMIDITY_CALIB_DATA_ADDR, reg_data, 7);

    calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
    calib_data->dig_h6 = (int8_t)reg_data[6];
}


void bme280_parse_sensor_data(bme280_uncomp_data *uncomp_data){
    uint8_t reg_data[8];
    I2C1_ReadDataBlock(BME280_I2C_ADDR, PRESS_MSB, reg_data, 8);
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (uint32_t)reg_data[0] << 12;
    data_lsb = (uint32_t)reg_data[1] << 4;
    data_xlsb = (uint32_t)reg_data[2] >> 4;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t)reg_data[3] << 12;
    data_lsb = (uint32_t)reg_data[4] << 4;
    data_xlsb = (uint32_t)reg_data[5] >> 4;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb = (uint32_t)reg_data[6] << 8;
    data_lsb = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}

static uint32_t compensate_pressure(struct bme280_uncomp_data *uncomp_data,
                                    struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_p6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_p5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data->dig_p4) * 65536);
    var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data->dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_p1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000){
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else{
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)calib_data->dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_p8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_p7) / 16));

        if (pressure < pressure_min){
            pressure = pressure_min;
        }
        else if (pressure > pressure_max){
            pressure = pressure_max;
        }
    }
    else{
        pressure = pressure_min;
    }
    return pressure;
}
static uint32_t compensate_humidity(struct bme280_uncomp_data *uncomp_data,
                                    struct bme280_calib_data *calib_data){
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncomp_data->humidity * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
    var4 = ((int32_t)calib_data->dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max){
        humidity = humidity_max;
    }
    return humidity;
}
static uint32_t compensate_pressure(struct bme280_uncomp_data *uncomp_data,
                                    struct bme280_calib_data *calib_data){
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t pressure;
    uint32_t pressure_min = 3000000;
    uint32_t pressure_max = 11000000;

    var1 = ((int64_t)calib_data->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data->dig_p6;
    var2 = var2 + ((var1 * (int64_t)calib_data->dig_p5) * 131072);
    var2 = var2 + (((int64_t)calib_data->dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calib_data->dig_p3) / 256) + ((var1 * ((int64_t)calib_data->dig_p2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)calib_data->dig_p1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0){
        var4 = 1048576 - uncomp_data->pressure;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t)calib_data->dig_p9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)calib_data->dig_p8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calib_data->dig_p7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (pressure < pressure_min){
            pressure = pressure_min;
        }
        else if (pressure > pressure_max){
            pressure = pressure_max;
        }
    }
    else{
        pressure = pressure_min;
    }
    return pressure;
}




void bme280_compensate_data(const bme280_uncomp_data *uncomp_data,
                            bme280_data *comp_data,
                            bme280_calib_data *calib_data){
    
    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)){
        /* Initalize to zero */
        comp_data->temperature = 0;
        comp_data->pressure = 0;
        comp_data->humidity = 0;


        /* Compensate the temperature data */
        comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        
        /* Compensate the pressure data */
        comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        
        /* Compensate the humidity data */
        comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
    }
}























void bme280_parse_settings(BME280_DeviceSettings *settings){
    bme280_num_settings n_settings;
    _bme280_get_settings(&n_settings);
    
    //  Device operational mode
    switch(n_settings.mode){
        case BME280_SLEEP_MODE:
            settings->Mode = "Sleep mode";
            break;
        case BME280_FORCED_MODE:
            settings->Mode = "Forced mode";
            break;
        case BME280_NORMAL_MODE:
            settings->Mode = "Normal mode";
            break;
        }
    // Temperature oversampling
    switch(n_settings.osrs_t){
        case BME280_NO_OVERSAMPLING:
            settings->Temperature_oversampling = "OFF";
            break;
        case BME280_OVERSAMPLING_1X:
            settings->Temperature_oversampling = "x1";
            break;
        case BME280_OVERSAMPLING_2X:
            settings->Temperature_oversampling = "x2";
            break;
        case BME280_OVERSAMPLING_4X:
            settings->Temperature_oversampling = "x4";
            break;
        case BME280_OVERSAMPLING_8X:
            settings->Temperature_oversampling = "x8";
            break;
        case BME280_OVERSAMPLING_16X:
            settings->Temperature_oversampling = "x16";
            break;
    }
    // Pressure oversampling
    switch(n_settings.osrs_p){
        case BME280_NO_OVERSAMPLING:
            settings->Pressure_oversampling = "OFF";
            break;
        case BME280_OVERSAMPLING_1X:
            settings->Pressure_oversampling = "x1";
            break;
        case BME280_OVERSAMPLING_2X:
            settings->Pressure_oversampling = "x2";
            break;
        case BME280_OVERSAMPLING_4X:
            settings->Pressure_oversampling = "x4";
            break;
        case BME280_OVERSAMPLING_8X:
            settings->Pressure_oversampling = "x8";
            break;
        case BME280_OVERSAMPLING_16X:
            settings->Pressure_oversampling = "x16";
            break;
    }
    // Humidity oversampling
    switch(n_settings.osrs_h){
        case BME280_NO_OVERSAMPLING:
            settings->Humidity_oversampling= "OFF";
            break;
        case BME280_OVERSAMPLING_1X:
            settings->Humidity_oversampling = "x1";
            break;
        case BME280_OVERSAMPLING_2X:
            settings->Humidity_oversampling = "x2";
            break;
        case BME280_OVERSAMPLING_4X:
            settings->Humidity_oversampling = "x4";
            break;
        case BME280_OVERSAMPLING_8X:
            settings->Humidity_oversampling = "x8";
            break;
        case BME280_OVERSAMPLING_16X:
            settings->Humidity_oversampling = "x16";
            break;
    }
    switch(n_settings.filter){
        case BME280_FILTER_COEFF_OFF:
            settings->Filter = "OFF";
            break;
        case BME280_FILTER_COEFF_2:
            settings->Filter = "x2";
            break;
        case BME280_FILTER_COEFF_4:
            settings->Filter = "x4";
            break;
        case BME280_FILTER_COEFF_8:
            settings->Filter = "x8";
            break;
        case BME280_FILTER_COEFF_16:
            settings->Filter = "x16";
            break;
    }
    
    switch(n_settings.standby_time){
        case BME280_STANDBY_TIME_0_5_MS:
            settings->Time_standby = "0.5 ms";
            break;
        case BME280_STANDBY_TIME_62_5_MS:
            settings->Time_standby = "62.5 ms";
            break;
        case BME280_STANDBY_TIME_125_MS:
            settings->Time_standby = "125 ms";
            break;
        case BME280_STANDBY_TIME_250_MS:
            settings->Time_standby = "250 ms";
            break;
        case BME280_STANDBY_TIME_500_MS:
            settings->Time_standby = "500 ms";
            break;
        case BME280_STANDBY_TIME_1000_MS:
            settings->Time_standby = "1000 ms";
            break;
        case BME280_STANDBY_TIME_10_MS:
            settings->Time_standby = "10 ms";
            break;
        case BME280_STANDBY_TIME_20_MS:
            settings->Time_standby = "20 ms";
            break;
    }
}

