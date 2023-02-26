#include "BME280.h"


void read_temperature(bme280_uncomp_data *data){
    uint8_t temp[3];
    I2C1_ReadDataBlock(BME280_CHIP_ID, TEMP_MSB, temp, 3);
    data->temperature = (((temp[0] << 8) | temp[1]) << 8 | temp[2]);
    data->temperature = data->temperature >> 4;
}

// Read pressure into BME280 data type structure
void read_pressure(bme280_uncomp_data *data){
    uint8_t temp[3];
    I2C1_ReadDataBlock(BME280_CHIP_ID, PRESS_MSB, temp, 3);
    data->pressure = (((temp[0] << 8) | temp[1]) << 8 | temp[2]);
    data->pressure = data->pressure >> 4;
}

// Read humidity into BME280 data type structure
void read_humidity(bme280_uncomp_data *data){   
    uint8_t temp[2];
    I2C1_ReadDataBlock(BME280_CHIP_ID, HUM_MSB, temp, 2);

    data->humidity = ((temp[0] << 8) | temp[1]);
}

/*
*@brief Reads 8 bit register
*@param reg_addr: register address
*@return uint8_t value of register
*/
uint8_t BME280_Read1ByteRegister(uint8_t reg_addr){
    uint8_t buffer = I2C1_Read1ByteRegister(BME280_CHIP_ID, reg_addr);
    return buffer;
}

/*
*@brief Reads 16 bit register (or two 8bit registers).
Can be used to read one 16bit register or two stacked 8bit registers.
*@param reg_addr: register address
*@return uint16_t value of registers
*/
uint16_t BME280_Read2ByteRegister(uint8_t reg_addr){
    uint16_t buffer = I2C1_Read2ByteRegister(BME280_CHIP_ID, reg_addr);
    return buffer;
}

/*
*@brief Writes 8bit value to 8 bit register
*@param reg_addr: register address
*@param data_tw: data to write to register
*/
void BME280_Write1ByteRegister(uint8_t reg_addr, uint8_t data_tw){
    I2C1_Write1ByteRegister(BME280_CHIP_ID, reg_addr, data_tw);
}

/*
*@brief Write 16 bit value to 16 bit register (or two 8bit registers).
*@param reg_addr: register address
*@param data_tw: data to write to register
*/
void BME280_Write2ByteRegister(uint8_t reg_addr, uint16_t data_tw){
    I2C1_Write2ByteRegister(BME280_CHIP_ID, reg_addr, data_tw);
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
    BME280_Write1ByteRegister(CTRL_MEAS, BME280_SLEEP_MODE);
    BME280_ReadnReplace(CTRL_HUM, 0x01);
    BME280_ReadnReplace(CONFIG_REG, 0x80);
    BME280_Write1ByteRegister(CTRL_MEAS, BME280_FORCED_MODE);
    
}

void BME280_read_calibration_data(bme280_calib_data *data) {
    //Burst read from 0x88 to 0xA1 [0xA0 not used]
    uint8_t temp_calib[26];
    uint8_t humidity_calib[7];
    I2C1_ReadDataBlock(BME280_CHIP_ID, BME280_REGISTER_DIG_T1, temp_calib, 26);
    I2C1_ReadDataBlock(BME280_CHIP_ID,BME280_REGISTER_DIG_H2, humidity_calib, 7);

    data->dig_T1 = (temp_calib[0] << 8) | temp_calib[1];
    data->dig_T2 = (temp_calib[2] << 8) | temp_calib[3];
    data->dig_T3 = (temp_calib[4] << 8) | temp_calib[5];

    data->dig_P1 = (temp_calib[6] << 8) | temp_calib[7];
    data->dig_P2 = (temp_calib[8] << 8) | temp_calib[9];
    data->dig_P3 = (temp_calib[10] << 8) | temp_calib[11];
    data->dig_P4 = (temp_calib[12] << 8) | temp_calib[13];
    data->dig_P5 = (temp_calib[14] << 8) | temp_calib[15];
    data->dig_P6 = (temp_calib[16] << 8) | temp_calib[17];
    data->dig_P7 = (temp_calib[18] << 8) | temp_calib[19];
    data->dig_P8 = (temp_calib[20] << 8) | temp_calib[21];
    data->dig_P9 = (temp_calib[22] << 8) | temp_calib[23];

    data->dig_H1 = temp_calib[25];
    data->dig_H2 = (humidity_calib[0] << 8) | humidity_calib[1];
    data->dig_H3 = humidity_calib[2];
    data->dig_H4 = (((humidity_calib[3] << 8) | humidity_calib[4]) << 4) | (humidity_calib[5] & 0xF);
    data->dig_H5 = humidity_calib[5];
    data->dig_H6 = humidity_calib[6];
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
    I2C1_ReadDataBlock(BME280_CHIP_ID, CTRL_HUM, register_buffer, 4);

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

