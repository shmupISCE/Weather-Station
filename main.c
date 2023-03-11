#include "mcc_generated_files/mcc.h"
#include "BME280.h"     //Weather click
#include "OPT3001.h"    //Ambient 2 click
#include <stdio.h>


void print_bme280_settings(BME280_DeviceSettings *settings){
    printf("Temperature oversampling: %s\n", settings->Temperature_oversampling);
    printf("Pressure oversampling: %s\n", settings->Pressure_oversampling);
    printf("Humidity oversampling: %s\n", settings->Humidity_oversampling);
    printf("Filter: %s\n", settings->Filter);
    printf("Time standby: %s\n", settings->Time_standby);
}

void main(void){
    // Initialize the device
    SYSTEM_Initialize();
    
    bme280_calib_data calib_data;
    bme280_uncomp_data uncomp_data;
    bme280_data comp_data;
    
<<<<<<< HEAD
    BME280_Init();
    BME280_readFactoryCalibration(&calib_data);
            
            
    while (1) {
        //Set to Forced mode
        BME280_WakeUp()
        // Read raw data
        BME280_read_measurement(&uncomp_data);
        //Compensate raw data and return to data structure BME280_data
        BME280_compensate_data(&uncomp_data, &comp_data, &calib_data);
        BME280_NormalizeMeasurements(&comp_data);
        printf("Temperature: %d \tPressure: %d\tHumidity: %d %%", comp_data.temperature, comp_data.pressure, comp_data.humidity);
=======
    BME280_Config(BME280_OVERSAMPLING_1X,
                  BME280_OVERSAMPLING_1X, 
                  BME280_OVERSAMPLING_1X, 
                  BME280_FILTER_COEFF_OFF, 
                  BME280_STANDBY_TIME_250_MS, 
                  BME280_SLEEP_MODE);
    

    BME280_read_calibration_data(&calib_data);
            
            
    while (1) {
         BME280_WakeUp();   //Forced mode
         // Read raw data
         bme280_parse_sensor_data(&uncomp_data);
         //Compensate raw data and return to data structure bme280_data
         bme280_compensate_data(&uncomp_data, &comp_data, &calib_data);
>>>>>>> 069fe2a9022b2a3cf95112ac8b5d713f12fe5c6a
    }
}

/**
 End of File
 */














void single_shot_register(void) {
    uint8_t temp_msb_reg;
    uint8_t temp_lsb_reg;
    uint8_t temp_xlsb_reg;

    uint8_t humidity_msb_reg;
    uint8_t humidity_lsb_reg;

    uint8_t pressure_msb_reg;
    uint8_t pressure_lsb_reg;
    uint8_t pressure_xlsb_reg;

    pressure_msb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xF7);
    pressure_lsb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xF8);
    pressure_xlsb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xF9);

    temp_msb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xFA);
    temp_lsb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xFB);
    temp_xlsb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xFC);

    humidity_msb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xFD);
    humidity_lsb_reg = I2C1_Read1ByteRegister(BME280_I2C_ADDR, 0xFE);

    printf("*** Temperature registers ***");
    printf("\nTemp MSB: %d\nTemp LSB: %d\nTemp XLSB: %d\n", temp_msb_reg, temp_lsb_reg, temp_xlsb_reg);
    printf("*** Humidity registers  ***");
    printf("\nHumidity MSB: %d\nHumidity LSB: %d\n", humidity_msb_reg, humidity_lsb_reg);
    printf("*** Pressure registers  ***");
    printf("\nPressure MSB: %d\nPressure LSB: %d\nPressure XLSB: %d\n", pressure_msb_reg, pressure_lsb_reg, pressure_xlsb_reg);
}