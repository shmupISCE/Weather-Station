#include "BME280.h"
#include "iAQ-core.h"
#include "OPT3001.h"

void main(void){
    // All data types for BME280
    bme280_data *calculated_data;
    bme280_uncomp_data *uncomp_data;
    bme280_calib_data *calib_data;
    // Initialize the sensor
    default_bme280_init();
    // Burst read raw data from register
    read_all(uncomp_data);
    bme280_compensate_data(uncomp_data, calculated_data, calib_data);

}