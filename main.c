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

    BME280_Init();
    BME280_ReadFactoryCalibration(&calib_data);
            
            
    while (1) {
        //Set to Forced mode
        BME280_WakeUp()
        // Read raw data
        BME280_read_measurement(&uncomp_data);
        //Compensate raw data and return to data structure BME280_data
        BME280_compensate_data(&uncomp_data, &comp_data, &calib_data);
        BME280_NormalizeMeasurements(&comp_data);
        printf("Temperature: %d \tPressure: %d\tHumidity: %d %%", comp_data.temperature, comp_data.pressure, comp_data.humidity);

}
