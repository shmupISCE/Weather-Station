#ifndef BME280_H
#define BME280_H

#include <stdint.h>


#define BME280_CHIP_ID 0x60
#define BME280_ID_REG 0xD0

#define MEAN_SEA_LEVEL_PRESSURE 1013


typedef struct {
    /*< Compensated pressure */
    double pressure;

    /*< Compensated temperature */
    double temperature;

    /*< Compensated humidity */
    double humidity;

}bme280_data;

//Raw sensor measurement data from BME280
typedef struct{
    /*< un-compensated pressure */
    uint32_t pressure;

    /*< un-compensated temperature */
    uint32_t temperature;

    /*< un-compensated humidity */
    uint16_t humidity;

}bme280_uncomp_data;

void read_to_struct(bme280_uncomp_data *data)
void read_raw_data(void)

//Get functions
uint16_t get_temperature(void);
uint16_t get_pressure(void);
uint16_t get_humidity(void);

// Read individual to data struct
void read_temperature(bme280_uncomp_data *data);
void read_pressure(bme280_uncomp_data *data);
void read_humidity(bme280_uncomp_data *data);

// State functions 
void default_sleep_mode(void);
void default_normal_mode(void);


#endif