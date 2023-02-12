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

typedef struct{
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
  int32_t t_fine;   //Variable to store the intermediate temperature coefficient
} bme280_calib_data;


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

void read_all(bme280_uncomp_data *data);
void default_bme280_init(void)
void bme280_compensate_data(const bme280_uncomp_data *uncomp_data,
                               bme280_data *comp_data,
                               bme280_calib_data *calib_data);

// State functions 
void default_sleep_mode(void);
void default_normal_mode(void);


#endif