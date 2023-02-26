#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <xc.h>
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "BME280_def.h"



void BME280_init(void);
void BME280_read_calibration_data(bme280_calib_data *data);

void bme280_parse_settings(BME280_DeviceSettings *settings);
void BME280_set_mode(uint8_t BME280_MODE);

void read_temperature(bme280_uncomp_data *data);
void read_pressure(bme280_uncomp_data *data);
void read_humidity(bme280_uncomp_data *data);


uint8_t BME280_Read1ByteRegister(uint8_t reg_addr);
uint16_t BME280_Read2ByteRegister(uint8_t reg_addr);

void BME280_Write1ByteRegister(uint8_t reg_addr, uint8_t data_tw);
void BME280_Write2ByteRegister(uint8_t reg_addr, uint16_t data_tw);


#endif	/* BME280_H */

