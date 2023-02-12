#ifndef OPT3001_H
#define OPT3001_H

#include <stdint.h>

#define OPT3001_RESULT      0x00
#define OPT3001_CONFIG      0x01
/*  Manufacturer ID   */
#define OPT3001_MANF_ID     0x7E
/*  Device ID   */
#define OPT3001_DEV_ID      0x7F

double calculate_lux(void);
void config_sensor(void);
uint16_t get_data(void);


#endif