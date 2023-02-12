#ifndef OPT3001_H
#define OPT3001_H

#include <stdint.h>

#define OPT3001_RESULT_REG      0x00
#define OPT3001_CONFIG_REG      0x01
#define OPT3001_DEV_ID          0x7F

float calculate_lux(void);
void config_sensor(void);
uint16_t get_data(void);


#endif  //OPT3001_H