#ifndef iAQ-core
#define iAQ-core 

#include <stdint.h>

/*  CO2 sensor  */
#define AQ_DEV_ADDR 0x5A
#define AQ_READ_DATA 0xB5


typedef struct{
    uint16_t prediction;
    uint8_t status;
    int32_t resistance;
    uint16_t Tvoc_prediction;
}aq_info;

void read_prediction_data(aq_info *data);
void full_burst_read(aq_info *data);

#endif // !iAQ-core
