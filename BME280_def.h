#ifndef BME280_DEF_H
#define	BME280_DEF_H

#ifdef	__cplusplus
extern "C" {
#endif

#define BME280_CHIP_ID 0X76
#define BME280_ID_REG 0xD0

#define CTRL_HUM                                  (0xF2)
#define STATUS_REG                                (0xF3) 
#define CTRL_MEAS                                 (0xF4) 
#define CONFIG_REG                                (0xF5)
                        
#define PRESS_MSB                                 (0xF7)
#define PRESS_LSB                                 (0xF8)
#define PRESS_XLSB                                (0xF9)
                        
#define TEMP_MSB                                  (0xFA)
#define TEMP_LSB                                  (0xFB)
#define TEMP_XLSB                                 (0xFC)

#define HUM_MSB                                   (0xFD)
#define HUM_LSB                                   (0xFE)

//Temperature compensations registers
#define BME280_REGISTER_DIG_T1                    (0x88)
#define BME280_REGISTER_DIG_T2                    (0x8A)
#define BME280_REGISTER_DIG_T3                    (0x8C)
             
/* )Pressure compensations registers */
#define BME280_REGISTER_DIG_P1                    (0x8E)
#define BME280_REGISTER_DIG_P2                    (0x90)
#define BME280_REGISTER_DIG_P3                    (0x92)
#define BME280_REGISTER_DIG_P4                    (0x94)
#define BME280_REGISTER_DIG_P5                    (0x96)
#define BME280_REGISTER_DIG_P6                    (0x98)
#define BME280_REGISTER_DIG_P7                    (0x9A)
#define BME280_REGISTER_DIG_P8                    (0x9C)
#define BME280_REGISTER_DIG_P9                    (0x9E)
           
/* Humidity compensations registers */
#define BME280_REGISTER_DIG_H1                    (0xA1)
#define BME280_REGISTER_DIG_H2                    (0xE1)
#define BME280_REGISTER_DIG_H3                    (0xE3)
#define BME280_REGISTER_DIG_H4                    (0xE4)
#define BME280_REGISTER_DIG_H5                    (0xE5)
#define BME280_REGISTER_DIG_H6                    (0xE7)
        
#define BME280_SLEEP_MODE                         UINT8_C(0x00)
#define BME280_FORCED_MODE                        UINT8_C(0x01)
#define BME280_NORMAL_MODE                        UINT8_C(0x03)

/* Oversampling macros */
#define BME280_NO_OVERSAMPLING                    UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X                    UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X                    UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X                    UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X                    UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X                   UINT8_C(0x05)

/* Standby duration selection */
#define BME280_STANDBY_TIME_0_5_MS                (0x00)
#define BME280_STANDBY_TIME_62_5_MS               (0x01)
#define BME280_STANDBY_TIME_125_MS                (0x02)
#define BME280_STANDBY_TIME_250_MS                (0x03)
#define BME280_STANDBY_TIME_500_MS                (0x04)
#define BME280_STANDBY_TIME_1000_MS               (0x05)
#define BME280_STANDBY_TIME_10_MS                 (0x06)
#define BME280_STANDBY_TIME_20_MS                 (0x07)

/* Filter coefficient  */
#define BME280_FILTER_COEFF_OFF                   (0x00)
#define BME280_FILTER_COEFF_2                     (0x01)
#define BME280_FILTER_COEFF_4                     (0x02)
#define BME280_FILTER_COEFF_8                     (0x03)
#define BME280_FILTER_COEFF_16                    (0x04)
    
/* Macros for bit masking */
#define BME280_SENSOR_MODE_MSK                    UINT8_C(0x03)
#define BME280_CTRL_HUM_MSK                       UINT8_C(0x07)
#define BME280_CTRL_PRESS_MSK                     UINT8_C(0x1C)
#define BME280_CTRL_TEMP_MSK                      UINT8_C(0xE0)
#define BME280_FILTER_MSK                         UINT8_C(0x1C)
#define BME280_STANDBY_MSK                        UINT8_C(0xE0)
#define BME280_SPI3W_E_MSK                        UINT8_C(0x01)

typedef struct{
    uint8_t ctrl_meas_reg;
    uint8_t config_reg;
    uint8_t ctrl_hum_reg;
}bme280_settings_reg;

typedef struct{
    uint8_t osrs_t, osrs_p, mode;
    uint8_t osrs_h;
    uint8_t standby_time, filter, spi3w_en;
}bme280_num_settings;



typedef struct{
    char Mode[11];
    char Temperature_oversampling[5];
    char Pressure_oversampling[5];
    char Humidity_oversampling[5];
    char Filter[5];
    char Time_standby[5];
}BME280_DeviceSettings;


//Raw sensor measurement data from BME280
typedef struct{
    /*< un-compensated pressure */
    int32_t pressure;

    /*< un-compensated temperature */
    int32_t temperature;

    /*< un-compensated humidity */
    uint16_t humidity;

}bme280_uncomp_data;

typedef struct {
    /*< Compensated pressure */
    float pressure;

    /*< Compensated temperature */
    float temperature;

    /*< Compensated humidity */
    float humidity;

}bme280_data;

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

#ifdef	__cplusplus
}
#endif

#endif	/* BME280_DEF_H */

