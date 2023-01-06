#include "BME280.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"

#define SLEEP_MODE 0x24     // Oversampling set to x1
#define FORCED_MODE 0x25    // Oversampling set to x1
#define NORMAL_MODE 0x27    // Oversampling set to x1

#define CTRL_HUM 0xF2
#define STATUS_REG 0xF3
#define CTRL_MEAS 0xF4
#define CONFIG_REG 0xF5

#define PRESS_LSB 0xF8
#define PRESS_MSB 0xF7

#define TEMP_MSB 0xFA
#define TEMP_LSB 0xFB

#define HUM_MSB 0xFD
#define HUM_LSB 0xFE

//Temperature compensations registers
#define BME280_REGISTER_DIG_T1        0x88
#define BME280_REGISTER_DIG_T2        0x8A
#define BME280_REGISTER_DIG_T3        0x8C

//Pressure compensations registers
#define BME280_REGISTER_DIG_P1        0x8E
#define BME280_REGISTER_DIG_P2        0x90
#define BME280_REGISTER_DIG_P3        0x92
#define BME280_REGISTER_DIG_P4        0x94
#define BME280_REGISTER_DIG_P5        0x96
#define BME280_REGISTER_DIG_P6        0x98
#define BME280_REGISTER_DIG_P7        0x9A
#define BME280_REGISTER_DIG_P8        0x9C
#define BME280_REGISTER_DIG_P9        0x9E

//Humidity compensations registers
#define BME280_REGISTER_DIG_H1        0xA1
#define BME280_REGISTER_DIG_H2        0xE1
#define BME280_REGISTER_DIG_H3        0xE3
#define BME280_REGISTER_DIG_H4        0xE4
#define BME280_REGISTER_DIG_H5        0xE5
#define BME280_REGISTER_DIG_H6        0xE7

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


// Read temperature into bme280 data type structure
// Reads MSB first then LSB
void read_temperature(bme280_uncomp_data *data)
{
    uint8_t temp[1];
    uint8_t read_msb = I2C1_ReadDataBlock(BME280_CHIP_ID, TEMP_MSB, temp, 2);
    data->temperature = (temp[0] << 8) | temp[1];
}

// Read pressure into bme280 data type structure
void read_pressure(bme280_uncomp_data *data)
{
    uint8_t temp[1];
    uint8_t read_msb = I2C1_ReadDataBlock(BME280_CHIP_ID, PRESS_MSB, temp, 2);
    data->pressure = (temp[0] << 8) | temp[1];
}

// Read humidity into bme280 data type structure
void read_humidity(bme280_uncomp_data *data)
{   
    uint8_t temp[1];
    uint8_t read_msb = I2C1_ReadDataBlock(BME280_CHIP_ID, HUM_MSB, temp, 2);

    data->humidity = (temp[0] << 8) | temp[1];
}

//TODO: Change 16bit -> 32bit and add XL values from registers
// Returns the 16 bit value of the temperature without compensation
uint16_t get_temperature(void)
{
    uint8_t temp_val[1];
    I2C1_ReadDataBlock(BME280_CHIP_ID, TEMP_MSB, temp_val, 2);
    return temp_val = ((uint16_t)temp_val[0] << 8) | temp_val[1];
}

// Returns the 16 bit value of the pressure without compensation
uint32_t get_pressure(void)
{
    uint8_t pres_val[2];
    I2C1_ReadDataBlock(BME280_CHIP_ID, PRESS_MSB, pres_val, 2);
    return pres_val = ((uint32_t)pres_val[0] << 8) | pres_val[1];
}

// Returns the 16 bit value of the humidity without compensation
uint16_t get_humidity(void)
{
    uint8_t hum_val[1];
    I2C1_ReadDataBlock(BME280_CHIP_ID, HUM_MSB, hum_val, 2);
    return hum_val = ((uint16_t)hum_val[0] << 8) | hum_val[1];
}

static void read_calibration_data(bme280_calib_data *data) {
    data->dig_T1 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_T1);
    data->dig_T2 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_T2);
    data->dig_T3 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_T3);

    data->dig_P1 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P1);
    data->dig_P3 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P3);
    data->dig_P2 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P2);
    data->dig_P5 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P5);
    data->dig_P6 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P6);
    data->dig_P7 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P7);
    data->dig_P8 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P8);
    data->dig_P9 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P9);
    data->dig_P4 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_P4);

    data->dig_H1 = I2C1_Read1ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H1);
    data->dig_H2 = I2C1_Read2ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H2);
    data->dig_H3 = I2C1_Read1ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H3);
    data->dig_H4 = (I2C1_Read1ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H4) << 4) | (I2C1_Read1ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H4+1) & 0xF);
    data->dig_H5 = (I2C1_Read1ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H5+1) << 4) | (I2C1_Read1ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H5) >> 4);
    data->dig_H6 = I2C1_Read1ByteRegister(BME280_CHIP_ID, BME280_REGISTER_DIG_H6);

}

static double compensate_temperature(const bme280_uncomp_data *uncomp_data, bme280_calib_data *calib_data){
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = ((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_T1) / 1024.0;
    var1 = var1 * ((double) calib_data-> dig_T2);

    var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double) calib_data->dig_T1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_T3);
    calib_data->t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)      {temperature = temperature_min;}
    else if (temperature > temperature_max) {temperature = temperature_max;}
    
    return temperature;
}

static double compensate_pressure(const bme280_uncomp_data  *uncomp_data,
                                  const bme280_calib_data *calib_data){
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_P4) * 65536.0);
    var3 = ((double)calib_data->dig_P3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calib_data->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_P1);

    // Avoid exception caused by division by zero 
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_P9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)calib_data->dig_P8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)calib_data->dig_P7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else //Invalid case
    {
        pressure = pressure_min;
    }

    return pressure;
}

static double compensate_humidity(const bme280_uncomp_data *uncomp_data,
                                  const bme280_calib_data *calib_data)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)calib_data->t_fine) - 76800.0;
    var2 = (((double)calib_data->dig_H4) * 64.0 + (((double)calib_data->dig_H5) / 16384.0) * var1);
    var3 = uncomp_data->humidity - var2;
    var4 = ((double)calib_data->dig_H2) / 65536.0;
    var5 = (1.0 + (((double)calib_data->dig_H3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data->dig_H6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data->dig_H1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

/*  Packaging of read functions  */
void read_raw_data(void){
    read_temperature();
    read_humidity();
    read_pressure();
}

static void bme280_compensate_data(const struct bme280_uncomp_data *uncomp_data,
                               bme280_data *comp_data,
                               bme280_calib_data *calib_data){
    /*  Read calibration data from registers  */
    read_calibration_data();
    read_raw_data();

    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)){
    /*  Initialize to zero    */
    comp_data->temperature = 0;
    comp_data->pressure = 0;
    comp_data->humidity = 0;

    /*  Compensate the temperature data  */
    comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
    /*  Compensate the pressure data     */
    comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
    /*  Compensate the humidity data     */
    comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
    }
}
// Burst read of all the data in the data register.
void read_all(bme280_uncomp_data *data){
    //  Burst read from 0xF7 to 0xFE
    uint8_t temp[8];
    I2C1_ReadDataBlock(BME280_CHIP_ID, HUM_LSB, temp, 8);

    data->pressure = ((temp[0] << 8) | temp[1]) << 8 | temp[2];
    data->temperature = ((temp[3] << 8) | temp[4]) << 8 | temp[5];
    data->humidity = (temp[6] <<8) | temp[7];
}

/*  Controls oversampling of humidity data. */
static void bme280_osrs_h_x1(){
    uint8_t osrs_h = 0x03;  //  oversampling x1

    uint8_t reg_value = I2C1_Read1ByteRegister(BME280_CHIP_ID, CTRL_HUM);
    reg_value = reg_value | osrs_h;
    I2C1_Write1ByteRegister(BME280_CHIP_ID, CTRL_HUM, reg_value);
}

/*  Sets the normal mode with default oversampling (1x)    
    T oversampling set to x1
    P oversampling set to x1
    Normal mode 11                                      */
static void default_normal_mode(void){
    I2C1_Write1ByteRegister(BME280_CHIP_ID, CTRL_MEAS, NORMAL_MODE);
}

/*  Sets the sleep mode with default oversampling (1x)    
    T oversampling set to x1
    P oversampling set to x1
    Sleep mode 00                                       */
static void default_sleep_mode(void){
    I2C1_Write1ByteRegister(BME280_CHIP_ID, CTRL_MEAS, SLEEP_MODE);
}

/*
MEASUREMENT TIME = 8 ms
ODR_max = 1000/8 = 125 Hz
Cycle time = MEASUREMENT TIME + t_sb = 1008 ms = 1.008s
*/
void default_init(void){

    // Sleep mode to get access to all registers
    default_sleep_mode();
    // Set oversampling to x1 for humidity
    bme280_osrs_h_x1();
    // Set config register do fault values
    default_config_reg();
    // Set Forced mode
    default_forced_mode();
}

/*  Sets the forced mode with default oversampling (1x)    
    T oversampling set to x1
    P oversampling set to x1
    Forced mode 01                                     */
void default_forced_mode(){
    I2C1_Write1ByteRegister(BME280_CHIP_ID, CTRL_MEAS, SLEEP_MODE);
}
/*  t_sb = 1000ms between 2 measurements    
    Filter OFF
    3-wire SPI mode OFF
*/
void default_config_reg(){
    uint8_t register = I2C1_Read1ByteRegister(BME280_CHIP_ID, CONFIG_REG);
    /*  t_sb = 1000ms between 2 measurements    
        Filter OFF
    */
   uint8_t preset = 0xA0;
   preset = preset | register;
   I2C1_Write1ByteRegister(BME280_CHIP_ID, CONFIG_REG, preset);
}