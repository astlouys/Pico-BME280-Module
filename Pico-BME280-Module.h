/* ============================================================================================================================================================= *\
   Pico-BME280-Module.h
   St-Louys Andre - May 2025
   astlouys@gmail.com
   Revision 12-MAY-2025
   Langage: C
\* ============================================================================================================================================================= */

#ifndef __PICO_BME280_MODULE_H
#define __PICO_BME280_MODULE_H



/* $PAGE */
/* $TITLE=Include files. */
/* ============================================================================================================================================================= *\
                                                                      Include files.
\* ============================================================================================================================================================= */
#include "hardware/gpio.h"
#include "hardware/i2c.h"



/* $PAGE */
/* $TITLE=Definitions. */
/* ============================================================================================================================================================= *\
                                                                        Definitions.
\* ============================================================================================================================================================= */
/* GPIO used for BME280. */
// GPIO 06   I2C     I2C SDA (data  line for DS3231).
// GPIO 07   I2C     I2C SCL (clock line for DS3231).
#define SDA                          6
#define SCL                          7
#define I2C_PORT                  i2c1

#define BME280_ADDRESS 0x77

#define BME280_REGISTER_UNIQUE_ID  0x83;

#define BME280_REGISTER_CALIB00    0x88
#define BME280_REGISTER_CALIB01    0x89
#define BME280_REGISTER_CALIB02    0x8A
#define BME280_REGISTER_CALIB03    0x8B
#define BME280_REGISTER_CALIB04    0x8C
#define BME280_REGISTER_CALIB05    0x8D
#define BME280_REGISTER_CALIB06    0x8E
#define BME280_REGISTER_CALIB07    0x8F
#define BME280_REGISTER_CALIB08    0x90
#define BME280_REGISTER_CALIB09    0x91
#define BME280_REGISTER_CALIB10    0x92
#define BME280_REGISTER_CALIB11    0x93
#define BME280_REGISTER_CALIB12    0x94
#define BME280_REGISTER_CALIB13    0x95
#define BME280_REGISTER_CALIB14    0x96
#define BME280_REGISTER_CALIB15    0x97
#define BME280_REGISTER_CALIB16    0x98
#define BME280_REGISTER_CALIB17    0x99
#define BME280_REGISTER_CALIB18    0x9A
#define BME280_REGISTER_CALIB19    0x9B
#define BME280_REGISTER_CALIB20    0x9C
#define BME280_REGISTER_CALIB21    0x9D
#define BME280_REGISTER_CALIB22    0x9E
#define BME280_REGISTER_CALIB23    0x9F
#define BME280_REGISTER_CALIB24    0xA0
#define BME280_REGISTER_CALIB25    0xA1

#define BME280_REGISTER_ID         0xD0
#define BME280_REGISTER_RESET      0xE0
 
#define BME280_REGISTER_CALIB26    0xE1
#define BME280_REGISTER_CALIB27    0xE2
#define BME280_REGISTER_CALIB28    0xE3
#define BME280_REGISTER_CALIB29    0xE4
#define BME280_REGISTER_CALIB30    0xE5
#define BME280_REGISTER_CALIB31    0xE6
#define BME280_REGISTER_CALIB32    0xE7
#define BME280_REGISTER_CALIB33    0xE8
#define BME280_REGISTER_CALIB34    0xE9
#define BME280_REGISTER_CALIB35    0xEA
#define BME280_REGISTER_CALIB36    0xEB
#define BME280_REGISTER_CALIB37    0xEC
#define BME280_REGISTER_CALIB38    0xED
#define BME280_REGISTER_CALIB39    0xEE
#define BME280_REGISTER_CALIB40    0xEF
#define BME280_REGISTER_CALIB41    0xF0

#define BME280_REGISTER_CTRL_HUM   0xF2
#define BME280_REGISTER_STATUS     0xF3
#define BME280_REGISTER_CTRL_MEAS  0xF4
#define BME280_REGISTER_CONFIG     0xF5

#define BME280_REGISTER_PRESS_MSB  0xF7
#define BME280_REGISTER_PRESS_LSB  0xF8
#define BME280_REGISTER_PRESS_XLSB 0xF9

#define BME280_REGISTER_TEMP_MSB   0xFA
#define BME280_REGISTER_TEMP_LSB   0xFB
#define BME280_REGISTER_TEMP_XLSB  0xFC

#define BME280_REGISTER_HUM_MSB    0xFD
#define BME280_REGISTER_HUM_LSB    0xFE



/* $PAGE */
/* $TITLE=Variable definitions. */
/* ============================================================================================================================================================= *\
                                                                      Variable definitions.
\* ============================================================================================================================================================= */
/* Main structure containing all BME280-related data. */
struct struct_bme280
{
  /* Generic parameters. */
  UINT8  DeviceId;    // a "true" BME280 should have 0x60 as device Id.
  UINT32 UniqueId;
  UINT32 Errors;      // cumulative number of errors while trying to read BME280 sensor.
  UINT32 ReadCycles;  // cumulative number of read cycles from BME280.

  /* Calibration data for the specific BME280 used (stored in BME280's non-volatile memory). */
  UINT8  CalibData[42];
  
  /* BME280 calibration parameters computed from data written in the device. */
  /* Temperature-related parameters. */
  UINT16  DigT1;
  INT16   DigT2;
  INT16   DigT3;

  /* Pressure-related parameters. */
  UINT16  DigP1;
  INT16   DigP2;
  INT16   DigP3;
  INT16   DigP4;
  INT16   DigP5;
  INT16   DigP6;
  INT16   DigP7;
  INT16   DigP8;
  INT16   DigP9;

  /* Humidity-related parameters. */
  UCHAR   DigH1;
  INT16   DigH2;
  UCHAR   DigH3;
  INT16   DigH4;
  INT16   DigH5;
  CHAR    DigH6;

  /* Return values. */
  float TemperatureC;
  float TemperatureF;
  float Humidity;
  float Pressure;
  float Altitude; // not trusted value.
  float Humidex;  // wrong value for now.
};



/* $PAGE */
/* $TITLE=Function prototypes. */
/* ============================================================================================================================================================= *\
                                                                     Function prototypes.
\* ============================================================================================================================================================= */
/* Read temperature, humidity and barometric pressure. */
UINT8 bme280_get_temp(struct struct_bme280 *StructBME280);

/* Compute calibration parameters from calibration data stored in the specific BME280 device used. */
UINT8 bme280_compute_calib_param(struct struct_bme280 *StructBME280);

/* Soft-reset the BME280. */
UINT8 bme280_init(void);

/* Read BME280 calibration data from the device's non volatile memory. */
UINT8 bme280_read_calib_data(struct struct_bme280 *StructBME280);

/* Read BME280 configuration. */
UINT8 bme280_read_config(void);

/* Read BME280 device ID. */
UINT8 bme280_read_device_id(void);

/* Read extra BME280 registers. */
UINT8 bme280_read_all_registers();

/* Read all BME280 data registers (temperature, humidity and barometric pressure). */
UINT8 bme280_read_registers(UINT8 *Register);

/* Read BME280 current status. */
UINT8 bme280_read_status(void);

/* Read BME280 unique id ("serial number" of the specific device used). */
UINT32 bme280_read_unique_id(void);

#endif  // __PICO_BME280_MODULE_H
