/* ============================================================================================================================================================= *\
   Pico-BME280-Module.c
   St-Louys Andre - May 2025
   astlouys@gmail.com
   Revision 19-MAY-2025
   Langage: C
   Version 1.00

   Raspberry Pi Pico C-language module to add a BME280 sensor to a user program.

   NOTE:
   This program is provided without any warranty of any kind. It is provided
   simply to help the user develop his own program.

   REVISION HISTORY:
   =================
   19-MAY-2025 1.00 - Initial release.
\* ============================================================================================================================================================= */



/* ============================================================================================================================================================= *\
                                                                               Include files.
\* ============================================================================================================================================================= */
#include "baseline.h"
#include "stdio.h"

#include "Pico-BME280-Module.h"



/* ============================================================================================================================================================= *\
                                                                                Definitions.
\* ============================================================================================================================================================= */
/// #define RELEASE_VERSION  ///



/* ============================================================================================================================================================= *\
                                                                              Global variables.
\* ============================================================================================================================================================= */



/* ============================================================================================================================================================= *\
                                                                             Function prototypes.
\* ============================================================================================================================================================= */
/* Log info to log file. */
extern void log_info(UINT LineNumber, const UCHAR *FunctionName, UCHAR *Format, ...);





/* $PAGE */
/* $TITLE=bme280_compute_calib_param() */
/* ============================================================================================================================================================= *\
                               Compute calibration parameters with the help of calibration data read from device's non volatile memory.
\* ============================================================================================================================================================= */
UINT8 bme280_compute_calib_param(struct struct_bme280 *StructBME280)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;
#endif  // RELEASE_VERSION


  UINT16 Dum1UInt16;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_compute_calib_param()\r");


  /* Compute calibration parameters for temperature. */
  StructBME280->DigT1 = (UINT16)(StructBME280->CalibData[0] + (StructBME280->CalibData[1] << 8));
  StructBME280->DigT2 =  (INT16)(StructBME280->CalibData[2] + (StructBME280->CalibData[3] << 8));
  StructBME280->DigT3 =  (INT16)(StructBME280->CalibData[4] + (StructBME280->CalibData[5] << 8));


  /* Compute calibration parameters for atmospheric pressure. */
  StructBME280->DigP1 = (UINT16)(StructBME280->CalibData[6] + (StructBME280->CalibData[7] << 8));
  StructBME280->DigP2 = (INT16)((StructBME280->CalibData[8] + (StructBME280->CalibData[9] << 8)) & 0xFFFF);

  Dum1UInt16       = (((StructBME280->CalibData[11] << 8) & 0xFF00) + StructBME280->CalibData[10]);
  StructBME280->DigP3 = (INT16)Dum1UInt16;

  Dum1UInt16       = (((StructBME280->CalibData[13] << 8) & 0xFF00) + StructBME280->CalibData[12]);
  StructBME280->DigP4 = (INT16)Dum1UInt16;

  Dum1UInt16       = (((StructBME280->CalibData[15] << 8) & 0xFF00) + StructBME280->CalibData[14]);
  StructBME280->DigP5 = (INT16)Dum1UInt16;

  Dum1UInt16       = (((StructBME280->CalibData[17] << 8) & 0xFF00) + StructBME280->CalibData[16]);
  StructBME280->DigP6 = (INT16)Dum1UInt16;

  Dum1UInt16       = (((StructBME280->CalibData[19] << 8) & 0xFF00) + StructBME280->CalibData[18]);
  StructBME280->DigP7 = (INT16)Dum1UInt16;

  Dum1UInt16       = (((StructBME280->CalibData[21] << 8) & 0xFF00) + StructBME280->CalibData[20]);
  StructBME280->DigP8 = (INT16)Dum1UInt16;

  Dum1UInt16       = (((StructBME280->CalibData[23] << 8) & 0xFF00) + StructBME280->CalibData[22]);
  StructBME280->DigP9 = (INT16)Dum1UInt16;


  /* Compute calibration parameters for humidity. */
  StructBME280->DigH1 =   (UCHAR)StructBME280->CalibData[25];
  StructBME280->DigH2 =  (INT16)(StructBME280->CalibData[26] + (StructBME280->CalibData[27] << 8));
  StructBME280->DigH3 =   (UCHAR)StructBME280->CalibData[28];
  StructBME280->DigH4 = (INT16)((StructBME280->CalibData[29] << 4) +  (StructBME280->CalibData[30] & 0x0F));
  StructBME280->DigH5 = (INT16)((StructBME280->CalibData[31] << 4) + ((StructBME280->CalibData[30] & 0xF0) >> 4));
  StructBME280->DigH6 =    (CHAR)StructBME280->CalibData[32];


  if (FlagLocalDebug)
  {
    log_info(__LINE__, __func__, "Temperature:\r");
    log_info(__LINE__, __func__, "DigT1: 0x%8.8X   %7u (UINT16)\r",    StructBME280->DigT1, StructBME280->DigT1);
    log_info(__LINE__, __func__, "DigT2: 0x%8.8X   %7d (INT16)\r",     StructBME280->DigT2, StructBME280->DigT2);
    log_info(__LINE__, __func__, "DigT3: 0x%8.8X   %7d (INT16)\r\r\r", StructBME280->DigT3, StructBME280->DigT3);

    log_info(__LINE__, __func__, "Pressure:\r");
    log_info(__LINE__, __func__, "DigP1: 0x%8.8X   %7u (UINT16)\r",    StructBME280->DigP1,           StructBME280->DigP1);
    log_info(__LINE__, __func__, "DigP2: 0x%8.8X   %7d (INT16)\r",    (StructBME280->DigP2 & 0xFFFF), StructBME280->DigP2);
    log_info(__LINE__, __func__, "DigP3: 0x%8.8X   %7d (INT16)\r",     StructBME280->DigP3,           StructBME280->DigP3);
    log_info(__LINE__, __func__, "DigP4: 0x%8.8X   %7d (INT16)\r",     StructBME280->DigP4,           StructBME280->DigP4);
    log_info(__LINE__, __func__, "DigP5: 0x%8.8X   %7d (INT16)\r",     StructBME280->DigP5,           StructBME280->DigP5);
    log_info(__LINE__, __func__, "DigP6: 0x%8.8X   %7d (INT16)\r",    (StructBME280->DigP6 & 0xFFFF), StructBME280->DigP6);
    log_info(__LINE__, __func__, "DigP7: 0x%8.8X   %7d (INT16)\r",     StructBME280->DigP7,           StructBME280->DigP7);
    log_info(__LINE__, __func__, "DigP8: 0x%8.8X   %7d (INT16)\r",    (StructBME280->DigP8 & 0xFFFF), StructBME280->DigP8);
    log_info(__LINE__, __func__, "DigP9: 0x%8.8X   %7d (INT16)\r\r\r", StructBME280->DigP9,           StructBME280->DigP9);

    log_info(__LINE__, __func__, "Humidity:\r");
    log_info(__LINE__, __func__, "DigH1: 0x%8.8X   %7u (UCHAR)\r",      StructBME280->DigH1, StructBME280->DigH1);
    log_info(__LINE__, __func__, "DigH2: 0x%8.8X   %7d (INT16)\r",      StructBME280->DigH2, StructBME280->DigH2);
    log_info(__LINE__, __func__, "DigH3: 0x%8.8X   %7u (UCHAR)\r",      StructBME280->DigH3, StructBME280->DigH3);
    log_info(__LINE__, __func__, "DigH4: 0x%8.8X   %7d (INT16)\r",      StructBME280->DigH4, StructBME280->DigH4);
    log_info(__LINE__, __func__, "DigH5: 0x%8.8X   %7d (INT16)\r",      StructBME280->DigH5, StructBME280->DigH5);
    log_info(__LINE__, __func__, "DigH6: 0x%8.8X   %7d (CHAR)\r\r\r\r", StructBME280->DigH6, StructBME280->DigH6);
  }

  return 0;
}





/* $PAGE */
/* $TITLE=bme280_get_temp() */
/* ============================================================================================================================================================= *\
                                              Read temperature, humidity and atmospheric pressure from BME280.
                                                 NOTE: Altitude and humidex values are not trusted for now.
\* ============================================================================================================================================================= */
UINT8 bme280_get_temp(struct struct_bme280 *StructBME280)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_ON;
#endif  // RELEASE_VERSION


  INT16 Dum1Int16;

  UINT16 Dum1UInt16;
  UINT8 HumParam;
  UINT8 Loop1UInt8;
  UINT8 Loop2UInt8;
  UINT8 Buffer[8];

  UINT32 ReturnCode;
  UINT32 ReturnCode1;
  UINT32 ReturnCode2;

  INT32 Dum1Int32;
  INT32 Dum2Int32;
  INT32 FinalHum;
  INT32 RawTemp;
  INT32 RawHum;
  INT32 RawPress;
  INT32 TempFactor;

  INT64 Dum1Int64;
  INT64 Dum2Int64;
  INT64 FinalPress;

  float Alpha;
  float Dum1Float;
  float DewPoint;


  /* One more temperature reading from BME280. */
  ++StructBME280->ReadCycles;

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_get_temp()\r");


  /* Request a BME280 reading and oversampling for temperature and atmospheric pressure. */
  /* Bits 1 and 0: BME280 sensor mode.  00 = sleep mode; 01, 10 = forced mode; 11 = normal mode.
     Bits 4, 3, 2: oversampling for barometric pressure data.
     Bits 7, 6, 5: oversampling for temperature data.
     Note: 0b101 = oversampling X 16; 0b100 = oversampling X 8; 0b011 = oversampling X 4;
           0b010 = oversampling X 2;  0b001 = oversampling X 1; 0b000 = skip oversampling. */
  Buffer[0] = BME280_REGISTER_CTRL_MEAS;
  Buffer[1] = 0b00100000   // temperature oversampling X 1
            + 0b00000100   // pressure oversampling X 1
            + 0b00000001;  // BME280 in "force" mode

  /* Send address of the BME280's ctrl_meas, while checking for an eventual error code. */
  Buffer[1] = 0x24; // sleep mode
  ReturnCode = i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, true);
  sleep_ms(300);
  
  Buffer[1] = 0x25; // force mode
  ReturnCode = i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, true);
  sleep_ms(300);
  
  Buffer[1] = 0x24; // sleep mode
  ReturnCode = i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, true);
  sleep_ms(300);
  
  Buffer[1] = 0x25; // force mode
  ReturnCode = i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, true);
  sleep_ms(300);
  
  if (ReturnCode == PICO_ERROR_GENERIC)
  {
    ++StructBME280->Errors;
    return 0xFF;
  }

  /* Next, read temperature, humidity and atmospheric pressure raw data.
     Give some time for the device to complete operation
     (for some reason, the "status" register doesn't seem to indicate a pending operation) . */
  bme280_read_registers(Buffer);

  /* Compute raw values. */
  RawPress = ((Buffer[0] << 12) + (Buffer[1] << 4) + ((Buffer[2] & 0xF0) >> 4));
  RawTemp  = ((Buffer[3] << 12) + (Buffer[4] << 4) + ((Buffer[5] & 0xF0) >> 4));
  RawHum   = ((Buffer[6] <<  8) +  Buffer[7]);

  if (FlagLocalDebug)
  {
    log_info(__LINE__, __func__, "RawPress: %5.5X\r",   RawPress);
    log_info(__LINE__, __func__, "RawTemp:  %5.5X\r",   RawTemp);
    log_info(__LINE__, __func__, "RawHum:   %5.4X\r\r", RawHum);
  }

  /* Compute actual temperature. */
  Dum1Int32 = ((((RawTemp >> 3)  - ((INT32)StructBME280->DigT1 << 1))) * ((INT32)StructBME280->DigT2)) >> 11;
  Dum2Int32 = (((((RawTemp >> 4) - ((INT32)StructBME280->DigT1)) * ((RawTemp >> 4) - ((INT32)StructBME280->DigT1))) >> 12) * ((INT32)StructBME280->DigT3)) >> 14;
  TempFactor = Dum1Int32 + Dum2Int32;
  StructBME280->TemperatureC = ((TempFactor * 5 + 128) >> 8) / 100.0;
  StructBME280->TemperatureF = ((StructBME280->TemperatureC * 9) / 5.0) + 32;
  if (FlagLocalDebug) log_info(__LINE__, __func__, "Temperature:     %3.3f 'C   %3.3f 'F\r", StructBME280->TemperatureC, StructBME280->TemperatureF);



  /* Compute actual relative humidity. */
  FinalHum = (TempFactor - ((INT32)76800));
  FinalHum = (((((RawHum << 14) - (((INT32)StructBME280->DigH4) << 20) - (((INT32)StructBME280->DigH5) * FinalHum)) +
                ((INT32)16384)) >> 15) *
              (((((((FinalHum * ((INT32)StructBME280->DigH6)) >> 10) *
                  (((FinalHum * ((INT32)StructBME280->DigH3)) >> 11) + ((INT32)32768))) >> 10) + ((INT32)2097152)) *
                   ((INT32)StructBME280->DigH2) + 8192) >> 14));
  FinalHum = (FinalHum - (((((FinalHum >> 15) * (FinalHum >> 15)) >> 7) * ((INT32)StructBME280->DigH1)) >> 4));
  FinalHum = (FinalHum < 0 ? 0 : FinalHum);
  FinalHum = (FinalHum > 419430400 ? 419430400 : FinalHum);
  FinalHum = (INT32)(FinalHum >> 12);
  StructBME280->Humidity = FinalHum / 1024.0;
  if (FlagLocalDebug)
  {
    log_info(__LINE__, __func__, "Humidity:        ");
    printf("%2.3f %c\r", StructBME280->Humidity, 37);  // overcome handling of percent sign in function log_info().
  }



  /* Compute actual atmospheric pressure. */
  Dum1Int64 = ((INT64)TempFactor) - 128000;
  Dum2Int64 = Dum1Int64 * Dum1Int64 * (INT64)StructBME280->DigP6;
  Dum2Int64 = Dum2Int64 + ((Dum1Int64 * (INT64)StructBME280->DigP5) << 17);
  Dum2Int64 = Dum2Int64 + (((INT64)StructBME280->DigP4) << 35);
  Dum1Int64 = ((Dum1Int64 * Dum1Int64 * (INT64)StructBME280->DigP3) >> 8) + ((Dum1Int64 * (INT64)StructBME280->DigP2) << 12);
  Dum1Int64 = (((((INT64)1) << 47) + Dum1Int64)) * ((INT64)StructBME280->DigP1) >> 33;
  if (Dum1Int64 == 0) return 0;  // to prevent a division by zero.
  
  FinalPress = 1048576 - RawPress;
  FinalPress = (((FinalPress << 31) - Dum2Int64) * 3125) / Dum1Int64;
  Dum1Int64 = (((INT64)StructBME280->DigP9) * (FinalPress  >> 13) * (FinalPress >> 13)) >> 25;
  Dum2Int64 = (((INT64)StructBME280->DigP8) *  FinalPress) >> 19;
  FinalPress = (UINT32)((FinalPress + Dum1Int64 + Dum2Int64) >> 8) + (((INT64)StructBME280->DigP7) << 4);
  StructBME280->Pressure = FinalPress / 25600.0;
  if (FlagLocalDebug) log_info(__LINE__, __func__, "Pressure:       %4.3f hPa\r\r", StructBME280->Pressure);


  /* NOTE: Relative altitude may not be an interesting value to calculate since this value changes with normal and usual pressure changes due to atmospheric factors. */
  /*** Compute approximate altitude above sea level, based on atmospheric pressure and current temperature.
       NOTE: 1013.25 hPa is the standard pressure at sea level. ***
  StructBME280->Altitude = 44330.0 * (1.0 - pow(StructBME280->Pressure / 1013.25, 0.190294957));

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Altitude:       %4.3f - seems to vary too much with pressure change over time\r\r\r", StructBME280->Altitude);
  ***/

  /* NOTE: Equation below to calculate the humidex factor is wrong. To be updated... */
  /* Compute humidex factor, based on current temperature, relative humidity and atmospheric pressure. */
  /* Humidex is not valid if temperature is below 0'C. */
  /*** Part requiring rework... ***
  if (StructBME280->TemperatureC < 0) StructBME280->Humidex = 0;

  Alpha = ((17.27 * StructBME280->TemperatureC) / (237.7 + StructBME280->TemperatureC)) + log(StructBME280->Humidity / 100);
  DewPoint = (237.7 * Alpha) / (17.27 - Alpha);

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Dew point:      %4.3f - need to be confirmed\r\r\r", DewPoint);

  StructBME280->Humidex = (TempC + (0.5555 * (StructBME280->Pressure - 10.0)));

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Humidex:        %4.3f - current equation is incorrect\r\r\r\r\r", StructBME280->Humidex);
  ***/

  return 0;
}





/* $PAGE */
/* $TITLE=bme280_init() */
/* ============================================================================================================================================================= *\
                                                                 Initialize the BME280 sensor.
\* ============================================================================================================================================================= */
UINT8 bme280_init(void)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;
#endif  // RELEASE_VERSION

  UINT8 Buffer[2];

  UINT32 ReturnCode;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_init()\r");


  /* Initialize GPIOs used for BME280. */
  i2c_init(I2C_PORT, 400000);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_set_function(SCL, GPIO_FUNC_I2C);
	gpio_pull_up(SDA);
	gpio_pull_up(SCL);


  /* ---------------------- Reset the BME280. ----------------------- */
  Buffer[0] = BME280_REGISTER_RESET;
  Buffer[1] = 0xB6; // reset command.

  /* Send the soft reset command to the BME280's while checking for an eventual error code. */
  if (i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, true) == PICO_ERROR_GENERIC)
  {
    log_info(__LINE__, __func__, "Error %d while trying to communicate with BME280 through I2C protocol.\r");
    return 0xFF;
  }

  /* --------- Configure oversampling for humidity reading. --------- */
  /* Note: 0b101 = oversampling X 16; 0b100 = oversampling X 8; 0b011 = oversampling X 4;
           0b010 = oversampling X 2;  0b001 = oversampling X 1; 0b000 = skip oversampling. */
  Buffer[0] = BME280_REGISTER_CTRL_HUM;
  Buffer[1] = 0b00000001;

  /* Send the humidity oversampling configuration to the BME280's while checking for an eventual error code. */
  if (i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, true) == PICO_ERROR_GENERIC)
    return 0xFF;

  /* ----------------- Configure BME280 sensor mode ----------------- */
  /* --- and oversampling for temperature and atmospheric pressure --- */
  /* Bits 1 and 0: BME280 sensor mode.  00 = sleep mode; 01, 10 = forced mode; 11 = normal mode.
     Bits 4, 3, 2: oversampling for barometric pressure data.
     Bits 7, 6, 5: oversampling for temperature data.
     Note: 0b101 = oversampling X 16; 0b100 = oversampling X 8; 0b011 = oversampling X 4;
           0b010 = oversampling X 2;  0b001 = oversampling X 1; 0b000 = skip oversampling. */
  Buffer[0] = BME280_REGISTER_CTRL_MEAS;
  Buffer[1] = 0b00100000 + 0b00000100 + 0x00000000;

  /* Send temperature and pressure oversampling configuration to the BME280's, along with "sleep mode". */
  if (i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, true) == PICO_ERROR_GENERIC)
    return 0xFF;

  /* ---------------------- Set configuration ---------------------- */
  /* Bit 0: 0 for I2C protocol; 1 for SPI protocol.
     Bit 1: not used
     Bits 2, 3, 4: Time constant of IIR filter (010 = filter coefficient: 4).
     Bits 5, 6, 7: Determine inactive time duration in "normal" mode. (100 = 500 msec). */
  Buffer[0] = BME280_REGISTER_CONFIG;
  Buffer[1] = 0b10001000;

  /* Send configuration parameters to the BME280's config register. */
  if (i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 2, false) == PICO_ERROR_GENERIC)
    return 0xFF;

  return 0;
}





/* $PAGE */
/* $TITLE=bme280_read_all_registers() */
/* ============================================================================================================================================================= *\
                     Read all BME280 readable registers except calibration data and device ID, which are supported by other specific functions.
\* ============================================================================================================================================================= */
UINT8 bme280_read_all_registers()
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_ON;
#endif  // RELEASE_VERSION


  UCHAR Buffer[13];

  UINT8 Loop1UInt8;

  UINT32 ReturnCode;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_read_all_registers()\r");


  /* Give adress of the first BME280 register to read (lowest address). */
  Buffer[0] = BME280_REGISTER_CTRL_HUM;

  i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 1, true);
  ReturnCode = i2c_read_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 13, false);
  if (ReturnCode == PICO_ERROR_GENERIC)
    return 0xFF;

    if (FlagLocalDebug)
  {
    /* Validate return code, to make sure register read was successful. */
    log_info(__LINE__, __func__, "Read(%u)\r\r", ReturnCode);

    /* Display ctrl_hum register content. */
    log_info(__LINE__, __func__, "ctrl_hum:   %2.2X\r", Buffer[0]);

    /* Display status register content. */
    log_info(__LINE__, __func__, "status:     %2.2X\r", Buffer[1]);

    /* Display ctrl_meas register content. */
    log_info(__LINE__, __func__, "ctrl_meas:  %2.2X\r", Buffer[2]);

    /* Display config register content. */
    log_info(__LINE__, __func__, "config:     %2.2X\r", Buffer[3]);

    /* Display unknown register content. */
    log_info(__LINE__, __func__, "unknown:    %2.2X\r", Buffer[4]);

    /* Display press_msb register content. */
    log_info(__LINE__, __func__, "press_msb:  %2.2X\r", Buffer[5]);

    /* Display press_lsb register content. */
    log_info(__LINE__, __func__, "press_lsb:  %2.2X\r", Buffer[6]);

    /* Display press_xlsb register content. */
    log_info(__LINE__, __func__, "press_xlsb: %2.2X\r", Buffer[7]);

    /* Display temp_msb register content. */
    log_info(__LINE__, __func__, "temp_msb:   %2.2X\r", Buffer[8]);
  
    /* Display temp_lsb register content. */
    log_info(__LINE__, __func__, "temp_lsb:   %2.2X\r", Buffer[9]);

    /* Display temp_xlsb register content. */
    log_info(__LINE__, __func__, "temp_xlsb:  %2.2X\r", Buffer[10]);

    /* Display hum_msb register content. */
    log_info(__LINE__, __func__, "hum_msb:    %2.2X\r", Buffer[11]);

    /* Display hum_lsb register content. */
    log_info(__LINE__, __func__, "hum_lsb:    %2.2X\r\r\r\r\r", Buffer[12]);
  }

  return 0;
}





/* $PAGE */
/* $TITLE=bme280_read_calib_data() */
/* ============================================================================================================================================================= *\
       Read BME280 calibration data for the specific device used.
\* ============================================================================================================================================================= */
UINT8 bme280_read_calib_data(struct struct_bme280 *StructBME280)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_ON;
#endif  // RELEASE_VERSION


  UCHAR Buffer;

  UINT8 Loop1UInt8;

  UINT32 ReturnCode;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_read_calib_data()\r");


  /* Read first chunk of BME280 calibration data. */
  Buffer = BME280_REGISTER_CALIB00;

  i2c_write_timeout_us(I2C_PORT, BME280_ADDRESS, &Buffer, 1, true, 10000000);
  ReturnCode = i2c_read_timeout_us(I2C_PORT, BME280_ADDRESS, StructBME280->CalibData, 26, false, 10000000);

  /* Read second chunk of BME280 calibration data. */
  Buffer = BME280_REGISTER_CALIB26;

  i2c_write_timeout_us(I2C_PORT, BME280_ADDRESS, &Buffer, 1, true, 10000000);
  ReturnCode = i2c_read_timeout_us(I2C_PORT, BME280_ADDRESS, &StructBME280->CalibData[26], 16, false, 10000000);

  if (FlagLocalDebug)
  {
    /* Send calibration data to log file. */
    for (Loop1UInt8 = 0; Loop1UInt8 < 26; ++Loop1UInt8)
      log_info(__LINE__, __func__, "Calib[%2.2u]  Address: 0x%2.2X = %2.2X\r", Loop1UInt8, (Loop1UInt8 + 0x88), StructBME280->CalibData[Loop1UInt8]);

    for (Loop1UInt8 = 0; Loop1UInt8 < 16; ++Loop1UInt8)
      log_info(__LINE__, __func__, "Calib[%2.2u]  Address: 0x%2.2X = %2.2X\r", (Loop1UInt8 + 26), (0xE1 + Loop1UInt8), StructBME280->CalibData[Loop1UInt8 + 26]);

    printf("\r\r");
  }

  return 0;
}





/* $PAGE */
/* $TITLE=bme280_read_config() */
/* ============================================================================================================================================================= *\
                                                                   Read BME280 configuration register.
\* ============================================================================================================================================================= */
UINT8 bme280_read_config(void)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_ON;
#endif  // RELEASE_VERSION


  UINT8 Buffer;

  UINT32 ReturnCode;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_read_config()\r");


  Buffer = BME280_REGISTER_CONFIG;

  i2c_write_blocking(I2C_PORT, BME280_ADDRESS, &Buffer, 1, true);
  i2c_read_blocking(I2C_PORT, BME280_ADDRESS, &Buffer, 1, false);

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Config: 0x%2.2X\r\r", Buffer);

  return Buffer;
}





/* $PAGE */
/* $TITLE=bme280_read_device_id() */
/* ============================================================================================================================================================= *\
                                                                         Read BME280 device ID.
                       Should be 0x60 for a true Bosch BME280, or 0x56 and 0x57 for BMP280 samples and 0x58 for BMP280 mass production units.
\* ============================================================================================================================================================= */
UINT8 bme280_read_device_id(void)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;
#endif  // RELEASE_VERSION


  UINT8 Buffer;

  INT32 ReturnCode;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_read_device_id()\r");

  Buffer = BME280_REGISTER_ID;

  i2c_write_timeout_us(I2C_PORT, BME280_ADDRESS, &Buffer, 1, true, 10000000);
  ReturnCode = i2c_read_timeout_us(I2C_PORT, BME280_ADDRESS, &Buffer, 1, false, 10000000);

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Return code: %d     Device ID: 0x%2.2X\r\r\r", ReturnCode, Buffer);

  return Buffer;
}





/* $PAGE */
/* $TITLE=bme280_read_registers() */
/* ============================================================================================================================================================= *\
                                         Read BME280 registers containing temperature, humidity and atmospheric pressure.
\* ============================================================================================================================================================= */
UINT8 bme280_read_registers(UINT8 *Buffer)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_ON;
#endif  // RELEASE_VERSION


  UINT8 Loop1UInt8;

  UINT32 ReturnCode;


  /* Give address of the first BME280 register to read (lowest address). */
  Buffer[0] = BME280_REGISTER_PRESS_MSB;

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_read_registers(%2.2X)\r", Buffer[0]);

  Loop1UInt8 = 0;
  while (bme280_read_status() != 0)
    ++Loop1UInt8;

  i2c_write_blocking(I2C_PORT, BME280_ADDRESS, Buffer, 1, true);
  i2c_read_blocking( I2C_PORT, BME280_ADDRESS, Buffer, 8, false);

  if (FlagLocalDebug)
  {
    /* Display registers read from BME280. */
    log_info(__LINE__, __func__, "Registers:  (wait: %4u)     ", Loop1UInt8);
    for (Loop1UInt8 = 0; Loop1UInt8 < 8; ++Loop1UInt8)
    {
      printf("%2.2X ", Buffer[Loop1UInt8]);
      if (Loop1UInt8 != (8 - 1)) printf("- ");  // display separator.
    }
    printf("\r\r");
  }

  return 0;
}





/* $PAGE */
/* $TITLE=bme280_read_status() */
/* ============================================================================================================================================================= *\
                                                                        Read BME280 current status.
                                                              (Should be 0x00 when no operation is pending).
\* ============================================================================================================================================================= */
UINT8 bme280_read_status(void)
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;
#endif  // RELEASE_VERSION


  UINT8 Buffer;

  UINT32 ReturnCode;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_read_status()\r");


  Buffer = BME280_REGISTER_STATUS;

  i2c_write_blocking(I2C_PORT, BME280_ADDRESS, &Buffer, 1, true);
  i2c_read_blocking(I2C_PORT, BME280_ADDRESS, &Buffer, 1, false);

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Status: 0x%2.2X\r\r", Buffer);

  /* Bits 0 and 3 indicate that bme280 operation is on-going. */
  return (Buffer & 0x9);
}





/* $PAGE */
/* $TITLE=bme280_read_unique_id() */
/* ============================================================================================================================================================= *\
                                                    Read BME280 unique id ("serial number" of the specific device used.
\* ============================================================================================================================================================= */
UINT32 bme280_read_unique_id()
{
#ifdef RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;  // should be OFF all times
#else  // RELEASE_VERSION
  UINT8 FlagLocalDebug = FLAG_OFF;
#endif  // RELEASE_VERSION


  UINT8 Buffer[4];

  UINT32 ReturnCode;
  UINT32 UniqueId;


  if (FlagLocalDebug) log_info(__LINE__, __func__, "Entering bme280_read_unique_id()\r");


  /* Give adress of the first BME280 register to read (lowest address). */
  Buffer[0] = BME280_REGISTER_UNIQUE_ID;

  i2c_write_timeout_us(I2C_PORT, BME280_ADDRESS, Buffer, 1, true, 10000000);
  ReturnCode = i2c_read_timeout_us(I2C_PORT, BME280_ADDRESS, Buffer, 4, false, 10000000);
  if (ReturnCode == PICO_ERROR_GENERIC) return 0xFF;

  UniqueId = ((((UINT32)Buffer[3] + ((UINT32)Buffer[2] << 8)) & 0x7FFFF) << 16) +
             (((UINT32)Buffer[1]) << 8) + (UINT32)Buffer[0];

  if (FlagLocalDebug) log_info(__LINE__, __func__, "Return code: %d    Hex: %4.4X-%4.4X\r", ReturnCode, ((UniqueId & 0xFFFF0000) >> 16), UniqueId & 0xFFFF);

  return UniqueId;
}
