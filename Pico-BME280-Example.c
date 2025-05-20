/* ============================================================================================================================================================= *\
   Pico-BME280-Example.c
   St-Louys Andre - May 2025
   astlouys@gmail.com
   Revision 19-MAI-2025
   Langage: C
   Version 1.00

   Raspberry Pi Pico example on how to use the Pico-BME280-Module to integrate a BME280 sensor to a C-Language program.

   NOTE:
   THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
   WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
   TIME. AS A RESULT, THE AUTHOR SHALL NOT BE HELD LIABLE FOR ANY DIRECT, 
   INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
   THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
   INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCT.


   REVISION HISTORY:
   =================
   19-MAY-2025 1.00 - Initial release.
\* ============================================================================================================================================================= */



/* $PAGE */
/* $TITLE=Definitions and macros. */
/* ============================================================================================================================================================= *\
                                                                       Definitions and macros.
\* ============================================================================================================================================================= */
/* Firmware version. */
#define FIRMWARE_VERSION "1.00"  ///




/* $PAGE */
/* $TITLE=Include files. */
/* ============================================================================================================================================================= *\
                                                                          Include files
\* ============================================================================================================================================================= */
#include "baseline.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "Pico-BME280-Module.h"



/* $PAGE */
/* $TITLE=Global variables declaration / definition. */
/* ============================================================================================================================================================= *\
                                                            Global variables declaration / definition.
\* ============================================================================================================================================================= */



/* $PAGE */
/* $TITLE=Function prototypes. */
/* ============================================================================================================================================================= *\
                                                                     Function prototypes.
\* ============================================================================================================================================================= */
/* Retrieve Pico's Unique ID from the flash IC. */
void get_pico_unique_id(UCHAR *UniqueId);

/* Read a string from stdin. */
void input_string(UCHAR *String);

/* Send a string to log file. */
void log_info(UINT LineNumber, const UCHAR *FunctionName, UCHAR *Format, ...);

/* Convert a string to lowercase. */
UCHAR *util_to_lower(UCHAR *String);





/* $PAGE */
/* $TITLE=Main program entry point. */
/* ============================================================================================================================================================= *\
                                                                      Main program entry point.
\* ============================================================================================================================================================= */
UINT main()
{
  UCHAR String[32];
  UCHAR UniqueId[25];

  UINT8 Dum1UInt8;

  UINT16 Delay;

  struct struct_bme280 StructBME280;


  /* Initialize GPIOs. */
  stdio_init_all();


  /* --------------------------------------------------------------------------------------------------------------------------- *\
                                                    Wait for USB CDC connection.
                            System will give up and abort the Firmware after a 2 minutes waiting time.
  \* --------------------------------------------------------------------------------------------------------------------------- */
  /* Give some time to start a terminal emulation program. */
  Delay = 0;
  while (stdio_usb_connected() == 0)
  {
    ++Delay;
    sleep_ms(50);  // 50 msec added to current wait time for a USB CDC connection.

    /* If we waited for more than this number of "50 msec" for a USB CDC connection, get out of the loop and abort the Firmware. */
    if (Delay > 2400) break;
  }


  /* Check if USB CDC connection has been detected.*/
  if (stdio_usb_connected())
    log_info(__LINE__, __func__, "USB CDC connection has been established.\r");
  else
    exit(1);  // abort Firmware.


  get_pico_unique_id(UniqueId);

  log_info(__LINE__, __func__, "==============================================================================================================\r");
  log_info(__LINE__, __func__, "                                             Pico-BME280-Example\r");
  log_info(__LINE__, __func__, "                                    Part of the ASTL Smart Home ecosystem.\r");
  log_info(__LINE__, __func__, "                                    Pico unique ID: <%s>.\r", UniqueId);
  log_info(__LINE__, __func__, "==============================================================================================================\r");
  log_info(__LINE__, __func__, "Main program entry point (Delay: %u msec waiting for USB CDC connection).\r", (Delay * 50));



  /* Initialize BME280 sensor. */
  if (bme280_init())
  {
    log_info(__LINE__, __func__, "BME280 initialization error... Aborting Firmware...\r\r\r");
    sleep_ms(200);
    exit(1);
  }



  /* Read BME280 device ID to make sure it is a "real" BME280, since some companies sell BMP280 while saying they are BME280. */
  /* BME280 device ID is 0x60, while 0x56 and 0x57 are BMP280 samples and 0x58 are BMP280 mass production. */
  StructBME280.DeviceId = bme280_read_device_id();
  switch (StructBME280.DeviceId)
  {
    case 0x60:
      log_info(__LINE__, __func__, "BME280 device ID: 0x60\r");
      log_info(__LINE__, __func__, "Your device seems to be a 'real' BME280.\r");
      break;

    case 0x56:
    case 0x57:
      log_info(__LINE__, __func__, "BMP280 device ID: 0x%2.2X (sample production units)\r", StructBME280.DeviceId);
      log_info(__LINE__, __func__, "Your device seems not to be a true BME280\r");
    break;

    case 0x58:
      log_info(__LINE__, __func__, "BMP280 device ID: 0x58 (mass production units)\r");
      log_info(__LINE__, __func__, "Your device seems not to be a true BME280\r");
    break;

    default:
      log_info(__LINE__, __func__, "Unrecognized BME280 device ID: 0x%2.2X\r", StructBME280.DeviceId);
      log_info(__LINE__, __func__, "Your device seems not to be a true BME280\r");
    break;
  }

  /* Read BME280 unique id ("serial number" of the specific device used). */
  StructBME280.UniqueId = bme280_read_unique_id();
  log_info(__LINE__, __func__, "BME280 Unique ID: %4.4X-%4.4X\r", ((StructBME280.UniqueId & 0xFFFF0000) >> 16), StructBME280.UniqueId & 0xFFFF);

  /* Read calibration data of the specific device used (written in BME280's non volatile memory). */
  bme280_read_calib_data(&StructBME280);

  /* Compute calibration parameters from calibration data. */
  bme280_compute_calib_param(&StructBME280);

  /* Read current temperature conditions. */
  while (1)
  {
    bme280_get_temp(&StructBME280);

    log_info(__LINE__, __func__, "Temperature:     %3.3f 'C   %3.3f 'F\r", StructBME280.TemperatureC, StructBME280.TemperatureF);
    log_info(__LINE__, __func__, "Humidity:        ");
    printf("%2.3f %c\r", StructBME280.Humidity, 37);  // overcome handling of percent sign in function log_info().
    log_info(__LINE__, __func__, "Pressure:       %4.3f hPa\r\r", StructBME280.Pressure);

    log_info(__LINE__, __func__, "Press <Enter> to proceed with another BME280 read cycle:\r");
    log_info(__LINE__, __func__, "or <ESC> to switch the Pico in upload mode: ");
    input_string(String);
    if (String[0] == 0x1B)
    {
      reset_usb_boot(0l, 0l);
      sleep_ms(200);
    }
  }
}





/* $PAGE */
/* $TITLE=get_pico_unique_id() */
/* ============================================================================================================================================================= *\
                                                           Retrieve Pico's Unique ID from the flash IC.
\* ============================================================================================================================================================= */
void get_pico_unique_id(UCHAR *UniqueId)
{
  UINT8 Loop1UInt8;

  pico_unique_board_id_t board_id;


  /* Retrieve Pico Unique ID from its flash memory IC. */
  pico_get_unique_board_id(&board_id);

  /* Build the Unique ID string in hex. */
  UniqueId[0] = 0x00;  // initialize as null string on entry.
  for (Loop1UInt8 = 0; Loop1UInt8 < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++Loop1UInt8)
  {
    sprintf(&UniqueId[strlen(UniqueId)], "%2.2X", board_id.id[Loop1UInt8]);
    if ((Loop1UInt8 % 2) && (Loop1UInt8 != 7)) sprintf(&UniqueId[strlen(UniqueId)], "-");
  }

  return;
}





/* $PAGE */
/* $TITLE=input_string() */
/* ============================================================================================================================================================= *\
                                                                    Read a string from stdin.
\* ============================================================================================================================================================= */
void input_string(UCHAR *String)
{
  INT8 DataInput;

  UINT8 FlagLocalDebug = FLAG_OFF;
  UINT8 Loop1UInt8;

  UINT32 IdleTimer;


  if (FlagLocalDebug) printf("Entering input_string().\r");

  Loop1UInt8 = 0;
  IdleTimer  = time_us_32();  // initialize time-out timer with current system timer.
  do
  {
    DataInput = getchar_timeout_us(50000);

    switch (DataInput)
    {
      case (PICO_ERROR_TIMEOUT):
      case (0):
#if 0
        /* This code block if we want input_string() to return after a timeout wait time. */ 
        if ((time_us_32() - IdleTimer) > 300000000l)
        {
          printf("[%5u] - Input timeout %lu - %lu = %lu!!\r\r\r", __LINE__, time_us_32(), IdleTimer, time_us_32() - IdleTimer);
          String[0]  = 0x1B;  // time-out waiting for a keystroke.
          Loop1UInt8 = 1;     // end-of-string will be added when exiting while loop.
          DataInput  = 0x0D;
        }
#endif  // 0
        continue;
      break;

      case (8):
        /* <Backspace> */
        IdleTimer = time_us_32();  // restart time-out timer.
        if (Loop1UInt8 > 0)
        {
          --Loop1UInt8;
          String[Loop1UInt8] = 0x00;
          printf("%c %c", 0x08, 0x08);  // erase character under the cursor.
        }
      break;

      case (27):
        /* <ESC> */
        IdleTimer = time_us_32();  // restart time-out timer.
        if (Loop1UInt8 == 0)
        {
          String[Loop1UInt8++] = (UCHAR)DataInput;
          String[Loop1UInt8++] = 0x00;
        }
        printf("\r");
      break;

      case (0x0D):
        /* <Enter> */
        IdleTimer = time_us_32();  // restart time-out timer.
        if (Loop1UInt8 == 0)
        {
          String[Loop1UInt8++] = (UCHAR)DataInput;
          String[Loop1UInt8++] = 0x00;
        }
        printf("\r");
      break;

      default:
        IdleTimer = time_us_32();  // restart time-out timer.
        printf("%c", (UCHAR)DataInput);
        String[Loop1UInt8] = (UCHAR)DataInput;
        // printf("Loop1UInt8: %3u   %2.2X - %c\r", Loop1UInt8, DataInput, DataInput);  /// for debugging purposes.
        ++Loop1UInt8;
      break;
    }
    sleep_ms(10);
  } while((Loop1UInt8 < 128) && (DataInput != 0x0D));

  String[Loop1UInt8] = '\0';  // end-of-string
  /// printf("\r\r\r");

  /* Optionally display each character entered. */
  /***
  for (Loop1UInt8 = 0; Loop1UInt8 < 10; ++Loop1UInt8)
    printf("%2u:[%2.2X]   ", Loop1UInt8, String[Loop1UInt8]);
  printf("\r");
  ***/

  if (FlagLocalDebug) printf("Exiting input_string().\r");

  return;
}





/* $PAGE */
/* $TITLE=log_info() */
/* ============================================================================================================================================================= *\
                                                                      Send a string to log file.
\* ============================================================================================================================================================= */
void log_info(UINT LineNumber, const UCHAR *FunctionName, UCHAR *Format, ...)
{
  UCHAR Dum1Str[512];
  UCHAR Dum2Str[512];
  UCHAR TimeStamp[128];

  UINT FunctionSize = 25;  // specify space reserved to display function name including the two "[]".
  UINT Loop1UInt;
  UINT StartChar;

  va_list argp;


  /* If there is no terminal connected, bypass the display. */
  if (!stdio_usb_connected()) return;


  /* Transfer the text to print to variable Dum1Str. */
  va_start(argp, Format);
  vsnprintf(Dum1Str, sizeof(Dum1Str), Format, argp);
  va_end(argp);
  strcpy(Dum2Str, Dum1Str);  // make a copy to a working variable (Dum2Str).

  /// printf("[%5u] - <%s> <%s>\r", __LINE__, Dum1Str, Dum2Str);  /// debug



  /* ----------------------------------------------------------------------------------------------------------------------- *\
                                              Handling of <HOME> special control code.
  \* ----------------------------------------------------------------------------------------------------------------------- */
  /* Trap special control code for <HOME>. Replace "home" by appropriate control characters for "Home" on a VT101. */
  if (strcmp(util_to_lower(Dum2Str), "home") == 0)
  {
    Dum1Str[0] = 0x1B; // ESC code
    Dum1Str[1] = '[';
    Dum1Str[2] = 'H';
    Dum1Str[3] = 0x00;
  }



  /* ----------------------------------------------------------------------------------------------------------------------- *\
                                              Handling of <CLS> special control code.
  \* ----------------------------------------------------------------------------------------------------------------------- */
  /* Trap special control code for <CLS>. Replace "cls" by appropriate control characters for "Clear screen" on a VT101. */
  if (strcmp(util_to_lower(Dum2Str), "cls") == 0)  // has been converted to lowercase above.
  {
    Dum1Str[0] = 0x1B; // ESC code
    Dum1Str[1] = '[';
    Dum1Str[2] = '2';
    Dum1Str[3] = 'J';
    Dum1Str[4] = 0x00;
  }



  /* ----------------------------------------------------------------------------------------------------------------------- *\
                                  Displaying source code line number and caller Pico's core number
                        followed by time stamp is communication with real-time clock IC has been initialized
       then caller's function name (will be truncated if longer than the size specified at the beginning of this function)
                                 finally, the text to be sent to log file as specified in the call.  
  \* ----------------------------------------------------------------------------------------------------------------------- */
  /* Line header will not be printed if first character is a '-',
     or if first character is a line feed '\r' when we simply want to do line spacing in the debug log,
     or if first character is the beginning of a control stream (for example 'home' or 'cls'). */
  
  /* If first character is a line feed ('\r'), display the text as is. */
  if (Dum2Str[0] == '\r') printf(Dum1Str);

  /// if ((Dum2Str[0] != '-') && (Dum2Str[0] != '\r') && (Dum2Str[0] != 0x1B) && (Dum2Str[0] != '|'))
  /// {
    /* Print line number and caller Pico's core number. */
    /// printf("[%5u] - <%s> <%s>\r", __LINE__, Dum1Str, Dum2Str);  /// debug
    printf("[%5u %u] ", LineNumber, get_core_num());  // if program is longer than 99999 lines of code, replace [%5u] by [%6u]

#if 0
  /* ----------------------------------------------------------------------------------------------------------------------- *\
                                     If the environment allows it, display current time stamp.
  \* ----------------------------------------------------------------------------------------------------------------------- */
    if (FlagDS3231Init)
    {
      /* Retrieve current time stamp. */
      ds3231_get_time(&CurrentTime);

      /* Print time stamp. */
      printf("[%2.2d-%s-%2.2d  %2.2d:%2.2d:%2.2d] ", CurrentTime.DayOfMonth, ShortMonth[CurrentTime.Month], (CurrentTime.Year % 1000), CurrentTime.Hour, CurrentTime.Minute, CurrentTime.Second);
    }
    else
    {
      printf("                      ");  // to properly align function name and log text.
    }
#endif  // 0

    /* Print function name and align all function names in log file (except if the name is too long to fit in the size given). */
    sprintf(Dum2Str, "[%s]", FunctionName);

    /* Check if function name is too long for a clean format in the lrteiog. */
    if (strlen(Dum2Str) > FunctionSize)
    {
      /// printf("Dum2Str > 27: <%s>\r", Dum2Str);
      Dum2Str[FunctionSize - 3] = '~';   // mark indicating function name has been truncated.
      Dum2Str[FunctionSize - 2] = ']';   // truncate function name length when it is too long.
      Dum2Str[FunctionSize - 1] = 0x00;  // end-of-string.
      /***
      printf("Loop %u = ");
      for (Loop1UInt = 0; Loop1UInt < 20; ++Loop1UInt)
        printf("%2u   ", Dum2Str[Loop1UInt]);
      ***/
    }
    printf("%s", Dum2Str);  // display caller's function name.

    /* Pad function name with blanks when it is shorter than specified length. */
    for (Loop1UInt = strlen(FunctionName); Loop1UInt < FunctionSize; ++Loop1UInt)
      printf(" ");

    printf("- ");
  /// }

  /* Display data to log file. */
  // uart_write_blocking(uart0, (UINT8 *)LineString, strlen(LineString));
  printf(Dum1Str);

  return;
}





/* $PAGE */
/* $TITLE=util_to_lower() */
/* ============================================================================================================================================================= *\
                                                              Convert a string to lowercase.
\* ============================================================================================================================================================= */
UCHAR *util_to_lower(UCHAR *String)
{
  UINT8 CharPointer;


  CharPointer = 0;
  while (String[CharPointer])
  {
    if ((String[CharPointer] >= 0x41) && (String[CharPointer] <= 0x5A))
      String[CharPointer] |= 0x20;

    ++CharPointer;
  }

  return String;
}





