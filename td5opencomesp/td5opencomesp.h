#pragma once

#define TD5_OPEN_COM_VERSION_MAJOR 0
#define TD5_OPEN_COM_VERSION_MINOR 1
#define TD5_OPEN_COM_VERSION_PATCH 12

//#define HOME_WIFI_SSID      "Vodafone-C58594756"
//#define HOME_WIFI_PASSWORD  "ANaRnCPnRygn22cp"

// OBD
#define K_OUT 20
#define K_IN 19

// SD card
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define SD_CMD_MOSI 11    // SD card master output slave input pin
#define SD_CLK  12    // SD card clock pin
#define SD_D0_MISO 13    // SD card master input slave output pin
#define SD_SS   15    // SD card select pin (use "RS485 RX" to avoid flickering)

//#define HOME_WIFI_SSID      "Vodafone-C58594756"
//#define HOME_WIFI_PASSWORD  "ANaRnCPnRygn22cp"

// Events loop 
#include "esp_event.h"
extern esp_event_loop_handle_t  event_loop;

// RTC
#include "src/rtc/ds3231.h"
#include <ESP32Time.h>
typedef i2c_dev_t DS3231_DEV; // I2C device struct
extern DS3231_DEV rtc_ext;		
extern ESP32Time rtc;
void update_rtc_ext(byte sec, byte min, byte hour, byte day, byte month, int year);


// Preferences
#include <Preferences.h>
#define RW_MODE false
extern Preferences td5Prefs;

// Objects positions
#define UI_TABLE_ECU_FAULT_CODES_POS_X 120
#define UI_TABLE_ECU_FAULT_CODES_POS_Y 0
#define UI_TABLE_ECU_FAULT_CODES_WIDTH 470
#define UI_TABLE_ECU_FAULT_CODES_HEIGHT 320

// Fonts
#define UI_TABLE_ECU_FAULT_CODES_FONT lv_font_montserrat_20
