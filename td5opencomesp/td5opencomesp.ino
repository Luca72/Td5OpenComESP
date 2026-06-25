#pragma GCC push_options
#pragma GCC optimize("O3")

#include <Arduino.h>
#include "td5opencomesp.h"

//#include <WiFi.h>
//const char* ssid = HOME_WIFI_SSID;
//const char* password = HOME_WIFI_PASSWORD;
//#include <Wire.h>

#undef ESP_UTILS_LOG_TAG
#define ESP_UTILS_LOG_TAG "Main"
#include "esp_lib_utils.h"

#include "td5comm.h"
#include "td5commhandler.h"
#include "uieventshandler.h"

#include "lvgl.h"
#include "lvgl_v8_port.h"
#include <esp_display_panel.hpp>
#include "esp_panel_board_custom_conf.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

#define USE_UI
#ifdef USE_UI
  #include "src/ui/ui.h"
  #include "uipostinit.h"
#endif

// Event loop
esp_event_loop_handle_t event_loop;
/* Event source task related definitions */
ESP_EVENT_DEFINE_BASE(COMM_EVENTS);
ESP_EVENT_DEFINE_BASE(UI_EVENTS);

Td5Comm *td5;
Preferences td5Prefs;

//#include <RTClib.h>
//RTC_DS3231  rtc;
DS3231_DEV rtc_ext;
ESP32Time rtc;
void update_rtc_ext(byte sec, byte min, byte hour, byte day, byte month, int year) {
  tm datetime = {
    .tm_sec = sec,
    .tm_min = min,
    .tm_hour = hour,
    .tm_mday = day,
    .tm_mon = month+1,
    .tm_year = year,
  };
  if (!ds3231_set_time(&rtc_ext, &datetime) != ESP_OK) {
    ESP_UTILS_LOGI("RTC Could not update time.");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  ESP_UTILS_LOGI("ESP32S3 LVGL");
  ESP_UTILS_LOGI("Td5OpenCom ESP32 v.%d.%d.%d", TD5_OPEN_COM_VERSION_MAJOR, TD5_OPEN_COM_VERSION_MINOR, TD5_OPEN_COM_VERSION_PATCH);

  // Initialize and retrieve Preferences
  ESP_UTILS_LOGI("Initialize Preferences");
  td5Prefs.begin("Preferences", RW_MODE); // Open our namespace (or create it if it doesn't exist)
  bool prefsInit = td5Prefs.isKey("startConnected"); // Test for the existence of the "already initialized" key.
  if (prefsInit == false) {
    ESP_UTILS_LOGI("First time using it: setup standard preferences.");
    // If prefsInit is 'false', the key "nvsInit" does not yet exist therefore this
    //  must be our first-time run. We need to set up our Preferences namespace keys. So...

    //  first-time run we will create
    //  our keys and store the initial "factory default" values.
    td5Prefs.putBool("startConnected", false);
    td5Prefs.putBool("instrModeGauge", false);
  }    

  // Connect to WiFi
  /*
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  */

  // Init PSRAM
  if(psramInit()) {
    ESP_UTILS_LOGI("PSRAM is correctly initialized");
  }else {
    ESP_UTILS_LOGI("PSRAM not available");
  }

  // Initialize OBD comunication
  ESP_UTILS_LOGI("Create Td5 object");
  td5 = new Td5Comm;
  td5->init();

  // Create the event loops
  ESP_UTILS_LOGI("Create UI events handler");
  esp_event_loop_args_t event_loop_args = {
    .queue_size = 20,
    .task_name = "event_loop_task", // task will be created
    .task_priority = uxTaskPriorityGet(NULL),
    .task_stack_size = 3072,
    .task_core_id = tskNO_AFFINITY
  };
  ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &event_loop));

  // Initialize I2C (Wire)
  /*
  ESP_UTILS_LOGI("Initialize I2C");
  Wire.begin(ESP_PANEL_BOARD_EXPANDER_I2C_IO_SDA, ESP_PANEL_BOARD_EXPANDER_I2C_IO_SCL); // Initialize the I2C device and set SDA to 8 and SCL to 9
  */

  // Initialize I2C and RTC
  ESP_UTILS_LOGI("RTC Initialize");
  if (ds3231_init_desc(&rtc_ext, I2C_NUM_0, static_cast<gpio_num_t>(ESP_PANEL_BOARD_TOUCH_I2C_IO_SDA), static_cast<gpio_num_t>(ESP_PANEL_BOARD_TOUCH_I2C_IO_SCL)) != ESP_OK) {
    ESP_UTILS_LOGI("RTC Couldn't find device");
  }
  else {
    bool lostpower;
    if (ds3231_get_lost_power(&rtc_ext, &lostpower) != ESP_OK) {
      ESP_UTILS_LOGI("RTC Could not get lost power info.");
    }
    else{
      if (lostpower) {
        ESP_UTILS_LOGI("RTC lost power, let's set the time: %s %s", __DATE__, __TIME__);
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        if (ds3231_set_build_time(&rtc_ext, __DATE__, __TIME__) != ESP_OK) {
          ESP_UTILS_LOGI("RTC Could not set time.");
        }
        if (ds3231_reset_lost_power(&rtc_ext) != ESP_OK) {
          ESP_UTILS_LOGI("RTC lost power flag resetted.");
        }
      }
      struct tm rtc_info;	// rtc info data
      if (ds3231_get_time(&rtc_ext, &rtc_info) != ESP_OK) {
        ESP_UTILS_LOGI("RTC Could not get time.");
      }
      else {
        rtc.setTime(rtc_info.tm_sec, rtc_info.tm_min, rtc_info.tm_hour, rtc_info.tm_mday, rtc_info.tm_mon, rtc_info.tm_year);
        ESP_UTILS_LOGI("RTC actual date and time: %02d-%02d-%02d %02d:%02d:%02d", rtc.getDay(), rtc.getMonth()+1, rtc.getYear(), rtc.getHour(true), rtc.getMinute(),rtc.getSecond());
      }
    }
  }

  // Initialize board
  ESP_UTILS_LOGI("Initializing board");
  Board *board = new Board();
  board->init();
  // For I2C Touch
  static_cast<esp_panel::drivers::BusI2C *>(board->getTouch()->getBus())->configI2C_HostSkipInit();
  // For I2C IO_Expander
  board->getIO_Expander()->skipInitHost();
  #if LVGL_PORT_AVOID_TEARING_MODE
  auto lcd = board->getLCD();
  // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
  lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
  #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
  auto lcd_bus = lcd->getBus();
  /**
    * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
    * "bounce buffer" functionality to enhance the RGB data bandwidth.
    * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
    */
  if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
      static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
  }
  #endif
  #endif
  assert(board->begin());

  // Initialize LVGL and UI
  ESP_UTILS_LOGI("Initializing LVGL");
  lvgl_port_init(board->getLCD(), board->getTouch());

  ESP_UTILS_LOGI("Creating UI");
  lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
    ui_init();
    ui_post_init();
  lvgl_port_unlock(); /* Release the mutex */
  
  ui_update_clock();

  // Initialize UI events handler comm handler
  ESP_UTILS_LOGI("Initialize UI events handler");
  uiEventsInit();

  // Initialize Td5 comm handler
  ESP_UTILS_LOGI("Initialize Td5 comm handler");
  td5CommInit();

  // Initialize RTC
  /*
  ESP_UTILS_LOGI("Initialize RTC");
  if (! rtc.begin()) {
    ESP_UTILS_LOGI("Couldn't find RTC");
  }
  else{
    if (rtc.lostPower()) {
      ESP_UTILS_LOGI("RTC lost power, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    ui_update_clock();
  }
  */

  // initialize SD card
  ESP_UTILS_LOGI("Initialize SD card");
  SPI.setHwCs(false);
  SPI.begin(SD_CLK, SD_D0_MISO, SD_CMD_MOSI, SD_SS);
  if (!SD.begin(SD_SS)) {
    ESP_UTILS_LOGI("Card Mount Failed"); // SD card mounting failed
  }
  else {
    ESP_UTILS_LOGI("Card Mount Succeed"); // SD card mounting OK
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    ESP_UTILS_LOGI("No SD card attached"); // No SD card connected
  }
  
  // Get free memory
  ESP_UTILS_LOGI("Free Memory: %u bytes\n", ESP.getFreeHeap());
  ESP_UTILS_LOGI("Free PSRAM: %u bytes\n", ESP.getMaxAllocPsram());
}

void loop ()
{

}
