#pragma once

#include "lvgl.h"

#include "src/ui/ui_helpers.h"
#include "src/ui/ui_events.h"

#define TEST_UI true    

// Helper functions
void ui_update_clock();

// UI Events
extern void ui_event_ScreenStartup(lv_event_t * e);
extern void ui_event_ButtonScreenInstrument(lv_event_t * e);
extern void ui_event_ButtonScreenFuelling(lv_event_t * e);
extern void ui_event_ButtonScreenEcu(lv_event_t * e);
extern void ui_event_ButtonScreenTest(lv_event_t * e);
extern void ui_event_ButtonScreenPreferences(lv_event_t * e);
extern void ui_event_ButtonStartupConn(lv_event_t * e);
extern void ui_event_ButtonEcuFaultCodesUpdate(lv_event_t * e);
extern void ui_event_ButtonEcuFaultCodesClear(lv_event_t * e);

extern void ui_event_RollerPreferencesHourSet(lv_event_t * e);
extern void ui_event_RollerPreferencesMinuteSet(lv_event_t * e);
extern void ui_event_CalendarPreferencesDateSet(lv_event_t * e);

extern void ui_event_ScreenPreferences(lv_event_t * e);
extern void ui_event_ScreenInstrument(lv_event_t * e);
extern void ui_event_TabViewEcu(lv_event_t * e);
extern void ui_event_ButtonPreferencesDateAndTimeUpdate(lv_event_t * e);

bool uiEventsInit();


// Screens handlers
extern int8_t active_screen;
extern int8_t next_screen;
extern int8_t active_ecu_tab;

bool uiGetActiveScreen();

enum {
  SCREEN_STARTUP,
  SCREEN_INSTRUMENT,
  SCREEN_FUELLING,
  SCREEN_ECU,
  SCREEN_TEST,
  SCREEN_PREFERENCES,
  
  SCREEN_UNKNOWN // init screen
};

enum {
  ECU_TAB_DATA,
  ECU_TAB_FAULT_CODES,
  ECU_TAB_SETTINGS,
  ECU_TAB_FAULT_MAP,
  
  ECU_TAB_UNKNOWN // init tab  
};

enum {
    UI_INSTRUMENT_GAUGES_TAB,
    UI_INSTRUMENT_METERS_TAB,
};

ESP_EVENT_DECLARE_BASE(UI_EVENTS);         // declaration of the task events family
enum {
    UI_PID_RPM_EVENT,
    UI_PID_TURBO_PRESSURE_MAF_EVENT,
    UI_PID_TEMPERATURES_EVENT,
    UI_PID_LIVE_FUELLING_EVENT,
    UI_PID_AMBIENT_PRESSURE_EVENT,
    UI_PID_BATTERY_VOLTAGE_EVENT,
    UI_PID_VEHICLE_SPEED_EVENT,
    UI_CONNECTING,
    UI_LOST_CONNECTION,
    UI_CONNECTED,
    UI_READ_ECU_DATA_EVENT,
    UI_READ_FAULT_CODES_EVENT,
    UI_READ_ECU_SETTINGS_EVENT,
    UI_CLEAR_FAULT_CODES_EVENT,
};
