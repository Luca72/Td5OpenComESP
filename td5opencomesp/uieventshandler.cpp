#include "td5opencomesp.h"
#include "lvgl_v8_port.h"

#include "lvgl.h"
#include "src/ui/ui.h"
#include "uipostinit.h"

#include "td5comm.h"

#define ESP_UTILS_LOG_TAG "UiEventsHandler"
#include "esp_lib_utils.h"
#include "uieventshandler.h"

#include "td5commhandler.h"

#include "esp_event_base.h"

#define TIMER_PERIOD    60000000  // period of the timer event source in microseconds
esp_timer_handle_t TIMER_CLOCK_UPDATE;

// Helpers functions
float calc_avg_fuel_consumpt()
{
  static double avgFuelConsumptSum = 10.0;
  static long avgFuelConsumptCnt = 1.0; // MAX 1000000000L
  static byte offlimFuelConsumptCnt = 0.0;
  static float avgFuelConsumpt = 0.0;
  float instFuelConsumpt = (td5->getCurrentInj() * (float)td5->getEngineRPM()) * 0.000178571;
  float instFuelConsumptKml = (instFuelConsumpt > 0 ? ((float)td5->getVehicleSpeed() / instFuelConsumpt) : 0.0);

  if((instFuelConsumpt > 0) && ((float)td5->getVehicleSpeed() > 5) && ((float)td5->getEngineRPM() > 800))
  {
    if(instFuelConsumptKml < 15)
    {
      avgFuelConsumptSum = avgFuelConsumptSum + instFuelConsumptKml;
      avgFuelConsumptCnt++;
      avgFuelConsumpt = avgFuelConsumptSum / (double) avgFuelConsumptCnt;
    }
    else
    {
      if(offlimFuelConsumptCnt > 5)
      {
        avgFuelConsumptSum = avgFuelConsumptSum + (instFuelConsumptKml < 18 ? instFuelConsumptKml : 18.0);
        avgFuelConsumptCnt++;
        offlimFuelConsumptCnt = 0;
        avgFuelConsumpt = avgFuelConsumptSum / (double) avgFuelConsumptCnt;
      }
      else {
        offlimFuelConsumptCnt++;
      }
    }
    if(avgFuelConsumptCnt > 1000000000L)
    {
      avgFuelConsumptSum =  avgFuelConsumpt * 1000;
      avgFuelConsumptCnt = 1000;
    }
  }	

	return avgFuelConsumpt;
}

lv_color_t eval_background_color(float value, float wrn_min, float wrn_max, float alm_min, float alm_max){
  if((value >= wrn_min) && (value < wrn_max)){
    return lv_color_hex(0xFFEB3B);  // 0xFFEB3B yellow
  }
  else if((value >= alm_min) && (value < alm_max)){
    return lv_color_hex(0xF44336);  // 0xF44336 red
  }
  else {
    return lv_color_hex(0xFFFFFF);  // 0xFFFFFF white
  }
}

lv_color_t eval_background_color(int16_t value, int wrn_min, int wrn_max, int alm_min, int alm_max){
  return eval_background_color((float) value, (float) wrn_min, (float) wrn_max, (float) alm_min, (float) alm_max);
}

void ui_update_clock()
{
  char time[6];
  //sprintf(time, "%02d-%02d-%d, %02d:%02d", rtc.getYear(), rtc.getMonth()+1, rtc.getDay(), rtc.getHour(true), rtc.getMinute());
  sprintf(time, "%02d:%02d", rtc.getHour(true), rtc.getMinute());
  lvgl_port_lock(-1); // Lock the mutex due to the LVGL APIs are not thread-safe 
    lv_label_set_text(ui_LabelPanelDate, time);
  lvgl_port_unlock(); // Release the mutex
  if(active_screen == SCREEN_PREFERENCES) {
    lvgl_port_lock(-1); // Lock the mutex due to the LVGL APIs are not thread-safe 
      lv_roller_set_selected(ui_RollerPreferencesHourSet, rtc.getHour(true), LV_ANIM_OFF);
      lv_roller_set_selected(ui_RollerPreferencesMinuteSet, rtc.getMinute(), LV_ANIM_OFF);      
    lvgl_port_unlock(); // Release the mutex     
  }
}

// Callback that will be executed when the timer period lapses. Posts the timer expiry event
// to the default event loop.
static void timer_clock_update_callback(void* arg)
{
  ui_update_clock();
}


// Animations
lv_anim_t anim_RpmNeedle;
int16_t rpm_needle_current_angle = -750;
static void ui_animation_rpm_needle_callback(void * image, int32_t angle){
  lv_img_set_angle((lv_obj_t *) image, angle);
}

lv_anim_t anim_FuelConsNeedle;
int16_t fuel_cons_needle_current_angle = -600;
static void ui_animation_fuel_cons_needle_callback(void * image, int32_t angle){
  lv_img_set_angle((lv_obj_t *) image, angle);
}

lv_anim_t anim_TurboPNeedle;
int16_t turbo_p_needle_current_angle = -1200;
static void ui_animation_turbo_p_needle_callback(void * image, int32_t angle){
  lv_img_set_angle((lv_obj_t *) image, angle);
}

lv_anim_t anim_MAFNeedle;
int16_t maf_needle_current_angle = -1200;
static void ui_animation_maf_needle_callback(void * image, int32_t angle){
  lv_img_set_angle((lv_obj_t *) image, angle);
}

lv_anim_t anim_BatVNeedle;
int16_t bat_v_needle_current_angle = -600;
static void ui_animation_bat_v_needle_callback(void * image, int32_t angle){
  lv_img_set_angle((lv_obj_t *) image, angle);
}

lv_anim_t anim_DemandNeedle;
int16_t demand_needle_current_angle = -1200;
static void ui_animation_demand_needle_callback(void * image, int32_t angle){
  lv_img_set_angle((lv_obj_t *) image, angle);
}

lv_anim_t anim_CurrInjNeedle;
int16_t curr_inj_needle_current_angle = -1200;
static void ui_animation_curr_inj_needle_callback(void * image, int32_t angle){
  lv_img_set_angle((lv_obj_t *) image, angle);
}


// Event handlers
static void task_pid_rpm_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  switch (lv_tabview_get_tab_act(ui_TabViewInstrument)){
    case UI_INSTRUMENT_GAUGES_TAB:
      {
        int16_t angle = -1200 + (td5->getEngineRPM() * 0.6);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          //lv_img_set_angle(ui_ImageGaugeRPMNeedle, angle);
          lv_anim_set_values(&anim_RpmNeedle, rpm_needle_current_angle, angle);
          lv_anim_start(&anim_RpmNeedle);
        lvgl_port_unlock(); /* Release the mutex */
        rpm_needle_current_angle = angle;
      }
      break;
    case UI_INSTRUMENT_METERS_TAB:
      {
        //lv_color_t rpm_bg_color = (td5->getEngineRPM()<WRN_MAX_ENGINE_RPM ? lv_color_hex(0xFFFFFF) : (td5->getEngineRPM()>ALM_MAX_ENGINE_RPM ? lv_color_hex(0xF44336) : lv_color_hex(0xFFEB3B))); /* 0xFFEB3B yellow, 0xF44336 red */
        lv_color_t rpm_bg_color = eval_background_color(td5->getEngineRPM(), WRN_MIN_ENGINE_RPM, WRN_MAX_ENGINE_RPM, ALM_MIN_ENGINE_RPM, ALM_MAX_ENGINE_RPM);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_obj_set_style_bg_color(ui_LabelMeterRPMValue, rpm_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);  
          lv_label_set_text_fmt(ui_LabelMeterRPMValue, "%d", td5->getEngineRPM());
        lvgl_port_unlock(); /* Release the mutex */
      }
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_PID_RPM_EVENT event");
}

static void task_pid_turbo_pressure_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  switch (lv_tabview_get_tab_act(ui_TabViewInstrument)){
    case UI_INSTRUMENT_GAUGES_TAB:
      {
        int16_t turbo_angle = -900 + (td5->getTurboP() * 1500.0);
        int16_t maf_angle = -1200 + (td5->getMAF() * 6.0);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          //lv_img_set_angle(ui_ImageGaugeTurboPNeedle, turbo_angle);
          lv_anim_set_values(&anim_TurboPNeedle, turbo_p_needle_current_angle, turbo_angle);
          lv_anim_start(&anim_TurboPNeedle);      
          //lv_img_set_angle(ui_ImageGaugeMAFNeedle, maf_angle);
          lv_anim_set_values(&anim_MAFNeedle, maf_needle_current_angle, maf_angle);
          lv_anim_start(&anim_MAFNeedle);      
        lvgl_port_unlock(); /* Release the mutex */
        turbo_p_needle_current_angle = turbo_angle;
        maf_needle_current_angle = maf_angle;
      }
      break;
    case UI_INSTRUMENT_METERS_TAB:
      {
        //lv_color_t turbo_bg_color = (td5->getTurboP()<WRN_MAX_TURBO_P ? lv_color_hex(0xFFFFFF) : (td5->getTurboP()>ALM_MAX_TURBO_P ? lv_color_hex(0xF44336) : lv_color_hex(0xFFEB3B))); /* 0xFFEB3B yellow, 0xF44336 red */
        lv_color_t turbo_bg_color = eval_background_color(td5->getTurboP(), WRN_MIN_TURBO_P, WRN_MAX_TURBO_P, ALM_MIN_TURBO_P, ALM_MAX_TURBO_P);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_obj_set_style_bg_color(ui_LabelMeterTurboPValue, turbo_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);  
          lv_label_set_text_fmt(ui_LabelMeterTurboPValue, "%.2f", td5->getTurboP());   
          lv_label_set_text_fmt(ui_LabelMeterMAFValue, "%.0f", td5->getMAF());
        lvgl_port_unlock(); /* Release the mutex */
      }
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_PID_TURBO_PRESSURE_MAF_EVENT event");
}

static void task_pid_temperatures_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  //hmi->meterInletT.setValue(pidTemperatures.getfValue(1), 1);
  switch (lv_tabview_get_tab_act(ui_TabViewInstrument)){
    case UI_INSTRUMENT_GAUGES_TAB:
      {
        int16_t coolant_angle = -600 + ((limit_value(47.0, td5->getCoolantT(), 133.0 ) - 50.0) * 15.0);
        int16_t fuel_angle = -600 + ((limit_value(58.0, td5->getFuelT(), 102.0 ) - 60.0) * 30.0);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_img_set_angle(ui_ImageGaugeCoolantTNeedle, coolant_angle);
          lv_label_set_text_fmt(ui_LabelGaugeExtT, "%.1f", td5->getExtT());
          lv_img_set_angle(ui_ImageGaugeFuelTNeedle, fuel_angle);
        lvgl_port_unlock(); /* Release the mutex */
      }
      break;
    case UI_INSTRUMENT_METERS_TAB:
      {
        //lv_color_t coolant_bg_color = (td5->getCoolantT()<WRN_MAX_COOLANT_T ? lv_color_hex(0xFFFFFF) : (td5->getCoolantT()>ALM_MAX_COOLANT_T ? lv_color_hex(0xF44336) : lv_color_hex(0xFFEB3B))); /* 0xFFEB3B yellow, 0xF44336 red */
        lv_color_t coolant_bg_color = eval_background_color(td5->getCoolantT(), WRN_MIN_COOLANT_T, WRN_MAX_COOLANT_T, ALM_MIN_COOLANT_T, ALM_MAX_COOLANT_T);
        lv_color_t fuel_t_bg_color = eval_background_color(td5->getFuelT(), WRN_MIN_FUEL_T, WRN_MAX_FUEL_T, ALM_MIN_FUEL_T, ALM_MAX_FUEL_T);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_obj_set_style_bg_color(ui_LabelMeterCoolantTValue, coolant_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_label_set_text_fmt(ui_LabelMeterCoolantTValue, "%.1f", td5->getCoolantT());
          lv_label_set_text_fmt(ui_LabelMeterExtTValue, "%.1f", td5->getExtT());
          lv_obj_set_style_bg_color(ui_LabelMeterFuelTValue, fuel_t_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_label_set_text_fmt(ui_LabelMeterFuelTValue, "%.1f", td5->getFuelT());
        lvgl_port_unlock(); /* Release the mutex */     
      }
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_PID_TEMPERATURES_EVENT event");   
}

static void task_pid_live_fuelling_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  // ((((( CURRENT_INJ * (5/2) * RPM ) / 1000 ) * 60 ) / 0.84 ) / 1000)
  // instFuelConsumpt = (((((currentInjection*2.5*(float)engineRPM)/1000.0)*60.0)/0.84)/1000.0);
  float instFuelConsumpt = (td5->getCurrentInj() * (float)td5->getEngineRPM()) * 0.000178571;
  float instFuelConsumptKml = (instFuelConsumpt > 0 ? ((float)td5->getVehicleSpeed() / instFuelConsumpt) : LIM_MAX_FUEL_CONS);
  switch (lv_tabview_get_tab_act(ui_TabViewInstrument)){
    case UI_INSTRUMENT_GAUGES_TAB:
      {
        if((instFuelConsumptKml>LIM_MAX_FUEL_CONS) || (td5->getEngineRPM() < 5)) {
            instFuelConsumptKml = LIM_MAX_FUEL_CONS;  
          }
        int16_t angle = -600 + (limit_value(0.0, instFuelConsumptKml, 22.0 ) * 60);
        int16_t angle_demand = -1200 + (td5->getDriverDemand() * 30);
        int16_t angle_curr_inj = -1200 + (td5->getCurrentInj() * 30);
    	//int16_t maf_angle = -1200 + (td5->getMAF() * 3.0);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          //lv_img_set_angle(ui_ImageGaugeFuelConsNeedle, angle);
          lv_anim_set_values(&anim_FuelConsNeedle, fuel_cons_needle_current_angle, angle);
          lv_anim_start(&anim_FuelConsNeedle);
          //lv_img_set_angle(ui_ImageGaugeDemandNeedle, angle_demand);
          lv_anim_set_values(&anim_DemandNeedle, demand_needle_current_angle, angle_demand);
          lv_anim_start(&anim_DemandNeedle);
          //lv_img_set_angle(ui_ImageGaugeCurrInjNeedle, angle_curr_inj);
          lv_anim_set_values(&anim_CurrInjNeedle, curr_inj_needle_current_angle, angle_curr_inj);
          lv_anim_start(&anim_CurrInjNeedle);
        lvgl_port_unlock(); /* Release the mutex */
        demand_needle_current_angle = angle_demand;
        curr_inj_needle_current_angle = angle_curr_inj;
        fuel_cons_needle_current_angle = angle;
      }
      break;
    case UI_INSTRUMENT_METERS_TAB:
      {
        lv_color_t curr_inj_bg_color = ((td5->getDriverDemand() < td5->getSmokeLim()) && (td5->getDriverDemand() < td5->getTorqueLim())) ? lv_color_hex(0xFFFFFF) : lv_color_hex(0xFFEB3B); /* 0xFFEB3B yellow */
        //lv_color_t fuel_cons_bg_color = (((instFuelConsumptKml<=LIM_MIN_FUEL_CONS) || (instFuelConsumptKml>ALM_MAX_FUEL_CONS)) ? lv_color_hex(0xFFFFFF) : (instFuelConsumptKml>WRN_MAX_FUEL_CONS ? lv_color_hex(0xFFEB3B) : lv_color_hex(0xF44336))); /* 0xFFEB3B yellow, 0xF44336 red */
        lv_color_t fuel_cons_bg_color = eval_background_color(instFuelConsumptKml, WRN_MIN_FUEL_CONS, WRN_MAX_FUEL_CONS, ALM_MIN_FUEL_CONS, ALM_MAX_FUEL_CONS);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_obj_set_style_bg_color(ui_LabelMeterCurrInjValue, curr_inj_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_label_set_text_fmt(ui_LabelMeterCurrInjValue, "%.1f", td5->getCurrentInj());
          lv_obj_set_style_bg_color(ui_LabelMeterFuelConsValue, fuel_cons_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
          if((instFuelConsumptKml<LIM_MAX_FUEL_CONS) && (instFuelConsumptKml>LIM_MIN_FUEL_CONS) && (td5->getEngineRPM() > 5)) {
            lv_label_set_text_fmt(ui_LabelMeterFuelConsValue, "%.1f", instFuelConsumptKml);  
          }
          else {
            lv_label_set_text(ui_LabelMeterFuelConsValue, "--.-");
          }
          lvgl_port_unlock(); /* Release the mutex */
      }
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_PID_LIVE_FUELLING_EVENT event");     
}

static void task_pid_battery_voltage_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  switch (lv_tabview_get_tab_act(ui_TabViewInstrument)){
    case UI_INSTRUMENT_GAUGES_TAB:
      {
        int16_t angle = -600 + ((limit_value(8.0, td5->getBattery(), 16.5 ) - 8.0) * 150.0);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          //lv_img_set_angle(ui_ImageGaugeBatVNeedle, angle);
          lv_anim_set_values(&anim_BatVNeedle, bat_v_needle_current_angle, angle);
          lv_anim_start(&anim_BatVNeedle);       
        lvgl_port_unlock(); /* Release the mutex */
        bat_v_needle_current_angle = angle;
      }
      break;
    case UI_INSTRUMENT_METERS_TAB:
      {
        lv_color_t battery_bg_color = eval_background_color(td5->getBattery(), WRN_MIN_BATTERY, WRN_MAX_BATTERY, ALM_MIN_BATTERY, ALM_MAX_BATTERY);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_obj_set_style_bg_color(ui_LabelMeterBatVValue, battery_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_label_set_text_fmt(ui_LabelMeterBatVValue, "%.1f", td5->getBattery());
        lvgl_port_unlock(); /* Release the mutex */  
      }
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_PID_BATTERY_VOLTAGE_EVENT event");      
}

static void task_pid_ambient_pressure_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  switch (lv_tabview_get_tab_act(ui_TabViewInstrument)){
    case UI_INSTRUMENT_GAUGES_TAB:
      {
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_label_set_text_fmt(ui_LabelGaugeAmbientP, "%.2f", td5->getAmbientP());
        lvgl_port_unlock(); /* Release the mutex */
      }
      break;
    case UI_INSTRUMENT_METERS_TAB:
      {
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_label_set_text_fmt(ui_LabelMeterAmbientPValue, "%.2f", td5->getAmbientP());
        lvgl_port_unlock(); /* Release the mutex */  
      }
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_PID_AMBIENT_PRESSURE_EVENT event");    
}

static void task_pid_vehicle_speed_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  //lv_label_set_text_fmt(ui_LabelVehicleSpeedValue, "%d", td5->getVehicleSpeed());
	float instAvgConsumpt = calc_avg_fuel_consumpt();
  switch (lv_tabview_get_tab_act(ui_TabViewInstrument)){
    case UI_INSTRUMENT_GAUGES_TAB:
      {
        int16_t angle = -600 + (limit_value(0.0, instAvgConsumpt, 20.0 ) * 60);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_img_set_angle(ui_ImageGaugeAvgFuelConsNeedle, angle);
        lvgl_port_unlock(); /* Release the mutex */
      }
      break;
    case UI_INSTRUMENT_METERS_TAB:
      {
        //lv_color_t avg_fuel_cons_bg_color = (((instAvgConsumpt<=LIM_MIN_FUEL_CONS) || (instAvgConsumpt>ALM_MAX_FUEL_CONS)) ? lv_color_hex(0xFFFFFF) : (instAvgConsumpt>WRN_MAX_FUEL_CONS ? lv_color_hex(0xFFEB3B) : lv_color_hex(0xF44336))); /* 0xFFEB3B yellow, 0xF44336 red */
        lv_color_t avg_fuel_cons_bg_color = eval_background_color(instAvgConsumpt, WRN_MIN_FUEL_CONS, WRN_MAX_FUEL_CONS, ALM_MIN_FUEL_CONS, ALM_MAX_FUEL_CONS);
        lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */
          lv_obj_set_style_bg_color(ui_LabelMeterAvgFuelConsValue, avg_fuel_cons_bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);  
          lv_label_set_text_fmt(ui_LabelMeterAvgFuelConsValue, "%.1f", instAvgConsumpt);
        lvgl_port_unlock(); /* Release the mutex */
      }
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_PID_VEHICLE_SPEED_EVENT event");   
}

static void task_lost_connection_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  //int data = *((int*) event_data);
  _ui_screen_change(&ui_ScreenStartup, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenStartup_screen_init);
  lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
    lv_obj_t *ui_MsgBox;
    if(ui_MsgBox != NULL) delete ui_MsgBox;
    ui_MsgBox = lv_msgbox_create(NULL, "ERROR", "Lost connection to ECU", NULL, true);
    lv_obj_set_style_text_font(ui_MsgBox, &lv_font_montserrat_22, LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_MsgBox, &lv_font_montserrat_22, LV_PART_MAIN);
    lv_obj_set_size(ui_MsgBox, 300, 200);
    lv_obj_center(ui_MsgBox);     
    lv_obj_add_flag(ui_ImagePanelIconObdOn, LV_OBJ_FLAG_HIDDEN);
    lv_bar_set_value(ui_BarStartupConn, 0, LV_ANIM_OFF);
  lvgl_port_unlock(); /* Release the mutex */
  active_screen = SCREEN_STARTUP;
  //ESP_UTILS_LOGI("UI handler received UI_LOST_CONNECTION event");    
}

static void task_connecting_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  int conn_advance_perc = (int)((float)(data+1) * (100.0 / 7.0));
  lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
    lv_bar_set_value(ui_BarStartupConn, conn_advance_perc, LV_ANIM_ON);
  lvgl_port_unlock(); /* Release the mutex */
  //ESP_UTILS_LOGI("UI handler received UI_LOST_CONNECTION event");    
}

static void task_connected_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
    lv_obj_clear_flag(ui_ImagePanelIconObdOn, LV_OBJ_FLAG_HIDDEN);
  lvgl_port_unlock(); /* Release the mutex */
  switch(next_screen) {
    case SCREEN_INSTRUMENT:
      _ui_screen_change(&ui_ScreenInstrument, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenInstrument_screen_init);
      active_screen = SCREEN_INSTRUMENT;
      next_screen = SCREEN_STARTUP;
      ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_START_INSTRUMENT_MODE_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));    
      break;
    
    default:
      break;
  }
  //ESP_UTILS_LOGI("UI handler received UI_LOST_CONNECTED event");    
}

static void task_read_ecu_data_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  if (data == TD5_COMM_COMMAND_FAILED){
    lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
      lv_obj_t *ui_MsgBox;
      if(ui_MsgBox != NULL) delete ui_MsgBox;
      ui_MsgBox = lv_msgbox_create(NULL, "ERROR", "Ecu Read Data: reading failed.", NULL, true);
      lv_obj_set_style_text_font(ui_MsgBox, &lv_font_montserrat_22, LV_PART_MAIN);
      lv_obj_set_size(ui_MsgBox, 300, 200);
      lv_obj_center(ui_MsgBox);     
    lvgl_port_unlock(); /* Release the mutex */
  }
  else {
    char vin[18];pid_extract_ecu_vin(vin);
    char prod_date[11];pid_extract_ecu_prod_date(prod_date);
    char nnw[10];pid_extract_ecu_nnw(nnw);
    char homol[5];pid_extract_ecu_homol(homol);
    char ecu_model[10];pid_extract_ecu_model(ecu_model);
    char firmware[18];pid_extract_ecu_firmware(firmware);
    char ecu_status[7];pid_extract_ecu_status(ecu_status);

    lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
      lv_label_set_text_fmt(ui_LabelEcuInfoVIN, "VIN: %s", vin); 
      lv_label_set_text_fmt(ui_LabelEcuInfoProductionDate, "Product date: %s", prod_date); 
      lv_label_set_text_fmt(ui_LabelEcuInfoPN, "ECU P/N.: %s", nnw); 
      lv_label_set_text_fmt(ui_LabelEcuInfoHomol, "Homologation: %s", homol); 
      lv_label_set_text_fmt(ui_LabelEcuInfoModel, "ECU model: %s", ecu_model); 
      lv_label_set_text_fmt(ui_LabelEcuInfoFirmware, "Firmware: %s", firmware); 
      lv_label_set_text_fmt(ui_LabelEcuInfoStatus, "ECU status: %s", ecu_status); 
    lvgl_port_unlock(); /* Release the mutex */
  }
  //ESP_UTILS_LOGI("UI handler received UI_READ_ECU_DATA_EVENT event");
}

static void task_read_fault_codes_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  if (data == TD5_COMM_COMMAND_FAILED){
    lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
      lv_obj_t *ui_MsgBox;
      if(ui_MsgBox != NULL) delete ui_MsgBox;
      ui_MsgBox = lv_msgbox_create(NULL, "ERROR", "Ecu Read Fault Codes: reading failed.", NULL, true);
      lv_obj_set_style_text_font(ui_MsgBox, &lv_font_montserrat_22, LV_PART_MAIN);
      lv_obj_set_size(ui_MsgBox, 300, 200);
      lv_obj_center(ui_MsgBox);     
    lvgl_port_unlock(); /* Release the mutex */
  }
  else {
    int fault_codes_num = data;
    uint16_t row = 0;
    if(fault_codes_num > 0){
      lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
        while(row < MAX_FAULT_CODES) {
          if(row < (fault_codes_num - 1)){
            lv_table_set_cell_value(ui_TableEcuFaultCodes, row, 0, fault_code[td5->getFaultCode(row)]);
          }
          else {
            lv_table_set_cell_value(ui_TableEcuFaultCodes, row, 0, "");
          }
          row++;
        }
      lvgl_port_unlock(); /* Release the mutex */
    }
  }
  //ESP_UTILS_LOGI("UI handler received UI_READ_FAULT_CODES_EVENT event of %d fault codes", fault_codes_num);
}

static void task_clear_fault_codes_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  if (data == TD5_COMM_COMMAND_OK){
    /*
    lvgl_port_lock(-1);
      lv_obj_t *ui_MsgBox;
      if(ui_MsgBox != NULL) delete ui_MsgBox;
      ui_MsgBox = lv_msgbox_create(NULL, "", "Ecu Fault Codes CLEARED.", NULL, true);
      lv_obj_set_style_text_font(ui_MsgBox, &lv_font_montserrat_22, LV_PART_MAIN);
      lv_obj_set_size(ui_MsgBox, 300, 200);
      lv_obj_center(ui_MsgBox);     
    lvgl_port_unlock();
    */
    ESP_UTILS_LOGI("UI handler UI_READ_FAULT_CODES_EVENT successful");
  }
  else {
    lvgl_port_lock(-1); /* Lock the mutex due to the LVGL APIs are not thread-safe */  
      lv_obj_t *ui_MsgBox;
      if(ui_MsgBox != NULL) delete ui_MsgBox;
      ui_MsgBox = lv_msgbox_create(NULL, "ERROR", "Ecu Clear Fault Codes: reading failed.", NULL, true);
      lv_obj_set_style_text_font(ui_MsgBox, &lv_font_montserrat_22, LV_PART_MAIN);
      lv_obj_set_size(ui_MsgBox, 300, 200);
      lv_obj_center(ui_MsgBox);     
    lvgl_port_unlock(); /* Release the mutex */
  }
  //ESP_UTILS_LOGI("UI handler received UI_CLEAR_FAULT_CODES_EVENT event");
}

bool uiEventsInit()
{
  ESP_UTILS_LOGI("Initializing UI events");

 // Animations
  lv_anim_init(&anim_RpmNeedle);
  lv_anim_set_var(&anim_RpmNeedle, ui_ImageGaugeRPMNeedle);
  lv_anim_set_time(&anim_RpmNeedle, 250);
  lv_anim_set_exec_cb(&anim_RpmNeedle, ui_animation_rpm_needle_callback);
  lv_anim_set_path_cb(&anim_RpmNeedle, lv_anim_path_linear);

  lv_anim_init(&anim_FuelConsNeedle);
  lv_anim_set_var(&anim_FuelConsNeedle, ui_ImageGaugeFuelConsNeedle);
  lv_anim_set_time(&anim_FuelConsNeedle, 500);
  lv_anim_set_exec_cb(&anim_FuelConsNeedle, ui_animation_fuel_cons_needle_callback);
  lv_anim_set_path_cb(&anim_FuelConsNeedle, lv_anim_path_linear);

  lv_anim_init(&anim_TurboPNeedle);
  lv_anim_set_var(&anim_TurboPNeedle, ui_ImageGaugeTurboPNeedle);
  lv_anim_set_time(&anim_TurboPNeedle, 250);
  lv_anim_set_exec_cb(&anim_TurboPNeedle, ui_animation_turbo_p_needle_callback);
  lv_anim_set_path_cb(&anim_TurboPNeedle, lv_anim_path_linear);

  lv_anim_init(&anim_MAFNeedle);
  lv_anim_set_var(&anim_MAFNeedle, ui_ImageGaugeMAFNeedle);
  lv_anim_set_time(&anim_MAFNeedle, 250);
  lv_anim_set_exec_cb(&anim_MAFNeedle, ui_animation_maf_needle_callback);
  lv_anim_set_path_cb(&anim_MAFNeedle, lv_anim_path_linear);

  lv_anim_init(&anim_BatVNeedle);
  lv_anim_set_var(&anim_BatVNeedle, ui_ImageGaugeBatVNeedle);
  lv_anim_set_time(&anim_BatVNeedle, 250);
  lv_anim_set_exec_cb(&anim_BatVNeedle, ui_animation_bat_v_needle_callback);
  lv_anim_set_path_cb(&anim_BatVNeedle, lv_anim_path_linear);

  lv_anim_init(&anim_CurrInjNeedle);
  lv_anim_set_var(&anim_CurrInjNeedle, ui_ImageGaugeCurrInjNeedle);
  lv_anim_set_time(&anim_CurrInjNeedle, 250);
  lv_anim_set_exec_cb(&anim_CurrInjNeedle, ui_animation_curr_inj_needle_callback);
  lv_anim_set_path_cb(&anim_CurrInjNeedle, lv_anim_path_linear);

  lv_anim_init(&anim_DemandNeedle);
  lv_anim_set_var(&anim_DemandNeedle, ui_ImageGaugeDemandNeedle);
  lv_anim_set_time(&anim_DemandNeedle, 250);
  lv_anim_set_exec_cb(&anim_DemandNeedle, ui_animation_demand_needle_callback);
  lv_anim_set_path_cb(&anim_DemandNeedle, lv_anim_path_linear);

  // Event handlers
  ESP_UTILS_LOGI("Register UI event handlers");
  // Register the handler for task iteration event. The loop handle is provided as an argument in order for this example to display the loop the handler is being run on.
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_PID_RPM_EVENT, task_pid_rpm_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_pid_rpm_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_PID_TURBO_PRESSURE_MAF_EVENT, task_pid_turbo_pressure_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_pid_turbo_pressure_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_PID_TEMPERATURES_EVENT, task_pid_temperatures_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_pid_temperatures_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_PID_LIVE_FUELLING_EVENT, task_pid_live_fuelling_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_pid_live_fuelling_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_PID_BATTERY_VOLTAGE_EVENT, task_pid_battery_voltage_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_pid_battery_voltage_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_PID_AMBIENT_PRESSURE_EVENT, task_pid_ambient_pressure_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_pid_ambient_pressure_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_PID_VEHICLE_SPEED_EVENT, task_pid_vehicle_speed_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_pid_vehicle_speed_event_handler failed");    
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_LOST_CONNECTION, task_lost_connection_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_lost_connection_event_handler failed");    
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_CONNECTING, task_connecting_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_connecting_event_handler failed");    
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_CONNECTED, task_connected_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_connected_event_handler failed");    
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_READ_ECU_DATA_EVENT, task_read_ecu_data_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_read_ecu_data_event_handler failed");    
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_READ_FAULT_CODES_EVENT, task_read_fault_codes_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_read_fault_codes_event_handler failed");    
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, UI_EVENTS, UI_CLEAR_FAULT_CODES_EVENT, task_clear_fault_codes_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_clear_fault_codes_event_handler failed");    
  
  // Create and start the event sources
  ESP_UTILS_LOGI("Create and start clock update timer");
  esp_timer_create_args_t timer_args = {
      .callback = &timer_clock_update_callback,
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &TIMER_CLOCK_UPDATE));
  ESP_ERROR_CHECK(esp_timer_start_periodic(TIMER_CLOCK_UPDATE, TIMER_PERIOD));

  // Button handlers
  lv_obj_add_event_cb(ui_ScreenStartup, ui_event_ScreenStartup, LV_EVENT_SCREEN_LOAD_START, NULL);
  lv_obj_add_event_cb(ui_ButtonScreenInstrument, ui_event_ButtonScreenInstrument, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(ui_ButtonScreenFuelling, ui_event_ButtonScreenFuelling, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(ui_ButtonScreenEcu, ui_event_ButtonScreenEcu, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(ui_ButtonScreenTest, ui_event_ButtonScreenTest, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(ui_ButtonScreenPreferences, ui_event_ButtonScreenPreferences, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(ui_ButtonStartupConn, ui_event_ButtonStartupConn, LV_EVENT_PRESSED, NULL);  
  lv_obj_add_event_cb(ui_ButtonEcuFaultCodesUpdate, ui_event_ButtonEcuFaultCodesUpdate, LV_EVENT_PRESSED, NULL);  
  lv_obj_add_event_cb(ui_ButtonEcuFaultCodesClear, ui_event_ButtonEcuFaultCodesClear, LV_EVENT_PRESSED, NULL);

  lv_obj_add_event_cb(ui_RollerPreferencesHourSet, ui_event_RollerPreferencesHourSet, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_RollerPreferencesMinuteSet, ui_event_RollerPreferencesMinuteSet, LV_EVENT_VALUE_CHANGED, NULL);  
  lv_obj_add_event_cb(ui_CalendarPreferencesDateSet, ui_event_CalendarPreferencesDateSet, LV_EVENT_VALUE_CHANGED, NULL);  

  lv_obj_add_event_cb(ui_ScreenPreferences, ui_event_ScreenPreferences, LV_EVENT_SCREEN_LOAD_START, NULL);
  lv_obj_add_event_cb(ui_ScreenInstrument, ui_event_ScreenInstrument, LV_EVENT_SCREEN_LOAD_START, NULL);
  lv_obj_add_event_cb(ui_ScreenInstrument, ui_event_ScreenInstrument, LV_EVENT_SCREEN_UNLOAD_START, NULL);  
  lv_obj_add_event_cb(ui_TabViewEcu, ui_event_TabViewEcu, LV_EVENT_VALUE_CHANGED, NULL);
  
  lv_obj_add_event_cb(ui_ButtonPreferencesDateAndTimeUpdate, ui_event_ButtonPreferencesDateAndTimeUpdate, LV_EVENT_PRESSED, NULL); 

  active_screen = SCREEN_STARTUP;
  next_screen = SCREEN_STARTUP;

  return true;
}


// Screen handlers
int8_t active_screen = SCREEN_UNKNOWN;
int8_t next_screen = SCREEN_UNKNOWN;
int8_t active_ecu_tab = ECU_TAB_UNKNOWN;

bool uiGetActiveScreen() { 
  return active_screen; 
}

void ui_event_ButtonScreenInstrument(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_PRESSED) {
    int data = 0;
    if(td5->ecuIsConnected() || TEST_UI) {
      _ui_screen_change(&ui_ScreenInstrument, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenInstrument_screen_init);
      active_screen = SCREEN_INSTRUMENT;
      ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_START_INSTRUMENT_MODE_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
    }
    else {
      if(active_screen != SCREEN_STARTUP) {
        _ui_screen_change(&ui_ScreenStartup, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenStartup_screen_init);
        active_screen = SCREEN_STARTUP;
      }
      next_screen = SCREEN_INSTRUMENT;
      ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_CONNECT_TO_ECU_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
    }       
  }
}

void ui_event_ButtonScreenFuelling(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_PRESSED) {
    int data = 0;
    _ui_screen_change(&ui_ScreenFuelling, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenFuelling_screen_init);
    active_screen = SCREEN_FUELLING;
  }
}

void ui_event_ButtonScreenEcu(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_PRESSED) {
    _ui_screen_change(&ui_ScreenEcu, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenEcu_screen_init);
    int data = 0;
    ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_READ_ECU_DATA_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
    active_screen = SCREEN_ECU;
  }
}

void ui_event_TabViewEcu(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
	uint16_t selected_tab = lv_tabview_get_tab_act(ui_TabViewEcu);

  if((event_code == LV_EVENT_VALUE_CHANGED) && (selected_tab != active_ecu_tab))
  {
    if(selected_tab == ECU_TAB_DATA) {
      int data = 0;
      ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_READ_ECU_DATA_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
    }

    if(selected_tab == ECU_TAB_FAULT_CODES) {
      int data = 0;
      ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_READ_FAULT_CODES_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
    }
    
      active_ecu_tab = selected_tab;
  }
}

void ui_event_ButtonScreenTest(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_PRESSED) {
    int data = 0;
    _ui_screen_change(&ui_ScreenTest, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenTest_screen_init);
    active_screen = SCREEN_TEST;
  }
}

void ui_event_ButtonScreenPreferences(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_PRESSED) {
    int data = 0;
    _ui_screen_change(&ui_ScreenPreferences, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenPreferences_screen_init);
    active_screen = SCREEN_PREFERENCES;
  }
}

void ui_event_ButtonStartupConn(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  int data = 0;

  if(event_code == LV_EVENT_PRESSED) {
      ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_CONNECT_TO_ECU_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
  }
}

void ui_event_ButtonEcuFaultCodesUpdate(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  int data = 0;

  if(event_code == LV_EVENT_PRESSED) {
    ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_READ_FAULT_CODES_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
  }
}

void ui_event_ButtonEcuFaultCodesClear(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  int data = 0;

  if(event_code == LV_EVENT_PRESSED) {
    ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_CLEAR_FAULT_CODES_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));
  }
}

void ui_event_RollerPreferencesHourSet(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
    }
}

void ui_event_RollerPreferencesMinuteSet(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
    }
}

void ui_event_CalendarPreferencesDateSet(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
      lv_calendar_date_t date;
      if(lv_calendar_get_pressed_date(ui_CalendarPreferencesDateSet, &date)) {
        lvgl_port_lock(-1); // Lock the mutex due to the LVGL APIs are not thread-safe 
          lv_calendar_set_today_date(ui_CalendarPreferencesDateSet, date.year, date.month, date.day);
        lvgl_port_unlock(); // Release the mutex     
      }          
    }
}

void ui_event_ScreenPreferences(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_SCREEN_LOAD_START) {
    lvgl_port_lock(-1); // Lock the mutex due to the LVGL APIs are not thread-safe 
      lv_roller_set_selected(ui_RollerPreferencesHourSet, rtc.getHour(true), LV_ANIM_OFF);
      lv_roller_set_selected(ui_RollerPreferencesMinuteSet, rtc.getMinute(), LV_ANIM_OFF);
      lv_calendar_set_today_date(ui_CalendarPreferencesDateSet, rtc.getYear(), rtc.getMonth()+1, rtc.getDay());
      lv_calendar_set_showed_date(ui_CalendarPreferencesDateSet, rtc.getYear(), rtc.getMonth()+1);      
    lvgl_port_unlock(); // Release the mutex     
  }
}

void ui_event_ScreenInstrument(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  int data = 0;
/*if(event_code == LV_EVENT_SCREEN_LOAD_START) {
   }*/
  if(event_code == LV_EVENT_SCREEN_UNLOAD_START) {
    ESP_ERROR_CHECK(esp_event_post_to(event_loop, COMM_EVENTS, COMM_STOP_INSTRUMENT_MODE_REQUEST_EVENT, &data, sizeof(data), portMAX_DELAY));       
  }    
}


void ui_event_ButtonPreferencesDateAndTimeUpdate(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  int data = 0;

  if(event_code == LV_EVENT_PRESSED) {
    const lv_calendar_date_t *pdate = lv_calendar_get_today_date(ui_CalendarPreferencesDateSet);
    lv_calendar_date_t date = *pdate;
    ESP_UTILS_LOGI("RTC actual date and time: %02d-%02d-%02d %02d:%02d:%02d", date.day, date.month, date.year, lv_roller_get_selected(ui_RollerPreferencesHourSet), lv_roller_get_selected(ui_RollerPreferencesMinuteSet),0);
    rtc.setTime(30, lv_roller_get_selected(ui_RollerPreferencesMinuteSet), lv_roller_get_selected(ui_RollerPreferencesHourSet), date.day, date.month, date.year);
    //rtc_ext.setRtcTime(30, lv_roller_get_selected(ui_RollerPreferencesMinuteSet), lv_roller_get_selected(ui_RollerPreferencesHourSet), 0 ,date.day, date.month, date.year-2000);
    update_rtc_ext(30, lv_roller_get_selected(ui_RollerPreferencesMinuteSet), lv_roller_get_selected(ui_RollerPreferencesHourSet), date.day, date.month, date.year);
    ui_update_clock();
  }
}

void ui_event_ScreenStartup(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
      lvgl_port_lock(-1); // Lock the mutex due to the LVGL APIs are not thread-safe 
        lv_bar_set_value(ui_BarStartupConn, 0, LV_ANIM_ON);
      lvgl_port_unlock(); // Release the mutex     
      ui_update_clock();
    }
}
