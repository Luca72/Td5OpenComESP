#include "td5opencomesp.h"

#define ESP_UTILS_LOG_TAG "Td5CommHandler"
#include "esp_lib_utils.h"

#include "td5commhandler.h"
#include "td5comm.h"

#include "uieventshandler.h"


static TaskHandle_t td5CommHandle = nullptr;

int commState = TD5_COMM_HANDLER_IDLE_STATE;
int data = 0;

static void td5CommTask(void *arg)
{
	uint8_t initStep;

    ESP_UTILS_LOGI("Starting Td5 comm task");

    while (1) {
        //ESP_UTILS_LOGI("Td5 comm task high watermark %d", uxTaskGetStackHighWaterMark(NULL));
        //ESP_UTILS_LOGI("Td5 comm task send COMM_PID_RPM_EVENT");
        //ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_RPM_EVENT, &data, sizeof(data), portMAX_DELAY));

		switch (commState){
			case TD5_COMM_HANDLER_IDLE_STATE:
				if(td5->ecuIsConnected()){
					// keep ecu alive
					if(td5->getLastReceivedPidElapsedTime() > KEEP_ALIVE_TIME){
						td5->getPid(&pidKeepAlive);  
					}
					// shutdown in case of too many frames lost
					if (td5->getConsecutiveLostFrames() > /*TD5_COMM_MAX_LOSTFRAMES*/1){
            td5->closeComm();
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_LOST_CONNECTION, &data, sizeof(data), portMAX_DELAY));
						ESP_UTILS_LOGI("[ERROR] Comm handler too many lost frames: close comunication ");
					}
				}
				break;
			
			case TD5_COMM_HANDLER_CONNECT_TO_ECU_STATE:
				if(!td5->ecuIsConnected()){
					initStep = TD5_COMM_INIT_SETUP;
					td5->setInitStep(initStep);
					// hmi->viewConn.setValue((int)(float)initStep*(100.0/7.0));
					do{
						if(td5->getInitStep() > initStep){
              data = initStep;
              ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_CONNECTING, &data, sizeof(data), portMAX_DELAY));
							ESP_UTILS_LOGI("Comm handler connecting to ecu at step: %i", initStep);
							initStep = td5->getInitStep();
							// hmi->viewConn.setValue((int)(float)initStep*(100.0/7.0));
						}						
						td5->initComm();
					}while (td5->getInitStep() != TD5_COMM_INIT_SETUP);
					td5->resetInitTime();
					if(td5->ecuIsConnected()){
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_CONNECTED, &data, sizeof(data), portMAX_DELAY));
						ESP_UTILS_LOGI("Comm handler connected to ecu succesfully");
						td5->getPid(&pidStartFuelling);
						commState = TD5_COMM_HANDLER_IDLE_STATE;
						break;
					}						
					else{
						ESP_UTILS_LOGI("[ERROR] Comm handler connection to ecu failed at init step: %i", initStep);
						commState = TD5_COMM_HANDLER_IDLE_STATE;
						break;
					}						
				}
				break;		
			
			case TD5_COMM_HANDLER_INSTRUMENT_STATE:
        if(td5->ecuIsConnected()){
          if(td5->getPid(&pidRPM) > 0) {
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_RPM_EVENT, &data, sizeof(data), portMAX_DELAY));
            break;
          }
          if(td5->getPid(&pidTurboPressureMaf) > 0) {
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_TURBO_PRESSURE_MAF_EVENT, &data, sizeof(data), portMAX_DELAY));
            break;
          }
          if(td5->getPid(&pidLiveFuelling) > 0) {
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_LIVE_FUELLING_EVENT, &data, sizeof(data), portMAX_DELAY));
            break;
          }
          if(td5->getPid(&pidTemperatures) > 0) {
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_TEMPERATURES_EVENT, &data, sizeof(data), portMAX_DELAY));
            break;
          }
          if(td5->getPid(&pidBatteryVoltage) > 0) {
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_BATTERY_VOLTAGE_EVENT, &data, sizeof(data), portMAX_DELAY)); 
            break;
          }
          if(td5->getPid(&pidAmbientPressure) > 0) {
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_AMBIENT_PRESSURE_EVENT, &data, sizeof(data), portMAX_DELAY));
            break;
          }  
          if(td5->getPid(&pidVehicleSpeed) > 0) {
            //vehicleSpeed = pidVehicleSpeed.getbValue(0);
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_PID_VEHICLE_SPEED_EVENT, &data, sizeof(data), portMAX_DELAY));
            break;
          }
          
          // shutdown in case of too many frames lost
					if (td5->getConsecutiveLostFrames() > TD5_COMM_MAX_LOSTFRAMES){
            td5->closeComm();
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_LOST_CONNECTION, &data, sizeof(data), portMAX_DELAY));
						ESP_UTILS_LOGI("[ERROR] Comm handler too many lost frames: close comunication ");
					}
        }
        if(!td5->ecuIsConnected()){
          commState = TD5_COMM_HANDLER_IDLE_STATE;  
        }
			  break;

      case TD5_COMM_HANDLER_READ_ECU_DATA_STATE:
        if(td5->ecuIsConnected()){
          if(td5->getEcuData() != -1) {
            data = TD5_COMM_COMMAND_OK;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_READ_ECU_DATA_EVENT, &data, sizeof(data), portMAX_DELAY));
          }          
          else {
            data = TD5_COMM_COMMAND_FAILED;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_READ_ECU_DATA_EVENT, &data, sizeof(data), portMAX_DELAY));
            ESP_UTILS_LOGI("[ERROR] Comm handler cannot read ECU data");
          }   
        }
        commState = TD5_COMM_HANDLER_IDLE_STATE;
        break;

      case TD5_COMM_HANDLER_READ_FAULT_CODES_STATE:
        if(td5->ecuIsConnected()) {
          int faultsNum = td5->getFaultCodes();      
          if(faultsNum != -1){
            data = faultsNum;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_READ_FAULT_CODES_EVENT, &data, sizeof(data), portMAX_DELAY));
          } 
          else {
            data = TD5_COMM_COMMAND_FAILED;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_READ_FAULT_CODES_EVENT, &data, sizeof(data), portMAX_DELAY));    
            ESP_UTILS_LOGI("[ERROR] Comm handler cannot read Fault codes");
          }
        }
        commState = TD5_COMM_HANDLER_IDLE_STATE;
        break;

      case TD5_COMM_HANDLER_READ_ECU_SETTINGS_STATE:
        if(td5->ecuIsConnected()) {
          if(td5->getEcuSettings() != -1){
            data = TD5_COMM_COMMAND_OK;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_READ_ECU_SETTINGS_EVENT, &data, sizeof(data), portMAX_DELAY));
          } 
          else {
            data = TD5_COMM_COMMAND_FAILED;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_READ_ECU_SETTINGS_EVENT, &data, sizeof(data), portMAX_DELAY));
            ESP_UTILS_LOGI("[ERROR] Comm handler cannot read ECU settings");
          }
        }
        commState = TD5_COMM_HANDLER_IDLE_STATE;
        break;

      case TD5_COMM_HANDLER_CLEAR_FAULT_CODES_STATE:
        if(td5->ecuIsConnected()) {
          int8_t response = td5->resetFaults();     
          if(response > 0){
            data = TD5_COMM_COMMAND_OK;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_CLEAR_FAULT_CODES_EVENT, &data, sizeof(data), portMAX_DELAY));
            commState = TD5_COMM_HANDLER_READ_FAULT_CODES_STATE;
            break;
          } 
          else {
            data = TD5_COMM_COMMAND_FAILED;
            ESP_ERROR_CHECK(esp_event_post_to(event_loop, UI_EVENTS, UI_CLEAR_FAULT_CODES_EVENT, &data, sizeof(data), portMAX_DELAY));
            ESP_UTILS_LOGI("[ERROR] Comm handler clear Fault codes failed");
          }
        }
        commState = TD5_COMM_HANDLER_IDLE_STATE;
        break;

		}

      vTaskDelay(pdMS_TO_TICKS(TD5_COMM_TASK_DELAY_MS * (commState == TD5_COMM_HANDLER_IDLE_STATE ? 100 : 1)));
    }
}

static void task_connect_to_ecu_request_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  ESP_UTILS_LOGI("Comm handler received UI_CONNECT_TO_ECU_REQUEST_EVENT event");
	if(commState == TD5_COMM_HANDLER_IDLE_STATE){
		commState = TD5_COMM_HANDLER_CONNECT_TO_ECU_STATE;
	}
}

static void task_start_instrument_mode_request_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  ESP_UTILS_LOGI("Comm handler received COMM_HANDLER_INSTRUMENT_STATE event");
	if(commState == TD5_COMM_HANDLER_IDLE_STATE){
		commState = TD5_COMM_HANDLER_INSTRUMENT_STATE;
	}
}

static void task_stop_instrument_mode_request_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  ESP_UTILS_LOGI("Comm handler received COMM_HANDLER_IDLE_STATE event");
	if(commState == TD5_COMM_HANDLER_INSTRUMENT_STATE){
		commState = TD5_COMM_HANDLER_IDLE_STATE;
	}
}

static void task_read_ecu_data_request_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  ESP_UTILS_LOGI("Comm handler received COMM_HANDLER_READ_ECU_DATA_STATE event");
	if(commState == TD5_COMM_HANDLER_IDLE_STATE){
		commState = TD5_COMM_HANDLER_READ_ECU_DATA_STATE;
	}
}

static void task_read_fault_codes_request_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  ESP_UTILS_LOGI("Comm handler received COMM_HANDLER_READ_FAULT_CODES_STATE event");
	if(commState == TD5_COMM_HANDLER_IDLE_STATE){
		commState = TD5_COMM_HANDLER_READ_FAULT_CODES_STATE;
	}
}

static void task_read_ecu_settings_request_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  ESP_UTILS_LOGI("Comm handler received COMM_HANDLER_READ_ECU_SETTINGS_STATE event");
	if(commState == TD5_COMM_HANDLER_IDLE_STATE){
		commState = TD5_COMM_HANDLER_READ_ECU_SETTINGS_STATE;
	}
}

static void task_clear_fault_codes_request_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
  int data = *((int*) event_data);
  ESP_UTILS_LOGI("Comm handler received COMM_HANDLER_CLEAR_FAULT_CODES_STATE event");
	if(commState == TD5_COMM_HANDLER_IDLE_STATE){
		commState = TD5_COMM_HANDLER_CLEAR_FAULT_CODES_STATE;
	}
}



bool td5CommInit()
{
  ESP_UTILS_LOGI("Initializing Td5 comm");

  ESP_UTILS_LOGI("Create Td5 comm task");
  BaseType_t core_id = (TD5_COMM_TASK_CORE < 0) ? tskNO_AFFINITY : TD5_COMM_TASK_CORE;
  BaseType_t ret = xTaskCreatePinnedToCore(td5CommTask, "td5comm", TD5_COMM_TASK_STACK_SIZE, NULL,
                     TD5_COMM_TASK_PRIORITY, &td5CommHandle, core_id);
  ESP_UTILS_CHECK_FALSE_RETURN(ret == pdPASS, false, "Create Td5 comm task failed");

  ESP_UTILS_LOGI("Register comm event handlers");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, COMM_EVENTS, COMM_CONNECT_TO_ECU_REQUEST_EVENT, task_connect_to_ecu_request_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_connect_to_ecu_request_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, COMM_EVENTS, COMM_START_INSTRUMENT_MODE_REQUEST_EVENT, task_start_instrument_mode_request_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_start_instrument_mode_request_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, COMM_EVENTS, COMM_STOP_INSTRUMENT_MODE_REQUEST_EVENT, task_stop_instrument_mode_request_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_stop_instrument_mode_request_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, COMM_EVENTS, COMM_READ_ECU_DATA_REQUEST_EVENT, task_read_ecu_data_request_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_read_ecu_data_request_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, COMM_EVENTS, COMM_READ_FAULT_CODES_REQUEST_EVENT, task_read_fault_codes_request_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_read_fault_codes_request_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, COMM_EVENTS, COMM_READ_ECU_SETTINGS_REQUEST_EVENT, task_read_ecu_settings_request_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_read_ecu_settings_request_event_handler failed");
  ESP_UTILS_CHECK_ERROR_RETURN(esp_event_handler_instance_register_with(event_loop, COMM_EVENTS, COMM_CLEAR_FAULT_CODES_REQUEST_EVENT, task_clear_fault_codes_request_event_handler, event_loop, NULL), false, "esp_event_handler_instance_register_with of task_clear_fault_codes_request_event_handler failed");
	
	commState = TD5_COMM_HANDLER_IDLE_STATE;
    
	return true;
}


