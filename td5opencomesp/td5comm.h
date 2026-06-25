#pragma once

#include <Arduino.h>

#define Td5RequestByteDelay       2
#define TD5_REQUEST_DELAY         55
#define READ_ATTEMPTS             7
#define INIT_FIRST_DELAY_TIME     300

#define MAX_FAULT_CODES           35

#define MAX_FAULT_CODE            280
#define KEEP_ALIVE_TIME           4500

#define INIT_FRAME          0x00
#define START_DIAG          0x01
#define REQ_SEED            0x02
#define SEND_KEY            0x03
#define START_FUELLING      0x04
#define ENGINE_RPM          0x05
#define TEMPERATURES        0x06
#define INLET_PRES_MAF      0x07
#define BATTERY_VOLT        0x08
#define AMBIENT_PRES        0x09
#define VEHICLE_SPEED       0x0A
#define THROTTLE_POS        0x0B
#define IO_CONTROL          0x0C
#define INJ_BALANCE         0x0D
#define RPM_ERROR           0x0E
#define EGR_MOD             0x0F
#define ILT_MOD             0x10
#define TWG_MOD             0x11
#define KEEP_ALIVE          0x12
#define FAULT_CODES         0x13
#define CLEAR_FAULTS        0x14
#define VIN_DATE_NNW_HOMOL  0x15
#define ECU_MODEL           0x16
#define VEHICLE_FUEL_MAP    0x17
#define ECU_STATUS          0x18
#define INJ_CODES           0x19
#define ECU_CONFIG          0x1A
#define LIVE_FUELLING       0x1B
#define TESTING_AC_CLUTCH   0x1C
#define TESTING_AC_FAN      0x1D
#define TESTING_MIL_LAMP    0x1E
#define TESTING_FUEL_PUMP   0x1F
#define TESTING_GLOW_PLUGS  0x20
#define TESTING_PLS_REV_CNT 0x21
#define TESTING_TWG         0x22
#define TESTING_TEMP_GAUGE  0x23
#define TESTING_EGR_ILT     0x24
#define TESTING_INJECTOR_1  0x25
#define TESTING_INJECTOR_2  0x26
#define TESTING_INJECTOR_3  0x27
#define TESTING_INJECTOR_4  0x28
#define TESTING_INJECTOR_5  0x29
#define INPUTS_STATE        0x2A
#define READ_MAP            0x2B
#define ECU_RELATED_1       0x2C
#define ECU_RELATED_2       0x2D

#define NULL_PID            0xFF

#define COMSTAT_INIT          0
#define COMSTAT_IDLE          1
#define COMSTAT_START_COMM    2
#define COMSTAT_SEND_FRAME    3
#define COMSTAT_RECV_FRAME    4
#define COMSTAT_DATA_READY    5
#define COMSTAT_ERROR         6

#define RDMAPSTAT_IDLE        0
#define RDMAPSTAT_INIT        1
#define RDMAPSTAT_RDFW        2
#define RDMAPSTAT_RDTB        3
#define RDMAPSTAT_END         4
#define RDMAPSTAT_ERROR       -1

// Declare IO
#define obdSerial            Serial1

class Td5Pid;

// Class Td5Comm
class Td5Comm
{
public:
  Td5Comm();
  void init();
  void initComm();
  void closeComm();
  int8_t getPid(Td5Pid* pid);
  boolean ecuIsConnected();
  boolean newDataIsAvailable();
  unsigned long getLastReceivedPidTime();
  unsigned long getLastReceivedPidElapsedTime();
  void setInitStep(byte init_step);
  byte getInitStep();
  int getLostFrames();
  int getConsecutiveLostFrames();
  void setReadMapStep(byte read_map_step){readmapStep = read_map_step;};
  byte getReadMapStep(){return readmapStep;};
  uint32_t getMapAddress(){return mapAddress;};

  void resetInitTime(){initTime = 0;};

// easy read calls
  int getFaultCodes();
  int getFaultCode(int index);
  int getFaultCodesCount(){return faultCodesCount;};
  int8_t resetFaults();
  int getEcuData();
  int getEcuSettings();
  uint8_t readMap();
  uint16_t getEngineRPM();
  float getTurboP();
  float getMAF();
  float getCurrentInj();
  float getDriverDemand();  
  float getSmokeLim();
  float getTorqueLim();
  float getCoolantT();
  float getExtT();
  float getFuelT();
  float getBattery();
  float getAmbientP();
  int16_t getVehicleSpeed();

private:  
  byte checksum(byte *data, byte len);
  boolean read_byte(byte * b);
  void write_byte(byte b);

protected:
  unsigned long lastReceivedPidTime;
  unsigned long initTime;
  byte initStep;  // Init is multistage, this is the counter
  byte readmapStep;  // Read Map is multistage, this is the counter
  boolean ecuConnection;  // Have we connected to the ECU or not
  boolean newDataAvailable;
  int lostFrames;
  int8_t consLostFrames;
  int faultCodesCount;
  uint32_t mapAddress;
  int8_t Td5RequestDelay;
  
// comm data
  struct
  {
    byte *req_buf;
    byte *resp_buf;
    byte status;
    byte index;
    byte active_pid;
  } com;
};

enum {
  TD5_COMM_INIT_SETUP,
  TD5_COMM_INIT_DRIVE_K_LINE_HIGH_300MS,
	TD5_COMM_INIT_DRIVE_K_LINE_LOW_25MS,
	TD5_COMM_INIT_DRIVE_K_LINE_HIGH_25MS,
	TD5_COMM_INIT_SEND_INIT_FRAME,
	TD5_COMM_INIT_SEND_START_DIAG,
	TD5_COMM_INIT_SEND_REQUEST_SEED,
	TD5_COMM_INIT_SEND_KEY,
};

extern Td5Comm* td5;


// Class Td5Pid
class Td5Pid
{
public:
  Td5Pid(byte ID, byte reqlen, byte resplen, long cycletime = 0);
  byte getRequestLen(){return ( requestFrame[0] + 2); };
  byte getResponseLen(){return ( responseFrame[0] + 2); };  
  void setCycleTime(long time){ cycleTime = time; };
  boolean getValue(float *fvalue, byte index = 0);
  boolean getValue(uint16_t *value, byte index = 0);
  boolean getValue(int *value, byte index = 0);
  float getfValue(byte index = 0);
  boolean isConsistent(){return (lastSeenTime != 0);};
  uint16_t getulValue(byte index = 0);
  int16_t getlValue(byte index = 0);
  byte getbValue(byte index = 0);
  
  void setRequestByte(byte value, byte pos);
  byte getResponseByte(byte pos);

  byte *requestFrame;
  byte *responseFrame;
  
  byte id;
  long cycleTime;
  long lastSeenTime;
  byte responseLength;  
};

// Generic functions
/*
void debug_log_byte(byte b);
void debug_log_frame(byte *datasent, byte sentlen, byte *datarecv, byte recvlen);
void debug_log_half_frame(byte *data, byte len);
*/
void compute_keys(uint8_t *seed, uint8_t *key);
void init_static_variables();
int16_t limit_value(int16_t min, int16_t val, int16_t max);
float limit_value(float min, float val, float max);

uint8_t td5_get_next_fault_code();
uint8_t td5_get_previous_fault_code();

// Data extraction functions
String pid_extract_injector_code(byte injector);
String pid_extract_ecu_vin();
void pid_extract_ecu_vin(char* buffer);
void pid_extract_ecu_prod_date(char* buffer);
void pid_extract_ecu_nnw(char* buffer);
void pid_extract_ecu_homol(char* buffer);
void pid_extract_ecu_model(char* buffer);
void pid_extract_ecu_firmware(char* buffer);
void pid_extract_ecu_status(char* buffer);
boolean pid_extract_ecu_configuration(byte bit_index);
boolean pid_extract_ecu_input_state(byte bit_index);

// Declare pids
extern Td5Pid pidInitFrame;
extern Td5Pid pidStartDiag;
extern Td5Pid pidRequestSeed;
extern Td5Pid pidSendKey;
extern Td5Pid pidRPM;
extern Td5Pid pidTurboPressureMaf;
extern Td5Pid pidTemperatures;
extern Td5Pid pidBatteryVoltage;
extern Td5Pid pidAmbientPressure;
extern Td5Pid pidStartFuelling;
extern Td5Pid pidKeepAlive;
extern Td5Pid pidFaultCodes;
extern Td5Pid pidResetFaults;
extern Td5Pid pidInjectorsBalance;
extern Td5Pid pidVehicleSpeed;
extern Td5Pid pidThrottlePosition;
extern Td5Pid pidRPMError;
extern Td5Pid pidEGR;
extern Td5Pid pidILT;
extern Td5Pid pidTWG;
extern Td5Pid pidVinDateNnwHomol;
extern Td5Pid pidEcuModel;
extern Td5Pid pidVehicleFuelMap;
extern Td5Pid pidEcuStatus;
extern Td5Pid pidInjCodes;
extern Td5Pid pidEcuConfiguration;
extern Td5Pid pidLiveFuelling;
extern Td5Pid pidTestingACClutch;
extern Td5Pid pidTestingACFan;
extern Td5Pid pidTestingMILLamp;
extern Td5Pid pidTestingFuelPump;
extern Td5Pid pidTestingGlowPlugs;
extern Td5Pid pidTestingPlsRevCnt;
extern Td5Pid pidTestingTWG;
extern Td5Pid pidTestingTempGauge;
extern Td5Pid pidTestingEGRILT;
extern Td5Pid pidTestingInj1;
extern Td5Pid pidTestingInj2;
extern Td5Pid pidTestingInj3;
extern Td5Pid pidTestingInj4;
extern Td5Pid pidTestingInj5;
extern Td5Pid pidInputsState;
extern Td5Pid pidReadMap;
extern Td5Pid pidEcuRelated1;
extern Td5Pid pidEcuRelated2;

#define PID_READY            1
#define PID_NOT_READY        0
#define PID_LOST_FRAME      -1
#define PID_NEGATIVE_ANSWER -2

extern const char *injector_codes[];
extern const char *injector_codes_last[];
extern const char *fault_code[];

// limits, warnings and alarms constants
#define LIM_MIN_ENGINE_RPM      0
#define LIM_MAX_ENGINE_RPM      5000
#define WRN_MIN_ENGINE_RPM      3500
#define WRN_MAX_ENGINE_RPM      4000
#define ALM_MIN_ENGINE_RPM      4000
#define ALM_MAX_ENGINE_RPM      5001

#define LIM_MIN_TURBO_P         0.0
#define LIM_MAX_TURBO_P         1.5
#define WRN_MIN_TURBO_P         1.0
#define WRN_MAX_TURBO_P         1.10
#define ALM_MIN_TURBO_P         1.10
#define ALM_MAX_TURBO_P         1.51

#define LIM_MIN_MAF             0.0
#define LIM_MAX_MAF             700.0
#define WRN_MIN_MAF             701.0  // disabled
#define WRN_MAX_MAF             701.0  // disabled
#define ALM_MIN_MAF             701.0  // disabled
#define ALM_MAX_MAF             701.0  // disabled

#define LIM_MIN_INJ            -10.0
#define LIM_MAX_INJ             70.0
#define WRN_MIN_INJ             71.0  // disabled
#define WRN_MAX_INJ             71.0  // disabled
#define ALM_MIN_INJ             71.0  // disabled
#define ALM_MAX_INJ             71.0  // disabled

#define LIM_MIN_COOLANT_T       -20.0
#define LIM_MAX_COOLANT_T       130.0
#define WRN_MIN_COOLANT_T       100.0
#define WRN_MAX_COOLANT_T       120.0
#define ALM_MIN_COOLANT_T       120.0
#define ALM_MAX_COOLANT_T       131.0

#define LIM_MIN_EXT_T           -20.0
#define LIM_MAX_EXT_T           130.0
#define WRN_MIN_EXT_T           131.0  // disabled
#define WRN_MAX_EXT_T           131.0  // disabled
#define ALM_MIN_EXT_T           131.0  // disabled
#define ALM_MAX_EXT_T           131.0  // disabled

#define LIM_MIN_FUEL_T          -20.0
#define LIM_MAX_FUEL_T          130.0
#define WRN_MIN_FUEL_T           80.0
#define WRN_MAX_FUEL_T          100.0
#define ALM_MIN_FUEL_T          100.0
#define ALM_MAX_FUEL_T          131.0

#define LIM_MIN_BATTERY         0.0
#define LIM_MAX_BATTERY         20.0
#define WRN_MIN_BATTERY         14.5
#define WRN_MAX_BATTERY         21.0
#define ALM_MIN_BATTERY         0.0
#define ALM_MAX_BATTERY         11.0

#define LIM_MIN_AMBIENT_P       0.0
#define LIM_MAX_AMBIENT_P       1.5
#define WRN_MIN_AMBIENT_P       1.51  // disabled
#define WRN_MAX_AMBIENT_P       1.51  // disabled
#define ALM_MIN_AMBIENT_P       1.51  // disabled
#define ALM_MAX_AMBIENT_P       1.51  // disabled

#define LIM_MIN_VEHICLE_SPD     0
#define LIM_MAX_VEHICLE_SPD     150
#define WRN_MIN_VEHICLE_SPD     151  // disabled
#define WRN_MAX_VEHICLE_SPD     151  // disabled
#define ALM_MIN_VEHICLE_SPD     151  // disabled
#define ALM_MAX_VEHICLE_SPD     151  // disabled

#define LIM_MIN_FUEL_CONS       1.0
#define LIM_MAX_FUEL_CONS       30.0
#define WRN_MIN_FUEL_CONS       5.0
#define WRN_MAX_FUEL_CONS       9.0
#define ALM_MIN_FUEL_CONS       1.0
#define ALM_MAX_FUEL_CONS       5.0
