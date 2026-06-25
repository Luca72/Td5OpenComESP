#include <Arduino.h>
#include "td5comm.h"

#include "td5opencomesp.h"

const byte pid_0x00[] = { 0x81, 0x13, 0xF7, 0x81, 0x0C };        // INIT_FRAME
const byte pid_0x01[] = { 0x02, 0x10, 0xA0, 0x00 };              // START_DIAG
const byte pid_0x02[] = { 0x02, 0x27, 0x01, 0x00 };              // REQ_SEED
const byte pid_0x03[] = { 0x04, 0x27, 0x02, 0x00, 0x00, 0x00 };  // SEND_KEY
const byte pid_0x04[] = { 0x02, 0x21, 0x20, 0x00 };              // START_FUELLING
const byte pid_0x05[] = { 0x02, 0x21, 0x09, 0x00 };              // ENGINE_RPM
const byte pid_0x06[] = { 0x02, 0x21, 0x1A, 0x00 };              // TEMPERATURES
const byte pid_0x07[] = { 0x02, 0x21, 0x1C, 0x00 };              // INLET_PRES_MAF
const byte pid_0x08[] = { 0x02, 0x21, 0x10, 0x00 };              // BATTERY_VOLT
const byte pid_0x09[] = { 0x02, 0x21, 0x23, 0x00 };              // AMBIENT_PRES
const byte pid_0x0A[] = { 0x02, 0x21, 0x0D, 0x00 };              // VEHICLE_SPEED
const byte pid_0x0B[] = { 0x02, 0x21, 0x1B, 0x00 };              // THROTTLE_POS
const byte pid_0x0C[] = { 0x03, 0x30, 0xC0, 0xF0, 0x00 };        // IO_CONTROL
const byte pid_0x0D[] = { 0x02, 0x21, 0x40, 0x00 };              // INJ_BALANCE
const byte pid_0x0E[] = { 0x02, 0x21, 0x21, 0x00 };              // RPM_ERROR
const byte pid_0x0F[] = { 0x02, 0x21, 0x37, 0x00 };              // EGR_MOD
const byte pid_0x10[] = { 0x02, 0x21, 0x45, 0x00 };              // ILT_MOD
const byte pid_0x11[] = { 0x02, 0x21, 0x38, 0x00 };              // TWG_MOD
const byte pid_0x12[] = { 0x02, 0x3E, 0x01, 0x00 };              // KEEP_ALIVE
const byte pid_0x13[] = { 0x02, 0x21, 0x3B, 0x00 };              // FAULT_CODES
const byte pid_0x14[] = { 0x14, 0x31, 0xDD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // CLEAR_FAULTS
const byte pid_0x15[] = { 0x02, 0x1A, 0x87, 0x00 };              // VIN_DATE_NNW_HOMOL
const byte pid_0x16[] = { 0x02, 0x1A, 0x9A, 0x00 };              // ECU_MODEL
const byte pid_0x17[] = { 0x02, 0x21, 0x32, 0x00 };              // VEHICLE_FUEL_MAP
const byte pid_0x18[] = { 0x02, 0x21, 0x24, 0x00 };              // ECU_STATUS
const byte pid_0x19[] = { 0x02, 0x21, 0x3D, 0x00 };              // INJ_CODES
const byte pid_0x1A[] = { 0x02, 0x21, 0x20, 0x00 };              // ECU_CONFIG
const byte pid_0x1B[] = { 0x02, 0x21, 0x1D, 0x00 };              // LIVE_FUELLING
const byte pid_0x1C[] = { 0x03, 0x30, 0xA3, 0xFF, 0x00 };        // TESTING AC CLUTCH
const byte pid_0x1D[] = { 0x03, 0x30, 0xA4, 0xFF, 0x00 };        // TESTING AC FAN
const byte pid_0x1E[] = { 0x03, 0x30, 0xA2, 0xFF, 0x00 };        // TESTING MIL LAMP
const byte pid_0x1F[] = { 0x03, 0x30, 0xA1, 0xFF, 0x00 };        // TESTING FUEL PUMP
const byte pid_0x20[] = { 0x03, 0x30, 0xB3, 0xFF, 0x00 };        // TESTING GLOW PLUGS
const byte pid_0x21[] = { 0x03, 0x30, 0xB7, 0xFF, 0x00 };        // TESTING PULSE REV COUNTER
const byte pid_0x22[] = { 0x07, 0x30, 0xBE, 0xFF, 0x00, 0x0A, 0x13, 0x88, 0x00 }; // TESTING TWG
const byte pid_0x23[] = { 0x03, 0x30, 0xBA, 0xFF, 0x00 };        // TESTING TEMPERATURE GAUGE
const byte pid_0x24[] = { 0x07, 0x30, 0xBD, 0xFF, 0x00, 0xFA, 0x13, 0x88, 0x00 };  // TESTING EGR ILT
const byte pid_0x25[] = { 0x03, 0x31, 0xC2, 0x01, 0x00 };        // TESTING INJECTOR 1
const byte pid_0x26[] = { 0x03, 0x31, 0xC2, 0x02, 0x00 };        // TESTING INJECTOR 2
const byte pid_0x27[] = { 0x03, 0x31, 0xC2, 0x03, 0x00 };        // TESTING INJECTOR 3
const byte pid_0x28[] = { 0x03, 0x31, 0xC2, 0x04, 0x00 };        // TESTING INJECTOR 4
const byte pid_0x29[] = { 0x03, 0x31, 0xC2, 0x05, 0x00 };        // TESTING INJECTOR 5
const byte pid_0x2A[] = { 0x02, 0x21, 0x1E, 0x00 };              // INPUTS STATE
const byte pid_0x2B[] = { 0x05, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00 }; // READ MAP
const byte pid_0x2C[] = { 0x02, 0x1A, 0x9B, 0x00 };              // ECU_RELATED_1
const byte pid_0x2D[] = { 0x02, 0x1A, 0x9C, 0x00 };              // ECU_RELATED_2

const byte *td5_pids[] = 
{ 
  pid_0x00, pid_0x01, pid_0x02, pid_0x03, pid_0x04, pid_0x05, pid_0x06, pid_0x07,
  pid_0x08, pid_0x09, pid_0x0A, pid_0x0B, pid_0x0C, pid_0x0D, pid_0x0E, pid_0x0F,
  pid_0x10, pid_0x11, pid_0x12, pid_0x13, pid_0x14, pid_0x15, pid_0x16, pid_0x17,
  pid_0x18, pid_0x19, pid_0x1A, pid_0x1B, pid_0x1C, pid_0x1D, pid_0x1E, pid_0x1F,
  pid_0x20, pid_0x21, pid_0x22, pid_0x23, pid_0x24, pid_0x25, pid_0x26, pid_0x27,
  pid_0x28, pid_0x29, pid_0x2A, pid_0x2B, pid_0x2C, pid_0x2D    
};

Td5Pid pidInitFrame(INIT_FRAME, 5, 5);
Td5Pid pidStartDiag(START_DIAG, 4, 3);
Td5Pid pidRequestSeed(REQ_SEED, 4, 6);
Td5Pid pidSendKey(SEND_KEY, 6, 4);
Td5Pid pidRPM(ENGINE_RPM, 4, 6, 250);
Td5Pid pidTurboPressureMaf(INLET_PRES_MAF, 4, 12, 300);
Td5Pid pidTemperatures(TEMPERATURES, 4, 20, 1000);
Td5Pid pidBatteryVoltage(BATTERY_VOLT, 4, 8, 1500);
Td5Pid pidAmbientPressure(AMBIENT_PRES, 4, 8, 2000);
Td5Pid pidStartFuelling(START_FUELLING, 4, 8);
Td5Pid pidKeepAlive(KEEP_ALIVE, 4, 3, KEEP_ALIVE_TIME);
Td5Pid pidFaultCodes(FAULT_CODES, 4, 39);
Td5Pid pidResetFaults(CLEAR_FAULTS, 22, 4);
Td5Pid pidInjectorsBalance(INJ_BALANCE, 4, 14, 500);
Td5Pid pidVehicleSpeed(VEHICLE_SPEED, 4, 5, 750);
Td5Pid pidThrottlePosition(THROTTLE_POS, 4, 14, 500);
Td5Pid pidRPMError(RPM_ERROR, 4, 6, 500);
Td5Pid pidEGR(EGR_MOD, 4, 6, 1000);
Td5Pid pidILT(ILT_MOD, 4, 6, 1000);
Td5Pid pidTWG(TWG_MOD, 4, 6, 1000);
Td5Pid pidVinDateNnwHomol(VIN_DATE_NNW_HOMOL, 4, 50);
Td5Pid pidEcuModel(ECU_MODEL, 4, 10);
Td5Pid pidVehicleFuelMap(VEHICLE_FUEL_MAP, 4, 28);
Td5Pid pidEcuStatus(ECU_STATUS, 4, 5);
Td5Pid pidInjCodes(INJ_CODES, 4, 22);
Td5Pid pidEcuConfiguration(ECU_CONFIG, 4, 8);
Td5Pid pidLiveFuelling(LIVE_FUELLING, 4, 22, 500);
Td5Pid pidTestingACClutch(TESTING_AC_CLUTCH, 5, 4);
Td5Pid pidTestingACFan(TESTING_AC_FAN, 5, 4);
Td5Pid pidTestingMILLamp(TESTING_MIL_LAMP, 5, 4);
Td5Pid pidTestingFuelPump(TESTING_FUEL_PUMP, 5, 4);
Td5Pid pidTestingGlowPlugs(TESTING_GLOW_PLUGS, 5, 4);
Td5Pid pidTestingPlsRevCnt(TESTING_PLS_REV_CNT, 5, 4);
Td5Pid pidTestingTWG(TESTING_TWG, 9, 4);
Td5Pid pidTestingTempGauge(TESTING_TEMP_GAUGE, 5, 4);
Td5Pid pidTestingEGRILT(TESTING_EGR_ILT, 9, 4);
Td5Pid pidTestingInj1(TESTING_INJECTOR_1, 5, 4);
Td5Pid pidTestingInj2(TESTING_INJECTOR_2, 5, 4);
Td5Pid pidTestingInj3(TESTING_INJECTOR_3, 5, 4);
Td5Pid pidTestingInj4(TESTING_INJECTOR_4, 5, 4);
Td5Pid pidTestingInj5(TESTING_INJECTOR_5, 5, 4);
Td5Pid pidInputsState(INPUTS_STATE, 4, 6, 500);
Td5Pid pidReadMap(READ_MAP, 7, 67);
Td5Pid pidEcuRelated1(ECU_RELATED_1, 4, 5);
Td5Pid pidEcuRelated2(ECU_RELATED_2, 4, 5);

///////////////////////////////////////////////////
//              Generic functions                //
///////////////////////////////////////////////////

void serial_on()
{
  obdSerial.begin(10400, SERIAL_8N1, K_IN, K_OUT);
}

void serial_off()
{
  obdSerial.end();
  pinMode(K_OUT, OUTPUT); // GPIO20
  pinMode(K_IN, INPUT);   // GPIO19
}


void compute_keys(uint8_t *seed, uint8_t *key)
{
  uint16_t seed16;
  uint16_t seed16_tmp = 0;
  uint8_t count = 0;
  uint8_t idx;
  uint8_t fib_tap = 0;

  seed16 = seed[0];
  seed16 <<= 8;
  seed16 += seed[1];
  
  count = ((seed16 >> 0xC & 0x8) | (seed16 >> 0x5 & 0x4) | (seed16 >> 0x3 & 0x2) | (seed16 & 0x1)) + 1;

  for (idx = 1; idx <= count; idx++) 
  {
    fib_tap = ((seed16 >> 1 ) ^ (seed16 >> 2 ) ^ (seed16 >> 8 ) ^ (seed16 >> 9 )) & 1;

    seed16_tmp = ((seed16 >> 1) | (fib_tap << 0xF));

    if ((seed16 >> 0x3 & 1) && (seed16 >> 0xD & 1))
    {
      seed16 =  seed16_tmp & ~1;  // clear lsb
    } 
    else 
    {
      seed16 = seed16_tmp | 1;   // set lsb
    }
  }

  key[1] = seed16;
  seed16 >>= 8;
  key[0] = seed16;   
}

int16_t limit_value(int16_t min, int16_t val, int16_t max)
{
  if(val < min)
    val = min;
  if(val > max)
    val = max;
  return val;
}

float limit_value(float min, float val, float max)
{
  if(val < min)
    val = min;
  if(val > max)
    val = max;
  return val;  
}


///////////////////////////////////////////////////
//                 Class Td5Comm                 //
///////////////////////////////////////////////////
Td5Comm::Td5Comm()
{
  lastReceivedPidTime = 0;
  initStep = 0;
  readmapStep = 0;
  mapAddress = 0x000000;
  lostFrames = 0;
  consLostFrames = 0;
  initTime = 0;
  ecuConnection = false;
  newDataAvailable = false;
  com.status = COMSTAT_INIT;
  Td5RequestDelay = TD5_REQUEST_DELAY;
}


void Td5Comm::init()
{
  // init pinouts
  pinMode(K_OUT, OUTPUT); // GPIO20
  pinMode(K_IN, INPUT);   // GPIO19
}

boolean Td5Comm::read_byte(byte * b)
{
  int readData;
  boolean success = true;
  byte t=0;

  while(t != READ_ATTEMPTS  && (readData=obdSerial.read())==-1) 
  {
    delay(1);
    t++;
  }

  if (t >= READ_ATTEMPTS) 
  {
    success = false;
  }

  if (success)
  {
    *b = (byte) readData;
  }

  return success;
}

void Td5Comm::write_byte(byte b)
{
  obdSerial.write(b);
  delay(Td5RequestByteDelay);  // ISO requires 5-20 ms delay between bytes.
  obdSerial.read(); 
}

int8_t Td5Comm::getPid(Td5Pid* pid)
{
  boolean gotData = false;
  byte responseIndex = 0;
  byte dataCaught = '\0';  
  byte dataLen = 0;
  
  if (pid->id != INIT_FRAME)
    dataLen = pid->requestFrame[0] + 2;
  else
    dataLen = 5;

  long currentTime = millis();

  if ((currentTime >= (lastReceivedPidTime + Td5RequestDelay)) && (currentTime >= (pid->lastSeenTime + pid->cycleTime)))
  {
    // Send the message
    pid->requestFrame[dataLen-1] = checksum(pid->requestFrame, dataLen-1);

    for (byte i = 0; i < dataLen; i++)
    {
      write_byte(pid->requestFrame[i]);
    }
    
    // Wait for response for 300 ms
    long waitResponseTime = currentTime + 300;
    do
    {
      // If we find any data, keep catching it until it ends
      while(read_byte(&dataCaught) && (responseIndex < pid->responseLength))
      {
        gotData = true;
        pid->responseFrame[responseIndex] = dataCaught;
        if(!((dataCaught ==0x00) && (responseIndex == 0)))
        {
          responseIndex++;
        }
      }
    } 
    while (millis() <= waitResponseTime && !gotData);

    if (gotData && (responseIndex > 1))
    {
      lastReceivedPidTime = millis();
      pid->lastSeenTime = lastReceivedPidTime;
      consLostFrames = 0;

      if(checksum(pid->responseFrame, responseIndex-1) == pid->responseFrame[responseIndex-1])      
      {
        if(pid->responseFrame[1] != 0x7F) 
        {
          return responseIndex;
        }
        else
        {
          return PID_NEGATIVE_ANSWER; // negative response
        }
      }
    }
  }
  else 
  {
    return PID_NOT_READY;
  }

  lostFrames += 1;
  consLostFrames += 1;
  return PID_LOST_FRAME;      
}


boolean Td5Comm::ecuIsConnected()
{
  return ecuConnection; 
}

boolean Td5Comm::newDataIsAvailable()
{
  if(newDataAvailable)
  {
    newDataAvailable = false;
    return true;
  }
  else
    return false;
}

unsigned long Td5Comm::getLastReceivedPidTime()
{
  return lastReceivedPidTime;
}

unsigned long Td5Comm::getLastReceivedPidElapsedTime()
{
  return (millis() - lastReceivedPidTime);
}

int Td5Comm::getLostFrames()
{
  return lostFrames;
};

int Td5Comm::getConsecutiveLostFrames()
{
  return consLostFrames;
};

byte Td5Comm::checksum(byte *data, byte len)
{
  byte crc=0;
  for(byte i=0; i<len; i++)
    crc=crc+data[i];
  return crc;
}

void Td5Comm::setInitStep(byte init_step)
{
  initStep = init_step;  
}

byte Td5Comm::getInitStep()
{
  return initStep;  
}

void Td5Comm::initComm()
{
  unsigned long currentTime = millis();
  
  switch (initStep)
  {
  case TD5_COMM_INIT_SETUP:
    ecuConnection = false;
    serial_off();
    initTime = currentTime + 300;
    initStep++;
    break;
	
  case TD5_COMM_INIT_DRIVE_K_LINE_HIGH_300MS:
    if (currentTime >= initTime)
    {
      digitalWrite(K_OUT, HIGH);
      initTime = currentTime + 300;
      initStep++;
    }
    break;
	
  case TD5_COMM_INIT_DRIVE_K_LINE_LOW_25MS:
  case TD5_COMM_INIT_DRIVE_K_LINE_HIGH_25MS:
    if (currentTime >= initTime)
    {
      digitalWrite(K_OUT, (initStep == 2 ? LOW : HIGH));
      initTime = currentTime + (initStep == 2 ? 25 : 25);
      initStep++;
    }
    break;
	
  case TD5_COMM_INIT_SEND_INIT_FRAME:
    if (currentTime >= initTime)
    {
      // switch now to 10400 bauds
      serial_on();
      
      // bit banging done, now verify connection at 10400 baud
      if (getPid(&pidInitFrame) <= 0)
      {
        initStep = 0;
        break;
      }

      lastReceivedPidTime = currentTime;
      initTime = currentTime + Td5RequestDelay;
      initStep++;       
    }
    break;
	
  case TD5_COMM_INIT_SEND_START_DIAG:
    if (currentTime >= initTime)
    {
      if (getPid(&pidStartDiag) <= 0)
      {
        initStep = 0;
        break;
      }

      lastReceivedPidTime = currentTime;
      initTime = currentTime + Td5RequestDelay;
      initStep++;       
    }
    break;
	
  case TD5_COMM_INIT_SEND_REQUEST_SEED:
    if (currentTime >= initTime)
    {
      if (getPid(&pidRequestSeed) <= 0)
      {
        initStep = 0;
        break;
      }
      
      lastReceivedPidTime = currentTime;
      initTime = currentTime + Td5RequestDelay;
      initStep++;       
    }
    break;
	
  case TD5_COMM_INIT_SEND_KEY:
    if (currentTime >= initTime)
    {
      uint8_t seed[2], key[2];
      seed[0] = pidRequestSeed.getResponseByte(3);
      seed[1] = pidRequestSeed.getResponseByte(4);
      compute_keys(seed, key);
      pidSendKey.setRequestByte(key[0], 3);
      pidSendKey.setRequestByte(key[1], 4);
      
      if (getPid(&pidSendKey) <= 0)
      {
        initStep = 0;
        break;
      }

      lastReceivedPidTime = currentTime;
      initTime = currentTime + Td5RequestDelay;

      delay(55);
      ecuConnection = true;
      initStep = 0;       
    }
    break;
	
  }
}

void Td5Comm::closeComm()
{
  obdSerial.end();
  lostFrames = 0;
  consLostFrames = 0;
  ecuConnection = false; 
}


int Td5Comm::getFaultCodes()
{
  faultCodesCount = 0;
  delay(Td5RequestDelay);
  if(getPid(&pidFaultCodes) > 0)
  {
    for(int i=0;i<MAX_FAULT_CODES;i++)
    {
      byte fault_code_byte = 0;
      fault_code_byte = pidFaultCodes.getResponseByte(i+3);
      for(int j=0;j<=7;j++)
      {
        if(bitRead(fault_code_byte,j))
        {
          faultCodesCount += 1;
        }        
      }
    }
    return faultCodesCount;
  }
  return -1;
}

int Td5Comm::getFaultCode(int index)
{
  int fault_codes_cnt = 0;
  byte fault_code_byte = 0;

  if(index < MAX_FAULT_CODE)
  {
    for(int i=0;i<MAX_FAULT_CODES;i++)
    {
      fault_code_byte = pidFaultCodes.getResponseByte(i+3);
      for(int j=0;j<=7;j++)
      {
        if(bitRead(fault_code_byte,j))
        {
          if(fault_codes_cnt == index)
          {
            return ((i*8)+j);   
          }
          else
          {
            fault_codes_cnt += 1;
          }
        }        
      }
    }
  }
  return -1;  
}

int8_t Td5Comm::resetFaults()
{
  delay(Td5RequestDelay);  
  return getPid(&pidResetFaults);
}


int Td5Comm::getEcuSettings()
{
  delay(Td5RequestDelay);
  if(getPid(&pidInjCodes) <= 0)
  {
    return -1;
  }

  delay(Td5RequestDelay);
  if(getPid(&pidEcuConfiguration) <= 0)
  {
    return -1;
  }

  return 1;
}

int Td5Comm::getEcuData()
{
  delay(Td5RequestDelay);
  if(getPid(&pidVinDateNnwHomol) <= 0)
  {
    return -1;
  }
  
  delay(Td5RequestDelay);
  if(getPid(&pidEcuModel) <= 0)
  {
    return -1;
  }
  
  delay(Td5RequestDelay);
  if(getPid(&pidVehicleFuelMap) <= 0)
  {
    return -1;
  }

  delay(Td5RequestDelay);
  if(getPid(&pidEcuStatus) <= 0)
  {
    return -1;
  }
  
  delay(Td5RequestDelay);
  if(getPid(&pidEcuRelated1) <= 0)
  {
    return -1;
  }

  delay(Td5RequestDelay);
  if(getPid(&pidEcuRelated2) <= 0)
  {
    return -1;
  }

  return 1;
}



uint8_t Td5Comm::readMap()
{
  static byte readBytes = 0;
  uint8_t dataRead = 0;
  int8_t pidResponse = 0;
  
  switch (readmapStep)
  {
    case RDMAPSTAT_INIT:
      Td5RequestDelay = 20; // temp. fasten reading
      mapAddress = 0x110000; 
      readmapStep++;
      readBytes=0x40;
      break;
    
    case RDMAPSTAT_RDFW:
      //delay(Td5RequestDelay);
      if(mapAddress > 0x128FC0)
      {
        dataRead = 0x07; 
        mapAddress = 0x13C000;
        readmapStep++;
        break;
      }
      pidReadMap.setRequestByte((byte)((mapAddress>>16)&0xFF), 2);
      pidReadMap.setRequestByte((byte)((mapAddress>>8)&0xFF), 3);
      pidReadMap.setRequestByte((byte)(mapAddress&0xFF), 4);
      pidReadMap.setRequestByte((byte)readBytes, 5);
      pidResponse = getPid(&pidReadMap);
      if(pidResponse > 0)
      {
        dataRead = 0x40;
        mapAddress+=0x40;
        readBytes=0x40;
      }
      else if(pidResponse != PID_NOT_READY)
      {
        readmapStep = RDMAPSTAT_ERROR;
      }  
      break;

    case RDMAPSTAT_RDTB:
      //delay(Td5RequestDelay);
      if(mapAddress > 0x13FFC0)
      {
        readmapStep++;
        break;
      }
      pidReadMap.setRequestByte((byte)((mapAddress>>16)&0xFF), 2);
      pidReadMap.setRequestByte((byte)((mapAddress>>8)&0xFF), 3);
      pidReadMap.setRequestByte((byte)(mapAddress&0xFF), 4);
      pidReadMap.setRequestByte((byte)readBytes, 5);      
      pidResponse = getPid(&pidReadMap);
      if(pidResponse > 0)
      {
        if(mapAddress < 0x13FFC0)
        {
          dataRead = 0x40;
          if(mapAddress < 0x13FF80)
          {
            readBytes=0x40;
          }
          else
          {
            readBytes=0x3C;
          }
        }
        else
        {
          dataRead = 0x3C;
        }
        mapAddress+=0x40;
      }    
      else if(pidResponse != PID_NOT_READY)
      {
        readmapStep = RDMAPSTAT_ERROR;
      }
      break;

    case RDMAPSTAT_END:
      mapAddress = 0x000000; 
      readmapStep = 0;
      Td5RequestDelay = 55; // restore standard reading
      break;
  }
  
  return dataRead;  
}


// easy read calls
uint16_t Td5Comm::getEngineRPM() {
  return limit_value(LIM_MIN_ENGINE_RPM, (int16_t)pidRPM.getulValue(), LIM_MAX_ENGINE_RPM);
};

float Td5Comm::getTurboP() {
  return limit_value(LIM_MIN_TURBO_P, (pidTurboPressureMaf.getfValue(0)-pidAmbientPressure.getfValue(1)), LIM_MAX_TURBO_P);
};

float Td5Comm::getMAF() {
  return limit_value(LIM_MIN_MAF, pidTurboPressureMaf.getfValue(2), LIM_MAX_MAF);
};

float Td5Comm::getCurrentInj() {
  return limit_value(LIM_MIN_INJ, pidLiveFuelling.getfValue(3), LIM_MAX_INJ);
};

float Td5Comm::getDriverDemand() {
  return limit_value(LIM_MIN_INJ, pidLiveFuelling.getfValue(0), LIM_MAX_INJ);
};

float Td5Comm::getSmokeLim() {
  return limit_value(LIM_MIN_INJ, pidLiveFuelling.getfValue(5), LIM_MAX_INJ);
};

float Td5Comm::getTorqueLim() {
  return limit_value(LIM_MIN_INJ, pidLiveFuelling.getfValue(6), LIM_MAX_INJ);
};

float Td5Comm::getCoolantT() {
  return limit_value(LIM_MIN_COOLANT_T, pidTemperatures.getfValue(0), LIM_MAX_COOLANT_T);
};

float Td5Comm::getExtT() {
  return limit_value(LIM_MIN_EXT_T, pidTemperatures.getfValue(2), LIM_MAX_EXT_T);
};

float Td5Comm::getFuelT() {
  return limit_value(LIM_MIN_FUEL_T, pidTemperatures.getfValue(3), LIM_MAX_FUEL_T);
};    

float Td5Comm::getBattery() {
  return limit_value(LIM_MIN_BATTERY, pidBatteryVoltage.getfValue(0), LIM_MAX_BATTERY);
}; 

float Td5Comm::getAmbientP() {
  return limit_value(LIM_MIN_AMBIENT_P, pidAmbientPressure.getfValue(1), LIM_MAX_AMBIENT_P);
};

int16_t Td5Comm::getVehicleSpeed() {
  return limit_value(LIM_MIN_VEHICLE_SPD, (int16_t)pidVehicleSpeed.getbValue(0), LIM_MAX_VEHICLE_SPD);
};



///////////////////////////////////////////////////
//                 Class Td5Pid                  //
///////////////////////////////////////////////////
Td5Pid::Td5Pid(byte ID, byte reqlen, byte resplen, long cycletime)
{
  id = ID;
  cycleTime = cycletime;
  lastSeenTime = 0;
  
  responseLength = resplen;  

  requestFrame= (byte *)malloc(sizeof(byte) * reqlen);
  for(int i = 0; i < reqlen; i++)
  {
    requestFrame[i] = td5_pids[id][i];     
  }
  responseFrame = (byte *)malloc(sizeof(byte) * resplen);
  for(int i = 0; i < resplen; i++)
  {
    responseFrame[i] = 0;     
  }  
}

boolean Td5Pid::getValue(float *fvalue, byte index)
{
  boolean dataValid = false;
  uint16_t value;
  
  switch(id)
  {
    case AMBIENT_PRES:
      value = (uint16_t)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      *fvalue = (float) value / 10000.0;
      dataValid = true;
      break;
    case TEMPERATURES:     // 1/10 Celsius degrees
      value = (uint16_t)(((responseFrame[3 + (index * 4)] * 256L) + responseFrame[4 + (index * 4)]) - 2732L);
      *fvalue = (float) value / 10.0;
      dataValid = true;
      break;
    case INLET_PRES_MAF:   // 1/10000 bar
      value = (uint16_t)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      switch(index)
      {
        case 0:
        case 1:
          *fvalue = (float) value / 10000.0;    
          break;
        case 2:
        case 3:
          *fvalue = (float) value / 10.0;    
          break;
      }
      dataValid = true;
      break;
    case BATTERY_VOLT:     // 1/1000 Volt
      value = (uint16_t)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      *fvalue = (float) value / 1000.0;
      dataValid = true;
      break;
    case THROTTLE_POS:     // 1/1000 Volt   
      value = (uint16_t)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      *fvalue = (float) value / 1000.0;
      dataValid = true;
      break;
    case EGR_MOD:         // %   
    case ILT_MOD:         // %   
    case TWG_MOD:         // %   
      value = (uint16_t)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      *fvalue = (float) value / 10.0;
      dataValid = true;
      break;

    default: // return value incompatible with this pid
      *fvalue = 0.0;
      dataValid = false;
      break;
  }

  return dataValid;
}

float Td5Pid::getfValue(byte index)
{
  switch(id)
  {
    case AMBIENT_PRES:
      return ((float) ((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]) / 10000.0);
    case TEMPERATURES:     // 1/10 Celsius degrees
      return ((float) (((responseFrame[3 + (index * 4)] * 256L) + responseFrame[4 + (index * 4)]) - 2732L) / 10.0);
    case INLET_PRES_MAF:   // 1/10000 bar
      switch(index)
      {
        case 0:
        case 1:
          return ((float) ((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]) / 10000.0);    
        case 2:
        case 3:
          return ((float) ((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]) / 10.0);    
      }
    case BATTERY_VOLT:     // 1/1000 Volt
      return ((float) ((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]) / 1000.0);
    case THROTTLE_POS:     // 1/1000 Volt   
      switch(index)
      {
        case 0:        
        case 1:
        case 2:
        case 4:        
          return ((float) ((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]) / 1000.0);
        case 3:
          return ((float) ((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]) / 100.0);
      }
    case EGR_MOD:         // %   
    case ILT_MOD:         // %   
    case TWG_MOD:         // %       
      return ((float) ((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]) / 1000.0);
    case LIVE_FUELLING:  
      switch(index)
      {
        case 1:
          return ((float) ( (int16_t)(responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)] ) / 10.0);            
        case 0:
        case 3:
        case 5:
        case 6:
        case 7:
          return ((float) ( (int16_t)(responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)] ) / 100.0);  
        case 2:
          return ((float) ( (int16_t)(responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)] ) / 1.0);  
        case 4:
          return ((float) ( (int16_t)(responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)] ) / 1.0);  
      }
    default: // return value incompatible with this pid
      return 0.0;
  }
}


boolean Td5Pid::getValue(uint16_t *value, byte index)
{
  boolean dataValid = false;
  
  switch(id)
  {
    case ENGINE_RPM:       // rpm
      *value = (uint16_t)((responseFrame[3] * 256L) + responseFrame[4]);
      dataValid = true;
      break;
    case RPM_ERROR:       // rpm
      *value = (uint16_t)((responseFrame[3] * 256L) + responseFrame[4]);
      dataValid = true;
      break;
    case LIVE_FUELLING:  
      switch(index)
      {
        case 1:
        case 2:        
          *value = (((int16_t)(responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)] ) / 10);            
          dataValid = true;
          break;
      }
      break;
   default: // return value incompatible with this pid
      *value = 0L;
      dataValid = false;
      break;
  }

  return dataValid;
}


boolean Td5Pid::getValue(int *value, byte index)
{
  boolean dataValid = false;
  
  switch(id)
  {
    case INJ_BALANCE:       // injectors balance
      *value = (int)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      dataValid = true;
      break;

    default: // return value incompatible with this pid
      *value = 0;
      dataValid = false;
      break;
  }

  return dataValid;
}


uint16_t Td5Pid::getulValue(byte index)
{
  switch(id)
  {
    case ENGINE_RPM:       // rpm
      return((uint16_t)((responseFrame[3] * 256L) + responseFrame[4]));
      break;
    case RPM_ERROR:       // rpm
      return((uint16_t)((responseFrame[3] * 256L) + responseFrame[4]));
      break;
    case LIVE_FUELLING:  
      switch(index)
      {
        case 1:
        case 2:
          return(((int16_t)(responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)] ) / 10);
          break;
      }
      break;
    case INJ_BALANCE:       // injectors balance
      return(int)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      break;      

    default: // return value incompatible with this pid
      return 0L;
  }
}

int16_t Td5Pid::getlValue(byte index)
{
  switch(id)
  {
    case ENGINE_RPM:       // rpm
      return(int16_t)((responseFrame[3] * 256L) + responseFrame[4]);
      break;
    case RPM_ERROR:       // rpm
      return(int16_t)((responseFrame[3] * 256L) + responseFrame[4]);
      break;
    case INJ_BALANCE:       // injectors balance
      return(int16_t)((responseFrame[3 + (index * 2)] * 256L) + responseFrame[4 + (index * 2)]);
      break;      

    default: // return value incompatible with this pid
      return 0L;
  }
}

byte Td5Pid::getbValue(byte index)
{
  switch(id)
  {
    case VEHICLE_SPEED:       // vehicle speed
      return (byte)(responseFrame[3]);
      
    case READ_MAP:            // read map
      return (byte)(responseFrame[2 + index]);
      
    default: // return value incompatible with this pid
      return 0;
  }
}

void Td5Pid::setRequestByte(byte value, byte pos)
{
  requestFrame[pos] = value;    
}

byte Td5Pid::getResponseByte(byte pos)
{
  return responseFrame[pos];    
}


///////////////////////////////////////////////////
//         Data extraction functions             //
///////////////////////////////////////////////////
String pid_extract_injector_code(byte injector)
{
  String txt;
  
  if(!pidInjCodes.isConsistent())
    return String("");
  txt = String((char*) injector_codes[(pidInjCodes.getResponseByte(3+injector) / 2)]) + " " + 
        String((char*) injector_codes[(pidInjCodes.getResponseByte(3+injector+6) / 2)]) + " " + 
        String((char*) injector_codes_last[pidInjCodes.getResponseByte(3+injector+12)]);
  return txt; 
}

/*
00 tachometer 01 wgt modul.  02 egr inlet   03 clutch       04 cruise ctr.  05 cruise lamp  06 act.eng.mnt. 07 accel. 3 way
08 aircon     09 rad. fan.   10 aux fan     11 road speed.  12 egr modul.   13 amb. sensor  14 fuel type    15 temp. gauge
16 N/A        17 N/A         18 N/A         19 fuel temp.   20 N/A          21 N/A          22 N/A          23 N/A
24 can bus    25 slabs       26 mil         27 auto gearb.  28 N/A,         29 N/A          30 N/A          31 N/A
*/
boolean pid_extract_ecu_configuration(byte bit_index)
{
  if((bit_index < 0) || (bit_index > 31))
    return false;

  uint32_t registry = 0L;

  registry = pidEcuConfiguration.getResponseByte(6) * 16777216L +
             pidEcuConfiguration.getResponseByte(5) * 65536L +
             pidEcuConfiguration.getResponseByte(4) * 256L +
             pidEcuConfiguration.getResponseByte(3);

  return bitRead(registry, bit_index);
}

/*
    00 Brake 2   01 Clutch    02 CC Main    03 CC Set/Acc 04 CC Resume  05 N/A      06 N/A        07 N/A
    08 N/A       09 N/A       10 A/C Fan    11 A/C Clutch 12 N/A        13 N/A      14 Trans.case 15 Brake 1
*/
boolean pid_extract_ecu_input_state(byte bit_index)
{
  if((bit_index < 0) || (bit_index > 15))
    return false;

  uint16_t registry = 0L;

  registry = pidInputsState.getResponseByte(4) * 256L +
             pidInputsState.getResponseByte(3);

  return bitRead(registry, bit_index);
}


void pid_extract_ecu_vin(char* buffer)
{
  char vin[18]; char buf[10];

  if(!pidVinDateNnwHomol.isConsistent()){
    strcpy(buffer, ""); 
    return;
  }    

  for(int i=0;i<11;i++)
  {
    vin[i] = pidVinDateNnwHomol.getResponseByte(i+3);
  }

  vin[11] = 0;
  sprintf(buf, "%02x%02x%02x", pidVinDateNnwHomol.getResponseByte(14), pidVinDateNnwHomol.getResponseByte(15), pidVinDateNnwHomol.getResponseByte(16));  
  strcat(vin, buf);

  vin[17] = 0; // put a final zero at the end
  strcpy(buffer, vin); 
}

void pid_extract_ecu_prod_date(char* buffer)
{
  char prod_date[11];

  if(!pidVinDateNnwHomol.isConsistent()) {
    strcpy(buffer, ""); 
    return;
  }  

  sprintf(prod_date, "%02x-%02x-%02x%02x", pidVinDateNnwHomol.getResponseByte(18), pidVinDateNnwHomol.getResponseByte(19), pidVinDateNnwHomol.getResponseByte(20), 
          pidVinDateNnwHomol.getResponseByte(21));
    
  prod_date[10] = 0; // put a final zero at the end
  strcpy(buffer, prod_date); 
}

void pid_extract_ecu_nnw(char* buffer)
{
  char nnw[10]; char buf[10];
  
  if(!pidVinDateNnwHomol.isConsistent()){
    strcpy(buffer, ""); 
    return;
  }

  for(int i=0;i<3;i++)
  {
    nnw[i] = pidVinDateNnwHomol.getResponseByte(i+23);
  }
  nnw[3]=0;
  sprintf(buf, "%02x%02x%02x", pidVinDateNnwHomol.getResponseByte(26), pidVinDateNnwHomol.getResponseByte(27), pidVinDateNnwHomol.getResponseByte(28));
  strcat(nnw, buf);
    
  nnw[9] = 0; // put a final zero at the end
  strcpy(buffer, nnw); 
}

void pid_extract_ecu_homol(char* buffer)
{
  char homol[5];

  if(!pidVinDateNnwHomol.isConsistent()){
    strcpy(buffer, ""); 
    return;
  }

  sprintf(homol, "%02x%02x", pidVinDateNnwHomol.getResponseByte(33), pidVinDateNnwHomol.getResponseByte(34));
    
  homol[4] = 0; // put a final zero at the end
  strcpy(buffer, homol);  
}

void pid_extract_ecu_model(char* buffer)
{
  char ecu_model[10]; char buf[10];

  if(!pidEcuModel.isConsistent()){
    strcpy(buffer, ""); 
    return;
  }

  for(int i=0;i<3;i++)
  {
    ecu_model[i] = pidEcuModel.getResponseByte(i+3);
  }
  ecu_model[3]=0;
  sprintf(buf, "%02x%02x%02x", pidEcuModel.getResponseByte(6), pidEcuModel.getResponseByte(7), pidEcuModel.getResponseByte(8));
  strcat(ecu_model, buf);

  ecu_model[9] = 0; // put a final zero at the end
  strcpy(buffer, ecu_model);  
}

void pid_extract_ecu_firmware(char* buffer)
{
  char firmware[18];

  if(!pidVehicleFuelMap.isConsistent()){
    strcpy(buffer, ""); 
    return;
  }

  for(int i=0;i<8;i++)
  {
    firmware[i] = pidVehicleFuelMap.getResponseByte(i+3);
  }
  firmware[8]='-';
  for(int i=0;i<8;i++)
  {
    firmware[i+9] = pidVehicleFuelMap.getResponseByte(i+11);
  }

  firmware[17] = 0; // put a final zero at the end
  strcpy(buffer, firmware);  
}


void pid_extract_ecu_status(char* buffer)
{
  char ecu_status[7];

  if(!pidEcuStatus.isConsistent()){
    strcpy(buffer, ""); 
    return;
  }
    
  if(pidEcuStatus.getResponseByte(3) == 0x02)
  { 
    strcpy(ecu_status, "ROBUST");
  }
  else
  { 
    strcpy(ecu_status, "UNKN.");
  }
  
  ecu_status[6] = 0; // put a final zero at the end
  strcpy(buffer, ecu_status);
}


// Injector codes
const char *injector_codes[] = { 
  "NC",
  "NG",
  "NL",
  "NN",
  "NB",
  "NF",
  "NH",
  "BM",
  "BE",
  "BD",
  "BC",
  "BG",
  "BL",
  "BN",
  "BB",
  "BF",
  "BH",
  "FM",
  "FE",
  "FD",
  "FC",
  "FG",
  "FL",
  "FN",
  "FB",
  "FF",
  "FH",
  "HM",
  "HE",
  "HD",
  "HC",
  "HG",
  "HL",
  "HN",
  "HB",
  "HF",
  "HH",
  "AM",
  "AE",
  "AD",
  "AC",
  "AG",
  "AL",
  "AN",
  "AB",
  "AF",
  "AH",
  "KM",
  "KE",
  "KD",
  "KC",
  "KG",
  "KL",
  "KN",
  "KB",
  "KF",
  "KH",
  "JM",
  "JE",
  "JD",
  "JC",
  "JG",
  "JL",
  "JN",
  "M",
  "MM",
  "ME",
  "MD",
  "MC",
  "MG",
  "ML",
  "MN",
  "MB",
  "MF",
  "MH",
  "EM",
  "EE",
  "ED",
  "EC",
  "EG",
  "EL",
  "EN",
  "EB",
  "EF",
  "EH",
  "DM",
  "DE",
  "DD",
  "DC",
  "DG",
  "DL",
  "DN",
  "DB",
  "DF",
  "DH",
  "CM",
  "CE",
  "CD",
  "CC",
  "CG",
  "CL",
  "CN",
  "CB",
  "CF",
  "CH",
  "GM",
  "GE",
  "GD",
  "GC",
  "GG",
  "GL",
  "GN",
  "GB",
  "GF",
  "GH",
  "LM",
  "LE",
  "LD",
  "LC",
  "LG",
  "LL",
  "LN",
  "LB",
  "LF",
  "LH",
  "NM",
  "NE",
  "ND"
};


// Injector codes last codes
const char *injector_codes_last[] = { 
  "M/A",
  "E/B",
  "F/C",
  "G/A",
  "H",
  "J",
  "K",
  "L",
  "M"
};

// Fault codes
const char *fault_code[] = {
  "1-1 egr inlet throttle diagnostics (L)",
  "1-2 turbocharger wastegate diagnostics (L)",
  "1-3 egr vacuum diagnostics (L)",
  "1-4 temperature gauge diagnostics (L)",
  "1-5 driver demand problem 1 (L)",
  "1-6 driver demand problem 2 (L)",
  "1-7 air flow circuit (L)",
  "1-8 manifold pressure circuit (L)",
  "2-1 inlet air temp. circuit (L)",
  "2-2 fuel temp. circuit (L)",
  "2-3 coolant temp. circuit (L)",
  "2-4 battery volts (L)",
  "2-5 reference voltage (L)",
  "2-6 ambient air temp. circuit (L)",
  "2-7 driver demand supply problem (L)",
  "2-8 ambient pressure circuit (L)",
  "3-1 egr inlet throttle diagnostics (L)",
  "3-2 turbocharger wastegate diagnostics (L)",
  "3-3 egr vacuum diagnostics (L)",
  "3-4 temperature gauge diagnostics (L)",
  "3-5 driver demand problem 1 (L)",
  "3-6 driver demand problem 2 (L)",
  "3-7 air flow circuit (L)",
  "3-8 manifold pressure circuit (L)",
  "4-1 inlet air temp. circuit (L)",
  "4-2 fuel temperature circuit (L)",
  "4-3 coolant temp. circuit (L)",
  "4-4 battery volts (L)",
  "4-5 reference voltage (L)",
  "4-6 ambient air temperature circuit (L)",
  "4-7 driver demand supply problem (L)",
  "4-8 ambient pressure circuit (L)",
  "5-1 egr inlet throttle diagnostics (C)",
  "5-2 turbocharger wastegate diagnostics (C)",
  "5-3 egr vacuum diagnostics (C)",
  "5-4 temperature gauge diagnostics (C)",
  "5-5 driver demand problem 1 (C)",
  "5-6 driver demand problem 2 (C)",
  "5-7 air flow circuit (C)",
  "5-8 manifold pressure circuit (C)",
  "6-1 inlet air temp. circuit (C)",
  "6-2 fuel temperature circuit (C)",
  "6-3 coolant temp. circuit (C)",
  "6-4 battery voltage problem (C)",
  "6-5 reference voltage (C)",
  "6-7 driver demand supply problem (C)",
  "6-8 ambient pressure circuit (C)",
  "7-1 cruise lamp drive over temp. (L)",
  "7-2 fuel used output drive over temp. (L)",
  "7-3 radiator fan drive over temp. (L)",
  "7-4 active engine mounting over temp. (L)",
  "7-5 turbocharger wastegate short circuit (L)",
  "7-6 egr inlet throttle short circuit (L)",
  "7-7 egr vacuum modulator short circuit (L)",
  "7-8 temperature gauge short circuit (L)",
  "8-1 air conditioning fan drive over temp. (L)",
  "8-2 fuel pump drive over temp. (L)",
  "8-3 tacho drive over temp. (L)",
  "8-4 gearbox/abs drive over temp. (L)",
  "8-5 air conditioning clutch over temp. (L)",
  "8-6 mil lamp drive over temp. (L)",
  "8-7 glow plug relay drive over temp. (L)",
  "8-8 glowplug lamp drive over temperature (L)",
  "9-1 fuel used output drive open load (L)",
  "9-2 cruise lamp drive open load (L)",
  "9-3 radiator fan drive open load (L)",
  "9-4 active engine mounting open load (L)",
  "9-5 turbocharger wastegate open load (L)",
  "9-6 egr inlett throttle open load (L)",
  "9-7 egr vacuum modulator open load (L)",
  "9-8 temperature gauge open load (L)",
  "10-1 air conditioning fan drive open load (L)",
  "10-2 fuel pump drive open load (L)",
  "10-3 tachometer open load (L)",
  "10-4 gearbox/abs drive open load (L)",
  "10-5 air conditioning clutch open load (L)",
  "10-6 mil lamp drive open load (L)",
  "10-7 glow plug lamp drive open load (L)",
  "10-8 glow plug relay drive open load (L)",
  "11-1 cruise control lamp drive over temperature (C)",
  "11-2 fuel used output drive over temperature (C)",
  "11-3 radiator fan drive over temperature (C)",
  "11-4 active engine mounting over temperature (C)",
  "11-5 turbocharger wastegate short circuit (C)",
  "11-6 egr inlet throttle short circuit (C)",
  "11-7 egr vacuum modulator short circuit (C)",
  "11-8 temperature gauge short circuit (C)",
  "12-1 air conditioning fan drive open load (C)",
  "12-2 fuel pump drive open load (C)",
  "12-3 tachometer open load (C)",
  "12-4 gearbox/abs drive open load (C)",
  "12-5 air conditioning clutch open load (C)",
  "12-6 mil lamp drive open load (C)",
  "12-7 glow plug relay drive open load (C)",
  "12-8 glowplug relay drive open load (C)",
  "13-1 cruise control lamp drive over temp. (C)",
  "13-2 fuel used output drive over temp. (C)",
  "13-3 radiator fan drive over temp. (C)",
  "13-4 active engine mounting over temp. (C)",
  "13-5 turbocharger wastegate short circuit (C)",
  "13-6 egr inlet throttle short circuit (C)",
  "13-7 egr vacuum modulator short circuit (C)",
  "13-8 temperature gauge short circuit (C)",
  "14-1 air conditioning fan drive open load (C)",
  "14-2 fuel pump drive open load (C)",
  "14-3 tachometer open load (C)",
  "14-4 gearbox/abs drive open load (C)",
  "14-5 air conditioning clutch open load (C)",
  "14-6 mil lamp drive open load (C)",
  "14-7 glow plug relay drive open load (C)",
  "14-8 glowplug relay drive open load (C)",
  "15-2 high speed crank (L)",
  "16-2 high speed crank (L)",
  "17-2 high speed crank (C)",
  "19-2 can rx/tx error (L)",
  "19-3 can tx/rx error (L)",
  "19-6 noisy crank signal has been detected (L)",
  "19-8 can has had reset failure (L)",
  "20-1 turbocharger under boosting (L)",
  "20-2 turbocharger over boosting (L)",
  "20-4 egr valve stuck open (L)",
  "20-5 egr valve stuck closed (L)",
  "21-4 driver demand 1 out of range (L)",
  "21-5 driver demand 2 out of range (L)",
  "21-6 problem detected with driver demand (L)",
  "21-7 inconsistencies found with driver demand (L)",
  "21-8 injector trim data corrupted (L)",
  "22-1 road speed missing (L)",
  "22-3 vehicle accel. outside bounds of cruise control (L)",
  "22-7 cruise control resume stuck closed (L)",
  "22-8 cruise control set stuck closed (L)",
  "23-1 excessive can bus off (C)",
  "23-2 can rx/tx error (C)",
  "23-3 can tx/rx error (C)",
  "23-4 unable to detect remote can mode (C)",
  "23-5 under boost has occurred on this trip (C)",
  "23-6 noisy crack signal has been detected (C)",
  "24-1 turbocharger under boosting (C)",
  "24-2 turbocharger over boosting (C)",
  "24-3 over boost has occurred this trip (C)",
  "24-4 egr valve stuck open (C)",
  "24-5 egr valve stuck closed (C)",
  "24-7 problem detected with auto gear box (C)",
  "25-4 driver demand 1 out of range (L)",
  "25-5 driver demand 2 out of range (L)",
  "25-6 problem detected with drive demand (C)",
  "25-7 inconsistencies found with driver demand (C)",
  "25-8 injector trim data corrupted (C)",
  "26-1 road speed missing (C)",
  "26-2 cruise control system problem (C)",
  "26-3 vehicle accel. outside bounds for cruise control (C)",
  "26-7 cruise control resume stuck closed (C)",
  "26-8 cruise control set stuck closed (C)",
  "27-1 inj. 1 peak charge long (L)",
  "27-2 inj. 2 peck charge long (L)",
  "27-3 inj. 3 peak charge long (L)",
  "27-4 inj. 4 peck charge long (L)",
  "27-5 inj. 5 peak charge long (L)",
  "27-6 inj. 6 peck charge long (L)",
  "27-7 topside switch failed post injection (L)",
  "28-1 inj. 1 peak charge short (L)",
  "28-2 inj. 2 peck charge short (L)",
  "28-3 inj. 3 peak charge short (L)",
  "28-4 inj. 4 peck charge short (L)",
  "28-5 inj. 5 peak charge short (L)",
  "28-6 inj. 6 peck charge short (L)",
  "28-7 topside switch failed pre injection (L)",
  "29-1 inj. 1 peak charge long (C)",
  "29-2 inj. 2 peck charge long (C)",
  "29-3 inj. 3 peak charge long (C)",
  "29-4 inj. 4 peck charge long (C)",
  "29-5 inj. 5 peak charge long (C)",
  "29-6 inj. 6 peck charge long (C)",
  "29-7 topside switch failed post injection (C)",
  "30-1 inj. 1 peak charge short (C)",
  "30-2 inj. 2 peck charge short (C)",
  "30-3 inj. 3 peak charge short (C)",
  "30-4 inj. 4 peck charge short (C)",
  "30-5 inj. 5 peak charge short (C)",
  "30-6 inj. 6 peck charge short (C)",
  "30-7 topside switch failed pre injection (C)",
  "31-1 inj. 1 open circuit (L)",
  "31-2 inj. 2 open circuit (L)",
  "31-3 inj. 3 open circuit (L)",
  "31-4 inj. 4 open circuit (L)",
  "31-5 inj. 5 open circuit (L)",
  "31-6 inj. 6 open circuit (L)",
  "32-1 inj. 1 short circuit (L)",
  "32-2 inj. 2 short circuit (L)",
  "32-3 inj. 3 short circuit (L)",
  "32-4 inj. 4 short circuit (L)",
  "32-5 inj. 5 short circuit (L)",
  "32-6 inj. 6 short circuit (L)",
  "33-1 inj. 1 open circuit (C)",
  "33-2 inj. 2 open circuit (C)",
  "33-3 inj. 3 open circuit (C)",
  "33-4 inj. 4 open circuit (C)",
  "33-5 inj. 5 open circuit (C)",
  "33-6 inj. 6 open circuit (C)",
  "34-1 inj. 1 short circuit (C)",
  "34-2 inj. 2 short circuit (C)",
  "34-3 inj. 3 short circuit (C)",
  "34-4 inj. 4 short circuit (C)",
  "34-5 inj. 5 short circuit (C)",
  "34-6 inj. 6 short circuit (C)",
  "35-1 inj. 1 partial short circuit (L)",
  "35-2 inj. 2 partial short circuit (L)",
  "35-3 inj. 3 partial short circuit (L)",
  "35-4 inj. 4 partial short circuit (L)",
  "35-5 inj. 5 partial short circuit (L)",
  "35-6 inj. 6 partial short circuit (L)"
};
