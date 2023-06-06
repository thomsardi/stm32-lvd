#include <Arduino.h>
#include <CANController.h>
#include <STM32FreeRTOS.h>
#include <Beastdevices_INA3221.h>
#include <DataDef.h>
#include <BatteryProcessing.h>
#include <EEPROM.h>

#define ON1 PA5
#define ON2 PA6
#define ON3 PA7
#define OFF1 PB0
#define OFF2 PB1
#define OFF3 PB10

#define FB1 PB14
#define FB2 PB13
#define FB3 PB12

#define LVD_VSAT_ADDR 0
#define LVD_OTHER_ADDR 4
#define LVD_BTS_ADDR 8
#define LVD_TOLERANCE_ADDR 12
#define NOMINAL_BAT_ADDR 16
#define UNIQUE_ID_ADDR 20
#define FLAG_SET_ADDR 24

#define idCanbusEnergyMeter 0x1D40C8E7

CANController can;
Beastdevices_INA3221 ina3221b(INA3221_ADDR40_GND);
BatteryProcessing batProc;
AdditionalCANData additionalCanData;
AlarmParameter voltageAlarm = {460, 460, 480};
TaskHandle_t relayTaskHandle;
TaskHandle_t canSenderTaskHandle;
// TaskHandle_t ina3221TaskHandle;

SemaphoreHandle_t myLock = xSemaphoreCreateMutex();

// QueueHandle_t relayTaskQueue;
QueueHandle_t canSenderTaskQueue;

uint32_t count;
uint32_t count2;
uint16_t uniqueId = 160;
uint8_t lostConnectionCounter = 0;
bool isOverriden;
unsigned long lastTime = 0;
unsigned long lastCheckTime = 0;

BatteryData batteryData[16];
size_t batterySize = sizeof(batteryData) / sizeof(batteryData[0]);

EhubRelayWrite ehubRelay;
KeepAliveCounter keepAliveCounter;
// put function declarations here:

void canHandler(CAN_msg_t msg)
{
  uint8_t data;
  bool state = true;
  count++;
  count2++;
  if (count > 9)
  {
    count = 0;
    // xQueueSend(relayTaskQueue, &data, 0);
    // xSemaphoreTake(myLock, 10);
  }
  if (count2 > 4)
  {
    count2 = 0;
    // xQueueSend(canSenderTaskQueue, &state, 0);
    // xSemaphoreTake(myLock, 10);
  }
  // Serial1.println("Count : " + String(count));
  // Serial1.print("ID : ");
  // Serial1.println(msg.id, HEX);

  // BatteryData *p;
  uint32_t frameId = msg.id;
  int length = msg.len;
  int index = 0;
  int baseAddr = 0;
  // Serial1.print("Frame Addr : ");
  // Serial1.println(frameId, HEX);
  // Serial1.println((frameId & 0xFFFFFFC0), HEX);
  switch (frameId & 0xFFFFFFC0)
  {
  case PACK_DATA:
    baseAddr = static_cast<int>(PACK_ADDR);
    index = baseAddr - frameId;
    // index = abs(index);
    // Serial1.println("index : " + String(index));
    // for (size_t i = 0; i < length; i++)
    // {
    //   Serial1.print(msg.data[i], HEX);
    //   Serial1.print(" ");
    // }
    batteryData[index - 1].isUpdated = 1;
    batteryData[index - 1].packVoltage = batProc.getPackVoltage(msg.data, 2, 2);
    batteryData[index - 1].packCurrent = batProc.getPackCurrent(msg.data, 4, 2);
    batteryData[index - 1].packSoc = batProc.getPackSoc(msg.data, 6, 2);
    batteryData[index - 1].updatePackCounter();
    // Serial1.println("Id : " + String(index));
    // Serial1.println("Pack Voltage : " + String(batteryData[index - 1].packVoltage));
    // Serial1.println("Pack Current : " + String(batteryData[index - 1].packCurrent));
    // Serial1.println("Pack Soc : " + String(batteryData[index - 1].packSoc));
    break;
  case MOSF_TEMP :
    baseAddr = static_cast<int>(MOSF_TEMP_ADDR);
    index = baseAddr - frameId;
    // index = abs(index);
    batteryData[index - 1].isUpdated = 1;
    batProc.updateMosfetStatus(msg.data[0], batteryData[index-1]);
    batteryData[index - 1].updateMosfetCounter();
    batteryData[index - 1].temperature.top = batProc.getTemperature(msg.data,3, 1);
    batteryData[index - 1].temperature.mid = batProc.getTemperature(msg.data,4, 1);
    batteryData[index - 1].temperature.bot = batProc.getTemperature(msg.data,5, 1);
    batteryData[index - 1].temperature.cmosTemp = batProc.getTemperature(msg.data,6, 1);
    batteryData[index - 1].temperature.dmosTemp = batProc.getTemperature(msg.data,7, 1);
    batteryData[index - 1].updateTemperatureCounter();
    // Serial1.println("Id : " + String(index - 1));
    // Serial1.println("Mosfet Status : " + String(batteryData[index - 1].mosfetStatus.val));
    // Serial1.println("Temperature Top : " + String(batteryData[index - 1].temperature.top));
    // Serial1.println("Temperature Mid : " + String(batteryData[index - 1].temperature.mid));
    // Serial1.println("Temperature Bot : " + String(batteryData[index - 1].temperature.bot));
    // Serial1.println("Temperature Cmos : " + String(batteryData[index - 1].temperature.cmosTemp));
    // Serial1.println("Temperature Dmos : " + String(batteryData[index - 1].temperature.dmosTemp));
    break;

  default:
    break;
  }

  if (frameId == 0x1D42C8E8) //ehub write lvd config
  {
    Serial1.println("Write lvd config");
    voltageAlarm.vsatAlarmVoltage = (msg.data[1] << 8) + msg.data[0];
    uint16_t temp;
    EEPROM.get(LVD_VSAT_ADDR, temp);
    if(temp != voltageAlarm.vsatAlarmVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_VSAT_ADDR, voltageAlarm.vsatAlarmVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    voltageAlarm.otherAlarmVoltage = (msg.data[3] << 8) + msg.data[2];
    EEPROM.get(LVD_OTHER_ADDR, temp);
    if(temp != voltageAlarm.otherAlarmVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_OTHER_ADDR, voltageAlarm.otherAlarmVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    voltageAlarm.btsAlarmVoltage = (msg.data[5] << 8) + msg.data[4];
    EEPROM.get(LVD_BTS_ADDR, temp);
    if(temp != voltageAlarm.btsAlarmVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_BTS_ADDR, voltageAlarm.btsAlarmVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    voltageAlarm.tolerance = (msg.data[7] << 8) + msg.data[6];
    EEPROM.get(LVD_TOLERANCE_ADDR, temp);
    if(temp != voltageAlarm.tolerance)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_TOLERANCE_ADDR, voltageAlarm.tolerance);
    }
    else
    {
      Serial1.println("Same data");
    }
    CAN_msg_t response;
    response.id = (frameId & 0x0000FFFF) + ((frameId & 0x00FF0000) << 8) + ((frameId & 0xFF000000) >> 8);
    Serial1.print("Response Id : ");
    Serial1.println(response.id, HEX);
    response.format = CAN_FORMAT::EXTENDED_FORMAT;
    response.type = CAN_FRAME::DATA_FRAME;
    response.len = msg.len;
    for (size_t i = 0; i < msg.len; i++)
    {
      response.data[i] = msg.data[i];
    }
    xQueueSend(canSenderTaskQueue, &response, 200);
  }

  if(frameId == 0x1D43C8E8) //ehub write system config
  {
    Serial1.println("Write system config");
    voltageAlarm.nominalBattery = (msg.data[1] << 8) + msg.data[0];
    uint16_t temp;
    EEPROM.get(NOMINAL_BAT_ADDR, temp);
    if(temp != voltageAlarm.nominalBattery)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(NOMINAL_BAT_ADDR, voltageAlarm.nominalBattery);
    }
    else
    {
      Serial1.println("Same data");
    }
    CAN_msg_t response;
    response.id = (frameId & 0x0000FFFF) + ((frameId & 0x00FF0000) << 8) + ((frameId & 0xFF000000) >> 8);
    Serial1.print("Response Id : ");
    Serial1.println(response.id, HEX);
    response.format = CAN_FORMAT::EXTENDED_FORMAT;
    response.type = CAN_FRAME::DATA_FRAME;
    response.len = msg.len;
    for (size_t i = 0; i < msg.len; i++)
    {
      response.data[i] = msg.data[i];
    }
    xQueueSend(canSenderTaskQueue, &response, 200);
  }

  if(frameId == 0x1D41C8E8) //ehub keep alive & override
  {
    Serial1.println("Keep alive");
    uint16_t id = (msg.data[1] << 8) + msg.data[0];
    // keepAliveCounter.cnt = (frameId & 0xFFFF0000) + (msg.data[3] << 24) + (msg.data[2] << 16) + (msg.data[1] << 8) + msg.data[0];
    // if (id == uniqueId)
    // {
    keepAliveCounter.cnt++;
    lostConnectionCounter = 0;
    if(msg.data[2] > 0)
    {
      isOverriden = true;
    }
    else
    {
      isOverriden = false;
    }
    // }
  }

  if(frameId == 0x1D40C8E8) //ehub write relay
  {
    Serial1.println("Ehub relay write");
    switch (msg.data[0])
    {
    case 1:
      ehubRelay.other = 1;
      break;
    case 2:
      ehubRelay.other = 0;
      break;
    case 3:
      ehubRelay.bts = 1;
      break;
    case 4:
      ehubRelay.bts = 0;
      break;
    case 5:
      ehubRelay.vsat = 1;
      break;
    case 6:
      ehubRelay.vsat = 0;
      break;
    default:
      break;
    }
    CAN_msg_t response;
    response.id = (frameId & 0x0000FFFF) + ((frameId & 0x00FF0000) << 8) + ((frameId & 0xFF000000) >> 8);
    Serial1.print("Response Id : ");
    Serial1.println(response.id, HEX);
    response.format = CAN_FORMAT::EXTENDED_FORMAT;
    response.type = CAN_FRAME::DATA_FRAME;
    response.len = msg.len;
    for (size_t i = 0; i < msg.len; i++)
    {
      response.data[i] = msg.data[i];
    }
    xQueueSend(canSenderTaskQueue, &response, 200);
  }


  // if(msg.id == idCanbusEnergyMeter)
  // {
  //   Serial1.println("EnergyMeter Found");
  // }
  // CAN_FORMAT format = static_cast<CAN_FORMAT>(msg.format);
  // switch (format)
  // {
  // case EXTENDED_FORMAT:
  //   Serial1.println("Format : Extended");
  //   break;
  
  // default:
  //   Serial1.println("Format : Standard");
  //   break;
  // }
  // CAN_FRAME frame = static_cast<CAN_FRAME>(msg.type);
  // switch (frame)
  // {
  // case REMOTE_FRAME:
  //   Serial1.println("Type : Remote");
  //   break;
  
  // default:
  //   Serial1.println("Type : Data");
  //   break;
  // }
  
  // Serial1.print("Data : ");
  // for (size_t i = 0; i < length; i++)
  // {
  //   Serial1.print(msg.data[i], HEX);
  //   Serial1.print(" ");
  // }
  // Serial1.println();
  // xSemaphoreGive(myLock);
  
}

int myFunction(int, int);

void vsatON()
{
  digitalWrite(ON1, HIGH);
  vTaskDelay(20);
  digitalWrite(ON1, LOW);
}

void vsatOFF()
{
  digitalWrite(OFF1, HIGH);
  vTaskDelay(20);
  digitalWrite(OFF1, LOW);
}

void btsON()
{
  digitalWrite(ON2, HIGH);
  vTaskDelay(20);
  digitalWrite(ON2, LOW);
}

void btsOFF()
{
  digitalWrite(OFF2, HIGH);
  vTaskDelay(20);
  digitalWrite(OFF2, LOW);
}

void otherON()
{
  digitalWrite(ON3, HIGH);
  vTaskDelay(20);
  digitalWrite(ON3, LOW);
}

void otherOFF()
{
  digitalWrite(OFF3, HIGH);
  vTaskDelay(20);
  digitalWrite(OFF3, LOW);
}

static void relayTask(void *arg)
{
  bool btsOn = true;
  bool vsatOn = true;
  bool otherOn = true;
  bool btsOnSimState = false;
  bool vsatOnSimState = false;
  bool otherOnSimState = false;
  uint8_t receivedData;
  int32_t totalVoltage = 0;
  uint8_t detectedBattery = 0;
  int32_t averageVoltage = 0;
  while(1)
  {
    // Serial1.println("Relay Task");
    detectedBattery = 0;
    averageVoltage = 0;
    totalVoltage = 0;
    for (size_t i = 0; i < batterySize; i++)
    {
      if (batteryData[i].isUpdated && batteryData[i].mosfetStatus.dmos) // if the value constanly updated and dmos is connected to load, count the battery
      {
        totalVoltage += batteryData[i].packVoltage;
        detectedBattery++;
      }
      else
      {
        continue;
      }
    }
    // Serial1.println("Detected Battery : " + String(detectedBattery));
    averageVoltage = totalVoltage / detectedBattery;
    // Serial1.println("Average Pack Voltage : " + String(averageVoltage));

    // Serial1.println("Bts Voltage Alarm : " + String(voltageAlarm.btsAlarmVoltage));
    // Serial1.println("Bts Upper Threshold : " + String(voltageAlarm.getBtsUpperThreshold()));
    // Serial1.println("Vsat Voltage Alarm : " + String(voltageAlarm.vsatAlarmVoltage));
    // Serial1.println("Vsat Upper Threshold : " + String(voltageAlarm.getVsatUpperThreshold()));
    // Serial1.println("Other Voltage Alarm : " + String(voltageAlarm.otherAlarmVoltage));
    // Serial1.println("Other Upper Threshold : " + String(voltageAlarm.getOtherUpperThreshold()));

    if (keepAliveCounter.lastCnt != keepAliveCounter.cnt)
    {
      keepAliveCounter.lastCnt = keepAliveCounter.cnt;
    }
    else
    {
      lostConnectionCounter++;
    }
    
    if(lostConnectionCounter > 50)
    {
      // Serial1.println("lost connection from Ehub");
      isOverriden = false;
      lostConnectionCounter = 0;
    }

    if(!isOverriden)
    {
      // Serial1.println("Is Not Overriden");
      if (averageVoltage <= voltageAlarm.btsAlarmVoltage)
      {
        btsOn = false;
      }
      if (averageVoltage <= voltageAlarm.vsatAlarmVoltage)
      {
        vsatOn = false;
      }
      if (averageVoltage <= voltageAlarm.otherAlarmVoltage)
      {
        otherOn = false;
      }

      /**
       * @brief this check state of vsat, if this should be on, then it check for relay feedback, if it's still not connected,
       *        re - trigger relay again
       * @details when vsat should be off, it will check the feedback, if it's still connected, re - trigger relay again. the state
       *          will get flipped when the voltage reach voltage alarm + tolerance
      */
      if (vsatOn) //if bts should be on
      {
        // if(!additionalCanData.relayState.vsat || !vsatOnSimState) //if feedback still off, re-trigger relay
        if(!vsatOnSimState) //if feedback still off, re-trigger relay
        {
          Serial1.println("===Vsat On===");
          vsatOnSimState = true;
          ehubRelay.vsat = true;
          // vsatON();
        }
      }
      else //if bts should be off
      {
        // if(additionalCanData.relayState.vsat || vsatOnSimState) //if feedback still on, re-trigger relay
        if(vsatOnSimState) //if feedback still on, re-trigger relay
        {
          Serial1.println("===Vsat Off===");
          vsatOnSimState = false;
          ehubRelay.vsat = false;
          // vsatOFF();
        }
        else
        {
          if(averageVoltage >= voltageAlarm.getVsatUpperThreshold())
          {
            vsatOn = true;
          }      
        }
      }
      
      if (btsOn) //if bts should be on
      {
        // if(!additionalCanData.relayState.bts || !btsOnSimState) //if feedback still off, re-trigger relay
        if(!btsOnSimState) //if feedback still off, re-trigger relay
        {
          Serial1.println("===Bts On===");
          btsOnSimState = true;
          ehubRelay.bts = true;
          // btsON();
        }      
      }
      else //if bts should be off
      {
        // if(additionalCanData.relayState.bts || btsOnSimState) //if feedback still on, re-trigger relay
        if(btsOnSimState) //if feedback still on, re-trigger relay
        {
          Serial1.println("===Bts Off===");
          btsOnSimState = false;
          ehubRelay.bts = false;
          // btsOFF();
        }
        else
        {
          if(averageVoltage >= voltageAlarm.getBtsUpperThreshold())
          {
            btsOn = true;
          }      
        }
      }

      if (otherOn) //if bts should be on
      {
        
        // if(!additionalCanData.relayState.other || !otherOnSimState) //if feedback still off, re-trigger relay
        if(!otherOnSimState) //if feedback still off, re-trigger relay
        {
          Serial1.println("===Other On===");  
          otherOnSimState = true;
          ehubRelay.other = true;
          // otherON();
        }
      }
      else //if bts should be off
      {
        // if(additionalCanData.relayState.other || otherOnSimState) //if feedback still on, re-trigger relay
        if(otherOnSimState) //if feedback still on, re-trigger relay
        {
          Serial1.println("===Other Off===");  
          otherOnSimState = false;
          ehubRelay.other = false;
          // otherOFF();
        }
        else
        {
          if(averageVoltage >= voltageAlarm.getOtherUpperThreshold())
          {
            otherOn = true;
          }
        }
      }
    }
    else
    {
      Serial1.println("Overriden");
      if(ehubRelay.vsat)
      {
        // if(!additionalCanData.relayState.vsat || !vsatOnSimState)
        if(!vsatOnSimState)
        {
          Serial1.println("Vsat On with override");
          vsatOnSimState = true;
          vsatON();
        }
      }
      else
      {
        // if(additionalCanData.relayState.vsat || vsatOnSimState)
        if(vsatOnSimState)
        {
          Serial1.println("Vsat off with override");
          vsatOnSimState = false;
          vsatOFF();
        }  
      }

      if (ehubRelay.bts)
      {
        // if(!additionalCanData.relayState.bts || !btsOnSimState)
        if(!btsOnSimState)
        {
          Serial1.println("Bts on with override");
          btsOnSimState = true;
          btsON();
        }
      }
      else
      {
        // if(additionalCanData.relayState.bts)
        if(btsOnSimState)
        {
          Serial1.println("Bts off with override");
          btsOnSimState = false;
          btsOFF();
        }
      }
      
      if (ehubRelay.other)
      {
        // if(!additionalCanData.relayState.other)
        if(!otherOnSimState)
        {
          Serial1.println("Other on with override");
          otherOnSimState = true;
          otherON();
        }
      }
      else
      {
        // if(additionalCanData.relayState.other)
        if(otherOnSimState)
        {
          Serial1.println("Other off with override");
          otherOnSimState = false;
          otherOFF();
        }
      }
    }
    vTaskDelay(200);
  }
}


static void canSenderTask(void *arg)
{
  bool isOff = true;
  CAN_msg_t msg;
  while(1)
  {
    if (xQueueReceive(canSenderTaskQueue, &msg, 100) == pdTRUE) 
    {
      can.send(&msg);
      // Serial1.println("Send CAN current & relay");
    }
    else
    {
      // msg to keep the battery awake
      CAN_msg_t msg;
      msg.id = 0x12345678;
      msg.format = CAN_FORMAT::EXTENDED_FORMAT;
      msg.type = CAN_FRAME::DATA_FRAME;
      msg.len = 8;
      msg.data[0] = 1;
      msg.data[1] = 2; // for relay contact on or off
      msg.data[2] = 3;
      msg.data[3] = 4;
      msg.data[4] = 5;
      msg.data[5] = 6;
      msg.data[6] = 7;
      msg.data[7] = 8;
      // can.send(&msg);
      // Serial1.println("Send CAN to wake up battery");

    }
    // vTaskDelay(1000);
  }
}

void ina3221Task()
{
  int limitUnderAmpere = 0.2;
  
  float current[3];
  float voltage[3];

  current[0] = -ina3221b.getCurrent(INA3221_CH3); // add - assign to negate the value
  // voltage[0] = ina3221b.getVoltage(INA3221_CH1);

  current[1] = -ina3221b.getCurrent(INA3221_CH2); // add - assign to negate the value
  // voltage[1] = ina3221b.getVoltage(INA3221_CH2);

  current[2] = -ina3221b.getCurrent(INA3221_CH1); // add - assign to negate the value
  // voltage[2] = ina3221b.getVoltage(INA3221_CH3);

  if (current[0] < limitUnderAmpere)
    current[0] = 0;
  if (current[1] < limitUnderAmpere)
    current[1] = 0;
  if (current[2] < limitUnderAmpere)
    current[2] = 0;

  // int current1bit1 = current[0];
  // int current1bit2 = (current[0] - current1bit1) * 100;

  // int current2bit1 = current[1];
  // int current2bit2 = (current[1] - current2bit1) * 100;

  // int current3bit1 = current[2];
  // int current3bit2 = (current[2] - current3bit1) * 100;

  additionalCanData.current[0] = current[0];
  additionalCanData.current[1] = current[1];
  additionalCanData.current[2] = current[2];

  // msg.data[2] = current1bit1;
  // msg.data[3] = current1bit2;
  // msg.data[4] = current2bit1;
  // msg.data[5] = current2bit2;
  // msg.data[6] = current3bit1;
  // msg.data[7] = current3bit2;
  // int dataDummy[8];
  // dataDummy[0] = 1; // standart start frame sundaya
  // dataDummy[1] = totalFo; // for relay contact on or off
  // dataDummy[2] = current1bit1;
  // dataDummy[3] = current1bit2;
  // dataDummy[4] = current2bit1;
  // dataDummy[5] = current2bit2;
  // dataDummy[6] = current3bit1;
  // dataDummy[7] = current3bit2;
  
  // sendCanbus(dataDummy);

  // Serial1.print("Channel 1: ");
  // Serial1.print(current[0], 1);
  // Serial1.println("A, ");

  // Serial1.print("Channel 2: ");
  // Serial1.print(current[1], 1);
  // Serial1.println("A, ");

  // Serial1.print("Channel 3: ");
  // Serial1.print(current[2], 1);
  // Serial1.println("A, ");

  // delay(10);
}

void setup() {
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  pinMode(ON1, OUTPUT);
  pinMode(ON2, OUTPUT);
  pinMode(ON3, OUTPUT);
  pinMode(OFF1, OUTPUT);
  pinMode(OFF2, OUTPUT);
  pinMode(OFF3, OUTPUT);
  pinMode(FB1, INPUT);
  pinMode(FB2, INPUT);
  pinMode(FB3, INPUT);
  

  // batteryData[0].mosfetStatus.cmos = 1;
  // batteryData[0].mosfetStatus.dmos = 0;
  xTaskCreate(relayTask,
    "RelayTask",
    configMINIMAL_STACK_SIZE + 100,
    NULL,
    tskIDLE_PRIORITY + 2,
    &relayTaskHandle);

  xTaskCreate(canSenderTask,
    "canSenderTask",
    configMINIMAL_STACK_SIZE,
    // 64,
    NULL,
    tskIDLE_PRIORITY + 1,
    &canSenderTaskHandle);

  // xTaskCreate(ina3221Task,
  //   "ina3221Task",
  //   configMINIMAL_STACK_SIZE,
  //   NULL,
  //   tskIDLE_PRIORITY + 2,
  //   &ina3221TaskHandle);

  // relayTaskQueue = xQueueCreate(5, sizeof(uint8_t));
  canSenderTaskQueue = xQueueCreate(10, sizeof(CAN_msg_t));

  ina3221b.begin();
  ina3221b.reset();
  ina3221b.setShuntRes(5, 5, 5);
  Serial1.begin(115200);
  can.onReceive(canHandler);
  if(!can.init(CAN_250KBPS, 0))
  {
    Serial1.println("CAN1 initialize fail");
  }
  Serial1.println("CAN1 initialize success");
  
  uint8_t isFlagSet;
  EEPROM.get(FLAG_SET_ADDR, isFlagSet);
  uint16_t temp;
  if(isFlagSet == 1)
  {
    Serial1.println("Get existing data from EEPROM");
    
    EEPROM.get(LVD_VSAT_ADDR, temp);
    Serial1.println("lvd vsat voltage : " + String(temp));
    voltageAlarm.vsatAlarmVoltage = temp;
    EEPROM.get(LVD_BTS_ADDR, temp);
    Serial1.println("lvd bts voltage : " + String(temp));
    voltageAlarm.btsAlarmVoltage = temp;
    EEPROM.get(LVD_OTHER_ADDR, temp);
    Serial1.println("lvd other voltage : " + String(temp));
    voltageAlarm.otherAlarmVoltage = temp;
    EEPROM.get(LVD_TOLERANCE_ADDR, temp);
    Serial1.println("lvd tolerance : " + String(temp));
    voltageAlarm.tolerance = temp;
    EEPROM.get(NOMINAL_BAT_ADDR, temp);
    Serial1.println("nominal bat voltage : " + String(temp));
    voltageAlarm.nominalBattery = temp;
    EEPROM.get(UNIQUE_ID_ADDR, temp);
    Serial1.println("uniqueid : " + String(temp));
    uniqueId = temp;
    EEPROM.get(FLAG_SET_ADDR, temp);
    Serial1.println("flag set : " + String(temp));
  }
  else
  {
    Serial1.println("Write new data to EEPROM");
    EEPROM.put(LVD_VSAT_ADDR, voltageAlarm.vsatAlarmVoltage);
    EEPROM.get(LVD_VSAT_ADDR, temp);
    Serial1.println("lvd vsat voltage : " + String(temp));
    EEPROM.put(LVD_BTS_ADDR, voltageAlarm.btsAlarmVoltage);
    EEPROM.get(LVD_BTS_ADDR, temp);
    Serial1.println("lvd bts voltage : " + String(temp));
    EEPROM.put(LVD_OTHER_ADDR, voltageAlarm.otherAlarmVoltage);
    EEPROM.get(LVD_OTHER_ADDR, temp);
    Serial1.println("lvd other voltage : " + String(temp));
    EEPROM.put(LVD_TOLERANCE_ADDR, voltageAlarm.tolerance);
    EEPROM.get(LVD_TOLERANCE_ADDR, temp);
    Serial1.println("lvd tolerance : " + String(temp));
    EEPROM.put(NOMINAL_BAT_ADDR, voltageAlarm.nominalBattery);
    EEPROM.get(NOMINAL_BAT_ADDR, temp);
    Serial1.println("nominal bat voltage : " + String(temp));
    EEPROM.put(UNIQUE_ID_ADDR, uniqueId);
    EEPROM.get(UNIQUE_ID_ADDR, temp);
    Serial1.println("uniqueid : " + String(temp));
    EEPROM.put(FLAG_SET_ADDR, 1);
    EEPROM.get(FLAG_SET_ADDR, isFlagSet);
    Serial1.println("flag set : " + String(isFlagSet));
  }

  // uint32_t bank1, bank2;
  // bank1 = 0x12345678 << 3;
  // bank1 = bank1 + 0x04; // Ext
  // bank2 = 0xFFFFFFFC; // Must be IDE=1
  // can.setFilter(0, 1, 0, 0, bank1, bank2);
  // can.setFilter(0, 1, 0, 0, 0x02, 0x1FFFFFFF);
  FilterConfig filterConfig = {
    .idConfig = {
      // .id = 0x760C840,
      .id = 0x540C840,
      .ideMode = FilterConfig::CanFormatAcceptanceMode::EXTENDED_FORMAT,
      .rtrMode = FilterConfig::CanFrameAcceptanceMode::DATA_FRAME
    },
    .maskConfig = {
      // .mask = 0xFF8FFC0,
      .mask = 0x5D8FF40,
      .ideCheck = FilterConfig::IdeCheck::IDE_UNCHECKED,
      .rtrCheck = FilterConfig::RtrCheck::RTR_UNCHECKED,
    }
  };
  bool result = can.filter(filterConfig);
  if(result)
  {
    Serial1.println("Success set filter");
  }
  else
  {
    Serial1.println("Failed to set filter");
  }
  vTaskStartScheduler();
}

void loop() {
  can.loop();
  
  additionalCanData.relayState.vsat = digitalRead(FB1);
  additionalCanData.relayState.bts = digitalRead(FB2);
  additionalCanData.relayState.other = digitalRead(FB3);

  if(millis() - lastCheckTime > 5000)
  {
    for (size_t i = 0; i < batterySize; i++)
    {
      if (batteryData[i].isUpdated)
      {
        if(batteryData[i].cnt.previousPackUpdatedCounter == batteryData[i].cnt.packUpdatedCounter)
        {
          batteryData[i].isUpdated = 0;
        }
      }
    }
    lastCheckTime = millis();
  }

  if (millis() - lastTime > 1000)
  {
    ina3221Task();
    // Serial1.println("Vsat Relay State : " + String(additionalCanData.relayState.vsat));
    // Serial1.println("Bts Relay State : " + String(additionalCanData.relayState.bts));
    // Serial1.println("Other Relay State : " + String(additionalCanData.relayState.other));

    lastTime = millis();

    int current1bit1 = additionalCanData.current[0];
    int current1bit2 = (additionalCanData.current[0] - current1bit1) * 100;

    int current2bit1 = additionalCanData.current[1];
    int current2bit2 = (additionalCanData.current[1] - current2bit1) * 100;

    int current3bit1 = additionalCanData.current[2];
    int current3bit2 = (additionalCanData.current[2] - current3bit1) * 100;

    CAN_msg_t msg;
    msg.id = 0x1D40C8E7;
    msg.format = CAN_FORMAT::EXTENDED_FORMAT;
    msg.type = CAN_FRAME::DATA_FRAME;
    msg.len = 8;
    msg.data[0] = 1;
    msg.data[1] = additionalCanData.relayState.val; // for relay contact on or off
    msg.data[2] = current1bit1;
    msg.data[3] = current1bit2;
    msg.data[4] = current2bit1;
    msg.data[5] = current2bit2;
    msg.data[6] = current3bit1;
    msg.data[7] = current3bit2;
    xQueueSend(canSenderTaskQueue, &msg, portMAX_DELAY);
  }

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}