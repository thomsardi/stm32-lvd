#include <Arduino.h>
#include <CANController.h>
#include <STM32FreeRTOS.h>
#include <Beastdevices_INA3221.h>
#include <DataDef.h>
#include <BatteryProcessing.h>
#include <EEPROM.h>
#include <Pack.h>

#define ON1 PA5 //  Vsat
#define ON2 PA6 //  BTS
#define ON3 PA7 //  Other
#define OFF1 PB0  //  Vsat
#define OFF2 PB1  //  BTS
#define OFF3 PB10 //  Other

/**
 * Pin remap
*/
#define VSAT_ON PA7
#define VSAT_OFF  PB10
#define BTS_ON  PA6
#define BTS_OFF PB1
#define OTHER_ON  PA5
#define OTHER_OFF PB0

#define FB1 PB14  //  Vsat
#define FB2 PB13  //  BTS
#define FB3 PB12  //  Other

#define LVD_LOW_VSAT_ADDR 0
#define LVD_LOW_OTHER_ADDR 4
#define LVD_LOW_BTS_ADDR 8
#define LVD_RECONNECT_VSAT_ADDR 12
#define LVD_RECONNECT_OTHER_ADDR  16
#define LVD_RECONNECT_BTS_ADDR  20
#define LVD_TOLERANCE_ADDR 24
#define NOMINAL_BAT_ADDR 28
#define UNIQUE_ID_ADDR 32
#define FLAG_SET_ADDR 36

#define idCanbusEnergyMeter 0x1D40C8E7

CANController can;
Beastdevices_INA3221 ina3221b(INA3221_ADDR40_GND);
BatteryProcessing batProc;
AdditionalCANData additionalCanData;
AlarmParameter voltageAlarm = {4600, 4600, 4700, 4700, 4700, 4800};
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

BatteryData batteryData[24];
size_t batterySize = sizeof(batteryData) / sizeof(batteryData[0]);

BatteryData dummyData[24];
Pack packData(dummyData, 24, 0);

EhubRelayWrite ehubRelay;
KeepAliveCounter keepAliveCounter;
bool isDataReady = false;
// put function declarations here:

void testCan(CAN_msg_t msg)
{
  Serial1.println("can handler block");
}

void canHandler(CAN_msg_t msg)
{
  uint32_t frameId = msg.id;
  HalfPackData halfPackData;
  HalfMosfetData halfMosfetData;
  int length = msg.len;
  int index = 0;
  int baseAddr = 0;
  // Serial1.print("Frame Addr : ");
  // Serial1.println(frameId, HEX);
  // return;
  // Serial1.println((frameId & 0xFFFFFFC0), HEX);
  switch (frameId & 0xFFFFFFC0)
  {
  case PACK_DATA:
    baseAddr = static_cast<int>(PACK_ADDR);
    index = baseAddr - frameId;
    // Serial1.println("Index : " + String(index));
    // return;
    // index = abs(index);
    // Serial1.println("index : " + String(index));
    // for (size_t i = 0; i < length; i++)
    // {
    //   Serial1.print(msg.data[i], HEX);
    //   Serial1.print(" ");
    // }
    halfPackData.id = index;
    halfPackData.isUpdated = 1;
    halfPackData.packVoltage = batProc.getPackVoltage(msg, 2, 2);
    halfPackData.packCurrent = batProc.getPackCurrent(msg, 4, 2);
    halfPackData.packSoc = batProc.getPackSoc(msg, 6, 2);
    packData.insert(halfPackData);
    isDataReady = true;
    // batteryData[index - 1].id = index;
    // batteryData[index - 1].isUpdated = 1;
    // batteryData[index - 1].packVoltage = batProc.getPackVoltage(msg, 2, 2);
    // batteryData[index - 1].packCurrent = batProc.getPackCurrent(msg, 4, 2);
    // batteryData[index - 1].packSoc = batProc.getPackSoc(msg, 6, 2);
    // batteryData[index - 1].incPackCounter();
    // Serial1.println("Id : " + String(index));
    // Serial1.println("Pack Voltage : " + String(batteryData[index - 1].packVoltage));
    // Serial1.println("Pack Current : " + String(batteryData[index - 1].packCurrent));
    // Serial1.println("Pack Soc : " + String(batteryData[index - 1].packSoc));
    // return;
    break;
  case MOSF_TEMP :
    baseAddr = static_cast<int>(MOSF_TEMP_ADDR);
    index = baseAddr - frameId;

    halfMosfetData.id = index;
    halfMosfetData.isUpdated = 1;
    batProc.updateMosfetStatus(msg.data[0], halfMosfetData);
    halfMosfetData.temperature.top = batProc.getTemperature(msg,3, 1);
    halfMosfetData.temperature.mid = batProc.getTemperature(msg,4, 1);
    halfMosfetData.temperature.bot = batProc.getTemperature(msg,5, 1);
    halfMosfetData.temperature.cmosTemp = batProc.getTemperature(msg,6, 1);
    halfMosfetData.temperature.dmosTemp = batProc.getTemperature(msg,7, 1);
    packData.insert(halfMosfetData);

    // batteryData[index - 1].id = index;
    // batteryData[index - 1].isUpdated = 1;    
    // batProc.updateMosfetStatus(msg.data[0], batteryData[index-1]);
    // batteryData[index - 1].temperature.top = batProc.getTemperature(msg,3, 1);
    // batteryData[index - 1].temperature.mid = batProc.getTemperature(msg,4, 1);
    // batteryData[index - 1].temperature.bot = batProc.getTemperature(msg,5, 1);
    // batteryData[index - 1].temperature.cmosTemp = batProc.getTemperature(msg,6, 1);
    // batteryData[index - 1].temperature.dmosTemp = batProc.getTemperature(msg,7, 1);
    // batteryData[index - 1].incMosfetCounter();
    // Serial1.println("Id : " + String(index - 1));
    // Serial1.println("Mosfet Status : " + String(batteryData[index - 1].mosfetStatus.val));
    // Serial1.println("Temperature Top : " + String(batteryData[index - 1].temperature.top));
    // Serial1.println("Temperature Mid : " + String(batteryData[index - 1].temperature.mid));
    // Serial1.println("Temperature Bot : " + String(batteryData[index - 1].temperature.bot));
    // Serial1.println("Temperature Cmos : " + String(batteryData[index - 1].temperature.cmosTemp));
    // Serial1.println("Temperature Dmos : " + String(batteryData[index - 1].temperature.dmosTemp));
    // return;
    break;

  default:
    break;
  }
  // return;

  if (frameId == 0x1D42C8E8) //ehub write lvd low config
  {
    // Serial1.println("Write lvd low config");
    voltageAlarm.vsatLowVoltage = (msg.data[1] << 8) + msg.data[0];
    uint16_t temp;
    EEPROM.get(LVD_LOW_VSAT_ADDR, temp);
    if(temp != voltageAlarm.vsatLowVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_LOW_VSAT_ADDR, voltageAlarm.vsatLowVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    voltageAlarm.otherLowVoltage = (msg.data[3] << 8) + msg.data[2];
    EEPROM.get(LVD_LOW_OTHER_ADDR, temp);
    if(temp != voltageAlarm.otherLowVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_LOW_OTHER_ADDR, voltageAlarm.otherLowVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    voltageAlarm.btsLowVoltage = (msg.data[5] << 8) + msg.data[4];
    EEPROM.get(LVD_LOW_BTS_ADDR, temp);
    if(temp != voltageAlarm.btsLowVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_LOW_BTS_ADDR, voltageAlarm.btsLowVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    // voltageAlarm.tolerance = (msg.data[7] << 8) + msg.data[6];
    // EEPROM.get(LVD_TOLERANCE_ADDR, temp);
    // if(temp != voltageAlarm.tolerance)
    // {
    //   Serial1.println("Write to EEPROM");
    //   EEPROM.put(LVD_TOLERANCE_ADDR, voltageAlarm.tolerance);
    // }
    // else
    // {
    //   Serial1.println("Same data");
    // }

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

  if (frameId == 0x1D43C8E8) //ehub write lvd reconnect config
  {
    Serial1.println("Write lvd reconnect config");
    voltageAlarm.vsatReconnectVoltage = (msg.data[1] << 8) + msg.data[0];
    uint16_t temp;
    EEPROM.get(LVD_RECONNECT_VSAT_ADDR, temp);
    if(temp != voltageAlarm.vsatReconnectVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_RECONNECT_VSAT_ADDR, voltageAlarm.vsatReconnectVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    voltageAlarm.otherReconnectVoltage = (msg.data[3] << 8) + msg.data[2];
    EEPROM.get(LVD_RECONNECT_OTHER_ADDR, temp);
    if(temp != voltageAlarm.otherReconnectVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_RECONNECT_OTHER_ADDR, voltageAlarm.otherReconnectVoltage);
    }
    else
    {
      Serial1.println("Same data");
    }

    voltageAlarm.btsReconnectVoltage = (msg.data[5] << 8) + msg.data[4];
    EEPROM.get(LVD_RECONNECT_BTS_ADDR, temp);
    if(temp != voltageAlarm.btsReconnectVoltage)
    {
      Serial1.println("Write to EEPROM");
      EEPROM.put(LVD_RECONNECT_BTS_ADDR, voltageAlarm.btsReconnectVoltage);
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

  if(frameId == 0x1D44C8E8) //ehub write system config
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
    // Serial1.println("Keep alive");
    // uint16_t id = (msg.data[1] << 8) + msg.data[0];
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
    // switch (msg.data[0])
    // {
    // case 1:
    //   ehubRelay.vsat = 1;
    //   break;
    // case 2:
    //   ehubRelay.vsat = 0;
    //   break;
    // case 3:
    //   ehubRelay.bts = 1;
    //   break;
    // case 4:
    //   ehubRelay.bts = 0;
    //   break;
    // case 5:
    //   ehubRelay.other = 1;
    //   break;
    // case 6:
    //   ehubRelay.other = 0;
    //   break;
    // default:
    //   break;
    // }
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
}

int myFunction(int, int);

void vsatON()
{
  // digitalWrite(ON1, HIGH);
  digitalWrite(VSAT_ON, HIGH);
  vTaskDelay(100);
  digitalWrite(VSAT_ON, LOW);
  // digitalWrite(ON1, LOW);
  
}

void vsatOFF()
{
  // digitalWrite(OFF1, HIGH);
  digitalWrite(VSAT_OFF, HIGH);
  vTaskDelay(100);
  digitalWrite(VSAT_OFF, LOW);
  // digitalWrite(OFF1, LOW);
}

void btsON()
{
  // digitalWrite(ON2, HIGH);
  digitalWrite(BTS_ON, HIGH);
  vTaskDelay(100);
  digitalWrite(BTS_ON, LOW);
  // digitalWrite(ON2, LOW);
}

void btsOFF()
{
  // digitalWrite(OFF2, HIGH);
  digitalWrite(BTS_OFF, HIGH);
  vTaskDelay(100);
  digitalWrite(BTS_OFF, LOW);
  // digitalWrite(OFF2, LOW);
}

void otherON()
{
  // digitalWrite(ON3, HIGH);
  digitalWrite(OTHER_ON, HIGH);
  vTaskDelay(100);
  digitalWrite(OTHER_ON, LOW);
  // digitalWrite(ON3, LOW);
}

void otherOFF()
{
  // digitalWrite(OFF3, HIGH);
  digitalWrite(OTHER_OFF, HIGH);
  vTaskDelay(100);
  digitalWrite(OTHER_OFF, LOW);
  // digitalWrite(OFF3, LOW);
}

static void relayTask(void *arg)
{
  bool btsOn = true;
  bool vsatOn = true;
  bool otherOn = true;
  bool btsOnSimState = false;
  bool vsatOnSimState = false;
  bool otherOnSimState = false;
  uint8_t timeToShow = 0;
  uint8_t receivedData;
  int32_t totalVoltage = 0;
  int32_t totalCurrent = 0;
  int32_t totalSoc = 0;
  uint8_t detectedBattery = 0;
  int32_t averageVoltage = 0;
  int32_t averageCurrent = 0;
  int32_t averageSoc = 0;
  bool isFirstRun = true;
  unsigned long previousTime = 0;
  while(1)
  {
    // Serial1.println("Relay Task");
    while(isFirstRun)
    {
      if((millis() - previousTime) > 10000 || isDataReady)
      // if(isDataReady)
      {
        isFirstRun = false;
        Serial1.println("Data Ready");
        // lastTime = millis();
      }
      vTaskDelay(500);
    }
    // detectedBattery = 0;
    // averageVoltage = 0;
    // averageCurrent = 0;
    // totalVoltage = 0;
    // totalCurrent = 0;
    // totalSoc = 0;
    // for (size_t i = 0; i < batterySize; i++)
    // {
    //   if (batteryData[i].isUpdated && batteryData[i].mosfetStatus.dmos) // if the value constanly updated and dmos is connected to load, count the battery
    //   {
    //     totalVoltage += batteryData[i].packVoltage;
    //     totalCurrent += batteryData[i].packCurrent;
    //     totalSoc += batteryData[i].packSoc;
    //     detectedBattery++;
    //   }
    //   else
    //   {
    //     continue;
    //   }
    // }
    
    // averageVoltage = totalVoltage / detectedBattery;
    // averageCurrent = totalCurrent;
    // averageSoc = totalSoc / detectedBattery;

    packData.calculate();
    averageVoltage = packData.getAverageVoltage();
    // Serial1.println("AVERAGE VOLTAGE : " + String(averageVoltage));

    if(timeToShow > 5)
    {
      // packData.printInfo();
      // Serial1.println("Detected Battery : " + String(detectedBattery));
      // Serial1.println("Average Pack Voltage : " + String(averageVoltage));
      // Serial1.println("Average Pack Current : " + String(averageCurrent));
      // Serial1.println("Average Pack Soc : " + String(averageSoc));
      Serial1.println("================Begin==============");
      Serial1.println("Total Battery : " + String(packData.getTotalBattery()));
      Serial1.println("Active Battery : " + String(packData.getActiveBattery()));
      Serial1.println("Inactive Battery : " + String(packData.getInactiveBattery()));
      Serial1.println("Average Pack Voltage : " + String(packData.getAverageVoltage()));
      Serial1.println("Total Pack Current : " + String(packData.getTotalCurrent()));
      Serial1.println("Average Pack Soc : " + String(packData.getAverageSoc()));

      Serial1.println("Bts Voltage Alarm : " + String(voltageAlarm.btsLowVoltage));
      Serial1.println("Bts Upper Threshold : " + String(voltageAlarm.btsReconnectVoltage));
      Serial1.println("Vsat Voltage Alarm : " + String(voltageAlarm.vsatLowVoltage));
      Serial1.println("Vsat Upper Threshold : " + String(voltageAlarm.vsatReconnectVoltage));
      Serial1.println("Other Voltage Alarm : " + String(voltageAlarm.otherLowVoltage));
      Serial1.println("Other Upper Threshold : " + String(voltageAlarm.otherReconnectVoltage));
      Serial1.println("================End==============");
      timeToShow = 0;
    }
    
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
      Serial1.println("lost connection from Ehub");
      isOverriden = false;
      lostConnectionCounter = 0;
    }

    if(!isOverriden)
    {
      // Serial1.println("Is Not Overriden");
      if (averageVoltage <= voltageAlarm.btsLowVoltage)
      {
        btsOn = false;
      }
      if (averageVoltage <= voltageAlarm.vsatLowVoltage)
      {
        vsatOn = false;
      }
      if (averageVoltage <= voltageAlarm.otherLowVoltage)
      {
        otherOn = false;
      }

      /**
       * @brief this check state of vsat, if this should be on, then it check for relay feedback, if it's still not connected,
       *        re - trigger relay again
       * @details when vsat should be off, it will check the feedback, if it's still connected, re - trigger relay again. the state
       *          will get flipped when the voltage reach voltage alarm + tolerance
      */
      if (vsatOn) //if vsat relay should be on
      {
        if(!additionalCanData.relayState.vsat) //if feedback still off, re-trigger relay
        // if(!vsatOnSimState) //if feedback still off, re-trigger relay
        {
          Serial1.println("===Vsat On===");
          vsatOnSimState = true;
          ehubRelay.vsat = true;
          vsatON();
        }
      }
      else //if vsat relay should be off
      {
        if(additionalCanData.relayState.vsat) //if feedback still on, re-trigger relay
        // if(vsatOnSimState) //if feedback still on, re-trigger relay
        {
          Serial1.println("===Vsat Off===");
          vsatOnSimState = false;
          ehubRelay.vsat = false;
          vsatOFF();
        }
        if(averageVoltage >= voltageAlarm.getVsatUpperThreshold())
        // if(averageVoltage >= voltageAlarm.vsatReconnectVoltage)
        {
          vsatOn = true;
        }      
        
      }
      
      if (btsOn) //if bts relay should be on
      {
        if(!additionalCanData.relayState.bts) //if feedback still off, re-trigger relay
        // if(!btsOnSimState) //if feedback still off, re-trigger relay
        {
          Serial1.println("===Bts On===");
          btsOnSimState = true;
          ehubRelay.bts = true;
          btsON();
        }      
      }
      else //if bts relay should be off
      {
        if(additionalCanData.relayState.bts) //if feedback still on, re-trigger relay
        // if(btsOnSimState) //if feedback still on, re-trigger relay
        {
          Serial1.println("===Bts Off===");
          btsOnSimState = false;
          ehubRelay.bts = false;
          btsOFF();
        }
        // if(averageVoltage >= voltageAlarm.getBtsUpperThreshold())
        if(averageVoltage >= voltageAlarm.btsReconnectVoltage)
        {
          btsOn = true;
        }      
      }

      if (otherOn) //if other relay should be on
      {
        
        if(!additionalCanData.relayState.other) //if feedback still off, re-trigger relay
        // if(!otherOnSimState) //if feedback still off, re-trigger relay
        {
          Serial1.println("===Other On===");  
          otherOnSimState = true;
          ehubRelay.other = true;
          otherON();
        }
      }
      else //if other relay should be off
      {
         if(additionalCanData.relayState.other) //if feedback still on, re-trigger relay
        // if(otherOnSimState) //if feedback still on, re-trigger relay
        {
          Serial1.println("===Other Off===");  
          otherOnSimState = false;
          ehubRelay.other = false;
          otherOFF();
        }
        if(averageVoltage >= voltageAlarm.getOtherUpperThreshold())
        // if(averageVoltage >= voltageAlarm.otherReconnectVoltage)
        {
          otherOn = true;
        }
      }
    }
    else
    {
      Serial1.println("Overriden");
      if(ehubRelay.vsat)  // if ehub activate vsat relay
      {
        if(!additionalCanData.relayState.vsat) //if feedback still off, re-trigger relay
        // if(!vsatOnSimState)
        {
          Serial1.println("Vsat On with override");
          vsatOnSimState = true;
          ehubRelay.vsat = true;
          vsatON();
        }
      }
      else  //if ehub deactivate vsat relay
      {
        if(additionalCanData.relayState.vsat) //if feedback still on, re-trigger relay
        // if(vsatOnSimState)
        {
          Serial1.println("Vsat off with override");
          vsatOnSimState = false;
          ehubRelay.vsat = false;
          vsatOFF();
        }  
      }

      if (ehubRelay.bts)  // if ehub activate bts relay
      {
        if(!additionalCanData.relayState.bts) //if feedback still off, re-trigger relay
        // if(!btsOnSimState)
        {
          Serial1.println("Bts on with override");
          btsOnSimState = true;
          ehubRelay.bts = true;
          btsON();
        }
      }
      else  //if ehub deactivate bts relay
      {
        if(additionalCanData.relayState.bts)  //if feedback still on, re-trigger relay
        // if(btsOnSimState)
        {
          Serial1.println("Bts off with override");
          btsOnSimState = false;
          ehubRelay.bts = false;
          btsOFF();
        }
      }
      
      if (ehubRelay.other) // if ehub activate other relay
      {
        if(!additionalCanData.relayState.other) //if feedback still off, re-trigger relay
        // if(!otherOnSimState)
        {
          Serial1.println("Other on with override");
          otherOnSimState = true;
          ehubRelay.other = true;
          otherON();
        }
      }
      else  //if ehub deactivate other relay
      {
        if(additionalCanData.relayState.other)  //if feedback still on, re-trigger relay
        // if(otherOnSimState)
        {
          Serial1.println("Other off with override");
          otherOnSimState = false;
          ehubRelay.other = false;
          otherOFF();
        }
      }
    }
    timeToShow++;
    vTaskDelay(200);
  }
}


static void canSenderTask(void *arg)
{
  bool isOff = true;
  CAN_msg_t msg;
  int idInc = 0;
  // int totalId = sizeof(batteryData) / sizeof(batteryData[0]);
  // Serial.println("Total size = " + String(totalId));
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
      // if(idInc >= packData.stack.size())
      // {
      //   idInc = 0;
      // }
      // BatteryData temp = packData.getData(idInc);
      for (size_t i = 1; i <= 24; i++)
      {
        /* code */
        CAN_msg_t msg;
        msg.id = WAKE_ADDR + (i << 8);
        msg.format = CAN_FORMAT::EXTENDED_FORMAT;
        msg.type = CAN_FRAME::DATA_FRAME;
        msg.len = 8;
        msg.data[0] = 0x33;
        msg.data[1] = 0x63;
        msg.data[2] = 0x60;
        msg.data[3] = 0x64;
        msg.data[4] = 0x64;
        msg.data[5] = 0x64;
        msg.data[6] = 0x53;
        msg.data[7] = 0x64;
        can.send(&msg);
        // vTaskDelay(20);
      }
      
        
      // Serial1.println("Send CAN to wake up battery");
      // Serial1.print("Frame id : ");
      // Serial1.println(msg.id, HEX);
      // Serial1.print("Data : ");
      // for (size_t i = 0; i < 8; i++)
      // {
      //   Serial1.print(msg.data[i], HEX);
      //   Serial1.print(" ");
      // }
      // Serial.println();
      // idInc++;
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
  
  // vsatON();
  // btsON();
  // otherON();

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
  Serial1.println("SETUP");
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
    
    EEPROM.get(LVD_LOW_VSAT_ADDR, temp);
    Serial1.println("lvd vsat voltage : " + String(temp));
    voltageAlarm.vsatLowVoltage = temp;
    EEPROM.get(LVD_LOW_BTS_ADDR, temp);
    Serial1.println("lvd bts voltage : " + String(temp));
    voltageAlarm.btsLowVoltage = temp;
    EEPROM.get(LVD_LOW_OTHER_ADDR, temp);
    Serial1.println("lvd other voltage : " + String(temp));
    voltageAlarm.otherLowVoltage = temp;
    EEPROM.get(LVD_TOLERANCE_ADDR, temp);
    Serial1.println("lvd tolerance : " + String(temp));
    voltageAlarm.tolerance = temp;

    EEPROM.get(LVD_RECONNECT_VSAT_ADDR, temp);
    Serial1.println("lvd vsat reconnect voltage : " + String(temp));
    voltageAlarm.vsatReconnectVoltage = temp;
    EEPROM.get(LVD_RECONNECT_BTS_ADDR, temp);
    Serial1.println("lvd bts reconnect voltage : " + String(temp));
    voltageAlarm.btsReconnectVoltage = temp;
    EEPROM.get(LVD_RECONNECT_OTHER_ADDR, temp);
    Serial1.println("lvd other reconnect voltage : " + String(temp));
    voltageAlarm.otherReconnectVoltage = temp;

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
    EEPROM.put(LVD_LOW_VSAT_ADDR, voltageAlarm.vsatLowVoltage);
    EEPROM.get(LVD_LOW_VSAT_ADDR, temp);
    Serial1.println("lvd vsat voltage : " + String(temp));
    EEPROM.put(LVD_LOW_BTS_ADDR, voltageAlarm.btsLowVoltage);
    EEPROM.get(LVD_LOW_BTS_ADDR, temp);
    Serial1.println("lvd bts voltage : " + String(temp));
    EEPROM.put(LVD_LOW_OTHER_ADDR, voltageAlarm.otherLowVoltage);
    EEPROM.get(LVD_LOW_OTHER_ADDR, temp);

    EEPROM.put(LVD_RECONNECT_VSAT_ADDR, voltageAlarm.vsatReconnectVoltage);
    EEPROM.get(LVD_RECONNECT_VSAT_ADDR, temp);
    Serial1.println("lvd vsat reconnect voltage : " + String(temp));
    EEPROM.put(LVD_RECONNECT_BTS_ADDR, voltageAlarm.btsReconnectVoltage);
    EEPROM.get(LVD_RECONNECT_BTS_ADDR, temp);
    Serial1.println("lvd bts reconnect voltage : " + String(temp));
    EEPROM.put(LVD_RECONNECT_OTHER_ADDR, voltageAlarm.otherReconnectVoltage);
    EEPROM.get(LVD_RECONNECT_OTHER_ADDR, temp);
    Serial1.println("lvd other reconnect voltage : " + String(temp));

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
  for (size_t i = 1; i <= 64; i++)
  {
    CAN_msg_t msg;
    msg.id = WAKE_ADDR + (i << 8);
    msg.format = CAN_FORMAT::EXTENDED_FORMAT;
    msg.type = CAN_FRAME::DATA_FRAME;
    msg.len = 8;
    msg.data[0] = 0x33;
    msg.data[1] = 0x63;
    msg.data[2] = 0x60;
    msg.data[3] = 0x64;
    msg.data[4] = 0x64;
    msg.data[5] = 0x64;
    msg.data[6] = 0x53;
    msg.data[7] = 0x64;
    can.send(&msg);
    delay(20);
  }  
  vTaskStartScheduler();
}

void loop() {
  // Serial1.println("Loop");
  can.loop();
  
  additionalCanData.relayState.vsat = digitalRead(FB1);
  additionalCanData.relayState.bts = digitalRead(FB2);
  additionalCanData.relayState.other = digitalRead(FB3);

  // Serial1.println("Vsat Status : " + String(additionalCanData.relayState.vsat));
  // Serial1.println("Bts Status : " + String(additionalCanData.relayState.bts));
  // Serial1.println("Other Status : " + String(additionalCanData.relayState.other));

  if(millis() - lastCheckTime > 5000)
  {
    // for (size_t i = 0; i < batterySize; i++)
    // {
    //   if (batteryData[i].isUpdated)
    //   {
    //     if(batteryData[i].cnt.previousPackUpdatedCounter == batteryData[i].cnt.packUpdatedCounter)
    //     {
    //       batteryData[i].isUpdated = 0;
    //     }
    //   }
    // }
    packData.removeUnusedData();
    lastCheckTime = millis();
  }

  if (millis() - lastTime > 1000)
  {
    ina3221Task();
    Serial1.println("Vsat Relay State : " + String(additionalCanData.relayState.vsat));
    Serial1.println("Bts Relay State : " + String(additionalCanData.relayState.bts));
    Serial1.println("Other Relay State : " + String(additionalCanData.relayState.other));

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