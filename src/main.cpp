#include <Arduino.h>
#include <CANController.h>
#include <STM32FreeRTOS.h>
#include <Beastdevices_INA3221.h>
#include <DataDef.h>
#include <BatteryProcessing.h>

#define ON1 PA5
#define ON2 PA6
#define ON3 PA7
#define OFF1 PB0
#define OFF2 PB1
#define OFF3 PB10

#define FB1 PB14
#define FB2 PB13
#define FB3 PB12

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
unsigned long lastTime = 0;
unsigned long lastCheckTime = 0;

BatteryData batteryData[16];
size_t batterySize = sizeof(batteryData) / sizeof(batteryData[0]);
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
  // Serial1.print("Data Addr : ");
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
    delay(500);
    digitalWrite(ON1, LOW);
    delay(500);
}

void vsatOFF()
{
    digitalWrite(OFF1, HIGH);
    delay(500);
    digitalWrite(OFF1, LOW);
    delay(500);
}

void btsON()
{
    digitalWrite(ON2, HIGH);
    delay(500);
    digitalWrite(ON2, LOW);
    delay(500);
}

void btsOFF()
{
    digitalWrite(OFF2, HIGH);
    delay(500);
    digitalWrite(OFF2, LOW);
    delay(500);
}

void otherON()
{
    digitalWrite(ON3, HIGH);
    delay(500);
    digitalWrite(ON3, LOW);
    delay(500);
}

void otherOFF()
{
    digitalWrite(OFF3, HIGH);
    delay(500);
    digitalWrite(OFF3, LOW);
    delay(500);
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
      if (batteryData[i].isUpdated)
      {
        totalVoltage += batteryData[i].packVoltage;
        detectedBattery++;
      }
      else
      {
        continue;
      }
    }
    Serial1.println("Detected Battery : " + String(detectedBattery));
    averageVoltage = totalVoltage / detectedBattery;
    Serial1.println("Average Pack Voltage : " + String(averageVoltage));

    Serial1.println("Bts Voltage : " + String(voltageAlarm.btsAlarmVoltage));
    Serial1.println("Vsat Voltage : " + String(voltageAlarm.vsatAlarmVoltage));
    Serial1.println("Other Voltage : " + String(voltageAlarm.otherAlarmVoltage));

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
      if(!additionalCanData.relayState.vsat || !vsatOnSimState) //if feedback still off, re-trigger relay
      {
        Serial1.println("===Vsat On===");
        vsatOnSimState = true;
        digitalWrite(ON1, HIGH);
        vTaskDelay(20);
        digitalWrite(ON1, LOW);
      }
    }
    else //if bts should be off
    {
      if(additionalCanData.relayState.vsat || vsatOnSimState) //if feedback still on, re-trigger relay
      {
        Serial1.println("===Vsat Off===");
        vsatOnSimState = false;
        digitalWrite(OFF1, HIGH);
        vTaskDelay(20);
        digitalWrite(OFF1, LOW);
      }
      else
      {
        if(averageVoltage >= (voltageAlarm.vsatAlarmVoltage + 4))
        {
          vsatOn = true;
        }      
      }
    }
    
    if (btsOn) //if bts should be on
    {
      if(!additionalCanData.relayState.bts || !btsOnSimState) //if feedback still off, re-trigger relay
      {
        Serial1.println("===Bts On===");
        btsOnSimState = true;
        digitalWrite(ON2, HIGH);
        vTaskDelay(20);
        digitalWrite(ON2, LOW);
      }      
    }
    else //if bts should be off
    {
      if(additionalCanData.relayState.bts || btsOnSimState) //if feedback still on, re-trigger relay
      {
        Serial1.println("===Bts Off===");
        btsOnSimState = false;
        digitalWrite(OFF2, HIGH);
        vTaskDelay(20);
        digitalWrite(OFF2, LOW);
      }
      else
      {
        if(averageVoltage >= (voltageAlarm.btsAlarmVoltage + 4))
        {
          btsOn = true;
        }      
      }
    }

    if (otherOn) //if bts should be on
    {
      
      if(!additionalCanData.relayState.other || !otherOnSimState) //if feedback still off, re-trigger relay
      {
        Serial1.println("===Other On===");  
        otherOnSimState = true;
        digitalWrite(ON3, HIGH);
        vTaskDelay(20);
        digitalWrite(ON3, LOW);
      }
    }
    else //if bts should be off
    {
      if(additionalCanData.relayState.other || otherOnSimState) //if feedback still on, re-trigger relay
      {
        Serial1.println("===Other Off===");  
        otherOnSimState = false;
        digitalWrite(OFF3, HIGH);
        vTaskDelay(20);
        digitalWrite(OFF3, LOW);
      }
      else
      {
        if(averageVoltage >= (voltageAlarm.otherAlarmVoltage + 4))
        {
          otherOn = true;
        }
      }
    }

    vTaskDelay(500);
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
      can.send(&msg);
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
  
  uint32_t bank1, bank2;
  bank1 = 0x12345678 << 3;
  bank1 = bank1 + 0x04; // Ext
  bank2 = 0xFFFFFFFC; // Must be IDE=1
  // can.setFilter(0, 1, 0, 0, bank1, bank2);
  // can.setFilter(0, 1, 0, 0, 0x02, 0x1FFFFFFF);
  FilterConfig filterConfig = {
    .idConfig = {
      .id = 0x760C860,
      .ideMode = STANDARD_FORMAT,
    },
    .maskConfig = {
      .mask = 0xFF8FFF0,
      .ideCheck = FilterConfig::IdeCheck::IDE_UNCHECKED,
      .rtrCheck = FilterConfig::RtrCheck::RTR_UNCHECKED,
    }
  };
  can.filter(filterConfig);
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