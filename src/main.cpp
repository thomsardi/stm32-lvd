#include <Arduino.h>
#include <CANController.h>
#include <STM32FreeRTOS.h>
#include <Beastdevices_INA3221.h>
#include <DataDef.h>

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
AdditionalCANData additionalCanData;

// TaskHandle_t relayTaskHandle;
TaskHandle_t canSenderTaskHandle;
// TaskHandle_t ina3221TaskHandle;

SemaphoreHandle_t myLock = xSemaphoreCreateMutex();

// QueueHandle_t relayTaskQueue;
QueueHandle_t canSenderTaskQueue;

uint32_t count;
uint32_t count2;
unsigned long lastTime = 0;
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
  Serial1.println("Count : " + String(count));
  Serial1.print("ID : ");
  Serial1.println(msg.id, HEX);
  if(msg.id == idCanbusEnergyMeter)
  {
    Serial1.println("EnergyMeter Found");
  }
  CAN_FORMAT format = static_cast<CAN_FORMAT>(msg.format);
  switch (format)
  {
  case EXTENDED_FORMAT:
    Serial1.println("Format : Extended");
    break;
  
  default:
    Serial1.println("Format : Standard");
    break;
  }
  CAN_FRAME frame = static_cast<CAN_FRAME>(msg.type);
  switch (frame)
  {
  case REMOTE_FRAME:
    Serial1.println("Type : Remote");
    break;
  
  default:
    Serial1.println("Type : Data");
    break;
  }
  int length = msg.len;
  Serial1.print("Data : ");
  for (size_t i = 0; i < length; i++)
  {
    Serial1.print(msg.data[i], HEX);
    Serial1.print(" ");
  }
  Serial1.println();
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
  bool isOff = true;
  uint8_t receivedData;
  while(1)
  {
    // if (xQueueReceive(relayTaskQueue, &receivedData, portMAX_DELAY) == pdTRUE) 
    // {
    //   Serial1.println("Received Data From Handler Can");
    //   if(xSemaphoreTake(myLock, portMAX_DELAY) == pdTRUE)
    //   {
    //     Serial1.println("Relay Task");
    //     if (isOff)
    //     {
    //       digitalWrite(ON2, HIGH);
    //       vTaskDelay(20);
    //       digitalWrite(ON2, LOW);
    //       isOff = false;
    //     }
    //     else
    //     {
    //       digitalWrite(OFF2, HIGH);
    //       vTaskDelay(20);
    //       digitalWrite(OFF2, LOW);
    //       isOff = true;
    //     }
    //   }
    //   xSemaphoreGive(myLock);
    // }
    // vTaskDelay(100);
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
      Serial1.println("Send CAN current & relay");
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
      Serial1.println("Send CAN to wake up battery");

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
  // xTaskCreate(relayTask,
  //   "RelayTask",
  //   configMINIMAL_STACK_SIZE,
  //   NULL,
  //   tskIDLE_PRIORITY + 4,
  //   &relayTaskHandle);

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
      .id = 0x750C860,
      // .id = 0x0,
      .ideMode = STANDARD_FORMAT,
    },
    .maskConfig = {
      .mask = 0xFF0FFF0,
      // .mask = 0x0,
      .ideCheck = FilterConfig::IdeCheck::IDE_UNCHECKED,
      .rtrCheck = FilterConfig::RtrCheck::RTR_UNCHECKED,
    }
  };
  can.filter(filterConfig);
  vTaskStartScheduler();
}

void loop() {
  can.loop();
  additionalCanData.relayState.BIT_0 = digitalRead(FB1);
  additionalCanData.relayState.BIT_1 = digitalRead(FB2);
  additionalCanData.relayState.BIT_2 = digitalRead(FB3);
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