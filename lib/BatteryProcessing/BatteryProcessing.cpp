#include <BatteryProcessing.h>

BatteryProcessing::BatteryProcessing()
{

}

/**
 * @brief   method to get pack voltage from battery can data
 * @param   data    an array with can bus data
 * @param   startIndex  start index of the array
 * @param   length  number of bytes that need to be concatenated
 * @param   maxValue the maxValue to calibrate the result
 * @return  value of pack voltage in 0.01V. e.g 512 means 51.20V
*/
int BatteryProcessing::getPackVoltage(CAN_msg_t msg, int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;

    if ((startIndex + length) > msg.len)
    {
        return 0;
    }

    uint8_t lowByte = msg.data[startIndex];
    uint8_t highByte = msg.data[startIndex+1];

    if (lowByte > 0x64) // if the lowbyte is higher than 100 (0x64), decrease the highbyte by 1
    {
        highByte--;
    }

    raw = (highByte << 8) + lowByte; 
    result = abs(maxValue - raw);
    return result;
}

/**
 * @brief   method to get pack current from battery can data
 * @param   data    an array with can bus data
 * @param   startIndex  start index of the array
 * @param   length  number of bytes that need to be concatenated
 * @param   maxValue the maxValue to calibrate the result
 * @return  current value in 0.1 A, e.g value 98 means 9.8 A
*/
int BatteryProcessing::getPackCurrent(CAN_msg_t msg, int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;

    if ((startIndex + length) > msg.len)
    {
        return 0;
    }

    uint8_t lowByte = msg.data[startIndex];
    uint8_t highByte = msg.data[startIndex+1];

    if (lowByte > 0x64) // if the lowbyte is higher than 100 (0x64), decrease the highbyte by 1
    {
        highByte--;
    }

    raw = (highByte << 8) + lowByte; 
    result = abs(maxValue - raw);
    return result;
}

/**
 * @brief   method to get pack SoC from battery can data
 * @param   data    an array with can bus data
 * @param   startIndex  start index of the array
 * @param   length  number of bytes that need to be concatenated
 * @param   maxValue the maxValue to calibrate the result
 * @return  SoC value in 0.1 percent, e.g value 99 means it is 9.9%
*/
int BatteryProcessing::getPackSoc(CAN_msg_t msg, int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;

    if ((startIndex + length) > msg.len)
    {
        return 0;
    }

    uint8_t lowByte = msg.data[startIndex];
    uint8_t highByte = msg.data[startIndex+1];

    if (lowByte > 0x64) // if the lowbyte is higher than 100 (0x64), decrease the highbyte by 1
    {
        highByte--;
    }

    raw = (highByte << 8) + lowByte; 
    result = abs(maxValue - raw);
    return result;
}

/**
 * @brief   method to get pack temperature from battery can data
 * @param   data    an array with can bus data
 * @param   startIndex  start index of the array
 * @param   length  number of bytes that need to be concatenated
 * @param   maxValue the maxValue to calibrate the result
 * @return  the value in celcius
*/
int BatteryProcessing::getTemperature(CAN_msg_t msg, int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;

    if((startIndex + length) > msg.len)
    {
        return 0;
    }

    for (size_t i = 0; i < length; i++)
    {
        int val = 0;
        val = (msg.data[startIndex + i] << (i*8));
        raw += val;
    }
    result = abs(maxValue - raw);
    return result;
}

/**
 * @brief   method to get mosfet status (dmos and cmos) of battery from can data
 * @param   data    an array with can bus data
 * @param   batteryData struct of battery data, refer to CANDef.h for more information
*/
void BatteryProcessing::updateMosfetStatus(uint8_t data, BatteryData &batteryData)
{
    switch (data)
    {
    case 0x53:
        batteryData.mosfetStatus.cmos = 1;
        batteryData.mosfetStatus.dmos = 0;
        break;
    case 0x42:
        batteryData.mosfetStatus.cmos = 0;
        batteryData.mosfetStatus.dmos = 1;
        break;
    case 0x31:
        // Serial1.println("Data 0x31");
        batteryData.mosfetStatus.cmos = 1;
        batteryData.mosfetStatus.dmos = 1;
        break;
    case 0x65:
        batteryData.mosfetStatus.cmos = 0;
        batteryData.mosfetStatus.dmos = 0;
        break;
    }
}

/**
 * @brief   method to update status of the HalfMosfetData struct
 * @param   data    can msg data
 * @param   halfMosfetData  HalfMosfetData struct data
*/
void BatteryProcessing::updateMosfetStatus(uint8_t data, HalfMosfetData &halfMosfetData)
{
    switch (data)
    {
    case 0x53:
        halfMosfetData.mosfetStatus.cmos = 1;
        halfMosfetData.mosfetStatus.dmos = 0;
        break;
    case 0x42:
        halfMosfetData.mosfetStatus.cmos = 0;
        halfMosfetData.mosfetStatus.dmos = 1;
        break;
    case 0x31:
        // Serial1.println("Data 0x31");
        halfMosfetData.mosfetStatus.cmos = 1;
        halfMosfetData.mosfetStatus.dmos = 1;
        break;
    case 0x65:
        halfMosfetData.mosfetStatus.cmos = 0;
        halfMosfetData.mosfetStatus.dmos = 0;
        break;
    }
}