#include <BatteryProcessing.h>

BatteryProcessing::BatteryProcessing()
{

}

int BatteryProcessing::getPackVoltage(uint8_t data[], int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;
    for (size_t i = 0; i < length; i++)
    {
        int val = 0;
        val = (data[startIndex + i] << (i*8));
        raw += val;
    }
    result = abs(maxValue - raw) * 2;
    return result;
}

int BatteryProcessing::getPackCurrent(uint8_t data[], int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;
    for (size_t i = 0; i < length; i++)
    {
        int val = 0;
        val = (data[startIndex + i] << (i*8));
        raw += val;
    }
    result = abs(maxValue - raw);
    return result;
}

int BatteryProcessing::getPackSoc(uint8_t data[], int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;
    for (size_t i = 0; i < length; i++)
    {
        int val = 0;
        val = (data[startIndex + i] << (i*8));
        raw += val;
    }
    result = abs(maxValue - raw);
    return result;
}

int BatteryProcessing::getTemperature(uint8_t data[], int startIndex, size_t length, int maxValue)
{
    int result = 0;
    int raw = 0;
    for (size_t i = 0; i < length; i++)
    {
        int val = 0;
        val = (data[startIndex + i] << (i*8));
        raw += val;
    }
    result = abs(maxValue - raw);
    return result;
}

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