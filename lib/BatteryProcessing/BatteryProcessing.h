#ifndef BATTERY_PROCESSING_H
#define BATTERY_PROCESSING_H

#include <Arduino.h>
#include <DataDef.h>
#include <CANDef.h>

class BatteryProcessing {
    public :
        BatteryProcessing();
        int getTemperature(CAN_msg_t msg, int startIndex, size_t length, int maxValue = 100);
        int getPackVoltage(CAN_msg_t msg, int startIndex, size_t length, int maxValue = 25700);
        int getPackCurrent(CAN_msg_t msg, int startIndex, size_t length, int maxValue = 25700);
        int getPackSoc(CAN_msg_t msg, int startIndex, size_t length, int maxValue = 25700);
        // void updateMosfetStatus(uint8_t data, BatteryData &batteryData);
        void updateMosfetStatus(uint8_t data, BatteryData &batteryData);
        void updateMosfetStatus(uint8_t data, HalfMosfetData &halfMosfetData);
    private :

};

#endif