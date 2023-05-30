#ifndef BATTERY_PROCESSING_H
#define BATTERY_PROCESSING_H

#include <Arduino.h>
#include <DataDef.h>

class BatteryProcessing {
    public :
        BatteryProcessing();
        int getTemperature(uint8_t data[], int startIndex, size_t length, int maxValue = 100);
        int getPackVoltage(uint8_t data[], int startIndex, size_t length, int maxValue = 25700);
        int getPackCurrent(uint8_t data[], int startIndex, size_t length, int maxValue = 25700);
        int getPackSoc(uint8_t data[], int startIndex, size_t length, int maxValue = 25700);
        void updateMosfetStatus(uint8_t data, BatteryData &batteryData);
    private :

};

#endif