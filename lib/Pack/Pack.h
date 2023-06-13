#ifndef PACK_H
#define PACK_H

#include <Arduino.h>
#include <Vector>
#include <DataDef.h>

class Pack {
    public :
        Pack(BatteryData batteryData[], size_t maxSize, size_t size = 0);
        ~Pack();
        Vector<BatteryData> stack;
        void insert(const HalfPackData &data);
        void insert(const HalfMosfetData &data);
        BatteryData getData(const int id);
        void remove(const int id);
        void calculate();
        void removeUnusedData();
        void printInfo();
        int32_t getAverageVoltage();
        int getTotalCurrent();
        int getAverageSoc();
        uint8_t getTotalBattery();
        uint8_t getActiveBattery();
        uint8_t getInactiveBattery();
    private :
        uint8_t _activeBattery = 0;
        uint8_t _totalBattery = 0;
        uint8_t _inactiveBattery = 0;
        int32_t _averageVoltage = 0;
        int _totalCurrent = 0;
        int _averageSoc = 0;
};

#endif