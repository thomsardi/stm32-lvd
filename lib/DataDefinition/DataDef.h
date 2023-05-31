#ifndef DATADEF_H
#define DATADEF_H

#include <Arduino.h>

struct BatteryData {
    
    union MosfetStatus 
    {
        struct
        {
            uint8_t cmos : 1;
            uint8_t dmos : 1;
            uint8_t reserved : 6;
        };
        uint8_t val;
    };

    struct Temperature
    {
        int top;
        int mid;
        int bot;
        int cmosTemp;
        int dmosTemp;  
    };

    struct UpdatedCounter
    {
        uint16_t previousTemperatureUpdatedCounter;
        uint16_t temperatureUpdatedCounter;
        uint16_t previousMosfetUpdatedCounter;
        uint16_t mosfetUpdatedCounter;
        uint16_t previousPackUpdatedCounter;
        uint16_t packUpdatedCounter;
    };

    MosfetStatus mosfetStatus;
    UpdatedCounter cnt;
    Temperature temperature;
    int packVoltage;
    int packCurrent;
    int packSoc;
    bool isUpdated;

    void updateTemperatureCounter() {
        cnt.previousTemperatureUpdatedCounter = cnt.temperatureUpdatedCounter;
        cnt.temperatureUpdatedCounter++;
    }

    void updateMosfetCounter() {
        cnt.previousMosfetUpdatedCounter = cnt.mosfetUpdatedCounter;
        cnt.mosfetUpdatedCounter++;
    }

    void updatePackCounter() {
        cnt.previousPackUpdatedCounter = cnt.packUpdatedCounter;
        cnt.packUpdatedCounter++;
    }
};

struct AlarmParameter
{
    int vsatAlarmVoltage;
    int otherAlarmVoltage;
    int btsAlarmVoltage;
    int nominalBattery = 480;
    uint16_t tolerance = 200; // in .01 percent
    
    int getVsatUpperThreshold() {
        return vsatAlarmVoltage + (nominalBattery * tolerance / 10000);
    }
    int getOtherUpperThreshold() {
        return otherAlarmVoltage + (nominalBattery * tolerance / 10000);
    }
    int getBtsUpperThreshold() {
        return btsAlarmVoltage + (nominalBattery * tolerance / 10000);
    }

};


struct AdditionalCANData {
    union {
        struct {
            uint8_t vsat: 1;           /*This is the raw bit for line0 status.*/
            uint8_t bts: 1;           /*This is the raw bit for line1 status.*/
            uint8_t other: 1;           /*This is the raw bit for line2 status.*/
            uint8_t reserved: 5;
        };
        uint8_t val;
    } relayState; 
    float current[3];
};

union EhubRelayWrite {
    struct 
    {
        uint8_t vsat : 1;
        uint8_t bts : 1;
        uint8_t other : 1;
        uint8_t :5;
    };
    uint8_t val;
};

struct KeepAliveCounter {
    uint32_t cnt;
    uint32_t lastCnt;
};

enum DataType {
    PACK_DATA = 0x764C840,
    MOSF_TEMP = 0x763C840,
    VCELL_1_4 = 0x762C840,
    VCELL_5_8 = 0x761C840,
    VCELL_9_12 = 0x760C840,
    VCELL_13_15 = 0x75FC840,
    MAX_MIN_PARAM = 0x75EC840
};

enum BaseAddress {
    PACK_ADDR = 0x764C864,
    MOSF_TEMP_ADDR = 0x763C864,
    VCELL_1_4_ADDR = 0x762C864,
    VCELL_5_8_ADDR = 0x761c864,
    VCELL_9_12_ADDR = 0x760c864,
    VCELL_13_15_ADDR = 0x75fc864,
    MAX_MIN_PARAM_ADDR = 0x75ec864
};

#endif