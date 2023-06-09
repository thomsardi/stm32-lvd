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
    uint16_t vsatLowVoltage = 4600;   //in 0.1V
    uint16_t otherLowVoltage = 4600;  //in 0.1V
    uint16_t btsLowVoltage = 4700;    //in 0.1V
    uint16_t vsatReconnectVoltage = 4700;
    uint16_t otherReconnectVoltage = 4700;
    uint16_t btsReconnectVoltage = 4800;
    uint16_t nominalBattery = 4800; // in 0.1V. 480 means 48V
    uint16_t tolerance = 200; // in .01 percent. 200 means 2%
    
    int getVsatUpperThreshold() {
        return vsatLowVoltage + (nominalBattery * tolerance / 10000);
    }
    int getOtherUpperThreshold() {
        return otherLowVoltage + (nominalBattery * tolerance / 10000);
    }
    int getBtsUpperThreshold() {
        return btsLowVoltage + (nominalBattery * tolerance / 10000);
    }

};

/**
 * @brief additional data to be append into can
*/
struct AdditionalCANData {
    union {
        struct {
            uint8_t other: 1;       /*This is the raw bit for other relay status.*/
            uint8_t bts: 1;         /*This is the raw bit for bts relay0 status.*/
            uint8_t vsat: 1;        /*This is the raw bit for vsat relay status.*/            
            uint8_t reserved: 5;    /*Reserved, not used*/    
        };
        uint8_t val;
    } relayState; 
    float current[3];
};

union EhubRelayWrite {
    struct 
    {
        uint8_t vsat : 1;       /*vsat relay write register*/
        uint8_t bts : 1;        /*bts relay write register*/
        uint8_t other : 1;      /*other relay write register*/
        uint8_t :5;
    };
    uint8_t val;
};

/**
 * @brief   struct for keep alive counter
 *          @note this contain the counter which will get updated 
*/
struct KeepAliveCounter {
    uint32_t cnt;
    uint32_t lastCnt;
};

/**
 * @brief   enum of frame id of specified can data
*/
enum DataType {
    PACK_DATA = 0x764C840,
    MOSF_TEMP = 0x763C840,
    VCELL_1_4 = 0x762C840,
    VCELL_5_8 = 0x761C840,
    VCELL_9_12 = 0x760C840,
    VCELL_13_15 = 0x75FC840,
    MAX_MIN_PARAM = 0x75EC840
};

/**
 * @brief   the base address of each battery data set
*/
enum BaseAddress {
    PACK_ADDR = 0x764C864,
    MOSF_TEMP_ADDR = 0x763C864,
    VCELL_1_4_ADDR = 0x762C864,
    VCELL_5_8_ADDR = 0x761c864,
    VCELL_9_12_ADDR = 0x760c864,
    VCELL_13_15_ADDR = 0x75fc864,
    MAX_MIN_PARAM_ADDR = 0x75ec864,
    WAKE_ADDR = 0x12640066
};

#endif