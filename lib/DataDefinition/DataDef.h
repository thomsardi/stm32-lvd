#ifndef DATADEF_H
#define DATADEF_H

#include <Arduino.h>
#include <Vector.h>

/**
 * @brief   struct that contain the collection of pack data and mosfet data
*/
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
    uint8_t id;
    int packVoltage;
    int packCurrent;
    int packSoc;
    bool isUpdated;
    bool isCmosOverTemperature;
    bool isDmosOverTemperature;

    int16_t getPackCounter()
    {
        return cnt.packUpdatedCounter;
    }
    
    int16_t getLastPackCounter()
    {
        return cnt.previousPackUpdatedCounter;
    }

    void updatePackPreviousCounter()
    {
        cnt.previousPackUpdatedCounter = cnt.packUpdatedCounter;
    }

    int16_t getMosfetCounter()
    {
        return cnt.mosfetUpdatedCounter;
    }
    
    int16_t getLastMosfetCounter()
    {
        return cnt.previousMosfetUpdatedCounter;
    }

    void updateMosfetPreviousCounter()
    {
        cnt.previousMosfetUpdatedCounter = cnt.mosfetUpdatedCounter;
    }

    void incPackCounter() {
        cnt.packUpdatedCounter++;
    }

    void incMosfetCounter() {
        cnt.mosfetUpdatedCounter++;
    }
};

/**
 * @brief   struct to store pack data CAN message
*/
struct HalfPackData {
    uint8_t id;
    bool isUpdated;
    int packVoltage;
    int packCurrent;
    int packSoc;
    uint16_t previousPackUpdatedCounter;
    uint16_t packUpdatedCounter;
};

/**
 * @brief   struct to store mosfet and temperature data CAN message
*/
struct HalfMosfetData {
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

    uint8_t id;
    bool isUpdated;
    bool isCmosOverTemperature;
    bool isDmosOverTemperature;
    MosfetStatus mosfetStatus;
    Temperature temperature;
    uint16_t previousMosfetCounter;
    uint16_t mosfetUpdatedCounter;

};

/**
 * @brief   alarm parameter for cut off voltage and reconnect voltage
*/
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
    WAKE_ADDR = 0x12640066,
};

/**
 * @brief   list of frame id
*/
enum PeriodicAddress {
    INA3221_ADDR = 0x1D40C8E7,
    LOW_VOLTAGE_PARAM_ADDR = 0x1D40C8E8,
    RECONNECT_VOLTAGE_PARAM_ADDR = 0x1D40C8E9,
    WAKE_ADDR_SET1 = 0x1264FE53,
    WAKE_ADDR_SET2 = 0x1263FE53,
    WAKE_ADDR_SET3 = 0x1262FE53,
    CMOS_DMOS_CONTROL_ADDR_1 = 0x19A10066,
    CMOS_DMOS_CONTROL_ADDR_2 = 0x19A1FF53,
    CMOS_DMOS_CONTROL_ADDR_3 = 0x12A40066,
    CMOS_DMOS_CONTROL_ADDR_4 = 0x12A4FF53

};

enum MosfetType {
    CHARGE_MOSFET_TYPE = 0xEB,
    DISCHARGE_MOSFET_TYPE = 0xEA
};

enum MosfetSelector {
    CHARGE_MOSFET_SELECTOR = 0x64,
    DISCHARGE_MOSFET_SELECTOR = 0x63
};

enum MosfetState {
    ON_STATE = 0x0F,
    OFF_STATE = 0xBA
};

#endif