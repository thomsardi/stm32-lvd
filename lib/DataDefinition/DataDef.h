#ifndef DATADEF_H
#define DATADEF_H

#include <Arduino.h>

struct BatteryData {
    
};

struct AdditionalCANData {
    union {
        struct {
            uint8_t BIT_0: 1;           /*This is the raw bit for line0 status.*/
            uint8_t BIT_1: 1;           /*This is the raw bit for line0 status.*/
            uint8_t BIT_2: 1;           /*This is the raw bit for line0 status.*/
            uint8_t reserved: 5;
        };
        int8_t val;
    } relayState; 
    float current[3];
};

#endif