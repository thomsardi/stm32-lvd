#ifndef CANCONTROLLER_H
#define CANCONTROLLER_H

#include <Arduino.h>
#include "CANDef.h"

// #define idCanbusEnergyMeter 490784999

class CANController {
    public:
        CANController();
        void loop();
        bool filter(const FilterConfig &filterConfig);
        void setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);
        bool init(BITRATE bitrate, int remap);
        void receive(CAN_msg_t *CAN_rx_msg);
        void send(CAN_msg_t *CAN_tx_msg);
        uint8_t available(void);
        void setup();
        void sendData(CAN_msg_t *txMsg);
        void receiveData();
        void onReceive(void (*handler)(CAN_msg_t));
    private:
        void (*_handler)(CAN_msg_t) = nullptr;
};


#endif