#include "CANController.h"

CANController::CANController()
{

}

/**
 * @brief   set filter for CAN bus, pass the config structure into this method to configure filter
 * @param   filterConfig
 *          @note   has a member of idConfig struct and maskConfig
*/
bool CANController::filter(const FilterConfig &filterConfig)
{
    uint32_t bank1, bank2;
    // bank2 = mask << 3;
    
    switch (filterConfig.idConfig.ideMode)
    {
        case FilterConfig::CanFormatAcceptanceMode::EXTENDED_FORMAT :
            if (filterConfig.idConfig.id > 0x1FFFFFFF)
            {
                return 0;
            }
            bank1 = filterConfig.idConfig.id << 3;
            bank1 = bank1 + (1 << 2);
            break;
        case FilterConfig::CanFormatAcceptanceMode::STANDARD_FORMAT :
            if (filterConfig.idConfig.id > 0x7FF)
            {
                return 0;
            }
            bank1 = filterConfig.idConfig.id << 21;
            break;

        default:
            break;
    }

    switch (filterConfig.idConfig.rtrMode)
    {
        case CAN_FRAME::REMOTE_FRAME :
            bank1 = bank1 + (1 << 1);
            break;
        
        default:
            break;
    }

    if (filterConfig.maskConfig.mask > 0x7FFFFFFF)
    {
        return 0;
    }

    bank2 = filterConfig.maskConfig.mask << 3;
    // bank1 = filterConfig.idConfig.id;
    // bank2 = filterConfig.maskConfig.mask;

    switch (filterConfig.maskConfig.ideCheck)
    {
        case FilterConfig::IdeCheck::IDE_CHECKED :
            bank2 = bank2 + (1 << 2);
            if (filterConfig.maskConfig.ideMode == FilterConfig::IdeMode::EXTENDED_ONLY)
            {
                bank1 = bank1 + (1 << 2);
            }
            break;
        default:
            break;
    }

    switch (filterConfig.maskConfig.rtrCheck)
    {
        case FilterConfig::RtrCheck::RTR_CHECKED :
            bank2 = bank2 + (1 << 1);
            if (filterConfig.maskConfig.rtrMode == FilterConfig::RtrMode::REMOTE_ONLY)
            {
                bank1 = bank1 + (1 << 1);
            }
            break;
        default:
            break;
    }
    

    Serial1.print("Filter bank 1 : ");
    Serial1.println(bank1, HEX);
    Serial1.print("Filter bank 2 : ");
    Serial1.println(bank2, HEX);
    // bank1 = msgId << 3;
    // bank1 = bank1 + 0x04; // Ext
    // bank2 = 0xFFFFFFFC; // Must be IDE=1
    // bank2 = mask << 3;
    setFilter(0, 1, 0, 0, bank1, bank2);
    return 1;
}

/**
 * @brief   init can bus
 * @param   bitrate bit rate of can bus
 * @param   remap   map into certain pin of stm32
 * @return  true when successfully initialize, else will produce false
*/
bool CANController::init(BITRATE bitrate, int remap)
{
    // Reference manual
    // https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf

    RCC->APB1ENR |= 0x2000000UL; // Enable CAN clock
    RCC->APB2ENR |= 0x1UL;       // Enable AFIO clock
    AFIO->MAPR &= 0xFFFF9FFF;    // reset CAN remap
                                 // CAN_RX mapped to PA11, CAN_TX mapped to PA12

    if (remap == 0)
    {
        RCC->APB2ENR |= 0x4UL;      // Enable GPIOA clock
        GPIOA->CRH &= ~(0xFF000UL); // Configure PA12(0b0000) and PA11(0b0000)
                                    // 0b0000
                                    //   MODE=00(Input mode)
                                    //   CNF=00(Analog mode)

        GPIOA->CRH |= 0xB8FFFUL; // Configure PA12(0b1011) and PA11(0b1000)
                                 // 0b1011
                                 //   MODE=11(Output mode, max speed 50 MHz)
                                 //   CNF=10(Alternate function output Push-pull
                                 // 0b1000
                                 //   MODE=00(Input mode)
                                 //   CNF=10(Input with pull-up / pull-down)

        GPIOA->ODR |= 0x1UL << 12; // PA12 Upll-up
    }

    if (remap == 2)
    {
        AFIO->MAPR |= 0x00004000; // set CAN remap
                                  // CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)

        RCC->APB2ENR |= 0x8UL;   // Enable GPIOB clock
        GPIOB->CRH &= ~(0xFFUL); // Configure PB9(0b0000) and PB8(0b0000)
                                 // 0b0000
                                 //   MODE=00(Input mode)
                                 //   CNF=00(Analog mode)

        GPIOB->CRH |= 0xB8UL; // Configure PB9(0b1011) and PB8(0b1000)
                              // 0b1011
                              //   MODE=11(Output mode, max speed 50 MHz)
                              //   CNF=10(Alternate function output Push-pull
                              // 0b1000
                              //   MODE=00(Input mode)
                              //   CNF=10(Input with pull-up / pull-down)

        GPIOB->ODR |= 0x1UL << 8; // PB8 Upll-up
    }

    if (remap == 3)
    {
        AFIO->MAPR |= 0x00005000; // set CAN remap
                                  // CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)

        RCC->APB2ENR |= 0x20UL;  // Enable GPIOD clock
        GPIOD->CRL &= ~(0xFFUL); // Configure PD1(0b0000) and PD0(0b0000)
                                 // 0b0000
                                 //   MODE=00(Input mode)
                                 //   CNF=00(Analog mode)

        GPIOD->CRH |= 0xB8UL; // Configure PD1(0b1011) and PD0(0b1000)
                              // 0b1000
                              //   MODE=00(Input mode)
                              //   CNF=10(Input with pull-up / pull-down)
                              // 0b1011
                              //   MODE=11(Output mode, max speed 50 MHz)
                              //   CNF=10(Alternate function output Push-pull

        GPIOD->ODR |= 0x1UL << 0; // PD0 Upll-up
    }

    CAN1->MCR |= 0x1UL; // Require CAN1 to Initialization mode
    while (!(CAN1->MSR & 0x1UL))
        ; // Wait for Initialization mode

    // CAN1->MCR = 0x51UL;                 // Hardware initialization(No automatic retransmission)
    CAN1->MCR = 0x41UL; // Hardware initialization(With automatic retransmission)

    // Set bit rates
    CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF));
    CAN1->BTR |= (((can_configs[bitrate].TS2 - 1) & 0x07) << 20) | (((can_configs[bitrate].TS1 - 1) & 0x0F) << 16) | ((can_configs[bitrate].BRP - 1) & 0x1FF);

    // Configure Filters to default values
    // CAN1->FMR |= 0x1UL;      // Set to filter initialization mode
    // CAN1->FMR &= 0xFFFFC0FF; // Clear CAN2 start bank

    // bxCAN has 28 filters.
    // These filters are used for both CAN1 and CAN2.
    // STM32F103 has only CAN1, so all 28 are used for CAN1
    // CAN1->FMR |= 0x1C << 8; // Assign all filters to CAN1

    // Set fileter 0
    // Single 32-bit scale configuration
    // Two 32-bit registers of filter bank x are in Identifier Mask mode
    // Filter assigned to FIFO 0
    // Filter bank register to all 0
    setFilter(0, 1, 0, 0, 0x0UL, 0x0UL);
    

    // CAN1->FMR &= ~(0x1UL); // Deactivate initialization mode

    uint16_t TimeoutMilliseconds = 1000;
    bool can1 = false;
    CAN1->MCR &= ~(0x1UL); // Require CAN1 to normal mode

    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++)
    {
        if ((CAN1->MSR & 0x1UL) == 0)
        {
            can1 = true;
            break;
        }
        delayMicroseconds(1000);
    }
    // Serial1.print("can1=");
    // Serial1.println(can1);

    if(!can1)
    {
        return false;
    }

    // if (can1)
    // {
    //     Serial1.println("CAN1 initialize ok");
    // }
    // else
    // {
    //     Serial1.println("CAN1 initialize fail!!");
    //     return false;
    // }

    return true;
}

/**
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */
void CANController::setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2)
{
    if (index > 27)
        return;

    CAN1->FA1R &= ~(0x1UL << index); // Deactivate filter

    // Configure Filters to default values
    CAN1->FMR |= 0x1UL;      // Set to filter initialization mode
    CAN1->FMR &= 0xFFFFC0FF; // Clear CAN2 start bank

    // bxCAN has 28 filters.
    // These filters are used for both CAN1 and CAN2.
    // STM32F103 has only CAN1, so all 28 are used for CAN1
    CAN1->FMR |= 0x1C << 8; // Assign all filters to CAN1

    if (scale == 0)
    {
        CAN1->FS1R &= ~(0x1UL << index); // Set filter to Dual 16-bit scale configuration
    }
    else
    {
        CAN1->FS1R |= (0x1UL << index); // Set filter to single 32 bit configuration
    }
    if (mode == 0)
    {
        CAN1->FM1R &= ~(0x1UL << index); // Set filter to Mask mode
    }
    else
    {
        CAN1->FM1R |= (0x1UL << index); // Set filter to List mode
    }

    if (fifo == 0)
    {
        CAN1->FFA1R &= ~(0x1UL << index); // Set filter assigned to FIFO 0
    }
    else
    {
        CAN1->FFA1R |= (0x1UL << index); // Set filter assigned to FIFO 1
    }

    CAN1->sFilterRegister[index].FR1 = bank1; // Set filter bank registers1
    CAN1->sFilterRegister[index].FR2 = bank2; // Set filter bank registers2

    CAN1->FA1R |= (0x1UL << index); // Activate filter
    CAN1->FMR &= ~(0x1UL); // Deactivate initialization mode
}

/**
 * Decodes CAN messages from the data registers and populates a
 * CAN message struct with the data fields.
 *
 * @precondition A valid CAN message is received
 * @param CAN_rx_msg - CAN message structure for reception
 *
 */
void CANController::receive(CAN_msg_t *CAN_rx_msg)
{
    uint32_t id = CAN1->sFIFOMailBox[0].RIR;
    if ((id & STM32_CAN_RIR_IDE) == 0)
    { // Standard frame format
        CAN_rx_msg->format = STANDARD_FORMAT;
        ;
        CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
    }
    else
    { // Extended frame format
        CAN_rx_msg->format = EXTENDED_FORMAT;
        ;
        CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
    }

    if ((id & STM32_CAN_RIR_RTR) == 0)
    { // Data frame
        CAN_rx_msg->type = DATA_FRAME;
    }
    else
    { // Remote frame
        CAN_rx_msg->type = REMOTE_FRAME;
    }

    CAN_rx_msg->len = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;

    CAN_rx_msg->data[0] = 0xFFUL & CAN1->sFIFOMailBox[0].RDLR;
    CAN_rx_msg->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
    CAN_rx_msg->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
    CAN_rx_msg->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
    CAN_rx_msg->data[4] = 0xFFUL & CAN1->sFIFOMailBox[0].RDHR;
    CAN_rx_msg->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
    CAN_rx_msg->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
    CAN_rx_msg->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);

    // Release FIFO 0 output mailbox.
    // Make the next incoming message available.
    CAN1->RF0R |= 0x20UL;
}

/**
 * Encodes CAN messages using the CAN message struct and populates the
 * data registers with the sent.
 *
 * @param CAN_tx_msg - CAN message structure for transmission
 *
 */
void CANController::send(CAN_msg_t *CAN_tx_msg)
{
    volatile int count = 0;

    uint32_t out = 0;
    if (CAN_tx_msg->format == EXTENDED_FORMAT)
    { // Extended frame format
        out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
    }
    else
    { // Standard frame format
        out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
    }

    // Remote frame
    if (CAN_tx_msg->type == REMOTE_FRAME)
    {
        out |= STM32_CAN_TIR_RTR;
    }

    CAN1->sTxMailBox[0].TDTR &= ~(0xF);
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;

    CAN1->sTxMailBox[0].TDLR = (((uint32_t)CAN_tx_msg->data[3] << 24) |
                                ((uint32_t)CAN_tx_msg->data[2] << 16) |
                                ((uint32_t)CAN_tx_msg->data[1] << 8) |
                                ((uint32_t)CAN_tx_msg->data[0]));
    CAN1->sTxMailBox[0].TDHR = (((uint32_t)CAN_tx_msg->data[7] << 24) |
                                ((uint32_t)CAN_tx_msg->data[6] << 16) |
                                ((uint32_t)CAN_tx_msg->data[5] << 8) |
                                ((uint32_t)CAN_tx_msg->data[4]));

    // Send Go
    CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

    // Wait until the mailbox is empty
    while (CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000)
        ;

    // The mailbox don't becomes empty while loop
    if (CAN1->sTxMailBox[0].TIR & 0x1UL)
    {
        Serial1.println("Send Fail");
        Serial1.println(CAN1->ESR);
        Serial1.println(CAN1->MSR);
        Serial1.println(CAN1->TSR);
    }
}

/**
 * @brief   detect can packet data
 * @return  data received in buffer as uint8_t format
*/
uint8_t CANController::available(void)
{
    // Check for pending FIFO 0 messages
    return CAN1->RF0R & 0x3UL;
}

// void CANController::setup()
// {
//     bool ret = init(CAN_250KBPS, 0); // CAN_RX mapped to PA11, CAN_TX mapped to PA12
//     if (!ret)
//         while (true)
//         {
//         };
// }

/**
 * @brief   send data of TX message buffer
 * @param   txMsg   message buffer, refer to CAN_msg_t structure
*/
void CANController::sendData(CAN_msg_t *txMsg)
{
    // CAN_msg_t CAN_TX_msg;

    // CAN_TX_msg.data[0] = dataCanbus[0];
    // CAN_TX_msg.data[1] = dataCanbus[1];
    // CAN_TX_msg.data[2] = dataCanbus[2];
    // CAN_TX_msg.data[3] = dataCanbus[3];
    // CAN_TX_msg.data[4] = dataCanbus[4];
    // CAN_TX_msg.data[5] = dataCanbus[5];
    // CAN_TX_msg.data[6] = dataCanbus[6];
    // CAN_TX_msg.data[7] = dataCanbus[7];

    // CAN_TX_msg.len = 8;

    // CAN_TX_msg.format = EXTENDED_FORMAT;
    // CAN_TX_msg.id = idCanbusEnergyMeter;
    send(txMsg);
}


// void CANController::receiveData()
// {
//     CAN_msg_t CAN_RX_msg;

//     if (available())
//     {
//         receive(&CAN_RX_msg);

//         // check id from canbus 1D40C8E8 or 490785000

//         if (CAN_RX_msg.id == idCanbusEnergyMeter + 1)
//         {
//             if (CAN_RX_msg.format == EXTENDED_FORMAT)
//             {
//                 int data1 = CAN_RX_msg.data[0];

//                 // Serial1.println("Data masuk : " + String(data1));

//                 // controlRelay(data1);
//             }
//         }
//     }
// }

/**
 * @brief   method to regularly check availability data on can bus line, this method must be called in routine or else the data will not get updated
*/
void CANController::loop()
{
    CAN_msg_t rxMsg;
    if(available())
    {
        // Serial1.println("Incoming Data");
        receive(&rxMsg);
        if(_handler != nullptr)
        {
            _handler(rxMsg);
        }
    }
}

/**
 * @brief   register handler / callback when can packet data is arrived
 *          @note   take a funtion pointer that receive CAN_msg_t and return nothing
 * @param   handler function which return nothing and takes a CAN_msg_t as parameter
*/
void CANController::onReceive(void (*handler)(CAN_msg_t))
{
    _handler = handler;
}