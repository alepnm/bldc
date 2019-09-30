#include "usart.h"


Port_TypeDef Ports[2];
const uint32_t baudrates[7] = { 2400u, 4800u, 9600u, 14400u, 19200u, 38400u, 57600u };


void USART_PortsInit(void){

    Ports[SECONDARY_PORT].handle = USART2;
    Ports[SECONDARY_PORT].Config.MbAddr = 0;
    Ports[SECONDARY_PORT].Config.Baudrate = BR19200;
    Ports[SECONDARY_PORT].Config.Parity = PARITY_NONE;
    Ports[SECONDARY_PORT].Config.StopBits = STOPBITS_1;
    Ports[SECONDARY_PORT].Config.DataBits = 8;

    Ports[SECONDARY_PORT].Registers.ePortState = USART_STATE_IDLE;
    Ports[SECONDARY_PORT].Registers.NewMessageReceivedFlag = 0;

    USART_Config(SECONDARY_PORT);

}


/*  */
void USART_Config(uint8_t ucPORT) {

    LL_USART_InitTypeDef USART_InitStruct = {0};

    do {
        LL_USART_Disable(Ports[ucPORT].handle);
    } while( LL_USART_IsEnabled(Ports[ucPORT].handle) );

    USART_InitStruct.BaudRate = baudrates[Ports[ucPORT].Config.Baudrate];

    switch(Ports[ucPORT].Config.Parity) {
    case PARITY_ODD:
        USART_InitStruct.Parity = LL_USART_PARITY_ODD;
    case PARITY_EVEN:
        USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
        break;
    default:
        USART_InitStruct.Parity = LL_USART_PARITY_NONE;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    }

    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(Ports[ucPORT].handle, &USART_InitStruct);

    do {
        LL_USART_Enable(Ports[ucPORT].handle);
    } while( !LL_USART_IsEnabled(Ports[ucPORT].handle) );

    LL_USART_EnableIT_RXNE(Ports[ucPORT].handle);
    LL_USART_DisableIT_TC(Ports[ucPORT].handle);
}


/*  */
void USART_Send( uint8_t ucPORT, void* data, size_t len ) {

    while(len--) {
        while(!LL_USART_IsActiveFlag_TC(Ports[ucPORT].handle));
        LL_USART_TransmitData8(Ports[ucPORT].handle, *((uint8_t*)data++));
    }

    Ports[ucPORT].Registers.ePortState = USART_STATE_ANSWER_WAITING;
    Ports[ucPORT].Registers.PortTimer = 100;
}


/*  */
void USART_SendByte(uint8_t ucPORT, char data) {
    LL_USART_TransmitData8(Ports[ucPORT].handle, data);

    Ports[ucPORT].Registers.ePortState = USART_STATE_ANSWER_WAITING;
    Ports[ucPORT].Registers.PortTimer = 100;
}

/*  */
void USART_SendString( uint8_t ucPORT, const char* str ) {

    uint8_t i = 0;

    while( *(str+i) ) {
        while(!LL_USART_IsActiveFlag_TC(Ports[ucPORT].handle));
        LL_USART_TransmitData8(Ports[ucPORT].handle, *(str+i));
        i++;
    }

    Ports[ucPORT].Registers.ePortState = USART_STATE_ANSWER_WAITING;
    Ports[ucPORT].Registers.PortTimer = 100;
}


/*  */
void USART_ClearRxBuffer(uint8_t ucPORT) {

    uint8_t i = 0;

    while(i < RX_BUFFER_SIZE) {
        Ports[ucPORT].Registers.RxBuffer[i++] = 0;
    }

    Ports[ucPORT].Registers.RxBufferIndex = 0;
    Ports[ucPORT].Registers.ReceivedData = 0;
}


/*  */
void USART_ClearTxBuffer(uint8_t ucPORT) {

    uint8_t i = 0;

    while(i < TX_BUFFER_SIZE) {
        Ports[ucPORT].Registers.TxBuffer[i++] = 0;
    }

    Ports[ucPORT].Registers.TxBufferIndex = 0;
}


/*  */
void USART_IRQ_Handler(uint8_t ucPORT) {

    if( LL_USART_IsActiveFlag_RXNE(Ports[ucPORT].handle) && LL_USART_IsEnabledIT_RXNE(Ports[ucPORT].handle) ) {

        Ports[ucPORT].Registers.ReceivedData = LL_USART_ReceiveData8(Ports[ucPORT].handle);

        *(Ports[ucPORT].Registers.RxBuffer + Ports[ucPORT].Registers.RxBufferIndex) = Ports[ucPORT].Registers.ReceivedData;

        Ports[ucPORT].Registers.RxBufferIndex++;

        Ports[ucPORT].Registers.ePortState = USART_STATE_ANSWER_WAITING;

        Ports[ucPORT].Registers.PortTimer = 10;
    }
}


/*  */
void USART_TimerHandler(uint8_t ucPORT) {

    if(Ports[ucPORT].Registers.PortTimer) Ports[ucPORT].Registers.PortTimer--;
    else {

        if( Ports[ucPORT].Registers.ePortState == USART_STATE_ANSWER_WAITING ) {
            Ports[ucPORT].Registers.ePortState = USART_STATE_IDLE;
            Ports[ucPORT].Registers.RxBufferIndex = 0;

            Ports[ucPORT].Registers.NewMessageReceivedFlag = SET;
        }
    }
}

