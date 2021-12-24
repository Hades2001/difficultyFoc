#ifndef _U_SERIAL_H_
#define _U_SERIAL_H_

#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_utils.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define USED_DMA_TRANSMISSION   1

#define TRANSMISSION_DIR_P2M    0
#define TRANSMISSION_DIR_M2P    1

enum {
    kSerial_IDEL = 0,
    kSerial_HA,
    kSerial_DIR,
    kSerial_CMD,
    kSerial_Lengt,
    kSerial_DATA,
    kSerial_CRC,
    kSerial_End,
    kSerial_STATE_MAX
};

typedef struct transmission_pack
{
    uint8_t id;
    uint8_t cmd;
    uint8_t length;
    uint8_t *data;
    uint8_t crc;
}transmission_pack_t;

typedef struct
{
    USART_TypeDef *huart;
    int state;
    uint16_t data_length;
    uint16_t data_cnt;

    uint8_t tx_buff[256];
    uint8_t rx_buff[256]; 
    uint8_t available;

    transmission_pack_t revice_pack;

    int16_t (*isAvailable)();
    void (*clearRDFlag)();
    int (*revicePack)(transmission_pack_t *pack_ptr);
    void (*sendPack)(uint8_t id,uint8_t cmd,uint8_t length,void *data);
    void (*printf)(const char *format, ...);
    void (*log)(int type,const char *format, ...);

}serial_t;

void usartIRQCB(uint8_t data);
void InitTransmission(USART_TypeDef *huart);

int16_t _isAvailable(void);
int _revicePack(transmission_pack_t *pack_ptr);
void _sendPack(uint8_t id,uint8_t cmd,uint8_t length,void *data);

extern serial_t serial;

#endif
