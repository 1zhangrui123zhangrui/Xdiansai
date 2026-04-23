#ifndef __24L01_H
#define __24L01_H
#include "stm32f1xx_hal.h"

//NRF24L01寄存器操作命令
#define NRF_READ_REG    0x00
#define NRF_WRITE_REG   0x20
#define RD_RX_PLOAD     0x61
#define WR_TX_PLOAD     0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define NOP             0xFF

//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define MAX_TX          0x10
#define TX_OK           0x20
#define RX_OK           0x40
#define OBSERVE_TX      0x08
#define CD              0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define NRF_FIFO_STATUS 0x17

// GPIO映射: CE->PA4, CSN->PA3, IRQ->PA2
#define NRF24L01_CE_0   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define NRF24L01_CE_1   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define NRF24L01_CSN_0  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define NRF24L01_CSN_1  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
#define NRF24L01_IRQ    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)

#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  4
#define RX_PLOAD_WIDTH  4

// 全局收发缓冲区，在main.c中使用
extern uint8_t NRF24L01_TxPacket[TX_PLOAD_WIDTH];
extern uint8_t NRF24L01_RxPacket[RX_PLOAD_WIDTH];

void NRF24L01_Init(void);
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_TxPacket_Send(uint8_t *txbuf);
uint8_t NRF24L01_RxPacket_Recv(uint8_t *rxbuf);
void NRF24L01_Send(void);
uint8_t NRF24L01_Receive(void);

#endif
