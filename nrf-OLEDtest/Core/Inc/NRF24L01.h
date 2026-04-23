#ifndef __HHNRF24L01_H
#define __HHNRF24L01_H 
uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte);
void NRF24L01_GPIO_Init(void);
void NRF24L01_W_CSN(uint8_t BitValue);
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data);
//3.2.读地址RegAddress的寄存器里的数据
uint8_t NRF24L01_ReadReg(uint8_t RegAddress);

void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);

extern uint8_t NRF24L01_TxPacket[];
extern uint8_t NRF24L01_RxPacket[];
uint8_t NRF24L01_Receive(void);
void NRF24L01_Send(void);
void NRF24L01_Init(void);
#endif
