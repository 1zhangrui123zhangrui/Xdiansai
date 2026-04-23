#include "main.h"
#include "24l01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};

uint8_t NRF24L01_TxPacket[TX_PLOAD_WIDTH] = {0};
uint8_t NRF24L01_RxPacket[RX_PLOAD_WIDTH] = {0};

extern SPI_HandleTypeDef hspi1;

static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);
    return Rxdata;
}

uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0xA5,0xA5,0xA5,0xA5,0xA5};
    uint8_t i;
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5);
    NRF24L01_Read_Buf(TX_ADDR, buf, 5);
    for (i = 0; i < 5; i++)
        if (buf[i] != 0xA5) break;
    return (i != 5) ? 1 : 0;
}

uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    NRF24L01_CSN_0;
    status = SPI1_ReadWriteByte(reg);
    SPI1_ReadWriteByte(value);
    NRF24L01_CSN_1;
    return status;
}

uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    NRF24L01_CSN_0;
    SPI1_ReadWriteByte(reg);
    reg_val = SPI1_ReadWriteByte(0xFF);
    NRF24L01_CSN_1;
    return reg_val;
}

uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, i;
    NRF24L01_CSN_0;
    status = SPI1_ReadWriteByte(reg);
    for (i = 0; i < len; i++)
        pBuf[i] = SPI1_ReadWriteByte(0xFF);
    NRF24L01_CSN_1;
    return status;
}

uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, i;
    NRF24L01_CSN_0;
    status = SPI1_ReadWriteByte(reg);
    for (i = 0; i < len; i++)
        SPI1_ReadWriteByte(pBuf[i]);
    NRF24L01_CSN_1;
    return status;
}

uint8_t NRF24L01_TxPacket_Send(uint8_t *txbuf)
{
    uint8_t sta;
    uint32_t timeout = 200000;
    NRF24L01_CE_0;
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);
    NRF24L01_CE_1;
    while (NRF24L01_IRQ != 0 && --timeout);
    if (timeout == 0) return 0xFF;
    sta = NRF24L01_Read_Reg(STATUS);
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta);
    if (sta & MAX_TX) {
        NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
        return MAX_TX;
    }
    if (sta & TX_OK)
        return TX_OK;
    return 0xFF;
}

uint8_t NRF24L01_RxPacket_Recv(uint8_t *rxbuf)
{
    uint8_t sta;
    sta = NRF24L01_Read_Reg(STATUS);
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta);
    if (sta & RX_OK) {
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
        return 0;
    }
    return 1;
}

void NRF24L01_RX_Mode(void)
{
    NRF24L01_CE_0;
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F);
    NRF24L01_CE_1;
}

void NRF24L01_TX_Mode(void)
{
    NRF24L01_CE_0;
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)TX_ADDRESS, TX_ADR_WIDTH);
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1A);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E);
    NRF24L01_CE_1;
}

void NRF24L01_Init(void)
{
    NRF24L01_CSN_1;
    NRF24L01_CE_0;
}

void NRF24L01_Send(void)
{
    NRF24L01_TX_Mode();
    NRF24L01_TxPacket_Send(NRF24L01_TxPacket);
    NRF24L01_RX_Mode();
}

uint8_t NRF24L01_Receive(void)
{
    return (NRF24L01_RxPacket_Recv(NRF24L01_RxPacket) == 0) ? 1 : 0;
}
