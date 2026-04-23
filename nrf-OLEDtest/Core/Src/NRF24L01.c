#include "gpio.h"
#include "NRF24L01_Define.h"    
/*1、引脚配置*/
//1.1.CE引脚
void NRF24L01_W_CE(GPIO_PinState BitValue)
{
//	GPIO_WriteBit(GPIOA, GPIO_Pin_0, (BitAction)BitValue);
	HAL_GPIO_WritePin(GPIOA, PA0_Pin, BitValue);
}
//1.2.片选引脚CSN
void NRF24L01_W_CSN(GPIO_PinState BitValue)
{
	HAL_GPIO_WritePin(GPIOA, PA1_Pin, BitValue);
}
//1.3.SPI时钟信号引脚
void NRF24L01_W_SCK(GPIO_PinState BitValue)
{
	HAL_GPIO_WritePin(GPIOA, PA2_Pin, BitValue);
}
//1.4.SPI MOSI引脚
void NRF24L01_W_MOSI(GPIO_PinState BitValue)
{
	HAL_GPIO_WritePin(GPIOA, PA3_Pin, BitValue);
}
//1.5.SPI MISO引脚
uint8_t NRF24L01_R_MISO(void)
{
	uint8_t BitValue;
	if (HAL_GPIO_ReadPin(GPIOA, PA4_Pin)==GPIO_PIN_RESET){
		BitValue=0;
	}else{
		BitValue=1;
	}
	return BitValue;
}
//1.5.初始化GPIO
void NRF24L01_GPIO_Init(void)
{
	NRF24L01_W_CE(0);
	NRF24L01_W_CSN(1);
	NRF24L01_W_SCK(0);
	NRF24L01_W_MOSI(0);
}


/*2、通信协议*/
uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte)
{
	uint8_t i;
	
	for (i = 0; i < 8; i ++)
	{
		//SPI移出数据
		if (Byte & 0x80)//取出最高位，位操作可以查看《999.STM32中的C语言.md》
		{
			NRF24L01_W_MOSI(1);//最高位假如是1，那么把MOSI引脚拉高
		}
		else
		{
			NRF24L01_W_MOSI(0);//最高位假如是0，那么把MOSI引脚拉低
		}
		Byte <<= 1;//左移一位
		
		//SCK置高电平
		NRF24L01_W_SCK(1);
		
		//SPI移入数据
		if (NRF24L01_R_MISO())//读取MISO引脚，假如是1，则给Byte最低位置1
		{
			Byte |= 0x01;
		}
	//	else
	//	{
	//		Byte &= ~0x01;//读取MISO引脚，假如是0，则给Byte最低位置0，由于前面Byte <<= 1，左移后最低位本身就是0，所以不加这个也可以
	//	}
		
		//SPI置低电平
		NRF24L01_W_SCK(0);//准备交换下一位
	}
	return Byte;
}

/*3、指令实现*/
//3.1.往地址RegAddress的寄存器里写入Data
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	NRF24L01_SPI_SwapByte(Data);
	NRF24L01_W_CSN(1);
}
//3.2.读地址RegAddress的寄存器里的单个字节数据
uint8_t NRF24L01_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	Data = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	NRF24L01_W_CSN(1);
	return Data;
}
//3.3.往地址RegAddress的寄存器里写入多个字节DataArray
void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	for (i = 0; i < Count; i ++)
	{
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	NRF24L01_W_CSN(1);
} 
//3.4.读地址RegAddress的寄存器里的多个字节数据
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	for (i = 0; i < Count; i ++)
	{
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	NRF24L01_W_CSN(1);
}
/*4、初级功能代码*/
//4.1.写发送的有效载荷
void NRF24L01_WriteTxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_W_TX_PAYLOAD);
	for (i = 0; i < Count; i ++)
	{
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	NRF24L01_W_CSN(1);
}
//4.2.读接收有效载荷
void NRF24L01_ReadRxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_R_RX_PAYLOAD);
	for (i = 0; i < Count; i ++)
	{
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	NRF24L01_W_CSN(1);
}
//4.3.清空发送FIFO
void NRF24L01_FlushTx(void)
{
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_TX);
	NRF24L01_W_CSN(1);
}
//4.4.清空接收FIFO
void NRF24L01_FlushRx(void)
{
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_RX);
	NRF24L01_W_CSN(1);
}
//4.5.读取状态
uint8_t NRF24L01_ReadStatus(void)
{
	uint8_t Status;
	NRF24L01_W_CSN(0);
	Status = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	NRF24L01_W_CSN(1);
	return Status;
}
/*5、中级功能代码*/
//5.1.让其进入PowerDown模式
void NRF24L01_PowerDown(void)
{ 
	NRF24L01_W_CE(0);//CE引脚控制
	uint8_t Config;
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);//读寄存器
	Config &= ~0x02;//修改一位
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);//写寄存器
}
//5.2.让其进入StandbyI模式
void NRF24L01_StandbyI(void)
{
	NRF24L01_W_CE(0);
	uint8_t Config;  
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	Config |= 0x02;
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);
}
//5.3.让其进入接收模式
void NRF24L01_RxMode(void)
{
	NRF24L01_W_CE(0);
	uint8_t Config;  
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	Config |= 0x03;
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);
	
	NRF24L01_W_CE(1);
}
//5.4.让其进入发送模式
void NRF24L01_TxMode(void)
{ 
	NRF24L01_W_CE(0);
	uint8_t Config; 
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	Config |= 0x02;
	Config &= ~0x01;
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config); 
	NRF24L01_W_CE(1);
}
/*6、高级功能代码*/
//6.1、芯片初始化


/*发送部分变量定义*/
uint8_t NRF24L01_TxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};		//发送地址，固定5字节
#define NRF24L01_TX_PACKET_WIDTH		4							//发送数据包宽度，范围：1~32字节
uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];				//发送数据包

/*接收部分变量定义*/
uint8_t NRF24L01_RxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};		//接收通道0地址，固定5字节
#define NRF24L01_RX_PACKET_WIDTH		4							//接收通道0数据包宽度，范围：1~32字节
uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];				//接收数据包

/**
  * 提示：设备A和设备B进行通信
  * A发B收时，A的发送地址、发送数据包宽度要与B的接收地址、接收数据包宽度对应相同
  * B发A收时，B的发送地址、发送数据包宽度要与A的接收地址、接收数据包宽度对应相同
  * 通常情况下，可以将A和B的发送地址、接收地址全设置一样，A和B的发送数据包宽度、接收数据包宽度也全设置一样
  * 这样A和B可以使用完全一样的模块程序，操作更加方便，也不容易搞混
  * 
  */
void NRF24L01_Init(void)
{
	/*先调用底层的端口初始化*/
	NRF24L01_GPIO_Init();
	
	/*初始化配置一系列寄存器，寄存器值的意义需参考手册中的寄存器描述*/
	/*以下配置通信双方必须保持一致，否则无法进行通信*/
	NRF24L01_WriteReg(NRF24L01_CONFIG, 0x08);		//配置寄存器，关闭中断，使能CRC，CRC为1字节，PWR_UP = 0，PRIM_RX = 0
	NRF24L01_WriteReg(NRF24L01_EN_AA, 0x3F);		//使能自动应答，开启接收通道0~通道5的自动应答
	NRF24L01_WriteReg(NRF24L01_EN_RXADDR, 0x01);	//使能接收通道，只开启接收通道0
	NRF24L01_WriteReg(NRF24L01_SETUP_AW, 0x03);		//设置地址宽度，地址宽度为5字节
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0x03);	//设置自动重传，间隔250us，重传3次
	NRF24L01_WriteReg(NRF24L01_RF_CH, 0x02);		//射频通道，频率为(2400 + 2)MHz = 2.402GHz
	NRF24L01_WriteReg(NRF24L01_RF_SETUP, 0x0E);		//射频设置，通信速率为2Mbps，发射功率为0dBm
	
	/*接收通道0的数据包宽度，设置为宏定义NRF24L01_RX_PACKET_WIDTH指定的值*/
	NRF24L01_WriteReg(NRF24L01_RX_PW_P0, NRF24L01_RX_PACKET_WIDTH);
	
	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*清空Rx FIFO的所有数据*/
	NRF24L01_FlushRx();
	
	/*给状态寄存器的位4（MAX_RT）、位5（TX_DS）和位6（RX_DR）写1，清标志位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
	
	/*初始化配置完成，芯片默认进入接收模式*/
	NRF24L01_RxMode();
} 
//6.2、发送
uint8_t NRF24L01_Send(void)
{
	uint8_t Status;
	uint8_t SendFlag;
	uint32_t Timeout;
	
	/*发送地址，设置为全局数组NRF24L01_TxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_TxAddress, 5);
	
	/*接收通道0地址，此处必须也设置为发送地址，用于接收应答*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_TxAddress, 5);
	
	/*写发送有效载荷，写入全局数组NRF24L01_TxPacket指定的数据，数据宽度为NRF24L01_TX_PACKET_WIDTH*/
	NRF24L01_WriteTxPayload(NRF24L01_TxPacket, NRF24L01_TX_PACKET_WIDTH);
	
	/*发送的地址和有效载荷写入完成，进入发送模式，开始发送数据*/
	NRF24L01_TxMode();
	
	/*指定超时时间，即循环读取状态寄存器的次数，具体值可以实测确定*/
	Timeout = 10000;
	
	/*循环读取状态寄存器*/
	while (1)
	{
		/*读取状态寄存器，保存至Status变量*/
		Status = NRF24L01_ReadStatus();
		
		/*超时计次*/
		Timeout --;
		if (Timeout == 0)			//如果计次减至0
		{
			SendFlag = 4;			//发送超时，置标志位为4
			NRF24L01_Init();		//发送出错，重新初始化一次设备，这样有助于设备从错误中恢复正常
			break;					//跳出循环
		}
		
		/*根据状态寄存器的值，判断发送状态*/
		if ((Status & 0x30) == 0x30)		//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
		{
			SendFlag = 3;			//状态寄存器的值不合法，置标志位为3
			NRF24L01_Init();		//发送出错，重新初始化一次设备，这样有助于设备从错误中恢复正常
			break;					//跳出循环
		}
		else if ((Status & 0x10) == 0x10)	//状态寄存器位4（MAX_RT）为1
		{
			SendFlag = 2;			//达到了最大重发次数仍未收到应答，置标志位为2
			NRF24L01_Init();		//发送出错，重新初始化一次设备，这样有助于设备从错误中恢复正常
			break;					//跳出循环
		}
		else if ((Status & 0x20) == 0x20)	//状态寄存器位5（TX_DS）为1
		{
			SendFlag = 1;			//发送成功，无错误，置标志位为1
			break;					//跳出循环
		}
	}
	
	/*给状态寄存器的位4（MAX_RT）和位5（TX_DS）写1，清标志位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x30);
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*发送完成后，恢复接收通道0原来的地址*/
	/*如果发送地址和接收通道0地址设置相同，则可不执行这一句*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
	
	/*发送完成，芯片恢复为接收模式*/
	NRF24L01_RxMode();
		
	/*返回发送标志位*/
	return SendFlag;
}
  
uint8_t NRF24L01_Receive(void)
{
	uint8_t Status, Config;
	uint8_t ReceiveFlag;
	
	/*读取状态寄存器，保存至Status变量*/
	Status = NRF24L01_ReadStatus();
	
	/*读取配置寄存器，保存至Config变量*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	
	/*根据配置寄存器和状态寄存器的值，判断接收状态*/
	if ((Config & 0x02) == 0x00)		//配置寄存器位1（PWR_UP）为0
	{
		ReceiveFlag = 3;				//设备仍处于掉电模式，置标志位为3
		NRF24L01_Init();				//接收出错，重新初始化一次设备，这样有助于设备从错误中恢复正常
	}
	else if ((Status & 0x30) == 0x30)	//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
	{
		ReceiveFlag = 2;				//状态寄存器的值不合法，置标志位为2
		NRF24L01_Init();				//接收出错，重新初始化一次设备，这样有助于设备从错误中恢复正常
	}
	else if ((Status & 0x40) == 0x40)	//状态寄存器位6（RX_DR）为1
	{
		ReceiveFlag = 1;				//接收到数据，置标志位为1
		
		/*读接收有效载荷，存放在全局数组NRF24L01_RxPacket中，数据宽度为NRF24L01_RX_PACKET_WIDTH*/
		NRF24L01_ReadRxPayload(NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);
		
		/*给状态寄存器的位6（RX_DR）写1，清标志位*/
		NRF24L01_WriteReg(NRF24L01_STATUS, 0x40);

		/*清空Rx FIFO的所有数据*/
		NRF24L01_FlushRx();
	}
	else
	{
		ReceiveFlag = 0;				//未接收到数据，置标志位为0
	}
	
	/*返回接收标志位*/
	return ReceiveFlag;
}

/**
  * 函    数：NRF24L01更新接收地址
  * 参    数：无
  * 返 回 值：无
  * 说    明：如果想在运行时动态修改接收地址，则可先向全局数组NRF24L01_RxAddress写入修改的地址
  * 		  然后再调用此函数，使修改的接收地址生效
  */
void NRF24L01_UpdateRxAddress(void)
{
	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
}





