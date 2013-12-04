#include <stm32f2xx.h>
#include "NRF24L01.h"
#include "config.h"
#include <stdio.h>
#include <string.h>



#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};  // 定义一个静态发送地址



u8 RX_BUF[TX_PLOAD_WIDTH];

u8 TX_BUF[TX_PLOAD_WIDTH];

/*配置MISO and MOSI SCLK 为复用功能（推挽）输出  Open207Z_SPIx  */
/*配置SPI NRF24L01+片选	GPIOB_PIN_15     	CSN   	通用推挽输出模式 */
/*配置SPI NRF24L01+模式选择	GPIOB_PIN_14    CE		通用推挽输出模式*/
/*配置SPI NRF24L01+中断信号	 GPIOB_PIN_13   IRQ		上拉输入模式*/



static void Initial_SPI(SPI_TypeDef* SPIx)  //初始化IOB端口
{
	SPI_InitTypeDef SPI_InitStruct;	 
	GPIO_InitTypeDef GPIO_InitStructure;

	//上层函数做的RCC_AHB1PeriphClockCmd(Open207Z_CSN_GPIO_CLK | Open207Z_CE_GPIO_CLK | Open207Z_IRQ_GPIO_CLK ,ENABLE);/* 使能GPIOA 时钟 */ 
	//#define Open207Z_CSN_GPIO_CLK							RCC_AHB1Periph_GPIOB
	RCC_AHB1PeriphClockCmd(Open207Z_SPIx_SCK_GPIO_CLK | Open207Z_SPIx_MISO_GPIO_CLK | Open207Z_SPIx_MOSI_GPIO_CLK,ENABLE);/* 使能 SPI1 时钟 */ 
  //#define Open207Z_SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
	//#define Open207Z_SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
	//#define Open207Z_SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA

//ST define
//#define RCC_APB1Periph_SPI2              ((uint32_t)0x00004000)
//#define RCC_APB1Periph_SPI3              ((uint32_t)0x00008000)
//#define RCC_APB2Periph_SPI1              ((uint32_t)0x00001000)
	
	//stm32f10x RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);                //使能SPI1时钟
	RCC_APB2PeriphClockCmd(Open207Z_SPIx_CLK,ENABLE);//RCC->APB2ENR |= 1<<12;RCC->APB2ENR |= 1<<2;//使能SPI1 时钟
	//#define Open207Z_SPIx_CLK                       RCC_APB2Periph_SPI1


  //Pin Alternate Function setting
	/* Connect PA5 to SPI1_SCK */
	GPIO_PinAFConfig(Open207Z_SPIx_SCK_GPIO_PORT, Open207Z_SPIx_SCK_SOURCE,  Open207Z_SPIx_SCK_AF);
	/* Connect PA6 to SPI1_MISO */
	GPIO_PinAFConfig(Open207Z_SPIx_MISO_GPIO_PORT, Open207Z_SPIx_MISO_SOURCE, Open207Z_SPIx_MISO_AF);
	/* Connect PA7 to SPI1_MOSI */
	GPIO_PinAFConfig(Open207Z_SPIx_MOSI_GPIO_PORT, Open207Z_SPIx_MOSI_SOURCE, Open207Z_SPIx_MOSI_AF);
	//#define Open207Z_SPIx_SCK_GPIO_PORT             GPIOA
	//#define Open207Z_SPIx_SCK_SOURCE                GPIO_PinSource5
	//#define Open207Z_SPIx_SCK_AF                    GPIO_AF_SPI1

	//  PA5--CLK; PA7--MOSI  复用推挽输入
	GPIO_InitStructure.GPIO_Pin = Open207Z_SPIx_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//Alternate Function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//GPIO_Speed_50MHz;GPIO_Speed_100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; 

	GPIO_Init(Open207Z_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = Open207Z_SPIx_MOSI_PIN;
	GPIO_Init(Open207Z_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);	
	
	//PA6--MISO  输入浮空
	GPIO_InitStructure.GPIO_Pin = Open207Z_SPIx_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//改为GPIO_Mode_IN后收到数据全零;
	//GPIO_Mode_AF; （模拟输入_AIN、输入浮空_IN_FLOATING、输入上拉_IPU、输入下拉_IPD、开漏输出_OUT_OD、推挽式输出_OUT_PP、推挽式复用输出_AF_PP、开漏复用输出_AF_OD）
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//GPIO Output type
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // GPIO Configuration PullUp PullDown
	GPIO_Init(Open207Z_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
	//STM32 SPI配置的为主模式，PA6口作为MISO，可以配置成GPIO_Mode_AF_PP，也可以配置成输入，都可以正常工作。类似的USART_RX，也都可以配置成GPIO_Mode_AF_PP或者输入。当一个GPIO端口配置为GPIO_Mode_AF_PP是，这个端口的内部结构框图如下：图中可以看到，片上外设的复用功能输出信号会连接到输出控制电路，然后在端口上产生输出信号。
//但是在芯片内部，MISO是SPI模块的输入引脚，而不是输出引脚，也就是说图中的"复用功能输出信号"根本不存在，因此"输出控制电路"不能对外产生输出信号。
//而另一方面看，即使在GPIO_Mode_AF_PP模式下，复用功能输入信号却与外部引脚之间相互连接，既MISO得到了外部信号的电平，实现了输入的功能。



	SPI_I2S_DeInit(Open207Z_SPIx);// SPI_Cmd(SPI2, DISABLE);             //必须先禁能,才能改变MODE
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;//SPI1设置为两线全双工
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; //SPI发送接收8位帧结构
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;//设置SPI1为主模式
	//SPI_InitStruct.SPI_CPOL = SPI_CPOL_High ;// SPI_CPOL_High;串行时钟在不操作时，时钟为高电平
	//SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;// CPOL_HIGH对应CPHA_2EDGE。SPI_CPHA_2Edge;第二个时钟沿开始采样数据
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low ;// SPI_CPOL_High;串行时钟在不操作时，时钟为高电平
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;// CPOL_HIGH对应CPHA_2EDGE。SPI_CPHA_2Edge;第二个时钟沿开始采样数据
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ;//NSS信号由软件（使用SSI位）管理
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//SPI_BaudRatePrescaler_8 128定义波特率预分频的值:波特率预分频值为8
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB; //数据传输从MSB位开始
	SPI_InitStruct.SPI_CRCPolynomial = 7;//CRC值计算的多项式

	SPI_Init(Open207Z_SPIx, &SPI_InitStruct);
	SPI_Cmd(Open207Z_SPIx, ENABLE);

}

static void SPI_Send_byte(SPI_TypeDef* SPIx,u8 data)
{

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPIx,data);

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)==RESET);
	SPI_I2S_ReceiveData(SPIx);

}

static u8 SPI_Receive_byte(SPI_TypeDef* SPIx,u8 data)
{

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPIx,data);

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(SPIx);

}

static void delay1us(u8 t)
{
	while(--t);
} 

/****向寄存器reg写一个字节，同时返回状态字节**************/
u8 SPI_RW_Reg(u8 reg,u8 value)
{
	u8 status;
	CSN(0);
	status=SPI_Receive_byte(Open207Z_SPIx,reg);   //select register  and write value to it
	//#define Open207Z_SPIx                           SPI1
	//#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
	//#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
	//#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
	//#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
	SPI_Send_byte(Open207Z_SPIx,value);   
	CSN(1);
	return(status); 
}
/****向寄存器reg读一个字节，同时返回状态字节**************/
u8 SPI_Read_Reg(u8 reg)
{
	u8 status;
	CSN(0);
	SPI_Send_byte(Open207Z_SPIx,reg);
	status=SPI_Receive_byte(Open207Z_SPIx,0);   //select register  and write value to it
	CSN(1);
	return(status);
}
/********读出bytes字节的数据*************************/
u8 SPI_Read_Buf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(Open207Z_SPIx,reg);       
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr]=SPI_Receive_byte(Open207Z_SPIx,0);
	CSN(1);
	return(status);
}

/****************写入bytes字节的数据*******************/
u8 SPI_Write_Buf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(Open207Z_SPIx,reg); 
	delay1us(1);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(Open207Z_SPIx,*pBuf++);
	CSN(1);
	return(status);
}
 /****************发送函数***************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	CE(0);			//StandBy I模式	
	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送
	CE(1);		 //置高CE，激发数据发送
	delay1us(10);
}

void RX_Mode(void)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度 
 
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // 使能接收通道0自动应答
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // 使能接收通道0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);                 // 选择射频通道0x40

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式
  	CE(1);
	delay1us(10);
}

void TX_Mode(u8 * tx_buf)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // 使能接收通道0自动应答
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // 使能接收通道0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发10次
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);         // 选择射频通道0x40
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电
	CE(1);
	delay1us(10);
} 

void nRF24L01_Initial(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_AHB1Periph_GPIOB  /* Enable the GPIOB clock */
	RCC_AHB1PeriphClockCmd(Open207Z_CSN_GPIO_CLK | Open207Z_CE_GPIO_CLK | Open207Z_IRQ_GPIO_CLK ,ENABLE);/* 使能GPIOA 时钟 */ 

	
	/*CE Initial*/
	/* NRF24L01.h Configure NRF24L01 pins: IRQ->PB14 and CSN->PB15  CE->PB13*/ 
	GPIO_InitStructure.GPIO_Pin =Open207Z_GPIO_Pin_CE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(Open207Z_CE_GPIO, &GPIO_InitStructure);	
	/*CSN Initial*/
 	GPIO_InitStructure.GPIO_Pin =Open207Z_GPIO_Pin_CSN;
	GPIO_Init(Open207Z_CSN_GPIO, &GPIO_InitStructure);

	/*IRQ Initial*/
	GPIO_InitStructure.GPIO_Pin = Open207Z_GPIO_Pin_IRQ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(Open207Z_IRQ_GPIO, &GPIO_InitStructure);
//到底用哪一个SPI口，取决于config.h开头中的#define OpenSPI2
//ce,csn,irq是三个spi共用，同时只能有一个使用。实际上占用的是spi2的针
//0-8Mbps 4-wire SPI serial interface
//8 bit command set
	Initial_SPI(Open207Z_SPIx);

}

/****************** 配置函数********************************/
void nRF24L01_Config(void)
{
          //initial io
	//CE(0);          //        CE=0 ;chip enable
	//CSN(1);       //CSN=1   Spi disable
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e); // Set PWR_UP bit, enable CRC(2 bytes) &Prim:RX. RX_DR enabled..
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f); // Enable Pipe0
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_AW, 0x02); // Setup address width=5 bytes
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP,0x07); // TX_PWR:0dBm, Datarate:2Mbps,
}
 
void NRF24L01_RegisiterTest(void)
{
    u8 status=0x00;
	CE(0);//StandBy I模式	
//IRQ(this signal is active low and is controlled by three maskable interrupt sources)
//CE(this signal is active high and is used to activate the chip in RX or TX mode)	//参考TX_Mode(TX_BUF);
	//while(IRQ);
	
	delay1us(10);
	
    //SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x08);                                         // Config reg, default reset value
    //SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);                                          // Default, Auto ack all pipe's
    //SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x03);                                      // Default, pipe 0 & 1 enabled
    //SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_AW, 0x03);                                       // Default, 5 bytes address
    //SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, MASK_IRQ_FLAGS);                               // Clear all IRQ flag
    //SPI_RW_Reg(FLUSH_TX,0);                                                       // Just in case, flush TX FIFO
    //SPI_RW_Reg(FLUSH_RX,0);                                                       // and RX FIFO
    //SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);                        // Writes TX_Addr
	
	status=SPI_Read_Reg(READ_REG_NRF24L01 + STATUS);	// 读取状态寄存其来判断数据接收状况
  //	CSN(0);
//Every new command must be started by a high to low transition on CSN.
	//SPI_Send_byte(Open207Z_SPIx,reg);
	//status=SPI_Receive_byte(Open207Z_SPIx,0);   //select register  and write value to it
	//CSN(1);


// 为了应答接收设备，接收通道0地址和发送地址相同
	status=SPI_Read_Buf(READ_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);
	
  status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
	status=SPI_Read_Buf(READ_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);
	
	status=SPI_Read_Buf(READ_REG_NRF24L01 + RX_ADDR_P1,RX_BUF,TX_ADR_WIDTH);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P2);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P3);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P4);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P5);
	
	status=SPI_Read_Buf(WRITE_REG_NRF24L01 + TX_ADDR,RX_BUF,TX_ADR_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x70);            // Read status & clear IRQ flag's
	status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_AA, 0x3f);       // 使能接收通道0自动应答0x01Enable auto ack pipe0
  //	CSN(0);
	//status=SPI_Receive_byte(Open207Z_SPIx,reg);   //select register  and write value to it
	//SPI_Send_byte(Open207Z_SPIx,value);   
	//CSN(1);
	status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_RXADDR, 0x3f);   // 使能接收通道0  0x01Enable pipe0
  status=SPI_RW_Reg(READ_REG_NRF24L01 + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发10次
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_CH, 40);         // 选择射频通道0x40
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	status=SPI_RW_Reg(READ_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
  status=SPI_RW_Reg(READ_REG_NRF24L01 + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电PWR_UP, CRC(2bytes) & Prim:TX.
	// IRQ收发完成中断响应，16位CRC，主发送
	//参考RX_Mode(void)
  status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度 
  status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_AA, 0x3f);               // 使能接收通道0自动应答
  status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_RXADDR, 0x3f);           // 使能接收通道0
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_CH, 40);                 // 选择射频通道0x40
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_SETUP, 0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
  status=SPI_RW_Reg(READ_REG_NRF24L01 + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式	
	
	

	status=SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址
  //我们的设备地址，收发相同u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};  // 定义一个静态发送地址

	//#define TX_ADR_WIDTH   	5  // 5字节宽度的发送/接收地址（用于许多设备组网）
	//#define TX_PLOAD_WIDTH 	32  // 数据通道收发的字节数，缓冲区长度，有效数据宽度



	//	status=SPI_Receive_byte(Open207Z_SPIx,reg); 
	//delay1us(1);
	//for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
	//	SPI_Send_byte(Open207Z_SPIx,*pBuf++);
	//SPI_Send_byte和SPI_Receive_byte内部一样，先发后收字节。收发数据可以使用同一个函数，因为SPI是同步输入输出的，在发送数据的时候已经在接受数据。

	//发送数据只需要这三句SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	//SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	
	//SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送

	status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同
	status=SPI_Read_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);

	status=SPI_Write_Buf(WR_TX_PLOAD, TX_BUF, TX_PLOAD_WIDTH); 			 // 装载数据
	status=SPI_Read_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x70);            // Read status & clear IRQ flag's
	status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // 使能接收通道0自动应答
  //	CSN(0);
	//status=SPI_Receive_byte(Open207Z_SPIx,reg);   //select register  and write value to it
	//SPI_Send_byte(Open207Z_SPIx,value);   
	//CSN(1);
	status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // 使能接收通道0
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发10次
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);         // 选择射频通道0x40
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电
	// IRQ收发完成中断响应，16位CRC，主发送

	//参考RX_Mode(void)
  status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度 
 
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // 使能接收通道0自动应答
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // 使能接收通道0
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);                 // 选择射频通道0x40

  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式


	CE(1);//置高CE，激发数据发送
	delay1us(10);
}

void NRF24L01_Send(void)
{
    u8 status=0x00;
	CE(0);
	TX_Mode(TX_BUF);
	while(IRQ);
	
	delay1us(10);
	status=SPI_Read_Reg(STATUS);	// 读取状态寄存其来判断数据接收状况
	printf("STATUS接受状态：0x%2x\r\n",status);
	if(status&TX_DS)	/*tx_ds == 0x20*/
	{
		printf("STATUS接受状态：0x%2x\r\n",status);
		printf("\r\n发送完数据：%s\r\n",RX_BUF);	
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);      // 清除TX，让IRQ拉低；
			
	}
	else if(status&MAX_RT)
		{
			printf("发送达到最大发送次数");	
			SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);      // 清除TX，让IRQ拉低；			
		}
	CE(1);

}

void NRF24L01_Receive(void)
{   
    u8 status=0x01;  
	CE(0);
	RX_Mode();
//	while(IRQ);

	
	delay1us(10);
	status=SPI_Read_Reg(STATUS);	// 读取状态寄存其来判断数据接收状况
	printf("STATUS接受状态：0x%2x\r\n",status);
	if(status & 0x40)			//接受中断标志位
	{
		printf("接受成功");

		SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

		printf("\r\n 接收到数据：%x\r\n",RX_BUF[0]);
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);      // 清除TX，让IRQ拉低
	}  
	CE(1);

}









