#include <stm32f2xx.h>
#include "NRF24L01.h"
#include "config.h"
#include <stdio.h>
#include <string.h>



#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};  // ����һ����̬���͵�ַ



u8 RX_BUF[TX_PLOAD_WIDTH];

u8 TX_BUF[TX_PLOAD_WIDTH];

/*����MISO and MOSI SCLK Ϊ���ù��ܣ����죩���  Open207Z_SPIx  */
/*����SPI NRF24L01+Ƭѡ	GPIOB_PIN_15     	CSN   	ͨ���������ģʽ */
/*����SPI NRF24L01+ģʽѡ��	GPIOB_PIN_14    CE		ͨ���������ģʽ*/
/*����SPI NRF24L01+�ж��ź�	 GPIOB_PIN_13   IRQ		��������ģʽ*/



static void Initial_SPI(SPI_TypeDef* SPIx)  //��ʼ��IOB�˿�
{
	SPI_InitTypeDef SPI_InitStruct;	 
	GPIO_InitTypeDef GPIO_InitStructure;

	//�ϲ㺯������RCC_AHB1PeriphClockCmd(Open207Z_CSN_GPIO_CLK | Open207Z_CE_GPIO_CLK | Open207Z_IRQ_GPIO_CLK ,ENABLE);/* ʹ��GPIOA ʱ�� */ 
	//#define Open207Z_CSN_GPIO_CLK							RCC_AHB1Periph_GPIOB
	RCC_AHB1PeriphClockCmd(Open207Z_SPIx_SCK_GPIO_CLK | Open207Z_SPIx_MISO_GPIO_CLK | Open207Z_SPIx_MOSI_GPIO_CLK,ENABLE);/* ʹ�� SPI1 ʱ�� */ 
  //#define Open207Z_SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
	//#define Open207Z_SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
	//#define Open207Z_SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA

//ST define
//#define RCC_APB1Periph_SPI2              ((uint32_t)0x00004000)
//#define RCC_APB1Periph_SPI3              ((uint32_t)0x00008000)
//#define RCC_APB2Periph_SPI1              ((uint32_t)0x00001000)
	
	//stm32f10x RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);                //ʹ��SPI1ʱ��
	RCC_APB2PeriphClockCmd(Open207Z_SPIx_CLK,ENABLE);//RCC->APB2ENR |= 1<<12;RCC->APB2ENR |= 1<<2;//ʹ��SPI1 ʱ��
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

	//  PA5--CLK; PA7--MOSI  ������������
	GPIO_InitStructure.GPIO_Pin = Open207Z_SPIx_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//Alternate Function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//GPIO_Speed_50MHz;GPIO_Speed_100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; 

	GPIO_Init(Open207Z_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = Open207Z_SPIx_MOSI_PIN;
	GPIO_Init(Open207Z_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);	
	
	//PA6--MISO  ���븡��
	GPIO_InitStructure.GPIO_Pin = Open207Z_SPIx_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//��ΪGPIO_Mode_IN���յ�����ȫ��;
	//GPIO_Mode_AF; ��ģ������_AIN�����븡��_IN_FLOATING����������_IPU����������_IPD����©���_OUT_OD������ʽ���_OUT_PP������ʽ�������_AF_PP����©�������_AF_OD��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//GPIO Output type
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // GPIO Configuration PullUp PullDown
	GPIO_Init(Open207Z_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
	//STM32 SPI���õ�Ϊ��ģʽ��PA6����ΪMISO���������ó�GPIO_Mode_AF_PP��Ҳ�������ó����룬�������������������Ƶ�USART_RX��Ҳ���������ó�GPIO_Mode_AF_PP�������롣��һ��GPIO�˿�����ΪGPIO_Mode_AF_PP�ǣ�����˿ڵ��ڲ��ṹ��ͼ���£�ͼ�п��Կ�����Ƭ������ĸ��ù�������źŻ����ӵ�������Ƶ�·��Ȼ���ڶ˿��ϲ�������źš�
//������оƬ�ڲ���MISO��SPIģ����������ţ�������������ţ�Ҳ����˵ͼ�е�"���ù�������ź�"���������ڣ����"������Ƶ�·"���ܶ����������źš�
//����һ���濴����ʹ��GPIO_Mode_AF_PPģʽ�£����ù��������ź�ȴ���ⲿ����֮���໥���ӣ���MISO�õ����ⲿ�źŵĵ�ƽ��ʵ��������Ĺ��ܡ�



	SPI_I2S_DeInit(Open207Z_SPIx);// SPI_Cmd(SPI2, DISABLE);             //�����Ƚ���,���ܸı�MODE
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;//SPI1����Ϊ����ȫ˫��
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; //SPI���ͽ���8λ֡�ṹ
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;//����SPI1Ϊ��ģʽ
	//SPI_InitStruct.SPI_CPOL = SPI_CPOL_High ;// SPI_CPOL_High;����ʱ���ڲ�����ʱ��ʱ��Ϊ�ߵ�ƽ
	//SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;// CPOL_HIGH��ӦCPHA_2EDGE��SPI_CPHA_2Edge;�ڶ���ʱ���ؿ�ʼ��������
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low ;// SPI_CPOL_High;����ʱ���ڲ�����ʱ��ʱ��Ϊ�ߵ�ƽ
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;// CPOL_HIGH��ӦCPHA_2EDGE��SPI_CPHA_2Edge;�ڶ���ʱ���ؿ�ʼ��������
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ;//NSS�ź��������ʹ��SSIλ������
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//SPI_BaudRatePrescaler_8 128���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB; //���ݴ����MSBλ��ʼ
	SPI_InitStruct.SPI_CRCPolynomial = 7;//CRCֵ����Ķ���ʽ

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

/****��Ĵ���regдһ���ֽڣ�ͬʱ����״̬�ֽ�**************/
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
/****��Ĵ���reg��һ���ֽڣ�ͬʱ����״̬�ֽ�**************/
u8 SPI_Read_Reg(u8 reg)
{
	u8 status;
	CSN(0);
	SPI_Send_byte(Open207Z_SPIx,reg);
	status=SPI_Receive_byte(Open207Z_SPIx,0);   //select register  and write value to it
	CSN(1);
	return(status);
}
/********����bytes�ֽڵ�����*************************/
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

/****************д��bytes�ֽڵ�����*******************/
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
 /****************���ͺ���***************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	CE(0);			//StandBy Iģʽ	
	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // װ�ؽ��ն˵�ַ
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // װ������	
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);   		 // IRQ�շ�����ж���Ӧ��16λCRC��������
	CE(1);		 //�ø�CE���������ݷ���
	delay1us(10);
}

void RX_Mode(void)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�� 
 
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // ʹ�ܽ���ͨ��0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);                 // ѡ����Ƶͨ��0x40

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
  	CE(1);
	delay1us(10);
}

void TX_Mode(u8 * tx_buf)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // װ������
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // ʹ�ܽ���ͨ��0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);         // ѡ����Ƶͨ��0x40
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�
	CE(1);
	delay1us(10);
} 

void nRF24L01_Initial(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_AHB1Periph_GPIOB  /* Enable the GPIOB clock */
	RCC_AHB1PeriphClockCmd(Open207Z_CSN_GPIO_CLK | Open207Z_CE_GPIO_CLK | Open207Z_IRQ_GPIO_CLK ,ENABLE);/* ʹ��GPIOA ʱ�� */ 

	
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
//��������һ��SPI�ڣ�ȡ����config.h��ͷ�е�#define OpenSPI2
//ce,csn,irq������spi���ã�ͬʱֻ����һ��ʹ�á�ʵ����ռ�õ���spi2����
//0-8Mbps 4-wire SPI serial interface
//8 bit command set
	Initial_SPI(Open207Z_SPIx);

}

/****************** ���ú���********************************/
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
	CE(0);//StandBy Iģʽ	
//IRQ(this signal is active low and is controlled by three maskable interrupt sources)
//CE(this signal is active high and is used to activate the chip in RX or TX mode)	//�ο�TX_Mode(TX_BUF);
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
	
	status=SPI_Read_Reg(READ_REG_NRF24L01 + STATUS);	// ��ȡ״̬�Ĵ������ж����ݽ���״��
  //	CSN(0);
//Every new command must be started by a high to low transition on CSN.
	//SPI_Send_byte(Open207Z_SPIx,reg);
	//status=SPI_Receive_byte(Open207Z_SPIx,0);   //select register  and write value to it
	//CSN(1);


// Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
	status=SPI_Read_Buf(READ_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);
	
  status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
	status=SPI_Read_Buf(READ_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);
	
	status=SPI_Read_Buf(READ_REG_NRF24L01 + RX_ADDR_P1,RX_BUF,TX_ADR_WIDTH);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P2);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P3);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P4);
	status=SPI_Read_Reg(READ_REG_NRF24L01 + RX_ADDR_P5);
	
	status=SPI_Read_Buf(WRITE_REG_NRF24L01 + TX_ADDR,RX_BUF,TX_ADR_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x70);            // Read status & clear IRQ flag's
	status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_AA, 0x3f);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��0x01Enable auto ack pipe0
  //	CSN(0);
	//status=SPI_Receive_byte(Open207Z_SPIx,reg);   //select register  and write value to it
	//SPI_Send_byte(Open207Z_SPIx,value);   
	//CSN(1);
	status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_RXADDR, 0x3f);   // ʹ�ܽ���ͨ��0  0x01Enable pipe0
  status=SPI_RW_Reg(READ_REG_NRF24L01 + SETUP_RETR, 0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_CH, 40);         // ѡ����Ƶͨ��0x40
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	status=SPI_RW_Reg(READ_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
  status=SPI_RW_Reg(READ_REG_NRF24L01 + CONFIG, 0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�PWR_UP, CRC(2bytes) & Prim:TX.
	// IRQ�շ�����ж���Ӧ��16λCRC��������
	//�ο�RX_Mode(void)
  status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�� 
  status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_AA, 0x3f);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  status=SPI_RW_Reg(READ_REG_NRF24L01 + EN_RXADDR, 0x3f);           // ʹ�ܽ���ͨ��0
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_CH, 40);                 // ѡ����Ƶͨ��0x40
  status=SPI_RW_Reg(READ_REG_NRF24L01 + RF_SETUP, 0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
  status=SPI_RW_Reg(READ_REG_NRF24L01 + CONFIG, 0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ	
	
	

	status=SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ
  //���ǵ��豸��ַ���շ���ͬu8 TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};  // ����һ����̬���͵�ַ

	//#define TX_ADR_WIDTH   	5  // 5�ֽڿ�ȵķ���/���յ�ַ����������豸������
	//#define TX_PLOAD_WIDTH 	32  // ����ͨ���շ����ֽ��������������ȣ���Ч���ݿ��



	//	status=SPI_Receive_byte(Open207Z_SPIx,reg); 
	//delay1us(1);
	//for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
	//	SPI_Send_byte(Open207Z_SPIx,*pBuf++);
	//SPI_Send_byte��SPI_Receive_byte�ڲ�һ�����ȷ������ֽڡ��շ����ݿ���ʹ��ͬһ����������ΪSPI��ͬ����������ģ��ڷ������ݵ�ʱ���Ѿ��ڽ������ݡ�

	//��������ֻ��Ҫ������SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // װ�ؽ��ն˵�ַ
	//SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // װ������	
	//SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);   		 // IRQ�շ�����ж���Ӧ��16λCRC��������

	status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
	status=SPI_Read_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);

	status=SPI_Write_Buf(WR_TX_PLOAD, TX_BUF, TX_PLOAD_WIDTH); 			 // װ������
	status=SPI_Read_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0,RX_BUF,TX_ADR_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x70);            // Read status & clear IRQ flag's
	status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  //	CSN(0);
	//status=SPI_Receive_byte(Open207Z_SPIx,reg);   //select register  and write value to it
	//SPI_Send_byte(Open207Z_SPIx,value);   
	//CSN(1);
	status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // ʹ�ܽ���ͨ��0
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);         // ѡ����Ƶͨ��0x40
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�
	// IRQ�շ�����ж���Ӧ��16λCRC��������

	//�ο�RX_Mode(void)
  status=SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�� 
 
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // ʹ�ܽ���ͨ��0
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);                 // ѡ����Ƶͨ��0x40

  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
  status=SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ


	CE(1);//�ø�CE���������ݷ���
	delay1us(10);
}

void NRF24L01_Send(void)
{
    u8 status=0x00;
	CE(0);
	TX_Mode(TX_BUF);
	while(IRQ);
	
	delay1us(10);
	status=SPI_Read_Reg(STATUS);	// ��ȡ״̬�Ĵ������ж����ݽ���״��
	printf("STATUS����״̬��0x%2x\r\n",status);
	if(status&TX_DS)	/*tx_ds == 0x20*/
	{
		printf("STATUS����״̬��0x%2x\r\n",status);
		printf("\r\n���������ݣ�%s\r\n",RX_BUF);	
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);      // ���TX����IRQ���ͣ�
			
	}
	else if(status&MAX_RT)
		{
			printf("���ʹﵽ����ʹ���");	
			SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);      // ���TX����IRQ���ͣ�			
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
	status=SPI_Read_Reg(STATUS);	// ��ȡ״̬�Ĵ������ж����ݽ���״��
	printf("STATUS����״̬��0x%2x\r\n",status);
	if(status & 0x40)			//�����жϱ�־λ
	{
		printf("���ܳɹ�");

		SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

		printf("\r\n ���յ����ݣ�%x\r\n",RX_BUF[0]);
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);      // ���TX����IRQ����
	}  
	CE(1);

}









