#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "BLE.h"
#include "main.h"
#include "em_timer.h"
#include "em_emu.h"
#include "common_vars.h"
#include "em_int.h"
#include "dma.h"
#include "crc.h"
#include "em_leuart.h"
#include "delayusingtimerint.h"


#define DMA_CH_RX    1 //DMA通道0
#define DMA_CH_TX    2 //DMA通道1

#define RX_BUF_SIZE        64 //UART接收数据BUF
#define TX_BUF_SIZE        64
//#define SYSCLOCK           14   //该数据表示系统时钟14M 

volatile bool BLE_ONLINE=false,BLE_APP_Model=true;
uint8_t BLE_STATE=BLE_STATE_IDLE;
union _BLE_CHIP BLE_DevChip;//器件信息


uint8_t insert_zero_number=16;


DMA_CB_TypeDef TX_DAM_CALLBACK ;//回调结构体定义
//DMA_CB_TypeDef RX_DAM_CALLBACK ;

char LeUartRxBuff[RX_BUF_SIZE];
char CopyRxBuff[RX_BUF_SIZE];

char LeUartTxBuff[RX_BUF_SIZE*2+32]; //128+8
//注意这里的数组要4字节对齐。
volatile bool TxDone=true;


volatile bool BleLeUartSta=BLE_UART_Closing;

volatile bool BLE_Responsed=false;
volatile bool crccheck=false;

void SendData2Host(uint8_t* p, uint8_t len);
void Rdy2DmaRx(void);
void getBleDeviceInfo(void);
void DisableLeUart(void);
void EnableLeUart(void);
void WaitLastTxDone(void);


uint16_t LeUartWorkTimeCount=0,LeUartTxCount=0; //

#define LeUartRxAllowWaitTime 10 // delaytime unit is 20ms , 10*20=200
#define LeUartTxInterval 4 //4*20=80ms,  delaytime unit is 20ms ,decide how many 0x00 header data to active CC254x


__STATIC_INLINE void ReChargeTimeCount(void)
{
	INT_Disable();
	LeUartWorkTimeCount=LeUartRxAllowWaitTime;
	INT_Enable();
}

void LeuartConfig (void)
{
	LEUART_Init_TypeDef tLeuartInit =
	{
		.enable   = leuartEnable,
		.refFreq  = 0,
		.baudrate = 115200,
		.databits = leuartDatabits8,
		.parity   = leuartNoParity,
		.stopbits = leuartStopbits1
	};

// CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);//用内核时钟的一半。使用内核时钟，它得到确定的波特率的配置与
	//使用LFXO时钟时设置的不一样。

	CMU_ClockEnable(cmuClock_CORELE,true);
	CMU_ClockEnable(cmuClock_LEUART0,true);
	CMU_ClockEnable(cmuClock_GPIO,true);

	LEUART_Reset(LEUART0);
	LEUART_Init(LEUART0, &tLeuartInit);//这里初始化LEUART


	LEUART0->ROUTE = LEUART_ROUTE_TXPEN |
	                 LEUART_ROUTE_RXPEN |
	                 LEUART_ROUTE_LOCATION_LOC4;

	GPIO_PinModeSet(BLE_TX_PORT,BLE_TX_PIN, gpioModePushPull,  1);
	GPIO_PinModeSet(BLE_RX_PORT,BLE_RX_PIN, gpioModeInputPull, 1);

	LEUART0->STARTFRAME = UART_DATA_START;//开始帧寄存器。0x3c  '<'
	LEUART0->SIGFRAME = UART_DATA_STOP;//信号帧寄存器。0x3e   ‘>’
/*
   当leuart接收到与定义的起始帧匹配的帧后，STARTF中断标志位被置位，如果SFUBRX位被置位，那么RXBLOCK
 将会被清除，同时起始帧被加载到RXbuffer
   当leuart接收到与定义的SIGFRAME一样的帧后，SIGF中断标志位被置位。
 */       
	//LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF|LEUART_IEN_STARTF);
	LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF); // only the stop char int，信号帧中断使能

    NVIC_SetPriority(LEUART0_IRQn, LEUART0_INT_LEVEL);
		
	NVIC_EnableIRQ(LEUART0_IRQn);
//这里的中断标志位信息在efm32wg_leuart.h中有定义。对应的寄存器是LEUART_IEN.
}

/**************************************************************************//**
 * @brief  Setup Low Energy UART with DMA operation
 *
 * The LEUART/DMA interaction is defined, and the DMA, channel and descriptor
 * is initialized. Then DMA starts to wait for and receive data, and the LEUART1
 * is set up to generate an interrupt when it receives the defined signal frame.
当接收到定义的信号帧后，产生中断。
 * The signal frame is set to '\r', which is the "carriage return" symbol.
 *
 *****************************************************************************/
/*----------------------------------------------------------------------------
 * DMA1完成了的回调函数（发送）
-----------------------------------------------------------------------------*/
void BleTxDMADone(unsigned int channel, bool primary, void *user)
{
	TxDone=true;
	LeUartTxCount=LeUartTxInterval;
}

/*------------------------------------------------------------------------------
 *LEuart MDA初始化
 *用到的资源：DMA0->RX DMA1->TX USART0
--------------------------------------------------------------------------------*/
void SetupLeuartDma(void)
{

    DMA_Init_TypeDef dmaInit;
//下面设置了2个通道，2个描述器。
   
	/* Setting up channel */
	DMA_CfgChannel_TypeDef chnl0Cfg =
	{
		.highPri   = false,                     /* Normal priority */
		.enableInt = false,                     /* No interupt enabled for callback functions */
		.select    = DMAREQ_LEUART0_RXDATAV,    /* Set LEUART1 RX data avalible as source of DMA signals 把LEUART1接收的数据看成DMA的源*/
		.cb        = NULL,                      /* No callback funtion */
	};


	/* Setting up channel descriptor */
	DMA_CfgDescr_TypeDef descr0Cfg =
	{
		.dstInc  = dmaDataInc1,       /* Increment destination address by one byte */
		.srcInc  = dmaDataIncNone,    /* Do no increment source address  */
		.size    = dmaDataSize1,      /* Data size is one byte */
		.arbRate = dmaArbitrate1,     /* Rearbitrate for each byte recieved*/
		.hprot   = 0,                 /* No read/write source protection */
	};

	/* Setting up channel */
	DMA_CfgChannel_TypeDef chnl1Cfg =
	{
		.highPri   = false,                     /* Normal priority */
		.enableInt = true,                     /* No interupt enabled for callback functions */
		.select    = DMAREQ_LEUART0_TXBL,    /* Set LEUART1 TX data avalible as source of DMA signals,这里吧TX中的数据作为DMA的信号源 */
		.cb        = &TX_DAM_CALLBACK,                      /* No callback funtion */
	};


	/* Setting up channel descriptor */
	DMA_CfgDescr_TypeDef descr1Cfg =
	{
		.dstInc  = dmaDataIncNone,       /* Increment destination address by one byte */
		.srcInc  = dmaDataInc1,    /* Do no increment source address  */
		.size    = dmaDataSize1,      /* Data size is one byte */
		.arbRate = dmaArbitrate1,     /* Rearbitrate for each byte recieved*/
		.hprot   = 0,                 /* No read/write source protection */
	};

        
        

	CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */

	dmaInit.hprot = 0;
    dmaInit.controlBlock = dmaControlBlock;
    DMA_Init(&dmaInit);

//这里配置了接收和发送通道	
	DMA_CfgChannel(DMA_CH_RX, &chnl0Cfg);
	DMA_CfgDescr(DMA_CH_RX, true, &descr0Cfg);
	DMA_CfgChannel(DMA_CH_TX, &chnl1Cfg);
	DMA_CfgDescr(DMA_CH_TX, true, &descr1Cfg);

        
	TX_DAM_CALLBACK.cbFunc  = BleTxDMADone;//调用回调函数。
	TX_DAM_CALLBACK.userPtr = NULL;	

	Rdy2DmaRx();//把LEUART接收到的数据搬移到leUartRXBuff[].中
}
//把LEUART0中接收的数据搬移到RxBuff（相当是内存），LEUART0中的数据蓝牙发送给efm32的。
void Rdy2DmaRx(void)
{
	/* Starting the transfer. Using Basic Mode */
	DMA_ActivateBasic(DMA_CH_RX,                /* Activate channel selected */
	                  true,                       /* Use primary descriptor */
	                  false,                      /* No DMA burst */
	                  (void *) &LeUartRxBuff,            /* Destination address *///目的地址
	                  (void *) &LEUART0->RXDATA,  /* Source address*///源地址
	                  RX_BUF_SIZE - 1);               /* Size of buffer minus1 */
}


uint8_t bleStFlg = 0x55;
uint8_t bleInfo[12] = {0x00};

void LEUART0_IRQHandler(void)
{
	uint32_t GucLeuartIF;
	GucLeuartIF = LEUART_IntGet(LEUART0);                               /* 得到中断标志位             */
	LEUART_IntClear(LEUART0, GucLeuartIF);                              /* 清除中断标志位               */

	ReChargeTimeCount();
	
	/* Signal frame found */
	if (GucLeuartIF & LEUART_IF_SIGF)// 主要处理数据中有'>' 的情况测试时程序进来了
	{
		int error=1;
		int foundStartChar=0;
		int start_add;
		int len;

		for(int i=0; i<RX_BUF_SIZE-4; i++)
		{
			if(LeUartRxBuff[i]==UART_DATA_START)//当i=3时就进入下面。
			{
				foundStartChar=1;
				start_add=i;//开始的位置是3
				len=LeUartRxBuff[i+1];//接下来是长度位，测得len=18
				if(LeUartRxBuff[start_add+len-1]==UART_DATA_STOP)
				{
					error=0;//进来了,说明接收到数据了。
					break;
				}
			}
		}
/*
说明：当首次进入中断，LeUartRxBuff从第3个元素开始有数据：0x3C,12,01,01,40,08,02
	  00,1b,f7,79,50,ff,d0,00,00,0,3e就表示当前BLE已经在boot状态，不需要发送转换命令
	  当LeUartRxBuff从第8个才有元素为05,07.。。。以及在第21个元素，才有数据
	  3C 11 01 01 8D 4C 01 41 1B 28 F7 79 50 FF D0 01 3E这表示在APP状态，需要转换。
	这只是其中一个蓝牙芯片的数据。
*/
		if(error==0)
		{	
		
		    BLE_ONLINE = true;
			  
//重新设置一个缓存用来存储ble的器件信息。CopyRxBuff,还是用来放应答信息。
			
			memcpy(CopyRxBuff,&LeUartRxBuff[start_add],len);
			  
            
			
			if((CopyRxBuff[0] == 0x3c) && (CopyRxBuff[1] == 0x10) && (CopyRxBuff[2] == 1) && (CopyRxBuff[14] == 1))//APP状态
			{
				bleStFlg = 1;	
			}
			else if((CopyRxBuff[0] == 0x3c) && (CopyRxBuff[1] == 0x12) &&(CopyRxBuff[2] == 1) && (CopyRxBuff[14] == 0))//boot状态
			{
				bleStFlg = 2;
			}
			
			//need to get the ACK from interrup for waiting check,这里UART_ID_CMD=2，UART_CMD_ACK=5；
			if(CopyRxBuff[UART_ID_CMD]==UART_CMD_ACK)
				{
				  BLE_Responsed=true;
				} 
/*
这里主要想检测当发送升级结束命令后BLE的返回信息：
通过测试不同的App，我发现返回的信息不同，本次测试采用的BLE的App是BLE-App-20140408-2.cfw.它升级成功后返回的数据
的第三部分是：00 00 00 00 00 00 3c 06 01 02 01 3e.
另外在这里检测是否收到该数据来判断升级是否成功不够充分，因为当BLE处于App模式时，对它复位后返回器件信息的同时也会返回该信息
所以不能仅仅用它来判断是否升级成功。
在本工程中我的另一个判断条件是：在发送完升级结束命令后，一直等待对该命令的响应。
通过上面两个条件，能够最终判断升级是否成功。

*/
			else if((CopyRxBuff[0] == 0x3c) && (CopyRxBuff[1] == 0x06) && (CopyRxBuff[2] == 0x01)
					&& (CopyRxBuff[3] == 0x02) && (CopyRxBuff[4] == 0x01) && (CopyRxBuff[5] == 0x3e))//升级结束返回的数据中第14个存储的是器件状态信息，若是1，则升级成功，转换到App模式。
				{
				  	crccheck = true;  
				}
			else
			{;}
			

			memset(LeUartRxBuff,0,sizeof(LeUartRxBuff));//buff清零反正上个数据包的干扰.到这里RxBuff中的数据变化了


			DMA_ActivateBasic(DMA_CH_RX,				/* Activate channel selected */
			                  true,					   /* Use primary descriptor */
			                  false,					   /* No DMA burst */
			                  (void *) &LeUartRxBuff,			  /* Destination address */
			                  (void *) &LEUART0->RXDATA,  /* Source address*///这里把LEUART0的数据搬移到内存RxBuff中
			                  RX_BUF_SIZE - 1);
		}

		else  if(foundStartChar==0)  //there is an error happened , reset the receiver
		{
			memset(LeUartRxBuff,0,sizeof(LeUartRxBuff));//clear the buff

			DMA_ActivateBasic(DMA_CH_RX,				/* Activate channel selected */
			                  true,					   /* Use primary descriptor */
			                  false,					   /* No DMA burst */
			                  (void *) &LeUartRxBuff,			  /* Destination address */
			                  (void *) &LEUART0->RXDATA,  /* Source address*/
			                  RX_BUF_SIZE - 1);
		}
	}
	//else  if (GucLeuartIF & LEUART_IF_STARTF)
	// {

	//	}
}

// 100ms at most to wait
void WaitLastTxDone(void)
{
	while(TxDone==false)//txDone=1,
	{
		EMU_EnterEM1();//DMA Not Done
	//	INT_Disable();
		if(LeUartWorkTimeCount==0)  //ReChargeTimeCount()函数对LeUartWorkTimeCount进行赋值。
		{
			INT_Enable();
			break;
		}
		INT_Enable();
	};

}

void LEUARTSentByDma(uint8_t comm_type,uint8_t *p ,uint8_t len)
{
	if(BleLeUartSta==BLE_UART_Closing)
	{
		EnableLeUart();
	}

	WaitLastTxDone();

//本程序中insert_zero_number = 16，因为当BLE处于App模式时，需要在发送的命令前面加上16个0用来等待BLE醒来。
	
	for(int i=0; i<insert_zero_number; i++)
	{LeUartTxBuff[i]=0;}//发送缓存中已经存储了16个0

	LeUartTxBuff[insert_zero_number+0]=UART_DATA_START;//发送缓存前面空16个0，接着发送起始以及后面的命令。
	LeUartTxBuff[insert_zero_number+1]=len+3+2+1;
	LeUartTxBuff[insert_zero_number+2]=comm_type;
	memcpy(&LeUartTxBuff[insert_zero_number+3],p,len);//从p内存开始的位置起拷贝len个字节，到LeUartTxBuff[insert_zero_number+3]指针的后面

	LeUartTxBuff[insert_zero_number+3+len+0]=0;//这里LeUartTxBuff的数据在原来的基础上增加len字节，最后补上2个0和结束标志
	LeUartTxBuff[insert_zero_number+3+len+1]=0;
	LeUartTxBuff[insert_zero_number+3+len+2]=UART_DATA_STOP;

	ReChargeTimeCount();

	TxDone=false;//下面使能了DMA的基本模式。
	DMA_ActivateBasic(DMA_CH_TX,                                         /*DMA通道1的设置       */
	                  true,
	                  false,
	                  (void *)(uint32_t) & LEUART0->TXDATA,
	                  (void *)(uint32_t) LeUartTxBuff,
	                  len-1+insert_zero_number+2+4); // 2= 2 0x00 at end ,  4 = < len ,type ,>
}


void MyLEUARTSentByDma(uint8_t comm_type,uint8_t *p ,uint8_t len)
{
  
	if(BleLeUartSta==BLE_UART_Closing)
	{
		EnableLeUart();
	}

	WaitLastTxDone();


	for(int i= 0; i < 6; i++)
	{
		LeUartTxBuff[i] = 0;
	}
	
	LeUartTxBuff[6+0]=UART_DATA_START;
	LeUartTxBuff[6+1]=len+3+2+1;
	LeUartTxBuff[6+2]=comm_type;
	memcpy(&LeUartTxBuff[6+3],p,len);

	LeUartTxBuff[6+3+len+0]=0;
	LeUartTxBuff[6+3+len+1]=0;
	LeUartTxBuff[6+3+len+2]=UART_DATA_STOP;

	ReChargeTimeCount();

	TxDone=false;
	DMA_ActivateBasic(DMA_CH_TX,                                         /*DMA通道1的设置       */
	                  true,
	                  false,
	                  (void *)(uint32_t) & LEUART0->TXDATA,
	                  (void *)(uint32_t) LeUartTxBuff,
	                  len-1+6+2+4); // 2= 2 0x00 at end ,  4 = < len ,type ,>

}

void BLE_INIT(void)
{
	CMU_ClockEnable( cmuClock_GPIO, true );

	// will eat 2uA current
	GPIO_PinModeSet(BLE_32K_PORT,BLE_32K_PIN,gpioModePushPull,0);  /* BLE 32KHZ CLOCK   PA1*/
	CMU->CTRL =(CMU->CTRL&~_CMU_CTRL_CLKOUTSEL1_MASK)|CMU_CTRL_CLKOUTSEL1_LFXO ;
	CMU->ROUTE = CMU_ROUTE_CLKOUT1PEN|CMU_ROUTE_LOCATION_LOC0;

	GPIO_PinModeSet(BLE_RST_PORT,BLE_RST_PIN,gpioModePushPull, 1);  /* BLE_ RESET    PF7*/
	BLE_RST_L();
	SysCtlDelay(8000*SYSCLOCK); 
	BLE_RST_H();

	
	LeuartConfig();
	SetupLeuartDma();//这里调用了DMA的设置
	EnableLeUart();//这里始终打开串口。
	BLE_ONLINE=false;

}


void EnableLeUart(void)
{
	GPIO_IntConfig(BLE_INT_PORT,BLE_INT_PIN,false,true,false); // close int
	CMU_ClockEnable(cmuClock_LEUART0,true);
	Rdy2DmaRx();//使用基本的模式开始发送 
	BleLeUartSta=BLE_UART_Opening;//打开BLE

}



//UART_CMD_INFOR命令主要查看器件的信息。
void getBleDeviceInfo(void)
{
	uint8_t COMM_DEV_INFO[2];
	COMM_DEV_INFO[0]=UART_TYPE_DEV;//这里可以改成DEV和INFO
	MyLEUARTSentByDma(UART_CMD_INFOR,COMM_DEV_INFO,1);//@p1,通信类型，@p2,数据，@p3,长度
}




//该命令主要用来把器件从App状态转换到boot，准备升级用，命令前面多16个0。
void AppToBoot(void)
{
	uint8_t COMM_DEV_INFO[3];
	COMM_DEV_INFO[0]=0;
	COMM_DEV_INFO[1]=0;
	COMM_DEV_INFO[2]=0;	
	LEUARTSentByDma(UART_CMD_UPDATA,COMM_DEV_INFO,3);
}


//升级命令前加6个0.
void MyBLE_Update_Start(void)
{
	uint8_t COMM_DEV_INFO[3];
	COMM_DEV_INFO[0]=0;
	COMM_DEV_INFO[1]=0;
	COMM_DEV_INFO[2]=0;
	MyLEUARTSentByDma(UART_CMD_UPDATA,COMM_DEV_INFO,3); 

}

//这里的checksum是升级数据的CRC
void  BLE_Update_End(uint16_t checksum)
{
	uint8_t COMM_DEV_INFO[3];
	
	COMM_DEV_INFO[0]=0x01;
	COMM_DEV_INFO[1]=checksum & 0xff;
	COMM_DEV_INFO[2]=checksum >> 8;
	MyLEUARTSentByDma(UART_CMD_UPDATA,COMM_DEV_INFO,3);

}



void WriteCC254xFlash(uint8_t *p)
{
	if(BleLeUartSta==BLE_UART_Closing)//我一直打开BLE
	{
		EnableLeUart();
	}

	WaitLastTxDone();
	
	while(TxDone==false)//
	{
		EMU_EnterEM1();//DMA Not Done
	};


	for(int i=0; i<6; i++)
		LeUartTxBuff[i]=0;

	LeUartTxBuff[6+0]=UART_DATA_START;
	LeUartTxBuff[6+1]=128+3+2+1;
	LeUartTxBuff[6+2]=UART_CMD_UPGRADE_DATA;
	memcpy(&LeUartTxBuff[6+3],p,128);

	LeUartTxBuff[6+3+128+0]=0;
	LeUartTxBuff[6+3+128+1]=0;
	LeUartTxBuff[6+3+128+2]=UART_DATA_STOP;

	ReChargeTimeCount();

	TxDone=false;
	DMA_ActivateBasic(DMA_CH_TX,           /*DMA通道1的设置       */
	                  true,
	                  false,
	                  (void *)(uint32_t) & LEUART0->TXDATA,
	                  (void *)(uint32_t) LeUartTxBuff,
	                  128-1+6+2+4); // 6=前面的0的个数，2= 2 0x00 at end ,  4 = < len ,type ,>（即开始，长度，类型，结束）4个字节
//-1说明：注意凡是涉及到数组时，数组末尾地址是数组元素个数减1.
}





