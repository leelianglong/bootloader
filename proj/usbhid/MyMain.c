/**************************************************************************//**
 * @file main.c
 * @brief USB HID keyboard device example.
 * @author Energy Micro AS
 * @version 1.2.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include <stdio.h>
#include "efm32.h"
#include "em_cmu.h"
#include "em_usb.h"
#include "em_msc.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_wdog.h"
#include "em_letimer.h"
#include "m25pxx.h"
#include "ble.h"
#include "em_leuart.h"
#include "globaldata.h"

#include "retargetserial.h"

#include "main.h"

#include "config.h"
#include "crc.h"
#include "boot.h"
#include "res.h"
#include "framebufferctrl.h"


uint32_t flashSize, flashPageSize;




/**************************************************************************//**
 *
 * This example shows how a HID keyboard can be implemented.
 *
 *****************************************************************************/


/*** Typedef's and defines. ***/

#define FW_TYPE_BOOT            0
#define FW_TYPE_APP             1

//����LED
#define LED_GPIOPORT  (gpioPortC)
#define LED_PIN   (10)
#define LED_TOGGLE()     GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
#define LED_ON()       GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
#define LED_OFF()      GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )


//FW�궨��
#define FW_STARTADDR           0
#define FW_APPSTART_ADDR       16     //Ƭ��flash�д洢APP����ʼ��ַ,ע���0��ʼ�ġ�
#define FWHEAD_LENGTH          16
#define FWTYPE_MCU             1
#define ONCEREADSIZE           2048   //ÿ�ζ�ȡ�Ĵ�С
#define RESET_MCU()            SCB->AIRCR = 0x05FA0004

#define FWTYPE_BLE             2
#define FWTYPE_BOOT            3


EFM32_PACK_START( 1 )

union _FW_INFO
{
	struct _INFO
	{
		uint16_t fw_type;
		uint16_t ver_num;
		uint32_t fw_length;
		uint16_t fw_crc;
		uint32_t Rev1;
		uint16_t Head_CRC;
	} INFO;
	uint8_t INFO_BUF[16];
};
union _FW_INFO FW_INFO;

EFM32_PACK_END()




//���Ź���ʼ��
WDOG_Init_TypeDef init =
{
	.enable     = true,               /* Start watchdog when init done */
	.debugRun   = false,              /* WDOG not counting during debug halt */
	.em2Run     = true,               /* WDOG counting when in EM2 */
	.em3Run     = true,               /* WDOG counting when in EM3 */
	.em4Block   = false,              /* EM4 can be entered */
	.swoscBlock = false,              /* Do not block disabling LFRCO/LFXO in CMU */
	.lock       = false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */
	.clkSel     = wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */
	.perSel     = wdogPeriod_16k,      /* Set the watchdog period to 4097 clock periods (ie ~2 seconds)*/
};


//LETIMER0�ĳ���
void LETIMER_setup(void)
{

	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);//�ȴ���Ƶ����������
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); //��������ʱ����Ҫ�ⲿ��Ƶ������Ϊʱ��Դ��
	CMU_ClockEnable(cmuClock_CORELE, true);//RTC�����еĵ͹������趼Ҫ��CORELF��ͬʱҲ�ǿ��Ź���ʱ��Դ
	CMU_ClockEnable(cmuClock_LETIMER0, true); //ʹ���Լ���ʱ��

	LETIMER_Init_TypeDef leTimerinit = LETIMER_INIT_DEFAULT;//�Ƚ���Ĭ�ϳ�ʼ��

	leTimerinit.debugRun = true;
	leTimerinit.comp0Top = true;
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0); //�Ƚ���0�ж�ʹ��
	NVIC_EnableIRQ(LETIMER0_IRQn);//��NVIC�ж�
	LETIMER_CompareSet(LETIMER0, 0, 500); //���ñȽ���0�ĳ�ʼֵ��ʱ15ms
	LETIMER_Init(LETIMER0, &leTimerinit);

}

volatile unsigned char efmCount = 0;
volatile unsigned char bleCount = 0;
volatile unsigned char bootCount = 0;

//�͹��Ķ�ʱ���ж�
void LETIMER0_IRQHandler(void)
{

	LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0); //����Ƚ���0���жϱ�־
	efmCount++;
	bleCount++;
	bootCount++;

	if((FW_INFO.INFO.fw_type == 1) && (efmCount == 12))
	{
		efmCount = 0;
		LED_TOGGLE();
	}
	else if((FW_INFO.INFO.fw_type == 2) && (bleCount == 30))
	{
		bleCount = 0;
		LED_TOGGLE();
	}
	else if((FW_INFO.INFO.fw_type == 3) && (bootCount == 3))
	{
		bootCount = 0;
		LED_TOGGLE();

	}

}

//��ȡƬ��flash�Ĵ�С��Ϣ
void FLASH_CalcPageSize(void)
{
	uint8_t family = *(uint8_t*)0x0FE081FE;

	flashSize = *(uint16_t*)0x0FE081F8 * 1024;

	flashPageSize = 4096;                 /* Assume Giant, 'H' */

	if ( family == 74 )
	{
		flashPageSize = 2048;               /* Leopard, 'J' */
	}
	else if ( family == 75 )
	{
		flashPageSize = 2048;               /* Leopard, 'W' */
	}
}


//��Ƭ��flash�����ݶ���������д��Ƭ��flash,ÿ�δ�Ƭ���2k,Ȼ��д��
void  InsideFlashWR(void)
{
	unsigned int i , j ;
	unsigned char appBuf[ONCEREADSIZE] = {0};
	unsigned int integer = 0;
	unsigned int decimal = 0;
	int align4 = 0;//4�ֽڶ���
	uint8_t wrFlg = 5;
	uint8_t ersFlg = 5;


	MSC_Init();//��Ƭ��flash���г�ʼ��������Ҫ������д��
	FLASH_CalcPageSize();//�Ȼ�ȡƬ��flash����Ϣ



	integer = FW_INFO.INFO.fw_length / ONCEREADSIZE;
	decimal = FW_INFO.INFO.fw_length % ONCEREADSIZE;

	SysCtlDelay(500000);//�����ʱȷ������С��app��ʱ���ܹ�˳����д��efm32��ָ��λ�á�

	for(i = 0; i < integer; i++) //����2k��������
	{
		while(IsFlashBusy());//����2�����Ƭ��flash�Ƿ���æ

		FlashRead((FW_APPSTART_ADDR + ONCEREADSIZE * i), appBuf, ONCEREADSIZE); //ÿ�ζ���Ƭ���2k

		while(IsFlashBusy());

		ersFlg = MSC_ErasePage((uint32_t*)(BOOTLOADER_SIZE + i * ONCEREADSIZE)); //ÿ�β���Ƭ�ڵ�2k

		while(ersFlg != mscReturnOk);

		ersFlg = 5;

//ͨ���鿴����ȫ���ɹ� ��
		for(j = 0; j < ONCEREADSIZE; j++)
		{
			if(*((uint8_t*)(BOOTLOADER_SIZE + i * ONCEREADSIZE + j)) != 0xFF) //���û�в�����ȷ������
				while(1);
		}

		wrFlg = MSC_WriteWord((uint32_t*)(BOOTLOADER_SIZE + i * ONCEREADSIZE), appBuf , ONCEREADSIZE); //ÿ��д2k

		while(wrFlg != mscReturnOk);

		wrFlg = 5;

		WDOG_Feed();
//������Ҫ�ϳ�ʱ�䣬�ڴ�ι��
	}

	if(decimal > 0)//ȷ������ʣ��
	{
		while(IsFlashBusy());

		FlashRead((FW_APPSTART_ADDR + ONCEREADSIZE * integer), appBuf, decimal); //appBuf�д洢�����ϴε����ݡ������ٶ�ȡһ�Σ�����ֻ��һ����

		while(IsFlashBusy());

		ersFlg = MSC_ErasePage((uint32_t*)(BOOTLOADER_SIZE + integer * ONCEREADSIZE)); //�ٶ����һ��ҳ������д���ʣ�µĲ���һ��ҳ������

		while(ersFlg != mscReturnOk);

		ersFlg = 5;

		for(j = 0; j < decimal; j++)
		{
			if(*((uint8_t*)(BOOTLOADER_SIZE + integer * ONCEREADSIZE + j)) != 0xff)
				while(1);
		}

		if((decimal % 4) == 0)//˵��ʣ����ֽ����ܱ�4��������ֱ��д��flash
		{
			wrFlg = MSC_WriteWord((uint32_t*)(BOOTLOADER_SIZE + integer * ONCEREADSIZE), appBuf , decimal);

			while(wrFlg != mscReturnOk);

			wrFlg = 5;

			WDOG_Feed();
		}
		else
		{
			align4 = decimal % 4;

			switch(align4)
			{
				case 1://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff; //ע�ⲹ����ʼ��ַ
					appBuf[decimal + 1] = 0xff;
					appBuf[decimal + 2] = 0xff;
					wrFlg = MSC_WriteWord((uint32_t*)(BOOTLOADER_SIZE + integer * ONCEREADSIZE), appBuf , decimal + 3);

					while(wrFlg != mscReturnOk);

					wrFlg = 5;
					WDOG_Feed();

				}
				break;

				case 2://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					appBuf[decimal + 1] = 0xff;
					wrFlg = MSC_WriteWord((uint32_t*)(BOOTLOADER_SIZE + integer * ONCEREADSIZE), appBuf , decimal + 2);

					while(wrFlg != mscReturnOk);

					wrFlg = 5;
					WDOG_Feed();
				}
				break;

				case 3://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					wrFlg = MSC_WriteWord((uint32_t*)(BOOTLOADER_SIZE + integer * ONCEREADSIZE), appBuf , decimal + 1);

					while(wrFlg != mscReturnOk);

					wrFlg = 5;
					WDOG_Feed();
				}
				break;

				default:
					break;
			}
		}
	}
}

void  CalcInsideFlashCRCandWrIn(void)
{
	unsigned char i = 0;
	unsigned char temp[4] = {0};
	unsigned int  inSideFlashCRC = 0;//Ƭ��flash��bootloadersize�����4���ֽڵ�CRC
	unsigned char flg = 1;

	for (i = 0; i < 16; i++)
	{
		DevChip.EFM32DeviceInfo[i] = *((unsigned char*)(EFM32_ADDRESS + i)); //��EFM32_ADDRESS��ַ���16���ֽ��ó�����
	}

	inSideFlashCRC = CRC_calc((uint8_t*)BOOTLOADER_SIZE, (uint8_t*)(DevChip.Device.memFlash * 1024 - 8));
	//�������CRCд�뵽Ƭ��flash�����4���ֽڴ�
	temp[0] = (unsigned char)(inSideFlashCRC & 0xff);
	temp[1] = (unsigned char)((inSideFlashCRC >> 8) & 0xff);
	temp[2] = (unsigned char)((inSideFlashCRC >> 16) & 0xff);
	temp[3] = (unsigned char)((inSideFlashCRC >> 24) & 0xff); //ʹ�ã�uint32_t*������ֱ�ӵ�ַ���������ȶ������Ǹ��ֽ�

	MSC_Init();
	FLASH_CalcPageSize();//�Ȼ�ȡƬ��flash����Ϣ

	flg = MSC_ErasePage((uint32_t*)0x3FC00);//������254-256k������,�����һҳ

	WDOG_Feed();

	while(flg !=  mscReturnOk);

	flg = 1;

	flg = MSC_WriteWord((uint32_t*)(DevChip.Device.memFlash * 1024 - 4), temp , 4); //д�������CRC,д�ĵ�ַ��Ƭ��flash�����4���ֽ�

	while(flg != mscReturnOk);
}

void DealError(void)
{
	unsigned char i = 0;
	volatile unsigned int flashCrcApp = 0;


	for (i = 0; i < 16; i++)
	{
		DevChip.EFM32DeviceInfo[i] = *((unsigned char*)(EFM32_ADDRESS + i)); //��EFM32_ADDRESS��ַ���16���ֽ��ó�����
	}

	flashCrcApp = CRC_calc((uint8_t*)BOOTLOADER_SIZE, (uint8_t*)(DevChip.Device.memFlash * 1024 - 8));//ע���������ķ�Χ��ĩβ������8���ֽ�ǰ��0x9800�Ժ�Ĳ��֡�

	if(flashCrcApp != (*((uint32_t*)(DevChip.Device.memFlash * 1024 - 4))))
	{
		RESET_MCU();//���и�λ
	}
	else//�����ȥִ��App������֮ǰ�ص����Ź�������ص����Ź���ϵͳ�ز�ͣ�ĸ�λ��
	{
		WDOG_Feed();

		BOOT_boot();//��ת��App�����С�
	}

}


void BootWriteIn(void)
{
	unsigned int i , j ;
	unsigned char appBuf[ONCEREADSIZE] = {0};
	unsigned int integer = 0;
	unsigned int decimal = 0;
	int align4 = 0;//4�ֽڶ���
	uint8_t wrFlg = 5;//������ʾ�Ƿ�д�ɹ���
	uint8_t errFlg = 5;



	MSC_Init();//��Ƭ��flash���г�ʼ��������Ҫ������д��
	FLASH_CalcPageSize();//�Ȼ�ȡƬ��flash����Ϣ



	integer = FW_INFO.INFO.fw_length / ONCEREADSIZE;
	decimal = FW_INFO.INFO.fw_length % ONCEREADSIZE;

	SysCtlDelay(500000);//�����ʱȷ������С��app��ʱ���ܹ�˳����д��efm32��ָ��λ�á�

	for(i = 0; i < integer; i++) //����2k��������
	{
		while(IsFlashBusy());

		FlashRead((FW_APPSTART_ADDR + ONCEREADSIZE * i), appBuf, ONCEREADSIZE); //ÿ�ζ���Ƭ���2k

		while(IsFlashBusy());

		/*
			   ����Ҫ��bootloader������д��efm32�ڲ�flash��ַ0������˴�0��ʼ����.
			   ע��:�����ǲ�����debugģʽ�²����ģ���Ϊdebugģʽ�£�ϵͳ�������0��ַ��ʼ�ģ�����������Ҫȥ����0��ַ��Ȼ������
			   loader����ֻ����ram���ܣ�Ȼ��ȥ����flash��
		*/
		errFlg = MSC_ErasePage((uint32_t*)(FW_STARTADDR + i * ONCEREADSIZE)); //ÿ�β���Ƭ�ڵ�2k,��0��ַ����

		while(errFlg != mscReturnOk);

		errFlg = 5;


		for(j = 0; j < ONCEREADSIZE; j++)
		{
			if(*((uint8_t*)(FW_STARTADDR + i * ONCEREADSIZE + j)) != 0xFF) //���û�в�����ȷ������
				while(1);
		}

		wrFlg = MSC_WriteWord((uint32_t*)(FW_STARTADDR + i * ONCEREADSIZE), appBuf , ONCEREADSIZE); //ÿ��д2k����0��ַ��ʼд

		while(wrFlg != mscReturnOk);

		wrFlg = 5;

		WDOG_Feed();


//������Ҫ�ϳ�ʱ�䣬�ڴ�ι��
	}

	if(decimal > 0)//ȷ������ʣ��
	{
		while(IsFlashBusy());

		FlashRead((FW_APPSTART_ADDR + ONCEREADSIZE * integer), appBuf, decimal); //appBuf�д洢�����ϴε����ݡ������ٶ�ȡһ�Σ�����ֻ��һ����

		while(IsFlashBusy());

		errFlg = MSC_ErasePage((uint32_t*)(FW_STARTADDR + integer * ONCEREADSIZE)); //�ٶ����һ��ҳ������д���ʣ�µĲ���һ��ҳ������

		while(errFlg != mscReturnOk);

		errFlg = 5;

		for(j = 0; j < decimal; j++)
		{
			if(*((uint8_t*)(FW_STARTADDR + integer * ONCEREADSIZE + j)) != 0xff)
				while(1);
		}

		if((decimal % 4) == 0)//˵��ʣ����ֽ����ܱ�4��������ֱ��д��flash
		{
			wrFlg = MSC_WriteWord((uint32_t*)(FW_STARTADDR + integer * ONCEREADSIZE), appBuf , decimal);

			while(wrFlg != mscReturnOk);

			wrFlg = 5;
			WDOG_Feed();
		}
		else
		{
			align4 = decimal % 4;

			switch(align4)
			{
				case 1://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff; //ע�ⲹ����ʼ��ַ
					appBuf[decimal + 1] = 0xff;
					appBuf[decimal + 2] = 0xff;
					wrFlg =  MSC_WriteWord((uint32_t*)(FW_STARTADDR + integer * ONCEREADSIZE), appBuf , decimal + 3);

					while(wrFlg != mscReturnOk);

					wrFlg = 5;

					WDOG_Feed();
				}
				break;

				case 2://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					appBuf[decimal + 1] = 0xff;
					wrFlg = MSC_WriteWord((uint32_t*)(FW_STARTADDR + integer * ONCEREADSIZE), appBuf , decimal + 2);

					while(wrFlg != mscReturnOk);

					wrFlg = 5;
					WDOG_Feed();
				}
				break;

				case 3://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					wrFlg = MSC_WriteWord((uint32_t*)(FW_STARTADDR + integer * ONCEREADSIZE), appBuf , decimal + 1);

					while(wrFlg != mscReturnOk);

					wrFlg = 5;
					WDOG_Feed();
				}
				break;

				default:
					break;
			}
		}
	}
}



void BootUpdate(void)
{
	unsigned int bootCRC = 0;
	uint8_t retry = 0;

	bootCRC = FlashCRC(0 + 16, FW_INFO.INFO.fw_length); //ֻ�����ݵ�CRC�������ļ�ͷ��

	if(bootCRC == FW_INFO.INFO.fw_crc) //��������CRC�ǶԵ�2014��5��4��17:41:53
	{

		BootWriteIn();//��ɴ�Ƭ���bootloader������д��Ƭ��flash��

		bootCRC = CRC_calc((uint8_t*)FW_STARTADDR, (uint8_t*)(FW_STARTADDR + FW_INFO.INFO.fw_length - 1));

		if(bootCRC == FW_INFO.INFO.fw_crc)//У��ղ�д���bootloader����ȷ��
		{

			while(IsFlashBusy());

			FlashSectorErase( 0 );

			while(IsFlashBusy());

			RESET_MCU();//���и�λ
		}
		else//���д���ˣ�������д4��
		{
			while(retry < 4)
			{
				retry++;
				BootWriteIn();
				bootCRC = CRC_calc((uint8_t*)FW_STARTADDR, (uint8_t*)(FW_STARTADDR + FW_INFO.INFO.fw_length - 1));

				if(bootCRC == FW_INFO.INFO.fw_crc)
				{
					while(IsFlashBusy());

					FlashSectorErase( 0 );

					while(IsFlashBusy());

					RESET_MCU();//���и�λ
				}
			}

			DealError();//�����д4�κ��ǲ��ԣ�ֱ�ӽ����쳣����
		}
	}
	else
	{

		DealError();
	}
}


void McuFwUpdate(void)
{
	unsigned int  calcCRC = 0;
	static uint8_t retry = 0;

	calcCRC = FlashCRC(0 + 16, FW_INFO.INFO.fw_length); //��������Ƭ��flash��CRC

	//�����Ƭ��flash�����CRC���ڱ������ļ�ͷ��CRC������Զ�ȡflash��ֵ��д��Ƭ��flash
	if(calcCRC == FW_INFO.INFO.fw_crc)
	{

		InsideFlashWR();//��ȡƬ��flash�е����ݲ�д��Ƭ�ڡ�����Ҫ����Ƭ��flash��CRC

		calcCRC = CRC_calc((uint8_t*)BOOTLOADER_SIZE, (uint8_t*)(BOOTLOADER_SIZE + FW_INFO.INFO.fw_length - 1)); //ע������Ҫ��1��ע���Ǵ�0��ʼ��

		if(calcCRC == FW_INFO.INFO.fw_crc)//���д�뵽Ƭ��Flash��App��CRC��Ƭ���ļ�ͷ�д洢��CRC��һ��Ҫ����д
		{
			//����Ƭ��flash��CRC��������д�����4���ֽڴ�
			CalcInsideFlashCRCandWrIn();

			//����Ƭ��flash���ļ�ͷ

			while(IsFlashBusy());

			FlashSectorErase( 0 );

			while(IsFlashBusy());

			RESET_MCU();//���и�λ
		}
		else//���Ƶ�Ƭ��flash�����ݵ�CRC��Ƭ��flash���ļ�ͷ�м�¼�Ĳ�һ��,ֱ���˳����쳣������
		{
			while(retry < 3)
			{
				retry++;
				InsideFlashWR();//������дһ��
				calcCRC = CRC_calc((uint8_t*)BOOTLOADER_SIZE, (uint8_t*)(BOOTLOADER_SIZE + FW_INFO.INFO.fw_length - 1));

				if(calcCRC == FW_INFO.INFO.fw_crc)
				{
					CalcInsideFlashCRCandWrIn();

					while(IsFlashBusy());

					FlashSectorErase( 0 );

					while(IsFlashBusy());

					RESET_MCU();
				}
			}

			DealError(); //�����ظ�д4�Σ����ǲ��ԵĻ����ͻ���ȥ��λ����������ߵ���������������
		}
	}
	else//Ƭ��flash���ݵ�CRC�����ļ�ͷ�д洢�Ĳ�һ��
	{
		DealError();
	}
}

void ReadBLEFwFromExflashAndWrIn(void)
{

	unsigned char bleFwBuf[128] = {0};
	int integer = 0;
	int decimal = 0;
	int temp    = 0;
	volatile unsigned int i = 0;
	static char page = 0;


	integer = FW_INFO.INFO.fw_length / 128; //  �ܹ���1952��128byte
	decimal = FW_INFO.INFO.fw_length % 128; //  ������0

	for( i = 0; i < integer; i++)
	{

		memset(CopyRxBuff, 0, 64);

		while(IsFlashBusy());

		FlashRead((FW_APPSTART_ADDR + 128 * i), bleFwBuf, 128); //ÿ�ζ���128�ֽ�

		while(IsFlashBusy());

		WriteCC254xFlash(bleFwBuf);

		char countTx = 60;

		while(!BLE_Responsed  && countTx )//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue
		{
			countTx--;
			SysCtlDelay(10000);//����������ʱ�ٻᶪ����
		}//ÿ����1�����ݣ�Ҫ���㹻��ʱ����BLE��Ӧ��Ȼ���Ӧ���ͻ��ˡ�

		if(countTx > 0)//˵��û�г�ʱ
		{

			if( (BLE_DevChip.BLE_Device.FW_VER1 == 2) && (BLE_DevChip.BLE_Device.FW_VER2 == 0))
			{
				//���ݾɰ汾��BLE��bootloader�����ﲻ������
			}
			else
			{
				page = (unsigned char)(i / 16);

				if((CopyRxBuff[0] == 0x3c ) && (CopyRxBuff[2] == 0x05))//�鿴���ص�ҳ�Բ��ԣ����ҳ����ֱ�ӵ��豸APP
				{
					if(CopyRxBuff[5] != (page + 1)) //
					{
						DealError();
					}
				}
			}
		}


		BLE_Responsed = false;

		if(i % 5 == 0)//ÿд5��ι��
		{
			WDOG_Feed();
		}

	}

	if(decimal > 0)
	{
		temp = decimal % 4;//BLE��flashҲҪ4���ֽڶ���д��

		FlashRead((FW_APPSTART_ADDR + 128 * integer), bleFwBuf, decimal); //����ֻ��ʣ�µ�decimal�ֽ�

		while(IsFlashBusy());

		if(temp == 0)
		{

			MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

			WDOG_Feed();

			while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

			BLE_Responsed = false;
		}
		else
		{
			WDOG_Feed();

			switch(temp)
			{
				case 1:
				{
					bleFwBuf[decimal + 0] = 0xff;
					bleFwBuf[decimal + 1] = 0xff;
					bleFwBuf[decimal + 2] = 0xff;

					MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

					while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

					BLE_Responsed = false;

				}
				break;

				case 2:
				{
					bleFwBuf[decimal + 0] = 0xff;
					bleFwBuf[decimal + 1] = 0xff;

					MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

					while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

					BLE_Responsed = false;

				}
				break;

				case 3:
				{
					bleFwBuf[decimal + 0] = 0xff;

					MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

					while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

					BLE_Responsed = false;
				}
				break;

				default:
					break;
			}
		}
	}
}



/*
ע��IAR�����Ż������ѡ��ͬ��Գ�������ܴ�Ĳ�ͬ��
���Ż�������Highʱ�������е���Щ�������ܱ��������Ż��������²������á�
������Գ���ʱ��ע�����ģʽ�ͷ���ģʽ���Ż�����Ҫѡ��һ�£������п����ڵ���ģʽ��ͨ���������ڷ��а汾�о�ͬ����
*/

void NewBleFwUpdate(void)
{
	unsigned int  calcCRC = 0;
	volatile unsigned char countChange = 160;
	volatile unsigned char countStart = 50;
	volatile char countCrc = 20;
	volatile char resetBle = 3;
	volatile char bleInfoCount = 60;
	volatile char enter = 0;


	SysCtlDelay(8000);//����ط������80000����ô�����ߵ�����ʱ��BLE_ONLINE

	while((BLE_ONLINE == false)  && resetBle)
	{
		resetBle--;

		BLE_RST_L();
		SysCtlDelay(8000 * SYSCLOCK);
		BLE_RST_H();

		SysCtlDelay(800000);
		SysCtlDelay(800000);
		SysCtlDelay(800000);//Ӳ����λһ�µȴ�ble����Ӧ
	}

	if(resetBle == 0 )//˵��3��Ӳ����λ��û�л��ѣ�ֱ��ȥMCU��APP
	{
		DealError();
	}


	calcCRC = FlashCRC(0 + 16, FW_INFO.INFO.fw_length); //��������Ƭ��flash��CRC

	if(calcCRC == FW_INFO.INFO.fw_crc)
	{

		while ((enter != 1) && bleInfoCount)
		{
			if((bleStFlg == 1) || (bleStFlg == 2))//�����ǰ�����״̬��Ϣ�����������һ��˵����APP״̬���ڶ�����boot״̬
			{
				enter = 1;
			}

			bleInfoCount--;
			SysCtlDelay(800000);
		}

		if(bleInfoCount == 0) //�ȴ��ܾ�û�л�ȡ������Ϣ�˳�
		{
			DealError();
		}

		if(bleStFlg == 1)
		{
			bleStFlg = 0x55;
			memcpy(BLE_DevChip.BLE_DeviceInfo, &CopyRxBuff[UART_ID_DATA + 1], sizeof(BLE_DevChip.BLE_DeviceInfo));

			if((BLE_DevChip.BLE_Device.WORKSTA == 1))//��ʾApp״̬
			{
				AppToBoot();//����ת������󣬵ȴ���Ӧ

				SysCtlDelay(1000);

				while(BLE_DevChip.BLE_Device.WORKSTA && countChange)
				{
					memcpy(BLE_DevChip.BLE_DeviceInfo, &CopyRxBuff[UART_ID_DATA + 1], sizeof(BLE_DevChip.BLE_DeviceInfo));

					//			  SysCtlDelay(50000);
					if(BLE_DevChip.BLE_Device.WORKSTA)//�������1˵���������Ǹ�״̬ת������û����Ӧ���ڷ���һ��
					{
						AppToBoot();
						SysCtlDelay(50000);
						memcpy(BLE_DevChip.BLE_DeviceInfo, &CopyRxBuff[UART_ID_DATA + 1], sizeof(BLE_DevChip.BLE_DeviceInfo));
						countChange--;
					}
				}

				if(countChange == 0)//״̬ת����ʱ��ֱ���˳�
				{
					DealError();
				}
			}

		}

		SysCtlDelay(800000);//Ҫ���㹻����ʱʹ��һ������ִ����ϸ���Ӧ����ڷ�����һ�����


		if((bleStFlg == 2) || (BLE_DevChip.BLE_Device.WORKSTA == 0))//���ble������boot����ͨ��״̬װ������֮��boot����ʼд���ݡ�
		{
			bleStFlg = 0x55;

			memcpy(BLE_DevChip.BLE_DeviceInfo, &CopyRxBuff[UART_ID_DATA + 1], sizeof(BLE_DevChip.BLE_DeviceInfo));

			MyBLE_Update_Start(); //����ȷ��������������ǰ����������boot״̬�����ܽ���������ʼ�����

			SysCtlDelay(80000);

			while(!BLE_Responsed  && countStart)//���������Ϊ�˺ι�������Ҫһֱ���Ϳ�ʼ����  2014��6��21��13:32:29
			{
				MyBLE_Update_Start();
				countStart--;
				SysCtlDelay(800000);
			}

			if(countStart == 0) //��ʼ��������û����Ӧ,ֱ���˵�MCU��APP
			{
				DealError();
			}

			BLE_Responsed = false;


			ReadBLEFwFromExflashAndWrIn();	//���д��ʱ��ȽϾ�	,�����������BLE_response=false

			WDOG_Feed();

			SysCtlDelay(50000);

			BLE_Update_End(FW_INFO.INFO.fw_crc);//���ͽ�����������,��������BLE��FW��CRC

			WDOG_Feed();//����ȴ�ʱ��ϳ����ڴ�ι����

			SysCtlDelay(800000);
			SysCtlDelay(800000);

			while(!BLE_Responsed && countCrc)
			{
				BLE_Update_End(FW_INFO.INFO.fw_crc);//���ͽ�����������,��������BLE��FW��CRC
				countCrc--;
				SysCtlDelay(800000);
				SysCtlDelay(800000);
				SysCtlDelay(800000);
				SysCtlDelay(800000);
				SysCtlDelay(800000);
			}//��������������Ҳ����Ӧ��ġ�

			if(countCrc == 0)
			{
				DealError();
			}

			BLE_Responsed = false;


			countChange = 160;


			while((bleStFlg != 1) && countChange)
			{
				countChange--;
				SysCtlDelay(900000);
			}

			if(countChange > 0)	//�յ�APP��Ϣ�Ų���
			{
				while(IsFlashBusy());

				FlashSectorErase( 0 );

				while(IsFlashBusy());
			}

			RESET_MCU();//����BLE���������������ϵͳ��λ��
		}

	}
	else
	{
		DealError();
	}
}



//���ϵ��¡��̻ƳȺ죬��BLE����
int main(void)
{

	unsigned int  fwHeadCRC;
	uint8_t  deviceCheck = 0;//�����ж�����I4����I2��

#ifndef DBG
	SCB->VTOR = 0x20000000;
#endif


	LETIMER_setup(); //���Ź���led��˸


	BLE_INIT();

	FLASH_POWER_BACK();//��Դһ�´�

	SysCtlDelay(10000);

	M25Pxx_INIT();


	GetFlashCapacity(0x37);//Ƭ��flash��Ϣ��ȡ


	WDOG_Init(&init);


//��ȡInformation block��USER data��������
	MSC_Init();

	SysCtlDelay(10000);
	FlashRead(FW_STARTADDR , FW_INFO.INFO_BUF , FWHEAD_LENGTH);//���ļ�ͷ

	while(IsFlashBusy());


	fwHeadCRC = CRC_calc(&FW_INFO.INFO_BUF[0], &FW_INFO.INFO_BUF[13]);//�Ѷ������ļ�ͷ����CRCУ��

	if(FW_INFO.INFO_BUF[10] == 0x40)//�ж��豸��������I4
	{
		deviceCheck = 1;
	}

	if(fwHeadCRC == FW_INFO.INFO.Head_CRC)//���ļ�ͷ�ǶԵġ�
	{

		switch(FW_INFO.INFO.fw_type)
		{
			case FWTYPE_MCU://efm32����
			{
				if(deviceCheck)
				{
					McuFwUpdate();
				}
				else
				{
					DealError();
				}
			}
			break;

			case FWTYPE_BLE ://���ͺ���2��BLE��
			{
				NewBleFwUpdate();

			}
			break;

			case FWTYPE_BOOT://bootloader��������
			{
				BootUpdate();
			}
			break;

			default://Ӧ���״���д��Ƭ��flash��û�����ݵ����
			{
				DealError();
				break;
			}
		}
	}
	else//�ļ�ͷ��CRC����
	{
		DealError();
	}

//  while(1);//loader����Զ����ִ�е��������ϵͳ������loader����
	/*
	  �þ䲻��Ҫ�������һ����д��Ƭ��flash��û�����ݣ��ᵼ�������ļ�ͷCRC�ļ��ͨ��
	  ���ǣ���û�к��ʵ��ļ�ͷ������ͻ��˵������ôϵͳ����Զ����loader�г���ȥ��
	  ����취��������switch�е�default�м���dealError������ʹϵͳ�Զ���ת��MCU�ڲ�CRC������
	  �ж���Ҫִ��App���Ǹ�λ��
	  */

}


