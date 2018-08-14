#include <string.h>
#include <stdbool.h>
#include "efm32.h"
#include "boot.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_timer.h"
#include "em_adc.h"
#include "em_wdog.h"
#include "em_rmu.h"
#include "em_rmu.h"
#include "GlobalData.h"



#include "bootld.h"

#include "main.h"
#include "res.h"

//After reset, the HFRCO frequency is 14 MHz
#define CORE_FREQ_HIGH cmuHFRCOBand_14MHz
#define CORE_FREQ_LOW  cmuHFRCOBand_1MHz

#define BAT_GO_LEVEL 3.5 //3.6

//����LED
#define LED_GPIOPORT  (gpioPortC)
#define LED_PIN   (10)

#define LED_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
#define LED_ON()       GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
#define LED_OFF()      GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )


//����Ǿ��Ե�ַ���ʣ���Ҫ��linker�ļ����ڼ���
//place at address mem: 0x000097FC    { readonly section ConstSection1 };�Ϳ����ھ���λ����д�ϰ汾�š�

#pragma location = "ConstSection1"
__root const char abc1[4] = {0, 0, BOOT_VER_M , BOOT_VER_S};


void initGPIO(void)
{

	CMU_ClockEnable(cmuClock_GPIO, true);//��GPIO��ʱ��
	GPIO_PinModeSet(KEY_GPIOPORT, KEY_PIN, gpioModeInputPull, 1);
	GPIO_PinModeSet(LED_GPIOPORT, LED_PIN, gpioModePushPull, 0); //����LEDģʽ
	GPIO_IntConfig(KEY_GPIOPORT, KEY_PIN, false, true, true);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	//NVIC_EnableIRQ(GPIO_ODD_IRQn);

	GPIO_PinModeSet(AFE_POWER_CON_PORT, AFE_POWER_CON_PIN, gpioModePushPull, 0); //��AFE�ĵ�Ҳ�ص���
	BOTTOM_POWER_OFF();

	//BOTTOM_POWER_ON();

	GPIO_PinModeSet(BACKLIGHT_PORT, BACKLIGHT_PIN, gpioModePushPull, 0);
	BACKLIGHT_ON();
	BACKLIGHT_OFF();

	GPIO_PinModeSet(MOTOR_PORT, MOTOR_PIN, gpioModePushPull, 0);
	MOTOR_ON();
	MOTOR_OFF();

	EXTVCC_ON();
	EXTVCC_OFF();

	GPIO_PinModeSet(PWM1_PORT, PWM1_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(PWM2_PORT, PWM2_PIN, gpioModePushPull, 0);
	PWM1_OFF();
	PWM2_OFF();
//������Ҫ�ǹر�һЩ����

}


long bat_val[4] = {0, 0, 0, 0}, bat_avg = 0; //�������

float BAT_VCC = 0;

unsigned char bat_index = 0, bat_checked = 0;

int timerCount = 0;//��ʱ��������

/* 976 Hz -> 1Mhz (clock frequency) / 1024 (prescaler)
  Setting TOP to 488 results in an overflow each 0.1 seconds */

#define TOP 100



//�͹��Ķ�ʱ���ж�
void LETIMER0_IRQHandler(void)
{
	LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0); //����Ƚ���0���жϱ�־
	timerCount++;


	while (ADC0->STATUS & ADC_STATUS_SINGLEACT);

	bat_val[bat_index++] = (long)ADC_DataSingleGet(ADC0); //���ص���ת��ֵ��

	if(bat_index >= 4) //���������ж�4�Σ�˵����ؼ�⵽��
	{
		bat_index = 0;
		bat_checked = 1; //˵����ؼ�⵽��
	}

	ADC_Start(ADC0, adcStartSingle);//��ʼɨ��򵥴�ת����@p1,ADC���ͣ�@p2ADC��ʼ�����͡�

}

const	ADC_InitSingle_TypeDef BAT_ADC_INIT =
{
	adcPRSSELCh0, /*   PRS ch0 (if enabled). */ 		   \
	adcAcqTime8,  /* 1 ADC_CLK cycle acquisition time. */	  \
	adcRef2V5,	/* 2.5V internal reference. */ 		  \
	adcRes12Bit, /* 12 bit resolution. */			   \
	adcSingleInpCh7, /* CH0 input selected. */		   \
	false, /* Single ended input. */			   \
	false,		/* PRS disabled. */ 		\
	false,	/* Right adjust. */ 								 \
	false  /* Deactivate conversion after one scan sequence. */ \
};

void ADC_INIT(void)
{
	CMU_ClockEnable(cmuClock_ADC0, true);

	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;//��ֵ�Ǻ궨�塣

	/* Init common settings for both single conversion and scan mode */
	init.timebase = ADC_TimebaseCalc(0);//����ʱ������������Ϊ0��ʾ�õ�ǰ���õĸ�Ƶ����ʱ��

	// init.lpfMode =adcLPFilterRC;//adcLPFilterDeCap;

	//init.warmUpMode=adcWarmupKeepScanRefWarm;

	/* Set ADC clock to 7 MHz, use default HFPERCLK */
	init.prescale = ADC_PrescaleCalc(7000000, 0);

	/* Set oversampling rate */
	//init.ovsRateSel = adcOvsRateSel32;//adcOvsRateSel32;

	ADC_Init(ADC0, &init);

	ADC_InitSingle(ADC0, &BAT_ADC_INIT);

}

//���״̬��ʼ��
void CHARGER_STA_INIT(void)
{
	/* Enable GPIO */
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* Configure GPIO port A2 as Key input ��PF4Ϊ���״̬����*/
	GPIO_PinModeSet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, gpioModeInputPull, 1);

	GPIO_IntConfig(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, true, true, true);

	GPIO_IntClear(1 << CHARGER_STA_PIN);

	/* Enabling Interrupt from GPIO_EVEN */
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);//���ж�ʹ��

}

unsigned char Charging = 0;

void GPIO_EVEN_IRQHandler(void)
{
	uint32_t status;
//����ж��¼�������
	status = GPIO->IF;
	GPIO->IFC = status;

	if (status & (1 << CHARGER_STA_PIN))
	{
		if(GPIO_PinInGet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN) == 0) //���������Ż��ǵ͵Ļ�����ʾ���ڳ塣
			Charging = 1;
		else
			Charging = 0;
	}
}


unsigned char charging_Counter = 0; //����������

/**************************************************************************//**
 * The main entry point.
 *****************************************************************************/

int main(void)
{
	/*��ʼ�����Ź��ṹ��*/
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
		.perSel     = wdogPeriod_4k,      /* Set the watchdog period to 4097 clock periods (ie ��Լ4s seconds)*/ //persel=10
	};

	//T = (2^(3+perSel)+1)/f;����f��ʾ��ѡ��Ŀ��Ź�ʱ�ӣ�����f=1Khz,perSel = 10;���ոøñ���ʽ���㿴�Ź�������8s

	CHIP_Init();

	initGPIO();

	//---------------------LETimer0���� ����Ҫ����led�ڲ�ͬ״���µ���˸Ƶ�ʣ��Լ�ι��Ƶ��
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);//�ȴ���Ƶ����������
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); //��������ʱ����Ҫ�ⲿ��Ƶ������Ϊʱ��Դ��
	CMU_ClockEnable(cmuClock_CORELE, true);//RTC�����еĵ͹������趼Ҫ��CORELF��ͬʱҲ�ǿ��Ź���ʱ��Դ
	CMU_ClockEnable(cmuClock_LETIMER0, true); //��ʹ���Լ���ʱ��

	LETIMER_Init_TypeDef leTimerinit = LETIMER_INIT_DEFAULT;//�Ƚ���Ĭ�ϳ�ʼ��

	leTimerinit.debugRun = true;
	leTimerinit.comp0Top = true;
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0); //�Ƚ���0�ж�ʹ��
	NVIC_EnableIRQ(LETIMER0_IRQn);//��NVIC�ж�
	LETIMER_CompareSet(LETIMER0, 0, 32768 / 10); //10HZ
	LETIMER_Init(LETIMER0, &leTimerinit);


	WDOG_Init(&init);//�����￪ʼ�󣬿��Ź����Ѿ������ˡ�

	//====================================

	ADC_INIT();

	SysCtlDelay(8000);//��ADCʱ�䣬ʹ����ɳ�ʼ��������

	//====================================
	CHARGER_STA_INIT();

	if(GPIO_PinInGet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN) == 0)
		Charging = 1; //��ʾ���ڳ��
	else
		Charging = 0;	//û���ʱֱ�ӵ�����

	//  ===================================

	while(1)
	{
		EMU_EnterEM2(true);

		WDOG_Feed();


		if(bat_checked)//�ڶ�ʱ��1���ж��н��е�ؼ�⣬�����⵽���
		{
			bat_checked = 0;
			bat_avg = 0;

			for(int i = 0; i < 4; i++)
			{
				bat_avg += bat_val[i]; //bat_val[]��ȫ�ֱ�������timer1���ж��а�ÿ�ε�ز���ֵ���������
			}

			bat_avg = bat_avg / 4;

			BAT_VCC = 2 * bat_avg * 2.5 / 4096;
		}

		if(BAT_VCC > BAT_GO_LEVEL) //����ѹ
			break;
		else if(Charging == 1)//��ʾ���Ļ���
		{
			if(timerCount > 4)
			{
				timerCount = 0;
				LED_TOGGLE();
			}
		}
		else
		{
			if(timerCount >= 19)
			{
				timerCount = 0;
				LED_TOGGLE();
			}
		}
	}

	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);

	__set_MSP( ( 0x20000000 + sizeof( bootloader ) + 0x800 ) & 0xFFFFFFF0 );//��������롰0xfffffff0�롱��Ҫ��Ϊ�˶��뵽256�ֽ���
	//��������ջָ��


	WDOG_Feed();//�ڰ���ǰ����ι��һ��

	/* Load the entire bootloader into SRAM. */
	memcpy( (void*)0x20000000, bootloader, sizeof( bootloader ) );//����bootloader��һ��������bootld.h,����usbhhidkbd���̵�bin�ļ�ת���ɵ�.hex�ļ���

	/* Start executing the bootloader. */

	WDOG_Enable(false);

	SysCtlDelay(50);//�ؿ��Ź���Ҫʱ�䣿

	BOOT_jump( *(uint32_t*)0x20000000, *(uint32_t*)0x20000004 );//@p1,��sp,@p2,��pc�����ڳ���ʹ�0x20000004ִ�С�
	/*��λ�ж������ĵ�ַ��00000004�������Ϊʲô��PC���ó�200000004*/
}