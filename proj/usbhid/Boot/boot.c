/**************************************************************************//**
 * @file
 * @brief Boot Loader
 * @author Energy Micro AS
 * @version 1.02
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2011 Energy Micro AS, http://www.energymicro.com</b>
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
#include <stdbool.h>

#include "efm32.h"
#include "config.h"
#include "boot.h"
#include "em_wdog.h"
#include "m25pxx.h"
#include "main.h"

/**************************************************************************//**
 * @brief Checks to see if the reset vector of the application is valid
���Ӧ�ó���ĸ�λ�������Ƿ���ȷ
 * @return false if the firmware is not valid, true if it is.
 *****************************************************************************/
bool BOOT_checkFirmwareIsValid(void)
{
  uint32_t pc;

  pc = *((uint32_t *) BOOTLOADER_SIZE + 1);

  if (pc < MAX_SIZE_OF_FLASH)
    return true;

  return false;
}

/**************************************************************************//**
 * @brief This function sets up the Cortex M-3 with a new SP and PC.�����µĶ�ջָ��ͳ��������
 *****************************************************************************/
#if defined ( __CC_ARM   )
__asm void BOOT_jump(uint32_t sp, uint32_t pc)
{
  /* Set new MSP, PSP based on SP (r0)*/
  msr msp, r0
  msr psp, r0

  /* Jump to PC (r1)*/
  mov pc, r1
}
#else
void BOOT_jump(uint32_t sp, uint32_t pc)
{
  (void) sp;
  (void) pc;

  /* Set new MSP, PSP based on SP (r0)*/
  __asm("msr msp, r0");
  __asm("msr psp, r0");

  /* Jump to PC (r1)*/
  __asm("mov pc, r1");
}
#endif

/**************************************************************************//**
 * @brief Boots the application
 *****************************************************************************/
void BOOT_boot(void)
{
  uint32_t pc, sp;

  /* Reset all used registers to their default value. */

  /* Disable all interrupts. */
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0xFFFFFFFF;

//  /* Disable USB */
  

  /* Reset memory system controller settings. */
  MSC->READCTRL  = _MSC_READCTRL_RESETVALUE;//�ڴ�ϵͳ���ƿ�����ã�����Ѷ���д���ó�Ĭ��
  MSC->WRITECTRL = _MSC_WRITECTRL_RESETVALUE;
  MSC->LOCK = 0;//��λ���ó�0����MSC�Ĵ�������������

  /* Reset GPIO settings. */
  GPIO->ROUTE = _GPIO_ROUTE_RESETVALUE;
  GPIO->P[4].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[4].DOUT  = _GPIO_P_DOUT_RESETVALUE;
  GPIO->P[5].MODEL = _GPIO_P_MODEL_RESETVALUE;
  GPIO->P[5].DOUT  = _GPIO_P_DOUT_RESETVALUE;
  
  GPIO->P[2].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[2].DOUT  = _GPIO_P_DOUT_RESETVALUE; //���ǹر�LED PC10
  
  FLASH_POWER_DOWN();//�ص�flash�ĵ�Դ
  GPIO->P[3].MODEL = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[3].DOUT  = _GPIO_P_DOUT_RESETVALUE;//flash�ĵ�Դ���Źر�PD5

  
  /* Reset DMA controller settings. */
  DMA->CONFIG     = _DMA_CONFIG_RESETVALUE;
  DMA->CTRLBASE   = _DMA_CTRLBASE_RESETVALUE;
  DMA->CH[0].CTRL = _DMA_CH_CTRL_RESETVALUE;
  DMA->CHENC      = 0xFFFFFFFF;//ͨ��ʹ������Ĵ���

  /* Reset TIMER0 settings.*/
  TIMER0->CMD        = TIMER_CMD_STOP;
  TIMER0->TOP        = _TIMER_TOP_RESETVALUE;
  TIMER0->CTRL       = _TIMER_CTRL_RESETVALUE;
  TIMER0->CC[0].CTRL = _TIMER_CC_CTRL_RESETVALUE;
  /*��ʱ��1���õ�boot�е�س��adc����Ƶ��*/
  TIMER1->CMD        = TIMER_CMD_STOP;
  TIMER1->TOP        = _TIMER_TOP_RESETVALUE;
  TIMER1->CTRL       = _TIMER_CTRL_RESETVALUE;
  TIMER1->CC[0].CTRL = _TIMER_CC_CTRL_RESETVALUE;
  
  /*���LETIMER0*/
  LETIMER0->CMD      = LETIMER_CMD_STOP;//�رյ͹��Ķ�ʱ��0
  LETIMER0->COMP0    = _LETIMER_COMP0_RESETVALUE;//�����¼����ģ��������ó�0
  LETIMER0->CTRL     = _LETIMER_CTRL_RESETVALUE;
  
  /*�رտ��Ź�*/
   WDOG_Enable(false);

   SysCtlDelay(500);//�ؿ��Ź���Ҫʱ��
  

  

//   while(1);//���������Ƿ�ѿ��Ź��ر��ˣ�2014��4��28��11:47:21
  
 //оƬ��ʵʱʱ������  
  /* Reset RTC settings. */
  RTC->IEN    = _RTC_IEN_RESETVALUE;//������0�������ж�ʹ��
  RTC->COMP0  = _RTC_COMP0_RESETVALUE;//��ֵ��0����RTCһ���Ͳ����ж��ˣ�������жϵĻ�
  RTC->CTRL   = _RTC_CTRL_RESETVALUE;

  /* Wait for LF peripheral syncronization. */
  //while (RTC->SYNCBUSY & _RTC_SYNCBUSY_MASK);
  //while (CMU->SYNCBUSY & CMU_SYNCBUSY_LFACLKEN0);

 
  /* Switch to default cpu clock. */
  CMU->CMD      = CMU_CMD_HFCLKSEL_HFRCO;//ת����HFRCO����ƵRC����������CMU��CMD�ļĴ���
  CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS | CMU_OSCENCMD_LFRCODIS;//������ʹ�ܻ�ʹ�ܼĴ���������Ѹ�Ƶ����͵�Ƶ������������

  /* Reset clock registers used. */
  CMU->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
  CMU->HFPERCLKDIV  = _CMU_HFPERCLKDIV_RESETVALUE;
  CMU->HFPERCLKEN0  = _CMU_HFPERCLKEN0_RESETVALUE;
  CMU->LFCLKSEL     = _CMU_LFCLKSEL_RESETVALUE;
  CMU->LFACLKEN0    = _CMU_LFACLKEN0_RESETVALUE;
  
  /* Set new vector table pointer �����µ�������ָ��*/
  SCB->VTOR = (uint32_t)BOOTLOADER_SIZE;//���СΪ38K
  
  /*
  SCB��ϵͳ���ƿ飬��Generic user guide ����˵��������������Ĵ�����������������Ĵ�����
  ���������ջָ��ĸ�λֵ���쳣����������cpu�������쳣��������
  ��λ���������ĵ�ַ�̶��ڵ�ַ0�ϣ���������Ȩģʽ�¿��Զ�VTORд�������������ʼ��ַ���¶�λ������λ���ϡ�����ķ�Χ��
  0x00000080��0x3fffff80���䡣
  */

  /* Read new SP and PC from vector table */
  sp = *((uint32_t *)BOOTLOADER_SIZE    );
  pc = *((uint32_t *)BOOTLOADER_SIZE + 1);

  BOOT_jump(sp, pc);//���������µĶ�ջָ��ͳ��������
}
