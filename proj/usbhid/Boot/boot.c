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
检查应用程序的复位向量表是否正确
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
 * @brief This function sets up the Cortex M-3 with a new SP and PC.设置新的堆栈指针和程序计数器
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
  MSC->READCTRL  = _MSC_READCTRL_RESETVALUE;//内存系统控制块的配置，这里把读、写配置成默认
  MSC->WRITECTRL = _MSC_WRITECTRL_RESETVALUE;
  MSC->LOCK = 0;//锁位配置成0，把MSC寄存器进行了锁。

  /* Reset GPIO settings. */
  GPIO->ROUTE = _GPIO_ROUTE_RESETVALUE;
  GPIO->P[4].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[4].DOUT  = _GPIO_P_DOUT_RESETVALUE;
  GPIO->P[5].MODEL = _GPIO_P_MODEL_RESETVALUE;
  GPIO->P[5].DOUT  = _GPIO_P_DOUT_RESETVALUE;
  
  GPIO->P[2].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[2].DOUT  = _GPIO_P_DOUT_RESETVALUE; //这是关闭LED PC10
  
  FLASH_POWER_DOWN();//关掉flash的电源
  GPIO->P[3].MODEL = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[3].DOUT  = _GPIO_P_DOUT_RESETVALUE;//flash的电源引脚关闭PD5

  
  /* Reset DMA controller settings. */
  DMA->CONFIG     = _DMA_CONFIG_RESETVALUE;
  DMA->CTRLBASE   = _DMA_CTRLBASE_RESETVALUE;
  DMA->CH[0].CTRL = _DMA_CH_CTRL_RESETVALUE;
  DMA->CHENC      = 0xFFFFFFFF;//通道使能清除寄存器

  /* Reset TIMER0 settings.*/
  TIMER0->CMD        = TIMER_CMD_STOP;
  TIMER0->TOP        = _TIMER_TOP_RESETVALUE;
  TIMER0->CTRL       = _TIMER_CTRL_RESETVALUE;
  TIMER0->CC[0].CTRL = _TIMER_CC_CTRL_RESETVALUE;
  /*定时器1是用到boot中电池充电adc采样频率*/
  TIMER1->CMD        = TIMER_CMD_STOP;
  TIMER1->TOP        = _TIMER_TOP_RESETVALUE;
  TIMER1->CTRL       = _TIMER_CTRL_RESETVALUE;
  TIMER1->CC[0].CTRL = _TIMER_CC_CTRL_RESETVALUE;
  
  /*清除LETIMER0*/
  LETIMER0->CMD      = LETIMER_CMD_STOP;//关闭低功耗定时器0
  LETIMER0->COMP0    = _LETIMER_COMP0_RESETVALUE;//是向下计数的，这里设置成0
  LETIMER0->CTRL     = _LETIMER_CTRL_RESETVALUE;
  
  /*关闭看门狗*/
   WDOG_Enable(false);

   SysCtlDelay(500);//关看门狗需要时间
  

  

//   while(1);//在这里检测是否把看门狗关闭了？2014年4月28日11:47:21
  
 //芯片的实时时钟设置  
  /* Reset RTC settings. */
  RTC->IEN    = _RTC_IEN_RESETVALUE;//这里是0，不打开中断使能
  RTC->COMP0  = _RTC_COMP0_RESETVALUE;//该值是0，即RTC一开就产生中断了，如果打开中断的话
  RTC->CTRL   = _RTC_CTRL_RESETVALUE;

  /* Wait for LF peripheral syncronization. */
  //while (RTC->SYNCBUSY & _RTC_SYNCBUSY_MASK);
  //while (CMU->SYNCBUSY & CMU_SYNCBUSY_LFACLKEN0);

 
  /* Switch to default cpu clock. */
  CMU->CMD      = CMU_CMD_HFCLKSEL_HFRCO;//转换到HFRCO（高频RC振荡器），给CMU的CMD的寄存器
  CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS | CMU_OSCENCMD_LFRCODIS;//振荡器的使能或不使能寄存器，这里把高频晶振和低频振荡器都不是能

  /* Reset clock registers used. */
  CMU->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
  CMU->HFPERCLKDIV  = _CMU_HFPERCLKDIV_RESETVALUE;
  CMU->HFPERCLKEN0  = _CMU_HFPERCLKEN0_RESETVALUE;
  CMU->LFCLKSEL     = _CMU_LFCLKSEL_RESETVALUE;
  CMU->LFACLKEN0    = _CMU_LFACLKEN0_RESETVALUE;
  
  /* Set new vector table pointer 设置新的向量表指针*/
  SCB->VTOR = (uint32_t)BOOTLOADER_SIZE;//其大小为38K
  
  /*
  SCB是系统控制块，在Generic user guide 中有说明。它包含多个寄存器，这里是向量表寄存器。
  向量表包含栈指针的复位值，异常向量（包含cpu的所有异常向量）。
  复位后该向量表的地址固定在地址0上，但是在特权模式下可以对VTOR写，把向量表的起始地址重新定位到其他位置上。这里的范围是
  0x00000080到0x3fffff80区间。
  */

  /* Read new SP and PC from vector table */
  sp = *((uint32_t *)BOOTLOADER_SIZE    );
  pc = *((uint32_t *)BOOTLOADER_SIZE + 1);

  BOOT_jump(sp, pc);//这里设置新的堆栈指针和程序计数器
}
