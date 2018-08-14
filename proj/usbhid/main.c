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

#include "retargetserial.h"

#include "main.h"

#include "config.h"
#include "crc.h"
#include "boot.h"
#include "res.h"
#include "framebufferctrl.h"

//union _CHIP DevChip;
union _UPGRADE_PARAM gUpgrade;
uint32_t GulUpgradeFlag = 0;
uint32_t GulPacketNum  = 0;
uint32_t GulFlashOffset  = 0;

uint32_t flashSize, flashPageSize;

uint8_t sequenceNumber = 0;

uint16_t UpdateType=0;

/**************************************************************************//**
 *
 * This example shows how a HID keyboard can be implemented.
 *
 *****************************************************************************/
#define DB_PRINTF(x) printf(x) 

/*** Typedef's and defines. ***/

#define POLL_TIMER              0
#define DEFAULT_POLL_TIMEOUT    24
#define HEARTBEAT_MASK          0xF

#define INTR_IN_EP_ADDR         0x81
#define INTR_OUT_EP_ADDR        0x01


#define BOOT_FW_VER_M  1
#define BOOT_FW_VER_S  3

#define FW_TYPE_BOOT  0
#define FW_TYPE_APP  1

//定义LED
#define LED_GPIOPORT  (gpioPortC)
#define LED_PIN   (10)

#define LED_TOGGLE()     GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)

/*** Function prototypes. ***/

static int  OutputReportReceived(USB_Status_TypeDef status,
                                 uint32_t xferred,
                                 uint32_t remaining);
static int  InputReportTransmitted(USB_Status_TypeDef status,
                                 uint32_t xferred,
                                 uint32_t remaining);
static int  SetupCmd(const USB_Setup_TypeDef *setup);
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState);


void FLASH_CalcPageSize(void);
void FLASH_massErase( uint32_t eraseCmd );
void FlashErase (uint32_t baseAddress, uint32_t endAddress);

/*** Include device descriptor definitions. ***/

#include "descriptors.h"

/*** Variables ***/
static int      pollTimeout;        /* Key poll rate, unit is ms. */
static uint8_t  idleRate;
static uint32_t  SetIdleBuffer;

/**************************************************************************//**
 * Disconnect USB link with optional delays.
 *****************************************************************************/
static void Disconnect( int predelay, int postdelay )
{
    if ( predelay )
    {
      /* Allow time to do a disconnect in a terminal program. */
      USBTIMER_DelayMs( predelay );
    }

    USBD_Disconnect();

    if ( postdelay )
    {
      /*
       * Stay disconnected long enough to let host OS tear down the
       * USB CDC driver.
       */
      USBTIMER_DelayMs( postdelay );
    }
}

/* Defining the watchdog initialization data */
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
  .perSel     = wdogPeriod_4k,      /* Set the watchdog period to 4097 clock periods (ie ~2 seconds)*/
};


void LETIMER_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  /* The CORELE clock is also necessary for the RTC and all
     low energy peripherals, but since this function
     is called before RTC_setup() the clock enable
     is only included here */
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LETIMER0, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);
  

  
  /* Set initial compare values for COMP0 */
  LETIMER_CompareSet(LETIMER0, 0, 500);//定时
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = true,                  /* Don't start counting when init completed - only with RTC compare match */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = true,                   /* Start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOAPulse,       /* Pulse output on output 0 */
  .ufoa1          = letimerUFOANone,        /* No output on output 1*/
  .repMode        = letimerRepeatOneshot    /* Count while REP != 0 */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
}


//低功耗定时器中断
void LETIMER0_IRQHandler(void)
{
    LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP0);//清除比较器0的中断标志
    LED_TOGGLE();
}

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
unsigned char inforData[16] = {0x30,0x31,0x32,0x33,0x34,0x35,0x52,0x58};
unsigned char *buf = "EWM0";

unsigned char readBuf[16] = {0};

int main(void)
{
  uint8_t i, ret, KeyMode = false;
  uint32_t FlashCRC;
  uint32_t data = 0;
 
  
  
  //程序是定义了DBG的，这个在工程设置模式中定义了。
  #ifndef  DBG
  /* Set new vector table pointer */
  SCB->VTOR = 0x20000000;//确保中断的入口地址在始终在RAM的起始地址
  #endif
  
  //这里要先关掉看门狗，因为boot中有开了看门狗，当程序一跳到这里来后就不能喂狗，会导致机器不断复位。
 // WDOG_Enable(false);//关掉看门狗
  LETIMER_setup();
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);


////下面对inforblock进行操作  操作成功。
  
//  MSC_Init();
//  
//  MSC_ErasePage((uint32_t *)EFM32_INFOBLOCK); //擦除整个页。
//  
//  MSC_WriteWord((uint32_t *)EFM32_INFOBLOCK, buf, 4);
//   
//  for(i = 0; i < 8;i++)
//  inforData[i] = *(unsigned char*)(EFM32_INFOBLOCK+i);//获取这里面的数据
// 
//  for(i = 0; i < 8;i++)
//  inforData[i] = *(unsigned char*)(EFM32_INFOBLOCK+i);//获取这里面的数据


///*第一次读到的结果是0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,0x80,一直到0x87*/  
//  if(inforData[0] == 0x45)
//  {
//  while(1);
//  }
//  

  
//片外flash的操作 经过测试说明可以跨sector写和读。
//  unsigned char flashBuf[16] = {1,2,3,4,5,6,7,8,9,10};
//  unsigned char readBuf[16] = {0};
//  M25Pxx_INIT(); 
//  
//  FLASH_POWER_BACK();
//  
//  GetFlashCapacity(0x37);//这个很关键
////  FlashSectorErase( 2047);
//////  FlashSectorErase( 1 );
////  FlashProgram( 0x7ff000, flashBuf, 10 );
////    
//  FlashRead(0x3000, readBuf, 16);
// 
  
//对片内256k flash的基本操作函数测试  经过测试这个可以实现对flash的擦除,读、写
//    FLASH_CalcPageSize();//先获取片内flash的信息
//    MSC_Init();
//    MSC_ErasePage((uint32_t *)0x3F800);
////    FlashErase(0x3F800, flashSize);//擦除第254-256k的区域
//    for(i = 0; i < 16;i++)
//    readBuf[i] = *(uint32_t *)(0x3FC00+i);//这里是直接地址访问
//    MSC_WriteWord((uint32_t *)(0x3FFFC), inforData , 4);//先写进去，每次写的数据个数必须能被4整除
//    for(i = 0; i < 4; i++)
//    readBuf[i] = *(uint32_t *)(0x3FFFC+i);
//    
//    readBuf[i+1] = *(uint32_t *)(0x3E800 +i+1);
//    
      
    
    
    
    
    
    

  for (i = 0; i < 16; i++) {
    DevChip.EFM32DeviceInfo[i] = *((unsigned char *)(EFM32_ADDRESS + i));//从EFM32_ADDRESS地址后的16个字节拿出来。
  }
  FlashCRC = CRC_calc((uint8_t *)BOOTLOADER_SIZE, (uint8_t *)(DevChip.Device.memFlash*1024 - 8));//这里为什么减8，应该是4，因为最后4个字节是存储这段
 //把flash中从BOOTLOADER地址处开始，把剩下的内容进行全部CRC校验。
 //测试时计算的数据是0x900b
 
  /*
   * Upgrade Mode Key = Low(PA.2-  )
   */
  CMU_ClockEnable(cmuClock_GPIO, true);//这是打开按键
  GPIO_PinModeSet(gpioPortA, 2, gpioModeInputPull, 1); //  20131225
  if ((GPIO_PortInGet(gpioPortA) & (1 << 2)) == 0)
  {
    KeyMode = true;
  }
  else 
  {
    KeyMode = false;//没有按键
  }

  //====for debugging ===

   //GPIO_PinModeSet(BACKLIGHT_PORT,BACKLIGHT_PIN,gpioModePushPull, 0);
   //BACKLIGHT_ON();
  //=====================
//  xx = *((uint32_t *)(DevChip.Device.memFlash*1024 - 4));
//  if(0x900b == xx)//经过测试这里得到的后4个字节的CRC是0x3c8a与上面计算的不一样了。
  if ((FlashCRC == *((uint32_t *)(DevChip.Device.memFlash*1024 - 4)))//flash的最后4个字节存储CRC，这里看看存储的crc和前面计算的crc是否一样  
      &&(KeyMode == false)) //在判断按键是否按下,这里是否说明，即使有新的bootloader，但是开机时一直按键，那么也不进行boot.
  	{ 
     ret = BOOT_checkFirmwareIsValid();
     if (ret == true) 
	 	{
         BOOT_boot();//该函数在boot.c文件中。
        }
     }

 

  //=======  MEM LCD =================
  initDMA4LCD();
  
  MEMLCD_Init();
  
  
  FB_clearBuffer();
	 
  /* Start LETIMER to toggle EXTCOMIN @ 1 Hz */
  //initLETIMER(); 打开 会在RAM里面跑不起来， 暂时避开
	
   
  //===================================
	
  DrawBMP(ICON_BOOTLOAD_STATUS,14,20);
  dmaStartFrameTransfer(0, MEMLCD_SIZE_Y - 1);

 
//  USBD_Init(&initstruct);//这里进行了USB初始化，为后面USB升级做准备
  
  GulUpgradeFlag = UPGRADE_IDLE;//这里UPGRADE_IDLE=0
  
  //GulUpgradeFlag该值在USBUpgradeHandler中进行更新
  
  for (;;) 
  	{  
          
     if (UPGRADE_BOOT == GulUpgradeFlag) //UPGRADE=5
	  {   
       Disconnect(2000, 1000);//升级的过程就是往flash中写APP数据的过程。
       //BOOT_boot();
       /* Write to the Application Interrupt/Reset Command Register to reset
       * the EFM32. See section 9.3.7 in the reference manual. */
       SCB->AIRCR = 0x05FA0004;//进行复位
           }

       EMU_EnterEM1();  //在低功耗模式下等待。
     }
}

/**************************************************************************//**
 * @brief
 *   Called on timer elapsed(流失) event. This function is called at a rate set
 *   by the host driver with the SET_IDLE setup command.根据主机设置命令来确定该函数调用的频率。
 *****************************************************************************/
static void PollTimeout(void)
{ 
  if (pollTimeout != 0)     /* Send report with current key state */
  {
      //USBD_Read(INTR_OUT_EP_ADDR, (void*) HIDOutReport, 64, OutputReportReceived);
      //USBD_Write(INTR_IN_EP_ADDR, (void*) HIDInReport, sizeof(HIDReport_TypeDef), InputReportTransmitted);
  }
  else        /* pollTimeout == 0, only send report on key status change */
  {
      //USBD_Read(INTR_OUT_EP_ADDR, (void*) HIDOutReport, 64, OutputReportReceived);
      //USBD_Write(INTR_IN_EP_ADDR, (void*) HIDInReport, sizeof(HIDReport_TypeDef), InputReportTransmitted);
  }
  
  /* Restart HID poll timer */
  if (pollTimeout)
  {
    USBTIMER_Start(POLL_TIMER, pollTimeout, PollTimeout);
  }
  else
  {
    USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
  }
}

/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts HID operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
//这些函数是不是在USB的状态发生变化后会自动调用？
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState)
{
  if (newState == USBD_STATE_CONFIGURED)
  {
    /* We have been configured, start HID functionality ! */
    if (oldState != USBD_STATE_SUSPENDED)   /* Resume ?   */
    {
      idleRate    = DEFAULT_POLL_TIMEOUT / 4;
      pollTimeout = DEFAULT_POLL_TIMEOUT;
    }
    USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
  }

  else if ((oldState == USBD_STATE_CONFIGURED) &&
           (newState != USBD_STATE_SUSPENDED))
  {
    /* We have been de-configured, stop HID functionality */
    USBTIMER_Stop(POLL_TIMER);
  }

  else if (newState == USBD_STATE_SUSPENDED)
  {
    /* We have been suspended, stop HID functionality */
    /* Reduce current consumption to below 2.5 mA.    */
    USBTIMER_Stop(POLL_TIMER);
  }

  //putchar('\n');
  //printf(USBD_GetUsbStateName(oldState));
  //printf(" -> ");
  //printf(USBD_GetUsbStateName(newState));
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements HID class specific commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/

static int SetupCmd(const USB_Setup_TypeDef *setup)
{
  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ((setup->Type == USB_SETUP_TYPE_STANDARD) &&
      (setup->Direction == USB_SETUP_DIR_IN) &&
      (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
  {
    /* A HID device must extend the standard GET_DESCRIPTOR command   */
    /* with support for HID descriptors.                              */
    switch (setup->bRequest)
    {
    case GET_DESCRIPTOR:
      /********************/
      if ((setup->wValue >> 8) == USB_HID_REPORT_DESCRIPTOR)
      {
        USBD_Write(0, (void*) ReportDescriptor,
                   EFM32_MIN(sizeof(ReportDescriptor), setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      else if ((setup->wValue >> 8) == USB_HID_DESCRIPTOR)
      {
        USBD_Write(0, (void*) &configDesc[ USB_CONFIG_DESCSIZE +
                                           USB_INTERFACE_DESCSIZE ],
                   EFM32_MIN(USB_HID_DESCSIZE, setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  else if ((setup->Type == USB_SETUP_TYPE_CLASS) &&
           (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
  {
    /* Implement the necessary HID class specific commands.           */
    switch (setup->bRequest)
    {
    case USB_HID_SET_REPORT:
      /********************/
      if (((setup->wValue >> 8) == 2) &&            /* Output report */
          ((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 0x40) &&               /* Report length */
          (setup->Direction == USB_SETUP_DIR_OUT))
      {
        /* Receive OUT Report */
        USBD_Read(0, (void*) (gHIDOutPacket.Buffer), 64, OutputReportReceived);
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_REPORT:
      /********************/
      if (((setup->wValue >> 8) == 1) &&            /* Input report  */
          ((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 0x40) &&               /* Report length */
          (setup->Direction == USB_SETUP_DIR_IN))
      {
          /* Send IN report */
          USBD_Write(0, (void*)(gHIDInPacket.Buffer), 64, InputReportTransmitted);
          retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_SET_IDLE:
      /********************/
      if (((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 0) &&
          (setup->Direction != USB_SETUP_DIR_IN))
      {
        idleRate    = setup->wValue >> 8;
        pollTimeout = 4 * idleRate;
        if (pollTimeout > DEFAULT_POLL_TIMEOUT)
        {
          pollTimeout = DEFAULT_POLL_TIMEOUT;
        }
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_IDLE:
      /********************/
      if ((setup->wValue == 0) &&                   /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 1) &&
          (setup->Direction == USB_SETUP_DIR_IN))
      {
        *((uint8_t *)SetIdleBuffer) = idleRate;
        USBD_Write(0, (void*) SetIdleBuffer, 1, NULL);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  return retVal;
}
//验证USB包的数量
uint8_t VerifyUSBPacketChecksum(void)
{
  uint8_t ret = true;
  uint16_t packetCRC;
  
  if ((gHIDOutPacket.Packet.FrameHeader != FRAME_CMD_HEADER)&&
    (gHIDOutPacket.Packet.FrameHeader != FRAME_DATA_HEADER)) {
    ret = false;
  }
  if(gHIDOutPacket.Packet.FrameTail != FRAME_TAIL) {
    ret = false;
  }
  packetCRC = CRC_calc(gHIDOutPacket.Packet.Data, (&gHIDOutPacket.Packet.Data[PACKET_DATA_LEN-1]));
  if (packetCRC != gHIDOutPacket.Packet.CRC ) {
    ret = false;
  }
  return ret;
}
//USB应答包
void USBAckPacket(uint8_t ack)
{
  uint16_t packetCRC;
  
  memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));
  
  gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
  gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;
  gHIDInPacket.Packet.Data[0] = ack;
  
  packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN-1]));
  gHIDInPacket.Packet.CRC = packetCRC;
}

void USBReadDeviceAckPacket(uint8_t ack)
{
  uint16_t packetCRC;
  
  memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));
  
  gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
  gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;
  
  gHIDInPacket.Packet.Data[0] = ack;
  
  memcpy((&gHIDInPacket.Packet.Data[1]), DevChip.EFM32DeviceInfo, sizeof(DevChip.EFM32DeviceInfo));

  // 20130828
  gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo)+1] = FW_TYPE_BOOT;
  gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo)+2] = BOOT_FW_VER_M;
  gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo)+3] = BOOT_FW_VER_S;
   
  packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN-1]));
  gHIDInPacket.Packet.CRC = packetCRC;  
}






//下面是对片上flash的操作，主要是main block,不包括information Block 
/***************************************************************************//**
*
* @brief
*   Calculate flash page size
*******************************************************************************/
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

/**************************************************************************//**
 *
 * Mass erase a flash block.
 *
 * @param eraseCmd The mass erase command to write to MSC WRITECMD register.
 *
 * This function will mass erase a 512K block on a Giant device.
 * This function will not return until the block has been erased.
 *****************************************************************************/
void FLASH_massErase( uint32_t eraseCmd )
{
  /* Enable writing to the MSC */
  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
  
  /* Unlock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_UNLOCK;

  /* Send Mass erase command */
  MSC->WRITECMD = eraseCmd;

  /* Waiting for erase to complete */
  while ((MSC->STATUS & MSC_STATUS_BUSY)){}

  /* Lock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_LOCK;
  
  /* Disable writing to the MSC */
  MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}


void FlashErase (uint32_t baseAddress, uint32_t endAddress)
{
  uint32_t  addr;
  uint8_t   ret;

  /* Erase flash */
  addr = baseAddress;
  
  /* Do first 512K block. */
  /* Check if it is possible to mass erase first block. */
  if ( ( addr == 0 ) && ( endAddress >= MASSERASE_BLOCK_SIZE ) ) {
    FLASH_massErase( MSC_WRITECMD_ERASEMAIN0 );
    addr += MASSERASE_BLOCK_SIZE;
  }
  else
  {
    while ( addr < MASSERASE_BLOCK_SIZE )
    {
      ret = MSC_ErasePage((uint32_t *)addr);//2K or 4K pagesize
      if (ret != mscReturnOk) {
        ret = MSC_ErasePage((uint32_t *)addr);
      }
      addr += flashPageSize;
    }
  }

  #if FLASH_SIZE >= (512 * 1024)

  /* Do second 512K block. */
  if ( flashSize > MASSERASE_BLOCK_SIZE )
  {
    /* Mass erase possible ? */
    if ( ( addr        == MASSERASE_BLOCK_SIZE      ) &&
         ( endAddress  >= 2 * MASSERASE_BLOCK_SIZE  )    )
    {
      FLASH_massErase( MSC_WRITECMD_ERASEMAIN1 );
    }
    else
    {
      while ( addr < 2 * MASSERASE_BLOCK_SIZE )
      {
        MSC_ErasePage((uint32_t *)addr);//2K or 4K pagesize
        addr += flashPageSize;
      }
    }
  	}
  #endif
}











//USB升级处理函数
static void USBUpgradeHandler(void)
{
  uint8_t ret, i;
  uint16_t FlashCRC;
  uint32_t CheckSum;
  
  //Verify USBPacket Checksum
  ret = VerifyUSBPacketChecksum();
  if (ret == false) {
    USBAckPacket(CMD_NACK);
    return;
  }
  
  switch (GulUpgradeFlag) //GulUpgradeFlag通过该变量来表示升级的不同状态
  	{
     case UPGRADE_IDLE:
           {
		   		  
            //if Read Device Information,Handle It
            if ((gHIDOutPacket.Packet.FrameHeader == FRAME_CMD_HEADER)&&
                (gHIDOutPacket.Packet.Data[0] == UPGRADE_READDEVICE)) //这里连接成功，读取器件信息
             {
             //Feedback Device Information packet
             USBReadDeviceAckPacket(CMD_ACK);
             } 
			else
            //else Start Upgrade Packet, Get the Upgrade param
            if ((gHIDOutPacket.Packet.FrameHeader == FRAME_CMD_HEADER)&&
                (gHIDOutPacket.Packet.Data[0] == UPGRADE_RUNNING))//这里升级开始了。
              {            
               memcpy(gUpgrade.data, gHIDOutPacket.Packet.Data, 60);//这里开辟了内存用来存储升级包数据
//这里每次更新60个字节   
               
              // Figure out correct flash geometry.
              FLASH_CalcPageSize();//这是操作片上的flash，计算他有多大，具体是读取其器件信息后就知道了

			  UpdateType=gUpgrade.Param.UPDATE_TYPE;//升级类型更新
              
              /* Enable DMA interface */
              MSC_Init();//使能了片上flash的写操作

			  //Erase Flash
              FlashErase(gUpgrade.Param.StartAddr, flashSize);
            
              GulPacketNum = 0;
              GulFlashOffset = 0;
              sequenceNumber = 0;//这里表示传递过来的第几个数据包。
            
              USBAckPacket(CMD_ACK);
                   
            static uint32_t flash,flash_end;
            
			if(UpdateType==UPDATE_APP)//如果升级的是APP，则flash_end
				 flash_end=flashSize;
			  else
			  	 flash_end=BOOTLOADER_SIZE;

            for (flash = gUpgrade.Param.StartAddr; flash < flash_end; flash++) //先把这一包的数据区擦除
			  {
              if (*((uint8_t *)(flash)) != 0xFF) //判断从升级开始的地址到flash结束的位置，这段flash的内容，这能停在这里吗？
			  	{
                while(1);//上面已经进行了擦除这里检测的肯定是0xff
                }
              }         
                  
            GulUpgradeFlag = UPGRADE_RUNNING;//升级的图片更新

			/* Enabling clock to the interface of the low energy modules (including the Watchdog)*/
            CMU_ClockEnable(cmuClock_CORELE, true);
			/* Initializing watchdog with choosen settings */
			WDOG_Init(&init);

          } 
		else {
          //Other Packet
          USBAckPacket(CMD_NACK);
      }
    }
    break;
    
  case UPGRADE_RUNNING://升级开始。
    {

	  WDOG_Feed();//在这里喂狗
	   
      if (GulPacketNum < gUpgrade.Param.PacketCnt) {
        //Data Packet
        if (gHIDOutPacket.Packet.FrameHeader == FRAME_DATA_HEADER) {
        //Wait the Upgrade packet
        GulFlashOffset = PACKET_DATA_LEN*GulPacketNum;//包的数据长度  乘以   包的个数得到本包该写到哪里
        
        for (i = 0; i < PACKET_DATA_LEN; i++) {//这里是一个字节一个字节的吧数据搬到这个二维数组中
          GucFlashBuffer[sequenceNumber&1][i] = gHIDOutPacket.Packet.Data[i];//把数据拿到缓存中进行写入。
        }//这里GucFlashBuffer为什么是[2][60]?这里2表示？
        
        //msc_Return_TypeDef MSC_WriteWord(uint32_t *address, void const *data, int numBytes)
         MSC_WriteWord((uint32_t *)(gUpgrade.Param.StartAddr + GulFlashOffset),
                        ((uint32_t *)(&GucFlashBuffer[sequenceNumber&1])),
                        60);//这里一次写了60个字节
//二维数组中1维数组名就是地址了，它指向一维的这一列。      
        sequenceNumber++;

		static int draw_count=20,draw_x=0, draw_y=50;

		if(draw_count==20)
		  {
		   draw_count=0;
		   DrawBMP(ICON_DOT3x3,draw_x,draw_y);
           dmaStartFrameTransfer(0, MEMLCD_SIZE_Y - 1);
		   draw_x+=8;
		   if(draw_x>=MEMLCD_SIZE_X)
		   	{draw_x=0;
		     draw_y+=8;
			 }
		 
			}
		draw_count++;
  

        GulPacketNum++;//上面进行了升级图片显示
        USBAckPacket(CMD_ACK);

        }
        //Command Packet
        else if (gHIDOutPacket.Packet.FrameHeader == FRAME_CMD_HEADER) {
          if (gHIDOutPacket.Packet.Data[0] == UPGRADE_RESTART) {
             GulUpgradeFlag = UPGRADE_IDLE;
             return;
          }
        }
        if (GulPacketNum == gUpgrade.Param.PacketCnt) {     
           GulUpgradeFlag = UPGRADE_DONE;
           GulPacketNum = 0;
           GulFlashOffset = 0;  
        }
      } else {
        
        GulUpgradeFlag = UPGRADE_DONE;
        GulPacketNum = 0;
        GulFlashOffset = 0;
      }
      //Fail goto UPGRADE_IDLE
      //Set Timeout
    }
    break;
    
  case UPGRADE_DONE://升级完成
    {
      //DB_PRINTF("Upgrade Data Send Done!\r\n");
      USBAckPacket(CMD_ACK);
      
      //Write Flash OK, Fill the CRC, and Lock Flash
      FlashCRC = CRC_calc((uint8_t *)BOOTLOADER_SIZE, (uint8_t *)(gUpgrade.Param.EndAddr));//计算刚才写进去的CRC
      if(FlashCRC != gUpgrade.Param.FlashCRC) //这里是验证刚才写的这个包的CRC，确保写正确。
	  	{
        //DB_PRINTF("Verify Tx CRC Fail!\r\n");
        GulUpgradeFlag = UPGRADE_IDLE;
        GulPacketNum = 0;
        GulFlashOffset = 0;
        } 
	  else 
	  	{
      	 if(UpdateType==UPDATE_APP)
			{
             //cal Whole Flash CRC，计算整个flash中的CRC，把它写在整个flash的末尾4个字节。
             CheckSum = CRC_calc((uint8_t *)BOOTLOADER_SIZE, (uint8_t *)(DevChip.Device.memFlash*1024 - 8));
             MSC_WriteWord((uint32_t *)(DevChip.Device.memFlash*1024 - 4), (uint32_t *)&CheckSum, 4);
          
			}
		 GulUpgradeFlag = UPGRADE_BOOT;
	  	}
	  MSC_Deinit();
  	}
    break;
  }
  return;
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_SET_REPORT
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int OutputReportReceived(USB_Status_TypeDef status,
                                uint32_t xferred,
                                uint32_t remaining)
{
//  uint8_t *p = gHIDOutPacket.Buffer, i;
  
  (void) remaining;

  /* We have received new data for NumLock, CapsLock and ScrollLock LED's */
  if ((status == USB_STATUS_OK) && (xferred == 64))
  {
    USBUpgradeHandler();

  }
  return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_GET_REPORT
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int InputReportTransmitted(USB_Status_TypeDef status,
                                uint32_t xferred,
                                uint32_t remaining)
{
  
  (void) remaining;

  /* We have received new data for NumLock, CapsLock and ScrollLock LED's */
  if ((status == USB_STATUS_OK) && (xferred == 64))
  {
    memset(gHIDInPacket.Buffer, 0, 64);

  }
  return USB_STATUS_OK;
}

