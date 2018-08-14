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

//����LED
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
  LETIMER_CompareSet(LETIMER0, 0, 500);//��ʱ
  
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


//�͹��Ķ�ʱ���ж�
void LETIMER0_IRQHandler(void)
{
    LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP0);//����Ƚ���0���жϱ�־
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
 
  
  
  //�����Ƕ�����DBG�ģ�����ڹ�������ģʽ�ж����ˡ�
  #ifndef  DBG
  /* Set new vector table pointer */
  SCB->VTOR = 0x20000000;//ȷ���жϵ���ڵ�ַ��ʼ����RAM����ʼ��ַ
  #endif
  
  //����Ҫ�ȹص����Ź�����Ϊboot���п��˿��Ź���������һ������������Ͳ���ι�����ᵼ�»������ϸ�λ��
 // WDOG_Enable(false);//�ص����Ź�
  LETIMER_setup();
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);


////�����inforblock���в���  �����ɹ���
  
//  MSC_Init();
//  
//  MSC_ErasePage((uint32_t *)EFM32_INFOBLOCK); //��������ҳ��
//  
//  MSC_WriteWord((uint32_t *)EFM32_INFOBLOCK, buf, 4);
//   
//  for(i = 0; i < 8;i++)
//  inforData[i] = *(unsigned char*)(EFM32_INFOBLOCK+i);//��ȡ�����������
// 
//  for(i = 0; i < 8;i++)
//  inforData[i] = *(unsigned char*)(EFM32_INFOBLOCK+i);//��ȡ�����������


///*��һ�ζ����Ľ����0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,0x80,һֱ��0x87*/  
//  if(inforData[0] == 0x45)
//  {
//  while(1);
//  }
//  

  
//Ƭ��flash�Ĳ��� ��������˵�����Կ�sectorд�Ͷ���
//  unsigned char flashBuf[16] = {1,2,3,4,5,6,7,8,9,10};
//  unsigned char readBuf[16] = {0};
//  M25Pxx_INIT(); 
//  
//  FLASH_POWER_BACK();
//  
//  GetFlashCapacity(0x37);//����ܹؼ�
////  FlashSectorErase( 2047);
//////  FlashSectorErase( 1 );
////  FlashProgram( 0x7ff000, flashBuf, 10 );
////    
//  FlashRead(0x3000, readBuf, 16);
// 
  
//��Ƭ��256k flash�Ļ���������������  ���������������ʵ�ֶ�flash�Ĳ���,����д
//    FLASH_CalcPageSize();//�Ȼ�ȡƬ��flash����Ϣ
//    MSC_Init();
//    MSC_ErasePage((uint32_t *)0x3F800);
////    FlashErase(0x3F800, flashSize);//������254-256k������
//    for(i = 0; i < 16;i++)
//    readBuf[i] = *(uint32_t *)(0x3FC00+i);//������ֱ�ӵ�ַ����
//    MSC_WriteWord((uint32_t *)(0x3FFFC), inforData , 4);//��д��ȥ��ÿ��д�����ݸ��������ܱ�4����
//    for(i = 0; i < 4; i++)
//    readBuf[i] = *(uint32_t *)(0x3FFFC+i);
//    
//    readBuf[i+1] = *(uint32_t *)(0x3E800 +i+1);
//    
      
    
    
    
    
    
    

  for (i = 0; i < 16; i++) {
    DevChip.EFM32DeviceInfo[i] = *((unsigned char *)(EFM32_ADDRESS + i));//��EFM32_ADDRESS��ַ���16���ֽ��ó�����
  }
  FlashCRC = CRC_calc((uint8_t *)BOOTLOADER_SIZE, (uint8_t *)(DevChip.Device.memFlash*1024 - 8));//����Ϊʲô��8��Ӧ����4����Ϊ���4���ֽ��Ǵ洢���
 //��flash�д�BOOTLOADER��ַ����ʼ����ʣ�µ����ݽ���ȫ��CRCУ�顣
 //����ʱ�����������0x900b
 
  /*
   * Upgrade Mode Key = Low(PA.2-  )
   */
  CMU_ClockEnable(cmuClock_GPIO, true);//���Ǵ򿪰���
  GPIO_PinModeSet(gpioPortA, 2, gpioModeInputPull, 1); //  20131225
  if ((GPIO_PortInGet(gpioPortA) & (1 << 2)) == 0)
  {
    KeyMode = true;
  }
  else 
  {
    KeyMode = false;//û�а���
  }

  //====for debugging ===

   //GPIO_PinModeSet(BACKLIGHT_PORT,BACKLIGHT_PIN,gpioModePushPull, 0);
   //BACKLIGHT_ON();
  //=====================
//  xx = *((uint32_t *)(DevChip.Device.memFlash*1024 - 4));
//  if(0x900b == xx)//������������õ��ĺ�4���ֽڵ�CRC��0x3c8a���������Ĳ�һ���ˡ�
  if ((FlashCRC == *((uint32_t *)(DevChip.Device.memFlash*1024 - 4)))//flash�����4���ֽڴ洢CRC�����￴���洢��crc��ǰ������crc�Ƿ�һ��  
      &&(KeyMode == false)) //���жϰ����Ƿ���,�����Ƿ�˵������ʹ���µ�bootloader�����ǿ���ʱһֱ��������ôҲ������boot.
  	{ 
     ret = BOOT_checkFirmwareIsValid();
     if (ret == true) 
	 	{
         BOOT_boot();//�ú�����boot.c�ļ��С�
        }
     }

 

  //=======  MEM LCD =================
  initDMA4LCD();
  
  MEMLCD_Init();
  
  
  FB_clearBuffer();
	 
  /* Start LETIMER to toggle EXTCOMIN @ 1 Hz */
  //initLETIMER(); �� ����RAM�����ܲ������� ��ʱ�ܿ�
	
   
  //===================================
	
  DrawBMP(ICON_BOOTLOAD_STATUS,14,20);
  dmaStartFrameTransfer(0, MEMLCD_SIZE_Y - 1);

 
//  USBD_Init(&initstruct);//���������USB��ʼ����Ϊ����USB������׼��
  
  GulUpgradeFlag = UPGRADE_IDLE;//����UPGRADE_IDLE=0
  
  //GulUpgradeFlag��ֵ��USBUpgradeHandler�н��и���
  
  for (;;) 
  	{  
          
     if (UPGRADE_BOOT == GulUpgradeFlag) //UPGRADE=5
	  {   
       Disconnect(2000, 1000);//�����Ĺ��̾�����flash��дAPP���ݵĹ��̡�
       //BOOT_boot();
       /* Write to the Application Interrupt/Reset Command Register to reset
       * the EFM32. See section 9.3.7 in the reference manual. */
       SCB->AIRCR = 0x05FA0004;//���и�λ
           }

       EMU_EnterEM1();  //�ڵ͹���ģʽ�µȴ���
     }
}

/**************************************************************************//**
 * @brief
 *   Called on timer elapsed(��ʧ) event. This function is called at a rate set
 *   by the host driver with the SET_IDLE setup command.������������������ȷ���ú������õ�Ƶ�ʡ�
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
//��Щ�����ǲ�����USB��״̬�����仯����Զ����ã�
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
//��֤USB��������
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
//USBӦ���
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






//�����Ƕ�Ƭ��flash�Ĳ�������Ҫ��main block,������information Block 
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











//USB����������
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
  
  switch (GulUpgradeFlag) //GulUpgradeFlagͨ���ñ�������ʾ�����Ĳ�ͬ״̬
  	{
     case UPGRADE_IDLE:
           {
		   		  
            //if Read Device Information,Handle It
            if ((gHIDOutPacket.Packet.FrameHeader == FRAME_CMD_HEADER)&&
                (gHIDOutPacket.Packet.Data[0] == UPGRADE_READDEVICE)) //�������ӳɹ�����ȡ������Ϣ
             {
             //Feedback Device Information packet
             USBReadDeviceAckPacket(CMD_ACK);
             } 
			else
            //else Start Upgrade Packet, Get the Upgrade param
            if ((gHIDOutPacket.Packet.FrameHeader == FRAME_CMD_HEADER)&&
                (gHIDOutPacket.Packet.Data[0] == UPGRADE_RUNNING))//����������ʼ�ˡ�
              {            
               memcpy(gUpgrade.data, gHIDOutPacket.Packet.Data, 60);//���￪�����ڴ������洢����������
//����ÿ�θ���60���ֽ�   
               
              // Figure out correct flash geometry.
              FLASH_CalcPageSize();//���ǲ���Ƭ�ϵ�flash���������ж�󣬾����Ƕ�ȡ��������Ϣ���֪����

			  UpdateType=gUpgrade.Param.UPDATE_TYPE;//�������͸���
              
              /* Enable DMA interface */
              MSC_Init();//ʹ����Ƭ��flash��д����

			  //Erase Flash
              FlashErase(gUpgrade.Param.StartAddr, flashSize);
            
              GulPacketNum = 0;
              GulFlashOffset = 0;
              sequenceNumber = 0;//�����ʾ���ݹ����ĵڼ������ݰ���
            
              USBAckPacket(CMD_ACK);
                   
            static uint32_t flash,flash_end;
            
			if(UpdateType==UPDATE_APP)//�����������APP����flash_end
				 flash_end=flashSize;
			  else
			  	 flash_end=BOOTLOADER_SIZE;

            for (flash = gUpgrade.Param.StartAddr; flash < flash_end; flash++) //�Ȱ���һ��������������
			  {
              if (*((uint8_t *)(flash)) != 0xFF) //�жϴ�������ʼ�ĵ�ַ��flash������λ�ã����flash�����ݣ�����ͣ��������
			  	{
                while(1);//�����Ѿ������˲���������Ŀ϶���0xff
                }
              }         
                  
            GulUpgradeFlag = UPGRADE_RUNNING;//������ͼƬ����

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
    
  case UPGRADE_RUNNING://������ʼ��
    {

	  WDOG_Feed();//������ι��
	   
      if (GulPacketNum < gUpgrade.Param.PacketCnt) {
        //Data Packet
        if (gHIDOutPacket.Packet.FrameHeader == FRAME_DATA_HEADER) {
        //Wait the Upgrade packet
        GulFlashOffset = PACKET_DATA_LEN*GulPacketNum;//�������ݳ���  ����   ���ĸ����õ�������д������
        
        for (i = 0; i < PACKET_DATA_LEN; i++) {//������һ���ֽ�һ���ֽڵİ����ݰᵽ�����ά������
          GucFlashBuffer[sequenceNumber&1][i] = gHIDOutPacket.Packet.Data[i];//�������õ������н���д�롣
        }//����GucFlashBufferΪʲô��[2][60]?����2��ʾ��
        
        //msc_Return_TypeDef MSC_WriteWord(uint32_t *address, void const *data, int numBytes)
         MSC_WriteWord((uint32_t *)(gUpgrade.Param.StartAddr + GulFlashOffset),
                        ((uint32_t *)(&GucFlashBuffer[sequenceNumber&1])),
                        60);//����һ��д��60���ֽ�
//��ά������1ά���������ǵ�ַ�ˣ���ָ��һά����һ�С�      
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
  

        GulPacketNum++;//�������������ͼƬ��ʾ
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
    
  case UPGRADE_DONE://�������
    {
      //DB_PRINTF("Upgrade Data Send Done!\r\n");
      USBAckPacket(CMD_ACK);
      
      //Write Flash OK, Fill the CRC, and Lock Flash
      FlashCRC = CRC_calc((uint8_t *)BOOTLOADER_SIZE, (uint8_t *)(gUpgrade.Param.EndAddr));//����ղ�д��ȥ��CRC
      if(FlashCRC != gUpgrade.Param.FlashCRC) //��������֤�ղ�д���������CRC��ȷ��д��ȷ��
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
             //cal Whole Flash CRC����������flash�е�CRC������д������flash��ĩβ4���ֽڡ�
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

