#ifndef _MAIN_H
#define _MAIN_H

#include "em_gpio.h"
#include "em_cmu.h"


//bootloader自身的版本号
#define BOOT_VER_M       1
#define BOOT_VER_S       3

// ================================
#define KEY_GPIOPORT  (gpioPortA)
#define KEY_PIN   (2)



//===============================
#define BLE_RX_PORT  (gpioPortA)  // 
#define BLE_RX_PIN   (0)

#define BLE_TX_PORT  (gpioPortF)  // 
#define BLE_TX_PIN   (2)


// UART RX is also as an interrupt input pin
#define BLE_INT_PORT  (gpioPortA)  // 
#define BLE_INT_PIN   (0)


#define BLE_RST_PORT  (gpioPortF)  // 
#define BLE_RST_PIN   (7)

#define BLE_32K_PORT  (gpioPortA)
#define BLE_32K_PIN   (1)


#define BLE_RST_L()   GPIO_PinOutClear(BLE_RST_PORT,BLE_RST_PIN)
#define BLE_RST_H()   GPIO_PinOutSet(BLE_RST_PORT,BLE_RST_PIN)



// ==================================

#if 1  //target board
#define LCD_SPI_USART           (USART1)
#define LCD_SPI_CMUCLOCK  (cmuClock_USART1)
#define LCD_SPI_GPIOPORT  (gpioPortD)
#define LCD_SPI_MOSIPIN   (0)
#define LCD_SPI_CLKPIN    (2)
#define LCD_SPI_LOCATION  (USART_ROUTE_LOCATION_LOC1)

#define LCD_EXTCOM_PORT    (gpioPortB)
#define LCD_EXTCOM_PIN    (11)

#define LCD_DISP_PORT    (gpioPortD)
#define LCD_DISP_PIN    (1)

#define LCD_CS_PORT    (gpioPortD)
#define LCD_CS_PIN    (3)

#define CHARGER_STA_GPIOPORT  (gpioPortF)
#define CHARGER_STA_PIN   (4)

#else  // prototying board

#define CHARGER_STA_GPIOPORT  (gpioPortA)
#define CHARGER_STA_PIN   (4)


#define LCD_SPI_USART           (USART0)
#define LCD_SPI_CMUCLOCK  (cmuClock_USART0)
#define LCD_SPI_GPIOPORT  (gpioPortE)
#define LCD_SPI_MOSIPIN   (10)
#define LCD_SPI_CLKPIN    (12)
#define LCD_SPI_LOCATION  (USART_ROUTE_LOCATION_LOC0)

#define LCD_DISP_PORT    (gpioPortE)
#define LCD_DISP_PIN    (11)

#define LCD_CS_PORT    (gpioPortE)
#define LCD_CS_PIN    (13)

#endif

#define LCD_EXTCOM_PORT    (gpioPortB)
#define LCD_EXTCOM_PIN    (11)

#define LCD_CS_H()    GPIO_PinOutSet(LCD_CS_PORT,LCD_CS_PIN )
#define LCD_CS_L()    GPIO_PinOutClear(LCD_CS_PORT,LCD_CS_PIN )


// ==================================
#define MEMS_SPI           (USART0)
#define MEMS_SPI_CMUCLOCK  (cmuClock_USART0)
#define MEMS_SPI_GPIOPORT  (gpioPortE)
#define MEMS_SPI_MOSIPIN   (10)
#define MEMS_SPI_MISOPIN   (11)
#define MEMS_SPI_CLKPIN    (12)
#define MEMS_SPI_LOCATION  (USART_ROUTE_LOCATION_LOC0)

#define MEMS_CS_PORT    (gpioPortE)
#define MEMS_CS_PIN    (13)

#define MEMS_CS_H()    GPIO_PinOutSet(MEMS_CS_PORT,MEMS_CS_PIN )
#define MEMS_CS_L()    GPIO_PinOutClear(MEMS_CS_PORT,MEMS_CS_PIN )

//==================================

#define AFE_INT_LOC        GPIO_ODD_IRQn

#define AFE_SPI           (USART2)
#define AFE_SPI_CMUCLOCK  (cmuClock_USART2)
#define AFE_SPI_GPIOPORT  (gpioPortC)
#define AFE_SPI_MOSIPIN   (2)
#define AFE_SPI_MISOPIN   (3)
#define AFE_SPI_CLKPIN    (4)
#define AFE_SPI_LOCATION  (USART_ROUTE_LOCATION_LOC0)

#define AFE_CS_PORT    (gpioPortC)
#define AFE_CS_PIN    (5)

#define AFE_CS_H()    GPIO_PinOutSet(AFE_CS_PORT,AFE_CS_PIN )
#define AFE_CS_L()    GPIO_PinOutClear(AFE_CS_PORT,AFE_CS_PIN )

#define AFE_PDNZ_PORT    (gpioPortC)
#define AFE_PDNZ_PIN    (0)

//#define AFE_PDNZ_H()    GPIO_PinOutSet(AFE_PDNZ_PORT,AFE_PDNZ_PIN )
//#define AFE_PDNZ_L()    GPIO_PinOutClear(AFE_PDNZ_PORT,AFE_PDNZ_PIN )

#define AFE_RESETZ_PORT    (gpioPortB)
#define AFE_RESETZ_PIN    (0)

#define AFE_RESETZ_H()    GPIO_PinOutSet(AFE_RESETZ_PORT,AFE_RESETZ_PIN )
#define AFE_RESETZ_L()    GPIO_PinOutClear(AFE_RESETZ_PORT,AFE_RESETZ_PIN )

#define AFE_ADC_DRDY_PORT    (gpioPortA)
#define AFE_ADC_DRDY_PIN    (5)
  
#define AFE_POWER_CON_PORT    (gpioPortC)
#define AFE_POWER_CON_PIN    (0)

#define BOTTOM_POWER_ON()    GPIO_PinOutSet(AFE_POWER_CON_PORT,AFE_POWER_CON_PIN )
#define BOTTOM_POWER_OFF()    GPIO_PinOutClear(AFE_POWER_CON_PORT,AFE_POWER_CON_PIN )

// ================================================

#define BACKLIGHT_PORT  (gpioPortC)  // 2013/06/08
#define BACKLIGHT_PIN   (13)

#define BACKLIGHT_ON()   GPIO_PinOutSet(BACKLIGHT_PORT,BACKLIGHT_PIN)
#define BACKLIGHT_OFF()   GPIO_PinOutClear(BACKLIGHT_PORT,BACKLIGHT_PIN)


//==========================================

#define MOTOR_PORT  (gpioPortC)  // 2013/06/08
#define MOTOR_PIN   (13)

#define MOTOR_ON()   GPIO_PinOutSet(MOTOR_PORT,MOTOR_PIN)
#define MOTOR_OFF()   GPIO_PinOutClear(MOTOR_PORT,MOTOR_PIN)

// ================================


#define CON3_3V_PORT  (gpioPortA)
#define CON3_3V_PIN   (8)


#define EXTVCC_ON()   GPIO_PinOutSet(CON3_3V_PORT,CON3_3V_PIN)
#define EXTVCC_OFF()   GPIO_PinOutClear(CON3_3V_PORT,CON3_3V_PIN)

//===================================

#define PWM1_PORT  (gpioPortE)
#define PWM2_PORT  (gpioPortE)


#define PWM1_PIN   (15)
#define PWM2_PIN   (14)


#define PWM1_ON()   GPIO_PinOutSet(PWM1_PORT,PWM1_PIN)
#define PWM2_ON()   GPIO_PinOutSet(PWM2_PORT,PWM2_PIN)


#define PWM1_OFF()   GPIO_PinOutClear(PWM1_PORT,PWM1_PIN)
#define PWM2_OFF()   GPIO_PinOutClear(PWM2_PORT,PWM2_PIN)

// ==============================================

extern bool USART0_Used;

#define MEMS_SEL 1
#define FLASH_SEL 2
#define CC2541_SEL 3

// =============================================
#define CC2541_PORT  (gpioPortD)
#define CC2541CS_PIN   (11)  // 2013/06/08

#define CC2541CS_L()   GPIO_PinOutClear(CC2541_PORT,CC2541CS_PIN)
#define CC2541CS_H()   GPIO_PinOutSet(CC2541_PORT,CC2541CS_PIN)

#define CC2541RST_PORT  (gpioPortF)  // 2013/06/08
#define CC2541RST_PIN   (7)

#define CC2541RDY_PORT  (gpioPortD)  // 2013/06/08
#define CC2541RDY_PIN   (10)

#define CC2541INT_PORT  (gpioPortD)  // 2013/06/08
#define CC2541INT_PIN   (12)

#define CC254132K_PORT  (gpioPortA)
#define CC254132K_PIN   (1)


#define CC2541RST_L()   GPIO_PinOutClear(CC2541RST_PORT,CC2541RST_PIN)
#define CC2541RST_H()   GPIO_PinOutSet(CC2541RST_PORT,CC2541RST_PIN)


//==========================

#define FLASH_SPI           (USART0)
#define FLASH_SPI_CMUCLOCK  (cmuClock_USART0)
#define FLASH_SPI_GPIOPORT  (gpioPortE)
#define FLASH_SPI_MOSIPIN   (10)
#define FLASH_SPI_MISOPIN   (11)
#define FLASH_SPI_CLKPIN    (12)
#define FLASH_SPI_LOCATION  (0)

#define FLASH_PW_PORT  (gpioPortD)
#define FLASH_PW_PIN    (5)


#define FLASH_SPI_CS_GPIOPORT  (gpioPortE)
#define FLASH_SPI_CSPIN     (15)

//这里实现上电和关掉
#define FLASH_PW_ON()     GPIO_PinOutSet(FLASH_PW_PORT,FLASH_PW_PIN)
#define FLASH_PW_OFF()    GPIO_PinOutClear(FLASH_PW_PORT,FLASH_PW_PIN)


#define  FLASH_CS_L()     GPIO_PinOutClear(FLASH_SPI_CS_GPIOPORT,FLASH_SPI_CSPIN)
#define  FLASH_CS_H()     GPIO_PinOutSet(FLASH_SPI_CS_GPIOPORT,FLASH_SPI_CSPIN)


#define UART0_SPI_GPIOPORT  (gpioPortE)
#define UART0_SPI_MOSIPIN   (10)
#define UART0_SPI_MISOPIN   (11)
#define UART0_SPI_CLKPIN    (12)



//==========中断优先级=============

#define BURTC_IRQn_LEVEL  255
#define USB_INT_LEVEL   254
#define LEUART0_INT_LEVEL 191
#define GPIO_EVEN_INT_LEVEL  191
#define GPIO_ODD_INT_LEVEL  191
#define LESENSE_INT_LEVEL  191
#define TIMER2_IRQn_LEVEL  191
#define LETIMER0_IRQn_LEVEL  191
#define DMA_IRQn_LEVEL 191
#define AFE_INT_LEVEL   191
#define  I2C0_IRQn_Level  190
#define  I2C1_IRQn_Level  190

//延时定时器的定义
#define TIMER_DEALY           (TIMER2)
#define cmuClock_TIMER_DEALY  (cmuClock_TIMER2)

#define TIMER_DEALY_IF     (TIMER_IF_OF)
#define TIMER_DEALY_IRQn       TIMER2_IRQn

//系统时钟
#define SYSCLOCK           14   //该数据表示系统时钟14M 
// *****************************************************************************/

/*** Typedef's and defines. ***/

/* Define USB endpoint addresses */
#define EP_DATA_OUT       0x01  /* Endpoint for USB data reception.       */
#define EP_DATA_IN        0x81  /* Endpoint for USB data transmission.    */
#define EP_NOTIFY         0x82  /* The notification endpoint (not used).  */

#define BULK_EP_SIZE     USB_MAX_EP_SIZE  /* This is the max. ep size.    */
#define USB_RX_BUF_SIZ   BULK_EP_SIZE /* Packet size when receiving on USB*/
#define USB_TX_BUF_SIZ   127    /* Packet size when transmitting on USB.  */

/* Calculate a timeout in ms corresponding to 5 char times on current     */
/* baudrate. Minimum timeout is set to 10 ms.                             */
#define RX_TIMEOUT    EFM32_MAX(10U, 50000 / (cdcLineCoding.dwDTERate))


void USARTx_CS_SEL(unsigned char sel);
void OPEN_ALL_DEVs(void);
extern void SysCtlDelay(unsigned long ulCount);
void CLOSE_ALL_DEVs(void);



extern unsigned int sysclk;
#endif

