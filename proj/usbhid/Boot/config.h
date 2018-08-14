/**************************************************************************//**
 * @file
 * @brief Bootloader Configuration.
 *    This file defines how the bootloader is set up.
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
#ifndef CONFIG_H
#define CONFIG_H

/************ DEBUG #define's ******************************************/

// 2013.10.25
/** The size of the bootloader flash image��ӳ���ļ���*/
#define BOOTLOADER_SIZE           (38*1024)       /* 38 KB *///˵�������Ҫ��App������0x7000��λ���ϣ���ô����Ӧ������28K,���ó�28KС�ˣ������ó�38K�ͺ��ˡ�

/** The maximum flash size of any EFM32 part */
#define MAX_SIZE_OF_FLASH         (1024*1024)     /* 1 MB */

/** The size of a mass erase block */
#define MASSERASE_BLOCK_SIZE      (512*1024)      /* 512 KB */

#define EFM32_ADDRESS (0x0FE081F0)  //�����ַ�Ӻζ������ο��ο��ֲ�5.6�ڣ�����ͼ�¼��efm32����Ϣ��

#define EFM32_INFOBLOCK   0x0FE00000//����efm32��information memory��user data����ʼ��ַ

#ifdef ewarm//���Ǳ�������ѡ��
#pragma pack(1)
#endif
#ifdef rvmdk
__packed 
#endif
union _CHIP{
  struct _DEVICE{
    uint32_t unique0;
    uint32_t unique1;
    uint16_t memFlash;
    uint16_t menRAM;
    uint16_t Number;
    uint8_t Family;
    uint8_t Rev;
  }Device;
  uint8_t EFM32DeviceInfo[16];
};
extern union _CHIP DevChip;

#ifdef ewarm
#pragma pack()
#endif
//����״̬
#define UPGRADE_IDLE       (0)
#define UPGRADE_RUNNING    (1)
#define UPGRADE_DONE       (2)
#define UPGRADE_READDEVICE (3)
#define UPGRADE_RESTART    (4)
#define UPGRADE_BOOT       (5)

extern uint32_t GulUpgradeFlag;


#ifdef ewarm
#pragma pack(1)
#endif
#ifdef rvmdk
__packed 
#endif
//Star Send [Start Upgrade Cmd:1Byte] + [Start Address 4byte] + [End Address 4byte] +
//[Packet Cnt 4byte] + [FlashCRC 2byte] + [Firmware Version 4byte]
//2013.10.25
//
// UPDATE_TYPE = 1 (Update App)
// UPDATE_TYPE = 2 (Update BOOT)

#define  UPDATE_APP 1
#define  UPDATE_BOOT 2

//��������
union _UPGRADE_PARAM{
  struct _PARAM{
    uint8_t cmd;
    uint32_t StartAddr;
    uint32_t EndAddr;
    uint32_t PacketCnt;
    uint16_t FlashCRC;
    uint32_t FirmwareVersion;
	uint16_t UPDATE_TYPE;
    uint8_t rev[60-19];
  }Param;
  uint8_t data[60];
};
extern union _UPGRADE_PARAM gUpgrade;

#ifdef ewarm
#pragma pack()
#endif

#endif
