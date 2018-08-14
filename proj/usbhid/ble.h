#ifndef _ble_H
#define _ble_H


#define BLE_ON  1
#define BLE_OFF 0

#define BLE_STATE_IDLE              0x00//����
#define BLE_STATE_ADVERTISING       0x01//���ǹ㲥ģʽ��
#define BLE_STATE_CONNECTED         0x02//����



#define BLE_ADVEN_1S  10
#define BLE_ADVEN_2S  20


#define UART_DATA_START			0x3c//'<'
#define UART_DATA_STOP			0x3e//'>'
//�������ݵ����ݣ�����ʼ�����ȡ��������͡��������ݡ�
#define UART_ID_START			0x00
#define UART_ID_LEN			    0x01
#define UART_ID_CMD			    0x02
#define UART_ID_DATA			0x03

//����������Է��͵����������ҪINFOR����鿴�����Ϣ
#define UART_CMD_2HOST  		0x00 //
#define UART_CMD_INFOR			0x01 //������Ϣ����
#define UART_CMD_UPDATA                 0x02//����������������
#define UART_CMD_UPGRADE_DATA           0x03//����Э�������3��ʾ�˰���Ҫ����������
#define UART_CMD_DATA			0x04
#define UART_CMD_ACK                    0x05
#define UART_CMD_TEST                   0x06




#define UART_DATA_COM_LEN		(20+1)
#define UART_DATA_IRTEMP		0x01
#define UART_DATA_ACCEL			0x02
#define UART_DATA_HEARTRATE		0x03
#define UART_DATA_BATT			0x04
#define UART_DATA_GYRO			0x05

#define UART_TYPE_ADVER			0x00//���ǹ㲥����
#define UART_TYPE_DEV			0x01//����������Ϣ��
#define UART_TYPE_INFOR			0x02//��������״̬��Ϣ��
#define UART_TYPE_COM			0x03//��ѯ���е�����




#define UART_LEN_FOU			0x04

#define UART_LEN_DEV			(UART_LEN_FOU+11)
#define Dev_FW_V1				0x00
#define Dev_FW_V2				0x0e

#define BOOTLOAD				0x00
#define BLE_APP					0x01

#define UART_LEN_STARTED		 (UART_LEN_FOU+2)
#define UART_INFOR_IDLE          0x00
#define UART_INFOR_ADVERTISING   0x01
#define UART_INFOR_CONNECTED     0x02


//ʹ�ܻ�ֹͣ�㲥
#define ADVER_EN            0x01
#define ADVER_DIS           0x00

#define crcCheck_Page         123

#define STATE_CMD_INFOR		0x00
#define STATE_COM_DATA		0x01


#define BLE_CH1 1
#define BLE_CH2 2
#define BLE_CH3 3
#define BLE_CH4 4
#define BLE_CH5 5


// BLE HOST Command

#define getRealData  0x01
#define FindMe       0x02
#define ClockSynch   0x03
#define AlertSynch   0x04
#define UserProfileSynch   0x05
#define PhoneComing  0x06	// ����
#define notifyFeature	0x10	// ���š��绰��֪ͨ������sms/calling֪ͨ��vibra/ring

//======================
#define FW_Update_COMM 0x14
#define FW_CRC_GET 0x15
#define FW_Update_DATA 0x16


//======================

#define ResetDevice  0xf0
#define RunOutBatQuick  0xf1
//���������ʱ�Ķ���
#define LeUartUsingDelayTimer      (1 << 3)


union _BLE_CHIP{
  struct _BLE_DEVICE{
  	uint8_t  MCUTYPE;  //  0x41//�Ҷ�����0x8D
  	uint8_t  MEMCAP;   // 0x08 = 256K//�Ҷ�����0x4c
  	uint8_t  FW_VER1; // 1
    uint8_t  FW_VER2; // 0//�Ҷ�����2
    uint32_t unique0;  // MAC address , the first 4bytes  //�Ҷ�����AF2B230D
    uint16_t unique1;  // the second 2 bytes , 6 bytes in total //�Ҷ�����0x9059          
	uint8_t  WORKSTA; // 0= ble bootload ,1 = ble app ////�Ҷ�����0x01
	uint8_t  Rev;////�Ҷ�����0x00
  }BLE_Device;
  uint8_t BLE_DeviceInfo[12];
};

extern uint8_t bleInfo[12];
extern uint8_t bleStFlg;
extern volatile bool  BLE_ONLINE;
extern uint8_t BLE_STATE;
extern union _BLE_CHIP BLE_DevChip;
extern char CopyRxBuff[64];//�������ж��л��õ�����������ݡ�

#define BLE_UART_Closing 0
#define BLE_UART_Opening 1

extern volatile bool BleLeUartSta;
extern volatile bool BLE_Responsed;
extern volatile bool crccheck;

void BLE_INIT(void);
void LeuartConfig (void);
void BLE_Update_Start(void);
void BLE_Update_End(uint16_t checksum);
void WriteCC254xFlash(uint8_t *p);
void BLE_ADVEN_CON(uint8_t onoff, uint8_t inteval);
void SendRealDataOverBLE();
void SendXYZ2Dongle(void);
void SendPPGECG(void);

void ParseBleUartPak(void);
void BLE_Close(void);
void BLE_Open(void);
void Send_1HZ_PacketOverBLE(void);


void getBleDeviceInfo(void);
void AppToBoot(void);
void  BLE_Update_End(uint16_t checksum);
void LEUARTSentByDma(uint8_t comm_type,uint8_t *p ,uint8_t len);

void MyLEUARTSentByDma(uint8_t comm_type,uint8_t *p ,uint8_t len);
void MyBLE_Update_Start(void);

#endif


