EFM32 LG/GG USB HID Bootloader
����Ӳ�������裺
1��EFM32 LG STK3500/GG STK3600
2����USB HID Bootloaderѹ��������λ������
\usb-hid-bootloader\boards\EFM32GG990-STK\examples\usbhid\iar·���µ�IAR workspace�򿪣�IAR6.50��������
3���ȱ���bootloader���̣�����bin�ļ���������ArrayConvert.exe����������ɵ�bin�ļ�ת��Ϊbootld.h�ļ�����������ӵ�Bootld�����У������ԭ�е�bootld.h�ļ�����
4�������������bin�ļ����ص�STK�������С�
5��ͨ��USB Device�߽�STK����PC�ˣ���оƬ�ϵ�ǰ�����Ȱ���PB0���������ϵ磻
6��PC��ʶ��USB HID Carder�豸��
7��������λ��������е�USBHIDBootloader.exe�����Read Device����ȡоƬ����Ϣ��
8�����load fileѡ����Ҫ������App bin�ļ���App Flashƫ��0x7000����Bootldռ��ǰ28K Flash�ռ䣩�����Upgrade�������������衣������ɣ���Ϣ��ʾ������ʾ�����Ϣ��

ps:��USB-TTL�߽ӵ�PD0-TX��PD1-RX��ͨ�����ڵ�������115200-8-1-N�����������Ϣ��



=============================
1. ��ROM������ԣ� ���� ��һ�� //SCB->VTOR = 0x20000000;