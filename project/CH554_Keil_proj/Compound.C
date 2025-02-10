/********************************** (C) COPYRIGHT ******************************
* File Name          :Compound_Dev.C												
* Author             : WCH                                                      
* Version            : V1.2                                                     
* Date               : 2017/02/24                                               
* Description        : A demo for USB compound device created by CH554, support 
					   keyboard , and HID-compliant device.                     
********************************************************************************/

#include 	".\Public\CH554.H"
#include 	".\Public\DEBUG.H"
#include 	"compound.h"
#include 	<stdio.h>
#include 	<stdlib.h>
#include 	<string.h>


#define 	THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define		BUFFER_SIZE				64
#define 	DUAL_BUFFER_SIZE		128
#define 	UsbSetupBuf     		((PUSB_SETUP_REQ)Ep0Buffer)
#define		L_WIN 					0X08
#define 	L_ALT 					0X04
#define		L_SHIFT					0X02
#define 	L_CTL					0X01
#define 	R_WIN 					0X80
#define 	R_ALT 					0X40
#define 	R_SHIFT					0X20
#define 	R_CTL					0X10
#define 	SPACE					0X2C
#define		ENTER					0X28

#define MOUSE 0

#pragma  NOAREGS

//UINT8X  	Ep0Buffer[THIS_ENDP0_SIZE]  _at_ 0x0000;  								// Endpoint 0, buffer OUT/OUT��the address must be even.
//UINT8X  	Ep1Buffer[BUFFER_SIZE] 		_at_ 0x000A;  								// Endpoint 1, buffer IN��the address must be even.
//UINT8X 		Ep2Buffer[DUAL_BUFFER_SIZE]	_at_ 0x0050;  								// Endpoint 2, buffer OUT[64]+IN[64]��the address must be even.

UINT8X  Ep0Buffer[64] _at_ 0x0000;                                 //�˵�0 ���ͺͽ��չ��û�������������ż��ַ
UINT8X  Ep1Buffer[64] _at_ 0x0040;                                                  //�˵�1�ϴ�������
UINT8X  Ep2Buffer[128] _at_ 0x0080;                                  //˵�2 ���պͷ���˫������,������ż��ַ
UINT8X  Ep3Buffer[64] _at_ 0x0100; //˵�2 OUT˫������,������ż��ַ
UINT8X  Ep3Buffer2[64] _at_ 0x0140; //˵�2 OUT˫������,������ż��ַ



/**************************** Global variable ********************************/	
PUINT8  	pDescr;                                                                	// USB enumerate complete flag.
USB_SETUP_REQ   					SetupReqBuf;                                   	// A buffer for Setup package.

UINT8  USB_RequestFlag= 0;
PUINT8 pDescr; //USB���ñ�־
UINT8I Endp3Busy = 0;
UINT8I SetupReq, SetupLen, Ready, Count, UsbConfig;

UINT8I Endp2Busy = 0;
UINT8I Endp1Busy;
UINT8I USBD0 = 0;       //����USB�˵���յ�������
UINT8I USBByteCount = 0;       //����USB�˵���յ�������
UINT8I USBBufOutPoint = 0;     //ȡ����ָ��
UINT8I LineCoding[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //��ʼ��������Ϊ57600��1ֹͣλ����У�飬8����λ��
UINT16I USB_STATUS = 0;
UINT8C MyLangDescr[] = {0x04, 0x03, 0x09, 0x04};
unsigned char  code String_1[]={0x12, 0x03,'y',0x00,'u',0x00,'l',0x00,'e',0x00,'i',0x00,'t',0x00,'a',0x00,'o',0x00};           //����������
unsigned char  code String_2[]={0x14, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00};     
unsigned char  code String_3[]={0x1a, 0x03,'4',0x00,'8',0x00,'E',0x00,'A',0x00,'8',0x00,'0',0x00,'6',0x00,'E',0x00,'3',0x00,'1',0x00,'3',0x00,'9',0x00};
unsigned char  code String_4[]={0x1c, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00,' ',0x00,'C',0x00,'D',0x00,'C',0x00};
unsigned char  code String_5[]={0x1c, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00,' ',0x00,'D',0x00,'C',0x00,'I',0x00};
unsigned char  code String_6[]={0x14, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00};


/**************************** Device Descriptor�豸������ *************************************/
UINT8C DevDesc[18] = {
    0x12,               // bLength
    0x01,               // bDescriptorType (Device)
    0x00,0x02,          // bcdUSB 2.00
    0xEF,               // bDeviceClass (Miscellaneous)
    0x02,               // bDeviceSubClass
    0x01,               // bDeviceProtocol
    0x40,               // bMaxPacketSize0 64
    0xC2,0x51,          // idVendor 0xC251
    0xF0,0x01,          // idProduct 0xF001
    0x00,0x01,          // bcdDevice 1.00
    0x01,               // iManufacturer (String Index)
    0x02,               // iProduct (String Index)
    0x03,               // iSerialNumber (String Index)
    0x01                // bNumConfigurations 1
};
/**************************** HID Report Descriptor *********************************/

UINT8C USBD_HID_ReportDescriptor[33] = 															// Report Descriptor, Composite device
{
	0x06, 0x00, 0xff, 	// Usage page Vendor defined
	0x09, 0x01, 		// Usage keyboard
	0xa1, 0x01, 		// Collation Application
	0x15, 0x00, 		// Logical min ( 0H )
	0x26, 0xff, 0x00,	// Logical max ( FFH )
	0x75, 0x08,  		// Report size ( 08H )
	0x95, 0x40, 		// Report count ( 40H )
	0x09, 0x01, 		// Mouse
	0x81, 0x02,  		// Input ( Data, Relative, Wrap )
	0x95, 0x40,  		// Logical min ( 0H )
	0x09, 0x01,	// Logical max ( FFH )
	0x91, 0x02, 		// Report size ( 08H )
	0x95, 0x01, 		// Report count ( 40H )
	0x09, 0x01, 		// Output ( Data, Relative, Wrap )
	0xB1, 0x02,
	0xC0
};


UINT8C CfgDesc[] =
{
//�����������������ӿڣ�
0x09,//�ֽ�������
0x02,//���������ͣ�����������
0x6B,0x00,//��������Ϣ�ܳ�
0x03,//��������֧�ֵĽӿڸ���
0x01,//��SetConfiguration����������������ѡ��������
0x00,//���������õ��ִ�����������ֵ����SetConfiguration����������ѡ�����õĲ���
0x80,//�������ԣ�����Ϊ��
0x32,//���������������λ2ma������Ϊ100ma               					
//�ӿڹ���������
0x08,//�ֽ�������
0x0B,//���������ͣ��ӿڹ���������
0x00,//��һ���ӿ�Ϊ0
0x02,//�ܹ������ӿ�
0x02,//CDC
0x02,//���⴮��
0x01,//Common AT commands
0x04,//USBD_CDC_ACM_CIF_STR_NUM             			
//CDC�ӿ�������
0x09,//�ֽ�������
0x04,//���������ͣ��ӿ�������
0x00,//Number of Interface
0x00,//Alternate setting
0x01,//One endpoint used
0x02,//CDC_COMMUNICATION_INTERFACE_CLASS
0x02,//CDC_ABSTRACT_CONTROL_MODEL
0x00,//no protocol used
0x04,//USBD_CDC_ACM_CIF_STR_NUM
//����Ϊ����������
//����������(ͷ)
0x05,//�ֽ�������
0x24,//���������ͣ�CS_INTERFACE 
0x00,//Header Func Desc
0x10,0x01,//CDC�汾�ţ�Ϊ0x0110�����ֽ����ȣ� 
//����������(û��������ӿ�)
0x05,//�ֽ�������
0x24,//���������ͣ�CS_INTERFACE 
0x01,//Call Management Func Desc
0x01,//device handles call management
0x01,//CDC data IF ID
//CDC��������
0x04,//�ֽ�������
0x24,//���������ͣ�CS_INTERFACE 
0x02,//Abstract Control Management desc
0x02,//֧��Set_Line_Coding��Set_Control_Line_State��Get_Line_Coding�����Serial_State֪ͨ
//CDC��������
0x05,//�ֽ�������
0x24,//���������ͣ�CS_INTERFACE 
0x06,//Union func desc
0x00,//����Ϊǰ����Ϊ0��CDC�ӿ�
0x01,//����Ϊ���������Ϊ1��������ӿ�
//�ж��ϴ��˵�������
0x07,//�ֽ�������
0x05,//���������ͣ��˵�������
0x81,//�˵�ĵ�ַ
0x03,//�˵�����ԣ��жϴ���
0x40,0x00,//�˵�֧�ֵ���������64�ֽ�
0x02,//�˿ڲ�ѯ��֡����� 
//����Ϊ�ӿ�1�����ݽӿڣ�������
//���ݽӿ�������
0x09,//�ֽ�������
0x04,//���������ͣ��ӿ�������
0x01,//�ӿں�
0x00,//�����õ�����ֵ
0x02,//�˽ӿ��õĶ˵�����
0x0A,//�ӿ���������ֵ������ΪCDC������
0x00,//������
0x00,//Э����
0x05,//�����˽ӿڵ��ִ�������������ֵ 
//CDC����˵�
0x07,//�ֽ�������
0x05,//���������ͣ��˵�������
0x02,//�˵��OUT��ַ
0x02,//�˵�����ԣ���������
0x40,0x00,//�˵�֧�ֵ���������64�ֽ�
0x00,//�˿ڵĲ�ѯʱ��
//CDC����˵� 
0x07,//�ֽ�������
0x05,//���������ͣ��˵�������
0x82,//�˵��IN��ַ
0x02,//�˵�����ԣ���������
0x40,0x00,//�˵�֧�ֵ���������64�ֽ�
0x00,//�˿ڵĲ�ѯʱ��
//����Ϊ�ӿ�2�����ݽӿڣ�������
//���ݽӿ�������
0x09,//�ֽ�������
0x04,//���������ͣ��ӿ�������
0x02,//�ӿں�
0x00,//�����õ�����ֵ
0x02,//�˽ӿ��õĶ˵�����
0x03,//�ӿ���������ֵ������ΪHID��
0x00,//������
0x00,//Э����
0x06,//�����˽ӿڵ��ִ�������������ֵ HID Descriptor
//HID�������� 
0x09,//�ֽ�������
0x21,//���������ͣ�HID������
0x00,0x01,//HID��淶������V1.0
0x00,//Ӳ��Ŀ�����
0x01,//����HID����������������
0x22,//��������������
0x21,0x00,//HID��������������
//HID�ж�����˵� 
0x07,//�ֽ�������
0x05,//���������ͣ��˵�������
0x83,//�˵��IN��ַ
0x03,//�˵�����ԣ��жϴ���
0x40,0x00,//�˵�֧�ֵ���������64�ֽ�
0x01,//�˿ڲ�ѯ��֡�����
//HID�ж�����˵� 
0x07,//�ֽ�������
0x05,//���������ͣ��˵�������
0x03,//�˵��OUT��ַ
0x03,//�˵�����ԣ��жϴ���
0x40,0x00,//�˵�֧�ֵ���������64�ֽ�
0x01//�˿ڲ�ѯ��֡�����
};


void Config_Uart1(UINT8 *cfg_uart)
{
    UINT32 uart1_buad = 0;
	UINT8I num=0,lenth=0;
    *((UINT8 *)&uart1_buad) = cfg_uart[3];
    *((UINT8 *)&uart1_buad + 1) = cfg_uart[2];
    *((UINT8 *)&uart1_buad + 2) = cfg_uart[1];
    *((UINT8 *)&uart1_buad + 3) = cfg_uart[0];
	CH554UART0SendByte(0x68);
	CH554UART0SendByte(7);
	CH554UART0SendByte(uart1_buad/1000000+'0');
	CH554UART0SendByte(uart1_buad%1000000/100000+'0');
	CH554UART0SendByte(uart1_buad%100000/10000+'0');
	CH554UART0SendByte(uart1_buad%10000/1000+'0');
	CH554UART0SendByte(uart1_buad%1000/100+'0');
	CH554UART0SendByte(uart1_buad%100/10+'0');
	CH554UART0SendByte(uart1_buad%10+'0');
}

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : Configure USB mode ��USB device init configure.Configure tie Endpoint, compound device, 
				           Endpoint 0 control trans, Endpoint 1/2 interrupt(IN).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_INT_FG = 0xFF;                                     // 清除所有中断标志
    USB_CTRL = 0x00;                                       // 清空USB控制寄存器
    UDEV_CTRL = bUD_PD_DIS;                               // 禁止DP/DM下拉电阻
    
    // 先配置端点
    UEP0_DMA = Ep0Buffer;                                                      
    UEP1_DMA = Ep1Buffer;
    UEP2_DMA = Ep2Buffer;
    UEP3_DMA = Ep3Buffer;

    // 配置端点控制
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
    UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;

    // 配置端点模式
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);
    UEP2_3_MOD = bUEP2_TX_EN | bUEP2_RX_EN | bUEP3_TX_EN | bUEP3_RX_EN;

    USB_DEV_AD = 0x00;
    
    // 配置为全速模式
    UDEV_CTRL &= ~bUD_LOW_SPEED;                          // 选择全速12M模式
    USB_CTRL &= ~bUC_LOW_SPEED;                           // 全速模式

    // 使能USB功能
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;
    UDEV_CTRL |= bUD_PORT_EN;                             // 使能物理端口

    // 使能中断
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}


void DeviceInterrupt(void) interrupt INT_NO_USB using 1 //USB�жϷ������,ʹ�üĴ�����1
{
    UINT8 len,temp,num_s;
    if (UIF_TRANSFER) //USB������ɱ�־
    {
        // 添加调试输出
        printf("USB Transfer INT: %02X\n", USB_INT_ST);
        
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
		case UIS_TOKEN_IN | 1: //endpoint 1# �˵������ϴ�
            UEP1_T_LEN = 0;      //Ԥʹ�÷��ͳ���һ��Ҫ���
            Endp1Busy = 0;
            UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //Ĭ��Ӧ��NAK
            break;

        case UIS_TOKEN_OUT | 2: //endpoint 2# �˵������´�
            if (U_TOG_OK)         // ��ͬ�������ݰ�������
            {
                USBByteCount = USB_RX_LEN;
                USBBufOutPoint = 0;                                             //ȡ����ָ�븴λ
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;       //�յ�һ�����ݾ�NAK�������������꣬���������޸���Ӧ��ʽ
            }
            break;

		case UIS_TOKEN_IN | 2: //endpoint 2# �˵������ϴ�
            UEP2_T_LEN = 0;      //Ԥʹ�÷��ͳ���һ��Ҫ���
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //Ĭ��Ӧ��NAK
			Endp2Busy = 0;
            break;

		case UIS_TOKEN_OUT | 3: //endpoint 2# �˵������´�
            if (U_TOG_OK)         // ��ͬ�������ݰ�������
            {
				//UEP3_T_LEN = 0;
				if(USB_RX_LEN)
				{
					USB_RequestFlag = 1;;
//					UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;
				}
            }
            break;

        case UIS_TOKEN_IN | 3: //endpoint 3# �˵������ϴ�
            Endp3Busy = 0;
            UEP3_T_LEN = 0;      //Ԥʹ�÷��ͳ���һ��Ҫ���
            UEP3_CTRL = UEP3_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //Ĭ��Ӧ��NAK
            break;

        
//        case UIS_TOKEN_OUT | 1: //endpoint 1# �˵������´�
//            if (U_TOG_OK)         // ��ͬ�������ݰ�������
//            {
//                USBByteCount = USB_RX_LEN;
//                USBBufOutPoint = 0;                                             //ȡ����ָ�븴λ
//                UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;       //�յ�һ�����ݾ�NAK�������������꣬���������޸���Ӧ��ʽ
//            }
//            break;

        case UIS_TOKEN_SETUP | 0: //SETUP����
            len = USB_RX_LEN;
            printf("Setup Token, len=%d\n", len);
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // �����ܳ���
                }
                len = 0;           // Ĭ��Ϊ�ɹ������ϴ�0����
                SetupReq = UsbSetupBuf->bRequest;
                switch (UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK)
                {
                case USB_REQ_TYP_STANDARD:
                    switch (SetupReq) //������
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:             //�豸������
                            pDescr = DevDesc; //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(DevDesc);
                            break;
                        case 2:             //����������
                            pDescr = CfgDesc; //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(CfgDesc);
                            break;
                        case 3: // �ַ���������
                            switch (UsbSetupBuf->wValueL)
                            {
                            case 0:
                                pDescr = (PUINT8)(&MyLangDescr[0]);
                                len = sizeof(MyLangDescr);
                                break;
                            case 1:
                                pDescr = (PUINT8)(&String_1[0]);
                                len = sizeof(String_1);
                                break;
                            case 2:
                                pDescr = (PUINT8)(&String_2[0]);
                                len = sizeof(String_2);
                                break;
                            case 3:
                                pDescr = (PUINT8)(&String_3[0]);
                                len = sizeof(String_3);
                                break;
                            case 4:
                                pDescr = (PUINT8)(&String_4[0]);
                                len = sizeof(String_4);
                                break;
                            case 5:
                                pDescr = (PUINT8)(&String_5[0]);
                                len = sizeof(String_5);
                                break;
							case 6:
                                pDescr = (PUINT8)(&String_6[0]);
                                len = sizeof(String_6);
                                break;
                            default:
                                len = 0xFF; // ��֧�ֵ��ַ���������
                                break;
                            }
                            break;
                        case 0x22:                                                    	// HID report descriptor											
							pDescr = USBD_HID_ReportDescriptor;                                  	// Write to buffer
							len = sizeof( USBD_HID_ReportDescriptor );	
							Ready = 1;
							break;
                        default:
                            len = 0xff; //��֧�ֵ�������߳���
                            break;
                        }
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; //�ݴ�USB�豸��ַ
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1)
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        if (UsbConfig) {
                            Ready = 1;  // 取消注释，允许设备配置完成标志
                        }
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                                       //Clear Feature
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // �˵�
                        {
                            switch (UsbSetupBuf->wIndexL)
                            {
							case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
							case 0x83:
                                UEP3_CTRL = UEP3_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x03:
                                UEP3_CTRL = UEP3_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF; // ��֧�ֵĶ˵�
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF; // ���Ƕ˵㲻֧��
                        }
                        break;
                    case USB_SET_FEATURE:                             /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* �����豸 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    /* ���û���ʹ�ܱ�־ */
                                }
                                else
                                {
                                    len = 0xFF; /* ����ʧ�� */
                                }
                            }
                            else
                            {
                                len = 0xFF; /* ����ʧ�� */
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* ���ö˵� */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
								case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* ���ö˵�1 IN STALL */
                                    break;
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* ���ö˵�2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* ���ö˵�2 OUT Stall */
                                    break;
                                case 0x83:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* ���ö˵�2 IN STALL */
                                    break;
                                case 0x03:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* ���ö˵�2 OUT Stall */
                                    break;
                                default:
                                    len = 0xFF; /* ����ʧ�� */
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF; /* ����ʧ�� */
                            }
                        }
                        else
                        {
                            len = 0xFF; /* ����ʧ�� */
                        }
                        break;
                    case USB_GET_STATUS:
                        //pDescr = (PUINT8)&USB_STATUS;
						Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if (SetupLen >= 2)
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; //����ʧ��
                        break;
                    }

                    break;
                case USB_REQ_TYP_CLASS: /*HID������*/
                    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_INTERF)
                    {
                        switch (SetupReq)
                        {
                        case 0x20://Configure
                            break;
                        case 0x21://currently configured
                            pDescr = LineCoding;
                            len = sizeof(LineCoding);
                            break;
                        case 0x22://generates RS-232/V.24 style control signals
							USBD0 = Ep0Buffer[2]+1;
                            break;
                        default:
                            len = 0xFF; /*���֧��*/
                            break;
                        }
                    }
                    break;
                case USB_REQ_TYP_VENDOR:
                    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                    {
                        switch (SetupReq)
                        {
                        case 0x20:                         //GetReport
                            if (UsbSetupBuf->wIndexL == 0x07)
                            {

                            }
                            break;
                        default:
                            len = 0xFF; /*���֧��*/
                            break;
                        }
                    }
                    break;
                default:
                    len = 0xFF;
                    break;
                }
                if (len != 0 && len != 0xFF)
                {
                    if (SetupLen > len)
                    {
                        SetupLen = len; //�����ܳ���
                    }
                    len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //���δ��䳤��
                    memcpy(Ep0Buffer, pDescr, len);                                 //�����ϴ�����
                    SetupLen -= len;
                    pDescr += len;
                }
            }
            else
            {
                len = 0xff; //�����ȴ���
            }
            if (len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; //STALL
            }
            else if (len <= THIS_ENDP0_SIZE) //�ϴ����ݻ���״̬�׶η���0���Ȱ�
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //Ĭ�����ݰ���DATA1������Ӧ��ACK
            }
            else
            {
                UEP0_T_LEN = 0;                                                      //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //Ĭ�����ݰ���DATA1,����Ӧ��ACK
            }
            break;
        case UIS_TOKEN_IN | 0: //endpoint0 IN
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case 0x20:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //���δ��䳤��
                memcpy(Ep0Buffer, pDescr, len);                                 //�����ϴ�����
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; //ͬ����־λ��ת
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; //״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0: // endpoint0 OUT
            len = USB_RX_LEN;
            if (SetupReq == 0x20) //���ô�������
            {
                if (U_TOG_OK)
                {
					num_s = 0;
					for(temp=0;temp<4;temp++)
					{
						if((Ep0Buffer[temp])!=(LineCoding[temp]))
							num_s++;
					}
					if(num_s)
					{
						memcpy(LineCoding, UsbSetupBuf, USB_RX_LEN);
						Config_Uart1(LineCoding);
					}

                    UEP0_T_LEN = 0;
                    UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // ׼���ϴ�0��
                }
            }
            else if (SetupReq == 0x09)
            {
                if (Ep0Buffer[0])
                {
                }
                else if (Ep0Buffer[0] == 0)
                {
                }
            }
            UEP0_CTRL ^= bUEP_R_TOG; //ͬ����־λ��ת
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0; //д0����ж�
    }
    if (UIF_BUS_RST) //�豸ģʽUSB���߸�λ�ж�
    {
        printf("USB Bus Reset\n");
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP3_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;                                                             //���жϱ�־
		USBByteCount = 0;       //USB�˵��յ��ĳ���
		UsbConfig = 0;          //�������ֵ
        Endp3Busy = 0;
		Endp2Busy = 0;
		Endp1Busy = 0;
    }
    if (UIF_SUSPEND) //USB���߹���/�������
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) //����
        {
        }
    }
    else
    {
        //������ж�,�����ܷ��������
        USB_INT_FG = 0xFF; //���жϱ�־
    }
}


/*******************************************************************************
* Function Name  : static void UploadData(void)
* Description    : Upload the HID code
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

/*******************************************************************************
* Function Name  : extern HIDValueHandle( void )
* Description    : Upload the HID code
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


/**************************** END *************************************/
