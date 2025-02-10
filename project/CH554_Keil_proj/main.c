/********************************** (C) COPYRIGHT ******************************
* File Name          :Compound_Dev.C											
* Author             : WCH                                                      
* Version            : V1.0                                                     
* Date               : 2017/03/15                                               
* Description        : A demo for USB compound device created by CH554, support 
					   keyboard and mouse, and HID compatible device.           
********************************************************************************/
#include ".\Public\CH554.H"
#include "Compound.h"
#include ".\Public\debug.h"
#include <stdio.h>
#include <string.h>
#include ".\Public\GPIO.H"
sbit check = P1^4;
UINT8 flag1_1 = 1;


extern UINT8 	FLAG;												  // Trans complete flag
extern UINT8 	EnumOK;												// Enum ok flag	

UINT8 Res;	
UINT8 D1_len=0,D2_len=0;
UINT8 Flag_3=0,Flag_4=0;
UINT8X USART_RX_BUF[66];     //?����??o3?,��?�䨮USART_REC_LEN??��??��.
UINT8X USART3_RX_BUF[140];     //?����??o3?,��?�䨮USART_REC_LEN??��??�.
//?����?���䨬?
//bit15��?	?����?����3������??
//bit14��?	?����?��?0x0d
//bit13~0��?	?����?��?��?��DD���??����y??
UINT16 USART_RX_STA=0;       //?����?���䨬?����??	  
UINT16 USART3_RX_STA=0;       //?����?���䨬?����??	
UINT16 num_1 =0,num_2=0;  				

void DAP_Thread(void)
{
    UINT8I num;

    if (USB_RequestFlag)
    {
		CH554UART0SendByte(0x66);
		num_2 = 0;
		num_1= 63;
		while(1)
		{
			num_2 += Ep3Buffer[num_1];
			if(num_2)break;
			num_1--;
		}
		CH554UART0SendByte(num_1+1);
		for(num=0;num<num_1+1;num++)
		{	
			CH554UART0SendByte(Ep3Buffer[num]);
		}
		while((USART_RX_STA&0x8000)==0);		
		USART_RX_STA = 0;	
		for(num=0;num<64;num++)
		{
			Ep3Buffer2[num] = USART_RX_BUF[num];//???1????
		}
		UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK; //enable receive
	    UEP3_T_LEN = 64;
	    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //enable send
		USB_RequestFlag = 0;
    }
}

void main( void )
{
	UINT8I num=0,lenth=0;
	UINT16X timeout;  // 使用CH554的XDATA存储类型
	
	CfgFsys();//时钟选择配置
	mDelaymS(5);//修改频率等待内部晶振稳定,关键
	Port1Cfg(3,4);                                                             //P16推挽输出模式
	
	// 初始化调试串口
	mInitSTDIO();
	printf("System Init Start\n");
	
	USBDeviceInit();//初始化USB设备
	printf("USB Device Init Done\n");
	
	EA = 1;	
	UEP1_T_LEN = 0;
	UEP2_T_LEN = 0;
	UEP3_T_LEN = 0;
	Endp2Busy = 0;
	
	while(flag1_1)
	{
		if(check == 0)
		{
			flag1_1=0;
		}
		mDelaymS(100);  
	}
	
	for(num=0;num<64;num++)
	{
		USART_RX_BUF[num]= 0;
	}
	
	timeout = 0;
	while(!UsbConfig && timeout < 1000) { // 等待USB枚举完成
		mDelaymS(1);
		timeout++;
		if(timeout % 100 == 0)
		{
			printf("Waiting for USB Config: %d ms\n", timeout);
			// 输出USB状态寄存器值以便调试
			printf("USB_CTRL: %02X, UDEV_CTRL: %02X\n", USB_CTRL, UDEV_CTRL);
		}
	}
	
	if(!UsbConfig) {
		printf("USB Enumeration Failed\n");
		while(1) {  // USB枚举失败
			mDelaymS(100);
		}
	}
	printf("USB Enumeration Success\n");
	
	while(1)
	{
//		if(USART_RX_STA&0x8000)
//		{		
//			USART_RX_STA = 0;	
//			for(num=0;num<64;num++)
//			{
//				CH554UART0SendByte(USART_RX_BUF[num]);
//			}
//		}
//		if(UsbConfig)//��λ���򿪴���
//		{
//			if(USBByteCount)   //USB���ն˵�������
//			{
//				if(!Endp2Busy)   //�˵㲻��æ�����к�ĵ�һ�����ݣ�ֻ���������ϴ���
//				{
//					lenth = USBByteCount;		
//					//д�ϴ��˵�
//					memcpy(Ep2Buffer+MAX_PACKET_SIZE,Ep2Buffer,lenth);
//					UEP2_T_LEN = lenth;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
//					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
//					Endp2Busy = 1;
//					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;//�����������꣬���������޸���Ӧ��ʽ
//					USBByteCount= 0;				
//				}				
//			}
//		}
		DAP_Thread();
       	if (USBByteCount)
        {
			lenth = USBByteCount;
			CH554UART0SendByte(0x67);
			CH554UART0SendByte(lenth);
			for(num=0;num<lenth;num++)
			{
				CH554UART0SendByte(Ep2Buffer[num]);
			}
//            CH554UART0SendByte(Ep2Buffer[USBBufOutPoint++]);
            USBByteCount=0;
            if (USBByteCount == 0)
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
        }
		if(USBD0)
		{
//			Ep2Buffer[64] =   USBD0;
//			UEP2_T_LEN = 1;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
//			UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
//			Endp2Busy = 1;
			
			CH554UART0SendByte(0x68);
			CH554UART0SendByte(1);
			CH554UART0SendByte(USBD0);
			USBD0 = 0;
		}
		if(USART3_RX_STA&0x8000)
		{
			if (!Endp2Busy)
			{
				lenth = USART3_RX_STA&0X0FFF;
				if(lenth<64)
				{
					memcpy(Ep2Buffer+64,&USART3_RX_BUF[0],lenth);
					UEP2_T_LEN = lenth;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
					Endp2Busy = 1;
					USART3_RX_STA = 0;
				}else if(lenth==64)
				{
					memcpy(Ep2Buffer+64,&USART3_RX_BUF[0],lenth);
					UEP2_T_LEN = lenth;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
					Endp2Busy = 1;
					while(Endp2Busy);	
					UEP2_T_LEN = 0;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
					Endp2Busy = 1;
					USART3_RX_STA = 0;
				}else if(lenth>64)
				{
					for(num=0;num<(lenth/64);num++)
					{
						memcpy(Ep2Buffer+64,&USART3_RX_BUF[num*64],64);
						UEP2_T_LEN = 64;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
						UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
						Endp2Busy = 1;
						while(Endp2Busy);	
						UEP2_T_LEN = 0;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
						lenth-=64;
					}
					memcpy(Ep2Buffer+64,&USART3_RX_BUF[num*64],lenth);
					UEP2_T_LEN = lenth;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
					Endp2Busy = 1;
					USART3_RX_STA = 0;
				}
			}
		}		
			
//		if(UartByteCount)
//				Uart_Timeout++;
//		if(!Endp2Busy)   //�˵㲻��æ�����к�ĵ�һ�����ݣ�ֻ���������ϴ���
//		{
//			lenth = UartByteCount;
//			if(lenth>0)
//			{
//				if(lenth>39 || Uart_Timeout>100)
//				{		
//					Uart_Timeout = 0;
//					UartByteCount -= lenth;			
//					//д�ϴ��˵�
//					Uart_Output_Point+=lenth;
//					if(Uart_Output_Point>=64)
//						Uart_Output_Point = 0;						
//					UEP2_T_LEN = lenth;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
//					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
//					Endp2Busy = 1;
//				}
//			}
//		}
//        if (USBByteCount)
//        { 
//			if (!Endp2Busy)
//			{
//				memcpy(Ep2Buffer+64,&Ep3Buffer[0],64);
//				Uart_Output_Point+=64;
//				UEP2_T_LEN = 64;                                                    //Ԥʹ�÷��ͳ���һ��Ҫ���
//				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //Ӧ��ACK
//				Endp2Busy = 1;
//			}
//        }		
    }
}

void Uart0_ISR(void) interrupt INT_NO_UART0
{
	if(RI)  //?����??D??(?����?��?��?��y?Y��?D?��?0x0d 0x0a?��?2)
	{
		Res = SBUF;	//?����??����?��?��?��y?Y
		if(((USART_RX_STA&0x8000)||(USART3_RX_STA&0x8000))==0)
		{
			if(((USART_RX_STA&0x4000)||(USART3_RX_STA&0x4000))==0)
			{
				if(Res==0x66)
				{
					USART_RX_STA|=0x4000;//?����?��???����
					Flag_3 = 1;
				}else if(Res==0x67)
				{
					USART3_RX_STA|=0x4000;//?����?��???����
					Flag_4 = 1;
				}else
				{
					USART_RX_STA = 0;
					USART3_RX_STA = 0;
				}
			}else if((((USART_RX_STA&0x4000))&&(~(USART3_RX_STA&0x4000))))
			{
				if(Flag_3)
				{
					D1_len = Res;
					Flag_3 = 0;
				}else
				{
					USART_RX_BUF[USART_RX_STA&0X0FFF]=Res ;
					USART_RX_STA++;
					if((USART_RX_STA&0x0FFF)==D1_len)USART_RX_STA|=0x8000;	//?����?����3����? 
				}
			}else if((((~USART_RX_STA&0x4000))&&((USART3_RX_STA&0x4000))))
			{
				if(Flag_4)
				{
					D2_len = Res;
					Flag_4 = 0;
				}else
				{
					USART3_RX_BUF[USART3_RX_STA&0X0FFF]=Res ;
					USART3_RX_STA++;
					if((USART3_RX_STA&0x0FFF)==D2_len)USART3_RX_STA|=0x8000;	//?����?����3����? 
				}
			}
		}else if((((USART_RX_STA&0x8000))&&(~(USART3_RX_STA&0x8000))))
		{
			if((USART3_RX_STA&0x4000)==0)
			{
				if(Res==0x67)
				{
					USART3_RX_STA|=0x4000;//?����?��???����
					Flag_4 = 1;
				}else
				{
					USART_RX_STA = 0;
				}
			}else
			{
				if(Flag_4)
				{
					D2_len = Res;
					Flag_4 = 0;
				}else
				{
					USART3_RX_BUF[USART3_RX_STA&0X0FFF]=Res ;
					USART3_RX_STA++;
					if((USART3_RX_STA&0x0FFF)==D2_len)USART3_RX_STA|=0x8000;	//?����?����3����? 
				}
			}
		}else if(((~(USART_RX_STA&0x8000))&&((USART3_RX_STA&0x8000))))
		{
			if((USART_RX_STA&0x4000)==0)
			{
				if(Res==0x66)
				{
					USART_RX_STA|=0x4000;//?����?��???����
					Flag_3 = 1;
				}else
				{
					USART_RX_STA = 0;
				}
			}else
			{
				if(Flag_3)
				{
					D1_len = Res;
					Flag_3 = 0;
				}else
				{
					USART_RX_BUF[USART_RX_STA&0X0FFF]=Res ;
					USART_RX_STA++;
					if((USART_RX_STA&0x0FFF)==D1_len)USART_RX_STA|=0x8000;	//?����?����3����? 
				}
			}
		}
		RI = 0;
	} 
}

/**************************** END *************************************/
