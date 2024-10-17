/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * @Note
 * This routine demonstrates CH569 as a USB host to operate udisk.
 * The file system of the USB disk needs to be FAT12/FAT16/FAT32.
*/

#include "CH56x_common.h"
#include "CH56xusb30h_LIB.h"
#include "CH56x_usb30h.h"
#include "CH56x_host_hs.h"
#include "CHRV3UFI.h"
#include "CH56x_UDISK.h"
#include "fat_process.h"
#include "Instrumentation.h"

/* Global Variable */
UINT8V U30_Check_Time = 0;
volatile DevInfo g_DevInfo;

__attribute__ ((aligned(16))) UINT8 pNTFS_BUF[512] __attribute__((section(".DMADATA")));
__attribute__ ((aligned(16))) UINT8 pDataBuf[16384] __attribute__((section(".DMADATA")));

__attribute__ ((aligned(4))) UINT8  RxBuffer[1] ;      // IN, must even address
__attribute__ ((aligned(4))) UINT8  TxBuffer[1] ;      // OUT, must even address

/* Function  */
void TMR0_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));

/*******************************************************************************
 * @fn        TMR0_IRQHandler
 *
 * @brief     TMR0 handler.
 *
 * @return    None
 */
void TMR0_IRQHandler( void )
{
    if( R8_TMR0_INT_FLAG & RB_TMR_IF_CYC_END )
    {
        R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
        if( gDeviceConnectstatus == USB_INT_CONNECT_U20 )
        {
             U30_Check_Time++;
             if( U30_Check_Time>= 5 )
             {
                 printf("u20_connect\n");
                 gDeviceConnectstatus = USB_INT_CONNECT;
                 U30_Check_Time = 0;
             }
        }
    }
}

/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     nitializes the UART1 peripheral.
 *
 * @param     baudrate - UART1 communication baud rate.
 *
 * @return    None
 */
void DebugInit(UINT32 baudrate)
{
	UINT32 x;
	UINT32 t = FREQ_SYS;

	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;
	R8_UART1_DIV = 1;
	R16_UART1_DL = x;
	R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART1_LCR = RB_LCR_WORD_SZ;
	R8_UART1_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<8) |(1<<7);
	R32_PA_DIR |= (1<<8);
}


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main( void )
{
    UINT8 s = 0;

    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);
    DebugInit(115200);
    PRINT("\n\nThis is USB3.0 host program\n");

    PFIC_EnableIRQ(USBSS_IRQn); // ANDAX
    PFIC_EnableIRQ(LINK_IRQn);           //enable USBSSH LINK global interrupt
    USB20Host_Init(ENABLE);              //USB2.0initialization
    TMR0_TimerInit(FREQ_SYS);            //3.0link timeout timer initialization
    TMR0_ITCfg( ENABLE ,RB_TMR_IE_CYC_END);
    PFIC_EnableIRQ( TMR0_IRQn );

    while(1)
    {
       mDelayuS(2);
       if( gDeviceConnectstatus == USB_INT_CONNECT )
       {
           if( gDeviceUsbType == USB_U30_SPEED )
           {
               printf("U30HOST_Enumerate\n");
               mDelaymS(50);
               USB30Host_Enum();
           }
           else
           {
               printf("U20HOST_Enumerate\n");
               mDelaymS(5);
               s = USB20Host_Enum( endpRXbuff );
               if(s != ERR_SUCCESS)
               {
                   printf("enumerate failed\n");
               }
           }
           printf("Enumerate_status=%02x\n",s);
           s = MS_Init(pNTFS_BUF);
           printf("udisk_init=%02x\n",s);
           if( s == USB_OPERATE_SUCCESS )
           {
        	   // Test reading a raw sector:
        	   uint32_t StartLba = 100000000;
        	   uint16_t SectCount = 1;

        	   s = MS_ReadSector( StartLba, SectCount, pDataBuf );

        	   printf("MS_ReadSector=%02x (%s)\n", s, pDataBuf);


        	   StartLba = 300000000;
        	   SectCount = 32;
#if 1
        	   for (int c = 16; c < 16384; c++)
        	   {
        		   pDataBuf[c] = /*255 -*/ (c % 256);
        		   pDataBuf[c] = pDataBuf[c-16];
        	   }
#endif

        	   R32_TMR1_CNT_END = 0xffffffff;
        	   R8_TMR1_CTRL_MOD = RB_TMR_ALL_CLEAR;
        	   R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN | RB_TMR_CAP_COUNT;

        	   start_instrumentation();

        	   ADD_INSTR_EVENT(EVENT_TYPE_0);

        	   for (uint32_t sector = 0; sector < 1024; sector += SectCount)
        	   {
//            	   s = MS_WriteSector(StartLba + sector, SectCount, pDataBuf);
            	   s = AA_WriteSector(StartLba + sector, SectCount, pDataBuf);
            	   if (s != USB_OPERATE_SUCCESS)
            	   {
                	   printf("ERROR: MS_WriteSector=%02x\n", s);
            	   }
        	   }

        	   ADD_INSTR_EVENT(EVENT_TYPE_1);

        	   uint32_t count = R32_TMR1_COUNT;

        	   print_instr_event_buffer();

        	   printf("MS_WriteSector=%02x\n", s);
        	   printf("Delay counter: %d (%d)\n", count, FREQ_SYS);
        	   printf("Data rate: %d MB/s\n", 512*1024*120/count);

        	   s = Fat_Init();
           }
           printf("end\n");
           printf("wait_disconnect\n");
           while(gDeviceConnectstatus == USB_INT_CONNECT) //Wait for the device to disconnect
           {
               mDelaymS( 100 );
               if( gDeviceUsbType == USB_U20_SPEED )
               {
                   R8_USB_INT_FG = RB_USB_IF_DETECT;
                   if( R8_USB_MIS_ST & RB_USB_ATTACH )
                   {

                   }
                   else   break;
               }
           }
           gDeviceConnectstatus = 0;
           gDeviceUsbType = 0;
           MY_USB30HOST_Init(DISABLE,endpTXbuff,endpRXbuff);
           USB20Host_Init(ENABLE);
           printf("disconnect\n");
           mDelaymS(10);
           R8_SAFE_ACCESS_SIG = 0x57;                  //enable safe access mode
           R8_SAFE_ACCESS_SIG = 0xa8;
           R8_RST_WDOG_CTRL = 0x40 | RB_SOFTWARE_RESET;
           while(1);
       }
       else if( R8_USB_INT_FG & RB_USB_IF_DETECT )     //Connect the USB2.0 device first, and then initialize the USB3.0 device connection
       {
           R8_USB_INT_FG = RB_USB_IF_DETECT;
           if( R8_USB_MIS_ST & RB_USB_ATTACH )
           {
               printf("USB2.0 DEVICE ATTACH !\n");
               gDeviceUsbType = USB_U20_SPEED;
               gDeviceConnectstatus = USB_INT_CONNECT_U20;
               mDelaymS(100);
               MY_USB30HOST_Init(ENABLE,endpTXbuff,endpRXbuff);
               USB20HOST_SetBusReset();
           }
           else
           {
               printf("USB2.0 DEVICE DEATTACH !\n");
           }
       }
    }
}


