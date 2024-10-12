/********************************** (C) COPYRIGHT *******************************
* File Name          : CHRV3UFI.h
* Author             : WCH
* Version            : V1.0
* Date               : 2014/09/09
* Description 		 : USB-flash File Interface for CHRV
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/* CHRV3 USB host file system interface, supports: FAT12/FAT16/FAT32 */

//#define DISK_BASE_BUF_LEN  512  /* The default cache size for FAT data is 512 bytes (can be set to 2048 or up to 4096 to support some large-sector USB drives). If set to 0, caching in this file will be disabled, and the application program should specify the buffer using pDISK_BASE_BUF */

/* If you need to reuse the FAT data cache to save RAM, you can define DISK_BASE_BUF_LEN as 0 to disable caching in this file, and the application program should assign the starting address of the cache shared with other programs to the variable pDISK_BASE_BUF before calling CHRV3LibInit */

//#define NO_DEFAULT_ACCESS_SECTOR  1   /* Disable the default sector read/write routines, and replace them with custom code below */
//#define NO_DEFAULT_DISK_CONNECT   1   /* Disable the default disk connection detection routine, and replace it with custom code below */
//#define NO_DEFAULT_FILE_ENUMER    1   /* Disable the default file enumeration callback routine, and replace it with custom code below */

//#include "CHRV3SFR.H"
#include "CH56xSFR.h"
#include "CHRV3UFI.h"
#include "Ch56x_UDISK.h"

CMD_PARAM_I mCmdParam;                     /* Command parameters */
#if     DISK_BASE_BUF_LEN > 0
//UINT8  DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((at(BA_RAM+SZ_RAM/2)));    /* External RAM's FAT data cache area, the length of the cache area is the size of one sector */
UINT8  DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4)));    /* External RAM's FAT data cache area, the length of the cache area is the size of one sector */
//UINT8  DISK_FAT_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4)));    /* External RAM's FAT data cache area, the length of the cache area is the size of one sector */
#endif

UINT8	HostCtrlTransfer( PUINT8 DataBuf, PUINT8 RetLen )
{

}
UINT8	CtrlClearEndpStall( UINT8 endp )
{

}
UINT8	USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT32 timeout )
{
}
UINT8	CtrlGetConfigDescrTB( void )
{

}

UINT8	CtrlSetUsbConfig( UINT8 cfg )
{
}
//UINT8	CHRV3BulkOnlyCmd( PUINT8 DataBuf )
//{
////	return MS_ScsiCmd_Process( DataBuf );
//}

/* The following code can be modified as needed */

#ifndef NO_DEFAULT_ACCESS_SECTOR    /* Defining NO_DEFAULT_ACCESS_SECTOR in the application program can disable the default sector read/write routines, and then use custom code to replace them */
//if ( use_external_interface ) {  // Replace with custom USB disk sector read/write routines
//    CHRV3vSectorSize=512;  // Set the actual sector size, which must be a multiple of 512, and this value is the sector size of the disk
//    CHRV3vSectorSizeB=9;   // Set the bit shift value for the actual sector size, 512 corresponds to 9, 1024 corresponds to 10, and 2048 corresponds to 11
//    CHRV3DiskStatus=DISK_MOUNTED;  // Manually set the device connection status as successful (just short of fully resolving the file system)
//}

UINT8	CHRV3ReadSector( UINT8 SectCount, PUINT8 DataBuf )  /* 从磁盘读取多个扇区的数据到缓冲区中 */
{
	UINT8	retry;
//	if ( use_external_interface ) return( extReadSector( CHRV3vLbaCurrent, SectCount, DataBuf ) );  /* 外部接口 */
//	printf("pDISK_FAT_BUF=%08lx\n",(UINT32)DataBuf);
	retry = MS_ReadSector( CHRV3vLbaCurrent,(UINT16)SectCount,DataBuf );
	return retry;
#if 0

	for( retry = 0; retry < 3; retry ++ ) {  /* 错误重试 */
		pCBW -> mCBW_DataLen = (UINT32)SectCount << CHRV3vSectorSizeB;  /* 数据传输长度 */
		pCBW -> mCBW_Flag = 0x80;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_READ10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (UINT8)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (UINT8)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (UINT8)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (UINT8)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* 执行基于BulkOnly协议的命令 */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* 磁盘操作错误 */
#endif
}

#ifdef	EN_DISK_WRITE
UINT8	CHRV3WriteSector( UINT8 SectCount, PUINT8 DataBuf )  /* 将缓冲区中的多个扇区的数据块写入磁盘 */
{

	UINT8	retry;
    retry = MS_WriteSector( CHRV3vLbaCurrent,(UINT16)SectCount,DataBuf );
    return retry;
//	if ( use_external_interface ) return( extWriteSector( CHRV3vLbaCurrent, SectCount, DataBuf ) );  /* 外部接口 */
#if 0
	for( retry = 0; retry < 3; retry ++ ) {  /* 错误重试 */
		pCBW -> mCBW_DataLen = (UINT32)SectCount << CHRV3vSectorSizeB;  /* 数据传输长度 */
		pCBW -> mCBW_Flag = 0x00;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_WRITE10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (UINT8)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (UINT8)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (UINT8)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (UINT8)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* 执行基于BulkOnly协议的命令 */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			mDelayuS( 200 );  /* 写操作后延时 */
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* 磁盘操作错误 */
#endif
}
#endif
#endif  // NO_DEFAULT_ACCESS_SECTOR

#ifndef	NO_DEFAULT_DISK_CONNECT			/* 在应用程序中定义NO_DEFAULT_DISK_CONNECT可以禁止默认的检查磁盘连接子程序,然后用自行编写的程序代替它 */

/*
约定: USB设备地址分配规则(参考USB_DEVICE_ADDR)
地址值  设备位置
0x02    内置Root-HUB0下的USB设备或外部HUB
0x03    内置Root-HUB1下的USB设备或外部HUB
0x1x    内置Root-HUB0下的外部HUB的端口x下的USB设备,x为1~n
0x2x    内置Root-HUB1下的外部HUB的端口x下的USB设备,x为1~n
*/
//UINT8 R8_USB_DEV_AD;
//UINT8 R8_USB_MIS_ST;
//UINT8 R8_UHOST_CTRL;
//UINT8 R8_USB_INT_FG;
UINT8 RB_UMS_DEV_ATTACH;
UINT8 RB_UMS_SUSPEND;
//#define		UHUB_DEV_ADDR	( CHRV3vRootPort ? R8_USB1_DEV_AD : R8_USB0_DEV_AD )
//#define		UHUB_MIS_STAT	( CHRV3vRootPort ? R8_USB1_MIS_ST : R8_USB0_MIS_ST )
//#define		UHUB_HOST_CTRL	( CHRV3vRootPort ? R8_UHOST1_CTRL : R8_UHOST0_CTRL )
//#define		UHUB_INT_FLAG	( CHRV3vRootPort ? R8_USB1_INT_FG : R8_USB0_INT_FG )
#define		UHUB_DEV_ADDR	R8_USB_DEV_AD
#define		UHUB_MIS_STAT	R8_USB_MIS_ST
#define		UHUB_HOST_CTRL	R8_UHOST_CTRL
#define		UHUB_INT_FLAG	R8_USB_INT_FG
#define		bUMS_ATTACH		RB_UMS_DEV_ATTACH
#define		bUMS_SUSPEND	RB_UMS_SUSPEND

/* 检查磁盘是否连接 */
UINT8	CHRV3DiskConnect( void )
{
	UINT8	ums, devaddr;
	UHUB_DEV_ADDR = UHUB_DEV_ADDR & 0x7F;
	ums = UHUB_MIS_STAT;
	devaddr = UHUB_DEV_ADDR;
	if ( devaddr == USB_DEVICE_ADDR ) {  /* 内置Root-HUB下的USB设备 */
//		if ( UHUB_HOST_CTRL & RB_UH_PORT_EN ) {  /* 内置Root-HUB下的USB设备存在且未插拔 */
		if ( ums & bUMS_ATTACH ) {  /* 内置Root-HUB下的USB设备存在 */
//			if ( ( UHUB_INT_FLAG & UIF_DETECT ) == 0 ) {  /* 内置Root-HUB下的USB设备存在且未插拔 */
			if ( ( ums & bUMS_SUSPEND ) == 0 ) {  /* 内置Root-HUB下的USB设备存在且未插拔 */
				return( ERR_SUCCESS );  /* USB设备已经连接且未插拔 */
			}
			else {  /* 内置Root-HUB下的USB设备存在 */
//mDiskConnect:
				CHRV3DiskStatus = DISK_CONNECT;  /* 曾经断开过 */
				return( ERR_SUCCESS );  /* 外部HUB或USB设备已经连接或者断开后重新连接 */
			}
		}
		else {  /* USB设备断开 */
mDiskDisconn:
			CHRV3DiskStatus = DISK_DISCONNECT;
			return( ERR_USB_DISCON );
		}
	}
#if 0
//#ifndef	FOR_ROOT_UDISK_ONLY
	else if ( devaddr > 0x10 && devaddr <= 0x14 ) {  /* 外部HUB的端口下的USB设备 */
//		if ( UHUB_HOST_CTRL & RB_UH_PORT_EN ) {  /* 内置Root-HUB下的外部HUB存在且未插拔 */
		if ( ums & bUMS_ATTACH ) {  /* 内置Root-HUB下的USB设备存在 */
//			if ( ( UHUB_INT_FLAG & UIF_DETECT ) == 0 ) {  /* 内置Root-HUB下的USB设备存在且未插拔 */
			if ( ( ums & bUMS_SUSPEND ) == 0 ) {  /* 内置Root-HUB下的USB设备存在且未插拔 */
				TxBuffer[ MAX_PACKET_SIZE - 1 ] = devaddr;  /* 备份 */
				UHUB_DEV_ADDR = USB_DEVICE_ADDR - 1 + ( UHUB_DEV_ADDR >> 4 );  /* 设置USB主机端的USB地址指向HUB */
				CHRV3IntStatus = HubGetPortStatus( TxBuffer[ MAX_PACKET_SIZE - 1 ] & 0x0F );  /* 查询HUB端口状态,返回在TxBuffer中 */
				if ( CHRV3IntStatus == ERR_SUCCESS ) {
					if ( TxBuffer[2] & (1<<(HUB_C_PORT_CONNECTION-0x10)) ) {  /* 检测到HUB端口上的插拔事件 */
						CHRV3DiskStatus = DISK_DISCONNECT;  /* 假定为HUB端口上的USB设备断开 */
						HubClearPortFeature( TxBuffer[ MAX_PACKET_SIZE - 1 ] & 0x0F, HUB_C_PORT_CONNECTION );  /* 清除HUB端口连接事件状态 */
					}
					UHUB_DEV_ADDR = TxBuffer[ MAX_PACKET_SIZE - 1 ];  /* 设置USB主机端的USB地址指向USB设备 */
					if ( TxBuffer[0] & (1<<HUB_PORT_CONNECTION) ) {  /* 连接状态 */
						if ( CHRV3DiskStatus < DISK_CONNECT ) {
							CHRV3DiskStatus = DISK_CONNECT;  /* 曾经断开过 */
						}
						return( ERR_SUCCESS );  /* USB设备已经连接或者断开后重新连接 */
					}
					else {
//						CHRV3DiskStatus = DISK_DISCONNECT;
//						return( ERR_USB_DISCON );
						CHRV3DiskStatus = DISK_CONNECT;
						return( ERR_HUB_PORT_FREE );  /* HUB已经连接但是HUB端口尚未连接磁盘 */
					}
				}
				else {
					UHUB_DEV_ADDR = TxBuffer[ MAX_PACKET_SIZE - 1 ];  /* 设置USB主机端的USB地址指向USB设备 */
					if ( CHRV3IntStatus == ERR_USB_DISCON ) {
//						CHRV3DiskStatus = DISK_DISCONNECT;
//						return( ERR_USB_DISCON );
						goto mDiskDisconn;
					}
					else {
						CHRV3DiskStatus = DISK_CONNECT;  /* HUB操作失败 */
						return( CHRV3IntStatus );
					}
				}
			}
			else {  /* 内置Root-HUB下的USB设备存在,外部HUB或USB设备已经连接或者断开后重新连接 */
//				CHRV3DiskStatus = DISK_CONNECT;  /* 曾经断开过 */
//				return( ERR_SUCCESS );  /* 外部HUB或USB设备已经连接或者断开后重新连接 */
				goto mDiskConnect;
			}
		}
		else {  /* 外部HUB断开 */
			CHRV3DiskStatus = DISK_DISCONNECT;
		}
	}
#endif
	else {
//		CHRV3DiskStatus = DISK_DISCONNECT;
//		return( ERR_USB_DISCON );
		goto mDiskDisconn;
	}
}
#endif  // NO_DEFAULT_DISK_CONNECT

#ifndef	NO_DEFAULT_FILE_ENUMER			/* 在应用程序中定义NO_DEFAULT_FILE_ENUMER可以禁止默认的文件名枚举回调程序,然后用自行编写的程序代替它 */
void xFileNameEnumer( void )			/* 文件名枚举回调子程序 */
{
/* 如果指定枚举序号CHRV3vFileSize为0xFFFFFFFF后调用FileOpen，那么每搜索到一个文件FileOpen都会调用本回调程序，
   回调程序xFileNameEnumer返回后，FileOpen递减CHRV3vFileSize并继续枚举直到搜索不到文件或者目录。建议做法是，
   在调用FileOpen之前定义一个全局变量为0，当FileOpen回调本程序后，本程序由CHRV3vFdtOffset得到结构FAT_DIR_INFO，
   分析结构中的DIR_Attr以及DIR_Name判断是否为所需文件名或者目录名，记录相关信息，并将全局变量计数增量，
   当FileOpen返回后，判断返回值如果是ERR_MISS_FILE或ERR_FOUND_NAME都视为操作成功，全局变量为搜索到的有效文件数。
   如果在本回调程序xFileNameEnumer中将CHRV3vFileSize置为1，那么可以通知FileOpen提前结束搜索。以下是回调程序例子 */
#if		0
	UINT8			i;
	UINT16			FileCount;
	PX_FAT_DIR_INFO	pFileDir;
	PUINT8			NameBuf;
	pFileDir = (PX_FAT_DIR_INFO)( pDISK_BASE_BUF + CHRV3vFdtOffset );  /* 当前FDT的起始地址 */
	FileCount = (UINT16)( 0xFFFFFFFF - CHRV3vFileSize );  /* 当前文件名的枚举序号,CHRV3vFileSize初值是0xFFFFFFFF,找到文件名后递减 */
	if ( FileCount < sizeof( FILE_DATA_BUF ) / 12 ) {  /* 检查缓冲区是否足够存放,假定每个文件名需占用12个字节存放 */
		NameBuf = & FILE_DATA_BUF[ FileCount * 12 ];  /* 计算保存当前文件名的缓冲区地址 */
		for ( i = 0; i < 11; i ++ ) NameBuf[ i ] = pFileDir -> DIR_Name[ i ];  /* 复制文件名,长度为11个字符,未处理空格 */
//		if ( pFileDir -> DIR_Attr & ATTR_DIRECTORY ) NameBuf[ i ] = 1;  /* 判断是目录名 */
		NameBuf[ i ] = 0;  /* 文件名结束符 */
	}
#endif
}
#endif  // NO_DEFAULT_FILE_ENUMER

UINT8	CHRV3LibInit( void )  /* 初始化CHRV3程序库,操作成功返回0 */
{
	if ( CHRV3GetVer( ) < CHRV3_LIB_VER ) return( 0xFF );  /* 获取当前子程序库的版本号,版本太低则返回错误 */
#if		DISK_BASE_BUF_LEN > 0
	pDISK_BASE_BUF = & DISK_BASE_BUF[0];  /* Points to the external RAM's cache data area */
	pDISK_FAT_BUF = & DISK_BASE_BUF[0];  /* Points to the external RAM's FAT data cache area, can be used together with pDISK_BASE_BUF to estimate RAM */
//	pDISK_FAT_BUF = & DISK_FAT_BUF[0];  /* Points to the external RAM's FAT data cache area, independent of pDISK_BASE_BUF to improve speed */
/* If you want to improve file access speed, you can call CHRV3LibInit in the main program and reassign pDISK_FAT_BUF to another independently allocated cache area of the same size as pDISK_BASE_BUF */
#endif
	CHRV3DiskStatus = DISK_UNKNOWN;  /* 未知状态 */
	CHRV3vSectorSizeB = 9;  /* 默认的物理磁盘的扇区是512B */
	CHRV3vSectorSize = 512;  // 默认的物理磁盘的扇区是512B,该值是磁盘的扇区大小
	CHRV3vStartLba = 0;  /* 默认为自动分析FDD和HDD */
	CHRV3vPacketSize = 1024;  /* USB存储类设备的最大包长度:64@FS,512@HS/SS,由应用程序初始化,枚举U盘后如果是高速或者超速那么及时更新为512 */
//	pTX_DMA_A_REG = (PUINT32)&R16_UH_TX_DMA;  /* 指向发送DMA地址寄存器,由应用程序初始化 */
//	pRX_DMA_A_REG = (PUINT32)&R16_UH_RX_DMA;  /* 指向接收DMA地址寄存器,由应用程序初始化 */
//	pTX_LEN_REG = (PUINT16)&R8_UH_TX_LEN;  /* 指向发送长度寄存器,由应用程序初始化 */
//	pRX_LEN_REG = (PUINT16)&R8_USB_RX_LEN;  /* 指向接收长度寄存器,由应用程序初始化 */

//CHRV3vRootPort = 0;  /* USB主机选择(类似Root-hub根集线器选端口) */
	return( ERR_SUCCESS );
}

