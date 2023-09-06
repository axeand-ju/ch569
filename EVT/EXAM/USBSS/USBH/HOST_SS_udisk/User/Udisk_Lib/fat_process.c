/********************************** (C) COPYRIGHT *******************************
* File Name          : fat_process.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "string.h"
#include "CH56xSFR.h"
#include "core_riscv.h"
#include "CHRV3UFI.h"
#include "fat_process.h"
#include "stdio.h"
#include "type.h"
#include "CH56x_host_hs.h"


#define CREAT              1
#define READ               0
#define WRITE              0
#define BYTEWRITE_READ     1
#define MODIFY             1
#define ERASE              0

__attribute__ ((aligned(16))) UINT8 Buffer[512] __attribute__((section(".DMADATA"))); //Data sending/receiving buffer of endpoint1

/*******************************************************************************
 * @fn       Fat_Init
 *
 * @return   None
 */
 UINT8 Fat_Init( void )
 {
    UINT8  i,s,c,count;
    UINT8  *pd;
    UINT8  *pd1;
    UINT32 temp32;
    pd = AA;
    pd1 = BB;
    i = CHRV3LibInit();

    printf("LIB_Init=%02x\n",i);
    CHRV3DiskStatus = DISK_MOUNTED; /* The disk has been initialized successfully, but the file system has not been analyzed or is not supported */
    CHRV3vSectorSize =  512;
    CHRV3vSectorSizeB = 0;
    temp32 = CHRV3vSectorSize;
    for( i = 0; i <= 12; i++ )
    {
        temp32 = temp32 >> 1;
        CHRV3vSectorSizeB++;
        if( ( temp32 & 0x01 )== 0x01 )
        {
            break;
        }
    }

#if CREAT

    strcpy( (char *)&mCmdParam.Create.mPathName,( const char * )"\\NEWFILE.TXT");  //create NEWFILE.TXT
    i = CHRV3FileOpen();
    printf("i=%x\n",i);
    if((i == ERR_MISS_DIR)||(i == ERR_MISS_FILE))
    {
        printf( "Find No File And Create\r\n" );
        i = CHRV3FileCreate();
    }
    if( i!= ERR_SUCCESS )  return i;

    mCmdParam.Close.mUpdateLen=1;
    i = CHRV3FileClose();                                   //Modify file information completed
    if( i != ERR_SUCCESS )  return i;
#endif


#if WRITE
    printf("write\n");
    strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");
    i = CHRV3FileOpen();
    if( i!= ERR_SUCCESS )return i;
    memset(Buffer,0x42,512);

    mCmdParam.Write.mDataBuffer = Buffer;
    mCmdParam.Write.mSectorCount = 1;
    i=CHRV3FileWrite( );                               //Write data to NEWFILE.txt

    if( i != ERR_SUCCESS )  printf("write_error\n");
    for(s=0;s!=5;s++)  printf("%02x ",Buffer[s]);
    printf("\n");

    mCmdParam.Close.mUpdateLen = 1;
   i= CHRV3FileClose();                                 //Write to NEWFILE.txt completed
#endif

#if READ
   printf("read\n");
    strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");    //open file
    i = CHRV3FileOpen();
    if( i != ERR_SUCCESS ) return i;

    mCmdParam.Read.mDataBuffer = pd;
    mCmdParam.Read.mSectorCount = 16;
    i=CHRV3FileRead( );
    if( i != ERR_SUCCESS ) return i;

    if( mCmdParam.Read.mSectorCount < 16 )
    {
      printf("err Number of sectors read only��%d\n",mCmdParam.Read.mSectorCount);
    }

    mCmdParam.Read.mDataBuffer = pd1;
    mCmdParam.Read.mSectorCount = 16;
    i=CHRV3FileRead( );
    if( i != ERR_SUCCESS ) return i;

    if( mCmdParam.Read.mSectorCount < 16 )
    {
        printf("err Number of sectors read only��%d\n",mCmdParam.Read.mSectorCount);
    }

    mCmdParam.Close.mUpdateLen = 1;
    i= CHRV3FileClose();
#endif

#if BYTEWRITE_READ
    printf( "ByteWrite\r\n" );
    strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");    //open file
    i = CHRV3FileOpen();
    if( i != ERR_SUCCESS ) return i;


    i = sprintf( (PCHAR)Buffer,"Note: \xd\xa������������ֽ�Ϊ��λ����U���ļ���д,����ʾ���ܡ�\xd\xa");
    for(c=0; c<10; c++)
    {
        mCmdParam.ByteWrite.mByteCount = i;                          /* Specify the number of bytes written this time */
        mCmdParam.ByteWrite.mByteBuffer = Buffer;                    /* Point to buffer */
        s = CHRV3ByteWrite( );                                       /* Write data to the file in bytes */
        if(s != ERR_SUCCESS )
        {
            printf("write error:%02x\n",s);
            return s;
        }
        printf("�ɹ�д�� %02X��\r\n",(UINT16)c);
    }
    /* Read the first N bytes of the file */
    count = 10;                                                      //Set the total length to read 100 bytes
    printf( "������ǰ%d���ַ���:\r\n",count );
    while ( count ) {                                                //If the file is too large to be read at one time, you can call CHRV3ByteRead to continue reading, and the file pointer will move back automatically
        if ( count > (MAX_PATH_LEN-1) ) c = MAX_PATH_LEN-1;          /* There is a lot of data left. The length of a single read/write cannot exceed sizeof( mCmdParam.Other.mBuffer ) */
        else c = count;                                              /* Last remaining bytes */
        mCmdParam.ByteRead.mByteCount = c;                           /* Request to read tens of bytes of data */
        mCmdParam.ByteRead.mByteBuffer= &Buffer[0];
        s = CHRV3ByteRead( );
        count -= mCmdParam.ByteRead.mByteCount;
        for ( i=0; i!=mCmdParam.ByteRead.mByteCount; i++ ) printf( "%c ", mCmdParam.ByteRead.mByteBuffer[i] );
        if ( mCmdParam.ByteRead.mByteCount < c ) {
            printf( "\r\n" );
            printf( "�ļ��Ѿ�����\r\n" );
            break;
        }
    }
    printf( "Close\r\n" );
    i = CHRV3FileClose( );
#endif

#if MODIFY

   strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");
   i = CHRV3FileOpen();
   if( i!= ERR_SUCCESS )         return i;

   mCmdParam.Modify.mFileAttr = 0xff;
   mCmdParam.Modify.mFileTime = 0xffff;
   mCmdParam.Modify.mFileDate = MAKE_FILE_DATE(2021, 12, 20);
   mCmdParam.Modify.mFileSize = 0xffffffff;
   i = CHRV3FileModify();                                  //Modify file information
   if( i!= ERR_SUCCESS ) return i;
   printf("modify=%02x\n",i);

   mCmdParam.Close.mUpdateLen = 1 ;
   i= CHRV3FileClose();                                    //Modify file information completed
   if( i!= ERR_SUCCESS )   return i;

#endif

#if ERASE

     strcpy( (char *)&mCmdParam.Erase.mPathName,( const char * )"\\NEWFILE.TXT");
     i = CHRV3FileErase();                                 //Modify file information
     if( i!= ERR_SUCCESS )return i;
     printf("erase=%02x\n",i);
#endif
     return i;
 }




