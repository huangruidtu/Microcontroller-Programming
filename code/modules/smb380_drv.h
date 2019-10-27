/*************************************************************************
 *
*    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2007
 *
 *    File name   : smb380_drv.c
 *    Description : SMB380 acceleration sensor driver include file
 *
 *    History :
 *    1. Date        : 13, February 2008
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 *
 *    $Revision: 28 $
 **************************************************************************/
#include "board.h"
#include "i2c0_drv.h"

#ifndef __SMB380_DRV_H
#define __SMB380_DRV_H

#define SMB380_ADDR   0x38

/*#define SMD380_READ   0x71
#define SMD380_WRITE  0x70*/

#define SMB380_CHIP_ID    0x00
#define SMB380_ACCX_ADDR  0x02

typedef enum _SMB380_Status_t
{
  SMB380_PASS = 0,
} SMB380_Status_t;

#pragma pack(1)
typedef struct _SMB380_Data_t
{
  Int16S AccX;
  Int16S AccY;
  Int16S AccZ;
  Int8U Temp;
} SMB380_Data_t, *pSMB380_Data_t;
#pragma pack()

typedef enum _SMB380_Range_t
{
  SMB380_2G = 0, SMB380_4G, SMB380_8G
} SMB380_Range_t;

typedef enum _SMB380_Bandwidth_t
{
  SMB380_25HZ = 0, SMB380_50HZ, SMB380_100HZ, SMB380_190HZ,
  SMB380_375HZ, SMB380_750HZ, SMB380_1500HZ
} SMB380_Bandwidth_t;

/*************************************************************************
 * Function Name: SMB380_Init
 * Parameters: none
 *
 * Return: SMB380_Status_t
 *
 * Description: SMB380 init
 *
 *************************************************************************/
SMB380_Status_t SMB380_Init(void);

/*************************************************************************
 * Function Name: SMB380_GetID
 * Parameters: none
 *
 * Return: SMB380_Status_t
 *
 * Description: SMB380 get chip ID and revision
 *
 *************************************************************************/
SMB380_Status_t SMB380_GetID (pInt8U pChipId, pInt8U pRevision);

/*************************************************************************
 * Function Name: SMB380_GetData
 * Parameters: none
 *
 * Return: SMB380_Status_t
 *
 * Description:
 *
 *************************************************************************/
SMB380_Status_t SMB380_GetData (pSMB380_Data_t pData);


#endif // __SMB380_DRV_H
