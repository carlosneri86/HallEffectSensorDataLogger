/*HEADER******************************************************************************************
* File name: DataLogger.c
* Date: Jan 1, 2016
* Author: B22385
*
**END********************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "DataLogger.h"
#include "fsl_rtc_driver.h"
#include "diskio.h"
#include "ff.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Defines & Macros Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Typedef Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Function Prototypes Section
///////////////////////////////////////////////////////////////////////////////////////////////////

static bool DataLogger_IsCardPresent(void);

static void DataLogger_RtcConfig(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Constants Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Constants Section
///////////////////////////////////////////////////////////////////////////////////////////////////
static const uint8_t DataLogger_EndOfLine[] =
{
		"\r\n"
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Variables Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section
///////////////////////////////////////////////////////////////////////////////////////////////////
static rtc_datetime_t DataLogger_RTCDate;

static FATFS DataLogger_FatFs;

static int8_t DataLogger_LogMessage[DATALOGGER_MAX_LOG_MESSAGE];
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section
///////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t DataLogger_Init(void)
{
	FRESULT FileSystemStatus;
	DRESULT DiskInitStatus;
	uint32_t DataLoggerInitStatus = DATA_LOGGER_ERROR;

	/* Initialize the SDCard detect pin */
	/* The data logger assumes */
	/* the SDCard is present an boot and */
	/* does not support hot plug */
	GPIO_DRV_Init(DATALOGGER_PIN_DETECT, NULL);

	/* Initialize SD Card detect input pin */
	GPIO_DRV_InputPinInit(&DATALOGGER_PIN_DETECT[0]);

	if(!DataLogger_IsCardPresent())
	{
		/* Wait for SD Card insertion */
		while (!DataLogger_IsCardPresent());
	}

	/* Once the SDCard is detected, start FS */
	DiskInitStatus = disk_initialize(SD);

	if(DiskInitStatus == RES_OK)
	{
		/* Select current logical device driver (0 = USB, 1 = SD) */
		FileSystemStatus = f_chdrive(SD);

		FileSystemStatus |= f_mount(SD, &DataLogger_FatFs);

		if(FileSystemStatus == FR_OK)
		{

			/* Now start the RTC */
			DataLogger_RtcConfig();
			DataLoggerInitStatus = DATA_LOGGER_OK;
		}
	}


	return(DataLoggerInitStatus);
}

uint32_t DataLogger_PostEvent(uint8_t * pLogMessage, uint16_t *pLogData, uint16_t LogDataSize)
{
	uint32_t	BytesWritten;
	FIL			LogFile;
	uint32_t	FileNewLine;
	FRESULT 	FileSystemStatus;
	uint32_t	MessageSize;
	uint16_t	MessageOffset = 0;
	uint32_t	PostEventStatus = DATA_LOGGER_ERROR;

	/* TODO: change strcat and sprintf with custom ones */

	/* check if the card still there*/
	if(DataLogger_IsCardPresent())
	{
		/* Open the file, move to the last line and log the new entry*/
		FileSystemStatus = f_open(&LogFile, DATALOGGER_FILE_NAME, FA_WRITE | FA_OPEN_ALWAYS);

		if(FileSystemStatus == FR_OK)
		{
			FileNewLine = f_size(&LogFile);

			/* Set file pointer to the start of new line in text file */
			FileSystemStatus = f_lseek(&LogFile, FileNewLine);

			if(FileSystemStatus == FR_OK)
			{
				/* get the current time stamp and fill the buffer*/
				RTC_DRV_GetDatetime(0,&DataLogger_RTCDate);

				memset(&DataLogger_LogMessage[0],0,DATALOGGER_MAX_LOG_MESSAGE);

				sprintf((char*)&DataLogger_LogMessage[0],"%.2d-%.2d-%d, %.2d:%.2d:%.2d, ",DataLogger_RTCDate.day,\
							DataLogger_RTCDate.month,DataLogger_RTCDate.year,DataLogger_RTCDate.hour,\
								DataLogger_RTCDate.minute,DataLogger_RTCDate.second);

				/* Add the application message if available*/
				if(pLogMessage != NULL)
				{
					strcat((char*)&DataLogger_LogMessage[0], (const char*)pLogMessage);
				}
				/* Add application data if available */
				if(LogDataSize)
				{
					/* Calculate the current string size*/
					MessageSize = strlen((const char *)&DataLogger_LogMessage[0]);
					/* Parse the data and add it to the message buffer */
					while(LogDataSize--)
					{
						sprintf((char*)&DataLogger_LogMessage[MessageSize],"%d,",pLogData[MessageOffset]);
						MessageSize = strlen((const char *)&DataLogger_LogMessage[0]);
						MessageOffset++;
					}
				}
				/* Set the end of line */
				strcat((char*)&DataLogger_LogMessage[0],(const char *)&DataLogger_EndOfLine[0]);
				MessageSize = strlen((const char *)&DataLogger_LogMessage[0]);
				/* Write the file */
				FileSystemStatus = f_write(&LogFile,&DataLogger_LogMessage[0], MessageSize, &BytesWritten);

				/* Close the log file */
				FileSystemStatus = f_close(&LogFile);

				PostEventStatus = DATA_LOGGER_OK;
			}
		}
	}
	return(PostEventStatus);
}

static bool DataLogger_IsCardPresent(void)
{
	uint32_t SDCardDetectPin;
	bool IsPresent = true;

	SDCardDetectPin = GPIO_DRV_ReadPinInput(DATALOGGER_PIN_DETECT_NAME);

	if(!SDCardDetectPin)
	{
		IsPresent = false;
	}

	return (IsPresent);
}

void DataLogger_RtcConfig(void)
{
    RTC_DRV_Init(0);


    // Set a start date time and start RTC
    DataLogger_RTCDate.year = 2016U;
    DataLogger_RTCDate.month = 1U;
    DataLogger_RTCDate.day = 1U;
    DataLogger_RTCDate.hour = 22U;
    DataLogger_RTCDate.minute = 05U;
    DataLogger_RTCDate.second = 0U;

    RTC_DRV_SetDatetime(0, &DataLogger_RTCDate);
}
