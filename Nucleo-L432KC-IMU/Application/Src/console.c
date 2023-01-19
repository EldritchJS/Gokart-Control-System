#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "appUSART.h"
#include "appLSM9DS1.h"
#include "memsTask.h"
#include "console.h"

#define HEADER_STR "Quaternion0,Quaternion1,Quaternion2,Quaternion3,Rotationn0,Rotation1,Rotation2,Gravity0,Gravity1,Gravity2,LinearAcceleration0,LinearAcceleration1,LinearAcceleration2,Heading,HeadingError"

static char lineBuffer[1024];
static char outBuffer[1024];

static uint8_t lineIndex = 0;
static uint8_t streamActiveFlag = 0;

static void processLine(void);
static void consoleTimerCallback(void const *arg);
static osTimerId consoleTimer;
static osStaticTimerDef_t consoleTimerCB;

osTimerStaticDef(ConsoleTimer, consoleTimerCallback, &consoleTimerCB);

static void consoleTimerCallback(void const *arg)
{
  (void) arg;

  osMessagePut(MEMSTaskRXEventQueue, 0, 0);
}

static void processLine(void)
{
  switch(lineBuffer[0])
  {
    case 'B':
    	if(streamActiveFlag==0)
    	{
    	  streamActiveFlag=1;
    	  sprintf(outBuffer, "%s\r\n", HEADER_STR);
    	  USART1TxStr(outBuffer);
    	  osTimerStart(consoleTimer, 10);
    	}
	    break;
    case 'E':
    	if(streamActiveFlag==1)
    	{
      	  streamActiveFlag=0;
    	  osTimerStop(consoleTimer);
    	}
    	break;
    case 'V':
    	if(streamActiveFlag==0)
    	{
    	  sprintf(outBuffer, "Version 0.1.13\r\n");
          USART1TxStr(outBuffer);
    	}
    	break;
    case '?':
    	if(streamActiveFlag==0)
    	{
    	  sprintf(outBuffer, "Menu\r\nB - Begin Streaming\r\nE - End Streaming\r\nV - Print Version\r\n? - This menu\r\n");
          USART1TxStr(outBuffer);
    	}
    	break;
    default:
    	if(streamActiveFlag==0)
    	{
    	  sprintf(outBuffer, "Unknown command: %c",lineBuffer[0]);
          USART1TxStr(outBuffer);
    	}
    	break;
  }
}

void console(void)
{
  uint8_t data;

  consoleTimer = osTimerCreate(osTimer(ConsoleTimer), osTimerPeriodic, NULL);
  IMUInit();
  for(;;)
  {
	USART1RxDataWait();
    while(USART1Rx(&data))
    {
      if((data=='\n')||(data=='\r'))
      {
    	lineBuffer[lineIndex]='\0';
        processLine();
        lineIndex=0;
      }
      else
      {
        lineBuffer[lineIndex]=data;
        lineIndex++;
      }
    }
    osDelay(1);
  }
}
