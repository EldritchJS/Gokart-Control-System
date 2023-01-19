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

void beginStreaming(void);
void endStreaming(void);
void printVersion(void);
static void printMenu(void);

typedef struct
{
  void    (*cmd)(void);
  char*   cmdName;
  char*   cmdInfo;
} CONSOLE_MENU_CMDS;

static CONSOLE_MENU_CMDS menuCmds[] = {
		{beginStreaming, "B", "Begin streaming if not already started"},
		{endStreaming, "E", "End streaming if running"},
		{printVersion, "V", "Print version info"},
};

static uint32_t numCmds;

static void consoleTimerCallback(void const *arg)
{
  (void) arg;

  osMessagePut(MEMSTaskRXEventQueue, 0, 0);
}

void beginStreaming(void)
{
  if(streamActiveFlag==0)
  {
    streamActiveFlag=1;
	sprintf(outBuffer, "%s\r\n", HEADER_STR);
	USART1TxStr(outBuffer);
	osTimerStart(consoleTimer, 10);
  }
}

void endStreaming(void)
{
  if(streamActiveFlag==1)
  {
    streamActiveFlag=0;
	osTimerStop(consoleTimer);
  }
}

void printVersion(void)
{
  if(streamActiveFlag==0)
  {
    sprintf(outBuffer, "Version: %s\r\n", VERSION_STR);
    USART1TxStr(outBuffer);
  }
}

static void printMenu(void)
{
  if(streamActiveFlag==0)
  {
	for(int i=0; i<numCmds; i++)
	{
	  sprintf(outBuffer, "    %-26s - %s\r\n",menuCmds[i].cmdName, menuCmds[i].cmdInfo);
	  USART1TxStr(outBuffer);
	}
    USART1TxStr("    '?' <ENTER> for help\r\n");
  }
}

static void processLine(void)
{
  int i;
  for(i=0; i<numCmds; i++)
  {
    if(lineBuffer[0]==menuCmds[i].cmdName[0])
    {
      menuCmds[i].cmd();
      break;
    }
    else if(lineBuffer[i] == '?')
    {
      printMenu();
      break;
    }
  }

  if(i==numCmds)
  {
	sprintf(outBuffer, "Unknown command: <%c>\r\n", lineBuffer[0]);
	USART1TxStr(outBuffer);
  }
}

void console(void)
{
  uint8_t data;

  numCmds = sizeof(menuCmds)/sizeof(CONSOLE_MENU_CMDS);
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
  }
}
