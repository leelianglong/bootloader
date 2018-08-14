/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2012  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.14 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to Energy Micro AS whose registered office
is situated at  Sandakerveien 118, N-0484 Oslo, NORWAY solely
for  the  purposes  of  creating  libraries  for Energy Micros ARM Cortex-M3, M4F
processor-based  devices,  sublicensed  and distributed  under the terms and
conditions  of  the   End  User  License Agreement supplied by Energy Micro AS. 
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : GUI_X.C
Purpose     : Config / System dependent externals for GUI
---------------------------END-OF-HEADER------------------------------
*/

#include "GUI.h"
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_msc.h"
#include "em_gpio.h"

#if GUI_OS == 1
#include "cmsis_os.h"
#include "task.h"

osMutexDef(GUIMutex);
osMutexId hMutexGUI;
#endif

/*********************************************************************
*
*       GUI_X_GetTime()
*
* Note:
*     @brief returns system time in milisecond unit.
*     @details Depending on configuration time is returned with 1ms or 10ms resolution.
*/
#if GUI_OS == 0
int GUI_X_GetTime(void) { 
  return 0;
}

#else
int GUI_X_GetTime(void) { 
	return (((int)xTaskGetTickCount()*(1000/configTICK_RATE_HZ)));
}
#endif


/*********************************************************************
*
*       GUI_X_Delay()
*
* Note:
*     @brief is used to stop code execution for specified time
*     @param contains number of miliseconds to suspend program.
*     @details This routine could enter into EM1 mode to reduce power consumption.
*/
#if GUI_OS == 0
void GUI_X_Delay(int ms) 
{ 
  int i,j;
	
	/* Implement delay function here */
  for ( i=0; i<ms; i++ ) {
		for ( j=0; j<5000; j++ );
	}
}
#else
void GUI_X_Delay(int ms) { 
	osDelay(ms);
}
#endif

/*********************************************************************
*
*       GUI_X_Init()
*
* Note:
*     @brief is called from GUI_Init is a possibility to init
*     some hardware which needs to be up and running before the GUI.
*     If not required, leave this routine blank.
*/

void GUI_X_Init(void) { }


/*********************************************************************
*
*       GUI_X_ExecIdle
*
* Note:
*  @brief Called if WM is in idle state
*/

void GUI_X_ExecIdle(void) { GUI_X_Delay(1000); }

/*********************************************************************
*
*      Logging: OS dependent

Note:
  Logging is used in higher debug levels only. The typical target
  build does not use logging and does therefor not require any of
  the logging routines below. For a release build without logging
  the routines below may be eliminated to save some space.
  (If the linker is not function aware and eliminates unreferenced
  functions automatically)

*/

void GUI_X_Log     (const char *s) { GUI_USE_PARA(s); }
void GUI_X_Warn    (const char *s) { GUI_USE_PARA(s); }
void GUI_X_ErrorOut(const char *s) { GUI_USE_PARA(s); }

#if GUI_OS == 0
void GUI_X_InitOS(void)    { /*OS_CreateRSema(&RSema);*/    }
void GUI_X_Unlock(void)    { /*OS_Unuse(&RSema);*/ }
void GUI_X_Lock(void)      { /*OS_Use(&RSema);*/  }
U32  GUI_X_GetTaskId(void) { return 0; /*(U32)OS_GetTaskID();*/ }
#else
/*********************************************************************
*
*      Multitasking:
*
*                 GUI_X_InitOS()
*                 GUI_X_GetTaskId()
*                 GUI_X_Lock()
*                 GUI_X_Unlock()
*
* Note:
*   The following routines are required only if emWin is used in a
*   true multi task environment, which means you have more than one
*   thread using the emWin API.
*   In this case the
*                       #define GUI_OS 1
*  needs to be in GUIConf.h
*/

void GUI_X_InitOS(void)    {
   hMutexGUI = osMutexCreate(osMutex(GUIMutex));
}
void GUI_X_Unlock(void)    {
  osMutexRelease( hMutexGUI );
}
void GUI_X_Lock(void) {
    osMutexWait(hMutexGUI, 1000);
}
U32  GUI_X_GetTaskId(void) { 
    return	((U32)(osThreadGetId()));
}
#endif

/*************************** End of file ****************************/
