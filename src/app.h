/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "configuration.h"
#include "peripheral/sercom/spi_master/plib_sercom1_spi_master.h"
#include "system/console/sys_console.h"
#include "system/command/sys_command.h"
#include "peripheral/port/plib_port.h"
#include "peripheral/rtc/plib_rtc.h"
#include "packs/ATSAME51J20A_DFP/same51j20a.h"



#define ESC_CLEAR_TERMINAL          "\033[2J"
#define ESC_CURSOR_X1Y1             "\033[1;1H"
#define ESC_HIDE_CURSOR             "\033[?25l"
#define ESC_CLEAR_LINE              "\033[2K"
#define ESC_RESETCOLOR              "\033[0m"
#define ESC_GREEN                   "\033[0;32m"
#define ESC_RED                     "\033[0;31m"
#define ESC_YELLOW                  "\033[1;33m"
#define ESC_BLUE                    "\033[0;36m"

#define MAX_REG_SIZE 255
#define MAX_CONFIG_SIZE 255

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/
    
    
    
    
extern char APP_Register_Buffer[MAX_REG_SIZE];
extern char APP_Config_Buffer[MAX_CONFIG_SIZE];

typedef enum
{
    /* Application's state machine's initial state. */
    APP_STATE_INITIALIZE,
    APP_STATE_TRANSFER_COMPLETE_WAIT,    
    APP_STATE_WRITE_DATA,        
    APP_STATE_SEND_READ_DATA_CMD,
    APP_STATE_READ_DATA,
    APP_STATE_VERIFY,
    APP_STATE_ERROR,
    /* TODO: Define states used by the application state machine. */

} APP_STATES;

struct ADCvariable {
    uint16_t adc_count;
    float input_voltage;
    bool adc;
    uint8_t ADCsendBuffer[32 + 1];
};


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;

    /* TODO: Define any additional data used by the application. */

} APP_DATA;




// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_SPICallBack(uintptr_t contextHandle);

void SPIEventHandler(uintptr_t context);

void APP_Initialize ( void );

void ADC_cmd_READ();
bool APP_AddCommandFunction();
static void rtcEventHandler (RTC_TIMER32_INT_MASK intCause, uintptr_t context);


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_H */

/*******************************************************************************
 End of File
 */

