/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include <string.h>


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************


#define ESC_CLEAR_TERMINAL          "\033[2J"
#define ESC_CURSOR_X1Y1             "\033[1;1H"
#define ESC_HIDE_CURSOR             "\033[?25l"
#define ESC_CLEAR_LINE              "\033[2K"
#define ESC_RESETCOLOR              "\033[0m"
#define ESC_GREEN                   "\033[0;32m"
#define ESC_RED                     "\033[0;31m"
#define ESC_YELLOW                  "\033[1;33m"
#define ESC_BLUE                    "\033[0;36m"


/*
 * Protocol:
 * Master:     [WR_CMD ADDR1 ADDR0 DATA0 DATA1 .. DATAN] ... ... ... ... ... ... [RD_CMD ADDR1 ADDR0] [DUMMY DUMMY .. DUMMY]
 * Slave:      [DUMMY  DUMMY DUMMY DUMMY DUMMY .. DUMMY] ... ... ... ... ... ... [DUMMY  DUMMY DUMMY] [DATA0 DATA1 .. DATAN] 
 * BUSY PIN:    ----------------------------------------------------------------
 * ____________|                                                                |____________________________________________
 * 
 * WR_CMD = 0x02
 * RD_CMD = 0x03
 * BUSY   = 0x01
 * READY  = 0x00
 */


int i=0;






int main(void) {

    /* Initialize all modules */
    SYS_Initialize(NULL);
    
    

    
    SYS_CONSOLE_PRINT("\r\n" \
            ESC_YELLOW \
            "=== SAME51J20A SPI configuration: "\
            " (" \
          __DATE__ " " __TIME__ ") ===" ESC_RESETCOLOR "\r\n");
    

    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE);
}


/*******************************************************************************
 End of File
 */

