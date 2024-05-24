/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "math.h"

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

//      Register name                       ADDR | no. of bits
#define APP_ADC_READ_ADCDATA                0x41
#define APP_ADC_OFFSETCALCODE               0x106E84
#define MASK(j) (1<<j)
//----------------------Commands Config.-----------------------// 
#define APP_CMD_DEVICE                      0x1
#define APP_CMD_WRITE                       0x2 
#define APP_CMD_READ                        0x1
//-----------------------Fast commands------------------------// 
#define APP_CMD_STANDBY                     0x6C
#define APP_CMD_SHUTDOWN                    0x70
#define APP_CMD_DEFAULT                     0x78
#define APP_CMD_CONVERSION                  0x68
//----------------------Registers address----------------------// 
#define APP_CMD_ADCDATA                     0x0 // 4/24/32 bites
char ADCDATA[15] = "0x0";
#define APP_CMD_CONFIG0                     0x1 // 8 bits
char CONFIG0[15] = "0x1";
#define APP_CMD_CONFIG1                     0x2 // 8 bits
char CONFIG1[15] = "0x2";
#define APP_CMD_CONFIG2                     0x3 // 8 bits
char CONFIG2[15] = "0x3";
#define APP_CMD_CONFIG3                     0x4 // 8 bits
char CONFIG3[15] = "0x4";
#define APP_CMD_IRQ                         0x5 // 8 bits
char IRQ[15] = "0x5";
#define APP_CMD_MUX                         0x6 // 8 bits
char MUX[15] = "0x6";
#define APP_CMD_SCAN                        0x7 // 24 bits
char SCAN[15] = "0x7";
#define APP_CMD_TIMER                       0x8 // 24 bits
char TIMER[15] = "0x8";
#define APP_CMD_OFFSETCAL                   0x9 // 24 bits
char OFFSETCAL[15] = "0x9";
#define APP_CMD_GAINCAL                      0xA // 24 bits
char GAINCAL[15] = "0xA";
#define APP_CMD_RESERVED0                   0xB // 24 bits
#define APP_CMD_RESERVED1                   0xC // 8 bits
#define APP_CMD_LOCK                        0xD // 8 bits
#define APP_CMD_RESERVED2                   0xE // 16 bits
#define APP_CMD_CRCCFG                      0xF // 16 bits
#define LINE_TERM                           "\r\n"          // line terminator

//----------------------SPI config.----------------------// 
#define APP_RX_BUFFER_SIZE                  256
#define APP_TX_BUFFER_SIZE                  256
#define APP_MEM_ADDR                        0x0010
#define APP_TEST_DATA                       "hello"
#define APP_TEST_DATA_SIZE                  strlen(APP_TEST_DATA)

SPI_TRANSFER_SETUP setup; //Structure

char APP_Register_Buffer[MAX_REG_SIZE];
char APP_Config_Buffer[MAX_CONFIG_SIZE];

//----------------------miscellaneous ----------------------// 
#define LED_On()                            LED_Clear()
#define LED_Off()                           LED_Set()
#define ref_voltage                (3.3f)
int32_t ADCval[2];
uint8_t rxData[APP_RX_BUFFER_SIZE];
uint8_t txData[APP_TX_BUFFER_SIZE];

APP_DATA appData;

APP_STATES nextState;
volatile bool isTransferDone = false;
bool ADCcycle = true;
bool configwrite;
bool isRTCTimerExpired;
bool ADC_IRQ;
bool once = true;

float resoloution = 8388608;



int ret;


//----------------------Command functions prototypes----------------------// 
static void _APP_Commands_ADC(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_REGISTERs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_READ_REG(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_WRITE_REG(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_SINGLE(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_SCAN(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_CONVERT(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_STANDBY(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_SHUTDOWN(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_DEFAULT(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_about(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

bool _APP_Commands_SCAN_mode();



//----------------------Commands----------------------// 
static const SYS_CMD_DESCRIPTOR appCmdTbl[] = {/* CMD            FUNCTION                            DESCRIPTION                  */
    {"ADC", _APP_Commands_ADC, "       : View all ADC configuration commands"},
    {"REGISTERs", _APP_Commands_REGISTERs, " : View all accessible internal ADC registers"},
    {"WRITE", _APP_Commands_WRITE_REG, "     : Write the specified register"},
    {"READ", _APP_Commands_READ_REG, "      : Read the specified register"},
    {"SINGLE", _APP_Commands_SINGLE, "    : Get a single conversion on the specified channel"},
    {"SCAN", _APP_Commands_SCAN, ": Get continuous conversion on the specified channel"},
    {"CONVERT", _APP_Commands_CONVERT, "   : ADC Conversion Start/Restart Fast Command"},
    {"STANDBY", _APP_Commands_STANDBY, "   : ADC Standby Mode Fast Command"},
    {"SHUTDOWN", _APP_Commands_SHUTDOWN, "  : ADC Shutdown Mode Fast Command"},
    {"DEFAULT", _APP_Commands_DEFAULT, "   : ADC Full Reset Fast Command"},
    {"about", _APP_Commands_about, "     : About the software/hardware"},
};

//----------------------Commands Initialization----------------------// 

bool APP_AddCommandFunction() {



    if (!SYS_CMD_ADDGRP(appCmdTbl, sizeof (appCmdTbl) / sizeof (*appCmdTbl), "ADC", ": ADC configuration commands")) //specifies the command group
    {
        return false;
    }

    return true;
}

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INITIALIZE;

    if (APP_AddCommandFunction()) {
        SYS_CONSOLE_PRINT(ESC_GREEN "Device booted correctly!" ESC_RESETCOLOR "\r\n");
    } else {
        SYS_CONSOLE_PRINT(ESC_RED "Error! --> ADC commands failed to initialize" ESC_RESETCOLOR "\r\n");

        /* TODO: Initialize your application's state machine and other
         * parameters.
         */

    }

}

void EIC_Pin14Callback(uintptr_t context) {
    // This means an interrupt condition has been sensed on EIC Pin 27.

    ADC_IRQ = true;
}

static void rtcEventHandler(RTC_TIMER32_INT_MASK intCause, uintptr_t context) {
    if (intCause & RTC_TIMER32_INT_MASK_CMP0) {
        isRTCTimerExpired = true;
    } else {
        isRTCTimerExpired = false;
    }
}



//----------------------Commands function----------------------// 

static void _APP_Commands_ADC(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    uint32_t ix;
    const SYS_CMD_DESCRIPTOR* pDcpt;
    const void* cmdIoParam = pCmdIO->cmdIoParam;


    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "---------- ADC configuration commands ----------");

    ix = 0;
    pDcpt = appCmdTbl;
    while (ix < (sizeof (appCmdTbl) / sizeof (*appCmdTbl))) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdStr);
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdDescr);
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
        ix++;
        pDcpt++;
    }

    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM);
}

void _APP_Commands_REGISTERs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    static const SYS_CMD_DESCRIPTOR appCmdTb2[] = {
        /* CMD            FUNCTION                            DESCRIPTION                  */
        {"0x0 ---- ADCDATA", _APP_Commands_REGISTERs, "   ---- 4/24/32 "},
        {"0x1 ---- CONFIG0", _APP_Commands_REGISTERs, "   ---- 8 "},
        {"0x2 ---- CONFIG1", _APP_Commands_REGISTERs, "   ---- 8 "},
        {"0x3 ---- CONFIG2", _APP_Commands_REGISTERs, "   ---- 8 "},
        {"0x4 ---- CONFIG3", _APP_Commands_REGISTERs, "   ---- 8 "},
        {"0x5 ---- IRQ", _APP_Commands_REGISTERs, "       ---- 8 "},
        {"0x6 ---- MUX", _APP_Commands_REGISTERs, "       ---- 8 "},
        {"0x7 ---- SCAN", _APP_Commands_REGISTERs, "      ---- 24 "},
        {"0x8 ---- TIMER", _APP_Commands_REGISTERs, "     ---- 24 "},
        {"0x9 ---- OFFSETCAL", _APP_Commands_REGISTERs, " ---- 24 "},
        {"0xA ---- GAINCAL", _APP_Commands_REGISTERs, "   ---- 24 "},
    };


    uint32_t ix;
    const SYS_CMD_DESCRIPTOR* pDcpt;
    const void* cmdIoParam = pCmdIO->cmdIoParam;


    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "----------- ADC internal registers ------------");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " --- Addr. -- Register -- No. of Bits ---");

    ix = 0;
    pDcpt = appCmdTb2;
    while (ix < (sizeof (appCmdTb2) / sizeof (*appCmdTb2))) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdStr);
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdDescr);
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
        ix++;
        pDcpt++;
    }

    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM);




}
/******************************************************************************/



//************Read register function************// 

void _APP_Commands_READ_REG(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: READ <register addr>\r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: READ 0x1\r\n");
        return;
    }
    strcpy(APP_Register_Buffer, argv[1]);


    if (strncmp(APP_Register_Buffer, ADCDATA, 5) == 0) {
        struct ADCvariable getvalue2;
        SYS_CONSOLE_MESSAGE("ADCDATA\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_ADCDATA;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        // SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 4)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        ADCval[2] = ((int32_t) rxData[3]);
        ADCval[2] = ADCval[2] | ((int32_t) (rxData[2]) << 8);
        ADCval[2] = ADCval[2] | ((int32_t) (rxData[1]) << 16);
        getvalue2.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

        SYS_CONSOLE_PRINT(ESC_GREEN "ADC voltage = %d.%06d V \r\n" ESC_RESETCOLOR, (int) getvalue2.input_voltage, (int) ((getvalue2.input_voltage - (int) getvalue2.input_voltage)*1000000.0));

    } else if (strncmp(APP_Register_Buffer, CONFIG0, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: CONFIG0\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_CONFIG0;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 2)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("CONFIG0 contains: 0x%X\r\n", rxData[1]);

    } else if (strncmp(APP_Register_Buffer, CONFIG1, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: CONFIG1\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_CONFIG1;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 2)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("CONFIG1 contains: 0x%X\r\n", rxData[1]);
    } else if (strncmp(APP_Register_Buffer, CONFIG2, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: CONFIG2\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_CONFIG2;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 2)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("CONFIG2: contains: 0x%X\r\n", rxData[1]);
    } else if (strncmp(APP_Register_Buffer, CONFIG3, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: CONFIG3\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_CONFIG3;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        // SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 2)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("CONFIG3 contains: 0x%X\r\n", rxData[1]);
    } else if (strncmp(APP_Register_Buffer, IRQ, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: IRQ\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_IRQ;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        // SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 2)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("IRQ contains: 0x%X\r\n", rxData[1]);
    } else if (strncmp(APP_Register_Buffer, MUX, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: MUX\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_MUX;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        // SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 2)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("MUX contains: 0x%X\r\n", rxData[1]);
    } else if (strncmp(APP_Register_Buffer, SCAN, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: SCAN\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_SCAN;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 4)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("SCAN contains: 0x%X\r\n", rxData[3]);
    } else if (strncmp(APP_Register_Buffer, TIMER, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: TIMER\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_TIMER;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 4)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("TIMER contains: 0x%X\r\n", rxData[3]);
    } else if (strncmp(APP_Register_Buffer, OFFSETCAL, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: OFFSETCAL\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_OFFSETCAL;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        // SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL); This will automatically pull CS line up after transfer.

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 4)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        ADCval[2] = ((int32_t) rxData[3]);
        ADCval[2] = ADCval[2] | ((int32_t) (rxData[2]) << 8);
        ADCval[2] = ADCval[2] | ((int32_t) (rxData[1]) << 16);

        SYS_CONSOLE_PRINT("OFFSETCAL contains: 0x%X\r\n", ADCval[2]);
    } else if (strncmp(APP_Register_Buffer, GAINCAL, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Reading: GAINCAL\r\n");
        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_GAINCAL;
        txData[3] = APP_CMD_READ;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, rxData, 4)) {

        }
        while (SERCOM1_SPI_IsBusy() == true);

        SYS_CONSOLE_PRINT("GAINCAL contains: 0x%x\r\n", rxData[3]);
    } else {
        SYS_CONSOLE_MESSAGE("Error! Invalid Register\r\n");
    }


    SPI_CS_Set();

}



//************Write register function************// 

void _APP_Commands_WRITE_REG(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    SPI_CS_Set();


    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 3) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: WRITE <register addr> <config>\r\n");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Ex: WRITE 0x46 0xE3\r\n");
        return;
    }

    strcpy(APP_Register_Buffer, argv[1]);
    strcpy(APP_Config_Buffer, argv[2]);
    SYS_CONSOLE_PRINT("Configuration to register: %x\r\n", *APP_Config_Buffer);



    if (strncmp(APP_Register_Buffer, CONFIG0, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Write: CONFIG0\r\n");

        txData[5] = *APP_Config_Buffer;

        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_CONFIG0;
        txData[3] = APP_CMD_WRITE;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_Write(&txData[4], 1)) {

        } else {
            SPI_CS_Clear();
        }
        RTC_Timer32Start();
        /*
        
        
        while (!isRTCTimerExpired == true){
            RTC_Timer32CallbackRegister(rtcEventHandler, 0);
        }
        
         */
        while (SERCOM1_SPI_IsTransmitterBusy() == true);

        SPI_CS_Clear();
        if (SERCOM1_SPI_Write(&txData[5], 1)) {

        }
        while (SERCOM1_SPI_IsTransmitterBusy() == true);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

    } else if (strncmp(APP_Register_Buffer, CONFIG3, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Write: CONFIG3\r\n");

        txData[5] = 0xC0;

        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_CONFIG3;
        txData[3] = APP_CMD_WRITE;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_Write(&txData[4], 1)) {

        } else {
            SPI_CS_Clear();
        }
        while (SERCOM1_SPI_IsTransmitterBusy() == true);
        SPI_CS_Clear();
        if (SERCOM1_SPI_Write(&txData[5], 1)) {

        }
        while (SERCOM1_SPI_IsTransmitterBusy() == true);

    } else if (strncmp(APP_Register_Buffer, CONFIG3, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Write: CONFIG3\r\n");

        txData[5] = 0xC0;

        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_CONFIG3;
        txData[3] = APP_CMD_WRITE;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_Write(&txData[4], 1)) {

        } else {
            SPI_CS_Clear();
        }
        while (SERCOM1_SPI_IsTransmitterBusy() == true);
        SPI_CS_Clear();
        if (SERCOM1_SPI_Write(&txData[5], 1)) {

        }
        while (SERCOM1_SPI_IsTransmitterBusy() == true);

    } else if (strncmp(APP_Register_Buffer, OFFSETCAL, 5) == 0) {
        SYS_CONSOLE_MESSAGE("Write: OFFSETCAL\r\n");

        uint32_t test = APP_ADC_OFFSETCALCODE;

        txData[1] = APP_CMD_DEVICE;
        txData[2] = APP_CMD_OFFSETCAL;
        txData[3] = APP_CMD_WRITE;

        txData[4] = ((uint8_t) txData[3]);
        txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
        txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);
        SYS_CONSOLE_PRINT("Sending data: 0x%x\r\n", txData[4]);
        SYS_CONSOLE_PRINT("Config. data: 0x%x\r\n", test);
        //SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[4], 1, NULL, 0)) {
            //while (SERCOM1_SPI_IsTransmitterBusy() == true);
        } else {
            SPI_CS_Clear();
        }

        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&test, 4, NULL, 0)) {
        }
    } else {

    }

    SPI_CS_Set();

}
/******************************************************************************/







//************Fast command function's************// 

void _APP_Commands_SINGLE(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    rxData[0] = 0;
    rxData[1] = 0;
    rxData[2] = 0;
    rxData[3] = 0;
    rxData[4] = 0;


    txData[1] = APP_CMD_CONVERSION;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);
    struct ADCvariable getvalue1;


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[1], 1)) {
        SYS_CONSOLE_MESSAGE("Initializing ADC single-shot...\r\n");
    } else {
        SYS_CONSOLE_MESSAGE("Error initializing ADC single-shot!\r\n");
        exit(0);
    }

    while (SERCOM1_SPI_IsTransmitterBusy() == true);


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);


    rxData[0] = 0;
    rxData[1] = 0;
    rxData[2] = 0;
    rxData[3] = 0;
    rxData[4] = 0;

    txData[1] = APP_ADC_READ_ADCDATA;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);
    SPI_CS_Clear();
    if (SERCOM1_SPI_WriteRead(&txData[1], 1, rxData, 4)) {
        SYS_CONSOLE_MESSAGE("Reading single-shot conversion...\r\n");
    }
    while (SERCOM1_SPI_IsBusy() == true);

    ADCval[2] = ((int32_t) rxData[3]);
    ADCval[2] = ADCval[2] | ((int32_t) (rxData[2]) << 8);
    ADCval[2] = ADCval[2] | ((int32_t) (rxData[1]) << 16);


    SYS_CONSOLE_PRINT("Receiving: 0x%x\r\n", ADCval[2]);
    SYS_CONSOLE_PRINT("Receiving: %d\r\n", (int) ADCval[2]);
    if (getbit(ADCval[2], 23)) {
        ADCval[2] = ADCval[2] | (255U << 24);
        SYS_CONSOLE_MESSAGE("Reading a negative number -#\r\n");
        SYS_CONSOLE_PRINT("Receiving: %x\r\n", ADCval[2]);
        getvalue1.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

        SYS_CONSOLE_PRINT(ESC_GREEN "ADC voltage = %d.%05d V \r\n" ESC_RESETCOLOR, (int) getvalue1.input_voltage, ~(int) ((getvalue1.input_voltage - (int) getvalue1.input_voltage)*100000.0));
    } else {
        getvalue1.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

        SYS_CONSOLE_PRINT(ESC_GREEN "ADC voltage = %d.%05d V \r\n" ESC_RESETCOLOR, (int) getvalue1.input_voltage, (int) ((getvalue1.input_voltage - (int) getvalue1.input_voltage)*100000.0));
    }

    while (!SPI_INT2_Set());

    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[1], 1)) {
        SYS_CONSOLE_MESSAGE("Initializing ADC single-shot...\r\n");
    } else {
        SYS_CONSOLE_MESSAGE("Error initializing ADC single-shot!\r\n");
        exit(0);
    }

    while (SERCOM1_SPI_IsTransmitterBusy() == true);


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);


    rxData[0] = 0;
    rxData[1] = 0;
    rxData[2] = 0;
    rxData[3] = 0;
    rxData[4] = 0;

    txData[1] = APP_ADC_READ_ADCDATA;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);
    SPI_CS_Clear();
    if (SERCOM1_SPI_WriteRead(&txData[1], 1, rxData, 4)) {
        SYS_CONSOLE_MESSAGE("Reading single-shot conversion...\r\n");
    }
    while (SERCOM1_SPI_IsBusy() == true);

    ADCval[2] = ((int32_t) rxData[3]);
    ADCval[2] = ADCval[2] | ((int32_t) (rxData[2]) << 8);
    ADCval[2] = ADCval[2] | ((int32_t) (rxData[1]) << 16);


    SYS_CONSOLE_PRINT("Receiving: 0x%x\r\n", ADCval[2]);
    SYS_CONSOLE_PRINT("Receiving: %d\r\n", (int) ADCval[2]);
    if (getbit(ADCval[2], 23)) {
        ADCval[2] = ADCval[2] | (255U << 24);
        SYS_CONSOLE_MESSAGE("Reading a negative number -#\r\n");
        SYS_CONSOLE_PRINT("Receiving: %x\r\n", ADCval[2]);
        getvalue1.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

        SYS_CONSOLE_PRINT(ESC_GREEN "ADC voltage = %d.%05d V \r\n" ESC_RESETCOLOR, (int) getvalue1.input_voltage, ~(int) ((getvalue1.input_voltage - (int) getvalue1.input_voltage)*100000.0));
    } else {
        getvalue1.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

        SYS_CONSOLE_PRINT(ESC_GREEN "ADC voltage = %d.%05d V \r\n" ESC_RESETCOLOR, (int) getvalue1.input_voltage, (int) ((getvalue1.input_voltage - (int) getvalue1.input_voltage)*100000.0));
    }

    SYS_CONSOLE_MESSAGE("ADC going into SleepMode state! \r\n");
}

static void _APP_Commands_SCAN(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {

    int i = 0;

    if (once) {
        once = false;
        _APP_Commands_SCAN_mode();
    } else {

    }




    for (i = 0; i < 50; i++) {
        struct ADCvariable getvalue1;

        rxData[0] = 0;
        rxData[1] = 0;
        rxData[2] = 0;
        rxData[3] = 0;
        rxData[4] = 0;

        txData[1] = APP_ADC_READ_ADCDATA;
        //SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);
        SPI_CS_Clear();
        if (SERCOM1_SPI_WriteRead(&txData[1], 1, rxData, 4)) {
            //SYS_CONSOLE_MESSAGE("Reading single-shot conversion...\r\n");
        }
        while (SERCOM1_SPI_IsBusy() == true);

        ADCval[2] = ((int32_t) rxData[3]);
        ADCval[2] = ADCval[2] | ((int32_t) (rxData[2]) << 8);
        ADCval[2] = ADCval[2] | ((int32_t) (rxData[1]) << 16);


        //SYS_CONSOLE_PRINT("Receiving: 0x%x\r\n", ADCval[2]);
       // SYS_CONSOLE_PRINT("Receiving: %d\r\n", (int) ADCval[2]);
        if (getbit(ADCval[2], 23)) {
            ADCval[2] = ADCval[2] | (255U << 24);
            //SYS_CONSOLE_MESSAGE("Reading a negative number -#\r\n");
            //SYS_CONSOLE_PRINT("Receiving: %x\r\n", ADCval[2]);
            getvalue1.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

            SYS_CONSOLE_PRINT(ESC_GREEN "ADC voltage = %d,%05d V \r\n " ESC_RESETCOLOR, (int) getvalue1.input_voltage, ~(int) ((getvalue1.input_voltage - (int) getvalue1.input_voltage)*100000.0));
        } else {
            getvalue1.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

            SYS_CONSOLE_PRINT(ESC_GREEN "ADC voltage = %d,%05d V \r\n" ESC_RESETCOLOR, (int) getvalue1.input_voltage, (int) ((getvalue1.input_voltage - (int) getvalue1.input_voltage)*100000.0));
        }

        while (PORT_PinRead(SPI_INT2_PIN));

    }
    //*********Read the ADC Continuous data*********//

    SYS_CONSOLE_MESSAGE("\r\n");
    SYS_CONSOLE_MESSAGE("ADC going into SleepMode state! \r\n");
    SPI_CS_Set();
}

bool _APP_Commands_SCAN_mode() {
    SPI_CS_Set();


    txData[1] = APP_CMD_CONVERSION;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);
    struct ADCvariable getvalue1;


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[1], 1)) {
        SYS_CONSOLE_MESSAGE("Initializing ADC single-shot...\r\n");
    } else {
        SYS_CONSOLE_MESSAGE("Error initializing ADC single-shot!\r\n");
        exit(0);
    }

    while (SERCOM1_SPI_IsTransmitterBusy() == true);


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    //*********Putting the ADC in Continuous mode*********//
    SYS_CONSOLE_MESSAGE("Setting the ADC in continuous mode...\r\n");

    txData[5] = 0xC0; // CONTINUOS MODE

    txData[1] = APP_CMD_DEVICE;
    txData[2] = APP_CMD_CONFIG3;
    txData[3] = APP_CMD_WRITE;

    txData[4] = ((uint8_t) txData[3]);
    txData[4] = txData[4] | ((uint8_t) (txData[2]) << 2);
    txData[4] = txData[4] | ((uint8_t) (txData[1]) << 6);

    SPI_CS_Clear();
    if (SERCOM1_SPI_Write(&txData[4], 1)) {

    } else {
        SYS_CONSOLE_MESSAGE(ESC_RED "Error initializing ADC continuous conversion!\r\n" ESC_RESETCOLOR);
    }
    while (SERCOM1_SPI_IsTransmitterBusy() == true);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[5], 1)) {

    }
    while (SERCOM1_SPI_IsTransmitterBusy() == true);

    SPI_CS_Set();
}

static void _APP_Commands_CONVERT(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    txData[1] = APP_CMD_CONVERSION;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[1], 1)) {
        SYS_CONSOLE_PRINT(ESC_GREEN"Analog value converted!\r\n" ESC_RESETCOLOR);
    } else {
        SYS_CONSOLE_PRINT(ESC_RED "Error initializing ADC conversion!\r\n" ESC_RESETCOLOR);
        exit(0);
    }

    while (SERCOM1_SPI_IsTransmitterBusy() == true);

}

static void _APP_Commands_STANDBY(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    txData[1] = APP_CMD_STANDBY;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[1], 1)) {
        SYS_CONSOLE_PRINT(ESC_YELLOW "ADC going in standby...\r\n" ESC_RESETCOLOR);
    } else {
        SYS_CONSOLE_PRINT(ESC_RED "Error initializing ADC conversion!\r\n" ESC_RESETCOLOR);
        exit(0);
    }

    while (SERCOM1_SPI_IsTransmitterBusy() == true);

}

static void _APP_Commands_SHUTDOWN(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    txData[1] = APP_CMD_SHUTDOWN;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[1], 1)) {
        SYS_CONSOLE_PRINT(ESC_YELLOW"ADC shutting down...\r\n" ESC_RESETCOLOR);
    } else {
        SYS_CONSOLE_PRINT(ESC_RED "Error initializing ADC conversion!\r\n" ESC_RESETCOLOR);
        exit(0);
    }

    while (SERCOM1_SPI_IsTransmitterBusy() == true);
}

static void _APP_Commands_DEFAULT(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    txData[1] = APP_CMD_DEFAULT;
    SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", txData[1]);


    SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
    SPI_CS_Clear();

    if (SERCOM1_SPI_Write(&txData[1], 1)) {
        SYS_CONSOLE_MESSAGE("Device full reset...\r\n");
    } else {
        SYS_CONSOLE_MESSAGE(ESC_YELLOW"Error initializing ADC single-shot!\r\n"ESC_RESETCOLOR);
        exit(0);
    }

    while (SERCOM1_SPI_IsTransmitterBusy() == true);
}

void _APP_Commands_about(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    SYS_CONSOLE_MESSAGE(ESC_BLUE"----- About this Build -----\r\n" ESC_RESETCOLOR\
 "Author/-s  : Andreas Birk Gustafson\r\n"\
                        "Project    : ADC configuration\r\n"\
                        "Company    : Hottinger Bruel & Kjaer\r\n"\
                        "Programmed : " __DATE__ " " __TIME__ "\r\n"\
                        "Device     : SAME51J20A\r\n"\
                        "\r\n");
}

/******************************************************************************/




//----------------------Math / conversion----------------------// 

int getbit(int w, unsigned j) {
    return ((w & MASK(j)) == 0) ? 0 : 1;
}



/******************************************************************************/




//----------------------SPI state-machine----------------------// 

void SPIEventHandler(uintptr_t context) {
    isTransferDone = true;

    /* De-assert the CS line */

    SPI_CS_Set();

}

void APP_Tasks(void) {
    struct ADCvariable getvalue1;

    while (ADCcycle == true) {

        /* Check the application's current state. */
        switch (appData.state) {
                /* Application's initial state. */
            case APP_STATE_INITIALIZE:

                SERCOM1_SPI_CallbackRegister(&SPIEventHandler, (uintptr_t) NULL);
                appData.state = APP_STATE_WRITE_DATA;

                break;


                /* TODO: implement your application state machine.*/

            case APP_STATE_TRANSFER_COMPLETE_WAIT:

                if (isTransferDone == true) {
                    isTransferDone = false;
                    /* Wait for the SPI slave to become ready before sending next commands */
                    while (SERCOM1_SPI_IsBusy() == true);
                    appData.state = nextState;
                }
                break;


            case APP_STATE_WRITE_DATA:
                configwrite = false;
                //while (SERCOM1_SPI_IsTransmitterBusy() == true);
                txData[0] = ("%x", *APP_Register_Buffer);
                txData[1] = ("%x", *APP_Config_Buffer);
                txData[2] = APP_CMD_READ;



                SPI_CS_Clear();
                if (SERCOM1_SPI_Write(&txData[0], sizeof (txData[0]))) {

                } else {
                    SPI_CS_Clear();

                }
                if (SERCOM1_SPI_Write(&txData[2], sizeof (txData[2]))) {
                    appData.state = APP_STATE_TRANSFER_COMPLETE_WAIT;
                    nextState = APP_STATE_SEND_READ_DATA_CMD;
                } else {

                }
                configwrite = true;
                break;

            case APP_STATE_SEND_READ_DATA_CMD:
                txData[1] = APP_CMD_DEVICE;
                txData[2] = APP_CMD_CONFIG0;
                txData[3] = APP_CMD_READ;

                txData[3] = ((uint8_t) txData[2]);
                txData[3] = txData[3] | ((uint8_t) (txData[1]) << 2);
                txData[3] = txData[3] | ((uint8_t) (txData[0]) << 6);
                SYS_CONSOLE_PRINT("Sending: 0x%x\r\n", (uint8_t) txData[3]);
                SPI_CS_Clear();
                SERCOM1_SPI_WriteRead(txData[3], 1, rxData, 4);
                appData.state = APP_STATE_TRANSFER_COMPLETE_WAIT;
                nextState = APP_STATE_VERIFY;
                break;

            case APP_STATE_READ_DATA:
                SPI_CS_Clear();
                SERCOM1_SPI_WriteRead(NULL, 0, rxData, 4);
                appData.state = APP_STATE_TRANSFER_COMPLETE_WAIT;
                nextState = APP_STATE_VERIFY;
                break;

            case APP_STATE_VERIFY:

                if (true) {
                    ADCval[2] = ((uint32_t) rxData[3]);
                    ADCval[2] = ADCval[2] | ((uint32_t) (rxData[2]) << 8);
                    ADCval[2] = ADCval[2] | ((uint32_t) (rxData[1]) << 16);

                    SYS_CONSOLE_PRINT("Return value: 0x%x\r\n", ADCval[2]);
                    //getvalue1.adc_count = *APP_Register_Buffer;
                    getvalue1.input_voltage = (float) ADCval[2] * ref_voltage / 8388608U;

                    SYS_CONSOLE_PRINT("ADC conversion:(" __DATE__ " " __TIME__ ") = %d.%02d V \r\n", (int) getvalue1.input_voltage, (int) ((getvalue1.input_voltage - (int) getvalue1.input_voltage)*100.0));

                    SYS_CONSOLE_MESSAGE("ADC going into SleepMode state! \r\n");
                    appData.state = APP_STATE_ERROR;
                } else {
                    appData.state = APP_STATE_ERROR;
                    ADCcycle = false;
                }
                break;

            case APP_STATE_ERROR:
                ADCcycle = false;
                break;

                /* The default state should never be executed. */
            default:
            {
                /* TODO: Handle error in application's state machine. */
                break;
            }
        }
    }
}

//static const SYS_CMD_DESCRIPTOR test;




/*******************************************************************************
 End of File
 */
