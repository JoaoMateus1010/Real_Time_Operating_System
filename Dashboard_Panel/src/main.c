/**
 *  \file   main.c
 *
 *  \brief  Example application main file. This application will toggle the led.
 *          The led toggling will be done inside an callback function, which
 *          will be called by Interrupt Service Routine. Interrupts are
 *          triggered manually and no external source is used to trigger
 *          interrupts.
 *
 */

/*
 * Copyright (C) 2014 - 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifdef USE_BIOS
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#if defined(SOC_AM65XX) || defined(SOC_J721E)
#if defined (__aarch64__)
#include <ti/sysbios/family/arm/v8a/Mmu.h>
#endif
#endif
#endif /* #ifdef USE_BIOS */

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include "GPIO_log.h"
#include "GPIO_board.h"

#include <ti/board/board.h>

#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#if defined(SOC_AM65XX) || defined(SOC_J721E)
#include <ti/drv/sciclient/sciclient.h>
#endif

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

#define DELAY_VALUE         (50U)   /* 50 msec */
#define TICK                (1U)

#define DELAY_VALUE_WCET_EICAS  (800U)
#define DELAY_VALUE_WCET_RNAV   (8000U)
#define DELAY_VALUE_WCET_EW     (1000U)
#define DELAY_VALUE_WCET_RADAR   (800U)

#define tskEICAS_FREQ   (20)
#define tskRNAV_FREQ    (1)
#define tskEW_FREQ      (5)
#define tskRADAR_FREQ   (10)

#define tskEICAS_PERIO   ((1)/(tskEICAS_FREQ))
#define tskENAV_PERIO    ((1)/(tskRNAV_FREQ))
#define tskEW_PERIO      ((1)/(tskEW_FREQ))
#define tskRADAR_PERIO   ((1)/(tskRADAR_FREQ))

#define tskEICAS_PRIORIDADE   (15)
#define tskRNAV_PRIORIDADE    (1)
#define tskEW_PRIORIDADE      (2)
#define tskRADAR_PRIORIDADE   (5)

#define USER_LED_0 (21)
#define USER_LED_1 (22)
#define USER_LED_2 (23)
#define USER_LED_3 (24)

#define MODE_NAV    (0)
#define MODE_COMBAT (1)

#define INT_NUMBER      (98)
#define PIN_NUMBER_INT  (16)

#define NUMBER_CONTROL_COCKPIT  (80)
/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);
void AppLoopDelay(uint32_t delayVal);
void tskEICAS();
void tskRNAV();
void tskEW();
void tskRADAR();
void isrFunc();
void swiFunc();
void mySwitchHook();


/* Callback function */
void AppGpioCallbackFxn(void);

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
/* GPIO clock and pinmux configurations */
extern void AppGPIOInit(void);
#endif

#if defined(idkAM574x) || defined(idkAM572x)
extern void GPIOApp_UpdateBoardInfo(void);
extern void GPIOAppUpdateConfig(uint32_t *gpioBaseAddr, uint32_t *gpioPin);
#endif

#if defined(SOC_AM65XX) || defined(SOC_J721E)
/* Main domain GPIO interrupt events */
#define MAIN_GPIO_INTRTR_GPIO0_BANK0_INT (0x000000C0) /* GPIO port 0 bank 0 interrupt event #, input to MAIN_GPIO_INTRTR */
#define MAIN_GPIO_INTRTR_GPIO1_BANK0_INT (0x000000C8) /* GPIO port 1 bank 0 interrupt event #, input to MAIN_GPIO_INTRTR */

/* Main domain GPIO interrupt events */
#define WKUP_GPIO_INTRTR_GPIO0_BANK0_INT (0x0000003C) /* GPIO port 0 bank 0 interrupt event #, input to WKUP_GPIO_INTRTR */


/* Main to MCU GPIO interrupt router mux output events */
#define CSL_MAIN2MCU_INTRTR_PLS_GPIOMUX_INT0_DFLT_PLS  (0x00000000)
#define CSL_MAIN2MCU_INTRTR_PLS_GPIOMUX_INT31_DFLT_PLS (0x0000001F)

void GPIO_configIntRouter(uint32_t portNum, uint32_t pinNum, uint32_t gpioIntRtrOutIntNum, GPIO_v0_HwAttrs *cfg)
{
    GPIO_IntCfg       *intCfg;
    uint32_t           bankNum;

    intCfg = cfg->intCfg;

#if defined (am65xx_evm) || defined (am65xx_idk) || defined(j721e_sim)
    
    /* no main domain GPIO pins directly connected to LEDs on GP EVM, 
       use WKUP domain GPIO pins which connected to LEDs on base board */
    cfg->baseAddr = CSL_WKUP_GPIO0_BASE;

    bankNum = pinNum/16; /* Each GPIO bank has 16 pins */

    /* WKUP GPIO int router input interrupt is the GPIO bank interrupt */
#if defined (__aarch64__)
#if defined (SOC_AM65XX)
    intCfg[pinNum].intNum = CSL_GIC0_INTR_WKUP_GPIOMUX_INTRTR0_BUS_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
    intCfg[pinNum].intNum = CSLR_WKUP_GPIOMUX_INTRTR0_IN_WKUP_GPIO0_GPIO_BANK_0 + bankNum;
#endif
#else
#if defined (SOC_AM65XX)
    intCfg[pinNum].intNum = CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
    intCfg[pinNum].intNum = CSLR_ARMSS0_CPU0_INTR_GPIOMUX_INTRTR0_OUTP_16 + bankNum;
#endif
#endif
    intCfg[pinNum].eventId = 0;
    intCfg[pinNum].intcMuxNum = INVALID_INTC_MUX_NUM;
    intCfg[pinNum].intcMuxInEvent = 0;
    intCfg[pinNum].intcMuxOutEvent = 0;
    
    /* Setup interrupt router configuration for gpio port/pin */
#else
    /* Use main domain GPIO pins directly connected to IDK EVM */

    bankNum = pinNum/16; /* Each GPIO bank has 16 pins */
    if (portNum == 0)
    {
        /* MAIN GPIO int router input interrupt is the GPIO bank interrupt */
#if defined (__aarch64__)
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_GIC0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_COMPUTE_CLUSTER0_GIC_SPI_GPIOMUX_INTRTR0_OUTP_8 + bankNum;
#endif
#else
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_MCU_ARMSS0_CPU0_INTR_MAIN2MCU_PLS_INTRTR0_OUTP_0 + bankNum;
#endif
#endif
    }
    else
    {
#if defined (__aarch64__)
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_GIC0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_6 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_COMPUTE_CLUSTER0_GIC_SPI_GPIOMUX_INTRTR0_OUTP_14 + bankNum;
#endif
#else
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_6 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_MCU_ARMSS0_CPU0_INTR_MAIN2MCU_PLS_INTRTR0_OUTP_6 + bankNum;
#endif
#endif
    }
    intCfg[pinNum].eventId = 0;
    intCfg[pinNum].intcMuxNum = INVALID_INTC_MUX_NUM;
    intCfg[pinNum].intcMuxInEvent = 0;
    intCfg[pinNum].intcMuxOutEvent = 0;

#endif
    GPIO_log("\nIntConfig:  portNum[%d],pinNum[%d],bankNum[%d], intNum[%d]",portNum,pinNum,bankNum,intCfg[pinNum].intNum);
}



#ifdef USE_BIOS
#if defined (__aarch64__)
Void InitMmu()
{
    Mmu_MapAttrs attrs;
    Bool         retVal;
    uint32_t     mapIdx = 0;

    Mmu_initMapAttrs(&attrs);

    attrs.attrIndx = 0;
    retVal = Mmu_map(0x00100000, 0x00100000, 0x00900000, &attrs); /* Main MMR0 cfg  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x00400000, 0x00400000, 0x00001000, &attrs); /* PSC0          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x01800000, 0x01800000, 0x00200000, &attrs); /* gicv3          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02400000, 0x02400000, 0x000c0000, &attrs); /* dmtimer        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
    
    mapIdx++;
    retVal = Mmu_map(0x02800000, 0x02800000, 0x00040000, &attrs); /* uart           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02000000, 0x02000000, 0x00100000, &attrs); /* main I2C       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    
    mapIdx++;
    retVal = Mmu_map(0x42120000, 0x42120000, 0x00001000, &attrs); /* Wkup I2C0       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02100000, 0x02100000, 0x00080000, &attrs); /* McSPI          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x00600000, 0x00600000, 0x00002000, &attrs); /* GPIO           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x42110000, 0x42110000, 0x00001000, &attrs); /* WKUP GPIO      */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x00a00000, 0x00a00000, 0x00040000, &attrs); /* MAIN INTR_ROUTERs */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x42200000, 0x42200000, 0x00001000, &attrs); /* WKUP INTR_ROUTER */
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }
    
    mapIdx++;
    retVal = Mmu_map(0x40f00000, 0x40f00000, 0x00020000, &attrs); /* MCU MMR0 CFG   */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x40d00000, 0x40d00000, 0x00002000, &attrs); /* PLL0 CFG       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x43000000, 0x43000000, 0x00020000, &attrs); /* WKUP MMR0 cfg  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02C40000, 0x02C40000, 0x00100000, &attrs); /* pinmux ctrl    */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x2A430000, 0x2A430000, 0x00001000, &attrs); /* ctrcontrol0    */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x030000000, 0x030000000, 0x10000000, &attrs); /* NAVSS used by sciclient  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }


    mapIdx++;
    retVal = Mmu_map(0x42000000, 0x42000000, 0x00001000, &attrs); /* PSC WKUP */
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }

    attrs.attrIndx = 7;
    mapIdx++;
    retVal = Mmu_map(0x80000000, 0x80000000, 0x03000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x70000000, 0x70000000, 0x04000000, &attrs); /* msmc           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

mmu_exit:
    if(retVal == FALSE)
    {
        System_printf("Mmu_map idx %d returned error %d", mapIdx, retVal);
        while(1);
    }

    return;
}
#endif /* #if defined (__aarch64__) */
#endif /* #ifdef USE_BIOS */
#endif /* #if defined(SOC_AM65XX) || defined(SOC_J721E) */

/*
 *  ======== Board_initI2C ========
 */
static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138) || defined(SOC_AM65XX) || defined(SOC_J721E)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);


#if defined(SOC_K2G)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#if defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);
#endif
#endif

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    Board_init(boardCfg);

#if defined(idkAM572x) || defined(idkAM574x)
    GPIOApp_UpdateBoardInfo();
#endif

    /* Modify the default GPIO configurations if necessary */
#if defined (am65xx_evm) || defined (am65xx_idk) || defined (j721e_sim)

    GPIO_configIntRouter(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, 0, &gpio_cfg);

    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#endif
}

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;

Task_Handle task0;
Task_Handle task1;
Task_Handle task2;
Task_Handle task3;

Swi_Handle swi0;

/* Create Tasks */
Task_Params task00Params;
Task_Params task01Params;
Task_Params task02Params;
Task_Params task03Params;

Semaphore_Handle semTask0;
Semaphore_Handle semTask1;
Semaphore_Handle semTask2;
Semaphore_Handle semTask3;

int iSwitchTask = 0;

Clock_Handle myClock;

int contTskEICAS    =0;
int contTskRNAV     =0;
int contTskEW       =0;
int contTskRADAR    =0;

int SystemModeOperation;

int SystemClkCockpitGlobal  =NUMBER_CONTROL_COCKPIT;
/*
 *  ======== test function ========
 */
#ifdef USE_BIOS
void gpio_test(UArg arg0, UArg arg1)
{
#else
int main()
{

    Board_initGPIO();
#endif
#if defined(SOC_AM574x) || defined(SOC_AM572x) || defined(SOC_AM571x)|| defined(SOC_AM335x) || defined(SOC_AM437x) || \
    defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2G) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138) || defined(SOC_AM65XX) || defined(SOC_J721E)
    uint32_t testOutput = 1;
#endif

    /* GPIO initialization */
    GPIO_init();

    /* Set the callback function */
    GPIO_setCallback(USER_LED0, AppGpioCallbackFxn);

    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(USER_LED0);

    /* Write high to gpio pin to control LED1 */
    GPIO_write((USER_LED1), GPIO_PIN_VAL_HIGH);
    AppDelay(DELAY_VALUE);

    GPIO_log("\n GPIO Led Blink Application \n");

#if defined(SOC_K2L) || defined(SOC_C6678) || defined(SOC_C6657)
    /* No GPIO pin directly connected to user LED's on K2L/C6678/C6657/AM65xx EVM, just trigger interrupt once */
    GPIO_toggle(USER_LED0);
    while (!gpio_intr_triggered);

    UART_printStatus("\n All tests have passed \n");
#ifdef USE_BIOS
    Task_exit();
#endif

#else

    while(1)
    {
#if defined(SOC_AM574x) || defined(SOC_AM572x) || defined(SOC_AM571x)|| defined(SOC_AM335x) || defined(SOC_AM437x)

#if defined (idkAM572x) || defined (idkAM574x)
        /* Update GPIO info based on the board */
        GPIOAppUpdateConfig(&gpioBaseAddr, &gpioPin);
#else
        gpioBaseAddr = GPIO_BASE_ADDR;
        gpioPin      = GPIO_LED_PIN;
#endif
        /* Trigger interrupt */
        GPIOTriggerPinInt(gpioBaseAddr, 0, gpioPin);
#endif
#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2G) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138) || defined(SOC_AM65XX) || defined(SOC_J721E)
        GPIO_toggle(USER_LED0);
#endif
        AppDelay(DELAY_VALUE);
        if (testOutput)
        {
            UART_printStatus("\n All tests have passed \n");
            testOutput = 0;
        }
    }
#endif
}

#ifdef USE_BIOS
/*
 *  ======== main ========
 */
int main(void)
{
 #if defined(SOC_AM65XX) || defined(SOC_J721E)
    Sciclient_ConfigPrms_t  sciClientCfg;
#endif

#if defined(SOC_AM65XX) || defined(SOC_J721E)
    Sciclient_configPrmsInit(&sciClientCfg);
    Sciclient_init(&sciClientCfg);
#endif
    /* Call board init functions */
    Board_initGPIO();

    SystemModeOperation = MODE_NAV;

    GPIODirModeSet(SOC_GPIO_1_REGS, USER_LED_0, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, USER_LED_1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, USER_LED_2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, USER_LED_3, GPIO_CFG_OUTPUT);

    GPIODirModeSet(SOC_GPIO_2_REGS, 10, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 12, GPIO_CFG_OUTPUT);

    GPIODirModeSet(SOC_GPIO_1_REGS, PIN_NUMBER_INT, GPIO_CFG_INPUT);
    GPIOIntTypeSet(SOC_GPIO_1_REGS, PIN_NUMBER_INT, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(SOC_GPIO_1_REGS, GPIO_INT_LINE_1, PIN_NUMBER_INT);

    Task_Params_init(&task00Params);
    Task_Params_init(&task01Params);
    Task_Params_init(&task02Params);
    Task_Params_init(&task03Params);

    task00Params.stackSize = 0x1400;
    task01Params.stackSize = 0x1400;
    task02Params.stackSize = 0x1400;
    task03Params.stackSize = 0x1400;

    task00Params.priority = tskEICAS_PRIORIDADE;
    task01Params.priority = tskRNAV_PRIORIDADE;
    task02Params.priority = tskEW_PRIORIDADE;
    task03Params.priority = tskRADAR_PRIORIDADE;

    task0 = Task_create(tskEICAS, &task00Params, NULL);
    task1 = Task_create(tskRNAV, &task01Params, NULL);
    task2 = Task_create(tskEW, &task02Params, NULL);
    task3 = Task_create(tskRADAR, &task03Params, NULL);

    /* Create Swi */
    Swi_Params swiParams;
    Swi_Params_init(&swiParams);
    swi0 = Swi_create(swiFunc, &swiParams, NULL);
    Swi_enable();

    /*Semaphores*/

    Semaphore_Params semParams0;
    Semaphore_Params_init(&semParams0);
    semParams0.mode = Semaphore_Mode_BINARY;
    semTask0 = Semaphore_create(1,&semParams0,NULL);

    Semaphore_Params semParams1;
    Semaphore_Params_init(&semParams1);
    semParams1.mode = Semaphore_Mode_BINARY;
    semTask1 = Semaphore_create(1,&semParams1,NULL);

    Semaphore_Params semParams2;
    Semaphore_Params_init(&semParams2);
    semParams2.mode = Semaphore_Mode_BINARY;
    semTask2 = Semaphore_create(1,&semParams2,NULL);

    Semaphore_Params semParams3;
    Semaphore_Params_init(&semParams3);
    semParams3.mode = Semaphore_Mode_BINARY;
    semTask3 = Semaphore_create(1,&semParams3,NULL);

    /* Create CloclkParams*/
    Clock_Params clkParams;
    Clock_Params_init(&clkParams);
    clkParams.startFlag=TRUE;
    clkParams.period=1;
    myClock = Clock_create(swiFunc,1,&clkParams,NULL);

    /* Create Hwi */
    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);
    hwiParams.enableInt = TRUE;
    Hwi_create(INT_NUMBER, isrFunc, &hwiParams, NULL);
    Hwi_enableInterrupt(INT_NUMBER);

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
    AppGPIOInit();
#endif

    /* Start BIOS */
    BIOS_start();
    return (0);
}
#endif

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal)
{
    Osal_delay(delayVal);
}

/*
 *  ======== AppLoopDelay ========
 */
void AppLoopDelay(uint32_t delayVal)
{
    volatile uint32_t i;

    for (i = 0; i < (delayVal * 1000); i++)
        ;
}

/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void)
{
    /* Toggle LED1 */
    GPIO_toggle(USER_LED1);
    AppLoopDelay(DELAY_VALUE);
    gpio_intr_triggered = 1;
}

//  MY FUNCTIONS

void isrFunc(){
    //UART_printStatus("ISR OK");
    if(SystemClkCockpitGlobal>=NUMBER_CONTROL_COCKPIT){
        if(SystemModeOperation==MODE_COMBAT){
            SystemModeOperation=MODE_NAV;
        }else{
            SystemModeOperation=MODE_COMBAT;
        }
        SystemClkCockpitGlobal=0;
    }else{
        //UART_printStatus("UNDER TIMER");
    }
    GPIOPinIntClear(SOC_GPIO_1_REGS, GPIO_INT_LINE_1, PIN_NUMBER_INT);
}

void swiFunc(){
    if(contTskEICAS==TICK){
        if((SystemModeOperation==MODE_NAV)||(SystemModeOperation==MODE_COMBAT)){
            Semaphore_post(semTask0);
        }
        contTskEICAS=0;
    }
    if(contTskRNAV==20*TICK){
        if((SystemModeOperation==MODE_NAV)||(SystemModeOperation==MODE_COMBAT)){
            Semaphore_post(semTask1);
        }
        contTskRNAV=0;
    }
    if(contTskEW==4*TICK){
        if(SystemModeOperation==MODE_COMBAT){
            Semaphore_post(semTask2);
        }
        contTskEW=0;
    }
    if(contTskRADAR==2*TICK){
        if(SystemModeOperation==MODE_COMBAT){
            Semaphore_post(semTask3);
        }
        contTskRADAR=0;
    }
    contTskEICAS++;
    contTskRNAV++;
    contTskEW++;
    contTskRADAR++;
    SystemClkCockpitGlobal++;
}

void myNewSwitch(){

}

void tskEICAS(){
    while(1){
        if((SystemModeOperation==MODE_NAV)||(SystemModeOperation==MODE_COMBAT)){
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_0, GPIO_PIN_HIGH);
                AppLoopDelay(DELAY_VALUE_WCET_EICAS/2);
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_0, GPIO_PIN_LOW);
                AppLoopDelay(DELAY_VALUE_WCET_EICAS/2);
                Semaphore_pend(semTask0,BIOS_WAIT_FOREVER);
        }
    }
}

void tskRNAV(){
    while(1){
        if((SystemModeOperation==MODE_NAV)||(SystemModeOperation==MODE_COMBAT)){
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_1, GPIO_PIN_HIGH);
                AppLoopDelay(DELAY_VALUE_WCET_RNAV/2);
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_1, GPIO_PIN_LOW);
                AppLoopDelay(DELAY_VALUE_WCET_RNAV/2);
                Semaphore_pend(semTask1,BIOS_WAIT_FOREVER);
        }
    }
}
void tskEW(){
    while(1){
        if(SystemModeOperation==MODE_COMBAT){
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_2, GPIO_PIN_HIGH);
                AppLoopDelay(DELAY_VALUE_WCET_EW/2);
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_2, GPIO_PIN_LOW);
                AppLoopDelay(DELAY_VALUE_WCET_EW/2);
                Semaphore_pend(semTask2,BIOS_WAIT_FOREVER);
        }else{
            Semaphore_pend(semTask2,BIOS_WAIT_FOREVER);
        }
    }
}
void tskRADAR(){
    while(1){
        if(SystemModeOperation==MODE_COMBAT){
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_3, GPIO_PIN_HIGH);
                AppLoopDelay(DELAY_VALUE_WCET_RADAR/2);
                GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_3, GPIO_PIN_LOW);
                AppLoopDelay(DELAY_VALUE_WCET_RADAR/2);
                Semaphore_pend(semTask3,BIOS_WAIT_FOREVER);
        }else{
            Semaphore_pend(semTask3,BIOS_WAIT_FOREVER);
        }
    }
}

/*SET HOOK mySwitch*/
void mySwitchHook(){
    //UART_printStatus("\n ENTROU HOOK \n");
    //Semaphore_post(semTask);
}
