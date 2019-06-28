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
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <MPU_6050_Register.h>
#include <math.h>
#include <ftoa.h>
#include <convert.h>
#include <string.h>
#if defined(SOC_AM65XX) || defined(SOC_J721E)
#include <ti/drv/sciclient/sciclient.h>
#endif

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#define DELAY_VALUE (50u)
//-----------------------  INICIO Defines Final Project   -------------------

#define SIGNAL_FL_SPEED_SENSOR  (117) //GPIO -> Input
#define SIGNAL_FR_SPEED_SENSOR  (49)  //GPIO -> Input
#define SIGNAL_RL_SPEED_SENSOR  (48)  //GPIO -> Input
#define SIGNAL_RR_SPEED_SENSOR  (60)  //GPIO -> Input

#define USER_CONTROL    (27) //GPIO -> Input

#define FORWARD_RIGHT_DIRECTION_1   (72)  //GPIO -> Output
#define FORWARD_RIGHT_DIRECTION_2   (73)  //GPIO -> Output
#define FORWARD_LEFT_DIRECTION_1    (75)  //GPIO -> Output
#define FORWARD_LEFT_DIRECTION_2    (74)  //GPIO -> Output
#define REAR_RIGHT_DIRECTION_1      (70)  //GPIO -> Output
#define REAR_RIGHT_DIRECTION_2      (71)  //GPIO -> Output
#define REAR_LEFT_DIRECTION_1       (77)  //GPIO -> Output
#define REAR_LEFT_DIRECTION_2       (76)  //GPIO -> Output

#define I2C_PCB9685_INSTANCE    (2)         //I2C_2
#define PCB9685_SLAVE_ADDR      (0x7A)      //I2C_2

#define I2C_MPU_6065_INSTANCE   (1)     //I2C_1
#define MPU_6050_SLAVE_ADDR     (0x68)  //I2C_1

#define USER_LED_0 (21) //GPIO -> Output
#define USER_LED_1 (22) //GPIO -> Output
#define USER_LED_2 (23) //GPIO -> Output
#define USER_LED_3 (24) //GPIO -> Output

#define MODE_1  (0) //Trajetória 1
#define MODE_2  (1) //Trajetória 2
#define MODE_3  (2) //Trajetória 3
#define MODE_4  (3) //Trajetória 4

#define LED_MODE_1 (USER_LED_0)
#define LED_MODE_2 (USER_LED_1)
#define LED_MODE_3 (USER_LED_2)
#define LED_MODE_4 (USER_LED_3)

#define INT_NUMBER_GPIO_0_A (96)
#define INT_NUMBER_GPIO_0_B (97)
#define INT_NUMBER_GPIO_1_A (98)
#define INT_NUMBER_GPIO_1_B (99)
#define INT_NUMBER_GPIO_2_A (32)
#define INT_NUMBER_GPIO_2_B (33)
#define INT_NUMBER_GPIO_3_A (62)
#define INT_NUMBER_GPIO_3_B (63)

#define TASK_CALIBRADOR_PRIORIDADE          (5)
#define TASK_READ_ROTA_PRIORIDADE           (4)
#define TASK_R1_PRIORIDADE                  (2)
#define TASK_R2_PRIORIDADE                  (2)
#define TASK_R3_PRIORIDADE                  (2)
#define TASK_R4_PRIORIDADE                  (2)
#define TASK_READ_MPU_PRIORIDADE            (7)

#define TASK_CALIBRADOR_PERIODO         (1)
#define TASK_READ_ROTA_PERIODO          (5*100)
#define TASK_R1_PERIODO                 (2)
#define TASK_R2_PERIODO                 (2)
#define TASK_R3_PERIODO                 (2)
#define TASK_R4_PERIODO                 (2)
#define TASK_READ_MPU_PERIODO           (1)

#define STATUS_PEND     (0)
#define STATUS_POST     (1)

#define NUMBER_OF_SAMPLE (5)

#define NUMBER_TIMER_MAX_WAITING    (5)

#define ROTA_NONE   (0)
#define ROTA_1      (1)
#define ROTA_2      (2)
#define ROTA_3      (3)
#define ROTA_4      (4)

#define MODE1           0x00

#define PRE_SCALE       0xFE

#define LED0_ON_L       0x06
#define LED0_ON_H       0x07
#define LED0_OFF_L      0x08
#define LED0_OFF_H      0x09

#define LED1_ON_L       0x0A
#define LED1_ON_H       0x0B
#define LED1_OFF_L      0x0C
#define LED1_OFF_H      0x0D

#define LED2_ON_L       0x0E
#define LED2_ON_H       0x0F
#define LED2_OFF_L      0x10
#define LED2_OFF_H      0x11

#define LED3_ON_L       0x12
#define LED3_ON_H       0x13
#define LED3_OFF_L      0x14
#define LED3_OFF_H      0x15

#define sensorFL    21   // 3 21 - 25 p9
#define sensorFR    17   // 1 17 - 23 p9
#define sensorRL    16   // 1 16 - 15 p9
#define sensorRR    28   // 1 28 - 12 p9

#define frd1    8   // 2  8  - 43 p8
#define fld1    11  // 2  11 - 42 p8
#define rrd1    6   // 2  6  - 45 p8
#define rld1    13  // 2  13 - 40 p8

#define frd2    9   // 2  9  - 44 p8
#define fld2    10  // 2  10 - 41 p8
#define rrd2    7   // 2  7  - 46 p8
#define rld2    12  // 2  12 - 39 p8

#define I2C_PCA_96685_INSTANCE      2
#define PCA_96685_SLAVE_ADDR     0x40

#define PCA9685_MODE1       0x00     /**< Mode Register 1 */
#define PCA9685_PRESCALE    0xFE    /**< Prescaler for PWM output frequency */

#define LED0_ON_L           0x06     /**< LED0 output and brightness control byte 0 */
#define LED0_ON_H           0x07     /**< LED0 output and brightness control byte 1 */
#define LED0_OFF_L          0x08     /**< LED0 output and brightness control byte 2 */
#define LED0_OFF_H          0x09     /**< LED0 output and brightness control byte 3 */

#define LED1_ON_L           0x0A
#define LED1_ON_H           0x0B
#define LED1_OFF_L          0x0C
#define LED1_OFF_H          0x0D

#define LED2_ON_L           0x0E
#define LED2_ON_H           0x0F
#define LED2_OFF_L          0x10
#define LED2_OFF_H          0x11

#define LED3_ON_L           0x12
#define LED3_ON_H           0x13
#define LED3_OFF_L          0x14
#define LED3_OFF_H          0x15

#define LED4_ON_L           0x16
#define LED4_ON_H           0x17
#define LED4_OFF_L          0x18
#define LED4_OFF_H          0x19

//-------------------------  FIM Defines Final Project   --------------------
/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);
void AppLoopDelay(uint32_t delayVal);
void tskCALIBRADOR();
void tskREADROTA();
void tskR1();
void tskR2();
void tskR3();
void tskR4();
void tskreadMPU();
void swiFuncCALIBRADOR();
void swiFuncREADROTA();
void swiFuncR1();
void swiFuncR2();
void swiFuncR3();
void swiFuncR4();
void swireadMPU();
float convertValRegToFloat(int val,int LSB);
void runSpaceInitToX(float finalSpace);
float getCurrAcellY();
char* convertFloatStr(float num);
//void ISR_SIGNAL_FL_SPEED_SENSOR();
//void ISR_SIGNAL_FR_SPEED_SENSOR();
//void ISR_SIGNAL_RL_SPEED_SENSOR();
//void ISR_SIGNAL_RR_SPEED_SENSOR();
void ISR_USER_CONTROL();
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
const unsigned CheckIfSigned[33]=
{0x00000000,
0x00000001,0x00000002,0x00000004,0x00000008,
0x00000010,0x00000020,0x00000040,0x00000080,
0x00000100,0x00000200,0x00000400,0x00000800,
0x00001000,0x00002000,0x00004000,0x00008000,
0x00010000,0x00020000,0x00040000,0x00080000,
0x00100000,0x00200000,0x00400000,0x00800000,
0x01000000,0x02000000,0x04000000,0x08000000,
0x10000000,0x20000000,0x40000000,0x80000000};
const unsigned ConvertToSigned[32]=
{0xffffffff,
0xfffffffe,0xfffffffc,0xfffffff8,0xfffffff0,
0xffffffe0,0xffffffc0,0xffffff80,0xffffff00,
0xfffffe00,0xfffffc00,0xfffff800,0xfffff000,
0xffffe000,0xffffc000,0xffff8000,0xffff0000,
0xfffe0000,0xfffc0000,0xfff80000,0xfff00000,
0xffe00000,0xffc00000,0xff800000,0xff000000,
0xfe000000,0xfc000000,0xf8000000,0xf0000000,
0xe0000000,0xc0000000,0x80000000};
const unsigned digits2bits[33]=
{0x00000000,
0x00000001,0x00000003,0x00000007,0x0000000f,
0x0000001f,0x0000003f,0x0000007f,0x000000ff,
0x000001ff,0x000003ff,0x000007ff,0x00000fff,
0x00001fff,0x00003fff,0x00007fff,0x0000ffff,
0x0001ffff,0x0003ffff,0x0007ffff,0x000fffff,
0x001fffff,0x003fffff,0x007fffff,0x00ffffff,
0x01ffffff,0x03ffffff,0x07ffffff,0x0fffffff,
0x1fffffff,0x3fffffff,0x7fffffff,0xffffffff};

volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;

Task_Handle task_CALIBRADOR;
Task_Handle task_READ_ROTA;
Task_Handle task_R1;
Task_Handle task_R2;
Task_Handle task_R3;
Task_Handle task_R4;
Task_Handle task_READ_MPU;

Swi_Handle swiFUNC_CALIBRADOR;
Swi_Handle swiFUNC_READ_ROTA;
Swi_Handle swiFUNC_R1;
Swi_Handle swiFUNC_R2;
Swi_Handle swiFUNC_R3;
Swi_Handle swiFUNC_R4;

/* Create Tasks */
Task_Params task_CALIBRADOR_Params;
Task_Params task_READ_ROTA_Params;
Task_Params task_R1_Params;
Task_Params task_R2_Params;
Task_Params task_R3_Params;
Task_Params task_R4_Params;
Task_Params task_READ_MPU_Params;

Semaphore_Handle semTask_CALIBRADOR;
Semaphore_Handle semTask_READ_ROTA;
Semaphore_Handle semTask_R1;
Semaphore_Handle semTask_R2;
Semaphore_Handle semTask_R3;
Semaphore_Handle semTask_R4;
Semaphore_Handle semTask_READ_MPU;


Clock_Handle clkTask_CALIBRADOR;
Clock_Handle clkTask_READ_ROTA;
Clock_Handle clkTask_R1;
Clock_Handle clkTask_R2;
Clock_Handle clkTask_R3;
Clock_Handle clkTask_R4;
Clock_Handle clkTask_READ_MPU;

I2C_HwAttrs i2c_cfg_MPU_6065;
I2C_HwAttrs i2c_cfg_PCB9685;

uint8_t cflag_Calibrador;
uint8_t cflag_Read_Rota;
uint8_t cflag_R1;
uint8_t cflag_R2;
uint8_t cflag_R3;
uint8_t cflag_R4;
uint8_t cflag_READ_MPU;

uint8_t number_of_sample_current = 0;

u_int8_t cflag_init_read_rota = FALSE;
u_int8_t cflag_init_R1 = FALSE;
u_int8_t cflag_init_R2 = FALSE;
u_int8_t cflag_init_R3 = FALSE;
u_int8_t cflag_init_R4 = FALSE;
u_int8_t cflag_init_READ_MPU = FALSE;

u_int8_t cflag_number_count = 0;

u_int8_t ROTA = ROTA_NONE;

u_int8_t cflag_isr_bt;

float inertial_X_ACCEL = 0;
float inertial_Y_ACCEL = 0;
float inertial_Z_ACCEL = 0;

float inertial_X_GYRO = 0;
float inertial_Y_GYRO = 0;
float inertial_Z_GYRO = 0;

float current_X_ACCEL = 0;
float current_Y_ACCEL = 0;
float current_Z_ACCEL = 0;

float current_X_GYRO = 0;
float current_Y_GYRO = 0;
float current_Z_GYRO = 0;
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
uint8_t readSensor(I2C_Handle h, uint8_t reg){
    uint8_t rxData = 0;
    uint8_t txData = 0;
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    memset(&txData, 0x00, sizeof(txData));
    t.slaveAddress = MPU_6050_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;
    transferStatus = I2C_transfer(h, &t);
    if(I2C_STS_SUCCESS != transferStatus){
        //UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
    return rxData;
}

void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val){
    uint8_t txData[2] = {0,0};
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    //memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = MPU_6050_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if(I2C_STS_SUCCESS != transferStatus){
           //UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
       }

}

void IMUSetUp(){
    I2C_Params i2cParams;
    I2C_Handle handle;
    I2C_Params_init(&i2cParams);
    //i2cParams.bitRate = I2C_400kHz;
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
    if(handle == NULL) UART_printf("ERROR");

//    UART_printf("Init Acc/Gyro Setup... \n");
//    UART_printf("Device Reset... \n");
    writeSensor(handle, PWR_MGMT_1, 0x00);
//    UART_printf("Accelerometer Config... \n");
    writeSensor(handle, ACCEL_CONFIG, 0x10);
//    UART_printf("Gyro Config... \n");
    writeSensor(handle, GYRO_CONFIG, 0x10);
    I2C_close(handle);
//    UART_printf("Acc/Gyro Setup Ended... \n");
}
float readACCEL_X(){
    float ret = 0;
    uint16_t number_to_return = 0;
    uint8_t high = 0;
    uint8_t low = 0;
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
    high = readSensor(handle, ACCEL_XOUT_H);
    low = readSensor(handle, ACCEL_XOUT_L);
    number_to_return |=high;
    number_to_return = number_to_return<<8;
    number_to_return |= low;
    ret = convertValRegToFloat(number_to_return, LSB_SENSITIVITY_ACCEL_8G);
    I2C_close(handle);
    return ret;
}
float readACCEL_Y(){
    float ret = 0;
    uint16_t number_to_return = 0;
        uint8_t high = 0;
        uint8_t low = 0;
        I2C_Params i2cParams;
        I2C_Handle handle = NULL;
        I2C_Params_init(&i2cParams);
        handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
        high = readSensor(handle, ACCEL_YOUT_H);
        low = readSensor(handle, ACCEL_YOUT_L);
        number_to_return |=high;
        number_to_return = number_to_return<<8;
        number_to_return |= low;
        ret = convertValRegToFloat(number_to_return, LSB_SENSITIVITY_ACCEL_8G);
        I2C_close(handle);
        return ret;
}
float readACCEL_Z(){
    float ret = 0;
        uint16_t number_to_return = 0;
        uint8_t high = 0;
        uint8_t low = 0;
        I2C_Params i2cParams;
        I2C_Handle handle = NULL;
        I2C_Params_init(&i2cParams);
        handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
        high = readSensor(handle, ACCEL_ZOUT_H);
        low = readSensor(handle, ACCEL_ZOUT_L);
        number_to_return |=high;
        number_to_return = number_to_return<<8;
        number_to_return |= low;
        ret = convertValRegToFloat(number_to_return, LSB_SENSITIVITY_ACCEL_8G);
        I2C_close(handle);
        return ret;
}

float readGYRO_X(){
    float ret = 0;
        uint16_t number_to_return = 0;
        uint8_t high = 0;
        uint8_t low = 0;
        I2C_Params i2cParams;
        I2C_Handle handle = NULL;
        I2C_Params_init(&i2cParams);
        handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
        high = readSensor(handle, GYRO_XOUT_H);
        low = readSensor(handle, GYRO_XOUT_L);
        number_to_return |=high;
        number_to_return = number_to_return<<8;
        number_to_return |= low;
        ret = convertValRegToFloat(number_to_return, LSB_SENSITIVITY_GYRO_1000);
        I2C_close(handle);
        return ret;
}
float readGYRO_Y(){
    float ret = 0;
            uint16_t number_to_return = 0;
            uint8_t high = 0;
            uint8_t low = 0;
            I2C_Params i2cParams;
            I2C_Handle handle = NULL;
            I2C_Params_init(&i2cParams);
            handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
            high = readSensor(handle, GYRO_YOUT_H);
            low = readSensor(handle, GYRO_YOUT_L);
            number_to_return |=high;
            number_to_return = number_to_return<<8;
            number_to_return |= low;
            ret = convertValRegToFloat(number_to_return, LSB_SENSITIVITY_GYRO_1000);
            I2C_close(handle);
            return ret;
}
float readGYRO_Z(){
    float ret = 0;
            uint16_t number_to_return = 0;
            uint8_t high = 0;
            uint8_t low = 0;
            I2C_Params i2cParams;
            I2C_Handle handle = NULL;
            I2C_Params_init(&i2cParams);
            handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
            high = readSensor(handle, GYRO_ZOUT_H);
            low = readSensor(handle, GYRO_ZOUT_L);
            number_to_return |=high;
            number_to_return = number_to_return<<8;
            number_to_return |= low;
            ret = convertValRegToFloat(number_to_return, LSB_SENSITIVITY_GYRO_1000);
            I2C_close(handle);
            return ret;
}
float convertValRegToFloat(int val,int LSB){
    int temp;
    temp = val & CheckIfSigned[16];
    if (temp != 0)
        temp = (val | ConvertToSigned[16-1]);
    else
        temp=val;
    float ret = (temp+0.0)/LSB;
    return ret;
}
float convertValRegToFloatGYRO(int val,int LSB){
    int temp;
    temp = val & CheckIfSigned[16];
    if (temp != 0)
        temp = (val | ConvertToSigned[16-1]);
    else
        temp=val;
    float ret = (temp+0.0)/LSB;
    return ret;
}
int convertBinarioToDecimal(int bin){
return 0;
}
int expNumber(int base,int exp){
    int number_to_return = base;
    for(int i=0;i<exp;i++){
        number_to_return *=number_to_return;
    }
    return number_to_return;
}
void setSpeedL0(uint8_t high, uint8_t low){

    UART_printf("roda 0\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
//    uint8_t data = 0;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED0_ON_H, 0x00);
    writeI2C(handle, LED0_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED0_OFF_H, 0x04);
        writeI2C(handle, LED0_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}

void setSpeedL1(uint8_t high, uint8_t low){
    UART_printf("roda 1\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
//    uint8_t data = 0;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED1_ON_H, 0x00);
    writeI2C(handle, LED1_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED1_OFF_H, 0x04);
        writeI2C(handle, LED1_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}

void setSpeedL2(uint8_t high, uint8_t low){
    UART_printf("roda 2\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
//    uint8_t data = 0;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED2_ON_H, 0x00);
    writeI2C(handle, LED2_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED2_OFF_H, 0x04);
        writeI2C(handle, LED2_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}

void setSpeedL3(uint8_t high, uint8_t low){
    UART_printf("roda 3\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
//    uint8_t data = 0;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED3_ON_H, 0x00);
    writeI2C(handle, LED3_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED3_OFF_H, 0x04);
        writeI2C(handle, LED3_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}
void frente(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_HIGH);

}

void esquerda(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_LOW);
}

void direita(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_HIGH);
}

void para(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_LOW);
}
uint8_t readI2C(I2C_Handle h, uint8_t reg){
    uint8_t rxData = 0;
    uint8_t txData = 0;

    I2C_Transaction t;
    int16_t transferStatus;

    I2C_transactionInit(&t);

    memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = PCA_96685_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;

    transferStatus = I2C_transfer(h, &t);

    if (I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n", transferStatus);
    }

    return rxData;
}

void writeI2C(I2C_Handle h, uint8_t reg, uint8_t val){
    uint8_t txData[2] = {0, 0};
    I2C_Transaction t;
    int16_t transferStatus;

    I2C_transactionInit(&t);

    memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = PCA_96685_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if (I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n", transferStatus);
    }
}
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
    I2C_socGetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg_MPU_6065);
    I2C_socSetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg_MPU_6065);

    I2C_socGetInitCfg(I2C_PCB9685_INSTANCE, &i2c_cfg_PCB9685);
    I2C_socSetInitCfg(I2C_PCB9685_INSTANCE, &i2c_cfg_PCB9685);

    I2C_init();
    Board_initGPIO();
        GPIODirModeSet(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_CFG_OUTPUT);

        GPIODirModeSet(SOC_GPIO_1_REGS, FORWARD_RIGHT_DIRECTION_1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, FORWARD_RIGHT_DIRECTION_2, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, FORWARD_LEFT_DIRECTION_1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, FORWARD_LEFT_DIRECTION_2, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, REAR_RIGHT_DIRECTION_1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, REAR_RIGHT_DIRECTION_2, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, REAR_LEFT_DIRECTION_1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, REAR_LEFT_DIRECTION_2, GPIO_CFG_OUTPUT);

        GPIODirModeSet(SOC_GPIO_0_REGS, USER_CONTROL, GPIO_CFG_INPUT);
        GPIOIntTypeSet(SOC_GPIO_0_REGS, USER_CONTROL, GPIO_INT_TYPE_RISE_EDGE);
        GPIOPinIntEnable(SOC_GPIO_0_REGS, GPIO_INT_LINE_1, USER_CONTROL);

        GPIODirModeSet(SOC_GPIO_3_REGS, sensorFL, GPIO_CFG_INPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, sensorFR, GPIO_CFG_INPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, sensorRL, GPIO_CFG_INPUT);
        GPIODirModeSet(SOC_GPIO_1_REGS, sensorRR, GPIO_CFG_INPUT);

        GPIODirModeSet(SOC_GPIO_2_REGS, frd1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_2_REGS, fld1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_2_REGS, rrd1, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_2_REGS, rld1, GPIO_CFG_OUTPUT);

        GPIODirModeSet(SOC_GPIO_2_REGS, frd2, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_2_REGS, fld2, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_2_REGS, rrd2, GPIO_CFG_OUTPUT);
        GPIODirModeSet(SOC_GPIO_2_REGS, rld2, GPIO_CFG_OUTPUT);
//
//        GPIODirModeSet(SOC_GPIO_3_REGS, SIGNAL_FL_SPEED_SENSOR, GPIO_CFG_INPUT);
//        GPIOIntTypeSet(SOC_GPIO_3_REGS, SIGNAL_FL_SPEED_SENSOR, GPIO_INT_TYPE_RISE_EDGE);
//        GPIOPinIntEnable(SOC_GPIO_3_REGS, GPIO_INT_LINE_1, SIGNAL_FL_SPEED_SENSOR);
//
//        GPIODirModeSet(SOC_GPIO_1_REGS, SIGNAL_FR_SPEED_SENSOR, GPIO_CFG_INPUT);
//        GPIOIntTypeSet(SOC_GPIO_1_REGS, SIGNAL_FR_SPEED_SENSOR, GPIO_INT_TYPE_RISE_EDGE);
//        GPIOPinIntEnable(SOC_GPIO_1_REGS, GPIO_INT_LINE_1, SIGNAL_FR_SPEED_SENSOR);
//
//        GPIODirModeSet(SOC_GPIO_1_REGS, SIGNAL_RL_SPEED_SENSOR, GPIO_CFG_INPUT);
//        GPIOIntTypeSet(SOC_GPIO_1_REGS, SIGNAL_RL_SPEED_SENSOR, GPIO_INT_TYPE_RISE_EDGE);
//        GPIOPinIntEnable(SOC_GPIO_1_REGS, GPIO_INT_LINE_1, SIGNAL_RL_SPEED_SENSOR);
//
//        GPIODirModeSet(SOC_GPIO_1_REGS, SIGNAL_RR_SPEED_SENSOR, GPIO_CFG_INPUT);
//        GPIOIntTypeSet(SOC_GPIO_1_REGS, SIGNAL_RR_SPEED_SENSOR, GPIO_INT_TYPE_RISE_EDGE);
//        GPIOPinIntEnable(SOC_GPIO_1_REGS, GPIO_INT_LINE_1, SIGNAL_RR_SPEED_SENSOR);
    /* Create Swi */
//    Swi_Params swiParams_CALIBRADOR;
//    Swi_Params_init(&swiParams_CALIBRADOR);
//    swiFUNC_CALIBRADOR = Swi_create(swiFuncCALIBRADOR, &swiParams_CALIBRADOR, NULL);
//
//    Swi_Params swiParams_READ_ROTA;
//    Swi_Params_init(&swiParams_READ_ROTA);
//    swiFUNC_READ_ROTA = Swi_create(swiFuncREADROTA, &swiParams_READ_ROTA, NULL);
////
//    Swi_Params swiParams_R1;
//    Swi_Params_init(&swiParams_R1);
//    swiFUNC_R1 = Swi_create(swiFuncR1, &swiParams_R1, NULL);
////
//    Swi_Params swiParams_R2;
//    Swi_Params_init(&swiParams_R2);
//    swiFUNC_R2 = Swi_create(swiFuncR2, &swiParams_R2, NULL);
//
//    Swi_Params swiParams_R3;
//    Swi_Params_init(&swiParams_R3);
//    swiFUNC_R3 = Swi_create(swiFuncR3, &swiParams_R3, NULL);
//
//    Swi_Params swiParams_R4;
//    Swi_Params_init(&swiParams_R4);
//    swiFUNC_R4 = Swi_create(swiFuncR4, &swiParams_R4, NULL);

    Swi_enable();

    /*Semaphores*/

    Semaphore_Params semParams_CALIBRADOR;
    Semaphore_Params_init(&semParams_CALIBRADOR);
    semParams_CALIBRADOR.mode = Semaphore_Mode_BINARY;
    semTask_CALIBRADOR = Semaphore_create(1,&semParams_CALIBRADOR,NULL);
//
    Semaphore_Params semParams_READ_ROTA;
    Semaphore_Params_init(&semParams_READ_ROTA);
    semParams_READ_ROTA.mode = Semaphore_Mode_BINARY;
    semTask_READ_ROTA = Semaphore_create(1,&semParams_READ_ROTA,NULL);
//
    Semaphore_Params semParams_R1;
    Semaphore_Params_init(&semParams_R1);
    semParams_R1.mode = Semaphore_Mode_BINARY;
    semTask_R1 = Semaphore_create(1,&semParams_R1,NULL);
//
    Semaphore_Params semParams_R2;
    Semaphore_Params_init(&semParams_R2);
    semParams_R2.mode = Semaphore_Mode_BINARY;
    semTask_R2 = Semaphore_create(1,&semParams_R2,NULL);

    Semaphore_Params semParams_R3;
    Semaphore_Params_init(&semParams_R3);
    semParams_R3.mode = Semaphore_Mode_BINARY;
    semTask_R3 = Semaphore_create(1,&semParams_R3,NULL);

    Semaphore_Params semParams_R4;
    Semaphore_Params_init(&semParams_R4);
    semParams_R4.mode = Semaphore_Mode_BINARY;
    semTask_R4 = Semaphore_create(1,&semParams_R4,NULL);

    Semaphore_Params semParams_READ_MPU;
    Semaphore_Params_init(&semParams_READ_MPU);
    semParams_READ_MPU.mode = Semaphore_Mode_BINARY;
    semTask_READ_MPU = Semaphore_create(1,&semParams_READ_MPU,NULL);
    /* Create CloclkParams*/
    Clock_Params clkParams_CALIBRADOR;
    Clock_Params_init(&clkParams_CALIBRADOR);
    clkParams_CALIBRADOR.startFlag=TRUE;
    clkParams_CALIBRADOR.period=TASK_CALIBRADOR_PERIODO;
    clkTask_CALIBRADOR = Clock_create(swiFuncCALIBRADOR,1,&clkParams_CALIBRADOR,NULL);

    Clock_Params clkParams_READ_ROTA;
    Clock_Params_init(&clkParams_READ_ROTA);
    clkParams_READ_ROTA.startFlag=TRUE;
    clkParams_READ_ROTA.period=TASK_READ_ROTA_PERIODO;
    clkTask_READ_ROTA = Clock_create(swiFuncREADROTA,1,&clkParams_READ_ROTA,NULL);

    Clock_Params clkParams_R1;
    Clock_Params_init(&clkParams_R1);
    clkParams_R1.startFlag=TRUE;
    clkParams_R1.period=TASK_R1_PERIODO;
    clkTask_R1 = Clock_create(swiFuncR1,1,&clkParams_R1,NULL);

    Clock_Params clkParams_R2;
    Clock_Params_init(&clkParams_R2);
    clkParams_R2.startFlag=TRUE;
    clkParams_R2.period=TASK_R2_PERIODO;
    clkTask_R2 = Clock_create(swiFuncR2,1,&clkParams_R2,NULL);

    Clock_Params clkParams_R3;
    Clock_Params_init(&clkParams_R3);
    clkParams_R3.startFlag=TRUE;
    clkParams_R3.period=TASK_R2_PERIODO;
    clkTask_R3 = Clock_create(swiFuncR3,1,&clkParams_R3,NULL);

    Clock_Params clkParams_R4;
    Clock_Params_init(&clkParams_R4);
    clkParams_R4.startFlag=TRUE;
    clkParams_R4.period=TASK_R4_PERIODO;
    clkTask_R4 = Clock_create(swiFuncR4,1,&clkParams_R4,NULL);

    Clock_Params clkParams_READ_MPU;
    Clock_Params_init(&clkParams_READ_MPU);
    clkParams_READ_MPU.startFlag=TRUE;
    clkParams_READ_MPU.period=TASK_READ_MPU_PERIODO;
    clkTask_READ_MPU = Clock_create(swireadMPU,1,&clkParams_READ_MPU,NULL);

    /* Create Hwi */
    Hwi_Params hwiParams_USER_CONTROL;
    Hwi_Params_init(&hwiParams_USER_CONTROL);
    hwiParams_USER_CONTROL.enableInt = TRUE;
    Hwi_create(INT_NUMBER_GPIO_0_A, ISR_USER_CONTROL, &hwiParams_USER_CONTROL, NULL);
    Hwi_enableInterrupt(INT_NUMBER_GPIO_0_A);

//    Hwi_Params hwiParams_E_M_3;
//    Hwi_Params_init(&hwiParams_E_M_3);
//    hwiParams_E_M_3.enableInt = TRUE;
//    Hwi_create(INT_NUMBER_GPIO_3_B, ISR_SIGNAL_FL_SPEED_SENSOR, &hwiParams_E_M_3, NULL);
//    Hwi_enableInterrupt(INT_NUMBER_GPIO_3_B);

    /*  TASKS  */

    Task_Params_init(&task_CALIBRADOR_Params);
    Task_Params_init(&task_READ_ROTA_Params);
    Task_Params_init(&task_R1_Params);
    Task_Params_init(&task_R2_Params);
    Task_Params_init(&task_R3_Params);
    Task_Params_init(&task_R4_Params);
    Task_Params_init(&task_READ_MPU_Params);

    task_CALIBRADOR_Params.stackSize = 0x1400;
    task_READ_ROTA_Params.stackSize = 0x1400;
    task_R1_Params.stackSize = 0x1400;
    task_R2_Params.stackSize = 0x1400;
    task_R3_Params.stackSize = 0x1400;
    task_R4_Params.stackSize = 0x1400;
    task_READ_MPU_Params.stackSize = 0x1400;

    task_CALIBRADOR_Params.priority = TASK_CALIBRADOR_PRIORIDADE;
    task_READ_ROTA_Params.priority = TASK_READ_ROTA_PRIORIDADE;
    task_R1_Params.priority = TASK_R1_PRIORIDADE;
    task_R2_Params.priority = TASK_R2_PRIORIDADE;
    task_R3_Params.priority = TASK_R3_PRIORIDADE;
    task_R4_Params.priority = TASK_R4_PRIORIDADE;
    task_READ_MPU_Params.priority = TASK_R4_PRIORIDADE;

    task_CALIBRADOR = Task_create(tskCALIBRADOR, &task_CALIBRADOR_Params, NULL);
    task_READ_ROTA = Task_create(tskREADROTA, &task_READ_ROTA_Params, NULL);
    task_R1 = Task_create(tskR1, &task_R1_Params, NULL);
    task_R2 = Task_create(tskR2, &task_R2_Params, NULL);
    task_R3 = Task_create(tskR3, &task_R3_Params, NULL);
    task_R4 = Task_create(tskR4, &task_R4_Params, NULL);
//    task_READ_MPU = Task_create(tskreadMPU, &task_READ_MPU_Params, NULL);

    cflag_Calibrador = STATUS_POST;
    cflag_Read_Rota = STATUS_POST;
    cflag_R1 = STATUS_POST;
    cflag_R2 = STATUS_POST;
    cflag_R3 = STATUS_POST;
    cflag_R4 = STATUS_POST;
    cflag_READ_MPU = STATUS_POST;
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

//void isrFunc(){
//    //UART_printStatus("ISR OK");
//    GPIOPinIntClear(SOC_GPIO_1_REGS, GPIO_INT_LINE_1, PIN_NUMBER_INT);
//}
/*---------------- TASK  ----------------*/
void tskCALIBRADOR(){
    IMUSetUp();
        setSpeedL0(0, 0);
        setSpeedL1(0, 0);
        setSpeedL2(0, 0);
        setSpeedL3(0, 0);
    while(1){
        if(cflag_Calibrador == STATUS_POST){
            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_0, GPIO_PIN_HIGH);
            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_1, GPIO_PIN_HIGH);
            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_2, GPIO_PIN_HIGH);
            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_3, GPIO_PIN_HIGH);
            UART_printf("SAMPLE: %d\n",number_of_sample_current);

            inertial_X_ACCEL += readACCEL_X();
            inertial_Y_ACCEL += readACCEL_Y();
            inertial_Z_ACCEL += readACCEL_Z();

            inertial_X_GYRO += readGYRO_X();
            inertial_Y_GYRO += readGYRO_Y();
            inertial_Z_GYRO += readGYRO_Z();

            cflag_Calibrador=STATUS_PEND;
            number_of_sample_current += 1;
            Semaphore_pend(semTask_CALIBRADOR, BIOS_WAIT_FOREVER);
        }
    }
}
void tskREADROTA(){
    while(1){
        if(cflag_init_read_rota == TRUE){
            if(cflag_Read_Rota==STATUS_POST){
                if(cflag_isr_bt > 50){
                    UART_printStatus("Lendo rota");
                }
                cflag_Read_Rota=STATUS_PEND;
                Semaphore_pend(semTask_READ_ROTA, BIOS_WAIT_FOREVER);
            }
        }
    }
}
void tskR1(){
    while(1){
        Semaphore_pend(semTask_R1, BIOS_WAIT_FOREVER);
        if(cflag_init_R1 == TRUE){
            UART_printStatus("TSK_R1\n");
            frente();
            int conta = 0;
            Task_sleep(50);
            while(conta != 13){
                if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
                    conta = conta + 1;
                    UART_printf("%d\n", conta);
                    Task_sleep(10);
                }
            }
            para();
            cflag_init_R1 = FALSE;
            ROTA = ROTA_NONE;
        }
    }
}
void tskR2(){
    while(1){
        Semaphore_pend(semTask_R2, BIOS_WAIT_FOREVER);
        if(cflag_init_R2 == TRUE){
            frente();
                int conta = 0;
                Task_sleep(50);
                while(conta != 26){
                    if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
                        conta = conta + 1;
                        UART_printf("%d\n", conta);
                        Task_sleep(10);
                    }
                }
                para();
                cflag_init_R2 = FALSE;
                ROTA = ROTA_NONE;
        }
    }
}
void tskR3(){
    Semaphore_pend(semTask_R3, BIOS_WAIT_FOREVER);
    while(1){
        if(cflag_init_R3 == TRUE){
            char buff_GYRO_Z[100];
            int conta = 0;
            float soma = 0;
            float atual;
            float base;
            frente();
            Task_sleep(50);
            frente();
            while(conta != 16){
                if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
                    conta = conta + 1;
                    UART_printf("%d\n", conta);
                    Task_sleep(10);
                }
            }
            para();
            base = readGYRO_Z();
            if(base < 0){
                base = (base*(-1));
            }
            soma = 0;
            esquerda();
            while(soma < 90){
                Task_sleep(40);
                atual = readGYRO_Z();
                if(atual < 0){
                    atual = (atual*(-1));
                }
                ftoa(atual, buff_GYRO_Z, 4);
                UART_printf("%s\n", buff_GYRO_Z);
                if(atual > (base + 1)){
                    soma = soma + (atual*0.7);
                }
            }
            para();
            conta = 0;
            frente();
                    Task_sleep(50);
                    frente();
                    while(conta != 16){
                        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
                            conta = conta + 1;
                            UART_printf("%d\n", conta);
                            Task_sleep(10);
                        }
                    }
                    para();
                    base = readGYRO_Z();
                    if(base < 0){
                        base = (base*(-1));
                    }
                    soma = 0;
                    esquerda();
                    base = 0;
                    while(soma < 90){
                        Task_sleep(40);
                        atual = readGYRO_Z();
                        if(atual < 0){
                            atual = (atual*(-1));
                        }
                        ftoa(atual, buff_GYRO_Z, 4);
                        UART_printf("%s\n", buff_GYRO_Z);
                        if(atual > (base + 1)){
                            soma = soma + (atual*0.7);
                        }
                    }
                    para();
                    conta = 0;
                    base = 0;
                    frente();
                    Task_sleep(50);
                                        frente();
                                        while(conta != 16){
                                            if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
                                                conta = conta + 1;
                                                UART_printf("%d\n", conta);
                                                Task_sleep(10);
                                            }
                                        }
                                        para();
                                        conta = 0;
                                        base = readGYRO_Z();
                                        if(base < 0){
                                            base = (base*(-1));
                                        }
                                        soma = 0;
                                        esquerda();
                                        base = 0;
                                        while(soma < 90){
                                            Task_sleep(40);
                                            atual = readGYRO_Z();
                                            if(atual < 0){
                                                atual = (atual*(-1));
                                            }
                                            ftoa(atual, buff_GYRO_Z, 4);
                                            UART_printf("%s\n", buff_GYRO_Z);
                                            if(atual > (base + 1)){
                                                soma = soma + (atual*0.7);
                                            }
                                        }
                                        para();
                                        frente();
                                        while(conta != 16){
                                                if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
                                                    conta = conta + 1;
                                                    UART_printf("%d\n", conta);
                                                    Task_sleep(10);
                                                }
                                            }
                                            para();

                                        conta = 0;
                                        base = 0;
                    cflag_init_R3 = FALSE;
            ROTA = ROTA_NONE;
        }
    }
}
void tskR4(){
    while(1){
        Semaphore_pend(semTask_R4, BIOS_WAIT_FOREVER);
        //        if(cflag_init_R4 == TRUE){
//            if(cflag_R4 == STATUS_POST){
                UART_printStatus("TSK_R4\n");
//                cflag_R4 = STATUS_PEND;
//            }
//        }
    }
}
void tskreadMPU(){
    while(1){
        if(cflag_init_READ_MPU == TRUE){
            if(cflag_READ_MPU == STATUS_POST){
                UART_printStatus("TSK_READ_MPU\n");
                cflag_READ_MPU = STATUS_PEND;
                Semaphore_pend(semTask_READ_MPU, BIOS_WAIT_FOREVER);
            }
        }
    }
}
/*----------------- SWI   -----------------*/
void swiFuncCALIBRADOR(){
    if(cflag_Calibrador == STATUS_PEND){
        cflag_Calibrador = STATUS_POST;
        if(number_of_sample_current < NUMBER_OF_SAMPLE-1){
            Semaphore_post(semTask_CALIBRADOR);
        }else{
            cflag_init_read_rota = TRUE;
            cflag_Read_Rota=STATUS_POST;
            inertial_X_ACCEL /= NUMBER_OF_SAMPLE;
            inertial_Y_ACCEL /= NUMBER_OF_SAMPLE;
            inertial_Z_ACCEL /= NUMBER_OF_SAMPLE;

            inertial_X_GYRO /= NUMBER_OF_SAMPLE;
            inertial_Y_GYRO /= NUMBER_OF_SAMPLE;
            inertial_Z_GYRO /= NUMBER_OF_SAMPLE;

            char buff_i_x_a[100];
            char buff_i_y_a[100];
            char buff_i_z_a[100];

            char buff_i_x_g[100];
            char buff_i_y_g[100];
            char buff_i_z_g[100];

            ftoa(inertial_X_ACCEL, buff_i_x_a, 2);
            ftoa(inertial_Y_ACCEL, buff_i_y_a, 2);
            ftoa(inertial_Z_ACCEL, buff_i_z_a, 2);
            ftoa(inertial_X_GYRO, buff_i_x_g, 2);
            ftoa(inertial_Y_GYRO, buff_i_y_g, 2);
            ftoa(inertial_Z_GYRO, buff_i_z_g, 2);

//            UART_printf("I_X: %s\n",buff_i_x_a);
//            UART_printf("I_Y: %s\n",buff_i_y_a);
//            UART_printf("I_Z: %s\n",buff_i_z_a);
//            UART_printf("I_X: %s\n",buff_i_x_g);
//            UART_printf("I_Y: %s\n",buff_i_y_g);
//            UART_printf("I_Z: %s\n",buff_i_z_g);

            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_0, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_1, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_2, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, USER_LED_3, GPIO_PIN_LOW);
            ROTA = ROTA_1;

        }
    }else{
        cflag_isr_bt +=1;
    }
}
void swiFuncREADROTA(){
    switch (ROTA){
            case (ROTA_1):
                UART_printStatus("ROTA 1\n");
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_HIGH);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                Semaphore_post(semTask_R1);
                break;
            case (ROTA_2):
                UART_printStatus("ROTA 2\n");
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_HIGH);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                Semaphore_post(semTask_R2);
                break;
            case (ROTA_3):
                UART_printStatus("ROTA 3\n");
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_HIGH);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                Semaphore_post(semTask_R3);
                break;
            case (ROTA_4):
                UART_printStatus("ROTA 4\n");
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
            GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_HIGH);
                Semaphore_post(semTask_R4);
                break;
            case (ROTA_NONE):
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                    break;
    }
}
void swiFuncR1(){
//    if(cflag_init_R1 == TRUE){
//        if(cflag_R1 == STATUS_PEND){
//            UART_printStatus("SWI_R1\n");
//            cflag_R1 = STATUS_POST;
//            Semaphore_post(semTask_R1);
//        }
//    }
}
void swiFuncR2(){
//    if(cflag_init_R2 == TRUE){
//        if(cflag_R2 == STATUS_PEND){
//            UART_printStatus("SWI_R2\n");
//            cflag_R2 = STATUS_POST;
//            Semaphore_post(semTask_R2);
//        }
//    }
}
void swiFuncR3(){
//    if(cflag_init_R3 == TRUE){
//        if(cflag_R3 == STATUS_PEND){
//            UART_printStatus("SWI_R3\n");
//            cflag_R3 = STATUS_POST;
//            Semaphore_post(semTask_R3);
//        }
//    }
}
void swiFuncR4(){
//    if(cflag_init_R4 == TRUE){
//        if(cflag_R4 == STATUS_PEND){
//            UART_printStatus("SWI_R4\n");
//            cflag_R4 = STATUS_POST;
//            Semaphore_post(semTask_R4);
//        }
//    }
}
void swireadMPU(){
    if(cflag_init_READ_MPU == TRUE){
        if(cflag_READ_MPU == STATUS_PEND){
            UART_printStatus("SWI_R4\n");
            cflag_READ_MPU = STATUS_POST;
            Semaphore_post(semTask_READ_MPU);
        }
    }
}
/*  ---------------  ISR  ----------------- */
//void ISR_SIGNAL_FL_SPEED_SENSOR(){
//        UART_printStatus("ISR FL OK");
//        GPIOPinIntClear(SOC_GPIO_3_REGS, GPIO_INT_LINE_1, SIGNAL_FL_SPEED_SENSOR);
//}
//void ISR_SIGNAL_FR_SPEED_SENSOR(){
//
//}
//void ISR_SIGNAL_RL_SPEED_SENSOR(){
//
//}
//void ISR_SIGNAL_RR_SPEED_SENSOR(){
//
//}
void ISR_USER_CONTROL(){
    if(cflag_isr_bt > 50){
        switch (ROTA){
                case (ROTA_NONE):
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_HIGH);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                cflag_init_R1 = TRUE;
                ROTA = ROTA_1;
                    break;
                case (ROTA_1):
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_HIGH);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                ROTA = ROTA_2;
                cflag_init_R2 = TRUE;
                    break;
                case (ROTA_2):
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_HIGH);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                ROTA = ROTA_3;
                cflag_init_R3 = TRUE;
                    break;
                case (ROTA_3):
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_HIGH);
                ROTA = ROTA_4;
                cflag_init_R4 = TRUE;
                    break;
                case (ROTA_4):
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_1, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_2, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_3, GPIO_PIN_LOW);
                GPIOPinWrite(SOC_GPIO_1_REGS, LED_MODE_4, GPIO_PIN_LOW);
                ROTA = ROTA_NONE;
                    break;
        }
        cflag_isr_bt = 0;
    }
    GPIOPinIntClear(SOC_GPIO_0_REGS, GPIO_INT_LINE_1, USER_CONTROL);
}
void runSpaceInitToX(float finalSpace){
    float Sf = finalSpace;
    float SCurrent = 0;
    float T0 = Clock_getTicks();
    float TF = Clock_getTicks();
    UART_printf("Andou: %s",convertFloatStr(Sf));
    while (SCurrent >= Sf){
        SCurrent += (((getCurrAcellY())*(pow((TF-T0),(2))))/2);
        AppLoopDelay(DELAY_VALUE);
        T0 = TF;
        TF = Clock_getTicks();
    }
}
float getCurrAcellY(){
    return inertial_Y_ACCEL - readACCEL_Y();
}

char* convertFloatStr(float num){
    char buff[100];
    ftoa(num, buff, 2);
    return buff;
}
