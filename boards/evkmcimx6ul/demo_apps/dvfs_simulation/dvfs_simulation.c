/*
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_common.h"

#include "fsl_gpt.h"
#include "fsl_gpio.h"
/*******************************************************************************
* Definitions
******************************************************************************/
/*
TASK_PERIOD = 1000: TASK period is 1s, low cpu loading, IDLE is 100%
TASK_PERIOD = 100:TASK period is 100ms, mid cpu loading, IDLE is 92%
TASK_PERIOD = 10:TASK period is 10ms, mid cpu loading, IDLE is 55%
TASK_PERIOD = 1:TASK period is 1ms, high cpu loading, IDLE is 10%
*/
#define TASK_PERIOD  1000	
/* The software timer period. */
#define TASK_TIMER_PERIOD_MS ((TASK_PERIOD) / portTICK_PERIOD_MS) 

#define TASK_CPU_LOW_LOADING  ((1000) / portTICK_PERIOD_MS) 
#define TASK_CPU_MID_LOW_LOADING  ((100) / portTICK_PERIOD_MS) 
#define TASK_CPU_MID_HIGH_LOADING  ((10) / portTICK_PERIOD_MS) 
#define TASK_CPU_HIGH_LOADING  ((1) / portTICK_PERIOD_MS) 

#define SW_TIMER_PERIOD_MS ((5000) / portTICK_PERIOD_MS)    //SW period is 5s

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
* Prototypes
******************************************************************************/
static void hello_task(void *pvParameters);
static void SwTimerCallback(TimerHandle_t xTimer);
int arm_voltage_setpoint(uint8_t target_vol);
int arm_freq_setpoint(uint32_t target_freq);

//Ben
char pcWriteBuffer[1024];
unsigned long idle_cycle_cnt = 0UL;

/* IMX6UL start from 396Mhz, ARM_CAP =1.0 V*/
uint32_t freq=396000000;
float vol=1.00;

uint32_t  ret=0;
int i,j=0;
uint8_t arm_reg_targ;
uint8_t freq_up_flag=0,freq_down_flag=0,change_flag=0;
const clock_arm_pll_config_t g_ccmConfigCPUPll_528Mhz = {.loopDivider = 88U}; //CPU clock = 24x88/2/2= 528Mhz
const clock_arm_pll_config_t g_ccmConfigCPUPll_396Mhz = {.loopDivider = 66U}; //CPU clock = 24x66/2/2= 396Mhz
const clock_arm_pll_config_t g_ccmConfigCPUPll_198Mhz = {.loopDivider = 33U}; //CPU clock = 24x33/2/2= 198Mhz

volatile uint32_t task_delay_time=TASK_CPU_MID_LOW_LOADING;//loading start from TASK_CPU_MID_LOW_LOADING

/*******************************************************************************
* Code
******************************************************************************/
void GPIO_IRQHandler(uint32_t giccIar, void *param)
{
  /* Default: Put the SW key to the OFF state*/
  if ((1U << 11) & GPIO_GetPinsInterruptFlags(GPIO3))
  {
    GPIO_ClearPinsInterruptFlags(GPIO3, 1U << 11);
    freq_up_flag=1;
    if(task_delay_time==TASK_CPU_LOW_LOADING)
    {
      task_delay_time = TASK_CPU_MID_LOW_LOADING ;
    }
    else  if(task_delay_time==TASK_CPU_MID_LOW_LOADING)
    {
      task_delay_time = TASK_CPU_MID_HIGH_LOADING ;
    }
    else  if(task_delay_time==TASK_CPU_MID_HIGH_LOADING)
    {
      task_delay_time = TASK_CPU_HIGH_LOADING ;
    }
  }
  
  if ((1U << 12) & GPIO_GetPinsInterruptFlags(GPIO3))
  {
    GPIO_ClearPinsInterruptFlags(GPIO3, 1U << 12);
    freq_down_flag=1;
    if(task_delay_time==TASK_CPU_HIGH_LOADING)
    {
      task_delay_time = TASK_CPU_MID_HIGH_LOADING ;
    }
    else  if(task_delay_time==TASK_CPU_MID_HIGH_LOADING)
    {
      task_delay_time = TASK_CPU_MID_LOW_LOADING ;
    }
    else  if(task_delay_time==TASK_CPU_MID_LOW_LOADING)
    {
      task_delay_time = TASK_CPU_LOW_LOADING ;
    }
  }	
  
}


/*Provides own implementation of vApplicationIRQHandler() to call SystemIrqHandler()
*directly without saving the FPU registers on interrupt entry.
*/
void vApplicationIRQHandler(uint32_t ulICCIAR)
{
  SystemIrqHandler(ulICCIAR);
}

/*#define configUSE_IDLE_HOOK 1 for idle callback test*/
void vApplicationIdleHook(void)
{
  idle_cycle_cnt++;
}
/*!
* @brief Application entry point.
*/
int main(void)
{
  TimerHandle_t SwTimerHandle = NULL;
  
  /* Define the init structure for the input switch pin */
  gpio_pin_config_t swConfig = {
    kGPIO_DigitalInput, 0, kGPIO_IntRisingEdge,
  };    
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitMemory();
  BOARD_InitDebugConsole();
  
  /* Install IRQ Handler */
  SystemInitIrqTable();
  
  /* Init input switch GPIO. */
  GPIO_PinInit(GPIO3, 11, &swConfig);
  GPIO_PinInit(GPIO3, 12, &swConfig);
  
  /* Enable SW Interrupt */
  SystemInstallIrqHandler(GPIO3_Combined_0_15_IRQn, GPIO_IRQHandler, NULL);
  EnableIRQ(GPIO3_Combined_0_15_IRQn);
  GPIO_EnableInterrupts(GPIO3, 1U << 11);
  GPIO_EnableInterrupts(GPIO3, 1U << 12);
  GPIO_ClearPinsInterruptFlags(GPIO3, 1U << 11);
  GPIO_ClearPinsInterruptFlags(GPIO3, 1U << 12);  
  
  PRINTF("DVFS Simulation Test!\r\n");
  
  arm_freq_setpoint(396000000);//freq set to 396M
  arm_voltage_setpoint(0xC); //ARM_CAP set to 1.0V   
  
  /* Create the software timer. */
  SwTimerHandle = xTimerCreate("SwTimer",          /* Text name. */
                               SW_TIMER_PERIOD_MS, /* Timer period. */
                               pdTRUE,             /* Enable auto reload. */
                               0,                  /* ID is not used. */
                               SwTimerCallback);   /* The callback function. */
  /* Start timer. */
  xTimerStart(SwTimerHandle, 0);
  xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);
  vTaskStartScheduler();
  for (;;)
    ;
}

/*!
* @brief Task responsible for printing of "Hello world." message.
*/
static void hello_task(void *pvParameters)
{
  for (;;)
  {
    if(change_flag == 1)
    {
      freq = CLOCK_GetFreq(kCLOCK_CpuClk);
      arm_reg_targ =  (PMU->REG_CORE) & 0xff; 
      if(arm_reg_targ==0x12)	 vol=1.15;
      else if(arm_reg_targ==0xC)	 vol=1.00;	
      else vol=0.925;;	
      change_flag=0;	
    }
    PRINTF("CPU Core Clock = %dHz, ARM voltage = %f V \r\n", freq,vol);
    vTaskDelay(task_delay_time);
    vTaskGetRunTimeStats(pcWriteBuffer);
    PRINTF("%s\r\n", pcWriteBuffer);
  }
}

/*!
* @brief Software timer callback.
*/
static void SwTimerCallback(TimerHandle_t xTimer)
{
  
  freq = CLOCK_GetFreq(kCLOCK_CpuClk);
  
  if(freq_down_flag==1)
  {
    if(freq==528000000)
    {
      arm_freq_setpoint(396000000);//freq set to 396M
      arm_voltage_setpoint(0xC); //ARM_CAP set to 1.0V    
    }
    else if(freq==396000000)
    {
      arm_freq_setpoint(198000000);//freq set to 198M
      arm_voltage_setpoint(0x9); //ARM_CAP set to 1.0V    
    }
    freq_down_flag = 0;
    change_flag = 1;
  }
  else if(freq_up_flag==1)
  {
    if(freq==396000000)
    {
      arm_voltage_setpoint(0x12); //ARM_CAP set to 1.15V    
      arm_freq_setpoint(528000000);//freq set to 528M
    }
    else if(freq==198000000)
    {
      arm_voltage_setpoint(0xC); //ARM_CAP set to 1.0V    
      arm_freq_setpoint(396000000);//freq set to 396M
    }
    freq_up_flag = 0;
    change_flag = 1;
  }  
}


//config target voltage for the ARM core power domain
int arm_voltage_setpoint(uint8_t target_vol)  //voltage to be set
{
  arm_reg_targ =  (PMU->REG_CORE) & 0xff; // get current voltage for the ARM core power domain
  
  if(target_vol>0x12||target_vol<0x9) return -1;
  
  if(arm_reg_targ > target_vol )
  {
    for(i=0, j= arm_reg_targ;i<=(arm_reg_targ-target_vol);i++)
    {
      PMU->REG_CORE = (PMU_REG_CORE_REG2_TARG(0x12) |PMU_REG_CORE_REG0_TARG(j));
      j--;
    }
  }	
  else
  {
    for(i=0, j= arm_reg_targ;i<=(target_vol-arm_reg_targ);i++)
    {
      PMU->REG_CORE = (PMU_REG_CORE_REG2_TARG(0x12) | PMU_REG_CORE_REG0_TARG(j));
      j++;
    }
  }
  return 0;
}

//config target freq for the ARM core in CCM
int arm_freq_setpoint(uint32_t target_freq)  //freq to be set
{
  //Change the CPU clock
  if (CLOCK_GetMux(kCLOCK_Pll1SwMux) == 0) /* CPU runs on ARM PLL */
  {
    CLOCK_SetMux(kCLOCK_StepMux, 0);   /* Set Step MUX to OSC */
    CLOCK_SetMux(kCLOCK_Pll1SwMux, 1); /* Let CPU run on Step MUX */
  }
  
  if(target_freq == 528000000)
  {
    CLOCK_InitArmPll(&g_ccmConfigCPUPll_528Mhz); /* Configure ARM PLL to 1056Mhz */
  }
  else if(target_freq == 396000000)
  {
    CLOCK_InitArmPll(&g_ccmConfigCPUPll_396Mhz); /* Configure ARM PLL to 792Mhz */
  }
  else if(target_freq == 198000000)
  {
    CLOCK_InitArmPll(&g_ccmConfigCPUPll_198Mhz); /* Configure ARM PLL to 396Mhz */
  }
  else
  {
    return -1;
  }
  
  CLOCK_SetMux(kCLOCK_Pll1SwMux, 0);    /* Now CPU runs again on ARM PLL at 528Mhz/396M/198Mhz (with divider 2) */
  return 0;
}  






