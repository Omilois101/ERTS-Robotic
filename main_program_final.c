
/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

// Author:      Mohd A. Zainol
// Date:        1 Oct 2018
// Chip:        MSP432P401R LaunchPad Development Kit (MSP-EXP432P401R) for TI-RSLK
// File:        main_program.c
// Function:    The main function of our code in FreeRTOS

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* TI includes. */
#include "gpio.h"

/* ARM Cortex */
#include <stdint.h>
#include "msp.h"
#include "SysTick.h"
#include "inc/CortexM.h"

#include "inc/songFile.h"   //   include the songFile header file
#include "inc/dcMotor.h"    //   include the dcMotor header file
#include "inc/bumpSwitch.h" //   include the bumpSwitch header file
#include "inc/outputLED.h"  //   include the outputLED header file
#include "inc/SysTick.h"    //   include the SysTick header file

#define SW1IN ((*((volatile uint8_t *)(0x42098004)))^1) //   bit-banded addresses positive logic of input switch S1
#define SW2IN ((*((volatile uint8_t *)(0x42098010)))^1) //   bit-banded addresses positive logic of input switch S2
#define REDLED (*((volatile uint8_t *)(0x42098040)))

uint8_t bumpSwitch_status;              // global variable to read bump switches value
SemaphoreHandle_t xbSemaphore= NULL;    // Semaphore Declaration


void main_program( void );              //Called by main() to create the main program application
static void Switch_Init(void);
static void prvConfigureClocks( void ); //The configuration of clocks for frequency

//-----------------------------------------------Shared Tasks------------------------------------------------------//
static void taskReadInputSwitch( void *pvParameters );  // a static void function for a task called "taskReadInputSwitch"
static void taskPlaySong( void *pvParameters );         // a static void function for a task called "taskPlaySong"
static void taskMasterThread( void *pvParameters );     // a static void function for a task called "taskMasterThread"

//-----------------------------------------------Polling Tasks------------------------------------------------------//
static void taskBumpSwitch( void *pvParameters );       // a static void function for a task called "taskBumpSwitch"
static void taskdcMotor( void *pvParameters );          // a static void function for a task called "taskdcMotor"
static void taskDisplayOutputLED( void *pvParameters ); // a static void function for a task called "taskDisplayOutputLED"

//-----------------------------------------------Interrupt Tasks----------------------------------------------------//
static void taskdcMotor_interrupts( void *pvParameters );
static void task_interrupts( void *pvParameters );


//-----------------------------------------------Shared Tasks------------------------------------------------------//
xTaskHandle taskHandle_BlinkRedLED;  // declare an identifier of task handler called "taskHandle_BlinkRedLED"
xTaskHandle taskHandle_PlaySong;     //   declare an identifier of task handler called "taskHandle_PlaySong"
xTaskHandle taskHandle_InputSwitch;  //   declare an identifier of task handler called "taskHandle_InputSwitch"

//-----------------------------------------------Polling Tasks------------------------------------------------------//
xTaskHandle taskHandle_BumpSwitch;   //   declare an identifier of task handler called "taskHandle_BumpSwitch"
xTaskHandle taskHandle_dcMotor;      //   declare an identifier of task handler called "taskHandle_dcMotor"
xTaskHandle taskHandle_OutputLED;    //   declare an identifier of task handler called "taskHandle_OutputLED"

//-----------------------------------------------Interrupt Tasks----------------------------------------------------//
xTaskHandle taskHandle_dcMotor_interrupts;
xTaskHandle taskHandle_interrupts;


void main_program( void )
{
    prvConfigureClocks();   // initialise the clock configuration
    Switch_Init();          //   initialise the switch
    SysTick_Init();         //   initialise systick timer
    RedLED_Init();          // initialise the red LED
    int i;
    int counter;

     do{
         // when no switch is pressed
         if (!(SW2IN | SW1IN)){
             for (i=0; i<1000000; i++);
             REDLED = 1;     // The red LED is blinking waiting for command
             continue;
         // time counter for switch 1 to detect the hold time when switch 1 is pressed
         }if(SW1IN){
             counter = 0;
             while(SW1IN){
                 SysTick_Wait10ms(10);
                 counter++;
             }
             if (counter < 5){ // Enter Polling Mode if Switch 1 is pressed but not hold for more than 5sec
                 xTaskCreate(taskMasterThread, "taskT", 128, NULL, 2, &taskHandle_BlinkRedLED);
                 xTaskCreate(taskPlaySong, "taskS", 128, NULL, 1, &taskHandle_PlaySong);
                 xTaskCreate(taskReadInputSwitch, "taskR", 128, NULL, 1, &taskHandle_InputSwitch);
                 xTaskCreate(taskBumpSwitch, "taskB", 128, NULL, 1, &taskHandle_BumpSwitch);
                 xTaskCreate(taskdcMotor, "taskM", 128, NULL, 1, &taskHandle_dcMotor);
                 xTaskCreate(taskDisplayOutputLED, "taskD", 128, NULL, 1, &taskHandle_OutputLED);
                 break;
             }
             else if (counter > 5){ // Enter Interrupt Mode if Switch 1 is pressed for more than 5sec
                 // Create a Semaphore
                 xbSemaphore =  xSemaphoreCreateBinary();

                 // Instantiation for Interrupts
                 BumpSwitch_Init();
                 BumpSwitch_Interupt_Init();
                 xTaskCreate(taskMasterThread, "taskT", 128, NULL, 2, &taskHandle_BlinkRedLED);
                 xTaskCreate(taskPlaySong, "taskS", 128, NULL, 1, &taskHandle_PlaySong);
                 xTaskCreate(taskReadInputSwitch, "taskR", 128, NULL, 1, &taskHandle_InputSwitch);
                 xTaskCreate(taskdcMotor_interrupts, "taskM", 128, NULL, 1, &taskHandle_dcMotor_interrupts);
                 xTaskCreate(task_interrupts, "taskD", 128, NULL, 1, &taskHandle_interrupts);
                 break;
             }
         }
     }while(1);

     vTaskStartScheduler(); //   start the scheduler

    /* INFO: If everything is fine, the scheduler will now be running,
    and the following line will never be reached.  If the following line
    does execute, then there was insufficient FreeRTOS heap memory
    available for the idle and/or timer tasks to be created. See the
    memory management section on the FreeRTOS web site for more details. */
    for( ;; );
}

// Interrupt Handler for Interrupt Mode
void PORT4_IRQHandler(void){
        bumpSwitch_status = P4->IV;      // 2*(n+1) where n is highest priority
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        P4->IFG &= ~0xED;   // clear flag
        vTaskResume(taskHandle_interrupts);
        xSemaphoreGiveFromISR(xbSemaphore, &xHigherPriorityTaskWoken ); // the interrupt handler will give a semaphore
        // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}
/*-----------------------------------------------------------------*/
/*------------------- FreeRTOS configuration ----------------------*/
/*-------------   DO NOT MODIFY ANYTHING BELOW HERE   -------------*/
/*-----------------------------------------------------------------*/
// The configuration clock to be used for the board
static void prvConfigureClocks( void )
{
    // Set Flash wait state for high clock frequency
    FlashCtl_setWaitState( FLASH_BANK0, 2 );
    FlashCtl_setWaitState( FLASH_BANK1, 2 );

    // This clock configuration uses maximum frequency.
    // Maximum frequency also needs more voltage.

    // From the datasheet: For AM_LDO_VCORE1 and AM_DCDC_VCORE1 modes,
    // the maximum CPU operating frequency is 48 MHz
    // and maximum input clock frequency for peripherals is 24 MHz.
    PCM_setCoreVoltageLevel( PCM_VCORE1 );
    CS_setDCOCenteredFrequency( CS_DCO_FREQUENCY_48 );
    CS_initClockSignal( CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
}

// The sleep processing for MSP432 board
void vPreSleepProcessing( uint32_t ulExpectedIdleTime ){}

#if( configCREATE_SIMPLE_TICKLESS_DEMO == 1 )
    void vApplicationTickHook( void )
    {
        /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */
        /* Only the full demo uses the tick hook so there is no code is
        executed here. */
    }
#endif
/*-----------------------------------------------------------------*/
/*-------------   DO NOT MODIFY ANYTHING ABOVE HERE   -------------*/
/*--------------------------- END ---------------------------------*/
/*-----------------------------------------------------------------*/

static void Switch_Init(void){
// negative logic built-in Button 1 connected to P1.1
// negative logic built-in Button 2 connected to P1.4
P1->SEL0 &= ~0x12;
P1->SEL1 &= ~0x12;      // configure P1.4 and P1.1 as GPIO
P1->DIR &= ~0x12;       // make P1.4 and P1.1 in
P1->REN |= 0x12;        // enable pull resistors on P1.4 and P1.1
P1->OUT |= 0x12;        // P1.4 and P1.1 are pull-up
}

// --------------------------------------Shared Task Functions ------------------------------------------
static void taskReadInputSwitch( void *pvParameters ){ //STOP MUSIC
    char i_SW1=0;
    int i;

    for( ;; )
    {
        if (SW1IN == 1) {
            i_SW1 ^= 1;                 // toggle the variable i_SW1 (0,1)
            for (i=0; i<1000000; i++);  // this waiting loop is used
                                        // to prevent the switch bounce.
        }
        if (i_SW1 == 1) {
            REDLED = 1;     // turn on the red LED
            vTaskSuspend(taskHandle_PlaySong); //   suspend the task taskHandle_PlaySong
        }
        else {
            REDLED = 0;     // turn off the red LED
            vTaskResume (taskHandle_PlaySong); //   resume the task taskHandle_PlaySong
        }

    }
}

static void taskMasterThread( void *pvParameters )
{
    int i;

    ColorLED_Init();  //   initialise the color LED
    RedLED_Init();   // initialise the red LED
//    BumpSwitch_Init();
 //   EnableInterrupts();
    while(!SW2IN){                  // Wait for SW2 switch
        for (i=0; i<1000000; i++);  // Wait here waiting for command
        REDLED = !REDLED;           // The red LED is blinking
    }

    REDLED = 0;     //   Turn off the RED LED, we no longer need that
    vTaskDelete(taskHandle_BlinkRedLED);     //   This function (taskMasterThread)is no longer needed

};

static void taskPlaySong( void *pvParameters ){
    for( ;; )
    {
        init_song_pwm();   //   initialise the song

        play_song();      //   play the song's function and run forever
    };

};


// --------------------------------------End Shared Task Functions -----------------------------------

// --------------------------------------Polling Task Functions --------------------------------------
static void taskBumpSwitch( void *pvParameters ){
    BumpSwitch_Init(); //   initialise bump switches
    for( ;; )
    {
        bumpSwitch_status = Bump_Read_Input();    //   use bumpSwitch_status as the variable and
                                                  //       use Bump_Read_Input to read the input
    };
};


static void taskDisplayOutputLED( void *pvParameters ){
    for( ;; )
    {
        outputLED_response(bumpSwitch_status);     //   use outputLED_response as the function and
                                                   //       use bumpSwitch_status as the parameter
    }
};

static void taskdcMotor( void *pvParameters ){

    for(;;){
       dcMotor_Init(); //   initialise the DC Motor
       dcMotor_Forward(500,1);
       if (bumpSwitch_status == 0x6D || bumpSwitch_status == 0xAD || bumpSwitch_status == 0xCD || bumpSwitch_status == 0xE5 || bumpSwitch_status == 0xE9 || bumpSwitch_status == 0xEC){ //   use a polling that continuously read from the bumpSwitch_status,
           dcMotor_response(bumpSwitch_status );
       }
   }
};
// --------------------------------------End Poling Task Functions ------------------------------------


// --------------------------------------Interrupt Task Functions--------------------------------------
static void taskdcMotor_interrupts( void *pvParameters ){ // set robort to move forward

    for(;;){
        dcMotor_Init(); //   initialise the DC Motor
        dcMotor_Forward(500,1);
    }
}

static void task_interrupts( void *pvParameters ){  // ISR
      EnableInterrupts();
      for( ;; )
       {
           xSemaphoreTake(xbSemaphore, portMAX_DELAY);
           vTaskSuspend(taskHandle_PlaySong);
           vTaskSuspend(taskHandle_dcMotor_interrupts);
           dcMotor_interrupts(bumpSwitch_status);
           vTaskDelay(600);
           OutputLED_interrupts(bumpSwitch_status);
           vTaskResume(taskHandle_dcMotor_interrupts);
           vTaskDelay(600);
           vTaskResume(taskHandle_PlaySong);
           vTaskSuspend(NULL);
        }
       DisableInterrupts();
};
// --------------------------------------End Interrupt Task Functions -----------------------------------
