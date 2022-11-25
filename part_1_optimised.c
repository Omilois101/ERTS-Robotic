/*
Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,.li7f
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Author:      Mohd A. Zainol, Reza Nejabati
// Date:        10/10/2018
// Chip:        MSP432P401R LaunchPad Development Kit (MSP-EXP432P401R) for TI-RSLK
// File:        Part1_Interrupt_bumpsw_motor.h
// Function:    Part 1 of ERTS, uses interrupt for bump switches to control the motor

// Libraries Included to Design
#include <stdint.h>
#include "msp.h"


// Led Color Choices
#define RED       0x01
#define GREEN     0x02
#define YELLOW    0x03
#define BLUE      0x04
#define PINK      0x05
#define SKYBLUE   0x06
#define WHITE     0x07


#define SW1IN ((*((volatile uint8_t *)(0x42098004)))^1)
#define SW2IN ((*((volatile uint8_t *)(0x42098010)))^1) // input: switch SW2
#define REDLED (*((volatile uint8_t *)(0x42098040)))    // output: red LED

char Mode;

// Initialize Bump sensors using interrupt
// Make six from Port 4 input pins
// Activate interface pull-up
// The pins are P4.7, 4.6, 4.5, 4.3, 4.2, 4.0
void BumpEdgeTrigger_Init(void){
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;      // configure as GPIO
    P4->DIR &= ~0xED;       // make in
    P4->REN |= 0xED;        // enable pull resistors
    P4->OUT |= 0xED;        // pull-up
    P4->IES |= 0xED;        // falling edge event
    P4->IFG &= ~0xED;       // clear flag
    P4->IE |= 0xED;         // arm the interrupt
    // priority 2 on port4
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000;
    // enable interrupt 38 in NVIC on port4
    NVIC->ISER[1] = 0x00000040;
}


// Uses P4IV IRQ handler to solve critical section/race
void PORT4_IRQHandler(void){

    uint8_t status;

    // The movement for coloured LED
    // WHITE:   Forward
    // BLUE:    Turn right
    // YELLOW:  Turn left
    // GREEN:   Backward

     // Interrupt Vector of Port4
      status = P4->IV;          // 2*(n+1) where n is highest priority
      Port2_Init();            // Initialize Port 2
      Motor_InitSimple();       // Initialize DC Motor
      switch(status){

        case 0x02: // Bump switch 1
            Port2_Output(RED);  // Change the coloured LED into Red (bump switch 1)
            Motor_StopSimple(80); // Stop to show bump switch triggered
            if(Mode == 'f'){
                Port2_Output(GREEN);             //  coloured LED set to green (backward)
                Motor_BackwardSimple(500, 200); // Move backward at 500 duty for 200ms
                Port2_Output(0);             // turn off the coloured LED
                Motor_StopSimple(100);       // Stop for 1000ms
                Port2_Output(YELLOW);        // Change the coloured LED into yellow (turn left)
                Motor_LeftSimple(500, 100);  // Make a left turn at 500 duty for 100ms
                Port2_Output(0);             // turn off the coloured LED
                Motor_StopSimple(100);       // Stop for 1000ms
                Clock_Delay1ms(10);        // delay for 10ms
         }else{
             Port2_Output(0);
             do{
                 Motor_StopSimple(1); // Stops motor
                 if ((SW2IN & SW1IN)){
                     break;
                 }
             }while(1);
         }
         break;
        case 0x06: // Bump switch 2
          Port2_Output(GREEN); // Change the coloured LED into green to show bump switch 2
          Motor_StopSimple(80); // Stop to show bump switch triggered
          if(Mode == 'f'){
            Port2_Output(GREEN);     // Change the coloured LED into green (backward)
            Motor_BackwardSimple(500, 200);      // Move backward at 500 duty for 200ms
            Port2_Output(0);               // turn off the coloured LED
            Motor_StopSimple(100);         // Stop for 1000ms
            Port2_Output(YELLOW);         // Change the coloured LED into yellow (turn left)
            Motor_LeftSimple(500, 200);    // Make a left turn at 500 duty for 200ms
            Port2_Output(0);              // turn off the coloured LED
            Motor_StopSimple(100);    // Stop for 1000ms
            Clock_Delay1ms(10) ;         // delay for 10ms
          }else{
              Port2_Output(0);    // turn off the coloured LED
              do{
                  Motor_StopSimple(1); // Stop for 1ms
                  if ((SW2IN & SW1IN)){
                      break;
                  }
              }while(1);
          }
          break;
        case 0x08: // Bump switch 3
          Port2_Output(BLUE);  // Change the coloured LED into blue to show bump switch 3
         Motor_StopSimple(80); //stop to show bump switch triggered
          if(Mode == 'f'){
             Port2_Output(GREEN); // Change the coloured LED into green (backward)
            Motor_BackwardSimple(500, 200);     // Move backward at 500 duty for 200ms
            Port2_Output(0);            // turn off the coloured LED
            Motor_StopSimple(100);      // Stop for 1000ms
            Port2_Output(YELLOW);         // Change the coloured LED into yellow (turn left)
            Motor_LeftSimple(500, 300);   // Make a left turn at 500 duty for 300ms
            Port2_Output(0);             // turn off the coloured LED
            Motor_StopSimple(100);       // Stop for 1000ms
            Clock_Delay1ms(10);          // delay for 10ms
           }else{
               Port2_Output(0);    // turn off the coloured LED
               do{
                   Motor_StopSimple(1); // Stop for 1ms
                   if ((SW2IN & SW1IN)){
                       break;
                   }
               }while(1);
           }
          break;
        case 0x0c: // Bump switch 4
            Port2_Output(YELLOW);   // Change the coloured LED into yellow to show bump switch 4
           Motor_StopSimple(80); //stop to show bump switch triggered
            if(Mode == 'f'){
               Port2_Output(GREEN); // Change the coloured LED into green (backward)
              Motor_BackwardSimple(500, 200);     // Move backward at 500 duty for 200ms
              Port2_Output(0);            // turn off the coloured LED
              Motor_StopSimple(100);      // Stop for 1000ms
              Port2_Output(BLUE);         // Change the coloured LED into blue (turn right)
              Motor_LeftSimple(500, 300);   // Make a right turn at 500 duty for 300ms
              Port2_Output(0);             // turn off the coloured LED
              Motor_StopSimple(100);       // Stop for 1000ms
              Clock_Delay1ms(10);          // delay for 10ms
             }else{
                 Port2_Output(0);    // turn off the coloured LED
                 do{
                     Motor_StopSimple(1); // Stop for 1ms
                     if ((SW2IN & SW1IN)){
                         break;
                     }
                 }while(1);
             }
            break;
        case 0x0e: // Bump switch 5
         Port2_Output(PINK); // Change the coloured LED into pink to show bump switch 5
         Motor_StopSimple(80); //stop to show bump switch triggered
         if(Mode == 'f'){
            Port2_Output(GREEN);             // Change the coloured LED into green (backward)
            Motor_BackwardSimple(500, 200);  // Move backward at 500 duty for 200ms
            Port2_Output(0);             // turn off the coloured LED
            Motor_StopSimple(100); // Stop for 1000ms
            Port2_Output(BLUE);          // Change the coloured LED into blue (turn right)
            Motor_RightSimple(500, 200);             // Make a right turn at 500 duty for 200ms
            Port2_Output(0); // turn off the coloured LED
            Motor_StopSimple(100); // Stop for 1000ms
            Clock_Delay1ms(10);          // delay for 10ms
           }else{
               Port2_Output(0);    // turn off the coloured LED
               do{
                   Motor_StopSimple(1); // Stop for 1ms
                   if ((SW2IN & SW1IN)){
                       break;
                   }
               }while(1);
            }
          break;
        case 0x10: // Bump switch 6
            Port2_Output(SKYBLUE);  // Change the coloured LED into pink to show bump switch 5
            Motor_StopSimple(80); //stop to show bump switch triggered
            if(Mode == 'f'){
               Port2_Output(GREEN); // Change the coloured LED into green (backward)
               Motor_BackwardSimple(500, 200); // Move backward at 500 duty for 200ms
               Port2_Output(0); // turn off the coloured LED
               Motor_StopSimple(100);  // Stop for 1000ms
               Port2_Output(BLUE); // Change the coloured LED into blue (turn right)
               Motor_RightSimple(500, 100);  // Make a left turn at 500 duty for 100ms
               Port2_Output(0); // turn off the coloured LED
               Motor_StopSimple(100);  // Stop for 1000ms
               break;
            }else{
                Port2_Output(0);    // turn off the coloured LED
                do{
                    Motor_StopSimple(1); // Stop for 1ms
                    if ((SW2IN & SW1IN)){
                        break;
                    }
                  }while(1);
           }
        break;
        case 0xED: // none of the switches are pressed
          break;
      }
      P4->IFG &= ~0xED; // clear flag
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read_Input(void){
  return (P4->IN&0xED); // read P4.7, 4.6, 4.5, 4.3, 4.2, 4.0 inputs
}


// bit-banded addresses, positive logic

// Function: checkbumpswitch(uint8_t status)
// Description: this is an alternative way that you can use, 
//              in which it uses polling method that comes from main function.
//              However it is important to note that:
//              1) the polling method is only useful for small program
//              2) the input mask in switch case (for polling method) is DIFFERENT from the 
//                 Nested Vectored Interrupt Controller (NVIC) which used in interrupt method.
void checkbumpswitch(uint8_t status)
{
      Port2_Init();            // Initialize Port 2
      Motor_InitSimple();       // Initialize DC Motor
      switch(status){

        case 0xEC: // Bump switch 1
            Port2_Output(RED);  // Change the coloured LED into Red (bump switch 1)
            Motor_StopSimple(10); // Stop to show bump switch triggered
            if(Mode == 'f'){
                Port2_Output(GREEN);             //  coloured LED set to green (backward)
                Motor_BackwardSimple(500, 200); // Move backward at 500 duty for 200ms
                Port2_Output(0);             // turn off the coloured LED
                Motor_StopSimple(100);       // Stop for 1000ms
                Port2_Output(YELLOW);        // Change the coloured LED into yellow (turn left)
                Motor_LeftSimple(500, 100);  // Make a left turn at 500 duty for 100ms
                Port2_Output(0);             // turn off the coloured LED
                Motor_StopSimple(100);       // Stop for 1000ms
                Clock_Delay1ms(10);          // delay for 10ms
         }else{
             Port2_Output(0);
             do{
                 Motor_StopSimple(1); // Stops motor
                 if ((SW2IN & SW1IN)){
                     break;
                 }
             }while(1);
         }
         break;
        case 0xE9: // Bump switch 2
          Port2_Output(GREEN); // Change the coloured LED into green to show bump switch 2
         Motor_StopSimple(80); //stop to show bump switch triggered
          if(Mode == 'f'){
            Port2_Output(GREEN);     // Change the coloured LED into green (backward)
            Motor_BackwardSimple(500, 200);      // Move backward at 500 duty for 200ms
            Port2_Output(0);               // turn off the coloured LED
            Motor_StopSimple(100);         // Stop for 1000ms
            Port2_Output(YELLOW);         // Change the coloured LED into yellow (turn left)
            Motor_LeftSimple(500, 200);    // Make a left turn at 500 duty for 200ms
            Port2_Output(0);              // turn off the coloured LED
            Motor_StopSimple(100);    // Stop for 1000ms
            Clock_Delay1ms(10);          // delay for 10ms
          }else{
              Port2_Output(0);    // turn off the coloured LED
              do{
                  Motor_StopSimple(1); // Stop for 1ms
                  if ((SW2IN & SW1IN)){
                      break;
                  }
              }while(1);
          }
          break;
        case 0xE5: // Bump switch 3
          Port2_Output(BLUE);  // Change the coloured LED into blue to show bump switch 3
         Motor_StopSimple(80); //stop to show bump switch triggered
          if(Mode == 'f'){
             Port2_Output(GREEN); // Change the coloured LED into green (backward)
            Motor_BackwardSimple(500, 200);     // Move backward at 500 duty for 200ms
            Port2_Output(0);            // turn off the coloured LED
            Motor_StopSimple(100);      // Stop for 1000ms
            Port2_Output(YELLOW);         // Change the coloured LED into yellow (turn left)
            Motor_LeftSimple(500, 300);   // Make a left turn at 500 duty for 300ms
            Port2_Output(0);             // turn off the coloured LED
            Motor_StopSimple(100);       // Stop for 1000ms
            Clock_Delay1ms(10);          // delay for 10ms
           }else{
               Port2_Output(0);    // turn off the coloured LED
               do{
                   Motor_StopSimple(1); // Stop for 1ms
                   if ((SW2IN & SW1IN)){
                       break;
                   }
               }while(1);
           }
          break;
        case 0xCD: // Bump switch 4
            Port2_Output(YELLOW);   // Change the coloured LED into yellow to show bump switch 4
           Motor_StopSimple(80); //stop to show bump switch triggered
            if(Mode == 'f'){
               Port2_Output(GREEN); // Change the coloured LED into green (backward)
              Motor_BackwardSimple(500, 200);     // Move backward at 500 duty for 200ms
              Port2_Output(0);            // turn off the coloured LED
              Motor_StopSimple(100);      // Stop for 1000ms
              Port2_Output(BLUE);         // Change the coloured LED into blue (turn right)
              Motor_LeftSimple(500, 300);   // Make a right turn at 500 duty for 300ms
              Port2_Output(0);             // turn off the coloured LED
              Motor_StopSimple(100);       // Stop for 1000ms
              Clock_Delay1ms(10);          // delay for 10ms
             }else{
                 Port2_Output(0);    // turn off the coloured LED
                 do{
                     Motor_StopSimple(1); // Stop for 1ms
                     if ((SW2IN & SW1IN)){
                         break;
                     }
                 }while(1);
             }
            break;
        case 0xAD: // Bump switch 5
         Port2_Output(PINK); // Change the coloured LED into pink to show bump switch 5
         Motor_StopSimple(80); //stop to show bump switch triggered
         if(Mode == 'f'){
            Port2_Output(GREEN);             // Change the coloured LED into green (backward)
            Motor_BackwardSimple(500, 200);  // Move backward at 500 duty for 200ms
            Port2_Output(0);             // turn off the coloured LED
            Motor_StopSimple(100); // Stop for 1000ms
            Port2_Output(BLUE);          // Change the coloured LED into blue (turn right)
            Motor_RightSimple(500, 200);             // Make a right turn at 500 duty for 200ms
            Port2_Output(0); // turn off the coloured LED
            Motor_StopSimple(100); // Stop for 1000ms
            Clock_Delay1ms(10);          // delay for 10ms
           }else{
               Port2_Output(0);    // turn off the coloured LED
               do{
                   Motor_StopSimple(1); // Stop for 1ms
                   if ((SW2IN & SW1IN)){
                       break;
                   }
               }while(1);
            }
          break;
        case  0x6D: // Bump switch 6
            Port2_Output(SKYBLUE);  // Change the coloured LED into pink to show bump switch 5
            Motor_StopSimple(80); //stop to show bump switch triggered
            if(Mode == 'f'){
               Port2_Output(GREEN); // Change the coloured LED into green (backward)
               Motor_BackwardSimple(500, 200); // Move backward at 500 duty for 200ms
               Port2_Output(0); // turn off the coloured LED
               Motor_StopSimple(100);  // Stop for 1000ms
               Port2_Output(BLUE); // Change the coloured LED into blue (turn right)
               Motor_RightSimple(500, 100);  // Make a left turn at 500 duty for 100ms
               Port2_Output(0); // turn off the coloured LED
               Motor_StopSimple(100);  // Stop for 1000ms
               break;
            }else{
                Port2_Output(0);    // turn off the coloured LED
                do{
                    Motor_StopSimple(1); // Stop for 1ms
                    if ((SW2IN & SW1IN)){
                        break;
                    }
                  }while(1);
           }
        break;
        case 0xED: // none of the switches are pressed
          break;
      }
}

void Port1_Init(void){
  P1->SEL0 &= ~0x01;
  P1->SEL1 &= ~0x01;        // configure P1.0 as GPIO
  P1->DIR |= 0x01;          //make P1.0 out, the built-in LED1
}

void Port2_Init(void){
    P2->SEL0 &= ~0xC7;
    P2->SEL1 &= ~0xC7;        // configure P2.2 P2.1 P2.0, and P2.6 P2.7 as GPIO
    P2->DIR |= 0x07;          // make P2.2-P2.0 out
    P2->DS |= 0x07;           // activate increased drive strength
    P2->OUT &= ~0x07;         // all LEDs off
    P2->DIR |= 0xC0;          // Direction of the motor
}

void Port2_Output(uint8_t data){
    // built-in red LED connected to P2.0
    // built-in green LED connected to P2.1
    // built-in blue LED connected to P2.2
    // write three outputs bits of P2
    P2->OUT = (P2->OUT&0xF8)|data;
}

void Switch_Init(void){
    // negative logic built-in Button 1 connected to P1.1
    // negative logic built-in Button 2 connected to P1.4
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;      // configure P1.4 and P1.1 as GPIO
    P1->DIR &= ~0x12;       // make P1.4 and P1.1 in
    P1->REN |= 0x12;        // enable pull resistors on P1.4 and P1.1
    P1->OUT |= 0x12;        // P1.4 and P1.1 are pull-up
}






void Move_Interrupt(void){
    BumpEdgeTrigger_Init();
    do{
        Port2_Output(WHITE);      // White is the colour to represent moving forward
        Motor_ForwardSimple(500,1);  // Motor moves forward for 1ms
        if ((SW2IN & SW1IN)){
            break;
        }
    }while(1);
    Port2_Output(0);
}




int main(void){

  Clock_Init48MHz();        // Initialize clock with 48MHz frequency
  Switch_Init();            // Initialize switches
  SysTick_Init();           // Initialize SysTick timer

  Port1_Init();             // Initialize P1.1 and P1.4 built-in buttons
  Port2_Init();             // Initialize P2.2-P2.0 built-in LEDs
  REDLED = 0;               // Turn off the red LED

  Motor_InitSimple();       // Initialize DC Motor
  BumpEdgeTrigger_Init();    // Initialize Bump Triggers

  EnableInterrupts();       // Clear the I bit

  do{
       __no_operation();      // the code will run without operation

      if (!(SW2IN | SW1IN)){
          SysTick_Wait10ms(10); // Wait here for every 100ms
          REDLED = !REDLED;     // The red LED is blinking waiting for command
          continue;
      }if(SW1IN){
          Move_Interrupt();
          Clock_Delay1ms(200);
          break;
      }
      if(SW2IN){
          Mode = 'f';
          Move_Interrupt();
          Clock_Delay1ms(200);
          break;
      }
  }while(1);

  SysTick_Wait10ms(100);
  DisableInterrupts();

  BumpEdgeTrigger_Init();
  Port2_Init();             // Initialise P2.2-P2.0 built-in LEDs

 while(1){

    uint8_t status;
      // Run forever
      // This section is used for Example 1 (seciton 5.8.1)
          if(SW1IN){
                  do{
                      __no_operation();
                      status = Bump_Read_Input();
                      if (status == 0x6D || status == 0xAD || status == 0xCD || status == 0xE5 || status == 0xE9 || status == 0xEC) {
                            checkbumpswitch(status);
                      }
                      Port2_Output(WHITE);      // White is the colour to represent moving forward
                      Motor_ForwardSimple(500,1);  // Motor moves forward for 1ms
                      if ((SW2IN & SW1IN)){
                          Port2_Output(0);
                          break;
                      }
                  }while(1);
              Clock_Delay1ms(1000);
              break;
          }else  if(SW2IN){
              do{
                  __no_operation();
                  status = Bump_Read_Input();
                  if (status == 0x6D || status == 0xAD || status == 0xCD || status == 0xE5 || status == 0xE9 || status == 0xEC) {
                        checkbumpswitch(status);
                  }
                  Mode = 'f';
                  Port2_Output(WHITE);      // White is the colour to represent moving forward
                  Motor_ForwardSimple(500,1);
                  if ((SW2IN & SW1IN)){
                      Port2_Output(0);
                      break;
                  }
              }while(1);
              break;
          }else if (!(SW2IN | SW1IN)){
                       SysTick_Wait10ms(10); // Wait here for every 100ms
                       REDLED = !REDLED;     // The red LED is blinking waiting for command
                       continue;
                   }
 }

  SysTick_Wait10ms(100);
  Port2_Output(0);
  while(1){
  do{            // Wait for SW2 switch
        SysTick_Wait10ms(10); // Wait here for every 100ms
        REDLED = 1;     // The red LED is blinking waiting for command
        }while(!(SW2IN | SW1IN));
}
}
