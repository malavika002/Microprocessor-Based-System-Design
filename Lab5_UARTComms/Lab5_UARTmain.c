// RSLK Self Test via UART

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

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
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
//#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))

// At 115200, the bandwidth = 11,520 characters/sec
// 86.8 us/character
// normally one would expect it to take 31*86.8us = 2.6ms to output 31 characters
// Random number generator
// from Numerical Recipes
// by Press et al.
// number from 0 to 31
uint32_t Random(void){
static uint32_t M=1;
  M = 1664525*M+1013904223;
  return(M>>27);
}

char WriteData,ReadData;
uint32_t NumSuccess,NumErrors;

void TestFifo(void){char data;
  while(TxFifo0_Get(&data)==FIFOSUCCESS){
    if(ReadData==data){
      ReadData = (ReadData+1)&0x7F; // 0 to 127 in sequence
      NumSuccess++;
    }else{
      ReadData = data; // restart
      NumErrors++;
    }
  }
}

uint32_t Size;

int Program5_1(void){
//int main(void){
    // test of TxFifo0, NumErrors should be zero
  uint32_t i;
  Clock_Init48MHz();
  WriteData = ReadData = 0;
  NumSuccess = NumErrors = 0;
  TxFifo0_Init();
  TimerA1_Init(&TestFifo,43);  // 83us, = 12kHz
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      TxFifo0_Put(WriteData);
      WriteData = (WriteData+1)&0x7F; // 0 to 127 in sequence
    }
    Clock_Delay1ms(10);
  }
}

char String[64];
uint32_t MaxTime,First,Elapsed;

int Program5_2(void){
//int main(void){
    // measurement of busy-wait version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  UART0_Init();
  WriteData = 'a';
  SysTick_Init(0x1000000,2); //OHL - using systick INT api
  MaxTime = 0;
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    UART0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec

    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    UART0_OutChar(CR);UART0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}

int Program5_3(void){
//int main(void){
    // measurement of interrupt-driven version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  EUSCIA0_Init();
  WriteData = 'a';
  SysTick_Init(0x1000000,2); //OHL - using systick INT api
  MaxTime = 0;
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    EUSCIA0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec
    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    EUSCIA0_OutChar(CR);EUSCIA0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}
int Program5_4(void){
//int main(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}

////LAB 4 - IR SENSOR/////
volatile uint32_t ADCflag = 0;
volatile uint32_t nr,nc,nl;
//volatile uint32_t raw17,raw12,raw16;
void SensorRead_ISR(void){  // runs at 2000 Hz
  uint32_t raw17,raw12,raw16;
  P1OUT ^= 0x01;         // profile
  P1OUT ^= 0x01;         // profile
  ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw12); // center is channel 12, P4.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  ADCflag = 1;           // semaphore
  P1OUT ^= 0x01;         // profile
}



//////////////

void TimedPause(uint32_t time){ //used for tachometer and bump-switch motor movement
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

/////LAB 4 - Tachometer/////
uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0;               // Timer A3 first edge, P10.4
int Done0;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time){
  P2_0 = P2_0^0x01;           // thread profile, P2.0
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                   // setup for next
  Done0 = 1;
}

uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2;               // Timer A3 first edge, P8.2
int Done2;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  P2_2 = P2_2^0x01;           // thread profile, P2.4
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                   // setup for next
  Done2 = 1;
}

void PeriodMeasure00(uint16_t time) {
    Period0 = 0;
}

void PeriodMeasure20(uint16_t time) {
    Period2 = 0;
}
#define PERIOD 1000

/*void toggle_GPIO(void){
    P2_4 ^= 0x01;     // create output
}*/

uint32_t main_count=0;

void Tachometer_Init(void){
    P2->SEL0 &= ~0x11;
    P2->SEL1 &= ~0x11;  // configure P2.0 and P2.4 as GPIO
    P2->DIR |= 0x11;    // P2.0 and P2.4 outputs
    First0 = First2 = 0;
    Done0 = Done2 = 0;   // set on subsequent
}
//additional function to print period
void Display_Period(uint32_t time){
    uint32_t main_count=0;
    while(time > 0){
      WaitForInterrupt();
      time--;
      main_count++;
      if(main_count%1000){
          UART0_OutString("Period0 = ");UART0_OutUDec5(Period0);UART0_OutString(" Period2 = ");UART0_OutUDec5(Period2);UART0_OutString(" \r\n");
      }
    }
}
/////////////
//////LAB 3 - Bump Switch collision Motor Movement//////

uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   P4->IFG &= ~0xED;                  // clear interrupt flags (reduce possibility of extra interrupt)
}
void MotorMovt(void){
    Motor_Forward(3000, 3000);
    UART0_OutString("Moving Straight Ahead!");
                       UART0_OutString("\r\n");
    //Write this as part of lab3 Bonus Challenge
    if(CollisionFlag){

        if (CollisionData == 0x33) {
                    UART0_OutString("Obstacle Straight Ahead!");
                    UART0_OutString("\r\n");
                    CollisionFlag = 0;
                    Motor_Right(3000,3000);
                    Clock_Delay1ms(1450);
                }

                else if (CollisionData == 0x1F) {
                   UART0_OutString("Obstacle on extreme left!");
                   UART0_OutString("\r\n");
                   CollisionFlag = 0;
                   Motor_Backward(3000,3000);
                   Clock_Delay1ms(500);
                   Motor_Right(3000,3000);
                   Clock_Delay1ms(100);
                }
                else if (CollisionData < 0x30 && CollisionData >=0x20) {
                   UART0_OutString("Hit at middle left!");
                   UART0_OutString("\r\n");
                   CollisionFlag = 0;
                   Motor_Backward(3000,3000);
                   Clock_Delay1ms(500);
                   Motor_Right(3000,3000);
                   Clock_Delay1ms(200);
                }
                else if (CollisionData < 0x38 && CollisionData >= 0x30) {
                   UART0_OutString("Obstacle centre left!");
                   UART0_OutString("\r\n");
                   CollisionFlag = 0;
                   Motor_Backward(3000,3000);
                   Clock_Delay1ms(500);
                   Motor_Right(3000,3000);
                   Clock_Delay1ms(300);
                   //CollisionFlag = 0;
                }
                else if (CollisionData < 0x3C && CollisionData >= 0x38) {
                   UART0_OutString("Obstacle centre right!");
                   UART0_OutString("\r\n");
                   CollisionFlag = 0;
                   Motor_Backward(3000,3000);
                   Clock_Delay1ms(500);
                   Motor_Left(3000,3000);
                   Clock_Delay1ms(300);
                   //CollisionFlag = 0;
                }
                else if (CollisionData < 0x3E && CollisionData >= 0x3C) {
                   UART0_OutString("Hit at middle right!");
                   UART0_OutString("\r\n");
                   CollisionFlag = 0;
                   Motor_Backward(3000,3000);
                   Clock_Delay1ms(500);
                   Motor_Left(3000,3000);
        Clock_Delay1ms(200);
                   //CollisionFlag = 0;
                }
                else if (CollisionData == 62 ) {
                   UART0_OutString("Obstacle on extreme right!");
                   UART0_OutString("\r\n");
                   Motor_Backward(3000,3000);
                   Clock_Delay1ms(500);
                   Motor_Left(3000,3000);
                   Clock_Delay1ms(100);
                }
                CollisionFlag = 0;
                Clock_Delay1ms(500);
            }


}
////////////////////
//////LAB 2 -  Reflectance Sensor/////
void sensor_reading(uint8_t data) {
    int shift_var, bit_fromdata;

    for (shift_var=7; shift_var>=0; shift_var--) {
        bit_fromdata = data >> shift_var;

        if (bit_fromdata & 1) {
            UART0_OutString("1 ");
        }
        else {
            UART0_OutString("0 ");
        }

    }
    UART0_OutString("\r\n");

}
//////////////////////
/////Reset////
void RSLK_Reset(void){
    DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  Motor_Init();
  Motor_Stop();
  LaunchPad_Init();
  Reflectance_Init();
  //IRSensor_Init();
  BumpInt_Init(&HandleCollision);
   //Bump_Init();
  //  Bumper_Init();
  Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
}

double rpm_left =0;
double rpm_right=0;
/////
// RSLK Self-Test
int main(void) {
  uint32_t cmd=0, menu=0;

  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  Motor_Init();
  Motor_Stop();
  LaunchPad_Init();
  //Bump_Init();
  //IRSensor_Init();
  Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();

  while(1){                     // Loop forever
      // write this as part of Lab 5
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR sensor test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensors Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] Extra Fn 1: Slow down in the dark Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[7] Extra Fn 2: Follow the lead Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("CMD: ");
            cmd=EUSCIA0_InUDec();
            EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      switch(cmd){
                      case 0:
                          RSLK_Reset();
                          menu=1;
                          cmd=0xDEAD;
                          break;

                          // ....
                          // ....
                      case 1:

                          DisableInterrupts();
                          Tachometer_Init();
                          //TimerA1_Init(&toggle_GPIO,10);
                          TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);
                          TimedPause(1000);
                          EnableInterrupts();

                          UART0_OutString("Test: Moving Forward ");UART0_OutString("\r\n");
                          Motor_Forward(3000,3000);
                          Clock_Delay1ms(1000);
                          //TimerA3_Stop();

                          UART0_OutString("Test: Moving Backward ");UART0_OutString("\r\n");
                          Motor_Backward(3000,3000);
                          Clock_Delay1ms(1000);
                          DisableInterrupts();
                          Period0 = Period2=0;
                          Tachometer_Init();
                          TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);
                          EnableInterrupts();
                          //TimerA3_Stop();

                          //Left Wheel
                          UART0_OutString("Left Wheel Test");UART0_OutString("\r\n");
                          Motor_Forward(3000,0);
                          Clock_Delay1ms(1000);
                          DisableInterrupts();
                          Period0 = Period2=0;
                          Tachometer_Init();
                          TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure20);
                          EnableInterrupts();
                          //TimerA3_Stop();


                          //Right Wheel
                          UART0_OutString("Right Wheel Test");UART0_OutString("\r\n");
                          Motor_Forward(0,3000);
                          Clock_Delay1ms(1000);
                          DisableInterrupts();
                          Period0 = Period2=0;
                          Tachometer_Init();
                          TimerA3Capture_Init(&PeriodMeasure00,&PeriodMeasure2);
                          EnableInterrupts();
                          //TimerA3_Stop();


                          Motor_Stop();

                          menu=1;
                          cmd=0xDEAD;
                          break;
                      case 2:
                              uint32_t raw17,raw12,raw16;
                              uint32_t s; int32_t n;
                              DisableInterrupts();
                              ADCflag = 0;
                              s = 256;
                              ADC0_InitSWTriggerCh17_12_16();
                              ADC_In17_12_16(&raw17,&raw12,&raw16);
                              LPF_Init(raw17,s);
                              LPF_Init2(raw12,s);
                              LPF_Init3(raw16,s);
                              TimerA1_Init(&SensorRead_ISR,250);
                              EnableInterrupts();
                            while(!LaunchPad_Input()) {
                                for(n=0; n<2000; n++){
                                  while(ADCflag == 0){};
                                  ADCflag = 0; // show every 2000th point
                                }
                                UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" cm,");
                                UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" cm,");
                                UART0_OutUDec5(RightConvert(nr));UART0_OutString(" cm\r\n");

                                Motor_Forward(3000,3000);
                                if (LeftConvert(nl) < 100 || CenterConvert(nc) < 100 || RightConvert(nr) < 100) {
                                    Motor_Stop();
                                }

                            }

                            TimerA1_Stop();
                            Motor_Stop();

                            menu=1;
                            cmd=0xDEAD;
                            break;
                      case 3:
                            DisableInterrupts();
                            CollisionFlag = 0;
                            BumpInt_Init(&HandleCollision);
                            TimerA1_Init(&MotorMovt, 50000);
                            TimedPause(500);
                            EnableInterrupts();
                            while(LaunchPad_Input()==0);
                            while(LaunchPad_Input());
                            //TimedPause(500);
                            TimerA1_Stop();
                            Motor_Stop();

                            menu=1;
                            cmd=0xDEAD;
                            break;
                      case 4:
                            uint8_t Data;
                            while(!LaunchPad_Input()){
                                Data = Reflectance_Read(1000);
                                sensor_reading(Data);
                                Clock_Delay1ms(500);
                            }

                            menu=1;
                            cmd=0xDEAD;
                            break;
                      case 5:
                            DisableInterrupts();
                            //UART0_Init();
                            First0 = First2 = 0;
                            Done0 = Done2 = 0;
                            //TimerA1_Init(&toggle_GPIO,10);
                            TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);
                            TimedPause(1000);
                            EnableInterrupts();

                            Motor_Forward(3000,3000);
                            Clock_Delay1ms(1000);
                            Display_Period(2000);
                            //TimerA3_Stop();

                            Motor_Stop();
                            menu=1;
                            cmd=0xDEAD;
                            break;
                      case 6:

                          EUSCIA0_OutString("This function slows down the motor if any of the reflectance sensors read black");
                          UART0_OutString("\r\n");
                          EUSCIA0_OutString("The thought behind this is for the robot to slow down at a speed bump, in real life");
                          UART0_OutString("\r\n");

                          DisableInterrupts();
                          LaunchPad_Init();
                          EnableInterrupts();
                          //
                          Data = 0;
                          int shift_var, bit_fromdata;
                          Motor_Forward(2000,2000);
                          while(!LaunchPad_Input()){
                              Data = Reflectance_Read(1000); //read reflectance sensor data
                              //convert the decimal value to binary in order to identify if surface is black or white,
                              for (shift_var=7; shift_var>=0; shift_var--) {
                                    bit_fromdata = Data >> shift_var;

                                    if (bit_fromdata & 1) {

                                        Motor_Forward(1000,1000);
                                        UART0_OutString("1 ");
                                        //EUSCIA0_OutString(" speed=1000 ");
                                        Clock_Delay1ms(500);
                                    }
                                    else {

                                        Motor_Forward(2000,2000);
                                        //EUSCIA0_OutString(" speed=3000 ");
                                        UART0_OutString("0 ");
                                        Clock_Delay1ms(500);
                                    }

                                }
                              UART0_OutString("\r\n");
                              Clock_Delay1ms(500);
                          }
                          menu=1;
                          cmd=0xDEAD;
                          break;
                      case 7:
                          uint32_t left_l,right_l,center_l;
                          EUSCIA0_OutString("This function makes the robot follow an object. We are only using center sensor");
                          UART0_OutString("\r\n");
                          EUSCIA0_OutString("Calibrate the object");
                          UART0_OutString("\r\n");
                          DisableInterrupts();
                          ADCflag = 0;
                          s = 256;
                          ADC0_InitSWTriggerCh17_12_16();
                          ADC_In17_12_16(&raw17,&raw12,&raw16);
                          LPF_Init(raw17,s);
                          LPF_Init2(raw12,s);
                          LPF_Init3(raw16,s);
                          TimerA1_Init(&SensorRead_ISR,250);
                          EnableInterrupts();
                          SensorRead_ISR();
                          while(!LaunchPad_Input()) {
                              for(n=0; n<2000; n++){
                                while(ADCflag == 0){};
                                ADCflag = 0; // show every 2000th point
                              }
                              UART0_OutString("Your object is: ");
                              UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" cm away\r\n");
                              Motor_Forward(1000,1000);
                              Clock_Delay1ms(500);
                              /*left_l= LeftConvert(nl);
                              right_l = RightConvert(nr);
                              center_l = CentreConvert(nc);

                              if((center_l < left_l) && (center_l<right_l))
                              {     Motor_Forward(1000,1000);
                                    Clock_Delay1ms(500);
                              }
                              else if((left_l<center_l)&&(left_l<right_l))
                              {     Motor_Left(500,500); Motor_Forward(1000,1000);
                                      Clock_Delay1ms(500);
                              }
                              else if((right_l<center_l)&&(right_l<left_l))
                              {     Motor_Right(500,500); Motor_Forward(1000,1000);
                                    Clock_Delay1ms(500);
                              }*/
                              if (CenterConvert(nc)>15/*||LeftConvert(nl)>13||RightConvert(nr)>13*/) {
                                  EUSCIA0_OutString("Re-calibrate! ");
                                  Motor_Stop();
                                  Clock_Delay1ms(1000);
                              }

                                                      }
                          menu=1;
                          cmd=0xDEAD;
                          break;
                      default:
                          menu =1;
                          break;
              }
              if(!menu)Clock_Delay1ms(3000);
              else{
                menu=0;
                  }

      // ....
      // ....



  }
}
