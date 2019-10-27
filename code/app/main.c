/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2008
 *
 *    File name   : main.c
 *    Description : Main module
 *
 *    History :
 *    1. Date        : 12, August 2008
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 *  This example project shows how to use the IAR Embedded Workbench for ARM
 * to develop code for the IAR LPC2478-SK board. It shows basic use of I/O,
 * timers, interrupts, LCD controllers and LCD touch screen.
 *
 *  A cursor is shown and moves when the screen is touched.
 *
 * Jumpers:
 *  EXT/JLINK  - depending of power source
 *  ISP_E      - unfilled
 *  RST_E      - unfilled
 *  BDS_E      - unfilled
 *  C/SC       - SC
 *
 * Note:
 *  After power-up the controller gets it's clock from internal RC oscillator that
 * is unstable and may fail with J-Link auto detect, therefore adaptive clocking
 * should always be used. The adaptive clock can be select from menu:
 *  Project->Options..., section Debugger->J-Link/J-Trace  JTAG Speed - Adaptive.
 *
 * The LCD shares pins with Trace port. If ETM is enabled the LCD will not work.
 *
 *    $Revision: 28 $
 **************************************************************************/
#include <includes.h>
#include <stdio.h>
#include <intrinsics.h>

#include "board.h"
#include "sys.h"
#include "sdram_64M_32bit_drv.h"
#include "drv_glcd.h"
#include "math.h"
//#include "drv_touch_scr.h"

#include "board.h"
#include "sys.h"

#include "smb380_drv.h"


#define NONPROT 0xFFFFFFFF
#define CRP1  	0x12345678
#define CRP2  	0x87654321
/*If CRP3 is selected, no future factory testing can be performed on the device*/
#define CRP3  	0x43218765

#ifndef SDRAM_DEBUG
#pragma segment=".crp"
#pragma location=".crp"
__root const unsigned crp = NONPROT;
#endif

#define TIMER0_TICK_PER_SEC   10000
#define TIMES_TO_GET_VALUE_PER_SEC 1
#define CUT_OFF_FREQUENCY 50


#define LCD_VRAM_BASE_ADDR ((Int32U)&SDRAM_BASE_ADDR)

unsigned char Smb380Id, Smb380Ver;

extern Int32U SDRAM_BASE_ADDR;
extern FontType_t Terminal_6_8_6;
extern FontType_t Terminal_9_12_6;
extern FontType_t Terminal_18_24_12;

int ADC_Voltage;
int ADC_Current;
float voltage;
float current;
float voltage_unfiltered;
float current_unfiltered;
float alpha;
//float sampling_interval;
float time_constant;

//global varables
int SamplingCount = 0;
int ZeroCount = 0;
float frequency = 0;
float ZeroTime1 = 0;
float ZeroTime2 = 0;
float voltage_previous = 0;
float current_previous = 0;
float voltage_previous_previous = 0;
float voltage_unfiltered_previous = 0;

double frequencies[TIMES_TO_GET_VALUE_PER_SEC] = {0};
double frequency_mean = 0;
double offset = 0;

float current_RMS_tmp;
float voltage_RMS_tmp;
float power_RMS_tmp;
float current_RMS;
float voltage_RMS;
float power_RMS;
   
float freq_offset = 0.99948;

double current_RMS_values[TIMES_TO_GET_VALUE_PER_SEC] = {0};
double current_RMS_mean = 0;

double voltage_RMS_values[TIMES_TO_GET_VALUE_PER_SEC] = {0};
double voltage_RMS_mean = 0;

//DFR variables
float bulbFrequency;
float socketFrequency;
float bulbCurrent;
float socketCurrent;
float bulbVoltage;
float socketVoltage;
float bulbPower;
float socketPower;
int bulbAuto;
int socketAuto;
int bulbOn;
int socketOn;
int bulbForceOn;
int socketForceOn;
/*
float c;
float d;
*/

int i = 0;

int offset_calculate = 0;
double current_offset = 0;

int conversion = 0;

char VoltageDisplay[10];
char CurrentDisplay[10];
char FrequencyDisplay[10];
char PowerDisplay[10];

int frequency_print_before_point = 0;
int frequency_print_after_point = 0;
double frequency_print_after_point_double = 0;
double frequency_print_after_point_four = 0;
int frequency_fourth_decibel = 0;

int voltage_print_before_point = 0;
int voltage_print_after_point = 0;
double voltage_print_after_point_double = 0;

int current_print_before_point = 0;
int current_print_after_point = 0;
double current_print_after_point_double = 0;

int power_print_before_point = 0;
int power_print_after_point = 0;
double power_print_after_point_double = 0;

int ADC_temp1;

int k = 0;
int h = 0;
int setup = 0;

int measure = 0;
int control = 0;

//int c = 0;

/*************************************************************************
 * Function Name: Timer0IntrHandler
 * Parameters: none
 *
 * Return: none
 *
 * Description: Timer 0 interrupt handler
 *
 *************************************************************************/
void Timer0IntrHandler (void)
{
  //store previous voltage
 
//  ADC_temp1 = AD0CR_bit.SEL;    //store the information in AD0CR_bit.SEL
    
  voltage_previous = voltage;   //store previous voltage
  current_previous = current;   //store previous current
    
  // Toggle USB Link LED
  USB_D_LINK_LED_FIO ^= USB_D_LINK_LED_MASK;    //flash USB 
  
 //calculate instant value of current and voltage  
//  AD0CR_bit.SEL = (1UL<<2);     //select AD0[2]
  if(ADDR2_bit.DONE==1)
  {
  ADC_Voltage = (ADDR2 & (0x0FFFF));    //get voltage from ADC
  }
  if(ADDR3_bit.DONE==1)
  {
//  AD0CR_bit.SEL = (1UL<<3);     //select AD0[3]
  ADC_Current = (ADDR3 & (0x0FFFF));    //get current from ADC
  }
  
  //calculate real instant voltage and current
  voltage = ADC_Voltage*2.0/(0xFFFF);   
  current = ADC_Current*2.0/(0xFFFF);
  current_unfiltered = (current-1)*1*1.414;  
  voltage_unfiltered = (voltage-1)*240*1.414;
    
  //apply low pass filter
  time_constant = (1/(2*3.14159265*CUT_OFF_FREQUENCY));

  alpha = (1/(1+time_constant*TIMER0_TICK_PER_SEC));
  voltage = alpha*voltage_unfiltered+(1-alpha)*voltage_previous;
//  current = alpha*current_unfiltered+(1-alpha)*current_previous;
  //need more comments

  //calculate zero crossing times
  if (SamplingCount < TIMER0_TICK_PER_SEC/TIMES_TO_GET_VALUE_PER_SEC)
  {
    if(voltage * voltage_previous < 0 & voltage > 0) //if there is a zero cross
    {
      if (ZeroTime1==0)//fist zero cross
      {
        ZeroTime1 = SamplingCount-(voltage/(voltage-voltage_previous));
        ZeroTime2 = SamplingCount-(voltage/(voltage-voltage_previous));
      }
      else
      {
        if((SamplingCount-ZeroTime2)>TIMER0_TICK_PER_SEC/50/5)
        {
          ZeroTime2 = SamplingCount-(voltage/(voltage-voltage_previous)); //final zero cross
          ZeroCount++; //counting zero crossing times over that period
        }
      }
    }

    //calculate RMS, using unfiltered value
    current_RMS_tmp = current_RMS_tmp + 1.0*current_unfiltered*current_unfiltered/TIMER0_TICK_PER_SEC;
    voltage_RMS_tmp = voltage_RMS_tmp + 1.0*voltage_unfiltered*voltage_unfiltered/TIMER0_TICK_PER_SEC;
    power_RMS_tmp = power_RMS_tmp + 1.0*current_unfiltered*voltage_unfiltered/TIMER0_TICK_PER_SEC;
    SamplingCount++;
  }
  
  // clear interrupt
//  AD0CR_bit.SEL = ADC_temp1; //restore original information from ADC_temp
  T0IR_bit.MR0INT = 1;
  VICADDRESS = 0;
}



/*************************************************************************
 * Function Name: main
 * Parameters: none
 *
 * Return: none
 *
 * Description: main
 *
 *************************************************************************/
int main(void)
{
Int32U cursor_x = (C_GLCD_H_SIZE - CURSOR_H_SIZE)/2, cursor_y = (C_GLCD_V_SIZE - CURSOR_V_SIZE)/2;
ToushRes_t XY_Touch;
Boolean Touch = FALSE;

  GLCD_Ctrl (FALSE);
  // Init GPIO
  GpioInit();
#ifndef SDRAM_DEBUG
  // MAM init
  MAMCR_bit.MODECTRL = 0;
  MAMTIM_bit.CYCLES  = 3;   // FCLK > 40 MHz
  MAMCR_bit.MODECTRL = 2;   // MAM functions fully enabled
  // Init clock
  InitClock();
  // SDRAM Init
  SDRAM_Init();
#endif // SDRAM_DEBUG
  // Init VIC
  VIC_Init();
  // GLCD init
  GLCD_Init (IarLogoPic.pPicStream, NULL);

  GLCD_Cursor_Dis(0);

  GLCD_Copy_Cursor ((Int32U *)Cursor, 0, sizeof(Cursor)/sizeof(Int32U));

  GLCD_Cursor_Cfg(CRSR_FRAME_SYNC | CRSR_PIX_32);

  GLCD_Move_Cursor(cursor_x, cursor_y);

  GLCD_Cursor_En(0);

  // Init touch screen
  TouchScrInit();

  // Touched indication LED
  USB_H_LINK_LED_SEL = 0; // GPIO
  USB_H_LINK_LED_FSET = USB_H_LINK_LED_MASK;
  USB_H_LINK_LED_FDIR |= USB_H_LINK_LED_MASK;

  // init socket
  FIO0DIR |= (1UL<<19); //P0[19] output
  FIO0SET |= (1UL<<19); //P0[19] set to HIGH
  FIO0PIN |= (1UL<<19);
  
  //int bulb
  FIO0DIR |= (1UL<<11);
  FIO0SET |= (1UL<<11);
  FIO0PIN |= (1UL<<11);
  
  //init parameters
  bulbAuto = 1;
  socketAuto = 1;
  bulbOn = 1;
  socketOn = 1;
  bulbForceOn = 0;
  socketForceOn = 0;
  
  //This part of initializing ADC is commented out since in the function 
  //"touchscrinit" has already initialized ADC. Redo the initailzing process 
  //may occur a conflict.
  /************************************************************************
  //init ADC
  AD0CR_bit.PDN = 0;
  PCONP_bit.PCAD = 1; //power AD converter
  *************************************************************************/
  
  //set P0.25 and P0.26 to AD[2]and AD[3], select channel of ADC
  PINSEL1 = (((unsigned)0x01)<<18) | (((unsigned)0x01)<<20);
  PCLKSEL0_bit.PCLK_ADC = 1; //enable ADC clock
  AD0CR_bit.SEL = 0x0f;      //channel 0,1,2,3
  
  
  //This part of initializing ADC is commented out since in the function 
  //"touchscrinit" has already initialized ADC. Redo the initailzing process 
  //may occur a conflict.
  /*************************************************************************
  AD0CR_bit.BURST = 1;       //burst mode
  AD0CR_bit.CLKDIV = 5;      //frequency cycle 3MHz (18MHz/(5+1))
  AD0CR_bit.SEL = 0x0f;      //channel 0,1,2,3
  AD0CR_bit.CLKS = 0;
  AD0CR_bit.START = 0;
  AD0CR_bit.PDN = 1;
  **************************************************************************/
  
  
  //apply a delay
  int dly = 10;
  for (; dly >0; dly--)
    for (int i = 0; i<5000; i++);

  
  
  // Enable TIM0 clocks
  PCONP_bit.PCTIM0 = 1; // enable clock

  // Init Timer0
  T0TCR_bit.CE = 0;     // counting  disable
  T0TCR_bit.CR = 1;     // set reset
  T0TCR_bit.CR = 0;     // release reset
  T0CTCR_bit.CTM = 0;   // Timer Mode: every rising PCLK edge
  T0MCR_bit.MR0I = 1;   // Enable Interrupt on MR0
  T0MCR_bit.MR0R = 1;   // Enable reset on MR0
  T0MCR_bit.MR0S = 0;   // Disable stop on MR0
  // set timer 0 period
  T0PR = 0;
  T0MR0 = SYS_GetFpclk(TIMER0_PCLK_OFFSET)/(TIMER0_TICK_PER_SEC);
  // init timer 0 interrupt
  T0IR_bit.MR0INT = 1;  // clear pending interrupt
  
  VIC_SetVectoredIRQ(Timer0IntrHandler,4,VIC_TIMER0);
  VICINTENABLE |= 1UL << VIC_TIMER0;

  T0TCR_bit.CE = 1;     // counting Enable
  
  
  #if 0
  SDRAM_Test();
#endif

#if 0
  pInt32U pDst = (pInt32U)LCD_VRAM_BASE_ADDR;
  for(Int32U k = 0; k < C_GLCD_V_SIZE; k++)
  {
    for(Int32U i = 0 ; 8 > i; i++)
    {
      for(Int32U j = 0; 40 > j; j++)
      {
        switch(i)
        {
        case 0:
          *pDst++ = 0;
           break;
        case 1:
          *pDst++ = 0xFF;
           break;
        case 2:
          *pDst++ = 0xFF00;
           break;
        case 3:
          *pDst++ = 0xFFFF;
           break;
        case 4:
          *pDst++ = 0xFF0000;
           break;
        case 5:
          *pDst++ = 0xFF00FF;
           break;
        case 6:
          *pDst++ = 0xFFFF00;
           break;
        case 7:
          *pDst++ = 0xFFFFFF;
           break;
        }
      }
    }
  }
#endif
  //
  SMB380_Init();

  SMB380_GetID(&Smb380Id, &Smb380Ver);

 // SMB380_Data_t XYZT;
  
  __enable_interrupt();
  
  
  //LCD Display
  GLCD_Ctrl (TRUE);
  
  GLCD_SetFont(&Terminal_18_24_12,0x000000,0x00F7F7F7);//black letters in white 
                                                       //window background for 
                                                       //displaying text
  
  GLCD_SetWindow(10,10,285,33);
  GLCD_TextSetPos(0,0);
  GLCD_print("\f Press Start to");
  
  
  GLCD_SetWindow(10,33,285,55);
  GLCD_TextSetPos(0,0);
  GLCD_print("\f measure the frequency");    //Displaying two rows
    
  
  
  GLCD_SetFont(&Terminal_18_24_12,0x0000FF,0x000cd4ff);//red letters in yellow 
                                                       //window background for 
                                                       //displaying button
  
  
  GLCD_SetWindow(55,135,135,160);
  GLCD_TextSetPos(0,0);
  GLCD_print("\f Start");        //Displaying a 'Start' botton

  GLCD_SetFont(&Terminal_18_24_12,0x000000,0x00F7F7F7);//Except displaying 
                                                       //buttons, in default,
                                                       //black letters in white 
                                                       //window background for 
                                                       //displaying text
  setup = 0;    //not yet started operating

  //Main Loop
  while(1)
  {            
    //ontaining information from touch screen
    if(TouchGet(&XY_Touch))
    {
      cursor_x = XY_Touch.X;
      cursor_y = XY_Touch.Y;
      GLCD_Move_Cursor(cursor_x, cursor_y);     //move cursor to new position
      if (FALSE == Touch)
      {
        Touch = TRUE;          
        USB_H_LINK_LED_FCLR = USB_H_LINK_LED_MASK;        
      }

      
      //looking for a touch at the start button
      if ((cursor_x > 55)&(cursor_x < 110)&(cursor_y < 160) &(cursor_y > 135))
      {
        GLCD_SetFont(&Terminal_18_24_12,0x0000FF,0x000cd4ff); 
        GLCD_SetWindow(188,135,268,160);
        GLCD_TextSetPos(0,0);
        GLCD_print("\fControl");  //displaying 'control' button
        
        GLCD_SetWindow(55,135,135,160);
        GLCD_TextSetPos(0,0);
        GLCD_print("\fMeasure");  //displaying 'measure' button
        
        GLCD_SetFont(&Terminal_18_24_12,0x000000,0x00F7F7F7);//back to default
                                                             //mode of display
                                                             //letters
        GLCD_SetWindow(55,195,268,218);
        GLCD_TextSetPos(0,0);
        GLCD_print("\f Start Measure!");    //displaying text for start measure
                                            //frequency

        control = 0;            //start to measure the frequency
        setup = 1;              //machine is operated
         
      }
      
      //looking for a touch at the control button after pressing start button
      if ((cursor_x > 213)&(cursor_x < 268)&(cursor_y < 160) &(cursor_y > 135) & (setup == 1))
      {
         GLCD_SetWindow(55,195,268,218);
         GLCD_TextSetPos(0,0);
         GLCD_print("\f Start Control!");    //displaying text for start control
         control = 1;           //Control the frequency
      }
    }
    else if(Touch)
    {
      USB_H_LINK_LED_FSET = USB_H_LINK_LED_MASK;
      Touch = FALSE;
    }
    
    
    //frequency calculation after pressing start button when machine is operated
    if (SamplingCount >= TIMER0_TICK_PER_SEC/TIMES_TO_GET_VALUE_PER_SEC & setup == 1)
    {      
      //calculating frequency per 'TIMES_TO_GET_VALUE_PER_SEC'
     // frequencies[i] = (1.0*(ZeroCount))*TIMER0_TICK_PER_SEC/(ZeroTime2-ZeroTime1);
      
      //fetching tmps
      current_RMS_values[i] = current_RMS_tmp;
      current_RMS_tmp = 0;
      
      //fetching tmps
      voltage_RMS_values[i] = voltage_RMS_tmp;
      voltage_RMS_tmp = 0;
      
      i++;
           
      //further calculation is made iff datum in one second are collected
      if (i >= TIMES_TO_GET_VALUE_PER_SEC)
      {
//        frequency_mean = 0;
        current_RMS_mean = 0;
        voltage_RMS_mean = 0;
        
        //accumulating tmps
        while (i != 0)
        {
//          frequency_mean = frequency_mean+frequencies[i-1]/TIMES_TO_GET_VALUE_PER_SEC;
          current_RMS_mean = current_RMS_mean + current_RMS_values[i-1];
          voltage_RMS_mean = voltage_RMS_mean + voltage_RMS_values[i-1];
          i--;
        }

        i = 0;
        
//        offset = frequency_mean-CUT_OFF_FREQUENCY;//wrong strategy (see report)
        
        //addressing mean values into variables
//        frequency = frequency_mean;
        frequency = ((1.0*(ZeroCount))*TIMER0_TICK_PER_SEC/(ZeroTime2-ZeroTime1))*freq_offset;
        current_RMS = sqrt(current_RMS_mean);
        voltage_RMS = sqrt(voltage_RMS_mean);
                
        //print frequency value
        GLCD_SetWindow(10,10,285,33);   //print window for displaying frequency
        GLCD_TextSetPos(0,0);
      
        //Since there is a difficulty of using '%lf' for printing a double or
        //float on the LCD screen, these values are converted into two integers,
        //one is the numbers before the decibel point and the other is the 
        //numbers after dicibel point. They are printed individually on the
        //screen.
        frequency_print_before_point = (int) frequency; //convert double/float into
                                                        //integer, with remaining
                                                        //the numbers before
                                                        //decibel point
        
        frequency_print_after_point_double = (frequency-frequency_print_before_point)*1000; //after subtracting that integer
                                                                                            //into the original double value,
                                                                                            //only the part after decibel point
                                                                                            //is remained. By multiplying a factor
                                                                                            //of 1000, the first three numbers
                                                                                            //after decibel point is shifted before
                                                                                            //decibal point.
        frequency_print_after_point = (int) frequency_print_after_point_double;

        
        if (frequency_print_after_point >= 100)
        {
          GLCD_print("\fFrequency: %d.%dHz", frequency_print_before_point, frequency_print_after_point);
        }
        else if (frequency_print_after_point >= 10) //if the first one number after decibel point is 0
        {
          GLCD_print("\fFrequency: %d.0%dHz", frequency_print_before_point, frequency_print_after_point);//an extra '0' after decibel point is needed
        }
        else //if the first two numbers after decibel point is 0
        {
          GLCD_print("\fFrequency: %d.00%dHz", frequency_print_before_point, frequency_print_after_point);//two extra 0s after decibel point is needed
        }
          
        
        //calculate offset current
        if (control == 0)               //if control strategy not yet applied
        {
          bulbAuto = 0;                 //bulb is manually controlled
          socketAuto = 0;
          bulbOn = 0;
          socketOn = 0;
          FIO0PIN &= ~((1UL<<11)|(1UL<<19));        //shut down bulb
                                                    //shut down socket
          current_offset = current_RMS; //Since no current going to bulb, the
                                        //measured current is RMS current
        }
        
        if (control == 1)               //if control strategy is applied
        {
          bulbAuto = 1;                 //bulb and socket are automatically controlled from
          socketAuto = 1;               //applied control strategy
          
          if (current_RMS < 0.2)     //if current is smaller than the value 
                                        //which slightly higher than offset current
          {
            current_offset = current_RMS;//the current is offset current
            current_RMS = 0;             //no current value is going to be displayed
          }
        
          if (current_RMS > 0.2)     //if the current is not offset current
          {
            current_RMS = current_RMS; //substract offset current
          }
      }
        
        
      if (bulbAuto == 1)              //apply control strategy
      {
        if (frequency<49.98)        //if frequency is lower than gap
        {
           bulbOn = 0;           //shutdown the bulb
           FIO0PIN &= ~(1UL<<11);
           
           
/********************************************************************
//                  current_offset = current_RMS;
//                  current_RMS = 0;
********************************************************************/              
        
        }
        if (frequency>50.02)            //if frequency is higher than gap
        {
           bulbOn = 1;
           FIO0PIN |= (1UL<<11);        //turn on the bulb

  //                current_RMS = current_RMS-current_offset;
        }
      }
      if (socketAuto == 1)
      {
         if (frequency<49.98)
         {
            socketOn = 0;
            FIO0PIN &= ~(1UL<<19); //This should be AND NOR operation
            //FIO0PIN &= (1UL<<19);
         }
         if (frequency>50.02)
         {
            socketOn = 1;
            FIO0PIN |= (1UL<<19);
         }          
      }
            

      if (bulbAuto == 1)
      {
         power_RMS = current_RMS*voltage_RMS;   //power RMS value available
      }
      if (bulbAuto == 0)
      {
         power_RMS = 0;                         //power RMS value unavailable
      }
        
      power_RMS_tmp = 0;                        //clear accumulator of RMS value
      
      //print voltage
      GLCD_SetWindow(10,35,285,58);
      GLCD_TextSetPos(0,0);
      
      //same process as printing frequency
      voltage_print_before_point = (int) voltage_RMS;
      voltage_print_after_point_double = (voltage_RMS-voltage_print_before_point)*1000;
      voltage_print_after_point = (int) voltage_print_after_point_double; //need more explaination
      
      if (voltage_print_after_point < 0)//since absolute value is needed after decibel point, therefore
                                        //if negative number, one has to take its opposite number
      {
        voltage_print_after_point = -voltage_print_after_point;
      }
      
      if (voltage_print_after_point >= 100 | voltage_print_after_point <= -100)
      {
        GLCD_print("\fVoltage: %d.%dV", voltage_print_before_point, voltage_print_after_point);
      }
      else if (voltage_print_after_point >= 10 | voltage_print_after_point <= -10)
      {
        GLCD_print("\fVoltage: %d.0%dV", voltage_print_before_point, voltage_print_after_point);
      }
      else
      {
        GLCD_print("\fVoltage: %d.00%dV", voltage_print_before_point, voltage_print_after_point);
      }
      
      //print current
      GLCD_SetWindow(10,60,285,83);
      GLCD_TextSetPos(0,0);
         
      //since maximum current is less than 1mA, one only needs to print in mA
      current_print_after_point_double = current_RMS*1000;
      current_print_after_point = (int) current_print_after_point_double;
       
        
      if (bulbAuto == 1)        //if control strategy is applied
      {
        GLCD_print("\fCurrent: %dmA", current_print_after_point);
      }
        
      if (bulbAuto == 0)        //if control stragegy is not applied, displaying leakage current in cable
      {
        GLCD_print("\fCurrent Leakage: %dmA", current_print_after_point);
      }
        
      //print power, same way of printing voltage
      GLCD_SetWindow(10,85,285,108);
      GLCD_TextSetPos(0,0);
      
      power_print_before_point = (int) power_RMS;
      power_print_after_point_double = (power_RMS-power_print_before_point)*1000;
      power_print_after_point = (int) power_print_after_point_double;
      
      if (power_print_after_point < 0)
      {
        power_print_after_point = -power_print_after_point;
      }
      
      if (power_print_after_point >= 100 | power_print_after_point <= -100)
      {
        GLCD_print("\fPower: %d.%dW", power_print_before_point, power_print_after_point);
      }
      else if (power_print_after_point >= 10 | power_print_after_point <= -10)
      {
        GLCD_print("\fPower: %d.0%dW", power_print_before_point, power_print_after_point);
      }
      else
      {
        GLCD_print("\fPower: %d.00%dW", power_print_before_point, power_print_after_point);
      }
      
    }
      
    //reset variables for a new second
    SamplingCount = 0;
    ZeroTime1 = 0;
    ZeroTime2 = 0;
    ZeroCount = 0;
      

      
    //calculate DFR
    bulbFrequency = frequency;
    socketFrequency = frequency;
    bulbCurrent = current_RMS;
    socketCurrent = current_RMS;
    bulbVoltage = voltage_RMS;
    socketVoltage = voltage_RMS;
    bulbPower = power_RMS;
    socketPower = power_RMS; 
  }
   
}

}