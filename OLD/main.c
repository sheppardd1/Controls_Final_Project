/*
 * Systems and Controls Final Project
 * James Cipparrone, Alex Revolus, and David Sheppard
 *
 * Purpose: Reads in data from a PTAT and uses the info to
 * control a fan by PWM in order to maintain a desired temperature
 * on the PTAT.
 *
 *
 * Ports:   6.0: ADC input from LM35
 *          6.1: ADC input from Pot (for setting desired temp)
 *          2.7: PWM output
 *
 * Note: some code was taken from sample code in the TI resource explorer
 * Based on Intro to Embedded Milestone 2 Code
 */


/*
 * Development Notes:
 *      Need to test "simultaneous" ADC readings
 *      PID control not yet tested
 *          kp, kd, ki values are just arbitrary
 *          code functionality not yet verified
 *      Not yet sure of actual value for adcSamplingPeriod
 *      May need to change reference voltage in code ?
 */

#include <msp430F5529.h>
#include <math.h>

const float kp = 1;             // proportional
const float ki = 1;             // integral
const float kd = 1;             // derivative
float integral = 0;             // integral value that accumulates over time
float derivative = 0;           // derivative
const float adcSamplingPeriod = 0.000005;   // I don't think this is correct

float potArray[3] = {0,0,0};              // ADCMEM1 reading (from pot)
float pot = 0;                      // median filtered pot readings
float adcReading = 0;               // readings from Analog-Digital Converter
float adcReading2 = 0;
float adcArray[3] = {0,0,0};    // 3 most recent ADCMEM0 values (for median filter)
int adcReady = 1;               // 1 if ADC conversion is done, else 0

float realTemp = 0;                 //temperature reading
float oldRealTemp = 0;              // previous temp reading
float desiredTemp = 20;         // initialization default: 20 C
int ccr = 111;                        // value to set the TA1CCR1 to  float median;
float median = 0;




void setPWM(float, float);                  //prototype for function that sets TA1CCR1 value
float PID(float, float);                    // PID controller
float medianFilter(float *);

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  REFCTL0 &= ~REFMSTR;                      // Reset REFMSTR to hand over control to
                                            //   ADC12_A ref control registers

  float kdt = kd / adcSamplingPeriod;       // value = kd / ADC sampling period

  //bit 2.7 will be PWM output***************************************************************
  P2SEL &= ~BIT7;                           //set 2.7 to be GPIO
  P2DIR |= BIT7;                            //set 2.7 to be output

  //port 6.0, 6.1 is analog input 0, 1*******************************************************
  P6DIR &= ~BIT0;                           //set 6.0 to be input
  P6DIR &= ~BIT1;                           //set 6.1 to be input
  P6SEL |= BIT0;                            //set 6.0 to be A0 (input of ADC)
  P6SEL |= BIT1;                            //set 6.1 to be A1

  //PWM initialization************************************************************************
  //using Timer A1
  TA1CCTL1 = CCIE;                          // CCR1 interrupt enabled for TA1
  TA1CCR0 = 262;                            //Set the period in the Timer A CCR0 to 1000 us.
  TA1CCR1 = 131;                            //The initial period in microseconds that the power is ON.
                                            //It's initialized to half the time, which translates to a 50% duty cycle.
  TA1CTL = TASSEL_2 + MC_1 + ID_2 + TAIE;   //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.

  //ADC sampling******************************************************************************
  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12IE = 0x01;                           // Enable interrupt
  ADC12CTL0 |= ADC12ENC;                    //Enable conversion

  __bis_SR_register(/*LPM0 + */GIE);        // LPM0 with interrupts enabled

  //polling for ADC values********************************************************************

  short i;

  while(1)
  {
    for(i = 2; i >= 0; --i){
        ADC12CTL0 |= ADC12SC;                   // Sampling and conversion start
        while(adcReady == 0){};                 //Wait for ADC to finish conversion
        adcArray[i] = adcReading;
        median = medianFilter(adcArray);
        potArray[i] = adcReading2;
        pot = medianFilter(potArray);
    }
    setPWM(kdt, median);                        //set PWM values to change duty cycle
    adcReady = 0;                               //ADC no longer ready

  }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
    adcReady = 1;                           //when this interrupt fires, ADC is ready
    adcReading = ADC12MEM0;                 //record ADC reading
    adcReading2 = ADC12MEM1;


    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
}

float medianFilter(float *a){
    // ONLY WORKS IF WINDOW OF FILTER IS 3 NUMBERS WIDE
    // This code is complicated, but also efficient
    // Checks to see if the current value is between the other two

    if((adcArray[0] <= a[1] && a[0] >= a[2]) || (a[0] >= a[1] && a[0] <= a[2]))
        return a[0];
    else if((a[1] <= a[0] && a[1] >= a[2]) || (a[1] >= a[0] && a[1] <= a[2]))
        return a[1];
    else
        return a[2];
}

//set CCR1 values to change duty Cycle of PWM: Low DC = slow fan, High DC = fast fan
void setPWM(float kdt, float median)
{

    //using the LM35 PTAT
    //PTAT equation: Vo = 10 mV/C + 0 mV
    //therefore, Temperature = Vo / 0.01

    //calculations*********************************************************************************************

    float analogVoltage = median * (5 / 4096);      //convert digital reading back to analog voltage value
                                                        // reference PTAT voltage is 5 V, ADC is 12 bits (2^12 = 4096)
    oldRealTemp = realTemp;
    realTemp = (analogVoltage / 0.01);     //convert analog voltage to temperature

    float error = pot - (realTemp - desiredTemp);          //determine error between real and desired temp

    ccr = PID(error, kdt);


    if(ccr < 0){                 //ensure ccr never gets negative
         ccr = 0;
    }
    else if(ccr > 250){         //ensure CCR1 never gets too close to CCR0 (CCR0 is 262). If CCR1 is too close to CCR0, they won't both fire
        ccr = 250;
    }

    TA1CCR1 = ccr;              //finally set the CCR1 value

}


float PID(float error, float kdt /*real and desired temp are globals*/){
    // Derivative:
    derivative = kd * (kdt * (realTemp - oldRealTemp));

    //Integral:
    integral = kp * (integral + (error * adcSamplingPeriod*3));  //using Reimann sums, and multiplying period by 3 since using median filter

    //Proportional
    float proportion = error * kp;

    float sum = derivative + integral + proportion;

    int new_ccr = ccr + sum * 2.5;          // new_ccr must be an int, so decimals purposely get truncated
                                            // using 2.5 multiplier since ccr goes from 0 to 250, so 2.5 normalizes the change

    return new_ccr;
}

//Timer Interrupt: used for software PWM
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TA1IV,14)) //testing timer interrupt vector
  {
    case  0: break;                          // No interrupt
    case  2: P2OUT &= ~BIT7;                 //if CCR1 is reached, set output low
             break;
    case  4: break;                          // CCR2 not used
    case  6: break;                          // reserved
    case  8: break;                          // reserved
    case 10: break;                          // reserved
    case 12: break;                          // reserved
    case 14: P2OUT |= BIT7;                  // if CCR0 overflows, set output high
             break;
    default: break;
  }
  TA1IV &= ~TA1IV_TA1IFG;                    // Clear the Timer interrupt Flag
}



//Info from Resource Explorer

/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430F552x Demo - ADC12, Repeated Sequence of Conversions
//
//  Description: This example shows how to perform a repeated sequence of
//  conversions using "repeat sequence-of-channels" mode.  AVcc is used for the
//  reference and repeated sequence of conversions is performed on Channels A0,
//  A1, A2, and A3. Each conversion result is stored in ADC12MEM0, ADC12MEM1,
//  ADC12MEM2, and ADC12MEM3 respectively. After each sequence, the 4 conversion
//  results are moved to A0results[], A1results[], A2results[], and A3results[].
//  Test by applying voltages to channels A0 - A3. Open a watch window in
//  debugger and view the results. Set Breakpoint1 in the index increment line
//  to see the array values change sequentially and Breakpoint2 to see the entire
//  array of conversion results in A0results[], A1results[], A2results[], and
//  A3results[]for the specified Num_of_Results.
//
//  Note that a sequence has no restrictions on which channels are converted.
//  For example, a valid sequence could be A0, A3, A2, A4, A2, A1, A0, and A7.
//  See the MSP430x5xx User's Guide for instructions on using the ADC12.
//
//               MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//    Vin0 -->|P6.0/CB0/A0      |
//    Vin1 -->|P6.1/CB1/A1      |
//    Vin2 -->|P6.2/CB2/A2      |
//    Vin3 -->|P6.3/CB3/A3      |
//            |                 |
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************
