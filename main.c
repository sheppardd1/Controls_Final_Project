/*
 * Systems and Controls Final Project
 * James Cipparrone, Alex Revolus, and David Sheppard
 *
 * Purpose: Reads in data from a PTAT and uses the info to
 * control a fan by PWM in order to maintain a desired temperature
 * on the PTAT.
 *
 *
 * Ports:   6.0: ADC input
 *          2.7: PWM output
 *          4.4: TX
 *          4.5: RX
 *
 * Note: some code was taken from sample code in the TI resource explorer
 * Based on Intro to Embedded Milestone 2 Code
 */


/*
 * Development Notes:
 *      Program still written as if desired temperature comes from UART instead of pot
 *          Need to read pot from ADC
 *          Need to act on ADC reading to get desired temperature
 *      PID control not yet tested
 *          kp, kd, ki values are just arbitrary
 *          code functionality not yet verified
 *      Still need to implement median filter to filter out bad ADC values
 *      Not yet sure of actual value for adcSamplingPeriod
 *      Not 100% sure how to get new CCR value from PID
 */

#include <msp430F5529.h>
#include <math.h>

const float kp = 1;
const float ki = 1;
const float kd = 1;
const float adcSamplingPeriod = 0.000005;   // I don't think this is correct

float realTemp;         //temperature reading
float oldRealTemp;      // previous temp reading
float integral;         // integral value that accumulates over time
float desiredTemp = 20; //initialization default: 20 C
float adcReading;       //reading from Analog-Digital Converter
int adcReady = 1;       //1 if ADC conversion is done, else 0
int ccr;                //value to set the TA1CCR1 to




void setPWM();          //prototype for function that sets TA1CCR1 value
float PID(float, float);            // PID controller

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  REFCTL0 &= ~REFMSTR;                      // Reset REFMSTR to hand over control to
                                            // ADC12_A ref control registers

  float kdt = kd / adcSamplingPeriod;       // value = kd / ADC sampling period

  //bit 2.7 will be PWM output***************************************************************
  P2SEL &= ~BIT7;                           //set 2.7 to be GPIO
  P2DIR |= BIT7;                            //set 2.7 to be output

  //port 6.0 is analog input 0***************************************************************
  P6DIR &= ~BIT0;                           //set 6.0 to be input
  P6SEL |= BIT0;                            //set 6.0 to be A0 (input of A to D)

  //UART Setup*******************************************************************************
  P4SEL |= BIT5 + BIT4;                     //enable UART for these pins
  UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA1CTL1 |= UCSSEL_2;
  // Use Table 24-5 in Family User Guide for BAUD rate calculation
  UCA1BR0 = 6;                              // 1MHz 9600 (see User's Guide)
  UCA1BR1 = 0;                              // 1MHz 9600
  UCA1MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  UCA1TXBUF = 0;                            //set RX buffer to 0 for testing purposes

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

  __bis_SR_register(LPM0 + GIE);        // LPM0 with interrupts enabled

  //polling for ADC values********************************************************************
  while(1)
  {
    ADC12CTL0 |= ADC12SC;                   // Sampling and conversion start
    while(adcReady == 0){};                 //Wait for ADC to finish conversion
    setPWM(kdt);                               //set PWM values to change duty cycle
    adcReady = 0;                           //ADC no longer ready

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

    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
}

//set CCR1 values to change duty Cycle of PWM: Low DC = slow fan, High DC = fast fan
void setPWM(float kdt)
{

    //using the LM60CIZ PTAT
    //PTAT equation: Vo = 10 mV/C + 0 mV
    //therefore, Temperature = Vo / 0.01

    //calculations*********************************************************************************************

    float analogVoltage = adcReading * (5 / 4096);      //convert digital reading back to analog voltage value
                                                        // reference PTAT voltage is 5 V, ADC is 12 bits (2^12 = 4096)
    oldRealTemp = realTemp;
    realTemp = (analogVoltage / 0.01);     //convert analog voltage to temperature

    float difference = realTemp - desiredTemp;          //determine difference between real and desired temp

    PID(difference, kdt);

    /* from milestone 2:
    //proportional control: change PWM based on difference in temperature**************************************
    if(difference > 1)                                  //if temp is too high
        ccr += (difference);                            //increase ccr value to speed up fan
    else if(difference < -1)                            //if temp is too low
        ccr += (difference);                            //decrease ccr value to slow down fan (difference is already negative if realTemp is too low)
    //else if within 1 degree, don't bother to change PWM
     */

    if(ccr < 0)                 //ensure ccr never gets negative
    {
        ccr = 0;
    }
    else if(TA1CCR1 > 250){     //ensure CCR1 never gets too close to CCR0 (CCR0 is 262)
                                //if CCR1 is too close to CCR0, they won't both fire
        ccr = 250;
    }

    TA1CCR1 = ccr;              //finally set the CCR1 value

}

float PID(float difference, float kdt /*real and desired temp are globals*/){
    // Derivative:
    derivative = kd * (kdt * (realTemp - oldRealTemp));

    //Integral:
    integral = kp * (intergral + (difference * adcSamplingPeriod));  //using Reimann sums

    //Proportional
    proportion = difference * kp;

    sum = derivative + interal + proportion;

    int new_ccr = ccr + sum * 2.5;          // new_ccr must be an int, so decimals purposely get truncated
                                            // using 2.5 multiplier since ccr goes from 0 to 250, so 2.5 normalizes the change

    return new_ccr;
}

//UART interrupt
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)

{
  switch(__even_in_range(UCA1IV,4))         //looking for a specific interrupt case: when RX has value
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG (UART is receiving data in RX)
    // Special case: if UART input is 0, we want it to send back the temperature reading instead of setting desired temp
    if(UCA1RXBUF == 0x00){
        while (!(UCA1IFG & UCTXIFG));       //wait for TX buffer to be ready
        UCA1TXBUF = realTemp;               //send out temp reading
    }
    //if UART input is not special case of 0, set desired temp
    else
        desiredTemp = UCA1RXBUF;
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
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
  TA1IV &= ~TA1IV_TA1IFG; // Clear the Timer interrupt Flag
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
//  MSP430F552x Demo - ADC12, Sample A10 Temp and Convert to oC and oF
//
//  Description: A single sample is made on A10 with reference to internal
//  1.5V Vref. Software sets ADC12SC to start sample and conversion - ADC12SC
//  automatically cleared at EOC. ADC12 internal oscillator times sample
//  and conversion. In Mainloop MSP430 waits in LPM4 to save power until
//  ADC10 conversion complete, ADC12_ISR will force exit from any LPMx in
//  Mainloop on reti.
//  ACLK = n/a, MCLK = SMCLK = default DCO ~ 1.045MHz, ADC12CLK = ADC12OSC
//
//  Uncalibrated temperature measured from device to devive will vary do to
//  slope and offset variance from device to device - please see datasheet.
//
//  NOTE:1.REFMSTR bit in REFCTL0 regsiter is reset to allow the ADC12_A reference
//    control regsiters handle the reference setting. Upon resetting the REFMSTR
//    bit, all the settings in REFCTL are 'dont care' and the legacy ADC12
//    control bits (ADC12REFON, ADC12REF2_5, ADC12TCOFF and ADC12REFOUT) control
//    the reference system.
//    2. Use the TLV calibrated temperature to measure temperature
//   (the TLV CALIBRATED DATA IS STORED IN THE INFORMATION SEGMENT, SEE DEVICE DATASHEET)
//
//                MSP430F552x
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |A10              |
//
//   F. Chen
//   Texas Instruments Inc.
//   Dec. 2012
//   Built with IAR Embedded Workbench Version: 5.51.1 & Code Composer Studio V5.2.1
//******************************************************************************
