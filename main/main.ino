/*
 * Systems and Control: Fan PID control final project
 * James Cipparrone, Alex Revolus, and David Sheppard
 * May 2019
 * 
 * 
 * OVERVIEW
 *  Pins
 *    6.0: temeprature sensor ADC input (LM35)
 *    6.1: potentiometer ADC input
 *    3.5: PWM output
 * Use
 *    Pot is used to set desired temperature
 *    LM35 is read and PID controller sets new PWM duty cycle
 *    Temperature reading and Pot reading (both voltages) are sent to terminal
 *    0 V on pot = 80 deg C, 3.3 V on pot = 50 deg C
 *    Desired temperature = -0.091 * pot_voltage + 0.8
 *    
 *    
 * Notes
 *  Approx 8 ms for computations (time to run loop minus delays)
 *  Need to put resistor of approx 5151 ohms in series with 10k pot since adc reference is 3.3VDC
 *  Desired relationship in volts: ptat ~= -0.091*pot + 0.8
 *  So, 0 V on pot gets us temp of 80 degrees C and 3.3 V on pot gets us 50 degrees C
 *  Therefore, low pot value gives us low PWM and high pot values gives us high PWM duty cycle
 *  Basic open loop design would be: 3.3V on pot = 100% duty cycle and 0V on pot gives 0% duty cycle
 */

// Ports
int lm35_pin = P6_0;              // connect PTAT output to A0 (6.0 on MSP430F5529LP)
int pot_pin = P6_1;               // connect potentiometer to A1 (6.1)
int pwm_out = P3_5;             
//int toggle_pin = P2_7;          // for determining computation time 

// PID Variables
const float kp = 150;             // proportional
const float ki = 1;               // integral
const float kd = 50;              // derivative
float integral = 0;               // integral value that accumulates over time
float derivative = 0;             // derivative
const float adc_sampling_period = 0.1027;   // rate at which ADC samples

// ADC Readings and constants
const float volts_per_bit = 3.3 / 4096; // reference is 3.3 V and ADC is 12 bits (2^12 = 4096)
float pot_array [3] = {0,0,0};    // 3 readings from pot
float pot = 0;                    // median filtered pot readings
float ptat_array [3] = {0,0,0};   // 3 readings from ptat
float ptat = 0;                   // median fitlered ptat value

// Calculated from ADC
float ptat_voltage = 0;
float old_ptat_voltage = 0;
float pot_voltage = 0;
float desired_ptat_voltage = 0.5;

// Parameters
int pwm_val = 150;                // can be from 0 to 255
float kdt = kd / adc_sampling_period; // setting this once instead of recalculating makes derivative calculation faster
float pot_scale = 1;              // for scaling pot reading
float ptat_offset = 0.5;
bool first_time = true;
const float slope = -0.3 / 3.3;   // equation: desired_ptat-voltage = slope * pot_voltage + 0.8


void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_out, OUTPUT);       // sets the pin as output
  //pinMode(toggle_pin, OUTPUT);
  Serial.begin(9600);             // setup serial
}

void loop() {

  //digitalWrite(toggle_pin, HIGH);
  
  // read the value from the sensor **************************************
  for(int i = 0; i < 3; ++i){
    ptat_array[i] = analogRead(lm35_pin);
    pot_array[i] = analogRead(pot_pin);
    delay(100);
  }

  // apply median fitler to readings *************************************
  ptat = medianFilter(ptat_array);
  pot = medianFilter(pot_array);

  // calcuate voltages and call PID controller to get new PWM value ******
  setPWM();

  // set PWM value *******************************************************
  analogWrite(pwm_out, pwm_val);
  //digitalWrite(toggle_pin, LOW);
  //delay(100);

  //display real and desired temperature on terminal *********************
  Serial.print("Temp: ");
  Serial.println(ptat_voltage);
  Serial.print("Pot: ");
  Serial.println(pot_voltage);
}

float medianFilter(float a[]){
  // ONLY WORKS IF WINDOW OF FILTER IS 3 NUMBERS WIDE
  // The methodology is not easy to read, but is efficient
  // Checks to see which of the values is between the other two

  if((a[0] <= a[1] && a[0] >= a[2]) || (a[0] >= a[1] && a[0] <= a[2]))      //if a0 is between a1 and a2
      return a[0];
  else if((a[1] <= a[0] && a[1] >= a[2]) || (a[1] >= a[0] && a[1] <= a[2])) //if a1 is between a0 and a2
      return a[1];
  else                                                                      // if a2 is between a0 and a1
      return a[2];
}

// set PWM values to change duty Cycle of PWM: Low DC = slow fan, High DC = fast fan
// PWM values range from 0 (off) to 255 (100% DC)
void setPWM()
{
  // using the LM35 PTAT
  // PTAT equation: Vo = 10 mV/C + 0 mV
  // therefore, Temperature = Vo / 0.01
  // however, system is based on voltage readings, not temperature

  //calculations*********************************************************************************************
  old_ptat_voltage = ptat_voltage;          // store previous ptat voltage
  /* Notes on ADC:
   *  reference PTAT voltage is 3.3 V
   *  ADC is 12 bits (2^12 = 4096)
   *  3.3 / 4096 ~= 0.000805664063 = volts_per_bit
   */
  ptat_voltage = ptat * (volts_per_bit);    // convert digital ptat reading back to analog voltage value
  pot_voltage = pot * (volts_per_bit);      // convert digital pot reading back to analog voltage value
  desired_ptat_voltage = slope * pot_voltage + 0.8;   // desired_ptat-voltage = slope * pot_voltage + 0.8

  // if ptat_votlage > desired, then temp is too high, so increase duty cycle
  float error = ptat_voltage - desired_ptat_voltage;

  //use PID controller ***************************************************************************************
  pwm_val = PID(error);
}

float PID(float error){
  // Derivative **********************************************************************************************
  if(!first_time){
    derivative = (kdt * (ptat_voltage - old_ptat_voltage));
  }
  //if system has just started (first_time == true), old_ptat_voltage will be 0, giving incorrect derivative
  else{
    derivative = 0;
    first_time = false;
  }

  //Integral *************************************************************************************************
  if(pwm_val != 255){ //ensure integral does not accumulate if system is alreasy at max PWM
    //using Reimann sums, and multiplying period by 3 since using median filter
    integral = ki * (integral + (error * adc_sampling_period*3));
  }

  //Proportional *********************************************************************************************
  float proportion = kp;

  // summing junction ****************************************************************************************
  float sum = (derivative + integral + proportion) * error;

  int new_pwm = pwm_val + sum * 2.55; // new_ccr must be an int, so decimals purposely get truncated;
                                      // using 2.55 since pwm_val goes from 0 to 255, so 2.55 normalizes it
  // account for out of bounds values
  if(new_pwm > 255)
    return 255;
  else if (new_pwm < 0)
    return 0;
  else
    return new_pwm;
}
