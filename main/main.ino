// Ports
int lm35_pin = 0;               // connect PTAT output to A0 (6.0 on MSP430F5529LP)
int pot_pin = 1;                // connect potentiometer to A1 (6.1)
int pwm_out = 0;                // need to figure out which pin this is on the board

// PID Variables
const float kp = 1;             // proportional
const float ki = 1;             // integral
const float kd = 1;             // derivative
float integral = 0;             // integral value that accumulates over time
float derivative = 0;           // derivative
const float adc_sampling_period = 0.1;   // rate at which ADC samples

// ADC Readings
float pot_array[3] = {0,0,0};   // 3 readings from pot
float pot = 0;                  // median filtered pot readings
float ptat_array[3] = {0,0,0};  // 3 readings from ptat
float ptat = 0;                 // median fitlered ptat value

// Calculated from ADC
float real_temp = 0;            //temperature reading
float old_real_temp = 0;        // previous temp reading

// Parameters
float desired_temp = 20;        // initialization to 20 deg C
int pwm_val = 100;              // can be from 0 to 255
float kdt = kd / adc_sampling_period; // setting this once instead of recalculating makes derivative calculation faster
float pot_scale = 1;             // for scaling pot reading


void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_out, OUTPUT);   // sets the pin as output
}

void loop() {
  
  // read the value from the sensor:
  for(int i = 0; i < 3; ++i){
    ptat_array[i] = analogRead(lm35_pin);
    pot_array[i] = analogRead(pot_pin);
    delay(100);
  }
  ptat = medianFilter(ptat_array);
  pot = medianFilter(pot_array);
  
  analogWrite(pwm_out, pwm_val);
}

float medianFilter(float a[]){
    // ONLY WORKS IF WINDOW OF FILTER IS 3 NUMBERS WIDE
    // This code is complicated, but also efficient
    // Checks to see if the current value is between the other two

    if((ptat_array[0] <= a[1] && a[0] >= a[2]) || (a[0] >= a[1] && a[0] <= a[2]))
        return a[0];
    else if((a[1] <= a[0] && a[1] >= a[2]) || (a[1] >= a[0] && a[1] <= a[2]))
        return a[1];
    else
        return a[2];
}

//set CCR1 values to change duty Cycle of PWM: Low DC = slow fan, High DC = fast fan
void setPWM()
{

    //using the LM35 PTAT
    //PTAT equation: Vo = 10 mV/C + 0 mV
    //therefore, Temperature = Vo / 0.01

    //calculations*********************************************************************************************

    float analog_voltage = ptat * (5 / 4096);      //convert digital reading back to analog voltage value
                                                   // reference PTAT voltage is 5 V, ADC is 12 bits (2^12 = 4096)
    old_real_temp = real_temp;
    real_temp = (analog_voltage / 0.01);     //convert analog voltage to temperature

    float error = (pot * pot_scale) - analog_voltage;
    
    pwm_val = PID(error);

}

float PID(float error){
    // Derivative:
    derivative = kd * (kdt * (real_temp - old_real_temp));

    //Integral:
    integral = ki * (integral + (error * adc_sampling_period*3));  //using Reimann sums, and multiplying period by 3 since using median filter

    //Proportional
    float proportion = error * kp;

    // summing junction
    float sum = derivative + integral + proportion;

    int new_pwm =  pwm_val + sum * 2.55;          // new_ccr must be an int, so decimals purposely get truncated;
                                                  // using 2.55 multiplier since ccr goes from 0 to 255, so 2.55 normalizes the change

    return new_pwm;
}
