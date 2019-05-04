/* Note: 
 *  Approx 6.5 ms for computations (time to run loop minus delays)
 *  Need to put resistor of approx 5151 ohms in series with pot since adc reference is 3.3VDC - done
 *  Desired relationship in volts: ptat = -0.091*pot + 0.8
 *  So, 0 V on pot gets us temp of 80 degrees C and 3.3 V on pot gets us 50 degrees C
 *  Therefore, low pot value gives us low PWM and hig hpot values gives us high PWM duty cycle
 *  Open loop design: 3.3V on pot = 100% duty cycle and 0V on pot gives 0% duty cycle
 */

// Ports
int lm35_pin = P6_0;               // connect PTAT output to A0 (6.0 on MSP430F5529LP)
int pot_pin = P6_1;                // connect potentiometer to A1 (6.1)
int pwm_out = P3_5;             
int toggle_pin = P2_7;

// PID Variables
const float kp = 150;             // proportional
const float ki = 1;             // integral
const float kd = 50;             // derivative
float integral = 0;             // integral value that accumulates over time
float derivative = 0;           // derivative
const float adc_sampling_period = 0.1027;   // rate at which ADC samples

// ADC Readings
float pot_array [3] = {0,0,0};   // 3 readings from pot
float pot = 0;                  // median filtered pot readings
float ptat_array [3] = {0,0,0};  // 3 readings from ptat
float ptat = 0;                 // median fitlered ptat value

// Calculated from ADC
float ptat_voltage = 0;
float old_ptat_voltage = 0;
float pot_voltage = 0;
float desired_ptat_voltage = 0.5;

// Parameters
int pwm_val = 150;              // can be from 0 to 255
float kdt = kd / adc_sampling_period; // setting this once instead of recalculating makes derivative calculation faster
float pot_scale = 1;            // for scaling pot reading
float ptat_offset = 0.5;
bool first_time = true;


void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_out, OUTPUT);   // sets the pin as output
  pinMode(toggle_pin, OUTPUT);
  Serial.begin(9600);              //  setup serial
}

void loop() {

  digitalWrite(toggle_pin, HIGH);
  // read the value from the sensor:
  for(int i = 0; i < 3; ++i){
    ptat_array[i] = analogRead(lm35_pin);
    pot_array[i] = analogRead(pot_pin);
    delay(100);
  }
  ptat = medianFilter(ptat_array);
  pot = medianFilter(pot_array);

  setPWM();
  
  analogWrite(pwm_out, pwm_val);
  digitalWrite(toggle_pin, LOW);
  delay(100);
  
  Serial.print("Temp: ");
  Serial.println(ptat_voltage);
  Serial.print("Pot: ");
  Serial.println(pot_voltage);
}

float medianFilter(float a[]){
    // ONLY WORKS IF WINDOW OF FILTER IS 3 NUMBERS WIDE
    // This code is complicated, but also efficient
    // Checks to see if the current value is between the other two

    if((a[0] <= a[1] && a[0] >= a[2]) || (a[0] >= a[1] && a[0] <= a[2]))
        return a[0];
    else if((a[1] <= a[0] && a[1] >= a[2]) || (a[1] >= a[0] && a[1] <= a[2]))
        return a[1];
    else
        return a[2];
}

//set CCR1 values to change duty Cycle of PWM: Low DC = slow fan, High DC = fast fan
float setPWM()
{

    //using the LM35 PTAT
    //PTAT equation: Vo = 10 mV/C + 0 mV
    //therefore, Temperature = Vo / 0.01

    //calculations*********************************************************************************************

    old_ptat_voltage = ptat_voltage;

    ptat_voltage = ptat * (3.3 / 4096);      //convert digital reading back to analog voltage value
                                                   // reference PTAT voltage is 3.3 V, ADC is 12 bits (2^12 = 4096)
    pot_voltage = pot * (3.3 / 4096);      //convert digital reading back to analog voltage value

    desired_ptat_voltage = -0.091 * pot_voltage + 0.8;

    float error = ptat_voltage - desired_ptat_voltage;
    //float error = (pot * pot_scale) - ptat_voltage;

    //use PID controller ***************************************************************************************
    pwm_val = PID(error);

}

float PID(float error){
    // Derivative:
    if(!first_time){
      derivative = (kdt * (ptat_voltage - old_ptat_voltage));
    }
    else{ //if system has just started (first_time == true), old_ptat_voltage will be 0, giving incorrect derivative. set derivative = 0 in this case
      derivative = 0;
      first_time = false;
    }

    //Integral:
    if(pwm_val != 255){ //ensure integral does not accumulate if system is alreasy at max PWM
      integral = ki * (integral + (error * adc_sampling_period*3));  //using Reimann sums, and multiplying period by 3 since using median filter
    }

    //Proportional
    float proportion = kp;

    // summing junction
    float sum = (derivative + integral + proportion) * error;

    int new_pwm = pwm_val + sum * 2.55;          // new_ccr must be an int, so decimals purposely get truncated;
                                                  // using 2.55 multiplier since ccr goes from 0 to 255, so 2.55 normalizes the change
    if(new_pwm > 255)
      return 255;
    else if (new_pwm < 0)
      return 0;
    else
      return new_pwm;
}
