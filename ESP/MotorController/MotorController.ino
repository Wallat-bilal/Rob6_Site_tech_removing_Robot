// Define the size of the arrays
#define NUM_MOTOR 5
///////DEBUG////////
bool DEBUG = false;
bool DEV = 0;
int test_motor_n = 0;
int print_Counter = 0;
//int Print_Motor_Number = 0;
float Print_desired_current[NUM_MOTOR] = { 0 };

/*
if (DEBUG == 1){


    Serial.print("debug");
    Serial.println(duty_cycle);
}
*/


// PWMPIN arrays
int PWMPINR[NUM_MOTOR] = { 33, 26, 14, 13, 2 };  //{ 33, 26, 14, 13, 2 };
int PWMPINL[NUM_MOTOR] = { 25, 27, 12, 4, 15 };  //{ 25, 27, 12, 4, 15 };

// PWMCHN arrays
int PWMCHNR[NUM_MOTOR] = { 0, 2, 4, 6, 8 };
int PWMCHNL[NUM_MOTOR] = { 1, 3, 5, 7, 9 };

// R_EN and L_EN arrays
int R_EN[NUM_MOTOR] = { 5, 5, 5, 5, 5 };
int L_EN[NUM_MOTOR] = { 18, 18, 18, 18, 18 };

// CURSENS array
int CURSENS[NUM_MOTOR] = { 36, 39, 34, 35, 32 };  //{ 36, 39, 34, 35, 32 }

///////////CONSTANTS///////////
const float kt = 77;  //64.8; ;//59.8; //62.9; //77;  //Motor torque constant for current torque conversion 77.0442

const float kh = 1.8;  //2.184; // h-bridge constant, this mean the desired current is X2 when detected

const float max_cur = 43;  // The supply current in amps

const float max_out_pwm = 255.0;  // Maximum output current for 8 bit to PWM max (255)
const float pwm_limit = 100.0;


float detected_current[NUM_MOTOR];
float pid_current[NUM_MOTOR];

//////////PID///////////
/*
const float Kp = 0.019739746799967;  // Proportional gain
const float Ki = 3.94794935999339;   // Integral gain
const float Kd = 0;                  // Derivative gain
float integral[NUM_MOTOR];
float last_error[NUM_MOTOR];
*/
////////////PID//////////

const float Kp = 0.0065360021791939;   // Proportional gain
const float Ki = 1.30720043583878;   // Integral gain
const float Kd = 0;  // Derivative gain
const float Kn = 100;    // the N coefecient or dreivative filtercoeffeient

// PID CONTROLLER:
// Kp = 0.225325958396314
// Ki = 2.72535925041695
// Kd = -0.09835834825468
// N = 2.29086765276777

float integral[NUM_MOTOR] = {0.0};
float last_error[NUM_MOTOR] = {0.0};



///////////PWM////////

const int frequence = 20000;  // PWM frequency of 1 KHz // max for stable frq, is 300kHz
const int resolution = 8;     // 16-bit resolution, 65,536 possible values

////////////////////SERIAL PARSER///////
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;
int wn = 0;

//////////Traj////////
const int trajectory_size = 1;                     // Length of each batch of received force trajectories
const int buffer_size = 5 * trajectory_size;         // Length of the sent data
float receivedArray[2 * buffer_size];                // Combined array of received wire force trajectories [motor1, motor2, motor3, motor4, motor5, motor1...]
float desired_trajectory[5][trajectory_size];        // Force trajectories for the motors
float desired_wire_vel[5][trajectory_size];          // Wire velocity trajectories for the motors
uint8_t byteArray[2 * buffer_size * sizeof(float)];  // Array of bytes to receive the serial data into
const int wifi_sampling_interval = 1000000;          // Sampling interval for wifi communication (in microseconds)
unsigned long last_wifi_sampling_time = 0;

/////////OVERFLOWCHECKER/////

float last_desired_trajectory = 0;
float last_velocety_trajectory = 0;


////moving average///////
const int numSamples = 100;                       // number of samples to take for moving average
float sampleArray[NUM_MOTOR][numSamples];         // array to store the samples
int currentIndex[NUM_MOTOR] = { 0, 0, 0, 0, 0 };  // current index of the array
float moving_avg_cur[NUM_MOTOR];                  // current moving average
float sensor_cur = 0;

/////////ESPCLOCK////
hw_timer_t *timer = NULL;
volatile int time_index = 0;

void IRAM_ATTR onTimer() {
  time_index++;
}
////////TIMER CONSTANTS////////
int PIDTIME = 10000;     //100 Hz, should probaly be 1000 for 1000Hz, or 100 for 10000Hz
int SENSORTIME = 10000;  // for updating sensor moving average
unsigned long current_time = 0;
unsigned long oldtime = 0;   // for old school timing
unsigned long oldtime2 = 0;  //has to be ammended




void setup() {
  // Serial for debugging
  Serial.begin(460800);

  // Communication with ESP module running WiFi
  Serial2.begin(460800, SERIAL_8N1, 16, 17);

  //This is for setting up the PWM genretaion
  // The direction output


  for (int i = 0; i < 5; i++) {

    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    pinMode(PWMPINR[i], OUTPUT);
    pinMode(PWMPINL[i], OUTPUT);


    // Configuration of channel 0 with the chosen frequency and resolution
    ledcSetup(PWMCHNR[i], frequence, resolution);
    ledcSetup(PWMCHNL[i], frequence, resolution);

    // Assigns the PWM channel to pin
    ledcAttachPin(PWMPINR[i], PWMCHNR[i]);
    ledcAttachPin(PWMPINL[i], PWMCHNL[i]);
  }







  // Initialize the hardware timer
  timer = timerBegin(0, 80, true);              // timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);  // attach the interrupt handler
  timerAlarmWrite(timer, 800000, true);         // 10,000 microseconds = 10 milliseconds, I devide with 800000 because clock speed is 80MHz, should be 100Hz
  timerAlarmEnable(timer);                      // enable the alarm
}



void loop() {
  current_time = micros();

  // read from wifi
  if (current_time >= last_wifi_sampling_time + wifi_sampling_interval) {
    //Serial.print("WiFi sampling frequency: ");
    //Serial.print(1000000 / (float(current_time) - float(last_wifi_sampling_time)), 3);
    //Serial.println(" Hz");
    last_wifi_sampling_time = current_time;

    // Read serial for new motor forces
    Serial2.readBytes(byteArray, sizeof(byteArray));

    // Decode data
    memcpy(receivedArray, byteArray, sizeof(receivedArray));

    for (int i = 0; i < buffer_size; i++) {
      int sub_array_index = i % 5;
      int sub_array_element_index = i / 5;
      desired_trajectory[sub_array_index][sub_array_element_index] = receivedArray[i];
    }
    for (int i = buffer_size; i < 2 * buffer_size; i++) {
      int sub_array_index = i % 5;
      int sub_array_element_index = (i - buffer_size) / 5;
      desired_wire_vel[sub_array_index][sub_array_element_index] = receivedArray[i];
    }
    /*
    Serial.println("Received data: ");
    for (int j = 0; j < trajectory_size; j++){
      for (int i = 0; i < 5; i++){
        Serial.print(receivedArray[i+j*5], 2);
        Serial.print("\t");
        Serial.print(receivedArray[i+int(trajectory_size*5)+j*5], 6);
        Serial.print("\t");
      }
      Serial.println();
    }
    */
  }


  if (time_index >= trajectory_size) {  // This is the interrupt service checker,  should change every 100Hz

    time_index = 0;  // reset the counter
  }


  if ((micros() - oldtime) > SENSORTIME) {  // old school timing at 1000 Hz

    if (DEV == 0) {
      for (int i = 0; i < 5; i++) {
        detected_current[i] = read_sensor(i);  // reads the sensor input, undone :/
      }
    } else {

      detected_current[test_motor_n] = read_sensor(test_motor_n);  // reads the sensor input, undone :/
    }

    oldtime = micros();
  }

  if (current_time >= oldtime2 + PIDTIME) {  // loop for pid update currently at 100Hz

    if (DEV == 0) {
      for (int i = 0; i < 5; i++) {

        motor_loop(desired_trajectory[i][time_index], desired_wire_vel[i][time_index], detected_current[i], i, pid_current);
      }
    } else {
      wn = serial_parser();

      motor_loop(wn, detected_current[test_motor_n], 0, test_motor_n, pid_current);
    }


    //Serial.print("Sampling frequency:");
    //Serial.println(1000000/(float(current_time)-float(oldtime2)), 3);
    oldtime2 = current_time + PIDTIME;  //KINDA SCUFFED I KNOW
    if (DEV == 0) {
      if (print_Counter >= 1) {
        /////Sensor current//////
        print_Counter = 1;
        for (int i = 0; i < 5; i++) {
          Serial.print("Des");
          Serial.print(i+1);
          Serial.print(": ");
          Serial.print(Print_desired_current[i], 3);
          Serial.print("\t");
          Serial.print("PID");
          Serial.print(i+1);
          Serial.print(": ");
          Serial.print(pid_current[i], 3);
          Serial.print("\t");
          Serial.print("Sen");
          Serial.print(i+1);
          Serial.print(": ");
          Serial.print(moving_avg_cur[i], 3);
          Serial.print("\t");
        }
        Serial.println();
      }
      print_Counter++;
    }
  }

  //end of loop
}



float read_sensor(int Motor_Number) {

  const int CUTOFFCUR = 1;                       // the current in amps cutting off detection
  const float SENSITIVTYFACTOR = 40.0 / 1000.0;  // specified in data sheet in 40 mV/A
  const float QOUTVOLT = 1.5261;                 //3.3 / 2;     //1.5261; this is the measured QV    // quintasentive voltage, factor on vcc 5V for output at 0 current
  float cut_off = SENSITIVTYFACTOR / CUTOFFCUR;

  float raw_vol = (3.3 / 4095) * ((analogRead(CURSENS[Motor_Number]) - 300) * 1.35);  // ESP32 ADC is nonlinear, is tuned with facor and scalar -340 *1.35

  float volt_sens = raw_vol - QOUTVOLT;  // + 0.007;  // idk know what 0.007 is, to not make voltage zero at no current??

  float sensor_cur = volt_sens / SENSITIVTYFACTOR;  // Actual current

  sampleArray[Motor_Number][currentIndex[Motor_Number]] = sensor_cur;

  // increment the current index, or reset it to 0 if it reaches the end of the array
  currentIndex[Motor_Number]++;
  if (currentIndex[Motor_Number] >= numSamples) {
    currentIndex[Motor_Number] = 0;
  }

  // calculate the moving average
  float sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += sampleArray[Motor_Number][i];
  }
  moving_avg_cur[Motor_Number] = sum / numSamples;


  /*
  if (DEBUG) {
    Serial.print("Sensor current  = ");
    Serial.print(moving_avg_cur[Motor_Number]);
    Serial.print(", to Motor N: ");
    Serial.println(Motor_Number + 1);
  }
  */

  return moving_avg_cur[Motor_Number];
}




// This controller gets a desired torque and current current and outputs a corrected current
/////////////////////////////////////////////////////////////////
void motor_loop(float Desired_Trajectory, float Velocety_Trajectory, float Detected_Current, int Motor_Number, float* pidCurrent) {

  //// Desired_Trajectory Velocety_Trajectory Owerflow checker
  if (DEV == 0) {
    if (((Desired_Trajectory > 10000.0) || (Velocety_Trajectory > 10000.0)) || ((Desired_Trajectory < -10000.0) || (Velocety_Trajectory < -10000.0))) {
      Desired_Trajectory = last_desired_trajectory;
      Velocety_Trajectory = last_velocety_trajectory;
    }
    last_desired_trajectory = Desired_Trajectory;
    last_velocety_trajectory = Velocety_Trajectory;

    //Check if velocety has negative sign
    if (Velocety_Trajectory > -1e-9){//10e-8) {  //Torque, This statement is for backwards reeling at low torques
                                                 // IT is send with value 400?, in desired_trajectory when wire lengths has to increase.

      float desired_current = (420 / kt) * kh;  // SKIFT DEN UD SÅ DEN PASSER -insert line speed constant
      if (Motor_Number == 4) {
        desired_current = desired_current + ((200 / kt) * kh);
      }


      //PWMGNRTR
      float cur_frac = (desired_current / kh) / max_cur;
      int(duty_cycle) = floor((cur_frac * max_out_pwm) + 0.5);

      //-11,69 amps, to -39 bits
      digitalWrite(R_EN[Motor_Number], HIGH);
      digitalWrite(L_EN[Motor_Number], HIGH);
      ledcWrite(PWMCHNR[Motor_Number], 0);
      ledcWrite(PWMCHNL[Motor_Number], abs(duty_cycle));


      // this is for resetting the PID controller
      last_error[Motor_Number] = 0;
      integral[Motor_Number] = 0;
      /*
    if (DEBUG == 1) {
      Serial.print("Negative Stall = -11.68 ");
      Serial.print(",  to Motor N: ");
      Serial.println(Motor_Number + 1);
    }
    */
      return;
    }
  }

  //MAKE CURRENT
  float desired_current = (Desired_Trajectory / kt) * kh;
  Print_desired_current[Motor_Number] = desired_current;




  /*
  if (DEBUG == 1) {
    Serial.print("Desired current = ");
    Serial.print(desired_current);
    Serial.print(",  to Motor N: ");
    Serial.println(Motor_Number + 1);
    Serial.println();
    */
  
  ///PID
  /*
  float error = desired_current - Detected_Current;
  integral[Motor_Number] += error;
  float derivative = error - last_error[Motor_Number];
  last_error[Motor_Number] = error;
  float out_cur = Kp * error + Ki * integral[Motor_Number] + Kd * derivative;
*/
  


 float error = desired_current - Detected_Current;
  integral[Motor_Number] += error;
  // Limit the integral term
  float max_integral = 100.0;
  float min_integral = -100.0;
  integral[Motor_Number] = constrain(integral[Motor_Number], min_integral, max_integral);

  float derivative = error - last_error[Motor_Number];
  // Apply a low-pass filter to the derivative term
  float filtered_derivative = (1 - Kn) * filtered_derivative + Kn * derivative;


  last_error[Motor_Number] = error;
  float out_cur = Kp * error + Ki * integral[Motor_Number] + Kd * filtered_derivative;
  out_cur = constrain(out_cur, -max_cur, max_cur);

  pidCurrent[Motor_Number] = out_cur;

/*
  if (DEBUG == 1) {
    Serial.print("PID output      = ");
    Serial.print(out_cur);
    Serial.print(",  to Motor N: ");
    Serial.println(Motor_Number);
  }
*/
  ////
  //PWMGNRTR
  float cur_frac = (out_cur / kh) / max_cur;
  int(duty_cycle) = floor((cur_frac * max_out_pwm) + 0.5);


  ///ANALOGWRITE
  if (duty_cycle > pwm_limit) {
    duty_cycle = pwm_limit;
  } else if (duty_cycle < -pwm_limit) {
    duty_cycle = -pwm_limit;
  }

  if (duty_cycle > 0) {
    digitalWrite(L_EN[Motor_Number], HIGH);  // changed form HIGH
    digitalWrite(R_EN[Motor_Number], HIGH);
    ledcWrite(PWMCHNL[Motor_Number], 0);  // might be unnesedsary
    ledcWrite(PWMCHNR[Motor_Number], abs(duty_cycle));

    //if (DEBUG == 1) {
    //  Serial.print("SEND positiv DC = ");
    //  Serial.print(duty_cycle);
    //  Serial.print(",   to Motor N: ");
    //  Serial.println(Motor_Number + 1);
    //}


  } else {
    digitalWrite(R_EN[Motor_Number], HIGH);
    digitalWrite(L_EN[Motor_Number], HIGH);
    ledcWrite(PWMCHNR[Motor_Number], 0);
    ledcWrite(PWMCHNL[Motor_Number], abs(duty_cycle));

    //if (DEBUG == 1) {
    //  Serial.print("Send negativ DC = ");
    //  Serial.print(duty_cycle);
    //  Serial.print(",   to Motor N: ");
    //  Serial.println(Motor_Number + 1);
    //}
  }
  /*
  if (DEBUG == 1) {
    Serial.println(" ");
  }*/

  ///END OF LOOP//////
}

/////////////////////////////////////////////////////////////////

//////////SERIAL PARSER//////////////

// Parse the serial input to receive new values for the wn
int serial_parser() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  // If new data is available, update the PID constants with the new values
  if (newData == true) {

    strcpy(tempChars, receivedChars);
    wn = atoi(strtok(tempChars, ","));
    test_motor_n = atoi(strtok(NULL, ",")) - 1;


    newData = false;
  }

  return wn;
}