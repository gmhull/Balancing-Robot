#include <Wire.h>


//byte eeprom_data[36];
int start;
unsigned long cycle_timer;
bool remote_controlled = false;
bool debug = true;

// Gyro / Accelerometer Variables
const int gyro_address = 0x68;
float gyro_axis[4], acc_axis[4], gyro_axis_cal[4];
float gyro_roll, gyro_pitch, gyro_yaw;
float acc_X, acc_Y, acc_Z;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
int temperature;
int gyro_cal_int;
float angle_acc angle_gyro;

// PID Variables
float pid_p_gain = 10;
float pid_i_gain = 1;
float pid_d_gain = 15;
#define pid_max = 400;
#define max_angle = 30;
float pid_i_mem, pid_error_temp, pid_last_d_error;
float pid_angle_input, balancing_setpoint, pid_setpoint;
float pid_output;

// Motor Variables
#define leftMotorStep 2
#define leftMotorDir 3
#define rightMotorStep 4
#define rightMotorDir 5
#define target_speed 30
int throttle_counter_left_motor, throttle_left_motor_memory, throttle_left_motor;
int throttle_counter_right_motor, throttle_right_motor_memory, throttle_right_motor;

// Ultrasonic Variables
const int trigPin[2] = {8, 10};
const int echoPin[2] = {9, 11};


void setup() {
  Serial.begin(57600);
  Serial.println("Starting");
  Wire.begin(); 

  // Setup interrupt timer settings
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);  // Enable Output Compare Match for timer 2. 
  // Caluculate the OCR register  with this equation: OCR2A = (speed * 16MHz / prescalar) - 1
  TCCR2B |= (1 << CS21);    // Prescalar = 8
  OCR2A = 39;               // 39 = (20us * 16MHz / 8) - 1
  TCCR2A |= (1 << WGM21);   // Set mode to CTC (clear time on compare)

  pinMode(leftMotorStep, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorStep, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);
  pinMode(trigPin[0], OUTPUT);
  pinMode(trigPin[1], OUTPUT);
  pinMode(echoPin[0], INPUT);
  pinMode(echoPin[1], INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up the MPU6050 chip
  init_gyro();
  calibrate_gyro();

  Serial.println("Ready");
  start = 1;
  cycle_timer = micros();
}

void loop() {
  /*
   * Calculate the angle
   * Run the PID controller to get the output
   * convert output to motor throttles
   * send the motor signals to run wheels
  */
  if (debug == true) {
    update_PID_inputs();
  }
  ////////////////////////////////////////////////////////////////////////
  // Angle Calculations
  ////////////////////////////////////////////////////////////////////////
  // Get gyro data
  read_gyro();

  // Calculate upright angle using accelerometer data.
  angle_acc = atan(acc_Y/sqrt(acc_X**2 + acc_Z**2))*57.296; // Calculate angle and convert to degrees.

  // Start the robot.  Set gyro angle equal to the accelerometer angle at the start.
  if (start == 0 && abs(angle_acc) < 0.5) {
    angle_gyro = angle_acc;
    start = 1;
  }

  // Use the accelerometer angle to correct drift from gyro.
  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;

  // Convert raw gyro data by dividing by 65.5.  Multiply by 0.004 to factor in the 4 ms cycle time.
  angle_gyro += (gyro_pitch / 65.5) * 0.004;

  if (debug == true) {
    Serial.print("Acc X: ");
    Serial.print(acc_X/8192);
    Serial.print(", Acc Y: ");
    Serial.print(acc_Y/8192);
    Serial.print(", Acc Z: ");
    Serial.println(acc_Z/8192);
    Serial.print("Gyro Roll: ");
    Serial.print(gyro_roll/65.5);
    Serial.print(", Gyro Pitch: ");
    Serial.print(gyro_pitch/65.5);
    Serial.print(", Gyro Yaw: ");
    Serial.println(gyro_yaw/65.5);
  }

  ////////////////////////////////////////////////////////////////////////
  // Ultrasonic Code
  ////////////////////////////////////////////////////////////////////////
  // Send and receive signals from the ultrasonic sensors.
//  send_signal(0);
//  us_distance[0] = receive_signal(0);
//  send_signal(1);
//  us_distance[1] = receive_signal(1);
  
  if (debug == true) {
//    Serial.print("Distance 1: ");
//    Serial.print(us_distance[0]);
//    Serial.print(", Distance 2: ");
//    Serial.println(us_distance[1]);
  }
  
  ////////////////////////////////////////////////////////////////////////
  // Motor Calculations
  ////////////////////////////////////////////////////////////////////////
  // Get the PID output value
  calculate_PID();

  // Set the motor throttle values equal to the PID output.
  throttle_left_motor = pid_output;
  throttle_right_motor = pid_output;

  // Move the robot depending on the remote input.
//  if (remote_controlled) {
//    if (moving forward && nothing in front) {
//      if (pid_setpoint < 2) pid_setpoint += 0.05;
//      if (pid_output < target_speed) pid_setpoint += 0.05;
//    }
//    if (moving backward && nothing in back) {
//      if (pid_setpoint > -2) pid_setpoint -= 0.05;
//      if (pid_output > -target_speed) pid_setpoint -= 0.05;
//    }
//    if (turning left) {
//      throttle_left_motor -= turning_speed;
//      throttle_right_motor += turning_speed;
//    }
//    if (turning right) {
//      throttle_left_motor += turning_speed;
//      throttle_right_motor -= turning_speed;
//    }
//    if (no signal) {
//      if (pid_setpoint < -0.5) pid_setpoint += 0.05;
//      else if (pid_setpoint > 0.5) pid_setpoint -= 0.05;
//      else pid_setpoint = 0;
//  }

  // Adjust self balance point to help robot find its steady point.  Only activate if the robot is not being commanded.
  if (pid_setpoint == 0) {
    if (pid_output < 0) balancing_setpoint += 0.001; 
    if (pid_output > 0) balancing_setpoint -= 0.001;
  }

  // Compensate for stepper motors non-linear behavior.
  if (throttle_left_motor > 0) throttle_left_motor = 405 - (1 / (throttle_left_motor + 9)) * 5500;
  if (throttle_left_motor < 0) throttle_left_motor = -405 - (1 / (throttle_left_motor - 9)) * 5500;

  if (throttle_right_motor > 0) throttle_right_motor = 405 - (1 / (throttle_right_motor + 9)) * 5500;
  if (throttle_right_motor < 0) throttle_right_motor = -405 - (1 / (throttle_right_motor - 9)) * 5500;

  // Get the final motor pulse time (throttle * 20 us)
  if (throttle_left_motor > 0) throttle_left_motor = 400 - throttle_left_motor;
  if (throttle_left_motor < 0) throttle_left_motor = -400 - throttle_left_motor;
  if (throttle_left_motor == 0) throttle_left_motor = 0;

  if (throttle_right_motor > 0) throttle_right_motor = 400 - throttle_right_motor;
  if (throttle_right_motor < 0) throttle_right_motor = -400 - throttle_right_motor;
  if (throttle_right_motor == 0) throttle_right_motor = 0;
  
  if (debug) {
//    throttle_left_motor = 65;
//    throttle_right_motor = -throttle_left_motor;
//    Serial.print("Left Memory: ");
//    Serial.print(throttle_left_motor_memory);
//    Serial.print(", Right Memory: ");
//    Serial.println(throttle_right_motor_memory);
  }

  // Control the time of each cycle to be 4 milliseconds.
  while(micros() - cycle_timer < 4000);
  cycle_timer = micros();
}

////////////////////////////////////////////////////////////////////////
// Send Motor Signals
////////////////////////////////////////////////////////////////////////
// Run this interrupt every 20us.
ISR(TIMER2_COMPA_vect){
  // Add motor signals.  Working range - 65 to 400 cycles.
  // Arduino ports B - 0-7, D - 8-13
  // Left Motor - step 2, dir 3
  throttle_counter_left_motor ++;                                   // Increment the memory variable by 1.
  if(throttle_counter_left_motor > throttle_left_motor_memory){     // Check if the motor throttle counter is greater than the memory variable.
    throttle_counter_left_motor = 0;                                // Reset throttle counter variable to 0.
    throttle_left_motor_memory = throttle_left_motor;               // Set new memory variable to the left motor throttle that was calculated.
    if(throttle_left_motor_memory < 0){                             // Check if the memory variable is negative.
      PORTD &= 0b11110111;                                          // Reverse direction of the motor by setting dir pin (3) LOW.
      throttle_left_motor_memory *= -1;                             // Set the memory variable to positive.
    }
    else PORTD |= 0b00001000;                                       // Set dir pin (3) HIGH.
  }
  else if(throttle_counter_left_motor == 1){
    PORTD |= 0b00000100;                                            // Set step pin (2) to HIGH
  } 
  else if(throttle_counter_left_motor == 2){
    PORTD &= 0b00000100;                                            // Set step pin (2) to LOW
  }
  
  // Right Motor - step 4, dir 5
  throttle_counter_right_motor ++;                                  // Increment the memory variable by 1.
  if(throttle_counter_right_motor > throttle_right_motor_memory){   // Check if the motor throttle counter is greater than the memory variable.
    throttle_counter_right_motor = 0;                               // Reset throttle counter variable to 0.
    throttle_right_motor_memory = throttle_right_motor;             // Set new memory variable to the right motor throttle that was calculated.
    if(throttle_right_motor_memory < 0){                            // Check if the memory variable is negative.
      PORTD &= 0b11011111;                                          // Reverse direction of the motor by setting dir pin (5) LOW.
      throttle_right_motor_memory *= -1;                            // Set the memory variable to positive.
    }
    else PORTD |= 0b00100000;                                       // Set dir pin (5) HIGH.
  }
  else if(throttle_counter_right_motor == 1){
    PORTD |= 0b00010000;                                            // Set step pin (4) to HIGH
  } 
  else if(throttle_counter_right_motor == 2){
    PORTD &= 0b00010000;                                            // Set step pin (4) to LOW
  }
}
