#include <Wire.h>
#include <NewPing.h>

// General Variables
int start;    // 0 = Stopped, 1 = Active
unsigned long cycle_timer, test_timer;
bool remote_controlled = false;
bool debug = true;
int test_timer_int;

// Gyro / Accelerometer Variables
const int gyro_address = 0x68;
float gyro_axis[4], acc_axis[4], gyro_axis_cal[4];
float gyro_roll, gyro_pitch, gyro_yaw;
float acc_X, acc_Y, acc_Z;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
int temperature;
int gyro_cal_int;
int gyro_cal_max = 600;
float angle_acc, angle_gyro, angle_acc_offset;

// PID Variables
float pid_p_gain = 15;
float pid_i_gain = 1.5;
float pid_d_gain = 30;
#define pid_max 400
#define max_angle 30
float pid_i_mem, pid_error_temp, pid_last_d_error;
float balancing_setpoint, pid_setpoint;
float pid_output, pid_output_left, pid_output_right;

// Motor Variables
#define leftMotorStep 2
#define leftMotorDir 3
#define rightMotorStep 4
#define rightMotorDir 5
#define target_speed 30
int left_motor, throttle_counter_left_motor, throttle_left_motor_memory, throttle_left_motor;
int right_motor, throttle_counter_right_motor, throttle_right_motor_memory, throttle_right_motor;

// Sonar Variables
#define SONAR_NUM 2             // Number of sonar sensors
#define MAX_DISTANCE 200        // Max calculated distance from sonar
#define CLOSE_DIST 15           // Safety margin for objects being too close (cm)
// Create an object for each sensor.  Give trigger pin, echo pin, and max distance for each
NewPing sonar[SONAR_NUM] = {
  NewPing(8, 9, MAX_DISTANCE),  // Front Sonar = 0
  NewPing(10, 11, MAX_DISTANCE) // Rear Sonar = 1
};
int sonar_count;
float sonar_dist[SONAR_NUM];    // Array to store the front and back distances

// Receiver Variables
int input_counter;


void setup() {
  Serial.begin(57600);
  Serial.println("Starting");
  Wire.begin(); 

  // Setup interrupt timer settings
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);  // Enable Output Compare Match for timer 0. 
  // Caluculate the OCR register  with this equation: OCR2A = (speed * 16MHz / prescalar) - 1
  TCCR2B |= (1 << CS21);    // Prescalar = 8
  OCR2A = 39;               // 39 = (20us * 16MHz / 8) - 1
  TCCR2A |= (1 << WGM21);   // Set mode to CTC (clear time on compare)

  pinMode(leftMotorStep, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorStep, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up the MPU6050 chip
  init_gyro();
  calibrate_gyro();
  
  // Angle offset found by reading the gyro angle when the robot is sitting on flat ground
  angle_acc_offset = -3.8;

  // Wait 0.5 seconds before starting
  delay(500);
  
  Serial.println("Ready");
  start = 0;
  cycle_timer = micros();
}


void loop() {
  // Reset the LED to off state
  digitalWrite(LED_BUILTIN, LOW);
  
  if (debug) debug_comments();

  ////////////////////////////////////////////////////////////////////////
  // Angle Calculations
  ////////////////////////////////////////////////////////////////////////
  // Get gyro data from the MPU6050 chip.
  read_gyro();

  // Set the max acceleration value to 1/-1
  acc_Z *= -1;                                // Compensate for gyro orientation.
  acc_Z /= 8192;                              // Convert raw acceleration data to real value.
  if (acc_Z > 1) acc_Z = 1;
  if (acc_Z < -1) acc_Z = -1;
  
  // Calculate upright angle using accelerometer data.
  angle_acc = asin((float)acc_Z) * 57.296;    // Calculate angle and convert to degrees.
  angle_acc -= angle_acc_offset;              // Subtract the angle offset based on how the chip is positioned.

  // Start the robot.  Set gyro angle equal to the accelerometer angle at the start.
  if (start == 0 && angle_acc < 0.5 && angle_acc > -0.5) {
    angle_gyro = angle_acc;
    start = 1;
  }

  // Convert raw gyro data by dividing by 131.  Multiply by 0.004 to factor in the 4 ms cycle time.
  angle_gyro += (gyro_pitch / 131) * 0.004;

  // Use the accelerometer angle to correct drift from gyro.
  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;

  ////////////////////////////////////////////////////////////////////////
  // Ultrasonic Code
  ////////////////////////////////////////////////////////////////////////
  // Each sonar sensor is pinged once every 12 cycles (48 ms).
//  sonar_count ++;
//  if (sonar_count < SONAR_NUM) {
//    sonar_dist[sonar_count] = sonar[sonar_count].ping_cm();
//  } else if (sonar_count == 12) {
//    // Set to -1 so that when the loop will start at 0 when the sonar_count increments.
//    sonar_count = -1;  
//  }
  
  ////////////////////////////////////////////////////////////////////////
  // Motor Calculations
  ////////////////////////////////////////////////////////////////////////
  // Get the PID output value
  calculate_PID();

  // Set the motor throttle values equal to the PID output.
  pid_output_left = pid_output;
  pid_output_right = pid_output;


  // Move the robot depending on the remote input.
//  if (remote_controlled) {
//    if (moving forward && sonar_dist[0] > CLOSE_DIST) {
//      if (pid_setpoint < 2) pid_setpoint += 0.05;
//      if (pid_output < target_speed) pid_setpoint += 0.05;
//    }
//    else if (moving backward && sonar_dist[1] > CLOSE_DIST) {
//      if (pid_setpoint > -2) pid_setpoint -= 0.05;
//      if (pid_output > -target_speed) pid_setpoint -= 0.05;
//    }
//    else if (turning left) {
//      throttle_left_motor -= turning_speed;
//      throttle_right_motor += turning_speed;
//    }
//    else if (turning right) {
//      throttle_left_motor += turning_speed;
//      throttle_right_motor -= turning_speed;
//    }
//    else {    // No signal
//      if (pid_setpoint < -0.5) pid_setpoint += 0.05;
//      else if (pid_setpoint > 0.5) pid_setpoint -= 0.05;
//      else pid_setpoint = 0;
//    }
//  }

  // Move the robot away if something gets too close to it
//  if (!remote_controlled && start == 1) {
//    if (sonar_dist[0] < CLOSE_DIST) {                           // Something is in front of the robot
//      if (pid_setpoint > -2) pid_setpoint -= 0.025;             // Start moving backwards
//      if (pid_output > -target_speed) pid_setpoint -= 0.025;
//    }
//    else if (sonar_dist[1] < CLOSE_DIST) {                      // Something is behind the robot
//      if (pid_setpoint < 2) pid_setpoint += 0.025;              // Start moving backwards
//      if (pid_output < target_speed) pid_setpoint += 0.025;
//    }
//    else {                                                      // No signal
//      if (pid_setpoint < -0.5) pid_setpoint += 0.025;
//      else if (pid_setpoint > 0.5) pid_setpoint -= 0.025;
//      else pid_setpoint = 0;
//    }
//  }

  // Adjust self balance point to help robot find its steady point.  Only activate if the robot is not being commanded.
  if (pid_setpoint == 0) {
    if (pid_output < 0) balancing_setpoint += 0.0015; 
    if (pid_output > 0) balancing_setpoint -= 0.0015;
  }

  // Compensate for stepper motors non-linear behavior.
  if (pid_output_left > 0) pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
  else if (pid_output_left < 0) pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

  if (pid_output_right > 0) pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
  else if (pid_output_right < 0) pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

  // Get the final motor pulse time (throttle * 20 us)
  if (pid_output_left > 0) left_motor = 400 - pid_output_left;
  else if (pid_output_left < 0) left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if (pid_output_right > 0) right_motor = 400 - pid_output_right;
  else if (pid_output_right < 0) right_motor = -400 - pid_output_right;
  else right_motor = 0;

  // Copy final motor calculations into throttle variables for the motor pulse timer to use.
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
  

  // Control the time of each cycle to be 4 milliseconds.  Otherwise the angle calculations will be off.
  if (micros() - cycle_timer > 4050) digitalWrite(LED_BUILTIN, HIGH);  // Flash the LED if the cycle is longer than expected
  while(micros() - cycle_timer < 4000);
  cycle_timer = micros();
}


////////////////////////////////////////////////////////////////////////
// Send Motor Signals
////////////////////////////////////////////////////////////////////////
// Run this interrupt every 20us.
ISR(TIMER2_COMPA_vect){
  // Left Motor - step 2, dir 3
  throttle_counter_left_motor ++;                                   // Increment the memory variable by 1.
  if(throttle_counter_left_motor > throttle_left_motor_memory){     // Check if the motor throttle counter is greater than the memory variable.
    throttle_counter_left_motor = 0;                                // Reset throttle counter variable to 0.
    throttle_left_motor_memory = throttle_left_motor;               // Set new memory variable to the left motor throttle that was calculated.
    if(throttle_left_motor_memory < 0){                             // Check if the memory variable is negative.
      PORTD |= 0b00001000;                                          // Reverse direction of the motor by setting dir pin (3) HIGH.
      throttle_left_motor_memory *= -1;                             // Set the memory variable to positive.
    } 
    else PORTD &= 0b11110111;                                       // Set dir pin (3) LOW.
  }
  else if(throttle_counter_left_motor == 1) PORTD |= 0b00000100;    // Set step pin (2) to HIGH
  else if(throttle_counter_left_motor == 2) PORTD &= 0b11111011;    // Set step pin (2) to LOW
  
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
  else if(throttle_counter_right_motor == 1) PORTD |= 0b00010000;   // Set step pin (4) to HIGH
  else if(throttle_counter_right_motor == 2) PORTD &= 0b11101111;   // Set step pin (4) to LOW
}
