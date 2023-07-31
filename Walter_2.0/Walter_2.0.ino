#include <Wire.h>

float pid_p_gain = 10;
float pid_i_gain = 1;
float pid_d_gain = 15;
int pid_max = 200;

byte eeprom_data[36];
int start;
unsigned long cycle_timer;
float max_angle = 30;

// Gyro / Accelerometer Variables
const int gyro_address = 0x68;
float gyro_axis[4], acc_axis[4], gyro_axis_cal[4];
float gyro_roll, gyro_pitch, gyro_yaw;
float acc_X, acc_Y, acc_Z;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
int temperature;
int gyro_cal_int;

// PID Variables
float pid_i_mem, pid_error_temp, pid_last_d_error;
float pid_angle_input, balancing_setpoint, pid_setpoint;
float pid_output;

// Motor Variables
const int leftMotorStep = 2;
const int leftMotorDir = 3;
const int rightMotorStep = 4;
const int rightMotorDir = 5;
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
  TIMSK2 |= (1 << OCIE2A); // Enable Output Compare Match for timer 2. 
  // Caluculate the OCR register  with this equation: OCR2A = (speed * 16MHz / prescalar) - 1
  TCCR2B |= (1 << CS21); // Prescalar = 8
  OCR2A = 39; // 39 = (20us * 16MHz / 8) - 1
  TCCR2A |= (1 << WGM21); // Set mode to CTC (clear time on compare)

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
  
  ////////////////////////////////////////////////////////////////////////
  // Angle Calculations
  ////////////////////////////////////////////////////////////////////////
  read_gyro();

//  angle = 

//  Serial.print("Acc X: ");
//  Serial.print(acc_X/8192);
//  Serial.print(", Acc Y: ");
//  Serial.print(acc_Y/8192);
//  Serial.print(", Acc Z: ");
//  Serial.println(acc_Z/8192);
//  Serial.print("Gyro Roll: ");
//  Serial.print(gyro_roll/65.5);
//  Serial.print(", Gyro Pitch: ");
//  Serial.print(gyro_pitch/65.5);
//  Serial.print(", Gyro Yaw: ");
//  Serial.println(gyro_yaw/65.5);

  ////////////////////////////////////////////////////////////////////////
  // Ultrasonic Code
  ////////////////////////////////////////////////////////////////////////
//  send_signal(0);
//  us_distance[0] = receive_signal(0);
//  send_signal(1);
//  us_distance[1] = receive_signal(1);
//  Serial.print("Distance 1: ");
//  Serial.print(us_distance[0]);
//  Serial.print(", Distance 2: ");
//  Serial.println(us_distance[1]);

  ////////////////////////////////////////////////////////////////////////
  // Motor Calculations
  ////////////////////////////////////////////////////////////////////////
  // Get the PID output value
  calculate_PID();
  //
  
//  for (int x = 10; x < 30; x ++)throttle_left_motor = x;
//  for (int x = 10; x < 200; x ++)throttle_right_motor = x;
  throttle_left_motor = 65;
  throttle_right_motor = -throttle_left_motor;
  Serial.print("Left Memory: ");
  Serial.print(throttle_left_motor_memory);
  Serial.print(", Right Memory: ");
  Serial.println(throttle_right_motor_memory);

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
  throttle_counter_left_motor ++; // Increment the memory variable by 1.
  if(throttle_counter_left_motor > throttle_left_motor_memory){ // Check if the motor throttle counter is greater than the memory variable.
    throttle_counter_left_motor = 0; // Reset throttle counter variable to 0.
    throttle_left_motor_memory = throttle_left_motor; // Set new memory variable to the left motor throttle that was calculated.
    if(throttle_left_motor_memory < 0){ // Check if the memory variable is negative.
      PORTD &= 0b11110111; // Reverse direction of the motor by setting dir pin (3) LOW.
      throttle_left_motor_memory *= -1; // Set the memory variable to positive.
    }
    else PORTD |= 0b00001000; // Set dir pin (3) HIGH.
  }
  else if(throttle_counter_left_motor == 1){
    PORTD |= 0b00000100; // Set step pin (2) to HIGH
  } 
  else if(throttle_counter_left_motor == 2){
    PORTD &= 0b00000100; // Set step pin (2) to LOW
  }
  
  // Right Motor - step 4, dir 5
  throttle_counter_right_motor ++; // Increment the memory variable by 1.
  if(throttle_counter_right_motor > throttle_right_motor_memory){ // Check if the motor throttle counter is greater than the memory variable.
    throttle_counter_right_motor = 0; // Reset throttle counter variable to 0.
    throttle_right_motor_memory = throttle_right_motor; // Set new memory variable to the right motor throttle that was calculated.
    if(throttle_right_motor_memory < 0){ // Check if the memory variable is negative.
      PORTD &= 0b11011111; // Reverse direction of the motor by setting dir pin (5) LOW.
      throttle_right_motor_memory *= -1; // Set the memory variable to positive.
    }
    else PORTD |= 0b00100000; // Set dir pin (5) HIGH.
  }
  else if(throttle_counter_right_motor == 1){
    PORTD |= 0b00010000; // Set step pin (4) to HIGH
  } 
  else if(throttle_counter_right_motor == 2){
    PORTD &= 0b00010000; // Set step pin (4) to LOW
  }
}
