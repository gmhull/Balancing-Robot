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

// Ultrasonic Variables
const int trigPin_1 = 8;
const int echoPin_1 = 9;
const int trigPin_2 = 10;
const int echoPin_2 = 11;


void setup() {
  Serial.begin(57600);
  Serial.println("Starting");
  Wire.begin(); 

  pinMode(leftMotorStep, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorStep, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up the MPU6050 chip
//  init_gyro();
//  calibrate_gyro();

  
//  digitalWrite(LED_BUILTIN, HIGH);
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
//  read_gyro();

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

  // Datasheet says 1us is the lowest delay for the high pulse
  // 640us was the lowest delay I could do for the low pulse
  Serial.println("Slow Spin");
  digitalWrite(rightMotorDir, HIGH);
  for (int x = 0; x < 200; x++){
    digitalWrite(rightMotorStep, HIGH);  
    delayMicroseconds(6); 
    digitalWrite(rightMotorStep, LOW);  
    delayMicroseconds(800); 
  }
  delay(1000);

  Serial.println("Fast Spin");
  digitalWrite(rightMotorDir, LOW);
  for (int x = 0; x < 200; x++){
    digitalWrite(rightMotorStep, HIGH);  
    delayMicroseconds(2); 
    digitalWrite(rightMotorStep, LOW);  
    delayMicroseconds(700); 
  }
  delay(1000);
  
  calculate_PID();

  ////////////////////////////////////////////////////////////////////////
  // Motor Calculations
  ////////////////////////////////////////////////////////////////////////


  // Control the time of each cycle to be 4 milliseconds.
  while(micros() - cycle_timer < 4000);
  cycle_timer = micros();
}
