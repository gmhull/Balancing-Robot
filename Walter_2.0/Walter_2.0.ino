#include <Wire.h>

float pid_p_gain = ;
float pid_i_gain = ;
float pid_d_gain = ;
int pid_max = 200;

byte eeprom_data[36];
int start;
int LED_PIN;
const int gyro_address = 0x68;
unsigned long cycle_timer;

// Gyro/Accelerometer Variables
float gyro_axis[4], acc_axis[4], gyro_axis_cal[4];
float gyro_X, gyro_Y, gyro_Z;
float acc_X, acc_Y, acc_Z;
float gyro_X_cal, gyro_Y_cal, gyro_Y_cal;
int temperature;

// PID Variables
float pid_i_mem, pid_temp_error, pid_last_d_error
float pid_angle_input, balancing_setpoint, pid_setpoint


void setup() {
  Serial.begin(9600);
  Wire.begin(); 

  // Set up the MPU6050 chip
  init_gyro();
  calibrate_gyro();

  

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
  calculate_pid();


  
  cycle_timer = micros()
}
