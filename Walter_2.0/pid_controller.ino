void calculate_PID(){
  // Calculate PID
  pid_error_temp = angle_gyro - balancing_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10) pid_error_temp += pid_output * 0.015;

  pid_i_mem += pid_i_gain * pid_error_temp;
  if(pid_i_mem > pid_max)pid_i_mem = pid_max;
  if(pid_i_mem < -pid_max)pid_i_mem = -pid_max;
  
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > pid_max)pid_output = pid_max;
  else if(pid_output < -pid_max)pid_output = -pid_max;
  
  pid_last_d_error = pid_error_temp;

  // Allow a small dead band at the peak
  if(pid_output < 5 && pid_output > -5)pid_output = 0;

  // Stop the robot if it goes past the tipping angle.
  if (start == 0 || angle_gyro > max_angle || angle_gyro < -max_angle){
    balancing_setpoint = 0;
    pid_i_mem = 0;
    pid_output = 0;
    start = 0;
  }
}
