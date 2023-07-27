void calculate_PID(){
  // Calculate PID
  pid_error_temp = pid_angle_input - balancing_setpoint - pid_setpoint;

  pid_i_mem += pid_i_gain * pid_error_temp;
  if(pid_i_mem > 400)pid_i_mem = 400;
  if(pid_i_mem < -400)pid_i_mem = -400;
  
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = pid_max;
  else if(pid_output < -400)pid_output = -pid_max;

  pid_last_d_error = pid_error_temp;

  // Allow a small dead band at the peak
  if(abs(pid_output) > 5)pid_output = 0;

  // Stop the robot if it goes past the tipping angle.
  if(abs(pid_angle_input) > max_angle){
    start = 0;
    balancing_setpoint = 0;
  }
}
