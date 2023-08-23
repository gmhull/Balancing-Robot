void debug_comments() {
  // 0 = Off, 1 = PID Tuning, 2 = Angle/Gyro Readings, 3 = MPU data, 
  // 4 = Sonar Sensors, 5 = PID Outputs, 6 = Motor Outputs, 7 = Timing
  int debug_section = 0;

  if (debug_section == 1) {
   update_PID_inputs();
  }
  else if (debug_section == 2) {
    Serial.print(start);
    Serial.print(" - ");
    Serial.print(angle_acc);
    Serial.print(" - ");
    Serial.println(angle_gyro);
  }
  else if (debug_section == 3) {
    // Print out acceleration and gyro data.
    Serial.print("Acc X: ");
    Serial.print(acc_X/8192);
    Serial.print(", Acc Y: ");
    Serial.print(acc_Y/8192);
    Serial.print(", Acc Z: ");
    Serial.print(acc_Z);
    Serial.print(", Gyro Roll: ");
    Serial.print(gyro_roll/131);
    Serial.print(", Gyro Pitch: ");
    Serial.print(gyro_pitch/131);
    Serial.print(", Gyro Yaw: ");
    Serial.print(gyro_yaw/131);
    Serial.print(", Final Angle: ");
    Serial.println(angle_gyro);
  }
  else if (debug_section == 4) {
    if (sonar_count >= 0 && sonar_count < SONAR_NUM) {
      Serial.print("Sonar sensor ");
      Serial.print(sonar_count);
      Serial.print(": ");
      Serial.print(sonar_dist[sonar_count]);
      Serial.print(" cm");
      Serial.print(", PID Setpoint: ");
      Serial.print(pid_setpoint);
      // Keep everything on one line
      if (sonar_count == SONAR_NUM-1) Serial.print("\n");
      else Serial.print(" / ");
    }
  }
  else if (debug_section == 5) {
    Serial.print("PID Output: ");
    Serial.print(pid_output);
    Serial.print(", PID Error: ");
    Serial.print(pid_error_temp);
    Serial.print(", Balancing Setpoint: ");
    Serial.print(balancing_setpoint);
    Serial.print(", PID Setpoint: ");
    Serial.print(pid_setpoint);
    Serial.print(", Final Angle: ");
    Serial.println(angle_gyro);
  }
  else if (debug_section == 6) {
    Serial.print("Left Throttle: ");
    Serial.println(throttle_left_motor);
  }
  else if (debug_section == 7) {
    test_timer_int ++;
    if (test_timer_int == 1) test_timer = micros();
    else if (test_timer_int == 2) {
      test_timer = micros() - test_timer;
      Serial.print("Cycle Time: ");
      Serial.println(test_timer);
    }
    else if (test_timer_int >= 6) test_timer_int = 0;
  }
}
