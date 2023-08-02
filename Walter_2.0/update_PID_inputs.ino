void update_PID_inputs() {
  byte input_byte;
  float pid_change_input;
  // Run when an input is sent and the robot is not active.
  if (start == 0 && Serial.available()){
    input_byte = Serial.read();
    input_counter = 0;
    if (toLowerCase(input_byte) == "p") {
      // Change PID p gain to input value
      Serial.print("Change the P gain value from ");
      Serial.print(pid_p_gain);
      Serial.println(" to:");
      pid_change_input = Serial.read();
      // Make sure the new PID value is valid
      if (pid_change_input >= 0 && isDigit(pid_change_input)) {
        pid_p_gain = pid_change_input;
        print_pid_vals(); // Print out the new PID values
      } else Serial.println("Entry is not valid. Enter a positive number.");
    }
    if (toLowerCase(input_byte) == "i") {
      // Change PID p gain to input value
      Serial.print("Change the I gain value from ");
      Serial.print(pid_i_gain);
      Serial.println(" to:");
      pid_change_input = Serial.read();
      // Make sure the new PID value is valid
      if (pid_change_input >= 0 && isDigit(pid_change_input)) {
        pid_d_gain = pid_change_input;
        print_pid_vals(); // Print out the new PID values
      } else Serial.println("Entry is not valid. Enter a positive number.");
    }
    if (toLowerCase(input_byte) == "d") {
      // Change PID p gain to input value
      Serial.print("Change the D gain value from ");
      Serial.print(pid_d_gain);
      Serial.println(" to:");
      pid_change_input = Serial.read();
      // Make sure the new PID value is valid
      if (pid_change_input >= 0 && isDigit(pid_change_input)) {
        pid_d_gain = pid_change_input;
        print_pid_vals(); // Print out the new PID values
      } else Serial.println("Entry is not valid. Enter a positive number.");
    }
  }
  if (input_counter <= 25) input_counter ++;
  else input_byte = 0x00;
}

void print_pid_vals() {
  Serial.print("P: ");
  Serial.print(pid_p_gain);
  Serial.print(", I: ");
  Serial.print(pid_i_gain);
  Serial.print(", D: ");
  Serial.println(pid_d_gain);
}
