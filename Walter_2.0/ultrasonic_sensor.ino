void send_signal(sensor_num) {
  digitalWrite(triggerPin[sensor_num], HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin[sensor_num], LOW);

  echo_timer[sensor_num] = micros();
}

float receive_signal(sensor_num) {
  float distance;
  time = pulseIn(echoPin[sensor_num], HIGH);
  distance = (0.034 * time) / 2;
  
  return distance;
}
