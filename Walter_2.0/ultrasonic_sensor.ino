void send_signal(sensor_num) {
  digitalWrite(trigPin[sensor_num], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin[sensor_num], LOW);

  echo_timer[sensor_num] = micros();
}

float receive_signal(sensor_num) {
  float time = pulseIn(echoPin[sensor_num], HIGH);
  float distance = (0.034 * time) / 2.;
  
  return distance;
}
