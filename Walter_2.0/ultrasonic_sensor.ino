void send_signal(int sensor_num) {
  digitalWrite(trigPin[sensor_num], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin[sensor_num], LOW);
}

float receive_signal(int sensor_num) {
  float time = pulseIn(echoPin[sensor_num], HIGH);
  float distance = (0.034 * time) / 2.;
  
  return distance;
}
