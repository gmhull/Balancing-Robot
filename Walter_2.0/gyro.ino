void read_gyro (){
  // Get all accelerometer, temp, and gyro data from the chip.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 14, true);

  while(Wire.available() < 14);
  acc_axis[1] = Wire.read()<<8|Wire.read();
  acc_axis[2] = Wire.read()<<8|Wire.read();
  acc_axis[3] = Wire.read()<<8|Wire.read();
  temperature = Wire.read()<<8|Wire.read();
  gyro_axis[1] = Wire.read()<<8|Wire.read();
  gyro_axis[2] = Wire.read()<<8|Wire.read();
  gyro_axis[3] = Wire.read()<<8|Wire.read();

  if(gyro_cal_int == gyro_cal_max){
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }

  gyro_roll = gyro_axis[1]; //gyro_axis[eeprom_data[6] & 0b00000011];
  gyro_pitch = gyro_axis[2]; //gyro_axis[eeprom_data[7] & 0b00000011];
  gyro_yaw = gyro_axis[3]; //gyro_axis[eeprom_data[8] & 0b00000011];
  
  acc_X = acc_axis[1]; //acc_axis[eeprom_data[7] & 0b00000011];
  acc_Y = acc_axis[2]; //acc_axis[eeprom_data[6] & 0b00000011];
  acc_Z = acc_axis[3]; //acc_axis[eeprom_data[8] & 0b00000011];
}

void init_gyro(){
  // By default the MPU6050 is asleep. We write 0 bits to the 6B register to wake it up.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Set the full scale of the gyro. Writing 0x08 to this registry sets the gyro scale to +/-250deg/s.
  // The data sheet says we need to divide the raw value by 131 to get the real values.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Set the full scale of the accelerometer. Writing 0x08 to this registry sets the accelerometer scale to +/-4g
  // The data sheet says we need to divide the raw value by 8192 to get the real values.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // Start filtering raw data to get consistent results.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  Serial.println("Gyro Initialized");
}

void calibrate_gyro (){
  for (gyro_cal_int = 0; gyro_cal_int < gyro_cal_max ; gyro_cal_int++){
    if(gyro_cal_int % 50 == 0)digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    read_gyro();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];
  }
  // Divide by the number of iterations to get the calibration
  gyro_axis_cal[1] /= gyro_cal_max;
  gyro_axis_cal[2] /= gyro_cal_max;
  gyro_axis_cal[3] /= gyro_cal_max;

  Serial.print("Gyro Calibrated -/- ");
  Serial.print(gyro_axis_cal[1]);
  Serial.print(" - ");
  Serial.print(gyro_axis_cal[2]);
  Serial.print(" - ");
  Serial.println(gyro_axis_cal[3]);
}
