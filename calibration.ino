///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Pada bagian ini prosedur level dan kompas ditangani.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_compass(void) {
  compass_calibration_on = 1;                                                //Tetapkan variabel compass_calibration_on untuk menonaktifkan penyesuaian nilai kompas mentah.
  red_led(HIGH);                                                             //Led merah akan menunjukkan bahwa kalibrasi kompas aktif.
  green_led(LOW);                                                            //Matikan led hijau karena kita tidak membutuhkannya.
  while (channel_2 < 1900) {                                                 //Tetap di loop ini sampai pilot menurunkan pitch stick pada transmiter.
    send_telemetry_data();                                                   //kirim data telemetry
    delayMicroseconds(3700);                                                 //Simulasikan loop program 250Hz.
    read_compass();                                                          //Baca nilai kompas mentah.
    //Dalam baris berikut nilai kompas maksimum dan minimum terdeteksi dan disimpan.
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
  }
  compass_calibration_on = 0;                                                //Reset variable compass_calibration_on.

  //Nilai maksimum dan minimum diperlukan untuk startup berikutnya dan disimpan
  for (error = 0; error < 6; error ++) EEPROM.write(0x10 + error, compass_cal_values[error]);

  setup_compass();                                                           //Inisialisasi kompas dan atur register yang benar.
  read_compass();                                                            //Bacadan kalkulasi data compas
  angle_yaw = actual_compass_heading;                                        //Set tujuan awal compas

  red_led(LOW);
  for (error = 0; error < 15; error ++) {
    green_led(HIGH);
    delay(50);
    green_led(LOW);
    delay(50);
  }

  error = 0;

  loop_timer = micros();                                                     //Set timer untuk loop berikutnya
}


void calibrate_level(void) {
  level_calibration_on = 1;

  while (channel_2 < 1100) {
    send_telemetry_data();                                                   //Send data telemetry
    delay(10);
  }
  red_led(HIGH);
  green_led(LOW);

  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  for (error = 0; error < 64; error ++) {
    send_telemetry_data();                                                   //Send data telemetry.
    gyro_signalen();
    acc_pitch_cal_value += acc_y;
    acc_roll_cal_value += acc_x;
    if (acc_y > 500 || acc_y < -500)error = 80;
    if (acc_x > 500 || acc_x < -500)error = 80;
    delayMicroseconds(3700);
  }

  acc_pitch_cal_value /= 64;
  acc_roll_cal_value /= 64;

  red_led(LOW);
  if (error < 80) {
    EEPROM.write(0x16, acc_pitch_cal_value);
    EEPROM.write(0x17, acc_roll_cal_value);
    //EEPROM.write(0x10 + error, compass_cal_values[error]);
    for (error = 0; error < 15; error ++) {
      green_led(HIGH);
      delay(50);
      green_led(LOW);
      delay(50);
    }
    error = 0;
  }
  else error = 3;
  level_calibration_on = 0;
  gyro_signalen();
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Hitung total vektor accelerometer.

  if (abs(acc_y) < acc_total_vector) {                                             //Cegah fungsi asin untuk menghasilkan NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //hitung sudut pitch
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Cegah fungsi asin untuk menghasilkan NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //kalkulasi sudut roll
  }
  angle_pitch = angle_pitch_acc;                                                   //Atur sudut pitch gyro sama dengan sudut pitch accelerometer ketika quadcopter dimulai.
  angle_roll = angle_roll_acc;
  loop_timer = micros();                                                           //Set timer untuk loop berikutnya.
}
