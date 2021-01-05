///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Pada bagian ini berbagai register MPU-6050 diatur.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void) {
  HWire.beginTransmission(gyro_address);                        //Mulai komunikasi dengan MPU-6050.
  HWire.write(0x6B);                                            //Masukan ke register PWR_MGMT_1 (hex 6B).
  HWire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  HWire.endTransmission();                                      //Akhiri transmisi dengan gyro.
  
  HWire.beginTransmission(gyro_address);                        //Mulai komunikasi dengan MPU-6050.
  HWire.write(0x1B);                                            //Masukan ke register GYRO_CONFIG (hex 1B).
  HWire.write(0x08);                                            //Atur bit register sebagai 00001000 (skala penuh 500dps).
  HWire.endTransmission();                                      //Akhiri transmisi dengan gyro.

  HWire.beginTransmission(gyro_address);                        //Mulai komunikasi dengan MPU-6050.
  HWire.write(0x1C);                                            //Masukan ke register ACCEL_CONFIG (hex 1C).
  HWire.write(0x10);                                            //Atur bit register sebagai 00010000 (+/- 8g full scale range).
  HWire.endTransmission();                                      //Akhiri transmisi dengan gyro.

  HWire.beginTransmission(gyro_address);                        //Mulai komunikasi dengan MPU-6050.
  HWire.write(0x1A);                                            //Masukan ke register CONFIG register (1A hex).
  HWire.write(0x03);                                            //Atur bit register sebagai 00000011 (Set Digital Low Pass Filter to ~43Hz).
  HWire.endTransmission();                                      //Akhiri transmisi dengan gyro.

  acc_pitch_cal_value  = EEPROM.read(0x16);
  acc_roll_cal_value  = EEPROM.read(0x17);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subrutin ini menangani kalibrasi gyro. Ini menyimpan rata gyro offset 2000 pembacaan.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  cal_int = 0;                                                                        //Set variable cal_int = 0.
  if (cal_int != 2000) {
    //Mari kita ambil beberapa sampel data gyro sehingga kita dapat menentukan rata-rata offset giro (kalibrasi).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Ambil 2000 bacaan untuk kalibrasi.
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                    //Ubah status yang dipimpin setiap 125 pembacaan untuk menunjukkan kalibrasi.
      gyro_signalen();                                                                //Baca Output gyro.
      gyro_roll_cal += gyro_roll;                                                     //Tambahkan nilai Roll ke variable gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Tambahkan nilai Pitch ke variable gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Tambahkan nilai Yaw ke variable gyro_yaw_cal.
      delay(4);                                                                       //Penundaan kecil untuk mensimulasikan loop 250Hz selama kalibrasi.
    }
    red_led(HIGH);                                                                     //Set output PB3 low.
    //Sekarang kita memiliki 2000 ukuran, kita perlu membagi 2000 untuk mendapatkan offset gyro rata-rata.
    gyro_roll_cal /= 2000;                                                            
    gyro_pitch_cal /= 2000;                                                           
    gyro_yaw_cal /= 2000;                                                             
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Bagian ini membaca data gyro dan accelerometer mentah dari MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  HWire.beginTransmission(gyro_address);                       //Mulai komunikasi dengan gyro.
  HWire.write(0x3B);                                           //Mulailah membaca @ register 43H dan selisih otomatis dengan setiap baca.
  HWire.endTransmission();                                     //Akhiri transmisi dengan gyro.
  HWire.requestFrom(gyro_address, 14);                         //Minta 14 byte dari MPU 6050.
  acc_y = HWire.read() << 8 | HWire.read();                    //Tambahkan byte rendah dan tinggi ke variabel acc_x.
  acc_x = HWire.read() << 8 | HWire.read();                    //Tambahkan byte rendah dan tinggi ke variabel acc_y.
  acc_z = HWire.read() << 8 | HWire.read();                    //Tambahkan byte rendah dan tinggi ke variabel acc_z.
  temperature = HWire.read() << 8 | HWire.read();              //Tambahkan byte rendah dan tinggi ke variabel temperature.
  gyro_roll = HWire.read() << 8 | HWire.read();                //Baca data sudut yang tinggi dan rendah.
  gyro_pitch = HWire.read() << 8 | HWire.read();               //Baca data sudut yang tinggi dan rendah.
  gyro_yaw = HWire.read() << 8 | HWire.read();                 //Baca data sudut yang tinggi dan rendah.
  gyro_pitch *= -1;                                            //ubah nilai data menjadi negative.
  gyro_yaw *= -1;                                              //ubah nilai data menjadi negative.

  if (level_calibration_on == 0) {
    acc_y -= acc_pitch_cal_value;                              //Kurangi nilai acc_y dengan acc_pitch_cal_value.
    acc_x -= acc_roll_cal_value;                               //Kurangi nilai acc_x dengan acc_roll_cal_value.
  }
  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;                                  //Kurangi nilai gyro_roll dengan gyro_roll_call.
    gyro_pitch -= gyro_pitch_cal;                                //Kurangi nilai gyro_pitch dengan gyro_pitch_call.
    gyro_yaw -= gyro_yaw_cal;                                    //Kurangi nilai gyro_yaw dengan gyro_yaw_call.
  }
}
