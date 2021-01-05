void read_compass() {
  HWire.beginTransmission(compass_address);                     //Mulai komunikasi dengan kompas.
  HWire.write(0x03);                                            //Kami ingin mulai membaca di lokasi heksadesimal 0x03.
  HWire.endTransmission();                                      //Akhiri transmisi dengan gyro.

  HWire.requestFrom(compass_address, 6);                        //Ambil 6 byte dari kompas.
  compass_y = HWire.read() << 8 | HWire.read();                 //Tambahkan low dan high byte ke variable compass_y.
  compass_y *= -1;                                              //Invert arah sumbu.
  compass_z = HWire.read() << 8 | HWire.read();                 //Tambahkan low dan high byte ke variable compass_z.
  compass_x = HWire.read() << 8 | HWire.read();                 //Tambahkan low dan high byte ke variable compass_x.
  compass_x *= -1;                                              //Invert arah sumbu
  
  //Sebelum kompas dapat memberikan pengukuran yang akurat perlu dikalibrasi. Saat memulai variabel kompas_offset dan kompas_skala dihitung
  //Bagian berikut akan menyesuaikan nilai kompas mentah sehingga dapat digunakan untuk perhitungan pos.
  if (compass_calibration_on == 0) {                            //Saat kompas tidak dikalibrasi.
    compass_y += compass_offset_y;                              //Tambahkan y-offset ke nilai mentah.
    compass_y *= compass_scale_y;                               //Scale y-value sehingga cocok dengan sumbu lainnya.
    compass_z += compass_offset_z;                              //Tambahkan z-offset ke nilai mentah.
    compass_z *= compass_scale_z;                               //Scale z-value sehingga cocok dengan sumbu lainnya.
    compass_x += compass_offset_x;                              //Tambahkan x-offset ke nilai mentah.
  }

  //Nilai kompas berubah ketika sudut roll dan pitch quadcopter berubah. Itulah alasan bahwa nilai x dan y perlu dihitung untuk posisi horizontal virtual.
  //Nilai 0,0174533 adalah phi / 180 karena fungsinya dalam radian sebagai pengganti derajat.
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  //Sekarang setelah nilai horisontal diketahui, arah dapat dihitung. Dengan baris kode berikut judulnya dihitung dalam derajat.
  //Harap dicatat bahwa atan2 menggunakan radian sebagai pengganti derajat. Itu sebabnya 180 / 3.14 digunakan.
  if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += declination;                                 //Tambahkan deklinasi ke kompas magnetik yang mengarah ke utara geografis.
  if (actual_compass_heading < 0) actual_compass_heading += 360;         //Jika heading kompas menjadi lebih kecil dari 0, 360 ditambahkan untuk menjaganya dalam rentang 0 hingga 360 derajat.
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360; //Jika heading kompas menjadi lebih besar dari 360, 360 dikurangi untuk menjaganya dalam rentang 0 hingga 360 derajat.
}

//Pada saat startup, register kompas perlu diatur. Setelah itu kalibrasi offset dan nilai skala dihitung.
void setup_compass() {
  HWire.beginTransmission(compass_address);                     //Mulai komunikasi dengan kompas.
  HWire.write(0x00);                                            //tulis ke Konfigurasi Register A (hex 00).
  HWire.write(0x78);                                            //Set bit Konfigurasi Register A menjadi 01111000 untuk mengatur laju sampel (rata-rata 8 pada 75Hz).
  HWire.write(0x20);                                            //Set bit Konfigurasi Register sebagai 00100000 untuk mengatur gain pada +/- 1.3Ga.
  HWire.write(0x00);                                            //Set bit Mode Register sebagai 00000000 untuk mengatur Mode Pengukuran Continue.
  HWire.endTransmission();                                      //Akhiri pengiriman data dengan kompass.

//Baca nilai kalibrasi dari EEPROM.
  for (error = 0; error < 6; error ++)compass_cal_values[error] = EEPROM.read(0x10 + error);
  error = 0;
//Hitung nilai kalibrasi offset dan nilai skala
  compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}


//Subrouting berikut menghitung perbedaan terkecil antara dua nilai heading.
float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}
