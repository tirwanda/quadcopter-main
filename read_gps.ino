///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Pada bagian ini modul GPS diatur dan dibaca.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gps_setup(void) {

  Serial1.begin(9600);
  delay(250);

  //Nonaktifkan pesan GPGSV dengan menggunakan protokol ublox.
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial1.write(Disable_GPGSV, 11);
  delay(350);   //Sedikit penundaan ditambahkan untuk memberi GPS waktu untuk merespons @ 9600bps.
  //Atur kecepatan refresh ke 5Hz dengan menggunakan protokol ublox.
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial1.write(Set_to_5Hz, 14);
  delay(350);   //Sedikit penundaan ditambahkan untuk memberi GPS waktu untuk merespons @ 9600bps.
  //Atur baud rate ke 57.6kbps dengan menggunakan protokol ublox.
  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                              };
  Serial1.write(Set_to_57kbps, 28);
  delay(200);

  Serial1.begin(57600);
  delay(200);
}

void read_gps(void) {
  while (Serial1.available() && new_line_found == 0) {                                                   //Tetap di loop ini selama ada informasi serial dari GPS yang tersedia.
    char read_serial_byte = Serial1.read();                                                              //Muat byte serial baru dalam variabel read_serial_byte.
    if (read_serial_byte == '$') {                                                                       //Jika byte baru sama dengan $ karakter.
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             //Hapus data lama dari array buffer yang masuk.
        incomming_message[message_counter] = '-';                                                        //Tulis "-" di setiap posisi.
      }
      message_counter = 0;                                                                               //Setel ulang variabel message_counter karena kami ingin mulai menulis di awal array.
    }
    else if (message_counter <= 99)message_counter ++;                                                   //Jika byte yang diterima tidak sama dengan $ karakter, tambahkan variabel message_counter.
    incomming_message[message_counter] = read_serial_byte;                                               //Tulis byte yang diterima baru ke posisi baru di array incomming_message.
    if (read_serial_byte == '*') new_line_found = 1;                                                     //Setiap garis NMEA diakhiri dengan *. Jika karakter ini terdeteksi, variabel new_line_found diatur ke 1.
  }

  //Jika perangkat lunak telah mendeteksi jalur NMEA baru, ia akan memeriksa apakah itu jalur yang valid yang dapat digunakan.
  if (new_line_found == 1) {                                                                             //Jika garis NMEA baru ditemukan.
    new_line_found = 0;                                                                                  //Setel ulang variabel new_line_found untuk baris berikutnya.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //Ketika tidak ada informasi GPS fix atau lintang / bujur tersedia.
      digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                                      //Ubah LED pada STM32 untuk menunjukkan penerimaan GPS.
      //Atur beberapa variabel ke 0 jika tidak ada informasi yang valid ditemukan oleh modul GPS. Ini diperlukan agar GPS hilang saat terbang.
      l_lat_gps = 0;
      l_lon_gps = 0;
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;
    }
    //Jika garis dimulai dengan GA dan jika ada perbaikan GPS, kami dapat memindai garis untuk lintang, bujur dan jumlah satelit.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
      lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;                              //Saring menit untuk garis GGA dikalikan dengan 10000000.
      lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;                               //Saring menit untuk garis GGA dikalikan dengan 1000000.
      lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;                                //Saring menit untuk garis GGA dikalikan dengan 100000.
      lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;                                 //Saring menit untuk garis GGA dikalikan dengan 10000.
      lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;                                  //Saring menit untuk garis GGA dikalikan dengan 1000.
      lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;                                   //Saring menit untuk garis GGA dikalikan dengan 100.
      lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;                                    //Saring menit untuk garis GGA dikalikan dengan 10.
      lat_gps_actual /= (long)6;                                                                         //Untuk mengkonversi menit ke derajat kita perlu membagi menit dengan 6.
      lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;                            //Tambahkan derajat dikalikan dengan 100000000.
      lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;                             //Tambahkan derajat dikalikan dengan 10000000.
      lat_gps_actual /= 10;                                                                              //Bagi dengan 10.

      lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;                              //Saring menit untuk garis GGA dikalikan dengan 10000000.
      lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;                               //Saring menit untuk garis GGA dikalikan dengan 1000000.
      lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;                                //Saring menit untuk garis GGA dikalikan dengan 100000.
      lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;                                 //Saring menit untuk garis GGA dikalikan dengan 10000.
      lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;                                  //Saring menit untuk garis GGA dikalikan dengan 1000.
      lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;                                   //Saring menit untuk garis GGA dikalikan dengan 100.
      lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;                                    //Saring menit untuk garis GGA dikalikan dengan 10.
      lon_gps_actual /= (long)6;                                                                         //Untuk mengkonversi menit ke derajat kita perlu membagi menit dengan 6.
      lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;                            //Tambahkan derajat dikalikan dengan 1000000000.
      lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;                             //Tambahkan derajat dikalikan dengan 100000000.
      lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;                              //Tambahkan derajat dikalikan dengan 10000000.
      lon_gps_actual /= 10;                                                                              //Bagi dengan 10.

      if (incomming_message[28] == 'N')latitude_north = 1;                                               //Saat terbang ke utara khatulistiwa, variabel latitude_north akan ditetapkan ke 1.
      else latitude_north = 0;                                                                           //Saat terbang ke selatan khatulistiwa, variabel latitude_north akan ditetapkan ke 0.

      if (incomming_message[42] == 'E')longiude_east = 1;                                                //Saat terbang ke timur dari meridian utama variabel longiude_east akan ditetapkan ke 1.
      else longiude_east = 0;                                                                            //Saat terbang ke barat meridian utama variabel longiude_east akan diatur ke 0.

      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   //Filter jumlah satelit dari jalur GAS.
      number_used_sats += (int)incomming_message[47] - 48;                                               //Filter jumlah satelit dari jalur GAS.
      
      if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //Jika ini pertama kalinya kode GPS digunakan.
        lat_gps_previous = lat_gps_actual;                                                               //Setel variabel lat_gps_previous ke variabel lat_gps_actual.
        lon_gps_previous = lon_gps_actual;                                                               //Setel variabel lon_gps_previous ke variabel lon_gps_actual.
      }

      lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              //Bagilah perbedaan antara garis lintang baru dan sebelumnya dengan sepuluh.
      lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              //Bagilah perbedaan antara garis bujur baru dan sebelumnya dengan sepuluh.
      
      l_lat_gps = lat_gps_previous;                                                                      //Setel variabel l_lat_gps ke nilai garis lintang sebelumnya.
      l_lon_gps = lon_gps_previous;                                                                      //Setel variabel l_lat_gps ke nilai garis bujur sebelumnya.

      lat_gps_previous = lat_gps_actual;                                                                 //Ingat nilai lintang baru di variabel lat_gps_previous untuk loop berikutnya.
      lon_gps_previous = lon_gps_actual;                                                                 //Ingat nilai bujur baru di variabel lat_gps_previous untuk loop berikutnya.

      //GPS diatur ke kecepatan refresh 5Hz. Di antara setiap 2 pengukuran GPS, 9 nilai GPS disimulasikan.
      gps_add_counter = 5;                                                                               //Set variabel gps_add_counter ke 5 sebagai penghitung waktu mundur loop
      new_gps_data_counter = 9;                                                                          //Atur new_gps_data_counter ke 9. Ini adalah jumlah nilai simulasi antara 2 pengukuran GPS.
      lat_gps_add = 0;                                                                                   //Reset variable lat_gps_add.
      lon_gps_add = 0;                                                                                   //Reset variable lon_gps_add.
      new_gps_data_available = 1;                                                                        //Atur new_gps_data_available untuk menunjukkan bahwa ada data baru yang tersedia.
    }

    //Jika saluran dimulai dengan SA dan jika ada perbaikan GPS, kami dapat memindai jalur untuk jenis perbaikan (tidak ada, 2D atau 3D).
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A')fix_type = (int)incomming_message[9] - 48;

  }

  //Setelah 5 loop program 5 x 4ms = 20 ms, gps_add_counter adalah 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 //Jika gps_add_counter bernilai 0 dan diperlukan simulasi GPS baru.
    new_gps_data_available = 1;                                                                           //Atur new_gps_data_available untuk menunjukkan bahwa ada data baru yang tersedia.
    new_gps_data_counter --;                                                                              //Kurangi new_gps_data_counter sehingga hanya akan ada 9 simulasi
    gps_add_counter = 5;                                                                                  //Atur variabel gps_add_counter ke 5 sebagai penghitung waktu mundur loop

    lat_gps_add += lat_gps_loop_add;                                                                      //Tambahkan bagian yang disimulasikan ke variabel buffer float karena l_lat_gps hanya dapat menyimpan bilangan bulat.
    if (abs(lat_gps_add) >= 1) {                                                                          //Jika nilai absolut dari lat_gps_add lebih besar dari 1.
      l_lat_gps += (int)lat_gps_add;                                                                      //Tambahkan nilai lat_gps_add dengan nilai lat_gps_add sebagai integer. Jadi tidak ada bagian desimal.
      lat_gps_add -= (int)lat_gps_add;                                                                    //Kurangi nilai lat_gps_add sebagai bilangan bulat sehingga nilai desimal tetap.
    }

    lon_gps_add += lon_gps_loop_add;                                                                      //Tambahkan bagian yang disimulasikan ke variabel buffer float karena l_lat_gps hanya dapat menyimpan bilangan bulat.
    if (abs(lon_gps_add) >= 1) {                                                                          //Jika nilai absolut dari lat_gps_add lebih besar dari 1.
      l_lon_gps += (int)lon_gps_add;                                                                      //Tambahkan nilai lat_gps_add dengan nilai lat_gps_add sebagai integer. Jadi tidak ada bagian desimal.
      lon_gps_add -= (int)lon_gps_add;                                                                    //Kurangi nilai lat_gps_add sebagai bilangan bulat sehingga nilai desimal tetap.
    }
  }

  if (new_gps_data_available) {                                                                           //Jika ada set data GPS baru tersedia.
    if (number_used_sats < 8)digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                //Ubah LED pada STM32 untuk menunjukkan penerimaan GPS.
    else digitalWrite(STM32_board_LED, LOW);                                                              //Nyalakan LED pada STM solid (fungsi LED terbalik). Periksa skema STM32.
    gps_watchdog_timer = millis();                                                                        //Reset the GPS watch dog tmer.
    new_gps_data_available = 0;                                                                           //Reset variable new_gps_data_available.

    if (flight_mode >= 3 && waypoint_set == 0) {                                                          //Jika mode penerbangan adalah 3 (tahan GPS) dan tidak ada titik arah yang ditetapkan.
      waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
      l_lat_waypoint = l_lat_gps;                                                                         //Ingat garis lintang saat ini sebagai titik penahan GPS.
      l_lon_waypoint = l_lon_gps;                                                                         //Ingat bujur saat ini sebagai titik penahanan GPS.
    }

    if (flight_mode >= 3 && waypoint_set == 1) {                                                          //Jika mode penahanan GPS dan titik arah disimpan.
      //Pengaturan GPS stick move
      if (flight_mode == 3 && takeoff_detected == 1) {
        if (!latitude_north) {
          l_lat_gps_float_adjust += 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //Koreksi selatan
        }
        else {
          l_lat_gps_float_adjust -= 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //Koreksi utara
        }

        if (!longiude_east) {
          l_lon_gps_float_adjust -= (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //Koreksi barat
        }

        else {
          l_lon_gps_float_adjust += (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //Koreksi timur
        }
      }

      if (l_lat_gps_float_adjust > 1) {
        l_lat_waypoint ++;
        l_lat_gps_float_adjust --;
      }
      if (l_lat_gps_float_adjust < -1) {
        l_lat_waypoint --;
        l_lat_gps_float_adjust ++;
      }

      if (l_lon_gps_float_adjust > 1) {
        l_lon_waypoint ++;
        l_lon_gps_float_adjust --;
      }
      if (l_lon_gps_float_adjust < -1) {
        l_lon_waypoint --;
        l_lon_gps_float_adjust ++;
      }

      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Hitung kesalahan garis lintang antara titik arah dan posisi aktual.
      gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Hitung kesalahan garis bujur antara titik jalan dan posisi aktual.

      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Kurangi posisi memori saat ini untuk memberikan ruang bagi nilai baru.
      gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Hitung perubahan baru antara tekanan aktual dan pengukuran sebelumnya.
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Tambahkan nilai baru ke nilai rata-rata jangka panjang.

      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Kurangi posisi memori saat ini untuk memberikan ruang bagi nilai baru.
      gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Hitung perubahan baru antara tekanan aktual dan pengukuran sebelumnya.
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Tambahkan nilai baru ke nilai rata-rata jangka panjang.
      gps_rotating_mem_location++;                                                                        //Tambah lokasi memori yang berputar.
      if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Mulai dari 0 ketika lokasi memori 35 tercapai.

      gps_lat_error_previous = gps_lat_error;                                                             //Ingat eror untuk loop berikutnya.
      gps_lon_error_previous = gps_lon_error;                                                             //Ingat eror untuk loop berikutnya.

      //Hitung koreksi GPS pitch and roll seolah-olah arah multicopter menghadap ke utara.
      //Bagian Proporsional = (float) gps_lat_error * gps_p_gain.
      //Bagian Derivatif = (float) gps_lat_total_avarage * gps_d_gain.
      gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
      gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

      if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //invert penyetelan pitch karena quadcopter terbang ke selatan khatulistiwa.
      if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //invert penyetelan roll karena quadcopter terbang ke barat meridian utama.

      //Karena koreksi dihitung seolah-olah hidung menghadap ke utara, kita perlu mengubahnya untuk pos saat ini.
      gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
      gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

      //Batasi koreksi maksimum hingga 300. Dengan cara ini kita masih memiliki kontrol penuh dengan pitch and roll stick pada transmiter.
      if (gps_roll_adjust > 300) gps_roll_adjust = 300;
      if (gps_roll_adjust < -300) gps_roll_adjust = -300;
      if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
      if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
    }
  }

  if (gps_watchdog_timer + 1000 < millis()) {                                                             //Jika watchdog timer terlampaui, sinyal GPS tidak ada.
    if (flight_mode >= 3 && start > 0) {                                                                  //Jika mode penerbangan 3 (tahan GPS).
      flight_mode = 2;                                                                                    //Atur mode penerbangan ke 2.
      error = 4;                                                                                          //output eror.
    }
  }

  if (flight_mode < 3 && waypoint_set > 0) {                                                              //Jika mode penahanan GPS dinonaktifkan dan titik arah ditetapkan.
    gps_roll_adjust = 0;                                                                                  //reset variabel gps_roll_adjust untuk menonaktifkan koreksi.
    gps_pitch_adjust = 0;                                                                                 //Reset variabel gps_pitch_adjust untuk menonaktifkan koreksi.
    if (waypoint_set == 1) {                                                                              //Jika titik arah disimpan
      gps_rotating_mem_location = 0;                                                                      //Atur gps_rotating_mem_location ke nol sehingga kita dapat mengosongkannya
      waypoint_set = 2;                                                                                   //Setel variabel waypoint_set menjadi 2 sebagai indikasi bahwa buffer tidak dihapus.
    }
    gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset lokasi gps_lon_rotating_mem saat ini.
    gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset lokasi gps_lat_rotating_mem saat ini.
    gps_rotating_mem_location++;                                                                          //Tambahkan variabel gps_rotating_mem_location untuk loop berikutnya.
    if (gps_rotating_mem_location == 36) {                                                                //Jika gps_rotating_mem_location sama dengan 36, semua lokasi buffer dihapus.
      waypoint_set = 0;                                                                                   //Setel ulang variabel waypoint_set ke 0.
      //Reset variabel yang digunakan untuk D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
      //Resset titik arah.
      l_lat_waypoint = 0;
      l_lon_waypoint = 0;
    }
  }
}
