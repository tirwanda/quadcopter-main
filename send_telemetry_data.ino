///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Bagian ini mengirimkan data telemetri ke remote.
//Output untuk monitor serial adalah PB0. Protokol adalah 1 bit awal, 8 bit data, no parity, 1 stop bit.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void send_telemetry_data(void) {
  telemetry_loop_counter++;                                                                 //Tambahkan variabel telemetry_loop_counter.
  if (telemetry_loop_counter == 1)telemetry_send_byte = 'J';                                //kirim data J sebagai tanda mulai.
  if (telemetry_loop_counter == 2)telemetry_send_byte = 'B';                                //Kirim data B sebagai tanda mulai.
  if (telemetry_loop_counter == 3) {
    check_byte = 0;
    telemetry_send_byte = error;                              //Kirim error.
  }
  if (telemetry_loop_counter == 4)telemetry_send_byte = flight_mode + return_to_home_step;                        //Kirim mode penerbangan sebagai byte.
  if (telemetry_loop_counter == 5)telemetry_send_byte = battery_voltage * 10;               //Kirim tegangan baterai sebagai byte.
  if (telemetry_loop_counter == 6) {
    telemetry_buffer_byte = temperature;                                                    //Simpan suhu karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari variabel suhu.
  }
  if (telemetry_loop_counter == 7)telemetry_send_byte = telemetry_buffer_byte >> 8;         //Kirim 8 byte terakhir dari variabel suhu.
  if (telemetry_loop_counter == 8)telemetry_send_byte = angle_roll + 100;                   //Kirim sudut roll sebagai byte. Tambahkan 100 untuk mencegah angka negatif.
  if (telemetry_loop_counter == 9)telemetry_send_byte = angle_pitch + 100;                  //Kirim sudut pitch sebagai byte. Tambahkan 100 untuk mencegah angka negatif.
  if (telemetry_loop_counter == 10)telemetry_send_byte = start;                             //Kirim eror sebagai byte.
  if (telemetry_loop_counter == 11) {
    if (start == 2) {                                                                       //Hanya kirim ketinggian ketika quadcopter terbang.
      telemetry_buffer_byte = 1000 + ((float)(ground_pressure - actual_pressure) * 0.117);  //Hitung ketinggian dan tambahkan 1000 untuk mencegah angka negatif.
    }
    else {
      telemetry_buffer_byte = 1000;                                                         //Kirim dan ketinggian 0 meter jika quadcopter tidak terbang.
    }
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari variabel ketinggian.
  }
  if (telemetry_loop_counter == 12)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte terakhir dari variabel ketinggian.

  if (telemetry_loop_counter == 13) {
    telemetry_buffer_byte = 1500 + takeoff_throttle;                                        //Simpan throttle lepas landas karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari variabel throttle take-off.
  }
  if (telemetry_loop_counter == 14)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte terakhir dari variabel throttle take-off.
  if (telemetry_loop_counter == 15) {
    telemetry_buffer_byte = angle_yaw;                                                      //Simpan heading kompas karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari variabel heading kompas.
  }
  if (telemetry_loop_counter == 16)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte terakhir dari variabel heading kompas.
  if (telemetry_loop_counter == 17)telemetry_send_byte = heading_lock;                      //Kirim variabel heading_lock sebagai byte.
  if (telemetry_loop_counter == 18)telemetry_send_byte = number_used_sats;                  //Kirim variabel number_used_sats sebagai byte.
  if (telemetry_loop_counter == 19)telemetry_send_byte = fix_type;                          //Kirim variabel fix_type sebagai byte.
  if (telemetry_loop_counter == 20) {
    telemetry_buffer_byte = l_lat_gps;                                                      //Simpan posisi lintang karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari variabel posisi lintang.
  }
  if (telemetry_loop_counter == 21)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte berikutnya dari variabel posisi lintang.
  if (telemetry_loop_counter == 22)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Kirim 8 byte berikutnya dari variabel posisi lintang.
  if (telemetry_loop_counter == 23)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Kirim 8 byte berikutnya dari variabel posisi lintang.
  if (telemetry_loop_counter == 24) {
    telemetry_buffer_byte = l_lon_gps;                                                      //Simpan posisi bujur karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari variabel posisi bujur.
  }
  if (telemetry_loop_counter == 25)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte berikutnya dari variabel posisi bujur.
  if (telemetry_loop_counter == 26)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Kirim 8 byte berikutnya dari variabel posisi bujur.
  if (telemetry_loop_counter == 27)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Kirim 8 byte berikutnya dari variabel posisi bujur.

  if (telemetry_loop_counter == 28) {
    telemetry_buffer_byte = adjustable_setting_1 * 100;                                     //Simpan adjustable_setting_1 karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari adjustable_setting_1.
  }
  if (telemetry_loop_counter == 29)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte berikutnya dari adjustable_setting_1.
  if (telemetry_loop_counter == 30) {
    telemetry_buffer_byte = adjustable_setting_2 * 100;                                     //Simpan adjustable_setting_2 karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari adjustable_setting_2.
  }
  if (telemetry_loop_counter == 31)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte berikutnya dari adjustable_setting_2.
  if (telemetry_loop_counter == 32) {
    telemetry_buffer_byte = adjustable_setting_3 * 100;                                     //Simpan adjustable_setting_3. karena dapat berubah selama loop berikutnya.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Kirim 8 byte pertama dari adjustable_setting_3.
  }
  if (telemetry_loop_counter == 33)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Kirim 8 byte berikutnya dari adjustable_setting_3.


  if (telemetry_loop_counter == 34)telemetry_send_byte = check_byte;                        //Kirim byte-cek.


  //Setelah 125 loop, variabel telemetry_loop_counter direset. Dengan cara ini, data telemetri dikirimkan setiap setengah detik.
  if (telemetry_loop_counter == 125)telemetry_loop_counter = 0;                             //Setelah 125 loop mereset variabel telemetry_loop_counter
  
  //Kirim telemetri kirim byte melalui protokol serial melalui output PB0.
  //Kirim bit awal terlebih dahulu.
  if (telemetry_loop_counter <= 34) {
    check_byte ^= telemetry_send_byte;
    GPIOB_BASE->BSRR = 0b1 << 16;                                                             //Atur ulang output PB0 ke 0 untuk membuat bit mulai.
    delayMicroseconds(104);                                                                   //Delay 104us (1s/9600bps)
    for (telemetry_bit_counter = 0; telemetry_bit_counter < 8; telemetry_bit_counter ++) {    //Buat loop untuk setiap bit di telemetry_bit_counter
      if (telemetry_send_byte >> telemetry_bit_counter & 0b1) GPIOB_BASE->BSRR = 0b1 << 0;    //Jika bit tertentu diatur, atur output PB0 ke 1
      else GPIOB_BASE->BSRR = 0b1 << 16;                                                      //Jika bit tertentu tidak diatur, reset output PB0 ke 0;
      delayMicroseconds(104);                                                                 //Delay 104us (1s/9600bps)
    }
    //Send a stop bit
    GPIOB_BASE->BSRR = 0b1 << 0;                                                              //Set output PB0 ke 1;
  }
}
