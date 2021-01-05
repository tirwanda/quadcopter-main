void read_barometer(void) {
  barometer_counter ++;

  /*
    Setiap kali fungsi ini disebut variabel barometer_counter bertambah. Dengan cara ini tindakan tertentu dijalankan pada saat yang tepat.
    Ini diperlukan karena meminta data dari MS5611 membutuhkan waktu sekitar 9ms untuk menyelesaikannya. 
  */

  if (barometer_counter == 1) {                                                 //ketika barometer_counter = 1.
    if (temperature_counter == 0) {                                             //dan temperature counter = 0.
      
      //mengambil data temperature dari MS-5611
      HWire.beginTransmission(MS5611_address);                                  //memulai penyaluran data dengan MS5611
      HWire.write(0x00);                                                        //kirim 0 untuk meminta data.
      HWire.endTransmission();                                                  //stop penyaluran data dari MS5611.
      HWire.requestFrom(MS5611_address, 3);                                     //mengambil 3byte data dari MS5611.
      
      // Simpan suhu di 5 lokasi memori berputar untuk mencegah lonjakan suhu.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Hitung suhu rata-rata dari 5 pengukuran terakhir.
    }
    else {
      //mengambil data tekanan dari MS-5611
      HWire.beginTransmission(MS5611_address);                                  //memulai penyaluran data dengan MS5611
      HWire.write(0x00);                                                        //kirim 0 untuk meminta data.
      HWire.endTransmission();                                                  //stop penyaluran data dari MS5611.
      HWire.requestFrom(MS5611_address, 3);                                     //mengambil 3byte data dari MS5611.
      raw_pressure = HWire.read() << 16 | HWire.read() << 8 | HWire.read();     //Geser masing-masing byte di posisi yang benar dan tambahkan mereka ke variabel raw_pressure.
    }

    temperature_counter ++;                                                     
    if (temperature_counter == 20) {                                            //ketika temperature counter = 20.
      temperature_counter = 0;                                                  //Reset temperature_counter variable.
      //mengambil data temperature
      HWire.beginTransmission(MS5611_address);                                  //memulai penyaluran data dengan MS5611
      HWire.write(0x58);                                                        //kirim 0x58 untuk meminta data temperature
      HWire.endTransmission();                                                  //stop penyaluran data dari MS5611.
    }
    else {                                                                      
      //mengambil data pressure
      HWire.beginTransmission(MS5611_address);                                  //memulai penyaluran data dengan MS5611
      HWire.write(0x48);                                                        //kirim 0x48 untuk meminta data tekanan.
      HWire.endTransmission();                                                  //stop penyaluran data dari MS5611.
    }
  }
  
  if (barometer_counter == 2) {                                                
    //Hitung tekanan seperti yang dijelaskan dalam datasheet MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    
    //Untuk mendapatkan nilai tekanan yang lebih halus, kami akan menggunakan memori putar 20 lokasi.
    pressure_total_average -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Kurangi posisi memori saat ini untuk memberikan ruang bagi nilai baru.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Hitung perubahan baru antara tekanan aktual dan pengukuran sebelumnya.
    pressure_total_average += pressure_rotating_mem[pressure_rotating_mem_location];                          //Tambahkan nilai baru ke nilai rata-rata jangka panjang.
    pressure_rotating_mem_location++;                                                                         //Tambah lokasi memori yang berputar.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Mulai dari 0 ketika lokasi memori 20 tercapai.
    actual_pressure_fast = (float)pressure_total_average / 20.0;                                              //Hitung tekanan rata-rata dari 20 pembacaan tekanan terakhir.

    //Untuk mendapatkan hasil yang lebih baik, kami akan menggunakan filter komplementer yang dapat disesuaikan dengan fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Hitung perbedaan antara nilai rata-rata cepat dan lambat.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //Jika selisihnya lebih besar maka 8 batasi selisihnya menjadi 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //Jika selisihnya lebih kecil maka -8 batasi selisihnya ke -8.
    //Jika perbedaannya lebih besar dari 1 atau lebih kecil dari -1 rata-rata lambat disesuaikan berdasarkan kesalahan antara rata-rata cepat dan lambat.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //Tekanan aktual digunakan dalam program untuk perhitungan ketinggian.
  }

  if (barometer_counter == 3) {                                                                               //ketika barometer counter = 3

    barometer_counter = 0;                                                                                    //reset barometer counter untuk perhitungan selanjutnya
    //Pada bagian berikut, buffer berputar digunakan untuk menghitung perubahan jangka panjang antara berbagai pengukuran tekanan.
    //Nilai total ini dapat digunakan untuk mendeteksi arah (atas / bawah) dan kecepatan quadcopter dan berfungsi sebagai D-controller dari total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //Selama perubahan ketinggian manual, deteksi atas / bawah dinonaktifkan.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Kurangi posisi memori saat ini untuk memberikan ruang bagi nilai baru.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Hitung perubahan baru antara tekanan aktual dan pengukuran sebelumnya.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Tambahkan nilai baru ke nilai rata-rata jangka panjang.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Simpan pengukuran saat ini untuk loop berikutnya.
    parachute_rotating_mem_location++;                                                                        //Tambah lokasi memori yang berputar.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Mulai dari 0 ketika lokasi memori 30 tercapai.

    if (flight_mode >= 2 && takeoff_detected == 1) {                                                          //Jika quadcopter dalam mode ketinggian dan terbang.
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //Jika belum diatur, setel setpoint ketinggian PID.
      //Ketika posisi throttle stick dinaikkan atau diturunkan, fungsi penahan ketinggian dinonaktifkan sebagian. Variabel manual_altitude_change
      //akan menunjukkan apakah ketinggian quadcopter diubah oleh pilot.
      manual_altitude_change = 0;                                                    //Reset variable manual_altitude_change = 0.
      manual_throttle = 0;                                                           //Set variable manual_throttle = 0.
      if (channel_3 > 1600) {                                                        //Jika throttle dinaikkan di atas 1600 us (60%).
        manual_altitude_change = 1;                                                  //Atur variabel manual_altitude_change ke 1 untuk menunjukkan bahwa ketinggian disesuaikan.
        pid_altitude_setpoint = actual_pressure;                                     //Sesuaikan setpoint ke nilai tekanan aktual sehingga output dari kontroler P- dan I adalah 0.
        manual_throttle = (channel_3 - 1600) / 3;                                    //Untuk mencegah perubahan ketinggian yang sangat cepat, batasi fungsi throttle.
      }
      if (channel_3 < 1400) {                                                        //Jika throttle diturunkan di bawah 1400us (40%).
        manual_altitude_change = 1;                                                  //Atur variabel manual_altitude_change ke 1 untuk menunjukkan bahwa ketinggian disesuaikan.
        pid_altitude_setpoint = actual_pressure;                                     //Sesuaikan setpoint ke nilai tekanan aktual sehingga output dari kontroler P- dan I adalah 0.
        manual_throttle = (channel_3 - 1400) / 5;                                    //Untuk mencegah perubahan ketinggian yang sangat cepat, batasi fungsi throttle.
      }

      //Hitung output PID dari penahan ketinggian.
      pid_altitude_input = actual_pressure;                                          //Set setpoint (pid_altitude_input) dari PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Hitung kesalahan antara setpoint dan nilai tekanan aktual.

      //Untuk mendapatkan hasil yang lebih baik, P-gain meningkat ketika kesalahan antara setpoint dan nilai tekanan aktual meningkat.
      //Variabel pid_error_gain_altitude akan digunakan untuk menyesuaikan P-gain dari kontroler-PID.
      pid_error_gain_altitude = 0;                                                   //Set pid_error_gain_altitude = 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //Jika eror antara setpoint dan tekanan aktual lebih besar dari 10 atau lebih kecil dari -10.
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //Variabel pid_error_gain_altitude dihitung berdasarkan kesalahan.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //untuk mencegah kenaikan extrim P-gain, itu harus dibatasi hingga 3.
      }

      //Pada bagian berikut, output-I dihitung. Ini akumulasi eror seiring waktu.
      //Faktor waktu dihapus ketika loop program berjalan pada 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //Pada baris berikut PID-output dihitung.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //Untuk mencegah output PID ekstrim, output harus dibatasi.
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    }

    //Jika fungsi penahan ketinggian dinonaktifkan, beberapa variabel perlu diatur ulang untuk memastikan dimulainya bumpless ketika fungsi penahan ketinggian diaktifkan kembali.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //Jika mode altitude hold tidak diatur dan setpoint ketinggian PID masih diatur.
      pid_altitude_setpoint = 0;                                                     
      pid_output_altitude = 0;                                                       
      pid_i_mem_altitude = 0;                                                        
      manual_throttle = 0;                                                           
      manual_altitude_change = 1;                                                    
    }
  }
}
