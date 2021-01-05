///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Take off dan Stop di atur.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_stop_takeoff(void) {
  if (channel_3 < 1050 && channel_4 < 1050)start = 1;                              //Untuk memulai motor: throttle low dan yaw left (step 1).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {                        //Ketika tongkat yaw kembali ke posisi tengah, mulailah motor (langkah 2).
    throttle = motor_idle_speed;                                                   //Atur throttle dasar ke variabel motor_idle_speed.
    angle_pitch = angle_pitch_acc;                                                 //Atur sudut pitch gyro sama dengan sudut pitch accelerometer ketika quadcopter dimulai.
    angle_roll = angle_roll_acc;                                                   //Atur sudut gyro roll sama dengan sudut roll accelerometer saat quadcopter dimulai.
    ground_pressure = actual_pressure;                                             //masukan tekanan di permukaan tanah untuk perhitungan ketinggian.
    course_lock_heading = angle_yaw;                                               //Atur arah kompas saat ini sebagai judul kunci kursus.
    acc_total_vector_at_start = acc_total_vector;                                  //Daftarkan akselerasi saat quadcopter dimulai.
    if(number_used_sats >= 5){
      lat_gps_home = l_lat_gps;
      lon_gps_home = l_lon_gps;
      home_point_recorded = 1;
    }
    else home_point_recorded = 0;
    start = 2;                                                                     //Set variabel start menjadi 2 untuk menunjukkan bahwa quadcopter dimulai.
    acc_alt_integrated = 0;                                                        //Reset nilai acc_alt_integrated.
    if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600) {        //Jika manual throttle terbang digunakan dan valid (antara 1400us dan 1600us pulsa).
      takeoff_throttle = manual_takeoff_throttle - 1500;                           //Gunakan throttle terbang manual.
      takeoff_detected = 1;                                                        //Set takeoff_detected ke 1, yang menunjukkan bahwa quadcopter sedang terbang.
      //Setel ulang pengontrol PID untuk lepas landas dengan lancar.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;
    }
    else if (manual_takeoff_throttle) {                                            //Jika nilai manual_takeoff_throttle tidak valid.
      error = 5;                                                                   //Error = 5.
      takeoff_throttle = 0;                                                        //Tidak ada nilai throttle takeoff.
      start = 0;                                                                   //Atur variabel start ke 0 untuk menghentikan motor.
    }
  }
  //Menghentikan motor: throttle low dan yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;                                                                     //Set variabel start ke 0 untuk menonaktifkan motor.
    takeoff_detected = 0;                                                          //reset takeoff_detected otomatis.
  }

  if (takeoff_detected == 0 && start == 2) {                                       //Ketika quadcopter dimulai dan tidak ada take-off terdeteksi.
    if (channel_3 > 1480 && throttle < 1750) throttle++;                           //Saat throttle setengah atau lebih tinggi, naikkan throttle.
    if (throttle == 1750)error = 6;                                                //Jika take-off tidak terdeteksi ketika throttle telah mencapai 1700: error = 6.
    if (channel_3 <= 1480) {                                                       //When the throttle is below the center stick position.
      if (throttle > motor_idle_speed)throttle--;                                  //Turunkan throttle ke variabel motor_idle_speed.
      //Reset pengontrol PID untuk take off dengan lancar.
      else {                                                                       //Saat throttle kembali dengan kecepatan idle, reset pengontrol PID.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;
      }
    }
    if (acc_z_average_short_total / 25 - acc_total_vector_at_start > 800) {        //Take-off terdeteksi saat quadcopter berakselerasi.
      takeoff_detected = 1;                                                        //Set variabel takeoff_detected ke 1 untuk menunjukkan take-off.
      pid_altitude_setpoint = ground_pressure - 15;                                //Set setpoint ketinggian di groundlevel + sekitar 2,2 meter (22).
      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - 1530;  //Jika throttle otomatis antara 1400 dan 1600us selama take-off, hitung throttle take-off.
      else {                                                                       //If the automated throttle is not between 1400 and 1600us during take-off.
        takeoff_throttle = 0;                                                      //Tidak ada throttle lepas landas yang dihitung.
        error = 7;                                                                 //Tampilkan error = 7, pada LED Red.
      }
    }
  }
}
