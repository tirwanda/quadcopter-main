///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subrutin untuk menghitung keluaran pid
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(void) {

  //set point PID dalam derajat per detik ditentukan oleh input receiver roll.
  //Dalam hal membaginya dengan 3 laju gulungan maks adalah sekitar 164 derajat per detik ((500-8) / 3 = 164d / dtk).
  pid_roll_setpoint = 0;
  //Kami membutuhkan dead band kecil berukuran 16us untuk hasil yang lebih baik.
  if (pid_roll_setpoint_base > 1508)pid_roll_setpoint = pid_roll_setpoint_base - 1508;
  else if (pid_roll_setpoint_base < 1492)pid_roll_setpoint = pid_roll_setpoint_base - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                          //Kurangi koreksi sudut dari nilai roll input penerima standar.
  pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //Set point PID dalam derajat per detik ditentukan oleh input penerima pitch.
  //Dalam hal membagi dengan 3 tingkat pitch maksimum sekitar 164 derajat per detik ((500-8) / 3 = 164d / s).
  pid_pitch_setpoint = 0;
  //Kami membutuhkan dead band kecil berukuran 16us untuk hasil yang lebih baik.
  if (pid_pitch_setpoint_base > 1508)pid_pitch_setpoint = pid_pitch_setpoint_base - 1508;
  else if (pid_pitch_setpoint_base < 1492)pid_pitch_setpoint = pid_pitch_setpoint_base - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                        //Kurangi koreksi sudut dari nilai input pitch penerima standar.
  pid_pitch_setpoint /= 3.0;                                                       //Bagi setpoint untuk pengontrol pitch PID dengan 3 untuk mendapatkan sudut dalam derajat.

  //Set point PID dalam derajat per detik ditentukan oleh input penerima yaw.
  //Dalam hal membagi dengan 3 tingkat max yaw adalah sekitar 164 derajat per detik ((500-8) / 3 = 164d / s).
  pid_yaw_setpoint = 0;
  //Kami membutuhkan dead band kecil berukuran 16us untuk hasil yang lebih baik.
  if (channel_3 > 1050) { //Jangan yaw saat mematikan motor.
    if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  //Perhitungan roll
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Perhitungan Pitch
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Perhitungan Yaw
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}
