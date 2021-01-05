///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Ini adalah algoritma program kembali ke rumah
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void return_to_home(void) {

  if (flight_mode == 4) {
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step 0 - membuat beberapa perhitungan dasar
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 0) {
      //Apakah quadcopter dekat? Kemudian mendarat tanpa kembali ke rumah.
      if (abs(lat_gps_home - l_lat_waypoint) < 90 && abs(lon_gps_home - l_lon_waypoint) < 90)return_to_home_step = 3;
      else {
        return_to_home_move_factor = 0.0;
        if (return_to_home_lat_factor == 1 || return_to_home_lon_factor == 1)return_to_home_step = 1;
        //cos(((float)l_lat_gps / 1000000.0)
        if (abs(lat_gps_home - l_lat_waypoint) >= abs(lon_gps_home - l_lon_waypoint)) {
          return_to_home_lon_factor = (float)abs(lon_gps_home - l_lon_waypoint) / (float)abs(lat_gps_home - l_lat_waypoint);
          return_to_home_lat_factor = 1;
        }
        else {
          return_to_home_lon_factor = 1;
          return_to_home_lat_factor = (float)abs(lat_gps_home - l_lat_waypoint) / (float)abs(lon_gps_home - l_lon_waypoint);
        }

        if (ground_pressure - actual_pressure < 170)return_to_home_decrease = 170 - (ground_pressure - actual_pressure);
        else return_to_home_decrease = 0;
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step - 1 tingkatkan ketinggian hingga 20 meter di atas permukaan tanah
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 1) {
      if (return_to_home_decrease <= 0)return_to_home_step = 2;
      if (return_to_home_decrease > 0) {
        pid_altitude_setpoint -= 0.035;
        return_to_home_decrease -= 0.035;
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step 2 - Kembali ke posisi asal
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 2) {
      if (lat_gps_home == l_lat_waypoint && lon_gps_home == l_lon_waypoint)return_to_home_step = 3;
      if (abs(lat_gps_home - l_lat_waypoint) < 160 && abs(lon_gps_home - l_lon_waypoint) < 160 && return_to_home_move_factor > 0.05)return_to_home_move_factor -= 0.00015;
      else if (return_to_home_move_factor < 0.20)return_to_home_move_factor += 0.0001;

      if (lat_gps_home != l_lat_waypoint) {
        if (lat_gps_home > l_lat_waypoint) l_lat_gps_float_adjust += return_to_home_move_factor * return_to_home_lat_factor;
        if (lat_gps_home < l_lat_waypoint) l_lat_gps_float_adjust -= return_to_home_move_factor * return_to_home_lat_factor;
      }
      if (lon_gps_home != l_lon_waypoint) {
        if (lon_gps_home > l_lon_waypoint) l_lon_gps_float_adjust += return_to_home_move_factor * return_to_home_lon_factor;
        if (lon_gps_home < l_lon_waypoint) l_lon_gps_float_adjust -= return_to_home_move_factor * return_to_home_lon_factor;
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step - 3 kurangi ketinggian dengan meningkatkan setpoint tekanan
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 3) {
      if (pid_altitude_setpoint > actual_pressure + 150)return_to_home_step = 4;
      pid_altitude_setpoint += 0.035;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step - 4 Stop motors
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 4) {
      start = 0;
      return_to_home_step = 5;
    }

  }
}
