///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
TwoWire HWire (2, I2C_FAST_MODE);          //Initiate I2C port 2 at 400kHz.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Pengaturan Gain untuk P-controller pitch and roll (default = 1.3).
float pid_i_gain_roll = 0.04;              //Pengaturan Gain untuk I-controller pitch and roll (default = 0.04). 0.05
float pid_d_gain_roll = 18.0;              //Pengaturan Gain untuk D-Controller Pitch and roll (default = 18.0). 15.0
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

/*
Selama penerbangan,tegangan baterai turun dan  berputar pada RPM yang lebih rendah. ini menyebabkan
efek negatif pada fungsi penahan ketinggian. dengan variable battery_compensation dapat mengkompensasi penurunan tegangan
baterai. tingkatan nilai ini ketika drone turun karena tegangan baterai turun selama penerbangan penahan non-ketinggian
*/
float battery_compensation = 40.0;

float pid_p_gain_altitude = 1.4;           //Pengaturan Gain untuk P-controller altitude (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Pengaturan Gain untuk I-controller altitude (default = 0.2).
float pid_d_gain_altitude = 0.75;          //Pengaturan Gain untuk D-controller altitude (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

float gps_p_gain = 2.7;                    //Pengaturan Gain untuk P-controller GPS (default = 2.7).
float gps_d_gain = 6.5;                    //Pengaturan Gain untuk I-controller GPS (defa22ult = 6.5).

float declination = 0.0;                   //mengatur deklinasi antara magnetik dan utara geografis.

int16_t manual_takeoff_throttle = 0;       //Masukkan titik melayang manual ketika deteksi lepas landas otomatis tidak diinginkan (antara 1400 dan 1600).
int16_t motor_idle_speed = 1100;           //Masukkan pulse minimum gas throttle motor saat idle (antara 1000 dan 1200). 1170 untuk DJI

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
uint8_t MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
uint8_t compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.

float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).

#define STM32_board_LED PC13               //Change PC13 if the LED on the STM32 is connected to another output.


#define variable_1_to_adjust dummy_float   //Ubah dummy_float ke pengaturan apa pun yang ingin di rubah.
#define variable_2_to_adjust dummy_float   //Ubah dummy_float ke pengaturan apa pun yang ingin di rubah.
#define variable_3_to_adjust dummy_float   //Ubah dummy_float ke pengaturan apa pun yang ingin di rubah.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t check_byte, flip32, start;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;
uint8_t channel_select_counter;
uint8_t level_calibration_on;

uint32_t telemetry_buffer_byte;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t channel_1_start, channel_1, channel_1_base, pid_roll_setpoint_base;
int32_t channel_2_start, channel_2, channel_2_base, pid_pitch_setpoint_base;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start, receiver_watchdog;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;

int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];

uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

int32_t acc_alt_integrated;

uint32_t loop_timer, error_timer, flight_mode_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float;

//variable compas
uint8_t compass_calibration_on, heading_lock;
int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;


//Variable Pressure
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
int32_t dT, dT_C5;

//variables Altitude PID
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_average;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

//Variable GPS
uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set, latitude_north, longiude_east ;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location, return_to_home_step;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;

float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
uint8_t home_point_recorded;
int32_t lat_gps_home, lon_gps_home;


//Adjust settings online
uint32_t setting_adjust_timer;
uint16_t setting_click_counter;
uint8_t previous_channel_6;
float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(4, INPUT_ANALOG);                                     //Analog input untuk baterey level
  //Port PB3 dan PB4 digunakan sebagai JTDO dan JNTRST secara default.
  //Fungsi berikut menghubungkan PB3 dan PB4 ke
  //fungsi outout alternatif.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                     //Setting PB3 dan PB4 menjadi output.

  pinMode(PB3, OUTPUT);                                         //Set PB3 sebagai output untuk LED hijau.
  pinMode(PB4, OUTPUT);                                         //Set PB4 sebagai output untuk LED merah.
  pinMode(STM32_board_LED, OUTPUT);                             //LED pada board STM32, digunakan sebagai indikator gps.
  digitalWrite(STM32_board_LED, HIGH);                          //Mematikan LED pada STM32, common vcc.

  green_led(LOW);                                               //Set output PB3 low.
  red_led(HIGH);                                                //Set output PB4 high.

  pinMode(PB0, OUTPUT);                                         //Set PB0 sebagai output telemetry TX.

  //EEPROM emulation setup
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;

  Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  delay(250);                                                 //Give the serial port some time to start to prevent data loss.

  timer_setup();                                                //Timer untuk receiver dan output ECS
  delay(50);                                                    //Give the timers some time to start.

  gps_setup();                                                  //Atur kecepatan baud dan refresh kecepatan keluaran dari modul GPS.

  //Check if the MPU-6050 is responding.
  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not responde.
    error = 1;                                                  //Set the error status to 1.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Check if the compass is responding.
  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(compass_address);                     //Start communication with the HMC5883L.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the HMC5883L did not responde.
    error = 2;                                                  //Set the error status to 2.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Periksa apakah barometer MS5611 merespons.
  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(MS5611_address);                      //Start communication with the MS5611.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MS5611 did not responde.
    error = 3;                                                  //Set the error status to 2.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.
  setup_compass();                                              //Initiallize the compass and set the correct registers.
  read_compass();                                               //Read and calculate the compass data.
  angle_yaw = actual_compass_heading;                           //Set the initial compass heading.

  //Buat penundaan 5 detik sebelum kalibrasi.
  for (count_var = 0; count_var < 1250; count_var++) {          //1250 loop 4 microseconds = 5 detik.
    if (count_var % 125 == 0) {                                 //Setiap 125 loops (500ms).
      digitalWrite(PB4, !digitalRead(PB4));                     //Status Led Berubah.
    }
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set start back to 0.
  calibrate_gyro();                                             //Calibrate the gyro offset.

  //Wait until the receiver is active.
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 4;                                                  //Set the error status to 4.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Delay 4ms to simulate a 250Hz loop
  }
  error = 0;                                                    //Reset the error status to 0.

  //Ketikan semuanya sudah normal. Led merah mati.
  red_led(LOW);                                                 //Set output PB4 low.

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (float)analogRead(4) / 112.81;

  //Untuk menghitung tekanan, 6 nilai kalibrasi perlu disurvei dari MS5611.
  //Nilai 2 byte ini disimpan di lokasi memori 0xA2 keatas
  for (start = 1; start <= 6; start++) {
    HWire.beginTransmission(MS5611_address);                    //Start komunikasi MPU-6050.
    HWire.write(0xA0 + start * 2);                              //Kirim alamat yang akan dibaca datanya.
    HWire.endTransmission();                                    //Stop pengiriman.

    HWire.requestFrom(MS5611_address, 2);                       //Meminta 2 byte data dari MS5611.
    C[start] = HWire.read() << 8 | HWire.read();                //Masukan byte low dan high ke variabel kalibrasi C[x].
  }

  OFF_C2 = C[2] * pow(2, 16);                                   //ini adalah nilai kalkulasi awal untuk di proses di main loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //ini adalah nilai kalkulasi awal untuk di proses di main loop.


  //The MS5611 needs a few readings to stabilize.
  for (start = 0; start < 100; start++) {                       //100 kali looping.
    read_barometer();                                           //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }
  actual_pressure = 0;                                          //Reset the pressure calculations.

  //Sebelum memulai nilai accelerometer rata-rata dimuat ke dalam variabel.
  for (start = 0; start <= 24; start++)acc_z_average_short[start] = acc_z;
  for (start = 0; start <= 49; start++)acc_z_average_long[start] = acc_z;
  acc_z_average_short_total = acc_z * 25;
  acc_z_average_long_total = acc_z * 50;
  start = 0;

  if (motor_idle_speed < 1000)motor_idle_speed = 1000;          //Batas minimum idle speed 1000us.
  if (motor_idle_speed > 1200)motor_idle_speed = 1200;          //Batas maximum idle speed 1200us.

  loop_timer = micros();                                        //timer untuk loop pertama
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if(receiver_watchdog < 750)receiver_watchdog ++;
  if(receiver_watchdog == 750 && start == 2){
    channel_1 = 1500;
    channel_2 = 1500;
    channel_3 = 1500;
    channel_4 = 1500;
    error = 8;
    if (number_used_sats > 5){
      if(home_point_recorded == 1)channel_5 = 2000;
      else channel_5 = 1750;
    }
    else channel_5 = 1500;    
  }
  //Beberapa fungsi hanya dapat diakses ketika quadcopter dimatikan.
  if (start == 0) {
    //Untuk kalibrasi kompas, gerakkan kedua batang ke kanan atas.
    //(channel_1 > 1900 && channel_2 > 1900 && channel_3 > 1900 && channel_4 > 1900)
    if (channel_1 > 1900 && channel_2 < 1100 && channel_3 > 1900 && channel_4 > 1900)calibrate_compass();        //(channel_1 > 1900 && channel_2 < 1100 && channel_3 > 1900 && channel_4 > 1900)
    //Kalibrasi level memindahkan kedua tongkat ke kiri atas.
    //(channel_1 < 1100 && channel_2 > 1900 && channel_3 > 1900 && channel_4 < 1100)
    if (channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100)calibrate_level();          //(channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100)
    //Change settings
    if (channel_6 >= 1900 && previous_channel_6 == 0) {
      previous_channel_6 = 1;
      if (setting_adjust_timer > millis())setting_click_counter ++;
      else setting_click_counter = 0;
      setting_adjust_timer = millis() + 1000;
      if (setting_click_counter > 3) {
        setting_click_counter = 0;
        change_settings();
      }
    }
    if (channel_6 < 1900)previous_channel_6 = 0;
  }

  heading_lock = 0;
  if (channel_6 > 1200)heading_lock = 1;                                           //If channel 6 is between 1200us and 1600us the flight mode is 2

  flight_mode = 1;                                                                 //In all other situations the flight mode is 1;
  if (channel_5 >= 1200 && channel_5 < 1600)flight_mode = 2;                       //If channel 5 is between 1200us and 1600us the flight mode is 2
  if (channel_5 >= 1600 && channel_5 < 1950)flight_mode = 3;                       //If channel 5 is between 1600us and 1900us the flight mode is 3
  if (channel_5 >= 1950 && channel_5 < 2100) {
    if (waypoint_set == 1 && home_point_recorded == 1 && start == 2)flight_mode = 4;
    else flight_mode = 3;
  }

  if (flight_mode <= 3) {
    return_to_home_step = 0;
    return_to_home_lat_factor = 0;
    return_to_home_lon_factor = 0;
  }

//=============================================================================================================================================================
  return_to_home();                                                                //Kembali ke step program return_to_home();
  flight_mode_signal();                                                            //Menampilkan flight_mode dari led hijau.
  error_signal();                                                                  //menampilkan indikator eror dari led merah.
  gyro_signalen();                                                                 //baca data gyro dan accelerometer.
  read_barometer();                                                                //baca dan kalkulasi data barometer.
  read_compass();                                                                  //baca dan kalkulasi data compas.

  if (gps_add_counter >= 0)gps_add_counter --;

  read_gps();

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Input pid Gyro adalah deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Input pid Gyro adalah deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Input pid Gyro adalah deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Hitung sudut pitch yang dilalui dan tambahkan ke variabel angle_pitch.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Hitung sudut roll yang dilalui dan tambahkan ke variabel angle_roll.
  angle_yaw += (float)gyro_yaw * 0.0000611;                                        //Hitung sudut yaw dan tambahkan ini ke variabel angle_yaw.
  if (angle_yaw < 0) angle_yaw += 360;                                             //Jika heading kompas menjadi lebih kecil dari 0, 360 ditambahkan untuk menjaganya dalam rentang 0 hingga 360 derajat.
  else if (angle_yaw >= 360) angle_yaw -= 360;                                     //Jika heading kompas menjadi lebih besar dari 360, 360 dikurangi untuk menjaganya dalam rentang 0 hingga 360 derajat.

  //0,000001066 = 0,0000611 * (3,142 (PI) / 180degr) Fungsi sin di arduino adalah dalam radian dan bukan derajat.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //Jika IMU telah yawed, transfer sudut roll ke sudut pitch.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //Jika IMU telah yawed, transfer sudut pitch ke roll angel.

  angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       //Hitung perbedaan antara gyro dan heading kompas dan buat jarak yg kecil.
  if (angle_yaw < 0) angle_yaw += 360;                                             //Jika heading kompas menjadi lebih kecil dari 0, 360 ditambahkan untuk menjaganya dalam rentang 0 hingga 360 derajat.
  else if (angle_yaw >= 360) angle_yaw -= 360;                                     //Jika heading kompas menjadi lebih besar dari 360, 360 dikurangi untuk menjaganya dalam rentang 0 hingga 360 derajat.


  //kalkulasi sudut accelerometer
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //kalkulasi nilai total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Cegah fungsi asin untuk menghasilkan NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //kalkulasi sudut pitch.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Cegah fungsi asin untuk menghasilkan NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //kalkulasi sudut roll.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

  vertical_acceleration_calculations();                                            //Calculate the vertical accelration.

  channel_1_base = channel_1;                                                      //Normally channel_1 is the pid_roll_setpoint input.
  channel_2_base = channel_2;                                                      //Normally channel_2 is the pid_pitch_setpoint input.
  gps_man_adjust_heading = angle_yaw;                                              //
  //Ketika mode heading_lock diaktifkan, setpoint roll dan pitch pid tergantung pada pos.
  //At startup the heading is registerd in the variable course_lock_heading.
  //penyimpangan arah adalah kalkulasi antara heading saat ini dan course_lock_heading.
  //Berdasarkan penyimpangan ini kontrol pitch and roll dihitung sehingga responsnya sama seperti pada startup.
  if (heading_lock == 1) {
    heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
    channel_1_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    channel_2_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
    gps_man_adjust_heading = course_lock_heading;

  }
  if (flight_mode >= 3 && waypoint_set == 1) {
    pid_roll_setpoint_base = 1500 + gps_roll_adjust;
    pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
  }
  else {
    pid_roll_setpoint_base = channel_1_base;
    pid_pitch_setpoint_base = channel_2_base;
  }

  //Karena kami menambahkan nilai penyesuaian GPS, kami perlu memastikan bahwa batas kontrol tidak melebihi.
  if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
  if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
  if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
  if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;

  calculate_pid();                                                                 //mengkalkulasi pid berdasarkan input receiver

  start_stop_takeoff();                                                            //Starting, stopping and take-off detection

  //Tegangan baterai diperlukan untuk kompensasi.
  //Filter komplementer digunakan untuk mengurangi noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);

  //Led akan menyala jika voltase baterai rendah. Pengaturan standar adalah 10.5V
  if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)error = 1;


  //Variabel base_throttle dihitung di bagian berikut. Ini membentuk throttle dasar untuk setiap motor.
  if (takeoff_detected == 1 && start == 2) {                                         //Jika quadcopter start dan terbang.
    throttle = channel_3 + takeoff_throttle;                                         //Throttle dasar adalah dari channel throttle + throttle take-off yang terdeteksi.
    if (flight_mode >= 2) {                                                          //Jika mode altitude aktif.
      throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;    //Throttle dasar adalah receiver channel throttle + throttle take-off yang terdeteksi + output pengontrol PID.
    }
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if (start == 2) {                                                                //Motor start.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Kalkulasi pulse untuk esc_1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Kalkulasi pulse untuk esc_2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Kalkulasi pulse untuk esc_3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Kalkulasi pulse untuk esc_4 (front-left - CW).

    if (battery_voltage < 12.40 && battery_voltage > 6.0) {                        //Is the battery connected?
      esc_1 += (12.40 - battery_voltage) * battery_compensation;                   //kompensasi pulasa esc_1 pada saat battrey drop
      esc_2 += (12.40 - battery_voltage) * battery_compensation;                   //kompensasi pulasa esc_2 pada saat battrey drop
      esc_3 += (12.40 - battery_voltage) * battery_compensation;                   //kompensasi pulasa esc_3 pada saat battrey drop
      esc_4 += (12.40 - battery_voltage) * battery_compensation;                   //kompensasi pulasa esc_4 pada saat battrey drop
    }

    if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;                        //Menjaga kecepatan motor stabil pada kondisi idle speed.
    if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;                        //Menjaga kecepatan motor stabil pada kondisi idle speed.
    if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;                        //Menjaga kecepatan motor stabil pada kondisi idle speed.
    if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;                        //Menjaga kecepatan motor stabil pada kondisi idle speed.

    if (esc_1 > 2000)esc_1 = 2000;                                                 //menjaga agar kecepatan maksimal motor tidak lebih dari 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                                 //menjaga agar kecepatan maksimal motor tidak lebih dari 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                                 //menjaga agar kecepatan maksimal motor tidak lebih dari 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                                 //menjaga agar kecepatan maksimal motor tidak lebih dari 2000us.
  }

  else {
    esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
  }


  TIMER4_BASE->CCR1 = esc_1;                                                       //Setel pulsa input receiver throttle ke pulsa output ESC 1.
  TIMER4_BASE->CCR2 = esc_2;                                                       //Setel pulsa input receiver throttle ke pulsa output ESC 2.
  TIMER4_BASE->CCR3 = esc_3;                                                       //Setel pulsa input receiver throttle ke pulsa output ESC 3.
  TIMER4_BASE->CCR4 = esc_4;                                                       //Setel pulsa input receiver throttle ke pulsa output ESC 4.
  TIMER4_BASE->CNT = 5000;                                                         //mengatur ulang timer 4 dan pulsa ESC dibuat secara langsung.

  send_telemetry_data();                                                           //kirim data telemetry ke LCD.

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Karena perhitungan sudut waktu loop menjadi sangat penting. Jika waktu loop
  //lebih panjang atau lebih pendek dari 4000us calculation angle dimatikan. Jika Anda memodifikasi kode, pastikan
  //bahwa waktu loop masih 4000us dan tidak lebih!
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050)error = 2;                                      //Keluaran jika waktu loop melebihi 4050us
  while (micros() - loop_timer < 4000);                                            //menunggu waktu loop hingga 4000us.
  loop_timer = micros();                                                           //mengatur timer untuk loop selanjutnya.
}
