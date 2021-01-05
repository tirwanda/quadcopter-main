///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Fungsi-fungsi ini menangani LED merah dan hijau. LED pada flip 32 terbalik. Itulah mengapa tes Flip32 diperlukan.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void red_led(int8_t level) {
  if (flip32)digitalWrite(PB4, !level);    //Jika Flip32 digunakan, balikkan hasilnya.
  else digitalWrite(PB4, level);           //Saat menggunakan BluePill, output tidak boleh dibalik.
}
void green_led(int8_t level) {
  if (flip32)digitalWrite(PB3, !level);    //Jika Flip32 digunakan, balikkan hasilnya.
  else digitalWrite(PB3, level);           //Saat menggunakan BluePill, output tidak boleh dibalik.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Di bagian ini sinyal eror LED dihasilkan.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void) {
  if (error >= 100) red_led(HIGH);                                                         //Ketika error = 100, LED selalu menyala.
  else if (error_timer < millis()) {                                                       //Jika nilai error_timer lebih kecil dari fungsi millis ().
    error_timer = millis() + 250;                                                          //Tetapkan interval error_timer berikutnya pada 250ms.
    if (error > 0 && error_counter > error + 3) error_counter = 0;                         //Jika ada kesalahan untuk dilaporkan dan error_counter > error +3 reset kesalahan.
    if (error_counter < error && error_led == 0 && error > 0) {                            //Jika urutan kesalahan flash tidak selesai (error_counter <error) dan LED mati.
      red_led(HIGH);                                                                       //LED on.
      error_led = 1;                                                                       //Atur bendera LED untuk menunjukkan bahwa LED menyala.
    }
    else {                                                                                 
      red_led(LOW);                                                                       
      error_counter++;                                                                     
      error_led = 0;                                                                       
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Pada bagian ini sinyal LED mode flight dihasilkan.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flight_mode_signal(void) {
  if (flight_mode_timer < millis()) {                                                      //Jika nilai error_timer lebih kecil dari fungsi millis ().
    flight_mode_timer = millis() + 250;                                                    //Tetapkan interval error_timer berikutnya pada 250ms.
    if (flight_mode > 0 && flight_mode_counter > flight_mode + 3) flight_mode_counter = 0; //Jika ada kesalahan untuk dilaporkan dan error_counter> error +3 reset kesalahan.
    if (flight_mode_counter < flight_mode && flight_mode_led == 0 && flight_mode > 0) {    //Jika urutan kesalahan flash tidak selesai (error_counter <error) dan LED mati.
      green_led(HIGH);                                                                     //LED on.
      flight_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 
      green_led(LOW);                                                                      
      flight_mode_counter++;                                                               
      flight_mode_led = 0;                                                                 
    }
  }
}
