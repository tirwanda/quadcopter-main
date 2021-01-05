///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dalam file ini, penghitung waktu untuk membaca pulsa receiver dan untuk membuat pulsa ESC keluaran diatur.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//More information can be found in these two videos:
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void handler_channel_1(void) {
  measured_time = TIMER2_BASE->CCR1 - measured_time_start;
  if (measured_time < 0)measured_time += 0xFFFF;
  measured_time_start = TIMER2_BASE->CCR1;
  if (measured_time > 3000) {
    channel_select_counter = 0;
    receiver_watchdog = 0;
    if (error == 8 && start == 2)error = 0;
  }
  else channel_select_counter++;

  if (channel_select_counter == 1)channel_1 = measured_time;
  if (channel_select_counter == 2)channel_2 = measured_time;
  if (channel_select_counter == 3)channel_3 = measured_time;
  if (channel_select_counter == 4)channel_4 = measured_time;
  if (channel_select_counter == 5)channel_5 = measured_time;
  if (channel_select_counter == 6)channel_6 = measured_time;
}