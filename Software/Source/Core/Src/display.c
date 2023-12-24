#include "display.h"

const unsigned char sat_icon[] = {
  0x00,0x01,0x00, 0x80,0x07,0x00, 0xc0,0x06,0x00, 0x60,0x30,0x00, 0x60,0x78,0x00,
  0xc0,0xfc,0x00, 0x00,0xfe,0x01, 0x00,0xff,0x01, 0x80,0xff,0x00, 0xc0,0x7f,0x06,
  0xc0,0x3f,0x06, 0x80,0x1f,0x0c, 0x80,0x4f,0x06, 0x19,0xc6,0x03, 0x1b,0x80,0x01,
  0x73,0x00,0x00, 0x66,0x00,0x00, 0x0e,0x00,0x00, 0x3c,0x00,0x00, 0x70,0x00,0x00
};

const unsigned char sat_icon_sig[3][7] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x18, 0x18, 0x70, 0x60, 0x00, 0x00, 0x00 },
  { 0x19, 0x1b, 0x73, 0x66, 0x0e, 0x3c, 0x70 }
};

const unsigned char heater_icon[] = {
  0xfe,0xff,0x07, 0x03,0x00,0x0c, 0x61,0x8c,0x09, 0x71,0xce,0x09, 0x31,0xc6,0x08,
  0x19,0x63,0x08, 0x19,0x63,0x08, 0x19,0x63,0x08, 0x19,0x63,0x08, 0x31,0xc6,0x08,
  0x71,0xce,0x09, 0x61,0x8c,0x09, 0x61,0x8c,0x09, 0x61,0x8c,0x09, 0x61,0x8c,0x09,
  0x71,0xce,0x09, 0x39,0xe7,0x08, 0x19,0x63,0x08, 0x03,0x00,0x0c, 0xfe,0xff,0x07
};

void display_sat_icon_sig(uint8_t x, uint8_t y, int32_t pat) {
  display_XBM(x, y+13, sat_icon_sig[pat], 7, 7, White);
}

void display_Latitude(void) {
  char str[32];
  float f = atof((const char *) GGA_Latitude);
  int i = f;
  int d = i/100;
  int m = i%100;
  int s = (f - i) * 60;
  sprintf(str,"LAT: %1s %3d,%2d,%2d", GGA_NSindicator, d, m, s);
  ssd1306_SetCursor(TPOS_LAT_X, TPOS_LAT_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

void display_Longitude(void) {
  char str[32];
  float f = atof((const char *) GGA_Longitude);
  int i = f;
  int d = i/100;
  int m = i%100;
  int s = (f - i) * 60;
  sprintf(str,"LON: %1s %3d,%2d,%2d", GGA_EWindicator, d, m, s);
  ssd1306_SetCursor(TPOS_LON_X, TPOS_LON_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

void draw_hline(uint8_t x, uint8_t y, uint8_t w) {
	for (int i=0; i < w; i++) {
		ssd1306_DrawPixel(x++, y, White);
	}
}

void display_heater_icon(uint8_t x, uint8_t y) {
  display_XBM(x, y, heater_icon, 20, 20, White);
}

void display_satellite_icon(uint8_t x, uint8_t y) {
  display_XBM(x, y, sat_icon, 20, 20, White);
}

void display_msg_warming_up(void) {
  ssd1306_SetCursor(TPOS_MSG_X, TPOS_MSG_Y);
  ssd1306_WriteString("  OCXO Warming up ", Font_7x10, White);
}

void display_msg_waiting_gps(void) {
  ssd1306_SetCursor(TPOS_MSG_X, TPOS_MSG_Y);
  ssd1306_WriteString("  Waiting for GPS ", Font_7x10, White);
}

void display_msg_not_load(void) {
  ssd1306_SetCursor(TPOS_MSG_X, TPOS_MSG_Y);
  ssd1306_WriteString("Not restored Data ", Font_7x10, White);
}

void display_msg_restore(int32_t d1, int32_t d2) {
  char str[32];
  sprintf(str,"Restored:%5ld,%2ldC", d1, d2);
  ssd1306_SetCursor(TPOS_MSG_X, TPOS_MSG_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

void display_msg_save(int32_t d1, int32_t d2) {
  char str[32];
  sprintf(str,"Saved:%5ld, %2ldC", d1, d2);
  ssd1306_SetCursor(TPOS_SAVE_X, TPOS_SAVE_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

void display_blank_row(uint8_t y, FontDef Font) {
  ssd1306_SetCursor(0, y);
  ssd1306_WriteString("                  ", Font, White);
}

void display_temperature(int32_t t) {
  char str[8];
  sprintf(str,"T:%2ldC", t);
  ssd1306_SetCursor(TPOS_TEMP_X, TPOS_TEMP_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

void display_pwm_data(void) {
  char str[8];
  itoa(get_control_pwm_data(), str, 10);// PWM data
  ssd1306_SetCursor(TPOS_PWM_X, TPOS_PWM_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

void display_time(void) {
  char str[4];
  strncpy(str, (char *)RMC_Time, 2);
  str[2]=0;
  ssd1306_SetCursor(TPOS_TIME_X, TPOS_TIME_Y);
  ssd1306_WriteString("UTC", Font_7x10, White);
  ssd1306_SetCursor(TPOS_TIME_X+7*3+4, TPOS_TIME_Y);
  ssd1306_WriteString(str, Font_7x10, White);
  ssd1306_WriteString(":", Font_7x10, White);
  strncpy(str, (char *)(RMC_Time+2), 2);
  ssd1306_WriteString(str, Font_7x10, White);
  ssd1306_WriteString(":", Font_7x10, White);
  strncpy(str, (char *)(RMC_Time+4), 2);
  ssd1306_WriteString(str, Font_7x10, White);
  ssd1306_WriteString(" ", Font_7x10, White);
}

void display_freq_error(void) {
  char str[16];
  draw_hline(30, 13, 96);
  draw_hline(30, 36, 96);
  sprintf(str,"%+4.3f", (float)(get_control_err_average())/1000.0f);
  ssd1306_SetCursor(TPOS_ERR_X, TPOS_ERR_Y);		// Averaged Error
  ssd1306_WriteString(str, Font_11x18, White);
  ssd1306_SetCursor(TPOS_ERR_X+11*6+4, TPOS_ERR_Y);
  ssd1306_WriteString("Hz", Font_11x18, White);
}

void display_summing_error(void) {
  char str[16];
  sprintf(str,"ERR:%4d(%3d)", (int) get_control_err_sum(), (int) get_control_time_cnt());
  ssd1306_SetCursor(TPOS_SERR_X, TPOS_SERR_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

void display_SatellitesUsed(void) {
  char str[8];
  ssd1306_SetCursor(TPOS_SATU_X, TPOS_SATU_Y);
  sprintf(str,"U:%2ld", GGA_SatellitesUsed);
  ssd1306_WriteString((char *)str, Font_7x10, White);
}

void display_SatellitesInView(void) {
  char str[8];
  ssd1306_SetCursor(TPOS_SATV_X, TPOS_SATV_Y);
  sprintf(str,"V:%2ld", GSV_SatellitesInView);
  ssd1306_WriteString((char *)str, Font_7x10, White);
}

void display_control_period(void) {
  char str[16];
  int32_t i;
  i = get_control_state();
  i = (i == 0)? 10 : i*10;
  sprintf(str,"%lds", i);
  ssd1306_SetCursor(TPOS_AVESEC_X, TPOS_AVESEC_Y);
  ssd1306_WriteString("AVERAGE", Font_7x10, White);
  ssd1306_SetCursor(TPOS_AVESEC_X+7*7+4, TPOS_AVESEC_Y);
  ssd1306_WriteString(str, Font_7x10, White);
}

//  Draw XBM to the screen buffer  7 Oct. 2023 T.K
//  XBM     => XBM to write
//  XBM_W   => XBM width
//  XBM_H   => XBM height
//  color   => Black or White
void display_XBM(uint8_t x, uint8_t y, const unsigned char* XBM, uint8_t XBM_width, uint8_t XBM_height, SSD1306_COLOR color)
{
	uint8_t i, j, k, d;
	d = 0;
	for (i = 0; i < XBM_height ; i++ ) {
		for (j=0; j < XBM_width ; j++ ) {
			k = j % 8;
			if ( k == 0) {
				d = *XBM;
				XBM++;
			}
			if ( (d >> k) & 0x01 ) {
				ssd1306_DrawPixel(x + j, y + i, color);
			} else {
				ssd1306_DrawPixel(x + j, y + i, !color);
			}
    	}
	}
}
