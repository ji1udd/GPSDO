#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "ssd1306.h"
#include "gps.h"
#include "ocxo.h"

#define TPOS_Y1			0		// display Y position
#define TPOS_Y2			16
#define TPOS_Y3			41
#define TPOS_Y4			53

#define IPOS_Center_X	54
#define IPOS_Center_Y	20
#define IPOS_Corner_X	2
#define IPOS_Corner_Y	5

#define TPOS_SATU_X		36
#define TPOS_SATU_Y		TPOS_Y1
#define TPOS_SATV_X		78
#define TPOS_SATV_Y		TPOS_Y1
#define TPOS_GPSVALID_X	120
#define TPOS_GPSVALID_Y	TPOS_Y1

#define TPOS_ERR_X		32
#define TPOS_ERR_Y		TPOS_Y2

#define TPOS_AVESEC_X	0
#define TPOS_AVESEC_Y	TPOS_Y3
#define TPOS_TEMP_X		92
#define TPOS_TEMP_Y		TPOS_Y3
#define TPOS_LAT_X		8
#define TPOS_LAT_Y		TPOS_Y3

#define TPOS_TIME_X		0
#define TPOS_TIME_Y		TPOS_Y4
#define TPOS_PWM_X 		92
#define TPOS_PWM_Y 		TPOS_Y4
#define TPOS_LON_X		8
#define TPOS_LON_Y 		TPOS_Y4

#define TPOS_MSG_X		0
#define TPOS_MSG_Y		TPOS_Y4
#define TPOS_SAVE_X		8
#define TPOS_SAVE_Y		TPOS_Y4
#define TPOS_SERR_X		0
#define TPOS_SERR_Y		TPOS_Y4

void display_sat_icon_sig(uint8_t x, uint8_t y, int32_t pat);
void display_blank_row(uint8_t y, FontDef Font);
void display_Latitude(void);
void display_Longitude(void);
void draw_hline(uint8_t x, uint8_t y, uint8_t w);
void display_heater_icon(uint8_t x, uint8_t y);
void display_satellite_icon(uint8_t x, uint8_t y);
void display_msg_warming_up(void);
void display_msg_waiting_gps(void);
void display_msg_not_load(void);
void display_msg_restore(int32_t d1, int32_t d2);
void display_msg_save(int32_t d1, int32_t d2);
void display_temperature(int32_t t);
void display_pwm_data(void);
void display_time(void);
void display_freq_error(void);
void display_summing_error(void);
void display_SatellitesUsed(void);
void display_SatellitesInView(void);
void display_control_period(void);
void display_XBM(uint8_t x, uint8_t y, const unsigned char* XBM, uint8_t XBM_width, uint8_t XBM_height, SSD1306_COLOR color);

#endif /* __DISPLAY_H */
