#ifndef __GPS_H
#define __GPS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#define GPS_MSG_SIZE 256

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;
extern uint8_t gps_msg[GPS_MSG_SIZE];

extern uint8_t RMC_Time[12];
extern uint8_t RMC_Status[2];
extern uint8_t GGA_Latitude[12];
extern uint8_t GGA_NSindicator[6];
extern uint8_t GGA_Longitude[12];
extern uint8_t GGA_EWindicator[6];
extern int32_t GGA_SatellitesUsed;
extern int32_t GSV_SatellitesInView;

void	uart6_rx_init(void);
int32_t	uart6_fgetc(void);
void	uart6_tx_init(void);
void	uart6_tx_it_st(void);
void	uart6_fputc(uint8_t ch);
void	uart6_fputs(uint8_t *str);
void	uart6_init(void);
void	uart6_err_ckcl(void);
void	HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void	HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
bool	get_gps_msg(void);
uint8_t *get_msg_item_str(uint8_t *s, uint8_t *d);
uint8_t *get_msg_item_int(uint8_t *s, int32_t *i);
int32_t msg_parse(uint8_t *s);

#endif /* __GPS_H */
