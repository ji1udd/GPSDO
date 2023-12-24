#ifndef __OCXO_H
#define __OCXO_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#define TARGET_FREQ		100000000 // unit 0.1Hz
#define ERR_THRESHOLD	3
#define Ki_COARSE		5
#define Ki_FINE			1
#define MAX_SUM_NUM		50

extern	bool	previous_cnt_valid;
extern	bool	update_cnt_1pps;
extern	int32_t	cnt_1pps;
extern	TIM_HandleTypeDef htim1;


void	ocxo_init(void);
int32_t ocxo_control(void);
int32_t get_control_state(void);
int32_t get_control_time_cnt(void);
int32_t get_control_err_sum(void);
int32_t get_control_pwm_data(void);
int32_t get_control_err_average(void);
void 	set_control_pwm_data(int32_t i);

#endif /* __OCXO_H */
