#include "ocxo.h"
#include "gps.h"

int32_t err_average = 0;
int32_t control_state = 0;
int32_t time_cnt = 0;
int32_t err_sum = 0;
int32_t pwm_data = 31000;

void set_control_pwm_data(int32_t i){
	pwm_data = i;
}

int32_t get_control_state(void){
	return control_state;
}

int32_t get_control_time_cnt(void){
	return time_cnt;
}

int32_t get_control_err_sum(void){
	return err_sum;
}

int32_t get_control_pwm_data(void){
	return pwm_data;
}

int32_t get_control_err_average(void){
	return err_average;
}

void ocxo_init(void){
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (uint16_t)pwm_data);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
	TIM1-> DIER = 0x0003;
}

int32_t ocxo_control(void){
	int32_t freq_err;
	int32_t freq_err_abs;
	int32_t Ki;

	if (update_cnt_1pps) {
	    update_cnt_1pps = false;
		freq_err = (int32_t) cnt_1pps - TARGET_FREQ;
		freq_err_abs =  abs(freq_err);
    	
		if (freq_err_abs < 100) {
			time_cnt++;
    		
			switch(control_state) {
			case 0:
				err_sum += freq_err;
				if (time_cnt > 10) {
					time_cnt = 1;
    				
					if (abs(err_sum) < 1) control_state = 1;
					err_average = (err_sum * 100 + 5) / 10;
					err_sum = 0;
				}

				Ki = freq_err_abs < ERR_THRESHOLD ? Ki_FINE : Ki_COARSE;
				pwm_data = pwm_data - (freq_err * Ki);
				break;

			default:
				err_sum += freq_err;
				if (time_cnt > (10 * control_state)) {
					time_cnt = 1;
    				
					if (err_sum == 0) {
						control_state++;
						if (control_state > MAX_SUM_NUM) control_state = MAX_SUM_NUM;
					} else if (err_sum > 1) {
						pwm_data--;
						previous_cnt_valid = false; // ignore 1PSS measurement once after PWM data is changed
					} else if (err_sum < -1) {
						pwm_data++;
						previous_cnt_valid = false; // ignore 1PSS measurement once after PWM data is changed
					}
					err_average = (err_sum * 100 + 5) / control_state / 10;
					err_sum = 0;
				}
				break;
			}

			if (pwm_data > 65535) pwm_data = 65535;
			else if (pwm_data < 0) pwm_data = 0;
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (uint16_t)pwm_data);
		}
		return 1;
	}
    else {
    	return 0;
    } // if
}
