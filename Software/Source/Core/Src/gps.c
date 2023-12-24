#include "gps.h"

uint8_t RMC_Time[12];
uint8_t RMC_Status[2];
uint8_t GGA_Time[12];
uint8_t GGA_Latitude[12];
uint8_t GGA_NSindicator[6];
uint8_t GGA_Longitude[12];
uint8_t GGA_EWindicator[6];
int32_t GGA_PositionFixIndicator;
int32_t GGA_SatellitesUsed;
int32_t GSV_NumberOfMessages;
int32_t GSV_MessageNumber;
int32_t GSV_SatellitesInView;

uint8_t gps_msg[GPS_MSG_SIZE];
int32_t msg_idx = 0;
uint8_t chksum;
uint8_t sum = 0;
int32_t sumcnt = 0;
bool    msg_err = true;

uint8_t rx6_buf[1];
uint8_t rx6_q_buf[256]; // 8bit r/w pointer
uint8_t rx6_q_r;
uint8_t rx6_q_w;
uint8_t tx6_buf[1];
uint8_t tx6_it_flg;
uint8_t tx6_q_buf[256]; // 8bit r/w pointer
uint8_t tx6_q_r;
uint8_t tx6_q_w;

void uart6_rx_init(void) {
  rx6_q_r = 0;
  rx6_q_w = 0;
  HAL_UART_Receive_DMA(&huart6, rx6_buf, 1);
}

int32_t uart6_fgetc(void) {
  if (rx6_q_r != rx6_q_w)
    return rx6_q_buf[rx6_q_r++];
  else
    return -1;
}

void uart6_tx_init(void) {
  tx6_q_r = 0;
  tx6_q_w = 0;
  tx6_it_flg = 0;
}

void uart6_tx_it_st(void) {
  tx6_buf[0] = tx6_q_buf[tx6_q_r++];
  HAL_UART_Transmit_DMA(&huart6, tx6_buf, 1);
  tx6_it_flg = 1;
}

void uart6_fputc(uint8_t ch) {
  tx6_q_buf[tx6_q_w++] = ch;
  if (tx6_it_flg == 0)
    uart6_tx_it_st();
}

void uart6_fputs(uint8_t *str) {
  while (*str != '\0') uart6_fputc(*str++);
}

void uart6_init(void) {
  uart6_rx_init();
  uart6_tx_init();
}

void uart6_err_ckcl(void) {
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_FLAG(&huart6,
//                        UART_CLEAR_NEF | UART_CLEAR_OREF |
                          UART_FLAG_RXNE | UART_FLAG_ORE);
  }
  else if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_NE) ||
            __HAL_UART_GET_FLAG(&huart6, UART_FLAG_FE) ||
            __HAL_UART_GET_FLAG(&huart6, UART_FLAG_PE) ) {
    HAL_UART_Abort(&huart6);
  }
  else  return;
  HAL_UART_Receive_DMA(&huart6, rx6_buf, 1);
}

void  HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6) {
     rx6_q_buf[rx6_q_w++] = rx6_buf[0];
     HAL_UART_Receive_DMA(&huart6, rx6_buf, 1);
  }
}

void  HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6) {
    tx6_it_flg = 0;
    if (tx6_q_r != tx6_q_w)
      uart6_tx_it_st();
  }
}

uint8_t hexto8(int32_t c) {
  uint8_t r = 0xFF;

  if ((c >= '0') && (c <= '9')) {
    r = c - '0';	// convert to 0 - 9
  } else if ((c >='A')&&(c <= 'F')) {
    r = c - '7';	// convert to 10 - 15
  }

  return r;
}

void init_msg(void){
  msg_idx = 0;
  sum = 0;
  sumcnt = 0;
}

bool get_gps_msg(void){
  int c;

  while ((c = uart6_fgetc()) != -1) {
	gps_msg[msg_idx++] = (uint8_t) c;	// store to buffer

    if (c == '\n') {					// LF code?
      gps_msg[msg_idx] = 0; 			// store delimiter
      init_msg();
      if (msg_err) {
        return false;
      } else {
    	msg_err = true;					// prepare for next ; true is fail-safe
        return true;					// complete to get a sentence
      }
    }
    else if (msg_idx == GPS_MSG_SIZE) {	// illegal length of sentence?
      init_msg();						// ignore this received sentence
      msg_err = true;
      return false;
    }
    else {
      switch (sumcnt) {					// sequence for check sum
      case 0:
        if (c == '*') {					// found check sum header?
          sumcnt++;
        } else if (c != '$') {			// calculate check sum
          sum ^= (uint8_t) c;
        }
        break;

      case 1:
        sumcnt++;
        chksum = (hexto8(c) << 4);
        break;

      case 2:
        sumcnt++;
        chksum += hexto8(c);
        if (chksum == sum ) {			// match check sum?
          msg_err = false;
        }
        break;

      default:
        break;
      }
    }
  }
  return false;
}

uint8_t *get_msg_item_str(uint8_t *s, uint8_t *d) {
	uint8_t c;
	while ((c = *s++) != ',') {
		*d++ = c;
	}
	*d = 0x0;
	return s;
}

uint8_t *get_msg_item_int(uint8_t *s, int32_t *i) {
	uint8_t str[16];
	uint8_t *r;

	r = get_msg_item_str(s, str);
	*i = atoi((char *) str);
    return r;
}

int32_t IsTimeValid( uint8_t * time) {
  int32_t i, j;
  uint8_t c;

  j = strlen((char *)time);
  if (j < 1)
    return -1;

  for (i=0; i<j ; i++) {
    c = *time++;
    if (i != 6) {
      if ((c < '0') || (c>'9'))
        return 1;
    } else {
      if (c !=  '.' )
        return 1;
    }
  }
  return 0;
}

int32_t msg_parse(uint8_t *s){
	uint8_t dtype[8];
	int32_t r;

	s = get_msg_item_str(s, dtype);
	if (strcmp((char *)dtype, "$GPRMC") == 0) {
		s = get_msg_item_str(s, RMC_Time);
		get_msg_item_str(s, RMC_Status);
		r = 1;
		if (strcmp((char *)RMC_Status, "A") == 0) r += 4;
		if (IsTimeValid(RMC_Time) == 0) r += 2;
		return r;

	} else if (strcmp((char *)dtype, "$GPGGA") == 0) {
		s = get_msg_item_str(s, GGA_Time);
		s = get_msg_item_str(s, GGA_Latitude);
		s = get_msg_item_str(s, GGA_NSindicator);
		s = get_msg_item_str(s, GGA_Longitude);
		s = get_msg_item_str(s, GGA_EWindicator);
		s = get_msg_item_int(s, &GGA_PositionFixIndicator);
		get_msg_item_int(s, &GGA_SatellitesUsed);
		return 0;

	} else if (strcmp((char *)dtype, "$GPGSV") == 0) {
		s = get_msg_item_int(s, &GSV_NumberOfMessages);
		s = get_msg_item_int(s, &GSV_MessageNumber);
		get_msg_item_int(s, &GSV_SatellitesInView);
		return 0;
	}
	return -1;
}
