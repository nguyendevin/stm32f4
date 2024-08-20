// usart.h

#ifndef USART_H_
#define USART_H_

#include "stm32f4.h"

#define USART_BR_1200				1200
#define USART_BR_2400				2400
#define USART_BR_9600				9600
#define USART_BR_19200				19200
#define USART_BR_38400				38400
#define USART_BR_57600				57600
#define USART_BR_115200				115200
#define USART_BR_230400				230400
#define USART_BR_460800				460800
#define USART_BR_921600				921600
#define USART_BR_2000000			2000000
#define USART_BR_3000000			3000000

#define USART_MODE_TX 				0
#define USART_MODE_RX 				1
#define USART_MODE_TXRX 			2

#define USART_PARITY_EVEN			0
#define USART_PARITY_ODD			1
#define USART_PARITY_DISABLE 		2

#define USART_WORD_LENGTH_8			0
#define USART_WORD_LENGTH_9			1

#define USART_STOP_BITS_1			0
#define USART_STOP_BITS_0_5			1
#define USART_STOP_BITS_2			2
#define USART_STOP_BITS_1_5			3

#define USART_HW_FLOW_CTRL_RTS		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTSCTS 	2
#define USART_HW_FLOW_CTRL_DISABLE 	3

#define USART_STATE_TX				0
#define USART_STATE_RX				1
#define USART_STATE_RDY				2

typedef struct
{
	uint32_t br;
	uint8_t  mode;
	uint8_t	 parity;
	uint8_t  word_length;
	uint8_t  stop_bits;
	uint8_t  hw_flow_ctrl;
} usart_cfg_t;

typedef struct
{
	uint8_t * p_tx_buf;
	uint8_t * p_rx_buf;
	uint32_t  tx_len;
	uint32_t  rx_len;
	uint8_t   tx_state;
	uint8_t   rx_state;
} usart_txn_t;

void usart_clock_enable(usart_reg_map_t * p_usartx);
void usart_clock_disable(usart_reg_map_t * p_usartx);

void usart_enable(usart_reg_map_t * p_usartx);
void usart_disable(usart_reg_map_t * p_usartx);

void usart_init(usart_reg_map_t * p_usartx, const usart_cfg_t * cfg);
void usart_deinit(usart_reg_map_t * p_usartx);

void usart_transmit_data(usart_reg_map_t * p_usartx, const uint8_t * p_data, uint32_t len);
void usart_receive_data(usart_reg_map_t * p_usartx, uint8_t * p_data, uint32_t len);
uint8_t usart_transmit_data_irq(usart_reg_map_t * p_usartx, usart_txn_t * txn, uint8_t * p_data, uint32_t len);
uint8_t usart_receive_data_irq(usart_reg_map_t * p_usartx, usart_txn_t * txn, uint8_t * p_data, uint32_t len);

void usart_irq_enable(uint8_t position);
void usart_irq_disable(uint8_t position);
void usart_irq_set_priority(uint8_t position, uint8_t priority);
void usart_irq_handler(usart_reg_map_t * p_usartx, usart_txn_t * txn);

__attribute__((weak)) void usart_irq_callback(usart_reg_map_t * p_usartx, usart_txn_t * txn, uint8_t event);

#endif /* USART_H_ */
