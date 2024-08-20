// i2c.h

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4.h"

#define I2C_ACK_DISABLE		0
#define I2C_ACK_ENABLE		1

#define I2C_FREQ_SM			100000
#define I2C_FREQ_FM			400000

#define I2C_DUTY_2			0
#define I2C_DUTY_16_9		1

#define I2C_SR_DISABLE 		0
#define I2C_SR_ENABLE		1

#define I2C_STATE_READY 	0
#define I2C_STATE_BUSY_TX 	1
#define I2C_STATE_BUSY_RX 	2

#define I2C_EV_STOPF		0
#define I2C_EV_RXNE			1
#define I2C_EV_TXE			2
#define I2C_EV_STX			3
#define I2C_EV_SRX			4

typedef struct
{
	uint8_t  ack;
	uint8_t  add;
	uint32_t freq;
	uint8_t  duty;
} i2c_cfg_t;

typedef struct
{
	uint8_t * p_tx_buf;
	uint8_t * p_rx_buf;
	uint32_t  tx_len;
	uint32_t  rx_len;
	uint8_t   txrx;
	uint8_t   addr;
	uint32_t  rx_size;
	uint8_t   sr;
} i2c_txn_t;

void i2c_clock_control(i2c_reg_map_t * p_i2cx, uint8_t toggle);

void i2c_init(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg);
void i2c_deinit(i2c_reg_map_t * p_i2cx);

void i2c_master_transmit_data(i2c_reg_map_t * p_i2cx, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr);
void i2c_master_receive_data(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr);
uint8_t i2c_master_transmit_data_irq(i2c_reg_map_t * p_i2cx, i2c_txn_t * txn, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr);
uint8_t i2c_master_receive_data_irq(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, i2c_txn_t * txn, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr);

void i2c_reset_transmit_data(i2c_reg_map_t * p_i2cx, i2c_txn_t * txn);
void i2c_reset_receive_data(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, i2c_txn_t * txn);

void i2c_slave_transmit_data(i2c_reg_map_t * p_i2cx, uint8_t data);
uint8_t i2c_slave_receive_data(i2c_reg_map_t * p_i2cx);

void i2c_irq_control_toggle(uint8_t position, uint8_t toggle);
void i2c_irq_set_priority(uint8_t position, uint8_t priority);
void i2c_irq_ev_handler(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, i2c_txn_t * txn);

__attribute__((weak)) void i2c_irq_callback(i2c_reg_map_t * p_i2cx, i2c_txn_t * txn, uint8_t event);

void i2c_peripheral_control(i2c_reg_map_t * p_i2cx, uint8_t toggle);

void i2c_generate_start_condition(i2c_reg_map_t * p_i2cx);
uint16_t i2c_get_flag_status(i2c_reg_map_t * p_i2cx, uint16_t flag);
void i2c_send_slave_address_write(i2c_reg_map_t * p_i2cx, uint8_t addr);
void i2c_send_slave_address_read(i2c_reg_map_t * p_i2cx, uint8_t addr);
void i2c_clear_addr_flag(i2c_reg_map_t * p_i2cx);
void i2c_ack_control(i2c_reg_map_t * p_i2cx, uint8_t toggle);
void i2c_generate_stop_condition(i2c_reg_map_t * p_i2cx);

#endif /* I2C_H_ */
