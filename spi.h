// spi.h

#ifndef SPI_H_
#define SPI_H_

#include <stddef.h>

#include "stm32f4.h"

#define SPI_CPHA_LOW 		0
#define SPI_CPHA_HIGH 		1

#define SPI_CPOL_LOW 		0
#define SPI_CPOL_HIGH 		1

#define SPI_MSTR_SLAVE 		0
#define SPI_MSTR_MASTER 	1

#define SPI_BR_2 			0
#define SPI_BR_4 			1
#define SPI_BR_8 			2
#define SPI_BR_16 			3
#define SPI_BR_32 			4
#define SPI_BR_64 			5
#define SPI_BR_128 			6
#define SPI_BR_256 			7

#define SPI_SPE_ENABLE 		0
#define SPI_SPE_DISABLE 	1

#define SPI_SSM_HW			0
#define SPI_SSM_SW			1

#define SPI_DFF_8 			0
#define SPI_DFF_16 			1

#define SPI_MODE_FD 		0
#define SPI_MODE_HD 		1
#define SPI_MODE_SR 		2

#define SPI_STATE_READY 	0
#define SPI_STATE_BUSY_TX 	1
#define SPI_STATE_BUSY_RX 	2

#define SPI_EVENT_TX 		0
#define SPI_EVENT_RX 		1
#define SPI_EVENT_OVR 		2

typedef struct
{
	uint8_t cpha;
	uint8_t cpol;
	uint8_t mstr;
	uint8_t br;
	uint8_t ssm;
	uint8_t dff;
	uint8_t mode;
} spi_cfg_t;

typedef struct
{
	uint8_t * p_tx_buf;
	uint8_t * p_rx_buf;
	uint32_t  tx_len;
	uint32_t  rx_len;
	uint8_t   tx_state;
	uint8_t   rx_state;
} spi_txn_t;

void spi_clock_enable(spi_reg_map_t * p_spix);
void spi_clock_disable(spi_reg_map_t * p_spix);

void spi_init(spi_reg_map_t * p_spix, const spi_cfg_t * cfg);
void spi_deinit(spi_reg_map_t * p_spix);

void spi_transmit_data(spi_reg_map_t * p_spix, const uint8_t * p_data, uint32_t len);
void spi_receive_data(spi_reg_map_t * p_spix, uint8_t * p_data, uint32_t len);
uint8_t spi_transmit_data_irq(spi_reg_map_t * p_spix, spi_txn_t * txn, uint8_t * p_data, uint32_t len);
uint8_t spi_receive_data_irq(spi_reg_map_t * p_spix, spi_txn_t * txn, uint8_t * p_data, uint32_t len);

void spi_irq_enable(uint8_t position);
void spi_irq_disable(uint8_t position);
void spi_irq_set_priority(uint8_t position, uint8_t priority);
void spi_irq_handler(spi_reg_map_t * p_spix, spi_txn_t * txn);

__attribute__((weak)) void spi_irq_callback(spi_reg_map_t * p_spix, spi_txn_t * txn, uint8_t event);

#endif /* SPI_H_ */
