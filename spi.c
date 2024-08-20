// spi.c

#include "spi.h"

static uint32_t spi_get_flag_status(const spi_reg_map_t * p_spix, uint32_t flag);

static void spi_irq_handle_txe(spi_reg_map_t * p_spix, spi_txn_t * txn);
static void spi_irq_handle_rxne(spi_reg_map_t * p_spix, spi_txn_t * txn);
static void spi_irq_handle_ovr(spi_reg_map_t * p_spix, spi_txn_t * txn);

void spi_clock_enable(spi_reg_map_t * p_spix)
{
	if (SPI1 == p_spix)
	{
		SPI1_CLOCK_ENABLE();
	}
	else if (SPI2 == p_spix)
	{
		SPI2_CLOCK_ENABLE();
	}
	else if (SPI3 == p_spix)
	{
		SPI3_CLOCK_ENABLE();
	}
	else
	{

	}
}

void spi_clock_disable(spi_reg_map_t * p_spix)
{
	if (SPI1 == p_spix)
	{
		SPI1_CLOCK_DISABLE();
	}
	else if (SPI2 == p_spix)
	{
		SPI2_CLOCK_DISABLE();
	}
	else if (SPI3 == p_spix)
	{
		SPI3_CLOCK_DISABLE();
	}
	else
	{

	}
}

void spi_init(spi_reg_map_t * p_spix, const spi_cfg_t * cfg)
{
	spi_clock_enable(p_spix);

	uint16_t tmp = 0;

	tmp |= (cfg->cpha << SPI_CR1_CPHA);

	tmp |= (cfg->cpol << SPI_CR1_CPOL);

	tmp |= (cfg->mstr << SPI_CR1_MSTR);

	tmp |= (cfg->br << SPI_CR1_BR);

	tmp |= (cfg->ssm << SPI_CR1_SSM);

	tmp |= (cfg->dff << SPI_CR1_DFF);

	switch (cfg->mode)
	{
		case SPI_MODE_HD:
			tmp |= (0x1 << SPI_CR1_BIDIMODE);
		break;

		case SPI_MODE_SR:
			tmp |= (0x1 << SPI_CR1_RXONLY);
		break;

		default:
		break;
	}

	p_spix->cr1 = tmp;
}

void spi_deinit(spi_reg_map_t * p_spix)
{
	if (SPI1 == p_spix)
	{
		SPI1_RESET();
	}
	else if (SPI2 == p_spix)
	{
		SPI2_RESET();
	}
	else if (SPI3 == p_spix)
	{
		SPI3_RESET();
	}
	else
	{

	}

	spi_clock_disable(p_spix);
}

void spi_transmit_data(spi_reg_map_t * p_spix, const uint8_t * p_data, uint32_t len)
{
	while (len > 0)
	{
		while (spi_get_flag_status(p_spix, (0x1 << SPI_SR_TXE)) == FLAG_RESET);

		switch (p_spix->cr1 & (0x1 << SPI_CR1_DFF))
		{
			case SPI_DFF_8:
				p_spix->dr = *p_data;
				p_data++;
				len--;
			break;

			case SPI_DFF_16:
				p_spix->dr = *((uint16_t *) p_data);
				(uint16_t *) p_data++;
				len-=2;
			break;

			default:
			break;
		}
	}
}

void spi_receive_data(spi_reg_map_t * p_spix, uint8_t * p_data, uint32_t len)
{
	while (len > 0)
	{
		while (spi_get_flag_status(p_spix, (0x1 << SPI_SR_RXNE)) == FLAG_RESET);

		switch (p_spix->cr1 & (0x1 << SPI_CR1_DFF))
		{
			case SPI_DFF_8:
				*p_data = p_spix->dr;
				p_data++;
				len--;
			break;

			case SPI_DFF_16:
				*((uint16_t *) p_data) = p_spix->dr;
				(uint16_t *) p_data++;
				len-=2;
			break;

			default:
			break;
		}
	}
}

uint8_t spi_transmit_data_irq(spi_reg_map_t * p_spix, spi_txn_t * txn, uint8_t * p_data, uint32_t len)
{
	uint8_t state = txn->tx_state;

	if (SPI_STATE_BUSY_TX != state)
	{
		p_spix->cr2 |= (0x1 << SPI_CR2_TXEIE);

		txn->p_tx_buf = p_data;

		txn->tx_len = len;

		txn->tx_state = SPI_STATE_BUSY_TX;
	}

	return state;
}

uint8_t spi_receive_data_irq(spi_reg_map_t * p_spix, spi_txn_t * txn, uint8_t * p_data, uint32_t len)
{
	uint8_t state = txn->rx_state;

	if (SPI_STATE_BUSY_RX != state)
	{
		p_spix->cr2 |= (0x1 << SPI_CR2_RXNEIE);

		txn->p_rx_buf = p_data;

		txn->rx_len = len;

		txn->rx_state = SPI_STATE_BUSY_RX;
	}

	return state;
}

void spi_irq_enable(uint8_t position)
{
	if (position < 32)
	{
		*NVIC_ISER0 |= (0x1 << position);
	}
	else if (position >= 32 && position < 64)
	{
		*NVIC_ISER1 |= (0x1 << (position % 32));
	}
	else if (position >= 64 && position < 96)
	{
		*NVIC_ISER2 |= (0x1 << (position % 64));
	}
	else if (position >= 96 && position < 128)
	{
		*NVIC_ISER3 |= (0x1 << (position % 96));
	}
	else if (position >= 128 && position < 160)
	{
		*NVIC_ISER4 |= (0x1 << (position % 128));
	}
	else if (position >= 160 && position < 192)
	{
		*NVIC_ISER5 |= (0x1 << (position % 160));
	}
	else if (position >= 192 && position < 224)
	{
		*NVIC_ISER6 |= (0x1 << (position % 192));
	}
	else if (position >= 224 && position < 256)
	{
		*NVIC_ISER7 |= (0x1 << (position % 224));
	}
	else
	{

	}
}

void spi_irq_disable(uint8_t position)
{
	if (position < 32)
	{
		*NVIC_ICER0 |= (0x1 << position);
	}
	else if (position >= 32 && position < 64)
	{
		*NVIC_ICER1 |= (0x1 << (position % 32));
	}
	else if (position >= 64 && position < 96)
	{
		*NVIC_ICER2 |= (0x1 << (position % 64));
	}
	else if (position >= 96 && position < 128)
	{
		*NVIC_ICER3 |= (0x1 << (position % 96));
	}
	else if (position >= 128 && position < 160)
	{
		*NVIC_ICER4 |= (0x1 << (position % 128));
	}
	else if (position >= 160 && position < 192)
	{
		*NVIC_ICER5 |= (0x1 << (position % 160));
	}
	else if (position >= 192 && position < 224)
	{
		*NVIC_ICER6 |= (0x1 << (position % 192));
	}
	else if (position >= 224 && position < 256)
	{
		*NVIC_ICER7 |= (0x1 << (position % 224));
	}
	else
	{

	}
}

void spi_irq_set_priority(uint8_t position, uint8_t priority)
{
	*(NVIC_IPR + (position / 4)) |= (priority << ((8 * (position % 4)) + 4));
}

void spi_irq_handler(spi_reg_map_t * p_spix, spi_txn_t * txn)
{
	if ((p_spix->sr | (0x1 << SPI_SR_TXE)) && (p_spix->cr2 | (0x1 << SPI_CR2_TXEIE)))
	{
		spi_irq_handle_txe(p_spix, txn);
	}

	if ((p_spix->sr | (0x1 << SPI_SR_RXNE)) && (p_spix->cr2 | (0x1 << SPI_CR2_RXNEIE)))
	{
		spi_irq_handle_rxne(p_spix, txn);
	}

	if ((p_spix->sr | (0x1 << SPI_SR_OVR)) && (p_spix->cr2 | (0x1 << SPI_CR2_ERRIE)))
	{
		spi_irq_handle_ovr(p_spix, txn);
	}
}

__attribute__((weak)) void spi_irq_callback(spi_reg_map_t * p_spix, spi_txn_t * txn, uint8_t event)
{

}

static uint32_t spi_get_flag_status(const spi_reg_map_t * p_spix, uint32_t flag)
{
	return (p_spix->sr & flag ? FLAG_SET : FLAG_RESET);
}

static void spi_irq_handle_txe(spi_reg_map_t * p_spix, spi_txn_t * txn)
{
	switch (p_spix->cr1 & (0x1 << SPI_CR1_DFF))
	{
		case SPI_DFF_8:
			p_spix->dr = *txn->p_tx_buf;
			txn->p_tx_buf++;
			txn->tx_len--;
		break;

		case SPI_DFF_16:
			p_spix->dr = *((uint16_t *) txn->p_tx_buf);
			(uint16_t *) txn->p_tx_buf++;
			txn->tx_len-=2;
		break;

		default:
		break;
	}

	if (txn->tx_len == 0)
	{
		p_spix->cr2 &= ~(0x1 << SPI_CR2_TXEIE);

		txn->p_tx_buf = NULL;

		txn->tx_len = 0;

		txn->tx_state = SPI_STATE_READY;

		spi_irq_callback(p_spix, txn, SPI_EVENT_TX);
	}
}

static void spi_irq_handle_rxne(spi_reg_map_t * p_spix, spi_txn_t * txn)
{
	switch (p_spix->cr1 & (0x1 << SPI_CR1_DFF))
	{
		case SPI_DFF_8:
			*txn->p_rx_buf = p_spix->dr;
			txn->p_rx_buf++;
			txn->rx_len--;
		break;

		case SPI_DFF_16:
			*((uint16_t *) txn->p_rx_buf) = p_spix->dr;
			(uint16_t *) txn->p_rx_buf++;
			txn->rx_len-=2;
		break;

		default:
		break;
	}

	if (txn->rx_len == 0)
	{
		p_spix->cr2 &= ~(0x1 << SPI_CR2_RXNEIE);

		txn->p_rx_buf = NULL;

		txn->rx_len = 0;

		txn->rx_state = SPI_STATE_READY;

		spi_irq_callback(p_spix, txn, SPI_EVENT_RX);
	}
}

static void spi_irq_handle_ovr(spi_reg_map_t * p_spix, spi_txn_t * txn)
{
	if (SPI_STATE_BUSY_TX != txn->tx_state)
	{
		p_spix->dr;
		p_spix->sr;
	}

	spi_irq_callback(p_spix, txn, SPI_EVENT_OVR);
}
