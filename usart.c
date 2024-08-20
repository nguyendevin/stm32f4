// usart.c

#include <stddef.h>

#include "usart.h"

static void usart_set_baud_rate(usart_reg_map_t * p_usartx, const usart_cfg_t * cfg);

static uint32_t usart_get_flag_status(const usart_reg_map_t * p_usartx, uint32_t flag);

void usart_clock_enable(usart_reg_map_t * p_usartx)
{
	if (USART1 == p_usartx)
	{
		USART1_CLOCK_ENABLE();
	}
	else if (USART2 == p_usartx)
	{
		USART2_CLOCK_ENABLE();
	}
	else if (USART3 == p_usartx)
	{
		USART3_CLOCK_ENABLE();
	}
	else if (UART4 == p_usartx)
	{
		UART4_CLOCK_ENABLE();
	}
	else if (UART5 == p_usartx)
	{
		UART5_CLOCK_ENABLE();
	}
	else if (USART6 == p_usartx)
	{
		USART6_CLOCK_ENABLE();
	}
	else
	{

	}
}

void usart_clock_disable(usart_reg_map_t * p_usartx)
{
	if (USART1 == p_usartx)
	{
		USART1_CLOCK_DISABLE();
	}
	else if (USART2 == p_usartx)
	{
		USART2_CLOCK_DISABLE();
	}
	else if (USART3 == p_usartx)
	{
		USART3_CLOCK_DISABLE();
	}
	else if (UART4 == p_usartx)
	{
		UART4_CLOCK_DISABLE();
	}
	else if (UART5 == p_usartx)
	{
		UART5_CLOCK_DISABLE();
	}
	else if (USART6 == p_usartx)
	{
		USART6_CLOCK_DISABLE();
	}
	else
	{

	}
}

void usart_enable(usart_reg_map_t * p_usartx)
{
	p_usartx->cr1 |= (0x1 << USART_CR1_UE);
}

void usart_disable(usart_reg_map_t * p_usartx)
{
	p_usartx->cr1 &= ~(0x1 << USART_CR1_UE);
}

void usart_init(usart_reg_map_t * p_usartx, const usart_cfg_t * cfg)
{
	usart_clock_enable(p_usartx);

	usart_set_baud_rate(p_usartx, cfg);

	uint16_t tmp = 0;

	switch (cfg->mode)
	{
		case USART_MODE_TX:
			tmp |= (0x1 << USART_CR1_TE);
		break;

		case USART_MODE_RX:
			tmp |= (0x1 << USART_CR1_RE);
		break;

		case USART_MODE_TXRX:
			tmp |= ((0x1 << USART_CR1_TE) | (0x1 << USART_CR1_RE));
		break;

		default:
		break;
	}

	switch (cfg->parity)
	{
		case USART_PARITY_EVEN:
			tmp &= ~(0x1 << USART_CR1_PS);
			tmp |= (0x1 << USART_CR1_PCE);
		break;

		case USART_PARITY_ODD:
			tmp |= (0x1 << USART_CR1_PS);
			tmp |= (0x1 << USART_CR1_PCE);
		break;

		case USART_PARITY_DISABLE:
		break;

		default:
		break;
	}

	tmp |= (cfg->word_length << USART_CR1_M);

	p_usartx->cr1 = tmp;

	tmp = 0;

	tmp |= (cfg->stop_bits << USART_CR2_STOP);

	p_usartx->cr2 = tmp;

	tmp = 0;

	switch (cfg->hw_flow_ctrl)
	{
		case USART_HW_FLOW_CTRL_RTS:
			tmp |= (0x1 << USART_CR3_RTSE);
		break;

		case USART_HW_FLOW_CTRL_CTS:
			tmp |= (0x1 << USART_CR3_CTSE);
		break;

		case USART_HW_FLOW_CTRL_RTSCTS:
			tmp |= ((0x1 << USART_CR3_RTSE) | (0x1 << USART_CR3_CTSE));
		break;

		default:
		break;
	}

	p_usartx->cr3 = tmp;
}

void usart_deinit(usart_reg_map_t * p_usartx)
{
	if (USART1 == p_usartx)
	{
		USART1_RESET();
	}
	else if (USART2 == p_usartx)
	{
		USART2_RESET();
	}
	else if (USART3 == p_usartx)
	{
		USART3_RESET();
	}
	else if (UART4 == p_usartx)
	{
		UART4_RESET();
	}
	else if (UART5 == p_usartx)
	{
		UART5_RESET();
	}
	else if (USART6 == p_usartx)
	{
		USART6_RESET();
	}
	else
	{

	}

	usart_clock_disable(p_usartx);
}

void usart_transmit_data(usart_reg_map_t * p_usartx, const uint8_t * p_data, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++)
	{
		while (usart_get_flag_status(p_usartx, (0x1 << USART_SR_TXE)) == FLAG_RESET);

		switch (p_usartx->cr1 & (0x1 << USART_CR1_M))
		{
			case USART_WORD_LENGTH_8:
				p_usartx->dr = *p_data;
				p_data++;
			break;

			case USART_WORD_LENGTH_9:
				p_usartx->dr = (*((uint16_t *) p_data) & 0x1FF);
				if (p_usartx->cr1 & (0x1 << USART_CR1_PCE))
				{
					p_data++;
					p_data++;
				}
				else
				{
					p_data++;
				}
			break;

			default:
			break;
		}
	}

	while (!usart_get_flag_status(p_usartx, (0x1 << USART_SR_TC)));
}

void usart_receive_data(usart_reg_map_t * p_usartx, uint8_t * p_data, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++)
	{
		while (usart_get_flag_status(p_usartx, (0x1 << USART_SR_RXNE)) == FLAG_RESET);

		switch (p_usartx->cr1 & (0x1 << USART_CR1_M))
		{
			case USART_WORD_LENGTH_8:
				if (p_usartx->cr1 & (0x1 << USART_CR1_PCE))
				{
					*p_data = (uint8_t) (p_usartx->dr & 0x7F);
				}
				else
				{
					*p_data = (uint8_t) (p_usartx->dr);
				}
				p_data++;
			break;

			case USART_WORD_LENGTH_9:
				if (p_usartx->cr1 & (0x1 << USART_CR1_PCE))
				{
					*p_data = (uint8_t) (p_usartx->dr);
					p_data++;
				}
				else
				{
					*((uint16_t *) p_data) = (p_usartx->dr & 0x1FF);
					p_data++;
					p_data++;
				}
			break;

			default:
			break;
		}
	}
}

uint8_t usart_transmit_data_irq(usart_reg_map_t * p_usartx, usart_txn_t * txn, uint8_t * p_data, uint32_t len)
{
	uint8_t state = txn->tx_state;

	if (USART_STATE_TX != state)
	{
		p_usartx->cr1 |= (0x1 << USART_CR1_TXEIE);

		p_usartx->cr1 |= (0x1 << USART_CR1_TCIE);

		txn->p_tx_buf = p_data;

		txn->tx_len = len;

		txn->tx_state = USART_STATE_TX;
	}

	return state;
}

uint8_t usart_receive_data_irq(usart_reg_map_t * p_usartx, usart_txn_t * txn, uint8_t * p_data, uint32_t len)
{
	uint8_t state = txn->rx_state;

	if (USART_STATE_RX != state)
	{
		p_usartx->cr1 |= (0x1 << USART_CR1_RXNEIE);

		txn->p_rx_buf = p_data;

		txn->rx_len = len;

		txn->rx_state = USART_STATE_RX;
	}

	return state;
}

void usart_irq_enable(uint8_t position)
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

void usart_irq_disable(uint8_t position)
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

void usart_irq_set_priority(uint8_t position, uint8_t priority)
{
	*(NVIC_IPR + (position / 4)) |= (priority << ((8 * (position % 4)) + 4));
}

void usart_irq_handler(usart_reg_map_t * p_usartx, usart_txn_t * txn)
{
	if ((p_usartx->sr |= (0x1 << USART_SR_RXNE)) && \
		(p_usartx->cr1 |= (0x1 << USART_CR1_RXNEIE)) && \
		(txn->rx_state == USART_STATE_RX))
	{
		if (txn->rx_len > 0)
		{
			switch (p_usartx->cr1 & (0x1 << USART_CR1_M))
			{
				case USART_WORD_LENGTH_8:
					if (p_usartx->cr1 & (0x1 << USART_CR1_PCE))
					{
						*txn->p_rx_buf = (uint8_t) (p_usartx->dr & 0x7F);
					}
					else
					{
						*txn->p_rx_buf = (uint8_t) (p_usartx->dr);
					}
					txn->p_rx_buf++;
					txn->rx_len--;
				break;

				case USART_WORD_LENGTH_9:
					if (p_usartx->cr1 & (0x1 << USART_CR1_PCE))
					{
						*txn->p_rx_buf = (uint8_t) (p_usartx->dr);
						txn->p_rx_buf++;
						txn->rx_len--;
					}
					else
					{
						*((uint16_t *) txn->p_rx_buf) = (p_usartx->dr & 0x1FF);
						txn->p_rx_buf+=2;
						txn->rx_len-=2;
					}
				break;

				default:
				break;
			}
		}

		if (txn->rx_len == 0)
		{
			p_usartx->cr1 &= ~(0x1 << USART_CR1_RXNEIE);
			txn->rx_state = USART_STATE_RDY;
			usart_irq_callback(p_usartx, txn, 0);
		}
	}

	if ((p_usartx->sr |= (0x1 << USART_SR_TC)) && \
	    (p_usartx->cr1 |= (0x1 << USART_CR1_TCIE)) && \
		(txn->rx_state == USART_STATE_TX))
	{
		if (txn->tx_len > 0)
		{
			p_usartx->sr &= ~(0x1 << USART_SR_TC);
			txn->p_tx_buf = NULL;
			txn->tx_len = 0;
			txn->tx_state = USART_STATE_RDY;
			usart_irq_callback(p_usartx, txn, 1);
		}
	}

	if ((p_usartx->sr |= (0x1 << USART_SR_TXE)) && \
		 (p_usartx->cr1 |= (0x1 << USART_CR1_TXEIE)) && \
		 (txn->rx_state == USART_STATE_TX))
	{
		if (txn->tx_len > 0)
		{
			switch (p_usartx->cr1 & (0x1 << USART_CR1_M))
			{
				case USART_WORD_LENGTH_8:
					p_usartx->dr = *txn->p_tx_buf;
					txn->p_tx_buf++;
					txn->tx_len--;
				break;

				case USART_WORD_LENGTH_9:
					p_usartx->dr = (*((uint16_t *) txn->p_tx_buf) & 0x1FF);
					if (p_usartx->cr1 & (0x1 << USART_CR1_PCE))
					{
						txn->p_tx_buf++;
						txn->p_tx_buf++;
						txn->tx_len-=2;
					}
					else
					{
						txn->p_tx_buf++;
						txn->tx_len--;
					}
				break;

				default:
				break;
			}
		}

		if (txn->tx_len == 0)
		{
			p_usartx->cr1 &= ~(0x1 << USART_CR1_TXEIE);
			txn->tx_state = USART_STATE_RDY;
			usart_irq_callback(p_usartx, txn, 2);
		}
	}
}

__attribute__((weak)) void usart_irq_callback(usart_reg_map_t * p_usartx, usart_txn_t * txn, uint8_t event)
{

}

static void usart_set_baud_rate(usart_reg_map_t * p_usartx, const usart_cfg_t * cfg)
{
	uint32_t tmp = 0;

	p_usartx->brr = tmp;
}

static uint32_t usart_get_flag_status(const usart_reg_map_t * p_usartx, uint32_t flag)
{
	return (p_usartx->sr & flag ? FLAG_SET : FLAG_RESET);
}
