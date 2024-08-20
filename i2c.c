// i2c.c

#include <stddef.h>

#include "rcc.h"
#include "i2c.h"

void i2c_clock_control(i2c_reg_map_t * p_i2cx, uint8_t toggle)
{
	if (ENABLE == toggle)
	{
		if (I2C1 == p_i2cx)
		{
			I2C1_CLOCK_ENABLE();
		}
		else if (I2C2 == p_i2cx)
		{
			I2C2_CLOCK_ENABLE();
		}
		else if (I2C3 == p_i2cx)
		{
			I2C3_CLOCK_ENABLE();
		}
		else
		{

		}
	}
	else
	{
		if (I2C1 == p_i2cx)
		{
			I2C1_CLOCK_DISABLE();
		}
		else if (I2C2 == p_i2cx)
		{
			I2C2_CLOCK_DISABLE();
		}
		else if (I2C3 == p_i2cx)
		{
			I2C3_CLOCK_DISABLE();
		}
		else
		{

		}
	}
}

void i2c_init(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg)
{
	i2c_clock_control(p_i2cx, ENABLE);

	uint32_t tmp = 0;

	tmp |= (cfg->ack << I2C_CR1_ACK);
	p_i2cx->cr1 = tmp;

	tmp = 0;

	tmp |= (rcc_get_apb1_freq() / 1000000);
	p_i2cx->cr2 = tmp;

	tmp = 0;

	tmp |= ((cfg->add << I2C_OAR1_ADD_7_1) | (0x1 << 14));
	p_i2cx->oar1 = tmp;

	tmp = 0;

	if (I2C_FREQ_SM == cfg->freq)
	{
		tmp |= (rcc_get_apb1_freq() / (2 * cfg->freq));
	}
	else
	{
		if (I2C_DUTY_2 == cfg->duty)
		{
			tmp |= (rcc_get_apb1_freq() / (3 * cfg->freq));
		}
		else
		{
			tmp |= (rcc_get_apb1_freq() / (25 * cfg->freq));
		}

		tmp |= ((cfg->duty << I2C_CCR_DUTY) | (0x1 << I2C_CCR_F_S));
	}

	p_i2cx->ccr = tmp;

	if (cfg->freq <= I2C_FREQ_SM)
	{
		tmp = (rcc_get_apb1_freq() / 1000000) + 1;

	}
	else
	{
		tmp = ((rcc_get_apb1_freq() * 300) / 1000000000) + 1;
	}

	p_i2cx->trise = (tmp & 0x3F);
}

void i2c_deinit(i2c_reg_map_t * p_i2cx)
{
	if (I2C1 == p_i2cx)
	{
		I2C1_CLOCK_DISABLE();
	}
	else if (I2C2 == p_i2cx)
	{
		I2C2_CLOCK_DISABLE();
	}
	else if (I2C3 == p_i2cx)
	{
		I2C3_CLOCK_DISABLE();
	}
	else
	{

	}

	i2c_clock_control(p_i2cx, DISABLE);
}

void i2c_master_transmit_data(i2c_reg_map_t * p_i2cx, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr)
{
	i2c_generate_start_condition(p_i2cx);

	while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_SB)));

	i2c_send_slave_address_write(p_i2cx, addr);

	while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_ADDR)));

	i2c_clear_addr_flag(p_i2cx);

	for (uint32_t i = 0; i < len; i++)
	{
		while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_TxE)));
		p_i2cx->dr = *(p_data + i);
	}

	while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_TxE)));

	while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_BTF)));

	if (I2C_SR_DISABLE == sr)
	{
		i2c_generate_stop_condition(p_i2cx);
	}
}

void i2c_master_receive_data(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr)
{
	i2c_generate_start_condition(p_i2cx);

	while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_SB)));

	i2c_send_slave_address_read(p_i2cx, addr);

	while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_ADDR)));

	if (len == 1)
	{
		i2c_ack_control(p_i2cx, DISABLE);

		if (I2C_SR_DISABLE == sr)
		{
			i2c_generate_stop_condition(p_i2cx);
		}

		i2c_clear_addr_flag(p_i2cx);

		while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_RxNE)));

		*p_data = p_i2cx->dr;
	}

	if (len > 1)
	{
		i2c_clear_addr_flag(p_i2cx);

		for (uint32_t i = len; i > 0; i--)
		{
			if (i == 2)
			{
				i2c_ack_control(p_i2cx, DISABLE);

				if (I2C_SR_DISABLE == sr)
				{
					i2c_generate_stop_condition(p_i2cx);
				}
			}

			while (FLAG_RESET == i2c_get_flag_status(p_i2cx, (0x1 << I2C_SR1_RxNE)));

			*(p_data + len - i) = p_i2cx->dr;
		}
	}

	if (I2C_ACK_ENABLE == cfg->ack)
	{
		i2c_ack_control(p_i2cx, ENABLE);
	}
}

uint8_t i2c_master_transmit_data_irq(i2c_reg_map_t * p_i2cx, i2c_txn_t * txn, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr)
{
	uint8_t state = txn->txrx;

	if (I2C_STATE_READY == state)
	{
		p_i2cx->cr2 |= (0x1 << I2C_CR2_ITERREN);

		p_i2cx->cr2 |= (0x1 << I2C_CR2_ITEVTEN);

		p_i2cx->cr2 |= (0x1 << I2C_CR2_ITBUFEN);

		i2c_generate_start_condition(p_i2cx);

		txn->p_tx_buf = p_data;

		txn->tx_len = len;

		txn->txrx = I2C_STATE_BUSY_TX;

		txn->addr = addr;

		txn->rx_size = len;

		txn->sr = sr;
	}

	return state;
}

uint8_t i2c_master_receive_data_irq(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, i2c_txn_t * txn, uint8_t * p_data, uint32_t len, uint8_t addr, uint8_t sr)
{
	uint8_t state = txn->txrx;

	if (I2C_STATE_READY == state)
	{
		p_i2cx->cr2 |= (0x1 << I2C_CR2_ITERREN);

		p_i2cx->cr2 |= (0x1 << I2C_CR2_ITEVTEN);

		p_i2cx->cr2 |= (0x1 << I2C_CR2_ITBUFEN);

		i2c_generate_start_condition(p_i2cx);

		txn->p_rx_buf = p_data;

		txn->rx_len = len;

		txn->txrx = I2C_STATE_BUSY_RX;

		txn->addr = addr;

		txn->rx_size = len;

		txn->sr = sr;
	}

	return state;
}

void i2c_reset_transmit_data(i2c_reg_map_t * p_i2cx, i2c_txn_t * txn)
{
	p_i2cx->cr2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	p_i2cx->cr2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	txn->p_tx_buf = NULL;

	txn->tx_len = 0;

	txn->txrx = I2C_STATE_READY;
}

void i2c_reset_receive_data(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, i2c_txn_t * txn)
{
	p_i2cx->cr2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	p_i2cx->cr2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	txn->p_rx_buf = NULL;

	txn->rx_len = 0;

	txn->txrx = I2C_STATE_READY;

	txn->rx_size = 0;

	if (I2C_ACK_ENABLE == cfg->ack)
	{
		i2c_ack_control(p_i2cx, ENABLE);
	}
}

void i2c_slave_transmit_data(i2c_reg_map_t * p_i2cx, uint8_t data)
{
	p_i2cx->dr = data;
}

uint8_t i2c_slave_receive_data(i2c_reg_map_t * p_i2cx)
{
	return ((uint8_t) p_i2cx->dr);
}

void i2c_irq_control_toggle(uint8_t position, uint8_t toggle)
{
	if (ENABLE == toggle)
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
	else
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
}

void i2c_irq_set_priority(uint8_t position, uint8_t priority)
{
	*(NVIC_IPR + (position / 4)) |= (priority << ((8 * (position % 4)) + 4));
}

void i2c_irq_ev_handler(i2c_reg_map_t * p_i2cx, i2c_cfg_t * cfg, i2c_txn_t * txn)
{
	if ((p_i2cx->cr2 & (0x1 << I2C_CR2_ITEVTEN)) && (p_i2cx->sr1 & (0x1 << I2C_SR1_SB)))
	{
		switch (txn->txrx)
		{
			case I2C_STATE_BUSY_TX:
				i2c_send_slave_address_write(p_i2cx, txn->addr);
			break;

			case I2C_STATE_BUSY_RX:
				i2c_send_slave_address_read(p_i2cx, txn->addr);
			break;

			default:
			break;
		}
	}

	if ((p_i2cx->cr2 & (0x1 << I2C_CR2_ITEVTEN)) && (p_i2cx->sr1 & (0x1 << I2C_SR1_ADDR)))
	{
		i2c_clear_addr_flag(p_i2cx);
	}

	if ((p_i2cx->cr2 & (0x1 << I2C_CR2_ITEVTEN)) && (p_i2cx->sr1 & (0x1 << I2C_SR1_STOPF)))
	{
		p_i2cx->cr1 |= 0x0;

		i2c_irq_callback(p_i2cx, txn, I2C_EV_STOPF);
	}

	if ((p_i2cx->cr2 & (0x1 << I2C_CR2_ITEVTEN)) && (p_i2cx->sr1 & (0x1 << I2C_SR1_BTF)))
	{
		if ((I2C_STATE_BUSY_TX == txn->txrx) && (p_i2cx->sr1 & (0x1 << I2C_SR1_TxE)) && (txn->tx_len == 0))
		{
			if (I2C_SR_DISABLE == txn->sr)
			{
				i2c_generate_stop_condition(p_i2cx);
			}

			i2c_reset_transmit_data(p_i2cx, txn);

			i2c_irq_callback(p_i2cx, txn, I2C_EV_TXE);
		}
	}

	if ((p_i2cx->cr2 & (0x1 << I2C_CR2_ITEVTEN)) && (p_i2cx->cr2 & (0x1 << I2C_CR2_ITBUFEN)) && (p_i2cx->sr1 & (0x1 << I2C_SR1_RxNE)))
	{
		if ((p_i2cx->sr2 & (0x1 << I2C_SR2_MSL)) && (I2C_STATE_BUSY_TX == txn->txrx))
		{
			if (txn->tx_len > 0)
			{
				p_i2cx->dr = *(txn->p_tx_buf);

				txn->tx_len--;

				txn->p_tx_buf++;

			}
		}
		else
		{
			if (p_i2cx->sr2 & (0x1 << I2C_SR2_TRA))
			{
				i2c_irq_callback(p_i2cx, txn, I2C_EV_STX);
			}
		}
	}

	if ((p_i2cx->cr2 & (0x1 << I2C_CR2_ITEVTEN)) && (p_i2cx->cr2 & (0x1 << I2C_CR2_ITBUFEN)) && (p_i2cx->sr1 & (0x1 << I2C_SR1_TxE)))
	{
		if ((p_i2cx->sr2 & (0x1 << I2C_SR2_MSL)) && (I2C_STATE_BUSY_RX == txn->txrx))
		{
			if (txn->rx_size == 1)
			{
				*txn->p_rx_buf = p_i2cx->dr;
				txn->rx_len--;
			}


			if (txn->rx_size> 1)
			{
				if (txn->rx_len == 2)
				{
					i2c_ack_control(p_i2cx, DISABLE);
				}

				*txn->p_rx_buf = p_i2cx->dr;
				txn->p_rx_buf++;
				txn->rx_len--;
			}

			if (txn->rx_len == 0)
			{
				if (I2C_SR_DISABLE == txn->sr)
				{
					i2c_generate_stop_condition(p_i2cx);
				}

				i2c_reset_receive_data(p_i2cx, cfg, txn);

				i2c_irq_callback(p_i2cx, txn, I2C_EV_RXNE);
			}
		}
		else
		{
			if (!(p_i2cx->sr2 & (0x1 << I2C_SR2_TRA)))
			{
				i2c_irq_callback(p_i2cx, txn, I2C_EV_SRX);
			}
		}
	}
}

__attribute__((weak)) void i2c_irq_callback(i2c_reg_map_t * p_i2cx, i2c_txn_t * txn, uint8_t event)
{

}

void i2c_peripheral_control(i2c_reg_map_t * p_i2cx, uint8_t toggle)
{
	if (ENABLE == toggle)
	{
		p_i2cx->cr1 |= (0x1 << I2C_CR1_PE);
	}
	else
	{
		p_i2cx->cr1 &= ~(0x1 << I2C_CR1_PE);
	}
}

void i2c_generate_start_condition(i2c_reg_map_t * p_i2cx)
{
	p_i2cx->cr1 |= (0x1 << I2C_CR1_START);
}

uint16_t i2c_get_flag_status(i2c_reg_map_t * p_i2cx, uint16_t flag)
{
	return (p_i2cx->sr1 & flag ? FLAG_SET : FLAG_RESET);
}

void i2c_send_slave_address_write(i2c_reg_map_t * p_i2cx, uint8_t addr)
{
	p_i2cx->dr = ((addr << 1) & ~(0x1));
}

void i2c_send_slave_address_read(i2c_reg_map_t * p_i2cx, uint8_t addr)
{
	p_i2cx->dr = ((addr << 1) | 0x1);
}

void i2c_clear_addr_flag(i2c_reg_map_t * p_i2cx)
{
	p_i2cx->sr1;
	p_i2cx->sr2;
}

void i2c_ack_control(i2c_reg_map_t * p_i2cx, uint8_t toggle)
{
	if (ENABLE == toggle)
	{
		p_i2cx->cr1 |= (0x1 << I2C_CR1_ACK);
	}
	else
	{
		p_i2cx->cr1 &= ~(0x1 << I2C_CR1_ACK);
	}
}

void i2c_generate_stop_condition(i2c_reg_map_t * p_i2cx)
{
	p_i2cx->cr1 |= (0x1 << I2C_CR1_STOP);
}
