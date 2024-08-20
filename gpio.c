// gpio.c

#include "gpio.h"

void gpio_clock_enable(gpio_reg_map_t * p_gpiox)
{
	if (GPIOA == p_gpiox)
	{
		GPIOA_CLOCK_ENABLE();
	}
	else if (GPIOB == p_gpiox)
	{
		GPIOB_CLOCK_ENABLE();
	}
	else if (GPIOC == p_gpiox)
	{
		GPIOC_CLOCK_ENABLE();
	}
	else if (GPIOD == p_gpiox)
	{
		GPIOD_CLOCK_ENABLE();
	}
	else if (GPIOE == p_gpiox)
	{
		GPIOE_CLOCK_ENABLE();
	}
	else if (GPIOF == p_gpiox)
	{
		GPIOF_CLOCK_ENABLE();
	}
	else if (GPIOG == p_gpiox)
	{
		GPIOG_CLOCK_ENABLE();
	}
	else if (GPIOH == p_gpiox)
	{
		GPIOH_CLOCK_ENABLE();
	}
	else if (GPIOI == p_gpiox)
	{
		GPIOI_CLOCK_ENABLE();
	}
	else
	{

	}
}

void gpio_clock_disable(gpio_reg_map_t * p_gpiox)
{
	if (GPIOA == p_gpiox)
	{
		GPIOA_CLOCK_DISABLE();
	}
	else if (GPIOB == p_gpiox)
	{
		GPIOB_CLOCK_DISABLE();
	}
	else if (GPIOC == p_gpiox)
	{
		GPIOC_CLOCK_DISABLE();
	}
	else if (GPIOD == p_gpiox)
	{
		GPIOD_CLOCK_DISABLE();
	}
	else if (GPIOE == p_gpiox)
	{
		GPIOE_CLOCK_DISABLE();
	}
	else if (GPIOF == p_gpiox)
	{
		GPIOF_CLOCK_DISABLE();
	}
	else if (GPIOG == p_gpiox)
	{
		GPIOG_CLOCK_DISABLE();
	}
	else if (GPIOH == p_gpiox)
	{
		GPIOH_CLOCK_DISABLE();
	}
	else if (GPIOI == p_gpiox)
	{
		GPIOI_CLOCK_DISABLE();
	}
	else
	{

	}
}

void gpio_init(gpio_reg_map_t * p_gpiox, const gpio_cfg_t * cfg)
{
	gpio_clock_enable(p_gpiox);

	p_gpiox->moder &= ~(0x3 << (2 * cfg->pin));

	if (GPIO_MODE_ANALOG >= cfg->mode)
	{
		p_gpiox->moder |= (cfg->mode << (2 * cfg->pin));
	}
	else
	{
		SYSCFG_CLOCK_ENABLE();

		SYSCFG->exticr[cfg->pin / 4] &= ~(0xF << (4 * (cfg->pin % 4)));
		SYSCFG->exticr[cfg->pin / 4] |= (GPIOX_TO_PORT(p_gpiox) << (4 * (cfg->pin % 4)));

		EXTI->imr |= (0x1 << cfg->pin);

		switch (cfg->mode)
		{
			case GPIO_MODE_RT:
				EXTI->rtsr |= (0x1 << cfg->pin);
				EXTI->ftsr &= ~(0x1 << cfg->pin);
			break;

			case GPIO_MODE_FT:
				EXTI->rtsr &= ~(0x1 << cfg->pin);
				EXTI->ftsr |= (0x1 << cfg->pin);
			break;

			case GPIO_MODE_RTFT:
				EXTI->rtsr |= (0x1 << cfg->pin);
				EXTI->ftsr |= (0x1 << cfg->pin);
			break;

			default:
			break;
		}
	}

	if ((GPIO_MODE_OUTPUT == cfg->mode) | (GPIO_MODE_ALTFN == cfg->mode))
	{
		p_gpiox->otyper &= ~(0x1 << (cfg->pin));
		p_gpiox->otyper |= (cfg->type << (cfg->pin));

		p_gpiox->ospeedr &= ~(0x3 << (2 * cfg->pin));
		p_gpiox->ospeedr |= (cfg->speed << (2 * cfg->pin));
	}

	p_gpiox->pupdr &= ~(0x3 << (2 * cfg->pin));
	p_gpiox->pupdr |= (cfg->pupd << (2 * cfg->pin));

	if (GPIO_MODE_ALTFN == cfg->mode)
	{
		p_gpiox->afr[cfg->pin / 8] &= ~(0xF << (4 * (cfg->pin % 8)));
		p_gpiox->afr[cfg->pin / 8] |= (cfg->altfn << (4 * (cfg->pin % 8)));
	}
}

void gpio_deinit(gpio_reg_map_t * p_gpiox)
{
	if (GPIOA == p_gpiox)
	{
		GPIOA_RESET();
	}
	else if (GPIOB == p_gpiox)
	{
		GPIOB_RESET();
	}
	else if (GPIOC == p_gpiox)
	{
		GPIOC_RESET();
	}
	else if (GPIOD == p_gpiox)
	{
		GPIOD_RESET();
	}
	else if (GPIOE == p_gpiox)
	{
		GPIOE_RESET();
	}
	else if (GPIOF == p_gpiox)
	{
		GPIOF_RESET();
	}
	else if (GPIOG == p_gpiox)
	{
		GPIOG_RESET();
	}
	else if (GPIOH == p_gpiox)
	{
		GPIOH_RESET();
	}
	else if (GPIOI == p_gpiox)
	{
		GPIOI_RESET();
	}
	else
	{

	}

	gpio_clock_disable(p_gpiox);
}

uint8_t gpio_read_pin(const gpio_reg_map_t * p_gpiox, uint8_t pin)
{
	return ((uint8_t) (p_gpiox->idr >> pin) & 0x1);
}

uint16_t gpio_read_port(const gpio_reg_map_t * p_gpiox)
{
	return ((uint16_t) p_gpiox->idr);
}

void gpio_write_pin(gpio_reg_map_t * p_gpiox, uint8_t pin, uint8_t data)
{
	if (data)
	{
		p_gpiox->odr |= (0x1 << pin);
	}
	else
	{
		p_gpiox->odr &= ~(0x1 << pin);
	}
}

void gpio_write_port(gpio_reg_map_t * p_gpiox, uint16_t data)
{
	p_gpiox->odr = data;
}

void gpio_toggle_pin(gpio_reg_map_t * p_gpiox, uint8_t pin)
{
	p_gpiox->odr ^= (0x1 << pin);
}

void gpio_irq_enable(uint8_t position)
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

void gpio_irq_disable(uint8_t position)
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

void gpio_irq_set_priority(uint8_t position, uint8_t priority)
{
	*(NVIC_IPR + (position / 4)) |= (priority << ((8 * (position % 4)) + 4));
}

void gpio_irq_handler(uint8_t pin)
{
	if (EXTI->pr & (0x1 << pin))
	{
		EXTI->pr |= (0x1 << pin);
	}
}
