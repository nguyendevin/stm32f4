// gpio.h

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f4.h"

#define GPIO_PIN_0 			0
#define GPIO_PIN_1 			1
#define GPIO_PIN_2 			2
#define GPIO_PIN_3 			3
#define GPIO_PIN_4 			4
#define GPIO_PIN_5 			5
#define GPIO_PIN_6 			6
#define GPIO_PIN_7 			7
#define GPIO_PIN_8 			8
#define GPIO_PIN_9 			9
#define GPIO_PIN_10 		10
#define GPIO_PIN_11 		11
#define GPIO_PIN_12 		12
#define GPIO_PIN_13 		13
#define GPIO_PIN_14 		14
#define GPIO_PIN_15 		15

#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_RT		4
#define GPIO_MODE_FT		5
#define GPIO_MODE_RTFT		6

#define GPIO_TYPE_PP		0
#define GPIO_TYPE_OD		1

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VHIGH	3

#define GPIO_PUPD_NO		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2

typedef struct
{
	uint8_t pin;
	uint8_t mode;
	uint8_t type;
	uint8_t speed;
	uint8_t pupd;
	uint8_t altfn;
} gpio_cfg_t;

void gpio_clock_enable(gpio_reg_map_t * p_gpiox);
void gpio_clock_disable(gpio_reg_map_t * p_gpiox);

void gpio_init(gpio_reg_map_t * p_gpiox, const gpio_cfg_t * cfg);
void gpio_deinit(gpio_reg_map_t * p_gpiox);

uint8_t gpio_read_pin(const gpio_reg_map_t * p_gpiox, uint8_t pin);
uint16_t gpio_read_port(const gpio_reg_map_t * p_gpiox);
void gpio_write_pin(gpio_reg_map_t * p_gpiox, uint8_t pin, uint8_t data);
void gpio_write_port(gpio_reg_map_t * p_gpiox, uint16_t data);
void gpio_toggle_pin(gpio_reg_map_t * p_gpiox, uint8_t pin);

void gpio_irq_enable(uint8_t position);
void gpio_irq_disable(uint8_t position);
void gpio_irq_set_priority(uint8_t position, uint8_t priority);
void gpio_irq_handler(uint8_t pin);

#endif /* GPIO_H_ */
