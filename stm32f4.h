// stm32f4.h

#ifndef STM32F4_H_
#define STM32F4_H_

#include <stdint.h>

#define RCC_BASE_ADDRESS 	0x40023800

#define GPIOA_BASE_ADDRESS 	0x40020000
#define GPIOB_BASE_ADDRESS 	0x40020400
#define GPIOC_BASE_ADDRESS 	0x40020800
#define GPIOD_BASE_ADDRESS 	0x40020C00
#define GPIOE_BASE_ADDRESS 	0x40021000
#define GPIOF_BASE_ADDRESS 	0x40021400
#define GPIOG_BASE_ADDRESS 	0x40021800
#define GPIOH_BASE_ADDRESS 	0x40021C00
#define GPIOI_BASE_ADDRESS 	0x40022000

#define SYSCFG_BASE_ADDRESS	0x40013800

#define EXTI_BASE_ADDRESS	0x40013C00

#define I2C1_BASE_ADDRESS 	0x40005400
#define I2C2_BASE_ADDRESS 	0x40005800
#define I2C3_BASE_ADDRESS 	0x40005C00

#define SPI1_BASE_ADDRESS 	0x40013000
#define SPI2_BASE_ADDRESS 	0x40003800
#define SPI3_BASE_ADDRESS 	0x40003C00

#define USART1_BASE_ADDRESS 0x40011000
#define USART2_BASE_ADDRESS 0x40004400
#define USART3_BASE_ADDRESS 0x40004800
#define UART4_BASE_ADDRESS 	0x40004C00
#define UART5_BASE_ADDRESS 	0x40005000
#define USART6_BASE_ADDRESS 0x40011400

typedef struct
{
	volatile uint32_t cr;
	volatile uint32_t pllcfgr;
	volatile uint32_t cfgr;
	volatile uint32_t cir;
	volatile uint32_t ahb1rstr;
	volatile uint32_t ahb2rstr;
	volatile uint32_t ahb3rstr;
	uint32_t 		  reserved0;
	volatile uint32_t apb1rstr;
	volatile uint32_t apb2rstr;
	uint32_t 		  reserved1[2];
	volatile uint32_t ahb1enr;
	volatile uint32_t ahb2enr;
	volatile uint32_t ahb3enr;
	uint32_t 		  reserved2;
	volatile uint32_t apb1enr;
	volatile uint32_t apb2enr;
	uint32_t 		  reserved3[2];
	volatile uint32_t ahb1lpenr;
	volatile uint32_t ahb2lpenr;
	volatile uint32_t ahb3lpenr;
	uint32_t 		  reserved4;
	volatile uint32_t apb1lpenr;
	volatile uint32_t apb2lpenr;
	uint32_t 		  reserved5[2];
	volatile uint32_t bdcr;
	volatile uint32_t csr;
	uint32_t 		  reserved6[2];
	volatile uint32_t sscgr;
	volatile uint32_t plli2scfgr;
} rcc_reg_map_t;

typedef struct
{
	volatile uint32_t moder;
	volatile uint32_t otyper;
	volatile uint32_t ospeedr;
	volatile uint32_t pupdr;
	volatile uint32_t idr;
	volatile uint32_t odr;
	volatile uint32_t bsrr;
	volatile uint32_t lckr;
	volatile uint32_t afr[2];
} gpio_reg_map_t;

typedef struct
{
	volatile uint32_t memrmp;
	volatile uint32_t pmc;
	volatile uint32_t exticr[4];
	uint32_t 		  reserved[2];
	volatile uint32_t cmpcr;
} syscfg_reg_map_t;

typedef struct
{
	volatile uint32_t imr;
	volatile uint32_t emr;
	volatile uint32_t rtsr;
	volatile uint32_t ftsr;
	volatile uint32_t swier;
	volatile uint32_t pr;
} exti_reg_map_t;

typedef struct
{
	volatile uint32_t cr1;
	volatile uint32_t cr2;
	volatile uint32_t oar1;
	volatile uint32_t oar2;
	volatile uint32_t dr;
	volatile uint32_t sr1;
	volatile uint32_t sr2;
	volatile uint32_t ccr;
	volatile uint32_t trise;
	volatile uint32_t fltr;
} i2c_reg_map_t;

typedef struct
{
	volatile uint32_t cr1;
	volatile uint32_t cr2;
	volatile uint32_t sr;
	volatile uint32_t dr;
	volatile uint32_t crcpr;
	volatile uint32_t rxcrcr;
	volatile uint32_t txcrcr;
	volatile uint32_t i2scfgr;
	volatile uint32_t i2spr;
} spi_reg_map_t;

typedef struct
{
	volatile uint32_t sr;
	volatile uint32_t dr;
	volatile uint32_t brr;
	volatile uint32_t cr1;
	volatile uint32_t cr2;
	volatile uint32_t cr3;
	volatile uint32_t gtpr;
} usart_reg_map_t;

#define RCC						((rcc_reg_map_t *) RCC_BASE_ADDRESS)

#define GPIOA					((gpio_reg_map_t *) GPIOA_BASE_ADDRESS)
#define GPIOB					((gpio_reg_map_t *) GPIOB_BASE_ADDRESS)
#define GPIOC					((gpio_reg_map_t *) GPIOC_BASE_ADDRESS)
#define GPIOD					((gpio_reg_map_t *) GPIOD_BASE_ADDRESS)
#define GPIOE					((gpio_reg_map_t *) GPIOE_BASE_ADDRESS)
#define GPIOF					((gpio_reg_map_t *) GPIOF_BASE_ADDRESS)
#define GPIOG					((gpio_reg_map_t *) GPIOG_BASE_ADDRESS)
#define GPIOH					((gpio_reg_map_t *) GPIOH_BASE_ADDRESS)
#define GPIOI					((gpio_reg_map_t *) GPIOI_BASE_ADDRESS)

#define SYSCFG					((syscfg_reg_map_t *) SYSCFG_BASE_ADDRESS)

#define NVIC_ISER0				((volatile uint32_t *) 0xE000E100)
#define NVIC_ISER1				((volatile uint32_t *) 0xE000E104)
#define NVIC_ISER2				((volatile uint32_t *) 0xE000E108)
#define NVIC_ISER3				((volatile uint32_t *) 0xE000E10C)
#define NVIC_ISER4				((volatile uint32_t *) 0xE000E110)
#define NVIC_ISER5				((volatile uint32_t *) 0xE000E114)
#define NVIC_ISER6				((volatile uint32_t *) 0xE000E118)
#define NVIC_ISER7				((volatile uint32_t *) 0xE000E11C)

#define NVIC_ICER0				((volatile uint32_t *) 0xE000E180)
#define NVIC_ICER1				((volatile uint32_t *) 0xE000E184)
#define NVIC_ICER2				((volatile uint32_t *) 0xE000E188)
#define NVIC_ICER3				((volatile uint32_t *) 0xE000E18C)
#define NVIC_ICER4				((volatile uint32_t *) 0xE000E190)
#define NVIC_ICER5				((volatile uint32_t *) 0xE000E194)
#define NVIC_ICER6				((volatile uint32_t *) 0xE000E198)
#define NVIC_ICER7				((volatile uint32_t *) 0xE000E19C)

#define NVIC_IPR 				((volatile uint32_t *) 0xE000E400)

#define EXTI					((exti_reg_map_t *) EXTI_BASE_ADDRESS)

#define I2C1					((i2c_reg_map_t *) I2C1_BASE_ADDRESS)
#define I2C2					((i2c_reg_map_t *) I2C2_BASE_ADDRESS)
#define I2C3					((i2c_reg_map_t *) I2C3_BASE_ADDRESS)

#define SPI1					((spi_reg_map_t *) SPI1_BASE_ADDRESS)
#define SPI2					((spi_reg_map_t *) SPI2_BASE_ADDRESS)
#define SPI3					((spi_reg_map_t *) SPI3_BASE_ADDRESS)

#define USART1					((usart_reg_map_t *) USART1_BASE_ADDRESS)
#define USART2					((usart_reg_map_t *) USART2_BASE_ADDRESS)
#define USART3					((usart_reg_map_t *) USART3_BASE_ADDRESS)
#define UART4					((usart_reg_map_t *) UART4_BASE_ADDRESS)
#define UART5					((usart_reg_map_t *) UART5_BASE_ADDRESS)
#define USART6					((usart_reg_map_t *) USART6_BASE_ADDRESS)

#define I2C_CR1_PE 				0
#define I2C_CR1_SMBUS 			1
#define I2C_CR1_SMBTYPE 		3
#define I2C_CR1_ENARP 			4
#define I2C_CR1_ENPEC 			5
#define I2C_CR1_ENGC 			6
#define I2C_CR1_NOSTRETCH 		7
#define I2C_CR1_START 			8
#define I2C_CR1_STOP 			9
#define I2C_CR1_ACK 			10
#define I2C_CR1_POS 			11
#define I2C_CR1_PEC 			12
#define I2C_CR1_ALERT 			13
#define I2C_CR1_SWRST 			15

#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD_7_1		1
#define I2C_OAR1_ADD_9_8		8
#define I2C_OAR1_ADDMODE		15

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

#define I2C_SR2_MSL 			0
#define I2C_SR2_BUSY 			1
#define I2C_SR2_TRA 			2
#define I2C_SR2_GENCALL 		4
#define I2C_SR2_SMBDEFAULT 		5
#define I2C_SR2_SMBHOST 		6
#define I2C_SR2_DUALF 			7
#define I2C_SR2_PEC 			8

#define I2C_CCR_CCR 			0
#define I2C_CCR_DUTY 			14
#define I2C_CCR_F_S 			15

#define SPI_CR1_CPHA 			0
#define SPI_CR1_CPOL 			1
#define SPI_CR1_MSTR 			2
#define SPI_CR1_BR 				3
#define SPI_CR1_SPE 			6
#define SPI_CR1_LSBFIRST 		7
#define SPI_CR1_SSI 			8
#define SPI_CR1_SSM 			9
#define SPI_CR1_RXONLY 			10
#define SPI_CR1_DFF 			11
#define SPI_CR1_CRCNEXT 		12
#define SPI_CR1_CRCEN 			13
#define SPI_CR1_BIDIOE 			14
#define SPI_CR1_BIDIMODE 		15

#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

#define USART_CR2_ADD   		0
#define USART_CR2_LBDL   		5
#define USART_CR2_LBDIE  		6
#define USART_CR2_LBCL   		8
#define USART_CR2_CPHA   		9
#define USART_CR2_CPOL   		10
#define USART_CR2_CLKEN   		11
#define USART_CR2_STOP   		12
#define USART_CR2_LINEN   		14

#define USART_CR3_EIE   		0
#define USART_CR3_IREN   		1
#define USART_CR3_IRLP  		2
#define USART_CR3_HDSEL   		3
#define USART_CR3_NACK   		4
#define USART_CR3_SCEN   		5
#define USART_CR3_DMAR  		6
#define USART_CR3_DMAT   		7
#define USART_CR3_RTSE   		8
#define USART_CR3_CTSE   		9
#define USART_CR3_CTSIE   		10
#define USART_CR3_ONEBIT   		11

#define USART_SR_PE        		0
#define USART_SR_FE        		1
#define USART_SR_NF        		2
#define USART_SR_ORE       		3
#define USART_SR_IDLE       	4
#define USART_SR_RXNE        	5
#define USART_SR_TC        		6
#define USART_SR_TXE        	7
#define USART_SR_LBD        	8
#define USART_SR_CTS        	9

#define GPIOA_RESET()			(RCC->ahb1rstr |= (1 << 0))
#define GPIOB_RESET()			(RCC->ahb1rstr |= (1 << 1))
#define GPIOC_RESET()			(RCC->ahb1rstr |= (1 << 2))
#define GPIOD_RESET()			(RCC->ahb1rstr |= (1 << 3))
#define GPIOE_RESET()			(RCC->ahb1rstr |= (1 << 4))
#define GPIOF_RESET()			(RCC->ahb1rstr |= (1 << 5))
#define GPIOG_RESET()			(RCC->ahb1rstr |= (1 << 6))
#define GPIOH_RESET()			(RCC->ahb1rstr |= (1 << 7))
#define GPIOI_RESET()			(RCC->ahb1rstr |= (1 << 8))

#define SYSCFG_RESET()			(RCC->apb2rstr |= (1 << 14))

#define I2C1_RESET()			(RCC->apb1rstr |= (1 << 21))
#define I2C2_RESET()			(RCC->apb1rstr |= (1 << 22))
#define I2C3_RESET()			(RCC->apb1rstr |= (1 << 23))

#define SPI1_RESET()			(RCC->apb2rstr |= (1 << 12))
#define SPI2_RESET()			(RCC->apb1rstr |= (1 << 14))
#define SPI3_RESET()			(RCC->apb1rstr |= (1 << 15))

#define USART1_RESET()			(RCC->apb2rstr |= (1 << 4))
#define USART2_RESET()			(RCC->apb1rstr |= (1 << 17))
#define USART3_RESET()			(RCC->apb1rstr |= (1 << 18))
#define UART4_RESET()			(RCC->apb1rstr |= (1 << 19))
#define UART5_RESET()			(RCC->apb1rstr |= (1 << 20))
#define USART6_RESET()			(RCC->apb2rstr |= (1 << 5))

#define GPIOA_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 0))
#define GPIOB_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 1))
#define GPIOC_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 2))
#define GPIOD_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 3))
#define GPIOE_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 4))
#define GPIOF_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 5))
#define GPIOG_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 6))
#define GPIOH_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 7))
#define GPIOI_CLOCK_ENABLE()	(RCC->ahb1enr |= (1 << 8))

#define SYSCFG_CLOCK_ENABLE()	(RCC->apb2enr |= (1 << 14))

#define I2C1_CLOCK_ENABLE()		(RCC->apb1enr |= (1 << 21))
#define I2C2_CLOCK_ENABLE()		(RCC->apb1enr |= (1 << 22))
#define I2C3_CLOCK_ENABLE()		(RCC->apb1enr |= (1 << 23))

#define SPI1_CLOCK_ENABLE()		(RCC->apb2enr |= (1 << 12))
#define SPI2_CLOCK_ENABLE()		(RCC->apb1enr |= (1 << 14))
#define SPI3_CLOCK_ENABLE()		(RCC->apb1enr |= (1 << 15))

#define USART1_CLOCK_ENABLE()	(RCC->apb2enr |= (1 << 4))
#define USART2_CLOCK_ENABLE()	(RCC->apb1enr |= (1 << 17))
#define USART3_CLOCK_ENABLE()	(RCC->apb1enr |= (1 << 18))
#define UART4_CLOCK_ENABLE()	(RCC->apb1enr |= (1 << 19))
#define UART5_CLOCK_ENABLE()	(RCC->apb1enr |= (1 << 20))
#define USART6_CLOCK_ENABLE()	(RCC->apb2enr |= (1 << 5))

#define GPIOA_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 0))
#define GPIOB_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 1))
#define GPIOC_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 2))
#define GPIOD_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 3))
#define GPIOE_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 4))
#define GPIOF_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 5))
#define GPIOG_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 6))
#define GPIOH_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 7))
#define GPIOI_CLOCK_DISABLE()	(RCC->ahb1enr &= ~(1 << 8))

#define SYSCFG_CLOCK_DISABLE()	(RCC->apb2enr &= ~(1 << 14))

#define I2C1_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 21))
#define I2C2_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 22))
#define I2C3_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 23))

#define SPI1_CLOCK_DISABLE()	(RCC->apb2enr &= ~(1 << 12))
#define SPI2_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 14))
#define SPI3_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 15))

#define USART1_CLOCK_DISABLE()	(RCC->apb2enr &= ~(1 << 4))
#define USART2_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 17))
#define USART3_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 18))
#define UART4_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 19))
#define UART5_CLOCK_DISABLE()	(RCC->apb1enr &= ~(1 << 20))
#define USART6_CLOCK_DISABLE()	(RCC->apb2enr &= ~(1 << 5))

#define GPIOX_TO_PORT(x) 		((GPIOA == x) ? 0 : \
								 (GPIOB == x) ? 1 : \
								 (GPIOC == x) ? 2 : \
								 (GPIOD == x) ? 3 : \
								 (GPIOE == x) ? 4 : \
								 (GPIOF == x) ? 5 : \
								 (GPIOG == x) ? 6 : \
								 (GPIOH == x) ? 7 : \
								 (GPIOI == x) ? 8 : 0)

#define EXTI0_IRQ_POSITION 		6
#define EXTI1_IRQ_POSITION 		7
#define EXTI2_IRQ_POSITION 		8
#define EXTI3_IRQ_POSITION 		9
#define EXTI4_IRQ_POSITION 		10
#define EXTI9_5_IRQ_POSITION 	23
#define EXTI15_10_IRQ_POSITION 	40

#define I2C1_EV_IRQ_POSITION	31
#define I2C1_ER_IRQ_POSITION	32
#define I2C2_EV_IRQ_POSITION	33
#define I2C2_ER_IRQ_POSITION	34
#define I2C3_EV_IRQ_POSITION 	72
#define I2C3_ER_IRQ_POSITION	73

#define SPI1_IRQ_POSITION 		35
#define SPI2_IRQ_POSITION 		36
#define SPI3_IRQ_POSITION 		51

#define USART1_IRQ_POSITION	    37
#define USART2_IRQ_POSITION	    38
#define USART3_IRQ_POSITION	    39
#define UART4_IRQ_POSITION	    52
#define UART5_IRQ_POSITION	    53
#define USART6_IRQ_POSITION	    71

#define DISABLE					0
#define ENABLE					1
#define FLAG_RESET				0
#define FLAG_SET				1

#endif /* STM32F4_H_ */
