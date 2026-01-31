#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32l0xx.h>

#ifndef DEBUG
#pragma message("This is release build")
#endif

#define USE_SLEEP_MODE

#define SLEEP_TIME_1S 	1000

#define BAD_LED			0xFFFFFFFF
#define MIN_LED			0
#define MAX_LED			13

#ifdef DEBUG
#define DEBUG_LED		1
#define SWCLK_LED		11
#define SWDIO_LED		10
#endif

#define BIT(x) (1 << (x))

#define poll(cond, attempts) ({ \
    int result = ERROR;            \
    for (int i = 0; i < (attempts); i++) { \
        if (cond) {             \
            result = SUCCESS;   \
            break;              \
        }                       \
    }                           \
    result;                     \
})

enum State {
	STATE_START,
	STATE_RUNNING_LED,
	STATE_SYMMETRY_RUNNING_LED,
#ifdef DEBUG
	STATE_DEBUG,
#endif
	STATE_LAST, // don't use it
};

static int state = STATE_START;
static bool is_lptim1_rdy = false;

/**
 * @brief Setup clocks to HSI16. HCLK has 1 MHz.
 */
int setup_clocks() {
	int ret = 0;

	RCC->CR = RCC_CR_HSIDIVEN | RCC_CR_HSION;
	ret = poll(RCC->CR & (RCC_CR_HSIRDY_Msk | RCC_CR_HSIDIVF_Msk), 10000);
	if (ret)
		return ret;

	RCC->CFGR = RCC_CFGR_STOPWUCK | RCC_CFGR_HPRE_DIV4 | RCC_CFGR_SW_HSI;
	ret = poll(RCC->CFGR & (RCC_CFGR_SWS_HSI), 10000);
	if (ret)
		return ret;

	RCC->IOPENR = RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;
	RCC->APB1ENR = RCC_APB1ENR_LPTIM1EN;
	RCC->APB2ENR = RCC_APB2ENR_DBGEN | RCC_APB2ENR_SYSCFGEN;

	RCC->IOPSMENR = RCC_IOPSMENR_GPIOASMEN;
	RCC->APB1SMENR = RCC_APB1SMENR_LPTIM1SMEN;
	RCC->APB2SMENR = RCC_APB2SMENR_DBGSMEN | RCC_APB2SMENR_SYSCFGSMEN;

	// TODO: Probably I should use LSI for the LPTIM
	RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_1; // HSI16 clock

	return SUCCESS;
}

/**
 * Group of GPIO:
 * * PB1 - input, interrupt
 * * PA0-PA14 - output, GPIO except in the debug mode:
 *   * PA13 - SWDIO
 *   * PA14 - SWCLK
 * * PC14, PC15 - output, GPIO
 */
int setup_gpio() {
	uint32_t val = 0;

	/**
	 * GPIOA setup
	 */
	val = GPIO_MODER_MODE0_0 \
		  | GPIO_MODER_MODE1_0 \
		  | GPIO_MODER_MODE2_0 \
		  | GPIO_MODER_MODE3_0 \
		  | GPIO_MODER_MODE4_0 \
		  | GPIO_MODER_MODE5_0 \
		  | GPIO_MODER_MODE6_0 \
		  | GPIO_MODER_MODE7_0 \
		  | GPIO_MODER_MODE9_0 \
		  | GPIO_MODER_MODE10_0;
#ifdef DEBUG
	val |= GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1;
#else
	val |= GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0;
#endif
	GPIOA->MODER = val;

#ifdef DEBUG
	/**
	 * According to the "27.3.3 Internal pull-up & pull-down on SWD pins":
	 * * SWDIO: input pull-up (0b01)
	 * * SWCLK: input pull-down (0b10)
	 */
	GPIOA->PUPDR = GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_1;
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL13 | GPIO_AFRH_AFSEL14);
#endif

	GPIOA->OTYPER = 0;  // push-pull
	GPIOA->OSPEEDR = 0; // low speed

	/**
	 * GPIOC setup
	 */
	GPIOC->MODER = GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0;
	GPIOC->OTYPER = 0;  // push-pull
	GPIOC->OSPEEDR = 0; // low speed

	/**
	 * GPIOB setup
	 */
	GPIOB->MODER &= ~GPIO_MODER_MODE1;
	GPIOB->PUPDR = GPIO_PUPDR_PUPD1_1;
	GPIOB->OTYPER = 0;  // push-pull
	GPIOB->OSPEEDR = 0; // low speed

	return SUCCESS;
}

int setup_interrupt() {
	// LPTIM1 interrupt
	NVIC_EnableIRQ(LPTIM1_IRQn);
	NVIC_SetPriority(LPTIM1_IRQn, 1);

	// PB1 interrupt
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PB;
	EXTI->IMR = EXTI_IMR_IM1;
	EXTI->RTSR = EXTI_RTSR_RT1;
	EXTI->FTSR &= ~EXTI_FTSR_FT1;

	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	return SUCCESS;
}

int setup_timer_lptim() {
	LPTIM1->IER = LPTIM_IER_CMPMIE;

	LPTIM1->CR &= ~LPTIM_CR_ENABLE;
	/**
	 * HSI16 is 16 MHz / 4 = 4 MHz
	 * 64 divider give us 62500 Hz for LPTIM. Because of the times is 16 bit
	 * we will can setup 1 second.
	 */
	LPTIM1->CFGR = LPTIM_CFGR_TIMOUT | (LPTIM_CFGR_PRESC_2 | LPTIM_CFGR_PRESC_1);
	LPTIM1->CR |= LPTIM_CR_ENABLE;

	return SUCCESS;
}

int start_timer_lptim(uint32_t ms) {
	if (ms > 1000)
		return ERROR;

	LPTIM1->CMP = (uint16_t)(ms * 62.5);
	LPTIM1->CR |= LPTIM_CR_SNGSTRT;

	return SUCCESS;
}

void delay(uint32_t val) {
	for (uint32_t i = 0; i < val; i++) {
		asm volatile ("nop");
	}
}

// TODO: Support Stop or Standby modes
void sleep_ms(uint32_t val) {
	start_timer_lptim(val);
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	asm volatile ("wfi");
	is_lptim1_rdy = false;
}

void LPTIM1_IRQHandler(void) {
	if (LPTIM1->ISR & LPTIM_ISR_CMPM_Msk) {
		LPTIM1->ICR |= LPTIM_ICR_CMPMCF;
		is_lptim1_rdy = true;
	}
}

void EXTI0_1_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PIF1) {
		// Simple debounce filter
		NVIC_DisableIRQ(EXTI0_1_IRQn);
		delay(20000);
		NVIC_EnableIRQ(EXTI0_1_IRQn);

		EXTI->PR |= EXTI_PR_PIF1;
		state++;
		if (state == STATE_LAST) {
			state = STATE_START;
		}
	}
}

GPIO_TypeDef * get_gpio_from_led(uint8_t led) {
	if ((led >=0) && (led < 12))
		return GPIOA;
	else if ((led >= 12) && (led < 14))
		return GPIOC;
	else
		return NULL;
}

uint32_t get_pin_from_led(uint8_t led) {
	switch (led) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			return led;
		case 8:
			return 9;
		case 9:
			return 10;
		case 10:
			return 13;
		case 11:
			return 14;
		case 12:
			return 14;
		case 13:
			return 15;
		default:
			return BAD_LED;
	}
}

int flash_led(uint8_t led) {
	GPIO_TypeDef *gpio = get_gpio_from_led(led);
	if (!gpio)
		return ERROR;
	uint32_t pin = get_pin_from_led(led);
	if (pin == BAD_LED)
		return ERROR;

	gpio->BSRR |= BIT(pin);			// BS - bit set
	sleep_ms(1);
	gpio->BSRR |= BIT(pin + 0x10);  // BR - bit reset
	sleep_ms(19);
	return SUCCESS;
}

int flash_two_leds(uint8_t led1, uint8_t led2) {
	GPIO_TypeDef *gpio1 = get_gpio_from_led(led1);
	if (!gpio1)
		return ERROR;
	GPIO_TypeDef *gpio2 = get_gpio_from_led(led2);
	if (!gpio2)
		return ERROR;

	uint32_t pin1 = get_pin_from_led(led1);
	if (pin1 == BAD_LED)
		return ERROR;
	uint32_t pin2 = get_pin_from_led(led2);
	if (pin2 == BAD_LED)
		return ERROR;

	gpio1->BSRR |= BIT(pin1);     		// BS - bit set
	sleep_ms(1);
	gpio1->BSRR |= BIT(pin1 + 0x10);  	// BR - bit reset
	sleep_ms(9);
	gpio2->BSRR |= BIT(pin2);			// BS - bit set
	sleep_ms(1);
	gpio2->BSRR |= BIT(pin2 + 0x10);  	// BR - bit reset
	sleep_ms(29);
	return SUCCESS;
}

void run_running_led_state() {
	for (int led = MIN_LED; led < MAX_LED + 1; led++) {
#ifdef DEBUG
		if ((led == SWCLK_LED) || (led == SWDIO_LED))
			continue;
#endif
		flash_led(led);
	}
}

void run_symmetry_running_led_state() {
	flash_led(1);
	flash_two_leds(2, 13);
	flash_two_leds(3, 12);
#ifndef DEBUG
	flash_two_leds(4, 11);
	flash_two_leds(5, 10);
#endif
	flash_two_leds(0, 9);
	flash_two_leds(6, 8);
	flash_led(7);
}

int main(void)
{
	int ret;

	ret = setup_clocks();
	if (ret)
		goto error_loop;

	ret = setup_timer_lptim();
	if (ret)
		goto error_loop;

	ret = setup_interrupt();
	if (ret)
		goto error_loop;

	/**
	 * Pins PA14 and PA13 are used as GPIO in release FW. But the pins
	 * are SWD pins also after reset. Without the delay we will not can
	 * update MC FW.
	 */
	sleep_ms(SLEEP_TIME_1S);
	ret = setup_gpio();
	if (ret)
		goto error_loop;

#ifdef DEBUG
	state = STATE_DEBUG;
#endif

	while (true) {
		switch (state) {
			case STATE_START:
				sleep_ms(SLEEP_TIME_1S);
				break;
			case STATE_RUNNING_LED:
				run_running_led_state();
				break;
			case STATE_SYMMETRY_RUNNING_LED:
				run_symmetry_running_led_state();
				break;
#ifdef DEBUG
			case STATE_DEBUG:
				flash_led(DEBUG_LED);
				break;
#endif
			default:
				goto error_loop;
		}
	}

error_loop:
	while (true) {
		delay(SLEEP_TIME_1S);
	}
}
