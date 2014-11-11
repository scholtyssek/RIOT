/*
 * This is the driver implementation for the cc2420 radio module.
 * The configuration is:
 * 	10 MHz spi clock
 * 	PA0 - CS
 * 	PA1 -
 * 	PA2 -
 * 	PA3 -
 * 	PA4 -
 * 	PA5 -
 */

#include "cc2420_arch.h"
#include "periph_conf.h"
#include "stm32f407xx.h"
#include "periph/spi.h"
#include "hwtimer.h"
#include "radio_driver.h"

#define CC2420_CS_LOW       GPIO_0_PORT->ODR &= ~(1 << GPIO_0_PIN); 	/* PA0 */
#define CC2420_CS_HIGH     	GPIO_0_PORT->ODR |= (1 << GPIO_0_PIN); 		/* PA0 */
#define CC2420_RESET_PORT	GPIOA
#define CC2420_CS_PIN		0											/* PA0 */
#define CC2420_RESET_PIN	(1 << 1)									/* PA1 */
#define CC2420_CCA_PIN		(1 << 2)									/* PA2 */
#define CC2420_SFD_PIN		(1 << 3)									/* PA3 */
#define CC2420_FIFOP_PIN	(1 << 4)									/* PA4 */

void cc2420_spi_init(void){
	/* configure necessary pin */
	// TODO configure RCC and direction for all SPI pins

	spi_init_master(SPI_0, SPI_CONF_FIRST_RISING, SPI_SPEED_5MHZ);
}

uint8_t cc2420_get_cca(void){
	return GPIO_0_PORT->ODR & CC2420_CCA_PIN;
}

void core_panic(int crash_code, const char *message){
	// TODO
}

void cc2420_init_interrupts(void){
	// TODO
}

uint8_t cc2420_get_sfd(void){
	return GPIO_0_PORT->ODR & CC2420_SFD_PIN;
}

uint8_t cc2420_get_fifop(void){
	return GPIO_0_PORT->ODR & CC2420_FIFOP_PIN;
}

void cc2420_spi_select(void){
	CC2420_CS_LOW
}

void cc2420_spi_unselect(void){
	CC2420_CS_HIGH
}

void cc2420_reset(void){
	CC2420_RESET_PORT->BSRRH |= CC2420_RESET_PIN;	// low
	hwtimer_wait(500);
	CC2420_RESET_PORT->BSRRL |= CC2420_RESET_PIN;	// high
}

uint8_t cc2420_txrx(uint8_t c){
	char buf = '\0';
	spi_transfer_byte(SPI_0, c, &buf);
	return buf;
}
