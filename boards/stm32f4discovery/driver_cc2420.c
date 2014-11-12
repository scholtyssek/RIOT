/*
 * This is the driver implementation for the cc2420 radio module.
 * The IO configuration is:
 * 	10 MHz spi clock
 * 	PA0 - CS
 * 	PA1 - FIFO TODO
 * 	PA2 - CCA TODO
 * 	PA3 - SFD TODO
 * 	PA4 - RESETN TODO
 * 	PA5 - SCK
 * 	PA6 - SO
 *	PA7 - SI
 *	PA8 - FIFOP TODO
 *
 *	Not connected:
 *	VRegEn
 *
 */

#include <stdio.h>
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
#define CC2420_FIFO_PIN		(1 << 1)									/* PA1 */
#define CC2420_CCA_PIN		(1 << 2)									/* PA2 */
#define CC2420_SFD_PIN		(1 << 3)									/* PA3 */
#define CC2420_RESET_PIN	(1 << 4)									/* PA4 */

#define CC2420_FIFOP_PIN	(1 << 8)									/* PA8 */

void cc2420_spi_init(void){
	printf("cc2420_spi_init()\r\n");
	/* configure necessary pin */
	// TODO configure RCC and direction for all SPI pins

	spi_init_master(SPI_0, SPI_CONF_FIRST_RISING, SPI_SPEED_10MHZ);
}

uint8_t cc2420_get_cca(void){
	return GPIO_0_PORT->ODR & CC2420_CCA_PIN;
}

uint8_t cc2420_get_sfd(void){
	printf("cc2420_get_sfd\r\n");
	return GPIO_0_PORT->ODR & CC2420_SFD_PIN;
}

uint8_t cc2420_get_fifop(void){
	printf("cc2420_get_fifop\r\n");
	return GPIO_0_PORT->ODR & CC2420_FIFOP_PIN;
}

void cc2420_spi_select(void){
//	printf("cc2420_spi_select\r\n");
	CC2420_CS_LOW
}

void cc2420_spi_unselect(void){
//	printf("cc2420_spi_unselect\r\n");
	CC2420_CS_HIGH
}

void cc2420_reset(void){
	printf("cc2420_reset\r\n");
	CC2420_RESET_PORT->BSRRH |= CC2420_RESET_PIN;	// low
	hwtimer_wait(500);
	CC2420_RESET_PORT->BSRRL |= CC2420_RESET_PIN;	// high
}

uint8_t cc2420_txrx(uint8_t c){
	printf("cc2420_txrx\r\n");
	char buf = '\0';
	buf = spi_transfer_byte(SPI_0, c, &buf);
	return buf;
}


void core_panic(int crash_code, const char *message){
	printf("core_panic occurred. Code %d\r\n", crash_code);
	// TODO reboot system with software reset
}

void cc2420_init_interrupts(void){
	printf("cc2420_init_interrupts\r\n");
	// TODO
}

void cc2420_gdo0_enable(void){
	printf("cc2420_gdo0_enable\r\n");
	// TODO
}

/* Disable all interrupts */

void cc2420_gdo0_disable(void){
	printf("cc2420_gdo0_disable\r\n");
	// TODO
}

void cc2420_gdo2_enable(void){
	printf("cc2420_gdo2_enable");
	// TODO
}
void cc2420_gdo2_disable(void){
	printf("cc2420_gdo2_disable\r\n");
	// TODO
}

void cc2420_before_send(void){
	printf("cc2420_before_send\r\n");
	// TODO disable interrupts before send. see header file
}

void cc2420_after_send(void){
	printf("cc2420_after_send");
	// TODO reenable interrupts after send. see header file
}
