/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       stm32f4.discovery cc2420 implementation
 *
 * @author      Marco Scholtyssek <marco.scholtyssek056@stud.fh-dortmund.de>
 *
 * @}
 */

#include "board.h"
#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"
#include "posix_io.h"
#include <unistd.h>
//#include <cpu-conf.h>
#include "debug.h"
#include "ps.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "cc2420.h"
#include "transceiver.h"
#include "stdio.h"

#include "receiver.h"

#define TRANSCEIVER TRANSCEIVER_DEFAULT
#define PA3 3

static void testCommand(int argc, char **argv) {
	(void) argc; /* the function takes no arguments */
	(void) argv;
	printf("running testCommand()\r\n");
}

static const shell_command_t sc[] = { { "test", "does a test", testCommand }, {
NULL,
NULL, NULL } };

shell_t shell;
static volatile kernel_pid_t _relay_pid = KERNEL_PID_UNDEF;
char relay_stack[KERNEL_CONF_STACKSIZE_MAIN];
char appserver_stack[KERNEL_CONF_STACKSIZE_MAIN];

/*
 * Thread function
 */
static void *stm32f4_cc2420_start(void *arg) {
	while (1) {
		puts("thread is running!\r\n");
		LED_GREEN_ON;
		sleep(2);
//		thread_yield();
		LED_GREEN_OFF;
	}

	return NULL;
}

static void startCC2420_thread(void) {
	_relay_pid = thread_create(relay_stack, sizeof(relay_stack),
	PRIORITY_MAIN - 2, CREATE_STACKTEST, stm32f4_cc2420_start, NULL, "relay");

	DEBUG("ccn-lite relay on thread_id %" PRIkernel_pid "...\n", _relay_pid);
	puts("startCC2420_thread");
//    while(1){

//    	sleep(1);
//    }
}

/**
 * Wenn der PIN high ist, wird 1 zurueckgegeben,
 * ansonsten 0
 */
uint32_t get_gpio(GPIO_TypeDef *port, uint8_t pin){
//	return ((port->ODR & (1 << pin)) >> pin);
    if (port->MODER & (3 << (pin * 2))) {       /* if configured as output */
        return ((port->ODR & (1 << pin)) >> pin);          /* read output data register */
    } else {
        return (port->IDR & (1 << pin)) >> pin;          /* else read input data register */
    }

}

/**
 * Setzt den entsprechenden PIN auf high
 */
void set_gpio(GPIO_TypeDef *port, uint8_t pin){
		port->ODR |= (1 << pin);	// high
}


/**
 * Setzt den entsprechenden PIN auf low
 */
void reset_gpio(GPIO_TypeDef *port, uint8_t pin){
		port->ODR &= ~(1 << pin);	// low
}

void init_gpio_output(GPIO_TypeDef *port, uint8_t pin){
	port->MODER &= ~(2 << (2 * pin));           /* set pin to output mode */
	port->MODER |= (1 << (2 * pin));
	port->OTYPER &= ~(1 << pin);                /* set to push-pull configuration */
	port->OSPEEDR |= (3 << (2 * pin));          /* set to high speed */
	port->PUPDR &= ~(3 << (2 * pin));           /* configure push-pull resistors */
	port->PUPDR |= (GPIO_PULLUP << (2 * pin));
	port->ODR &= ~(1 << pin);                   /* set pin to low signal */
}

void init_gpio_input(GPIO_TypeDef *port, uint8_t pin){
    port->MODER &= ~(3 << (2 * pin));           /* configure pin as input */
    port->PUPDR &= ~(3 << (2 * pin));           /* configure push-pull resistors */
    port->PUPDR |= (GPIO_PULLDOWN << (2 * pin));
}

int main(void) {
	board_init();	// wird fuer den STM32F4 im reset_handler() schon ausgefuehrt
	puts("Hello World!\r\n");

//	uart_init();

//	GPIOD->BSRRL = (uint16_t)0x8000;	// blue LED
//	GPIOD->BSRRL = ((uint16_t)0x2000);	// orange LED

//	LD3_ON;	// orange
//	LD6_ON;	// blue

	puts("starting shell...\r\n");
	puts("  posix open\r\n");
	(void) posix_open(uart0_handler_pid, 0);

	(void) puts("Welcome to RIOT!\r\n");

	// start a thread
//	startCC2420_thread();

	puts("  shell init\r\n");
	shell_init(&shell, sc, UART0_BUFSIZE, uart0_readc, uart0_putc);
	puts("  shell run\r\n");

//	gpio_init_out(GPIO_0, GPIO_PULLUP);
//
//	gpio_write(GPIO_0, 1);

	/*
	 *  PA3 testen
	 */
	GPIO_0_CLKEN();	// Takt am Port einschalten
	uint32_t signal = 0x00;
	timex_t delay = timex_set(1, 0);

	// PA3 auf high
	int pin = 3;
//	init_gpio_output(GPIOA, PA3);
//
//	printf("set PA3\r\n");
//	set_gpio(GPIOA, PA3);
//	printf("PA3 %ld\r\n", get_gpio(GPIOA, PA3));
//
//	vtimer_sleep(delay);
//
//	printf("reset PA3\r\n");
//	reset_gpio(GPIOA, PA3);
////	GPIOA->BSRRH |= (1 << PA3);

//	printf("PA3 %ld\r\n", GPIOA->ODR & (1 << pin));

	vtimer_sleep(delay);

	printf("input PA3\r\n");
	init_gpio_input(GPIOA, 3);
	printf("PA3 %ld\r\n", get_gpio(GPIOA, 3));
//	while(1){
//		printf("PA3 %ld\r\n", get_gpio(GPIOA, PA3));
//		/* Flush stdout */
//		printf("\f");
//		vtimer_sleep(delay);
//	}
//
//	// TODO Schleife entfernen
//	while(1){
//
//	}

//	shell_run(&shell);
//	startCC2420_thread();
	char a = '\0';

	/**
	 * intialize SPI bus for cc2420 communication
	 */
//	spi_init_master(SPI_1, SPI_CONF_FIRST_RISING, SPI_SPEED_10MHZ);
	spi_init_master(SPI_1, SPI_CONF_FIRST_RISING, SPI_SPEED_400KHZ);

	/*
	 * initialize tranceiver
	 */
	transceiver_init(TRANSCEIVER_CC2420);
	kernel_pid_t transceiver_pid = transceiver_start();

//	int transceiver_pid = transceiver_start(); 	// start transceiver thread
//	DEBUG("Transceiver started on thread %d", transceiver_pid);
//


//	cc2420_init(KERNEL_PID_LAST + 1);
	printf("cc2420 transceiver_init with pid %d\r\n", transceiver_pid);
//	cc2420_init(transceiver_pid);

	config_receiver();
	while(1){
	}

	//	cc2420_radio_driver.send

	// create cc2420 packet and send it
	cc2420_packet_t cc2420_packet;

//	cc2420_packet_t *cc2420_rx_buffer = (cc2420_packet_t*) malloc(sizeof(cc2420_packet_t));
	uint8_t buf;
//	ieee802154_node_addr_t *dest = (ieee802154_node_addr_t*) malloc(sizeof(ieee802154_node_addr_t));
//	dest->long_addr = 0x1;
//	dest->pan.addr = 0x1;
//	dest->pan.id = 0x1;

//	ieee802154_frame_init(&cc2420_packet.frame, &buf);
	cc2420_send(&cc2420_packet);
//	cc2420_radio_driver.send(PACKET_KIND_DATA, dest, true, true, &buf, sizeof(buf));
	//(&cc2420_packet, dest, true, true &buf)


//	/* set channel to CCNL_CHAN */
//	msg_t mesg;
//	transceiver_command_t tcmd;
//	int32_t c = CCNL_DEFAULT_CHANNEL;
//	tcmd.transceivers = TRANSCEIVER;
//	tcmd.data = &c;
//	mesg.content.ptr = (char *) &tcmd;
//	mesg.type = SET_CHANNEL;
//	msg_send_receive(&mesg, &mesg, transceiver_pid);
//	if (c == -1) {
//		puts("[transceiver] Error setting/getting channel");
//	} else {
//		printf("[transceiver] Got channel: %" PRIi32 "\n", c);

//	ieee802154_frame_t frame;
//	uint8_t buf;
//	ieee802154_frame_init(&frame, &buf);

//	cc2420_load_tx_buf()
//	cc2420_transmit_tx_buf();

//	ieee802154_frame_get_hdr_len (&frame);

		while (1) {
//		LED_GREEN_ON;

			LD6_OFF;	// blue

			sleep(2);

			LD6_ON;		// blue

//		LED_GREEN_OFF;
//		printf("UART0 Bufsize: %d\r\n", UART0_BUFSIZE);
//		thread_print_all();
			sleep(2);

//		gpio_write(GPIO_0, 0);
//		spi_transfer_byte(SPI_0, 't',&a);
//		gpio_write(GPIO_0, 1);
			if (a != '\0') {
				sleep(2);
			}
		}

		return 0;
	}
