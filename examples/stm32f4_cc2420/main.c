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

static void testCommand(int argc, char **argv) {
	(void) argc; /* the function takes no arguments */
	(void) argv;
	printf("runniing testCommand()\n");
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
	while(1){
		puts("thread is running!");
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

int main(void) {
//	uart_init()
	puts("Hello World!");

	board_init();
//	GPIOD->BSRRL = (uint16_t)0x8000;	// blue LED
//	GPIOD->BSRRL = ((uint16_t)0x2000);	// orange LED

//	LD3_ON;	// orange
//	LD6_ON;	// blue

	puts("starting shell...");
	puts("  posix open");
	(void) posix_open(uart0_handler_pid, 0);

	(void) puts("Welcome to RIOT!");

	// start a thread
	startCC2420_thread();


	puts("  shell init");
	shell_init(&shell, sc, UART0_BUFSIZE, uart0_readc, uart0_putc);
	puts("  shell run");
//	shell_run(&shell);
//	startCC2420_thread();

	while (1) {
//		LED_GREEN_ON;
		LED_RED_ON;
		sleep(2);
//		LED_GREEN_OFF;
		LED_RED_OFF;
		printf("UART0 Bufsize: %d\n", UART0_BUFSIZE);
//		thread_print_all();
		sleep(2);
	}

	return 0;
}
