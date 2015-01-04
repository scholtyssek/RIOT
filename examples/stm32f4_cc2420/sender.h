
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

void config_sender(void);
void send_test_data(void);
