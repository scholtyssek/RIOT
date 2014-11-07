# name of your application
APPLICATION = stm32f4_cc2420

# If no BOARD is found in the environment, use this default:
BOARD ?= native
#BOARD ?= stm32f4discovery

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += posix
USEMODULE += uart0
#USEMODULE += uart1
#USEMODULE += transceiver
#USEMODULE += cc2420
USEMODULE += shell

# the uart implementation uses ringbuffer and therefore needs lib
#USEMODULE += lib

include $(RIOTBASE)/Makefile.include
