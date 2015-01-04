#include "sender.h"

void config_sender(void){
	uint16_t addr = 0x0002, new_addr;

	cc2420_radio_driver.set_address(addr);
	cc2420_radio_driver.set_channel(18);
	cc2420_radio_driver.set_pan_id(0xffff);
	cc2420_radio_driver.set_tx_power(-10);

//	new_addr = cc2420_set_address(addr);
//	cc2420_set_channel(18);
//	cc2420_set_pan(0x1111);
//	cc2420_set_pan(0xffff);
//	cc2420_set_tx_power(-10);	// -10db?
	new_addr = cc2420_radio_driver.get_address();
	if (addr != new_addr) {
		printf("cc2420 address could not be set to: %d\r\n",
				new_addr);
	} else {
		printf("cc2420 address set to: %d\r\n",
				new_addr);
	}

//	int channel = cc2420_get_channel();
//	uint16_t address = cc2420_get_address();
//	uint16_t pan_id = cc2420_get_pan();
//	int tx_pwr = cc2420_get_tx_power();
	int channel = cc2420_radio_driver.get_channel;
	uint16_t address = cc2420_radio_driver.get_address();
	uint16_t pan_id = cc2420_radio_driver.get_pan_id();
	int tx_pwr = cc2420_radio_driver.get_tx_power();

	printf("cc2420 channel is: %d\r\n", channel);
	printf("cc2420 address is: %u\r\n", address);
	printf("cc2420 pan is: %u\r\n", pan_id);
	printf("cc2420 tx power: %d\r\n", tx_pwr);

	/* Flush stdout */
	printf("\f");
}

void send_test_data(void){
	uint8_t buf;
	timex_t delay_ = timex_set(0, 200);	// 10 ms
	radio_tx_status_t status;

	ieee802154_node_addr_t dest;
	dest.long_addr = 0x1;
	dest.pan.id = 0xffff;
	dest.pan.addr = 0xffff;

	while (1) {
		status = cc2420_radio_driver.send(PACKET_KIND_BEACON, dest, true, true,
				&buf, sizeof(buf));
		vtimer_sleep(delay_);
		printf("send status: %d\n\r", status);
	}
}
