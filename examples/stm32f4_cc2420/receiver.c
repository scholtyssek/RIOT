#include "receiver.h"

void receiver_callback(void *buf, unsigned int len, int8_t rssi, uint8_t lqi, bool crc_ok){
	// debug callback data
	printf("callback buf: %d\r\n", buf);
	printf("callback rssi: %d\r\n", rssi);
	printf("callback lqi: %d\r\n", lqi);
	printf("callback crc_ok: %d\r\n", crc_ok);
}

void config_receiver(void){
		while (!cc2420_is_on()) {
			LED_RED_ON;
		}
		LED_RED_OFF;
		LED_GREEN_ON;

		uint16_t addr = 0x0001, new_addr;

		cc2420_radio_driver.set_address(addr);
		cc2420_radio_driver.set_long_address(addr);
		cc2420_radio_driver.set_channel(18);
		cc2420_radio_driver.set_pan_id(0xffff);
		cc2420_radio_driver.set_tx_power(-10);

		/*
		 * Callback Funktion fuer den cc2420 Treiber
		 * cc2420_radio_driver.set_receive_callback()
		 */
		cc2420_radio_driver.set_receive_callback(&receiver_callback);

	//	new_addr = cc2420_set_address(addr);
	//	cc2420_set_channel(18);
	//	cc2420_set_pan(0x1111);
	//	cc2420_set_pan(0xffff);
	//	cc2420_set_tx_power(-10);	// -10db?
		if(addr != new_addr){
			printf("cc2420 address could not be set to: %d\r\n", cc2420_radio_driver.get_address);
		}else{
			printf("cc2420 address set to: %d\r\n",  cc2420_radio_driver.get_address);
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
