#include "tb_cxxrtl_io.h"

void main() {
	uint32_t holdoff = 127u;
	ahbscope_intr_init(holdoff);

	while(true) {
		usb_cdc_puts("test123\r\n");
		usleep(1000000);
	}
}