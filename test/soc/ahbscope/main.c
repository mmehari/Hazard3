#include "tb_cxxrtl_io.h"

#define AHBSCOPE_BASE      0x60000000u
#define AHBSCOPE_STATUS    (*(volatile uint32_t *)(AHBSCOPE_BASE + 0x0u))
#define AHBSCOPE_DATA      (*(volatile uint32_t *)(AHBSCOPE_BASE + 0x4u))

#define AHBSCOPE_PRIMED    0x10000000u
#define AHBSCOPE_TRIGGERED 0x20000000u
#define AHBSCOPE_STOPPED   0x40000000u
#define AHBSCOPE_LGLEN(x)  (((x) >> 20) & 0x1fu)
#define AHBSCOPE_HOLDOFF(x) ((x) & 0x000fffffu)

static void uart_puts_ila(uint32_t value) {
	char buf[5];

	buf[0] = ((value>>0)  & 0x7f) | 0x80;
	buf[1] = ((value>>7)  & 0x7f);
	buf[2] = ((value>>14) & 0x7f);
	buf[3] = ((value>>21) & 0x7f);
	buf[4] = ((value>>28) & 0x0f);

	uart_puts_data(buf, 5);
}

void main() {
	uint32_t status;
	uint32_t trigger_idx;
	uint32_t nsamples;

	uart_init();

	while(true) {

		// bit[31]=0 requests a reset/arm and bits[19:0] program holdoff
		AHBSCOPE_STATUS = 10;

		do {
			status = AHBSCOPE_STATUS;
		} while (!(status & AHBSCOPE_PRIMED));

		usb_cdc_puts("test123\r\n");

		do {
			status = AHBSCOPE_STATUS;
		} while (!(status & AHBSCOPE_TRIGGERED));

		do {
			status = AHBSCOPE_STATUS;
		} while (!(status & AHBSCOPE_STOPPED));

		nsamples = 1u << AHBSCOPE_LGLEN(status);
		trigger_idx = nsamples - AHBSCOPE_HOLDOFF(status) - 1u;

		// The scope returns data one read behind the address phase, so discard
		// the final status word before streaming captured samples.
		(void)AHBSCOPE_DATA;
		for (uint32_t i = 0; i < nsamples; ++i) {
			uint32_t sample = AHBSCOPE_DATA;
			uart_puts_ila(sample);
		}
		usleep(1000000);
	}
}