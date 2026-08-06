#include "tb_cxxrtl_io.h"

int main() {
	uart_intr_init();
	while(true) {
		char buf[2] = {0};
		uart_intr_gets(buf, 1, true);
		buf[0] = (buf[0] == '\r') ? '\n' : buf[0];
		uart_intr_puts(buf);
	}
	return 123;
}