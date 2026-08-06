#ifndef _TB_CXXRTL_IO_H
#define _TB_CXXRTL_IO_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "hazard3_irq.h"

// ----------------------------------------------------------------------------
// SOC IO hardware layout

#define TIMER_BASE 0x40000000
#define UART_BASE 0x40004000
#define USB_CDC_BASE 0x40008000

typedef struct {
	volatile uint32_t ctrl;
	uint32_t _pad0;
	volatile uint32_t mtime;
	volatile uint32_t mtimeh;
	volatile uint32_t mtimecmp;
	volatile uint32_t mtimecmph;
} timer_hw_t;

#define mm_timer ((timer_hw_t *const)(TIMER_BASE))

typedef union {
	uint32_t value;
	struct {
		uint32_t en : 1;
		uint32_t busy : 1;
		uint32_t txie : 1;
		uint32_t rxie : 1;
		uint32_t ctsen : 1;
		uint32_t _reserved0 : 3;
		uint32_t loopback : 1;
		uint32_t _reserved1 : 23;
	} bits;
} uart_csr_hw_t;

typedef union {
	uint32_t value;
	struct {
		uint32_t frac : 4;
		uint32_t intgr : 10;
		uint32_t _reserved0 : 18;
	} bits;
} uart_div_hw_t;

typedef union {
	uint32_t value;
	struct {
		uint32_t txlevel : 8;
		uint32_t txfull : 1;
		uint32_t txempty : 1;
		uint32_t txover : 1;
		uint32_t txunder : 1;
		uint32_t _reserved0 : 4;
		uint32_t rxlevel : 8;
		uint32_t rxfull : 1;
		uint32_t rxempty : 1;
		uint32_t rxover : 1;
		uint32_t rxunder : 1;
		uint32_t _reserved1 : 4;
	} bits;
} uart_fstat_hw_t;

typedef struct {
	volatile uart_csr_hw_t csr;
	volatile uart_div_hw_t div;
	volatile uart_fstat_hw_t fstat;
	volatile uint32_t tx;
	volatile uint32_t rx;
} uart_hw_t;

#define mm_uart ((uart_hw_t *const)(UART_BASE))

typedef union {
	uint32_t value;
	struct {
		uint32_t in_ready : 1;
		uint32_t out_valid : 1;
		uint32_t _reserved0 : 30;
	} bits;
} usb_cdc_fstat_hw_t;

typedef struct {
	volatile usb_cdc_fstat_hw_t fstat;
	volatile uint32_t tx_data;
	volatile uint32_t rx_data;
} usb_cdc_hw_t;

#define mm_usb_cdc ((usb_cdc_hw_t *const)(USB_CDC_BASE))

// ----------------------------------------------------------------------------
// SOC IO convenience functions

#define UART_INTR_BUF_SIZE 128u
#define UART_INTR_BUF_MASK (UART_INTR_BUF_SIZE - 1u)

static volatile uint32_t uart_intr_tx_head;
static volatile uint32_t uart_intr_tx_tail;
static volatile uint32_t uart_intr_rx_head;
static volatile uint32_t uart_intr_rx_tail;
static char uart_intr_tx_buf[UART_INTR_BUF_SIZE];
static char uart_intr_rx_buf[UART_INTR_BUF_SIZE];

static inline uint32_t uart_intr_irq_disable(void) {
	return read_clear_csr(mstatus, 0x8u);
}

static inline void uart_intr_irq_restore(uint32_t mstatus_prev) {
	if (mstatus_prev & 0x8u)
		set_csr(mstatus, 0x8u);
}

static void uart_intr_irq_handler(void) {
	while (!mm_uart->fstat.bits.rxempty) {
		uint32_t rx_head = uart_intr_rx_head;
		uart_intr_rx_buf[rx_head & UART_INTR_BUF_MASK] = (char)mm_uart->rx;
		asm volatile ("" : : : "memory");
		uart_intr_rx_head = rx_head + 1u;
		if (uart_intr_rx_head - uart_intr_rx_tail > UART_INTR_BUF_SIZE)
			uart_intr_rx_tail = uart_intr_rx_head - UART_INTR_BUF_SIZE;
	}

	while (!mm_uart->fstat.bits.txfull && uart_intr_tx_head != uart_intr_tx_tail) {
		uint32_t tx_tail = uart_intr_tx_tail;
		mm_uart->tx = (uint32_t)(uint8_t)uart_intr_tx_buf[tx_tail & UART_INTR_BUF_MASK];
		asm volatile ("" : : : "memory");
		uart_intr_tx_tail = tx_tail + 1u;
	}

	if (uart_intr_tx_head == uart_intr_tx_tail)
		mm_uart->csr.bits.txie = 0u;
}

static inline void uart_intr_init(void) {
	uart_csr_hw_t csr;
	uart_div_hw_t div;
	uint32_t mstatus_prev = uart_intr_irq_disable();

	uart_intr_tx_head = 0u;
	uart_intr_tx_tail = 0u;
	uart_intr_rx_head = 0u;
	uart_intr_rx_tail = 0u;

	csr.bits.en = 1u;
	csr.bits.txie = 0u;
	csr.bits.rxie = 1u;
	mm_uart->csr = csr;

	// 115200 baudrate with 48 MHz clock
	// 48 MHz / 115200 / 8 = 52.083333333 = 52 + 0.083333333 ≈ 52 + 1/16
	div.bits.intgr = 52u;
	div.bits.frac = 1u;
	mm_uart->div = div;

	external_irq_enable(true);

	uart_intr_irq_restore(mstatus_prev);
	global_irq_enable(true);
}

void __attribute__((interrupt)) isr_external_irq(void) {
	uart_intr_irq_handler();
}

void uart_intr_puts(const char *s) {
	while (*s) {
		while (uart_intr_tx_head - uart_intr_tx_tail >= UART_INTR_BUF_SIZE)
			asm volatile ("wfi");

		uint32_t mstatus_prev = uart_intr_irq_disable();
		uint32_t tx_head = uart_intr_tx_head;

		if (tx_head - uart_intr_tx_tail < UART_INTR_BUF_SIZE) {
			uart_intr_tx_buf[tx_head & UART_INTR_BUF_MASK] = *s++;
			asm volatile ("" : : : "memory");
			uart_intr_tx_head = tx_head + 1u;
			mm_uart->csr.bits.txie = 1u;
		}

		uart_intr_irq_restore(mstatus_prev);
	}
}

uint32_t uart_intr_gets(char *s, uint32_t len, bool blocking) {
	uint32_t copied = 0u;

	while (copied < len) {
		uint32_t mstatus_prev = uart_intr_irq_disable();
		uint32_t rx_tail = uart_intr_rx_tail;

		if (uart_intr_rx_head != rx_tail) {
			s[copied++] = uart_intr_rx_buf[rx_tail & UART_INTR_BUF_MASK];
			asm volatile ("" : : : "memory");
			uart_intr_rx_tail = rx_tail + 1u;
			uart_intr_irq_restore(mstatus_prev);
			continue;
		}

		uart_intr_irq_restore(mstatus_prev);
		if (!blocking)
			break;
		asm volatile ("wfi");
	}

	return copied;
}

#define UART_U32_BUF_SIZE 11u
 
static inline uint64_t read_mtime(void) {
	volatile uint32_t *lo = &(mm_timer->mtime);
	volatile uint32_t *hi = &(mm_timer->mtimeh);

	uint32_t h1, l, h2;
	do {
		h1 = *hi;
		l  = *lo;
		h2 = *hi;
	} while (h1 != h2);

	return ((uint64_t)h1 << 32) | l;
}

void usleep(uint32_t usec) {
	uint64_t start = read_mtime();
	while ((read_mtime() - start) < usec);
}

static inline void u32_to_buf(uint32_t value, char *buf) {
	char scratch[UART_U32_BUF_SIZE];
	uint32_t i = 0u;

	do {
		scratch[i++] = (char)('0' + value % 10u);
		value /= 10u;
	} while (value);

	for (uint32_t j = 0u; j < i; ++j)
		buf[j] = scratch[i - j - 1u];
	buf[i] = '\0';
}

static inline void usb_cdc_putc_blocking(char c) {
	while (!(mm_usb_cdc->fstat.bits.in_ready));
	mm_usb_cdc->tx_data = (uint32_t)c;
}

void usb_cdc_puts(const char *s) {
	while (*s) {
		usb_cdc_putc_blocking(*s++);
	}
}

void usb_cdc_puts_data(const char *s, uint32_t len) {
	for (uint32_t i = 0; i < len; ++i) {
		usb_cdc_putc_blocking(s[i]);
	}
}

void usb_cdc_puts_32(uint32_t value) {
	char buf[UART_U32_BUF_SIZE];
	u32_to_buf(value, buf);
	usb_cdc_puts(buf);
}

static void uart_init(void) {
	uart_csr_hw_t csr = {.value = 0u};
	uart_div_hw_t div = {.value = 0u};

	csr.bits.en = 1u;
	mm_uart->csr = csr;

	// 115200 baudrate with 48 MHz clock
	// 48 MHz / 115200 / 8 = 52.083333333 = 52 + 0.083333333 ≈ 52 + 1/16
	div.bits.intgr = 52u;
	div.bits.frac = 1u;
	mm_uart->div = div;
}

static inline void uart_putc_blocking(char c) {
	while (mm_uart->fstat.bits.txfull);
	mm_uart->tx = (uint32_t)c;
}

void uart_puts(const char *s) {
	while (*s) {
		uart_putc_blocking(*s++);
	}
}

void uart_puts_data(const char *s, uint32_t len) {
	for (uint32_t i = 0; i < len; ++i) {
		uart_putc_blocking(s[i]);
	}
}

void uart_puts_32(uint32_t value) {
	char buf[UART_U32_BUF_SIZE];
	u32_to_buf(value, buf);
	uart_puts(buf);
}

static inline char uart_getc_blocking(void) {
	while (mm_uart->fstat.bits.rxempty);
	return (char)mm_uart->rx;
}

void uart_gets(char *s, uint32_t maxlen) {
    uint32_t i = 0;
	char c;

	if (maxlen <= 1)
		return;

	while (true) {
		if (i + 1 < maxlen) {
			c = uart_getc_blocking();
			c = (c == '\r') ? '\n' : c;
			s[i++] = c;
			if (c == '\n')
			{
				s[i] = '\r';
				break;
			}
		} else {
			s[i] = '\0';
			break;
		}
	}
}

#endif
