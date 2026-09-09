#include "tb_cxxrtl_io.h"

/*
 * TM1637 4-digit display over GPIO bit-bang.
 *
 * Wiring (icepi-zero header comments in fpga_icepi_zero.lpf):
 *   gpio[15] / pin 10  -> CLK
 *   gpio[14] / pin 8  -> DIO
 *   5V/GND to the module
 *
 * Display format: SS.mm  (seconds . centiseconds), starting at 00.00 on boot.
 * mtime is a 1 us tick (see example_soc timer timebase).
 */

#ifndef TM1637_CLK_GPIO
#define TM1637_CLK_GPIO 15
#endif
#ifndef TM1637_DIO_GPIO
#define TM1637_DIO_GPIO 14
#endif

#define TM1637_CLK_MASK (1u << TM1637_CLK_GPIO)
#define TM1637_DIO_MASK (1u << TM1637_DIO_GPIO)
#define TM1637_PIN_MASK (TM1637_CLK_MASK | TM1637_DIO_MASK)

#define TM1637_HALF_US 500u

/* Brightness 0..7, display on. */
#define TM1637_BRIGHTNESS 7u

static const uint8_t digit_seg[10] = {
	0x3f, /* 0 */
	0x06, /* 1 */
	0x5b, /* 2 */
	0x4f, /* 3 */
	0x66, /* 4 */
	0x6d, /* 5 */
	0x7d, /* 6 */
	0x07, /* 7 */
	0x7f, /* 8 */
	0x6f, /* 9 */
};

static uint32_t gpio_out_shadow;

static inline void clk_set(int level) {
	if (level)
		gpio_out_shadow |= TM1637_CLK_MASK;
	else
		gpio_out_shadow &= ~TM1637_CLK_MASK;
	gpio_write(gpio_out_shadow);
}

static inline void dio_set(int level) {
	if (level)
		gpio_out_shadow |= TM1637_DIO_MASK;
	else
		gpio_out_shadow &= ~TM1637_DIO_MASK;
	gpio_write(gpio_out_shadow);
}

static void tm1637_delay(void) {
	usleep(TM1637_HALF_US);
}

static void tm1637_start(void) {
	dio_set(1);
	clk_set(1);
	tm1637_delay();
	dio_set(0);
	tm1637_delay();
	clk_set(0);
	tm1637_delay();
}

static void tm1637_stop(void) {
	clk_set(0);
	dio_set(0);
	tm1637_delay();
	clk_set(1);
	tm1637_delay();
	dio_set(1);
	tm1637_delay();
}

/* LSB-first byte; ACK is ignored (push-pull DIO). */
static void tm1637_write_byte(uint8_t b) {
	for (int i = 0; i < 8; i++) {
		clk_set(0);
		tm1637_delay();
		dio_set(b & 1u);
		tm1637_delay();
		clk_set(1);
		tm1637_delay();
		b >>= 1;
	}
	/* 9th clock for ACK */
	clk_set(0);
	dio_set(1);
	tm1637_delay();
	clk_set(1);
	tm1637_delay();
	clk_set(0);
	tm1637_delay();
}

static void tm1637_init(void) {
	gpio_out_shadow = TM1637_PIN_MASK; /* idle high */
	gpio_set_dir(TM1637_PIN_MASK);
	gpio_write(gpio_out_shadow);
	tm1637_stop();

	/* Display on + brightness */
	tm1637_start();
	tm1637_write_byte((uint8_t)(0x88u | (TM1637_BRIGHTNESS & 7u)));
	tm1637_stop();
}

/* segs[0] = leftmost digit on typical 4-digit modules. */
static void tm1637_show(const uint8_t segs[4]) {
	/* Auto-increment write mode */
	tm1637_start();
	tm1637_write_byte(0x40);
	tm1637_stop();

	tm1637_start();
	tm1637_write_byte(0xc0); /* start at grid 0 */
	for (int i = 0; i < 4; i++)
		tm1637_write_byte(segs[i]);
	tm1637_stop();
}

static void display_time_ss_mm(uint32_t total_ms) {
	uint32_t sec = (total_ms / 1000u) % 100u; /* wrap at 100 s for 2 digits */
	uint32_t cs  = (total_ms / 10u) % 100u;   /* centiseconds */

	uint8_t segs[4];
	segs[0] = digit_seg[(sec / 10u) % 10u];
	segs[1] = (uint8_t)(digit_seg[sec % 10u] | 0x80u); /* DP -> SS.mm */
	segs[2] = digit_seg[(cs / 10u) % 10u];
	segs[3] = digit_seg[cs % 10u];
	tm1637_show(segs);
}

int main(void) {
	tm1637_init();
	display_time_ss_mm(0);

	uint32_t last_cs = 0xffffffffu;
	while (1) {
		/* mtime counts microseconds from boot */
		uint64_t us = read_mtime();
		uint32_t total_ms = (uint32_t)(us / 1000ull);
		uint32_t cs = total_ms / 10u;

		if (cs != last_cs) {
			last_cs = cs;
			display_time_ss_mm(total_ms);
		}
	}

	return 0;
}
