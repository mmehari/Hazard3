#include "tb_cxxrtl_io.h"

/*
 * BMP581 pressure and temperature reader using GPIO bit-banged I2C.
 *
 * Wiring:
 *   gpio[0] -> SDA
 *   gpio[1] -> SCL
 *   Add external pull-ups to the sensor supply on both lines.
 *
 * The GPIO peripheral emulates open-drain outputs by switching each line
 * between output-low and input (released). The BMP581 address is selected
 * for SDO low; change BMP581_ADDR to 0x47 for SDO high.
 */

#define BMP581_SDA_GPIO 0
#define BMP581_SCL_GPIO 1
#define BMP581_SDA (1u << BMP581_SDA_GPIO)
#define BMP581_SCL (1u << BMP581_SCL_GPIO)
#define BMP581_PINS (BMP581_SDA | BMP581_SCL)

#define BMP581_ADDR 0x47u
#define BMP581_CHIP_ID 0x50u
#define BMP581_REG_CHIP_ID 0x01u
#define BMP581_REG_INT_SOURCE 0x15u
#define BMP581_REG_INT_STATUS 0x27u
#define BMP581_REG_STATUS 0x28u
#define BMP581_REG_TEMP_DATA 0x1du
#define BMP581_REG_OSR_CONFIG 0x36u
#define BMP581_REG_ODR_CONFIG 0x37u
#define BMP581_REG_CMD 0x7eu
#define BMP581_SOFT_RESET 0xb6u

#define I2C_DELAY_US 5u
#define I2C_TIMEOUT_US 100u
#define BMP581_DATA_READY_TIMEOUT_US 100000u

static uint32_t gpio_out;
static uint32_t gpio_dir;

static inline void gpio_commit(void) {
	mm_gpio->data = gpio_out;
	mm_gpio->dir = gpio_dir;
}

static inline void i2c_sda_low(void) {
	gpio_out &= ~BMP581_SDA;
	gpio_dir |= BMP581_SDA;
	gpio_commit();
}

static inline void i2c_sda_release(void) {
	gpio_out |= BMP581_SDA;
	gpio_dir &= ~BMP581_SDA;
	gpio_commit();
}

static inline void i2c_scl_low(void) {
	gpio_out &= ~BMP581_SCL;
	gpio_dir |= BMP581_SCL;
	gpio_commit();
}

static inline void i2c_scl_release(void) {
	gpio_out |= BMP581_SCL;
	gpio_dir &= ~BMP581_SCL;
	gpio_commit();
}

static bool i2c_scl_high(void) {
	i2c_scl_release();
	for (uint32_t elapsed = 0; elapsed < I2C_TIMEOUT_US; ++elapsed) {
		if (gpio_read() & BMP581_SCL)
			return true;
		usleep(1);
	}
	return false;
}

static void i2c_init(void) {
	gpio_out = BMP581_PINS;
	gpio_dir = 0u;
	gpio_commit();
}

static bool i2c_start(void) {
	i2c_sda_release();
	if (!i2c_scl_high())
		return false;
	usleep(I2C_DELAY_US);
	i2c_sda_low();
	usleep(I2C_DELAY_US);
	i2c_scl_low();
	return true;
}

static void i2c_stop(void) {
	i2c_sda_low();
	usleep(I2C_DELAY_US);
	(void)i2c_scl_high();
	usleep(I2C_DELAY_US);
	i2c_sda_release();
	usleep(I2C_DELAY_US);
}

static bool i2c_write_bit(bool bit) {
	if (bit)
		i2c_sda_release();
	else
		i2c_sda_low();
	usleep(I2C_DELAY_US);
	if (!i2c_scl_high())
		return false;
	usleep(I2C_DELAY_US);
	i2c_scl_low();
	return true;
}

static bool i2c_write_byte(uint8_t value) {
	for (uint32_t bit = 0; bit < 8; ++bit) {
		if (!i2c_write_bit((value & 0x80u) != 0u))
			return false;
		value <<= 1;
	}

	i2c_sda_release();
	usleep(I2C_DELAY_US);
	if (!i2c_scl_high())
		return false;
	usleep(I2C_DELAY_US);
	bool acknowledged = (gpio_read() & BMP581_SDA) == 0u;
	i2c_scl_low();
	return acknowledged;
}

static bool i2c_read_byte(uint8_t *value, bool acknowledge) {
	uint8_t result = 0u;
	i2c_sda_release();
	for (uint32_t bit = 0; bit < 8; ++bit) {
		result <<= 1;
		if (!i2c_scl_high())
			return false;
		usleep(I2C_DELAY_US);
		if (gpio_read() & BMP581_SDA)
			result |= 1u;
		i2c_scl_low();
		usleep(I2C_DELAY_US);
	}
	if (!i2c_write_bit(!acknowledge))
		return false;
	*value = result;
	return true;
}

static bool bmp581_write_reg(uint8_t reg, uint8_t value) {
	bool ok = i2c_start() &&
		i2c_write_byte((uint8_t)(BMP581_ADDR << 1)) &&
		i2c_write_byte(reg) &&
		i2c_write_byte(value);
	i2c_stop();
	return ok;
}

static bool bmp581_read_regs(uint8_t reg, uint8_t *data, uint32_t length) {
	if (!i2c_start() ||
	    !i2c_write_byte((uint8_t)(BMP581_ADDR << 1)) ||
	    !i2c_write_byte(reg) ||
	    !i2c_start() ||
	    !i2c_write_byte((uint8_t)((BMP581_ADDR << 1) | 1u))) {
		i2c_stop();
		return false;
	}

	for (uint32_t i = 0; i < length; ++i) {
		if (!i2c_read_byte(&data[i], i + 1u < length)) {
			i2c_stop();
			return false;
		}
	}
	i2c_stop();
	return true;
}

static void print_temperature(int32_t raw) {
	int32_t centidegrees =
		(int32_t)(((int64_t)raw * 100 + (raw < 0 ? -32768 : 32768)) / 65536);
	if (centidegrees < 0) {
		uart_puts("-");
		centidegrees = -centidegrees;
	}
	uart_puts_32((uint32_t)centidegrees / 100u);
	uart_puts(".");
	uint32_t fraction = (uint32_t)centidegrees % 100u;
	if (fraction < 10u)
		uart_puts("0");
	uart_puts_32(fraction);
	uart_puts(" C");
}

static bool bmp581_data_ready(void) {
	uint8_t int_status;
	uint64_t deadline = read_mtime() + BMP581_DATA_READY_TIMEOUT_US;
	do {
		if (!bmp581_read_regs(BMP581_REG_INT_STATUS, &int_status, 1u))
			return false;
		if (int_status & 0x01u)
			return true;
		usleep(1000);
	} while (read_mtime() < deadline);
	return false;
}

static bool bmp581_init(void) {
	uint8_t chip_id;
	uint8_t status;
	uint8_t int_status;

	if (!bmp581_read_regs(BMP581_REG_CHIP_ID, &chip_id, 1u)) {
		uart_puts("BMP581 chip ID I2C failure\r\n");
		return false;
	}
	uart_puts("BMP581 chip ID=");
	uart_puts_32(chip_id);
	uart_puts("\r\n");
	if (chip_id != BMP581_CHIP_ID) {
		uart_puts("BMP581 wrong chip ID\r\n");
		return false;
	}

	if (!bmp581_write_reg(BMP581_REG_CMD, BMP581_SOFT_RESET)) {
		uart_puts("BMP581 reset I2C failure\r\n");
		return false;
	}
	/* The BMP581 reset completion time is specified as 12 ms. */
	usleep(12000);

	if (!bmp581_read_regs(BMP581_REG_CHIP_ID, &chip_id, 1u)) {
		uart_puts("BMP581 post-reset chip ID I2C failure\r\n");
		return false;
	}
	if (chip_id != BMP581_CHIP_ID) {
		uart_puts("BMP581 post-reset wrong chip ID=");
		uart_puts_32(chip_id);
		uart_puts("\r\n");
		return false;
	}
	if (!bmp581_read_regs(BMP581_REG_STATUS, &status, 1u)) {
		uart_puts("BMP581 status I2C failure\r\n");
		return false;
	}
	if (!(status & 0x02u)) {
		uart_puts("BMP581 NVM not ready, status=");
		uart_puts_32(status);
		uart_puts("\r\n");
		return false;
	}
	if (status & 0x04u) {
		uart_puts("BMP581 NVM error, status=");
		uart_puts_32(status);
		uart_puts("\r\n");
		return false;
	}
	if (!bmp581_read_regs(BMP581_REG_INT_STATUS, &int_status, 1u)) {
		uart_puts("BMP581 interrupt status I2C failure\r\n");
		return false;
	}
	if (!(int_status & 0x10u)) {
		uart_puts("BMP581 POR incomplete, INT_STATUS=");
		uart_puts_32(int_status);
		uart_puts("\r\n");
		return false;
	}

	/* 1x oversampling, pressure enabled; continuous mode at 10 Hz. */
	if (!bmp581_write_reg(BMP581_REG_OSR_CONFIG, 0x40u) ||
	    !bmp581_write_reg(BMP581_REG_INT_SOURCE, 0x01u) ||
	    !bmp581_write_reg(BMP581_REG_ODR_CONFIG, 0x5fu)) {
		uart_puts("BMP581 configuration I2C failure\r\n");
		return false;
	}
	if (!bmp581_data_ready()) {
		uart_puts("BMP581 data-ready timeout\r\n");
		return false;
	}
	return true;
}

int main(void) {
	uart_init();
	i2c_init();
	uart_puts("BMP581 firmware\r\n");

	while (!bmp581_init()) {
		uart_puts("BMP581 not found\r\n");
		usleep(1000000);
	}

	uart_puts("BMP581 ready\r\n");
	while (1) {
		uint8_t data[6];
		if (bmp581_data_ready() &&
		    bmp581_read_regs(BMP581_REG_TEMP_DATA, data, sizeof(data))) {
			int32_t raw_temperature = (int32_t)(
				((uint32_t)data[2] << 16) |
				((uint32_t)data[1] << 8) |
				data[0]);
			if (raw_temperature & 0x00800000)
				raw_temperature |= (int32_t)0xff000000;
			uint32_t raw_pressure =
				((uint32_t)data[5] << 16) |
				((uint32_t)data[4] << 8) |
				data[3];

			uart_puts("temperature=");
			print_temperature(raw_temperature);
			uart_puts(" pressure=");
			uart_puts_32(raw_pressure / 64u);
			uart_puts(" Pa\r\n");
		} else {
			uart_puts("BMP581 I2C read error\r\n");
		}
		usleep(1000000);
	}

	return 0;
}
