#include "tb_cxxrtl_io.h"

/*
 * BMP581 pressure and temperature reader using the I2C peripheral.
 *
 * Wiring:
 *   gpio[0] / pin 27 -> SDA
 *   gpio[1] / pin 28 -> SCL
 *   Add external pull-ups to the sensor supply on both lines.
 *
 * Enabling the I2C peripheral claims SDA/SCL from GPIO. The BMP581
 * address is selected for SDO high; change BMP581_ADDR to 0x46 for SDO low.
 */

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

#define I2C_DIV_100KHZ 240u
#define BMP581_DATA_READY_TIMEOUT_US 100000u

static bool i2c_pending_start;

static void i2c_init(void) {
	i2c_hw_enable(I2C_DIV_100KHZ);
	i2c_pending_start = false;
}

static bool i2c_start(void) {
	i2c_pending_start = true;
	return true;
}

static void i2c_stop(void) {
	i2c_csr_hw_t csr = {.value = 0u};

	csr.bits.en = 1u;
	csr.bits.sto = 1u;
	i2c_hw_wait();
	mm_i2c->csr = csr;
	i2c_hw_wait();
	i2c_pending_start = false;
}

static bool i2c_write_byte(uint8_t value) {
	i2c_csr_hw_t csr = {.value = 0u};

	csr.bits.en = 1u;
	csr.bits.wr = 1u;
	csr.bits.sta = i2c_pending_start;
	i2c_pending_start = false;
	i2c_hw_wait();
	mm_i2c->data = value;
	mm_i2c->csr = csr;
	i2c_hw_wait();
	return !mm_i2c->csr.bits.rxack;
}

static bool i2c_read_byte(uint8_t *value, bool acknowledge) {
	i2c_csr_hw_t csr = {.value = 0u};

	csr.bits.en = 1u;
	csr.bits.rd = 1u;
	csr.bits.sta = i2c_pending_start;
	csr.bits.nack = !acknowledge;
	i2c_pending_start = false;
	i2c_hw_wait();
	mm_i2c->csr = csr;
	i2c_hw_wait();
	*value = (uint8_t)mm_i2c->data;
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
