#include "vl53l0x.h"

/*
 * Write sequence:
 * addr byte 0x52 + reg byte + data to write to the pointed reg
 *
 * Read sequence:
 * addr byte 0x52 + reg byte + STOP cond + addr byte 0x53 + read content of pointed teg byte
 *
 * Sequential write: (autoincrement of reg addr byte)
 * addr byte 0x52 + reg byte + data[for reg byte]_ + data[for next reg byte] + ...
 *
 * Sequential read: (autoincrement of reg addr byte)
 * addr byte 0x52 + reg byte + STOP cond + addr byte 0x53 + read data[0] + read data[1] + ...
 **/
#define ADDR_BYTE_WRITE (0x52)
#define ADDR_BYTE_READ	(0x53)

/*
 * I2C interface - reference registers
 * The registers can be used to validate the user I2C interface.
 **/
#define REF_REG_0		(0xC0)
#define REF_REG_0_VAL	(0xEE)
#define REF_REG_1		(0xC1)
#define REF_REG_1_VAL	(0xAA)
#define REF_REG_2		(0xC2)
#define REF_REG_2_VAL	(0x10)
#define REF_REG_3		(0x51)
#define REF_REG_3_VAL	(0x0099)
#define REF_REG_4		(0x61)
#define REF_REG_4_VAL	(0x0000)

// returns 0 if OK
static int check_args(vl53l0x_dev_t *dev)
{
	if (!dev || !dev->ll->delay_ms || \
		!dev->ll->i2c_write_reg || !dev->ll->i2c_write_reg_16bit || \
		!dev->ll->i2c_write_reg_32bit || !dev->ll->i2c_read_reg || \
		!dev->ll->i2c_read_reg_16bit || !dev->ll->i2c_read_reg_32_bit) {
		return 1;
	}

	return 0;
}

// returns 0 if OK
static int check_i2c_comm(vl53l0x_dev_t *dev)
{
	int ret = 0;

	if (dev->ll->i2c_read_reg(REF_REG_0) != REF_REG_0_VAL) {
		++ret;
	}

	if (dev->ll->i2c_read_reg(REF_REG_1) != REF_REG_1_VAL) {
		++ret;
	}

	if (dev->ll->i2c_read_reg(REF_REG_2) != REF_REG_2_VAL) {
		++ret;
	}

	if (dev->ll->i2c_read_reg_16bit(REF_REG_3) != REF_REG_3_VAL) {
		++ret;
	}

	if (dev->ll->i2c_read_reg_16bit(REF_REG_4) != REF_REG_4_VAL) {
		++ret;
	}

	return ret;
}

vl53l0x_ret_t vl53l0x_init(vl53l0x_dev_t *dev)
{
	if (check_args(dev)) {
		return VL53L0X_FAIL;
	}

	if (check_i2c_comm(dev)) {
		return VL53L0X_FAIL;
	}

	vl53l0x_power_up();

	/* Wait 1.2 ms max (according to spec) until vl53l0x fw boots */
	dev->ll->delay_ms(2);

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_shutdown(vl53l0x_dev_t *dev)
{
	if (!dev->ll->xshut_set || !dev->ll->xshut_reset) {
		return VL53L0X_FAIL;
	}

	dev->ll->xshut_reset();

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_power_up(vl53l0x_dev_t *dev)
{
	if (!dev->ll->xshut_set || !dev->ll->xshut_reset) {
		return VL53L0X_FAIL;
	}

	dev->ll->xshut_set();

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_customer_calibration(vl53l0x_dev_t *dev)
{
	return VL53L0X_OK;
}