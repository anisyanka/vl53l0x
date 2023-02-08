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
	return 0;
}

vl53l0x_ret_t vl53l0x_init(vl53l0x_dev_t *dev)
{
	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_get_dev_info(vl53l0x_dev_t *dev)
{
	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_customer_calibration(vl53l0x_dev_t *dev)
{
	return VL53L0X_OK;
}