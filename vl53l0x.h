#ifndef VL53L0X_H
#define VL53L0X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define VL53L0X_MAX_STRING_LENGTH 32

/* Any functions of the lib returns one of these values */
typedef enum {
	VL53L0X_OK,
	VL53L0X_FAIL,
} vl53l0x_ret_t;

/* Low Level functions which must be implemented by user */
typedef struct
{
	/* millisecond program delay */
	void (*delay_ms)(uint32_t ms);

	/* I2C communication low level functions */
	void (*i2c_write_reg)(uint8_t reg, uint8_t value);
	void (*i2c_write_reg_16bit)(uint8_t reg, uint16_t value);
	void (*i2c_write_reg_32bit)(uint8_t reg, uint32_t value);
	uint8_t (*i2c_read_reg)(uint8_t reg);
	uint16_t (*i2c_read_reg_16bit)(uint8_t reg);
	uint32_t (*i2c_read_reg_32_bit)(uint8_t reg); 
} vl53l0x_ll_t;

typedef struct
{
	/* shifted to the 1 bit left I2C address */
	uint8_t addr;

	/* hardware dependent functions */
	vl53l0x_ll_t *ll;
} vl53l0x_dev_t;

/* */
vl53l0x_ret_t vl53l0x_init(vl53l0x_dev_t *dev);

/*
 * There is an initial, once only, calibration step required that should be
 * applied at customer level during the manufacturing process. This flow takes
 * into account all parameters (cover glass, temperature & voltage)
 * from the application.
 **/
vl53l0x_ret_t vl53l0x_customer_calibration(vl53l0x_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif  // VL53L0X_H
