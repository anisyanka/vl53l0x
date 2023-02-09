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
	VL53L0X_FAIL
} vl53l0x_ret_t;

typedef enum {
	VL53L0X_SINGLE,
	VL53L0X_CONTINUOUS,
	VL53L0X_TIMED,
} vl53l0x_measure_mode_t;

/* Low Level functions which must be implemented by user */
typedef struct
{
	/* Millisecond program delay */
	void (*delay_ms)(uint32_t ms);

	/* I2C communication low level functions */
	void (*i2c_write_reg)(uint8_t reg, uint8_t value);
	void (*i2c_write_reg_16bit)(uint8_t reg, uint16_t value);
	void (*i2c_write_reg_32bit)(uint8_t reg, uint32_t value);
	void (*i2c_write_reg_multi)(uint8_t reg, uint8_t *src_buf, size_t count);
	uint8_t (*i2c_read_reg)(uint8_t reg);
	uint16_t (*i2c_read_reg_16bit)(uint8_t reg);
	uint32_t (*i2c_read_reg_32bit)(uint8_t reg);
	void (*i2c_read_reg_multi)(uint8_t reg, uint8_t *dst_buf, size_t count);

	/* Control power pin. Don't implement if don't use this pin */
	void (*xshut_set)(void);
	void (*xshut_reset)(void);
} vl53l0x_ll_t;

typedef struct
{
	/* shifted to the 1 bit left I2C address */
	uint8_t addr;

	/* Read by init ST API and used when starting measurement.
	 * Internal used only */
	uint8_t __stop_variable;

	uint8_t __measurement_mode;
	uint32_t __measurement_timeout;
	uint32_t __meas_time_bud_us;

	/* hardware dependent functions */
	vl53l0x_ll_t *ll;
} vl53l0x_dev_t;

/* Init and power control */
vl53l0x_ret_t vl53l0x_init(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_shutdown(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_power_up(vl53l0x_dev_t *dev);

/*
 * modes:
 * VL53L0X_SINGLE - ranging is performed only once after the start API function is called.
 *                  System returns to SW standby automatically.
 *
 * VL53L0X_CONTINUOUS - ranging is performed in a continuous way after the start API function
 *                      is called. As soon as the measurement is finished, another one
 *                      is started without delay. User has to stop the ranging to return
 *                      to SW standby. The last measurement is completed before stopping.
 *
 * VL53L0X_TIMED - ranging is performed in a continuous way after the start API function
 *                 is called. When a measurement is finished, another one is started
 *                 after a user defined delay (parameter time in this API).
 *                 This delay is inter-measurement period.
 **/
vl53l0x_ret_t vl53l0x_set_measurement_mode(vl53l0x_dev_t *dev,
								vl53l0x_measure_mode_t mode,
								uint32_t time);

#ifdef __cplusplus
}
#endif

#endif  // VL53L0X_H
