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

typedef enum {
	VL53L0X_POLLING,
	VL53L0X_INTERRUPT,
} vl53l0x_polling_int_mode_t;

typedef struct
{
	/* Range distance in millimeter */
	uint16_t uncorrected_range_mm;

	/* SPAD count for the return signal. 8.8 format
	 * To obtain Real value it should be divided by 256
	 **/
	uint16_t effective_spad_cnt;

	/*
	 * Range Status for the current measurement. This is device
	 * dependent. Value = 0 means value is valid.
	 **/
	uint8_t range_status;

} vl53l0x_range;


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
	uint16_t __measurement_timeout_ms;
	uint8_t __measurement_mode;
	uint8_t __polling_interrupt_mode;
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
 *                 after a user defined delay (parameter ms in this API).
 *                 This delay is inter-measurement period.
 **/
vl53l0x_ret_t vl53l0x_set_measurement_mode(vl53l0x_dev_t *dev,
								vl53l0x_measure_mode_t mode,
								uint16_t ms);

/*
 * Enable\disable generation low level on GPIO1 pin on sensor chip, when measurement done.
 * Works in any mode.
 **/
vl53l0x_ret_t vl53l0x_activate_gpio_interrupt(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_deactivate_gpio_interrupt(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_clear_flag_gpio_interrupt(vl53l0x_dev_t *dev); /* DO NOT call in a real ISR */

/*
 * Send specific command to start\stop measurement cycle according to the set mode
 *
 * Call this function every time in VL53L0X_SINGLE mode to trigger a start of measurement cycle
 *
 * Call this function one time in VL53L0X_CONTINUOUS and VL53L0X_TIMED modes.
 * Better to use  with interrupt activated.
 **/
vl53l0x_ret_t vl53l0x_start_measurement(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_stop_measurement(vl53l0x_dev_t *dev);

/*
 * Returns range in millimeters via 'range' parameter
 **/
vl53l0x_ret_t vl53l0x_get_range_mm_oneshot(vl53l0x_dev_t *dev, vl53l0x_range *range);

#ifdef __cplusplus
}
#endif

#endif  // VL53L0X_H
