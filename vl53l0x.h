#ifndef VL53L0X_H
#define VL53L0X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* Any functions of the lib returns one of these values */
typedef enum {
	VL53L0X_OK,
	VL53L0X_FAIL
} vl53l0x_ret_t;

typedef enum {
	VL53L0X_SINGLE_RANGING =  0,
	VL53L0X_CONTINUOUS_RANGING =  1,
	VL53L0X_SINGLE_HISTOGRAM =  2, /* not supported for now */
	VL53L0X_CONTINUOUS_TIMED_RANGING =  3, /* is not working for now */
	VL53L0X_SINGLE_ALS = 10, /* not supported for now */
	VL53L0X_GPIO_DRIVE = 20, /* not supported for now */
	VL53L0X_GPIO_OSC = 21, /* not supported for now */
} vl53l0x_measure_mode_t;

typedef enum {
	VL53L0X_GPIO_FUNC_OFF = 0, /* No Interrupt  */
	VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_LOW = 1, /* Level Low (value < thresh_low)  */
	VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_HIGH = 2, /* Level High (value > thresh_high) */
	VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_OUT = 3, /* Out Of Window (value < thresh_low OR value > thresh_high)  */
	VL53L0X_GPIO_FUNC_NEW_MEASURE_READY = 4, /* New Sample Ready  */
} vl53l0x_gpio_func_t;

typedef struct {
	/*
	 * Range distance in millimeter
	 * (without any correction, e.g. CrossTalk compensation)
	 **/
	uint16_t range_mm;
} vl53l0x_range;


/* Low Level functions which must be implemented by user */
typedef struct {
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

typedef struct {
	/* Defines the allowed total time for a single measurement */
	uint32_t measurement_timing_budget_microseconds;

	/*
	 * Defines time between two consecutive measurements (between two
	 * measurement starts). If set to 0 means back-to-back mode
	 **/
	uint32_t inter_measurement_period_milliseconds;

	/* Single, timed, continuous */
	uint8_t device_mode;

	/* Tells if Crosstalk compensation shall be enable or not */
	uint8_t xtalk_compensation_enable;

	/* CrossTalk compensation range in millimeter */
	uint16_t xtalk_compensation_range_millimeter;

	/*
	 * CrossTalk compensation rate in Mega counts per seconds.
	 * Expressed in 16.16 fixed point format.
	 **/
	uint32_t xtalk_compensation_rate_mega_cps;

	/* Range offset adjustment (mm) */
	int32_t range_offset_micrometers;

	/* This Array store all the Limit Check value for this device. Format 16.16 */
	uint32_t limit_checks_value[6];

	/* This Array store all the Limit Check enable for this device. */
	uint8_t limit_checks_enable[6];

	/* This Array store all the Status of the check linked to last measurement. */
	uint8_t limit_checks_status[6];

	/* Tells if Wrap Around Check shall be enable or not */
	uint8_t wrap_around_check_enable;
} vl53l0x_params_t;

typedef struct {
	uint8_t ref_spad_enables[6]; /* Reference Spad Enables */
	uint8_t ref_good_spad_map[6]; /* Reference Spad Good Spad Map */
} vl53l0x_spad_data_t;

typedef struct {
	/* Frequency used in 16.16 format */
	uint32_t osc_frequency_MHz;

	/* store the functionality of the GPIO */
	uint8_t gpio_func;

	/* Reference array sigma value in 1/100th of [mm] e.g. 100 = 1mm */
	uint16_t sigma_est_ref_array;

	/* Effective Pulse width for sigma estimate in 1/100th * of ns e.g. 900 = 9.0ns */
	uint16_t sigma_est_eff_pulse_width;

	/* Effective Ambient width for sigma estimate in 1/100th of ns * e.g. 500 = 5.0ns */
	uint16_t sigma_est_eff_amb_width;

	/* Target Ambient Rate for Ref spad management */
	uint16_t target_ref_rate;

	uint8_t module_id; /* Module ID */
	uint8_t revision; /* test Revision */
	char product_id[32];

	uint8_t reference_spad_count;
	uint8_t reference_spad_type;

	uint32_t part_uid_upper;
	uint32_t part_uid_lower;

	/* Linearity Corrective Gain value in x1000 */
	uint16_t linearity_corrective_gain;
} vl53l0x_dev_specific_params_t;

typedef struct {
	/*
	 * Hardware dependent functions.
	 * Users have to implement its in their application
	 **/
	vl53l0x_ll_t *ll;

	/* Current parameters of the device */
	vl53l0x_params_t cur_param;

	/* Specific parameters of the device */
	vl53l0x_dev_specific_params_t spec_param;

	/* Internal value for the sequence config */
	uint8_t sequence_config;

	/* Enable/Disable fractional part of ranging data */
	uint8_t range_fractional_enable;

	/* Info about Single Photon Avalanche Diodes matrix */
	vl53l0x_spad_data_t spad_data;

	uint8_t stop_variable;

} vl53l0x_dev_t;

/* Init and power control */
vl53l0x_ret_t vl53l0x_init(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_shutdown(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_power_up(vl53l0x_dev_t *dev);

/*
 * modes:
 * VL53L0X_SINGLE_RANGING - ranging is performed only once after the start API function is called.
 *                          System returns to SW standby automatically.
 *
 * VL53L0X_CONTINUOUS_RANGING - ranging is performed in a continuous way after the start API function
 *                              is called. As soon as the measurement is finished, another one
 *                              is started without delay. User has to stop the ranging to return
 *                              to SW standby. The last measurement is completed before stopping.
 *
 * VL53L0X_CONTINUOUS_TIMED_RANGING (IS NOT WORKING FOR NOW) - ranging is performed in a continuous way after the start API function
 *                                    is called. When a measurement is finished, another one is started
 *                                    after a user defined delay (parameter ms in this API).
 *                                    This delay is inter-measurement period.
 **/
vl53l0x_ret_t vl53l0x_set_measurement_mode(vl53l0x_dev_t *dev,
                                           vl53l0x_measure_mode_t mode,
                                           uint32_t ms);

/*
 * Enable\disable generation low level on GPIO1 pin on sensor chip, when measurement done.
 * Works in any mode.
 **/
vl53l0x_ret_t vl53l0x_activate_gpio_interrupt(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_deactivate_gpio_interrupt(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_clear_flag_gpio_interrupt(vl53l0x_dev_t *dev); /* DO NOT call this func in a real ISR */

/*
 * Send specific command to start\stop measurement cycle according to the set mode
 *
 * In SINGLE mode don't use this func. Use vl53l0x_get_range_mm_oneshot() API.
 * Call this function every time in VL53L0X_SINGLE mode to trigger a start of measurement cycle
 *
 * Call this function one time in CONTINUOUS and TIMED modes.
 * Better to use  with interrupt activated.
 **/
vl53l0x_ret_t vl53l0x_start_measurement(vl53l0x_dev_t *dev);
vl53l0x_ret_t vl53l0x_stop_measurement(vl53l0x_dev_t *dev);

/*
 * Returns range in millimeters via 'range' parameter
 **/
vl53l0x_ret_t vl53l0x_get_range_mm_oneshot(vl53l0x_dev_t *dev, vl53l0x_range *range);

/*
 * Returns range in millimeters via 'range' parameter
 * Enable GPIO interrupt and use this to get range after INT will occur.
 * Setup your pin like external interrupt, falling edge mode. (low level is active).
 *
 * DO NOT call this func in a real ISR
 * You will receive INT from the GPIO after each measurement in continuous mode
 * (±40ms) or use TIMED mode and set another time with vl53l0x_set_measurement_mode() API.
 **/
vl53l0x_ret_t vl53l0x_get_range_mm_continuous(vl53l0x_dev_t *dev, vl53l0x_range *range);

#ifdef __cplusplus
}
#endif

#endif  // VL53L0X_H
