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

#define VL53L0X_MAKEUINT16(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + \
		(uint16_t)lsb)

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

/*
 * Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
 * based on VL53L0X_calc_macro_period_ps()
 * PLL_period_ps = 1655; macro_period_vclks = 2304
 **/
#define CALC_MACRO_PERIOD(vcsel_period_pclks)	((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/* Register addresses from ST API file vl53l0x_device.h */
enum {
	SYSRANGE_START                              = 0x00,
	SYSTEM_THRESH_HIGH                          = 0x0C,
	SYSTEM_THRESH_LOW                           = 0x0E,
	SYSTEM_SEQUENCE_CONFIG                      = 0x01,
	SYSTEM_RANGE_CONFIG                         = 0x09,
	SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,
	SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,
	GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,
	SYSTEM_INTERRUPT_CLEAR                      = 0x0B,
	RESULT_INTERRUPT_STATUS                     = 0x13,
	RESULT_RANGE_STATUS                         = 0x14,
	RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
	RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
	RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
	RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
	RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,
	ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,
	I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,
	MSRC_CONFIG_CONTROL                         = 0x60,
	PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
	PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
	PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
	PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,
	FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
	FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
	FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
	FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
	PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
	PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,
	PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,
	SYSTEM_HISTOGRAM_BIN                        = 0x81,
	HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
	HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,
	FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
	CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,
	MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,
	SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
	IDENTIFICATION_MODEL_ID                     = 0xC0,
	IDENTIFICATION_REVISION_ID                  = 0xC2,
	OSC_CALIBRATE_VAL                           = 0xF8,
	GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,
	GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
	DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
	DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
	POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,
	VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV            = 0x89,
	ALGO_PHASECAL_LIM                           = 0x30,
	ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

#define VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE	0
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE	1
#define VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP		2
#define VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD	3
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC	4
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE	5
#define VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS	6

#define VL53L0X_SEQUENCESTEP_TCC	(0) /* Target CentreCheck identifier. */
#define VL53L0X_SEQUENCESTEP_DSS	(1) /* Dynamic Spad Selection function Identifier. */
#define VL53L0X_SEQUENCESTEP_MSRC	(2) /* Minimum Signal Rate Check function Identifier. */
#define VL53L0X_SEQUENCESTEP_PRE_RANGE	(3) /* Pre-Range check Identifier. */
#define	VL53L0X_SEQUENCESTEP_FINAL_RANGE	(4) /* Final Range Check Identifier. */
#define	VL53L0X_SEQUENCESTEP_NUMBER_OF_CHECKS	5

#define VL53L0X_VCSEL_PERIOD_PRE_RANGE	(0) /* Identifies the pre-range vcsel period. */
#define VL53L0X_VCSEL_PERIOD_FINAL_RANGE (1) /* Identifies the final range vcsel period. */

#define VL53L0X_FIXPOINT1616TOFIXPOINT313(Value) \
	(uint16_t)((Value>>3)&0xFFFF)
#define VL53L0X_FIXPOINT313TOFIXPOINT1616(Value) \
	(uint32_t)(Value<<3)

#define VL53L0X_FIXPOINT1616TOFIXPOINT97(Value) \
	(uint16_t)((Value>>9)&0xFFFF)
#define VL53L0X_FIXPOINT97TOFIXPOINT1616(Value) \
	(uint32_t)(Value<<9)

#define VL53L0X_FIXPOINT1616TOFIXPOINT412(Value) \
	(uint16_t)((Value>>4)&0xFFFF)
#define VL53L0X_FIXPOINT412TOFIXPOINT1616(Value) \
	(uint32_t)(Value<<4)

#define VL53L0X_DEFAULT_MAX_LOOP 2000

/** mask existing bit in #VL53L0X_REG_SYSRANGE_START*/
#define VL53L0X_REG_SYSRANGE_MODE_MASK		0x0F
/** bit 0 in #VL53L0X_REG_SYSRANGE_START write 1 toggle state in
 * continuous mode and arm next shot in single shot mode
 */
#define VL53L0X_REG_SYSRANGE_MODE_START_STOP	0x01
/** bit 1 write 0 in #VL53L0X_REG_SYSRANGE_START set single shot mode */
#define VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT	0x00
/** bit 1 write 1 in #VL53L0X_REG_SYSRANGE_START set back-to-back
 *  operation mode
 */
#define VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK	0x02
/** bit 2 write 1 in #VL53L0X_REG_SYSRANGE_START set timed operation
 *  mode
 */
#define VL53L0X_REG_SYSRANGE_MODE_TIMED			0x04
/** bit 3 write 1 in #VL53L0X_REG_SYSRANGE_START set histogram operation
 *  mode
 */
#define VL53L0X_REG_SYSRANGE_MODE_HISTOGRAM		0x08

/*
 * TCC: Target CentreCheck
 * MSRC: Minimum Signal Rate Check
 * DSS: Dynamic Spad Selection
 **/
typedef struct {
	uint8_t tcc_on; /* Reports if Target Centre Check On  */
	uint8_t msrc_on; /* Reports if MSRC On  */
	uint8_t dss_on; /* Reports if DSS On  */
	uint8_t pre_range_on; /* Reports if Pre-Range On	*/
	uint8_t final_range_on; /* Reports if Final-Range On  */
} sched_sequence_steps_t;

static uint8_t default_tuning_settings[] = {
	0x01, 0xFF, 0x01, 0x01, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x01, 0x09, 0x00,
	0x01, 0x10, 0x00, 0x01, 0x11, 0x00, 0x01, 0x24, 0x01, 0x01, 0x25, 0xff,
	0x01, 0x75, 0x00, 0x01, 0xFF, 0x01, 0x01, 0x4e, 0x2c, 0x01, 0x48, 0x00,
	0x01, 0x30, 0x20, 0x01, 0xFF, 0x00, 0x01, 0x30, 0x09, 0x01, 0x54, 0x00,
	0x01, 0x31, 0x04, 0x01, 0x32, 0x03, 0x01, 0x40, 0x83, 0x01, 0x46, 0x25,
	0x01, 0x60, 0x00, 0x01, 0x27, 0x00, 0x01, 0x50, 0x06, 0x01, 0x51, 0x00,
	0x01, 0x52, 0x96, 0x01, 0x56, 0x08, 0x01, 0x57, 0x30, 0x01, 0x61, 0x00,
	0x01, 0x62, 0x00, 0x01, 0x64, 0x00, 0x01, 0x65, 0x00, 0x01, 0x66, 0xa0,
	0x01, 0xFF, 0x01, 0x01, 0x22, 0x32, 0x01, 0x47, 0x14, 0x01, 0x49, 0xff,
	0x01, 0x4a, 0x00, 0x01, 0xFF, 0x00, 0x01, 0x7a, 0x0a, 0x01, 0x7b, 0x00,
	0x01, 0x78, 0x21, 0x01, 0xFF, 0x01, 0x01, 0x23, 0x34, 0x01, 0x42, 0x00,
	0x01, 0x44, 0xff, 0x01, 0x45, 0x26, 0x01, 0x46, 0x05, 0x01, 0x40, 0x40,
	0x01, 0x0E, 0x06, 0x01, 0x20, 0x1a, 0x01, 0x43, 0x40, 0x01, 0xFF, 0x00,
	0x01, 0x34, 0x03, 0x01, 0x35, 0x44, 0x01, 0xFF, 0x01, 0x01, 0x31, 0x04,
	0x01, 0x4b, 0x09, 0x01, 0x4c, 0x05, 0x01, 0x4d, 0x04, 0x01, 0xFF, 0x00,
	0x01, 0x44, 0x00, 0x01, 0x45, 0x20, 0x01, 0x47, 0x08, 0x01, 0x48, 0x28,
	0x01, 0x67, 0x00, 0x01, 0x70, 0x04, 0x01, 0x71, 0x01, 0x01, 0x72, 0xfe,
	0x01, 0x76, 0x00, 0x01, 0x77, 0x00, 0x01, 0xFF, 0x01, 0x01, 0x0d, 0x01,
	0x01, 0xFF, 0x00, 0x01, 0x80, 0x01, 0x01, 0x01, 0xF8, 0x01, 0xFF, 0x01,
	0x01, 0x8e, 0x01, 0x01, 0x00, 0x01, 0x01, 0xFF, 0x00, 0x01, 0x80, 0x00,
	0x00, 0x00, 0x00
};

/* Returns 0 if OK */
static int check_args(vl53l0x_dev_t *dev)
{
	if (!dev || !dev->ll->delay_ms || \
		!dev->ll->i2c_write_reg || !dev->ll->i2c_write_reg_16bit || \
		!dev->ll->i2c_write_reg_32bit || !dev->ll->i2c_read_reg || \
		!dev->ll->i2c_read_reg_16bit || !dev->ll->i2c_read_reg_32bit || \
		!dev->ll->i2c_write_reg_multi || !dev->ll->i2c_read_reg_multi) {
		return 1;
	}

	return 0;
}

/* Returns 0 if OK */
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

/* Returns 0 if OK */
static int read_strobe(vl53l0x_dev_t *dev)
{
	uint8_t strobe = 0;
	int timeout_cycles = 0;

	dev->ll->i2c_write_reg(0x83, 0x00);

	/* polling use timeout to avoid deadlock */
	while (strobe == 0) {
		strobe = dev->ll->i2c_read_reg(0x83);
		if (strobe != 0) {
			break;
		}

		dev->ll->delay_ms(2);
		if (timeout_cycles >= 20) {
			return 1;
		}
		++timeout_cycles;
	}

	dev->ll->i2c_write_reg(0x83, 0x01);
	return 0;
}

static uint32_t calc_macro_period_ps(uint8_t vcsel_period_pclks)
{
	uint64_t PLL_period_ps;
	uint32_t macro_period_vclks;
	uint32_t macro_period_ps;

	PLL_period_ps = 1655;
	macro_period_vclks = 2304;

	macro_period_ps = (uint32_t)(macro_period_vclks
		* vcsel_period_pclks * PLL_period_ps);

	return macro_period_ps;
}

/* To convert register value into us */
static uint32_t calc_timeout_us(uint16_t timeout_period_mclks,
                                uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ps;
	uint32_t macro_period_ns;
	uint32_t actual_timeout_period_us = 0;

	macro_period_ps = calc_macro_period_ps(vcsel_period_pclks);
	macro_period_ns = (macro_period_ps + 500) / 1000;

	actual_timeout_period_us =
		((timeout_period_mclks * macro_period_ns) + 500) / 1000;

	return actual_timeout_period_us;
}

static uint16_t encode_timeout(uint32_t timeout_macro_clks)
{
	/*!
	 * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
	 */

	uint16_t encoded_timeout = 0;
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_macro_clks > 0) {
		ls_byte = timeout_macro_clks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte = ls_byte >> 1;
			ms_byte++;
		}

		encoded_timeout = (ms_byte << 8)
				+ (uint16_t) (ls_byte & 0x000000FF);
	}

	return encoded_timeout;
}

/*
 * Decode sequence step timeout in MCLKs from register value
 * based on VL53L0X_decode_timeout()
 * Note: the original function returned a uint32_t, but the return value is
 * always stored in a uint16_t.
 **/
static uint16_t decode_timeout(uint16_t encoded_timeout)
{
  /* format: (LSByte * 2^MSByte) + 1 */
  return (uint16_t)((encoded_timeout & 0x00FF) <<
         (uint16_t)((encoded_timeout & 0xFF00) >> 8)) + 1;
}

static uint8_t decode_vcsel_period(uint8_t vcsel_period_reg)
{
	/*!
	 * Converts the encoded VCSEL period register value into the real
	 * period in PLL clocks
	 */
	return (vcsel_period_reg + 1) << 1;
}

/* Returns 0 if OK */
static int get_vcsel_pulse_period(vl53l0x_dev_t *dev, uint8_t type, uint8_t *period)
{
	uint8_t vcsel_period_reg;

	switch (type) {
	case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
		vcsel_period_reg = dev->ll->i2c_read_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD);
		break;
	case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
		vcsel_period_reg = dev->ll->i2c_read_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD);
		break;
	default:
		return 1;
	}

	*period = decode_vcsel_period(vcsel_period_reg);
	return 0;
}


/* Returns 0 if data is ready */
static int get_measurement_data_ready(vl53l0x_dev_t *dev)
{
	uint8_t status = 0;

	if (dev->spec_param.gpio_func == VL53L0X_GPIO_FUNC_NEW_MEASURE_READY) {
		status = dev->ll->i2c_read_reg(RESULT_INTERRUPT_STATUS);
		if (status & 0x18) {
			return 1; /* range error */
		}

		if ((status & 0x07) == VL53L0X_GPIO_FUNC_NEW_MEASURE_READY) {
			return 0;
		}

	} else if (dev->spec_param.gpio_func == VL53L0X_GPIO_FUNC_OFF) {
		status = dev->ll->i2c_read_reg(RESULT_RANGE_STATUS);
		if (status & 0x01) {
			return 0;
		} else {
			return 1;
		}
	}

	return 1;
}

/*
 * Returns 0 if OK
 * Based on VL53L0X_GetInterMeasurementPeriodMilliSeconds()
 **/
static int get_inter_measure_period(vl53l0x_dev_t *dev, uint32_t *val)
{
	uint16_t osc_calibrate_val;
	uint32_t period_ms;

	osc_calibrate_val = dev->ll->i2c_read_reg_16bit(OSC_CALIBRATE_VAL);
	period_ms = dev->ll->i2c_read_reg_32bit(SYSTEM_INTERMEASUREMENT_PERIOD);

	if (osc_calibrate_val != 0) {
		*val = period_ms / osc_calibrate_val;
	} else {
		return 1;
	}

	return 0;
}

/* Returns 0 if OK */
static int get_xtalk_compensation_rate_MegaCps(vl53l0x_dev_t *dev, uint32_t *xtalk_comp)
{
	uint16_t value;
	uint32_t temp_fix1616;

	value = dev->ll->i2c_read_reg_16bit(CROSSTALK_COMPENSATION_PEAK_RATE_MCPS);

	if (value == 0) {
		/* the Xtalk is disabled return value from memory */
		dev->cur_param.xtalk_compensation_enable = 0;
	} else {
		temp_fix1616 = VL53L0X_FIXPOINT313TOFIXPOINT1616(value);
		*xtalk_comp = temp_fix1616;
		dev->cur_param.xtalk_compensation_enable = 1;
	}

	return 0;
}

/* Return 0 if OK */
static int get_offset_calibration_data_micro_meter(vl53l0x_dev_t *dev, int32_t *offset)
{
	uint16_t range_offset_register;
	int16_t c_max_offset = 2047;
	int16_t c_offset_range = 4096;

	/* Note that offset has 10.2 format */
	range_offset_register = dev->ll->i2c_read_reg_16bit(ALGO_PART_TO_PART_RANGE_OFFSET_MM);
	range_offset_register = (range_offset_register & 0x0fff);

	/* Apply 12 bit 2's compliment conversion */
	if (range_offset_register > c_max_offset) {
		*offset = (int16_t)(range_offset_register - c_offset_range) * 250;
	} else {
		*offset = (int16_t)range_offset_register * 250;
	}

	return 0;
}

/* Returns 0 if OK */
static int get_limit_check_value(vl53l0x_dev_t *dev,
                                 uint16_t limit_check_id,
								 uint32_t *limit_check_value)
{
	uint8_t enable_zero_value = 0;
	uint16_t temp16;
	uint32_t temp_fix1616;

	switch (limit_check_id) {
	case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
		/* internal computation: */
		temp_fix1616 = dev->cur_param.limit_checks_value[VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE];
		enable_zero_value = 0;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
		temp16 = dev->ll->i2c_read_reg_16bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT);
		temp_fix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(temp16);
		enable_zero_value = 1;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
		/* internal computation: */
		temp_fix1616 = dev->cur_param.limit_checks_value[VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP];
		enable_zero_value = 0;
		break;

	case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
		/* internal computation: */
		temp_fix1616 = dev->cur_param.limit_checks_value[VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD];
		enable_zero_value = 0;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
	case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
		temp16 = dev->ll->i2c_read_reg_16bit(PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT);
		temp_fix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(temp16);
		enable_zero_value = 0;
		break;

	default:
		return 1;

	}

	if (enable_zero_value == 1) {
		if (temp_fix1616 == 0) {
			/* disabled: return value from memory */
			temp_fix1616 = dev->cur_param.limit_checks_value[limit_check_id];
			*limit_check_value = temp_fix1616;
			dev->cur_param.limit_checks_enable[limit_check_id] = 0;
		} else {
			*limit_check_value = temp_fix1616;
			dev->cur_param.limit_checks_value[limit_check_id] = temp_fix1616;
			dev->cur_param.limit_checks_enable[limit_check_id] = 1;
		}
	} else {
		*limit_check_value = temp_fix1616;
	}

	return 0;
}

/* Returns 0 if OK */
static int get_limit_check_enable(vl53l0x_dev_t *dev,
                                  uint16_t limit_check_id,
                                  uint8_t *limit_check_enable)
{
	uint8_t temp8;

	if (limit_check_id >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
		*limit_check_enable = 0;
		return 1;
	} else {
		temp8 = dev->cur_param.limit_checks_enable[limit_check_id];
		*limit_check_enable = temp8;
	}

	return 0;
}

/* Returns 0 if OK */
static int get_wrap_around_check_enable(vl53l0x_dev_t *dev, uint8_t *wrap)
{
	uint8_t data;

	data = dev->ll->i2c_read_reg(SYSTEM_SEQUENCE_CONFIG);
	dev->sequence_config = data;

	if (data & (0x01 << 7)) {
		*wrap = 0x01;
	} else {
		*wrap = 0x00;
	}

	dev->cur_param.wrap_around_check_enable = *wrap;
	return 0;
}

/* Returns 0 of OK */
static int sequence_step_enabled(uint8_t sequence_step_id,
								 uint8_t sequence_config,
								 uint8_t *sequence_step_enabled)
{
	*sequence_step_enabled = 0;

	switch (sequence_step_id) {
	case VL53L0X_SEQUENCESTEP_TCC:
		*sequence_step_enabled = (sequence_config & 0x10) >> 4;
		break;
	case VL53L0X_SEQUENCESTEP_DSS:
		*sequence_step_enabled = (sequence_config & 0x08) >> 3;
		break;
	case VL53L0X_SEQUENCESTEP_MSRC:
		*sequence_step_enabled = (sequence_config & 0x04) >> 2;
		break;
	case VL53L0X_SEQUENCESTEP_PRE_RANGE:
		*sequence_step_enabled = (sequence_config & 0x40) >> 6;
		break;
	case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
		*sequence_step_enabled = (sequence_config & 0x80) >> 7;
		break;
	default:
		return 1;
	}

	return 0;
}

/* Returns 0 if OK */
static int get_sequence_step_enables(vl53l0x_dev_t *dev,
                                     sched_sequence_steps_t *sched_step)
{
	uint8_t sequence_config = 0;

	sequence_config = dev->ll->i2c_read_reg(SYSTEM_SEQUENCE_CONFIG);

	if (sequence_step_enabled(VL53L0X_SEQUENCESTEP_TCC, sequence_config, &sched_step->tcc_on)) {
		return 1;
	}

	if (sequence_step_enabled(VL53L0X_SEQUENCESTEP_DSS, sequence_config, &sched_step->dss_on)) {
		return 1;
	}

	if (sequence_step_enabled(VL53L0X_SEQUENCESTEP_MSRC, sequence_config, &sched_step->msrc_on)) {
		return 1;
	}

	if (sequence_step_enabled(VL53L0X_SEQUENCESTEP_PRE_RANGE, sequence_config, &sched_step->pre_range_on)) {
		return 1;
	}

	if (sequence_step_enabled(VL53L0X_SEQUENCESTEP_FINAL_RANGE, sequence_config, &sched_step->final_range_on)) {
		return 1;
	}

	return 0;
}

/* Returns 0 if OK */
static int get_sequence_step_timeout(vl53l0x_dev_t *dev,
                                     uint8_t sequence_step_id,
                                     uint32_t *us)
{
	uint8_t cur_vcsel_period;
	uint8_t encoded_timeout_byte = 0;
	uint32_t timeout_us = 0;
	uint16_t pre_range_encoded_timeout = 0;
	uint16_t msrc_timeout_mclks;
	uint16_t pre_range_timeout_mclks;
	uint16_t final_range_timeout_mclks = 0;
	uint16_t final_range_encoded_timeout;
	sched_sequence_steps_t sched_sequence_steps;

	if ((sequence_step_id == VL53L0X_SEQUENCESTEP_TCC)	 ||
		(sequence_step_id == VL53L0X_SEQUENCESTEP_DSS)	 ||
		(sequence_step_id == VL53L0X_SEQUENCESTEP_MSRC)) {

		if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &cur_vcsel_period)) {
			return 1;
		}
		encoded_timeout_byte = dev->ll->i2c_read_reg(MSRC_CONFIG_TIMEOUT_MACROP);
		msrc_timeout_mclks = decode_timeout(encoded_timeout_byte);
		timeout_us = calc_timeout_us(msrc_timeout_mclks, cur_vcsel_period);
	} else if (sequence_step_id == VL53L0X_SEQUENCESTEP_PRE_RANGE) {
		/* Retrieve PRE-RANGE VCSEL Period */
		if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &cur_vcsel_period)) {
			return 1;
		}

		pre_range_encoded_timeout = dev->ll->i2c_read_reg_16bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
		pre_range_timeout_mclks = decode_timeout(pre_range_encoded_timeout);
		timeout_us = calc_timeout_us(pre_range_timeout_mclks, cur_vcsel_period);
	} else if (sequence_step_id == VL53L0X_SEQUENCESTEP_FINAL_RANGE) {

		if (get_sequence_step_enables(dev, &sched_sequence_steps)) {
			return 1;
		}

		pre_range_timeout_mclks = 0;

		if (sched_sequence_steps.pre_range_on) {
			/* Retrieve PRE-RANGE VCSEL Period */
			if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &cur_vcsel_period)) {
				return 1;
			}

			/* Retrieve PRE-RANGE Timeout in Macro periods * (MCLKS) */
			pre_range_encoded_timeout = dev->ll->i2c_read_reg_16bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
			pre_range_timeout_mclks = decode_timeout(pre_range_encoded_timeout);
		}

		/* Retrieve FINAL-RANGE VCSEL Period */
		if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &cur_vcsel_period)) {
			return 1;
		}

		/* Retrieve FINAL-RANGE Timeout in Macro periods (MCLKS) */
		final_range_encoded_timeout = dev->ll->i2c_read_reg_16bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
		final_range_timeout_mclks = decode_timeout(final_range_encoded_timeout);

		final_range_timeout_mclks -= pre_range_timeout_mclks;
		timeout_us = calc_timeout_us(final_range_timeout_mclks, cur_vcsel_period);
	}

	*us = timeout_us;
	return 0;
}

/* Returns 0 if OK */
static int set_sequence_step_timeout(vl53l0x_dev_t *dev,
                                     uint8_t sequence_step_id,
                                     uint32_t timeout_us)
{
	uint8_t cur_vcsel_period;
	uint8_t msrc_encoded_timeout;
	uint16_t pre_range_encoded_timeout;
	uint16_t pre_range_timeout_mclks;
	uint16_t msrc_range_timeout_mclks;
	uint32_t final_range_timeout_mclks;
	uint16_t final_range_encoded_timeout;
	sched_sequence_steps_t sched_seq_steps;

	if ((sequence_step_id == VL53L0X_SEQUENCESTEP_TCC)	 ||
		(sequence_step_id == VL53L0X_SEQUENCESTEP_DSS)	 ||
		(sequence_step_id == VL53L0X_SEQUENCESTEP_MSRC)) {

		if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &cur_vcsel_period)) {
			return 1;
		}

		msrc_range_timeout_mclks = calc_timeout_us(timeout_us, (uint8_t)cur_vcsel_period);
		if (msrc_range_timeout_mclks > 256) {
			msrc_encoded_timeout = 255;
		}
		else {
			msrc_encoded_timeout = (uint8_t)msrc_range_timeout_mclks - 1;
		}
		dev->ll->i2c_write_reg(MSRC_CONFIG_TIMEOUT_MACROP, msrc_encoded_timeout);
	} else {
		if (sequence_step_id == VL53L0X_SEQUENCESTEP_PRE_RANGE) {
			if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &cur_vcsel_period)) {
				return 1;
			}

			pre_range_timeout_mclks = calc_timeout_us(timeout_us, (uint8_t)cur_vcsel_period);
			pre_range_encoded_timeout = encode_timeout(pre_range_timeout_mclks);
			dev->ll->i2c_write_reg_16bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, pre_range_encoded_timeout);
		} else if (sequence_step_id == VL53L0X_SEQUENCESTEP_FINAL_RANGE) {

			/* For the final range timeout, the pre-range timeout
			 * must be added. To do this both final and pre-range
			 * timeouts must be expressed in macro periods MClks
			 * because they have different vcsel periods.
			 */

			if (get_sequence_step_enables(dev, &sched_seq_steps)) {
				return 1;
			}

			pre_range_timeout_mclks = 0;

			if (sched_seq_steps.pre_range_on) {

				/* Retrieve PRE-RANGE VCSEL Period */
				if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &cur_vcsel_period)) {
					return 1;
				}

				/* Retrieve PRE-RANGE Timeout in Macro periods (MCLKS) */
				pre_range_encoded_timeout = dev->ll->i2c_read_reg_16bit(0x51);
				pre_range_timeout_mclks = decode_timeout(pre_range_encoded_timeout);
			}

			/* Calculate FINAL RANGE Timeout in Macro Periods
			 * (MCLKS) and add PRE-RANGE value
			 */

			if (get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &cur_vcsel_period)) {
				return 1;
			}

			final_range_timeout_mclks = calc_timeout_us(timeout_us, (uint8_t) cur_vcsel_period);
			final_range_timeout_mclks += pre_range_timeout_mclks;
			final_range_encoded_timeout = encode_timeout(final_range_timeout_mclks);
			dev->ll->i2c_write_reg_16bit(0x71, final_range_encoded_timeout);
		} else
			return 1;
	}

	return 0;
}

/* Returns 0 if OK */
static int set_measurement_timing_budget_micro_seconds(vl53l0x_dev_t *dev,
                                                       uint32_t microsec)
{
	uint32_t final_range_timing_budget_us;
	sched_sequence_steps_t sched_seq_steps;
	uint32_t msrc_dcc_tcc_timeout_us = 2000;
	uint32_t start_overhead_us = 1910;
	uint32_t end_overhead_us = 960;
	uint32_t msrc_overhead_us = 660;
	uint32_t tcc_overhead_us = 590;
	uint32_t dss_overhead_us = 690;
	uint32_t pre_range_overhead_us = 660;
	uint32_t final_range_overhead_us = 550;
	uint32_t pre_range_timeout_us = 0;
	uint32_t sub_timeout = 0;

	final_range_timing_budget_us = microsec - (start_overhead_us + end_overhead_us);

	if (get_sequence_step_enables(dev, &sched_seq_steps)) {
		return 1;
	}

	if (sched_seq_steps.tcc_on  ||
		sched_seq_steps.msrc_on ||
		sched_seq_steps.dss_on) {

		/* TCC, MSRC and DSS all share the same timeout */
		if (get_sequence_step_timeout(dev, VL53L0X_SEQUENCESTEP_MSRC, &msrc_dcc_tcc_timeout_us)) {
			return 1;
		}

		/* Subtract the TCC, MSRC and DSS timeouts if they are
		 * enabled.
		 */

		/* TCC */
		if (sched_seq_steps.tcc_on) {

			sub_timeout = msrc_dcc_tcc_timeout_us + tcc_overhead_us;

			if (sub_timeout < final_range_timing_budget_us) {
				final_range_timing_budget_us -= sub_timeout;
			} else {
				/* Requested timeout too big. */
				return 1;
			}
		}

		/* DSS */
		if (sched_seq_steps.dss_on) {
			sub_timeout = 2 * (msrc_dcc_tcc_timeout_us +dss_overhead_us);

			if (sub_timeout < final_range_timing_budget_us) {
				final_range_timing_budget_us -= sub_timeout;
			} else {
				/* Requested timeout too big. */
				return 1;
			}
		} else if (sched_seq_steps.msrc_on) {
			/* MSRC */
			sub_timeout = msrc_dcc_tcc_timeout_us + msrc_overhead_us;

			if (sub_timeout < final_range_timing_budget_us) {
				final_range_timing_budget_us -= sub_timeout;
			} else {
				/* Requested timeout too big. */
				return 1;
			}
		}
	}

	if (sched_seq_steps.pre_range_on) {
		/* Subtract the Pre-range timeout if enabled. */
		if (get_sequence_step_timeout(dev, VL53L0X_SEQUENCESTEP_PRE_RANGE, &pre_range_timeout_us)) {
			return 1;
		}

		sub_timeout = pre_range_timeout_us + pre_range_overhead_us;

		if (sub_timeout < final_range_timing_budget_us) {
			final_range_timing_budget_us -= sub_timeout;
		} else {
			/* Requested timeout too big. */
			return 1;
		}
	}

	if (sched_seq_steps.final_range_on) {

		final_range_timing_budget_us -= final_range_overhead_us;

		/* Final Range Timeout
		 * Note that the final range timeout is determined by the timing
		 * budget and the sum of all other timeouts within the sequence.
		 * If there is no room for the final range timeout, then an
		 * error will be set. Otherwise the remaining time will be
		 * applied to the final range.
		 */
		if (set_sequence_step_timeout(dev, VL53L0X_SEQUENCESTEP_FINAL_RANGE, final_range_timing_budget_us)) {
			return 1;
		}
	}

	return 0;
}

/* Returns 0 if OK */
static int get_measurement_timing_budget_micro_seconds(vl53l0x_dev_t *dev,
                                                       uint32_t *microsec)
{
	sched_sequence_steps_t scheduler_sequence_steps;
	uint32_t final_range_timeout_us;
	uint32_t msrc_dcc_tcc_timeout_us = 2000;
	uint32_t start_overhead_us = 1910;
	uint32_t end_overhead_us = 960;
	uint32_t msrc_overhead_us = 660;
	uint32_t tcc_overhead_us = 590;
	uint32_t dss_overhead_us = 690;
	uint32_t pre_range_overhead_us = 660;
	uint32_t final_range_overhead_us = 550;
	uint32_t pre_range_timeout_us = 0;

	/* Start and end overhead times always present */
	*microsec = start_overhead_us + end_overhead_us;

	if (get_sequence_step_enables(dev, &scheduler_sequence_steps)) {
		return 1;
	}

	if (scheduler_sequence_steps.tcc_on  ||
		scheduler_sequence_steps.msrc_on ||
		scheduler_sequence_steps.dss_on) {

		if (get_sequence_step_timeout(dev, VL53L0X_SEQUENCESTEP_MSRC, &msrc_dcc_tcc_timeout_us)) {
			return 1;
		}

		if (scheduler_sequence_steps.tcc_on) {
			*microsec += msrc_dcc_tcc_timeout_us + tcc_overhead_us;
		}

		if (scheduler_sequence_steps.dss_on) {
			*microsec += 2 * (msrc_dcc_tcc_timeout_us + dss_overhead_us);
		} else if (scheduler_sequence_steps.msrc_on) {
			*microsec += msrc_dcc_tcc_timeout_us + msrc_overhead_us;
		}

	}

	if (scheduler_sequence_steps.pre_range_on) {
		if (get_sequence_step_timeout(dev, VL53L0X_SEQUENCESTEP_PRE_RANGE, &pre_range_timeout_us)) {
			return 1;
		}

		*microsec += pre_range_timeout_us + pre_range_overhead_us;
	}

	if (scheduler_sequence_steps.final_range_on) {
		if (get_sequence_step_timeout(dev, VL53L0X_SEQUENCESTEP_FINAL_RANGE, &final_range_timeout_us)) {
			return 1;
		}

		*microsec += (final_range_timeout_us + final_range_overhead_us);
	}

	dev->cur_param.measurement_timing_budget_microseconds = *microsec;
	return 0;
}

/* Returns 0 if OK */
static int get_device_parameters(vl53l0x_dev_t *dev)
{
	int i;

	dev->cur_param.xtalk_compensation_enable = 0;

	if (get_inter_measure_period(dev, &(dev->cur_param.inter_measurement_period_milliseconds))) {
		return 1;
	}

	if (get_xtalk_compensation_rate_MegaCps(dev, &(dev->cur_param.xtalk_compensation_rate_mega_cps))) {
		return 1;
	}

	if (get_offset_calibration_data_micro_meter(dev, &(dev->cur_param.range_offset_micrometers))) {
		return 1;
	}

	for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
		if (get_limit_check_value(dev, i, &(dev->cur_param.limit_checks_value[i]))) {
			return 1;
		}

		if (get_limit_check_enable(dev, i, &(dev->cur_param.limit_checks_enable[i]))) {
			return 1;
		}
	}

	if (get_wrap_around_check_enable(dev, &(dev->cur_param.wrap_around_check_enable))) {
		return 1;
	}

	/* Need to be done at the end as it uses VCSELPulsePeriod */
	if (get_measurement_timing_budget_micro_seconds(dev, &(dev->cur_param.measurement_timing_budget_microseconds))) {
		return 1;
	}

	return 0;
}

/* Returns 0 if OK */
static int set_sequence_step_enable(vl53l0x_dev_t *dev,
                                    uint8_t sequence_step_id,
                                    uint8_t sequence_step_enabled)
{
	uint8_t sequence_config = 0;
	uint8_t sequence_config_new = 0;
	uint32_t measurement_timing_budget_us = 0;

	sequence_config = dev->ll->i2c_read_reg(SYSTEM_SEQUENCE_CONFIG);
	sequence_config_new = sequence_config;

	if (sequence_step_enabled == 1) { /* Enable requested sequence step */
		switch (sequence_step_id) {
		case VL53L0X_SEQUENCESTEP_TCC:
			sequence_config_new |= 0x10;
			break;
		case VL53L0X_SEQUENCESTEP_DSS:
			sequence_config_new |= 0x28;
			break;
		case VL53L0X_SEQUENCESTEP_MSRC:
			sequence_config_new |= 0x04;
			break;
		case VL53L0X_SEQUENCESTEP_PRE_RANGE:
			sequence_config_new |= 0x40;
			break;
		case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
			sequence_config_new |= 0x80;
			break;
		default:
			return 1;
		}
	} else {
		switch (sequence_step_id) { /* Disable requested sequence step */
		case VL53L0X_SEQUENCESTEP_TCC:
			sequence_config_new &= 0xef;
			break;
		case VL53L0X_SEQUENCESTEP_DSS:
			sequence_config_new &= 0xd7;
			break;
		case VL53L0X_SEQUENCESTEP_MSRC:
			sequence_config_new &= 0xfb;
			break;
		case VL53L0X_SEQUENCESTEP_PRE_RANGE:
			sequence_config_new &= 0xbf;
			break;
		case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
			sequence_config_new &= 0x7f;
			break;
		default:
			return 1;
		}
	}

	if (sequence_config_new != sequence_config) {
		/* Apply New Setting */
		dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, sequence_config_new);
		dev->sequence_config = sequence_config_new;

		/* Recalculate timing budget */
		if (set_measurement_timing_budget_micro_seconds(dev, measurement_timing_budget_us)) {
			return 1;
		}
	}

	return 0;
}

static int data_init(vl53l0x_dev_t *dev)
{
	/* USE_I2C_2V8 */
	dev->ll->i2c_write_reg(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
		(dev->ll->i2c_read_reg(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) & 0xFE) | 0x01);

	dev->ll->i2c_write_reg(0x88, 0x00); /* Set I2C standard mode (400KHz) */

	dev->spec_param.linearity_corrective_gain = 1000;
	dev->spec_param.osc_frequency_MHz = 618660;
	dev->cur_param.xtalk_compensation_rate_mega_cps = 0;

	if (get_device_parameters(dev)) {
		return 1;
	}

	dev->cur_param.device_mode = VL53L0X_SINGLE_RANGING;
	dev->spec_param.sigma_est_ref_array = 100;
	dev->spec_param.sigma_est_eff_pulse_width = 900;
	dev->spec_param.sigma_est_eff_amb_width = 500;
	dev->spec_param.target_ref_rate = 0x0A00; /* 20 MCPS in 9:7 format */

	dev->ll->i2c_write_reg(0x80, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x00);
	dev->__stop_variable = dev->ll->i2c_read_reg(0x91);
	dev->ll->i2c_write_reg(0x00, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x80, 0x00);

	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

	return 0;
}

static void get_dev_info_from_device(vl53l0x_dev_t *dev)
{
	uint8_t byte;
	uint32_t tmp_32bit_word;
	uint8_t nvm_ref_good_spad_map[6];

	/* Start */
	dev->ll->i2c_write_reg(0x80, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x00);

	dev->ll->i2c_write_reg(0xFF, 0x06);
	byte = dev->ll->i2c_read_reg(0x83);
	dev->ll->i2c_write_reg(0x83, byte | 4);
	dev->ll->i2c_write_reg(0xFF, 0x07);
	dev->ll->i2c_write_reg(0x81, 0x01);
	dev->ll->delay_ms(2); /* polling delay */
	dev->ll->i2c_write_reg(0x80, 0x01);

	/* option 1 */
	dev->ll->i2c_write_reg(0x94, 0x6b);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);

	dev->spec_param.reference_spad_count = (uint8_t)((tmp_32bit_word >> 8) & 0x07f);
	dev->spec_param.reference_spad_type  = (uint8_t)((tmp_32bit_word >> 15) & 0x01);

	dev->ll->i2c_write_reg(0x94, 0x24);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);

	nvm_ref_good_spad_map[0] = (uint8_t)((tmp_32bit_word >> 24) & 0xff);
	nvm_ref_good_spad_map[1] = (uint8_t)((tmp_32bit_word >> 16) & 0xff);
	nvm_ref_good_spad_map[2] = (uint8_t)((tmp_32bit_word >> 8) & 0xff);
	nvm_ref_good_spad_map[3] = (uint8_t)(tmp_32bit_word & 0xff);

	dev->ll->i2c_write_reg(0x94, 0x25);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);

	nvm_ref_good_spad_map[4] = (uint8_t)((tmp_32bit_word >> 24) & 0xff);
	nvm_ref_good_spad_map[5] = (uint8_t)((tmp_32bit_word >> 16) & 0xff);

	for (int i = 0; i < 6; i++) {
		dev->spad_data.ref_good_spad_map[i] = nvm_ref_good_spad_map[i];
	}

	/* option 2 */
	dev->ll->i2c_write_reg(0x94, 0x02);
	read_strobe(dev);
	dev->spec_param.module_id = dev->ll->i2c_read_reg(0x90);

	dev->ll->i2c_write_reg(0x94, 0x7B);
	read_strobe(dev);
	dev->spec_param.revision = dev->ll->i2c_read_reg(0x90);

	dev->ll->i2c_write_reg(0x94, 0x77);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);

	dev->spec_param.product_id[0] = (char)((tmp_32bit_word >> 25) & 0x07f);
	dev->spec_param.product_id[1] = (char)((tmp_32bit_word >> 18) & 0x07f);
	dev->spec_param.product_id[2] = (char)((tmp_32bit_word >> 11) & 0x07f);
	dev->spec_param.product_id[3] = (char)((tmp_32bit_word >> 4) & 0x07f);

	byte = (uint8_t)((tmp_32bit_word & 0x00f) << 3);
	dev->ll->i2c_write_reg(0x94, 0x78);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);

	dev->spec_param.product_id[4] = (char)(byte +
			((tmp_32bit_word >> 29) & 0x07f));
	dev->spec_param.product_id[5] = (char)((tmp_32bit_word >> 22) & 0x07f);
	dev->spec_param.product_id[6] = (char)((tmp_32bit_word >> 15) & 0x07f);
	dev->spec_param.product_id[7] = (char)((tmp_32bit_word >> 8) & 0x07f);
	dev->spec_param.product_id[8] = (char)((tmp_32bit_word >> 1) & 0x07f);

	byte = (uint8_t)((tmp_32bit_word & 0x001) << 6);
	dev->ll->i2c_write_reg(0x94, 0x79);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);

	dev->spec_param.product_id[9] = (char)(byte +
			((tmp_32bit_word >> 26) & 0x07f));
	dev->spec_param.product_id[10] = (char)((tmp_32bit_word >> 19) & 0x07f);
	dev->spec_param.product_id[11] = (char)((tmp_32bit_word >> 12) & 0x07f);
	dev->spec_param.product_id[12] = (char)((tmp_32bit_word >> 5) & 0x07f);

	byte = (uint8_t)((tmp_32bit_word & 0x01f) << 2);
	dev->ll->i2c_write_reg(0x94, 0x7A);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);

	dev->spec_param.product_id[13] = (char)(byte +
			((tmp_32bit_word >> 30) & 0x07f));
	dev->spec_param.product_id[14] = (char)((tmp_32bit_word >> 23) & 0x07f);
	dev->spec_param.product_id[15] = (char)((tmp_32bit_word >> 16) & 0x07f);
	dev->spec_param.product_id[16] = (char)((tmp_32bit_word >> 9) & 0x07f);
	dev->spec_param.product_id[17] = (char)((tmp_32bit_word >> 2) & 0x07f);
	dev->spec_param.product_id[18] = '\0';

	/* option 4 */
	dev->ll->i2c_write_reg(0x94, 0x7B);
	read_strobe(dev);
	dev->spec_param.part_uid_upper = dev->ll->i2c_read_reg_32bit(0x90);

	dev->ll->i2c_write_reg(0x94, 0x7C);
	read_strobe(dev);
	dev->spec_param.part_uid_lower = dev->ll->i2c_read_reg_32bit(0x90);

	dev->ll->i2c_write_reg(0x94, 0x73);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);
	dev->__signal_rate_meas_fixed1104_400_mm = (tmp_32bit_word & 0x0000000ff) << 8;

	dev->ll->i2c_write_reg(0x94, 0x74);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);
	dev->__signal_rate_meas_fixed1104_400_mm |= ((tmp_32bit_word & 0xff000000) >> 24);

	dev->ll->i2c_write_reg(0x94, 0x75);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);
	dev->__dist_meas_fixed1104_400_mm = (tmp_32bit_word & 0x0000000ff) << 8;

	dev->ll->i2c_write_reg(0x94, 0x76);
	read_strobe(dev);
	tmp_32bit_word = dev->ll->i2c_read_reg_32bit(0x90);
	dev->__dist_meas_fixed1104_400_mm |= ((tmp_32bit_word & 0xff000000) >> 24);

	/* Finish */
	dev->ll->i2c_write_reg(0x81, 0x00);
	dev->ll->i2c_write_reg(0xFF, 0x06);
	byte = dev->ll->i2c_read_reg(0x83);
	dev->ll->i2c_write_reg(0x83, byte & 0xfb);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x80, 0x00);
}

static uint8_t is_aperture(uint32_t spad_index)
{
	#define REF_ARRAY_SPAD_0  0
	#define REF_ARRAY_SPAD_5  5
	#define REF_ARRAY_SPAD_10 10

	static uint32_t ref_array_quadrants[4] = {
		REF_ARRAY_SPAD_10,
		REF_ARRAY_SPAD_5,
		REF_ARRAY_SPAD_0,
		REF_ARRAY_SPAD_5
	};

	/*
	 * This function reports if a given spad index is an aperture SPAD by
	 * deriving the quadrant.
	 */
	uint32_t quadrant;
	uint8_t is_aper = 1;

	quadrant = spad_index >> 6;
	if (ref_array_quadrants[quadrant] == REF_ARRAY_SPAD_0)
		is_aper = 0;

	return is_aper;
}

void get_next_good_spad(uint8_t good_spad_array[], uint32_t size,
			uint32_t curr, int32_t *next)
{
	uint32_t start_index;
	uint32_t fine_offset;
	uint32_t c_spads_per_byte = 8;
	uint32_t coarse_index;
	uint32_t fine_index;
	uint8_t data_byte;
	uint8_t success = 0;

	/*
	 * Starting with the current good spad, loop through the array to find
	 * the next. i.e. the next bit set in the sequence.
	 *
	 * The coarse index is the byte index of the array and the fine index is
	 * the index of the bit within each byte.
	 */

	*next = -1;

	start_index = curr / c_spads_per_byte;
	fine_offset = curr % c_spads_per_byte;

	for (coarse_index = start_index; ((coarse_index < size) && !success);
				coarse_index++) {
		fine_index = 0;
		data_byte = good_spad_array[coarse_index];

		if (coarse_index == start_index) {
			/* locate the bit position of the provided current
			 * spad bit before iterating
			 */
			data_byte >>= fine_offset;
			fine_index = fine_offset;
		}

		while (fine_index < c_spads_per_byte) {
			if ((data_byte & 0x1) == 1) {
				success = 1;
				*next = coarse_index * c_spads_per_byte + fine_index;
				break;
			}
			data_byte >>= 1;
			fine_index++;
		}
	}
}

/* Returns 0 if OK */
static int enable_spad_bit(uint8_t spad_array[], uint32_t size, uint32_t spad_index)
{
	uint32_t c_spads_per_byte = 8;
	uint32_t coarse_index;
	uint32_t fineIndex;

	coarse_index = spad_index / c_spads_per_byte;
	fineIndex = spad_index % c_spads_per_byte;
	if (coarse_index >= size)
		return 1;
	else
		spad_array[coarse_index] |= (1 << fineIndex);

	return 0;
}

static int set_ref_spad_map(vl53l0x_dev_t *dev, uint8_t *ref_spad_arr)
{
	dev->ll->i2c_write_reg_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_arr, 6);
	return 0;
}

static int get_ref_spad_map(vl53l0x_dev_t *dev, uint8_t *ref_spad_arr)
{
	dev->ll->i2c_read_reg_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_arr, 6);
	return 0;
}

/* Returns 0 if OK */
static int enable_ref_spads(vl53l0x_dev_t *dev,
                            uint8_t aperture_spads,
                            uint8_t good_spad_array[],
                            uint8_t spad_array[],
                            uint32_t size,
                            uint32_t start,
                            uint32_t offset,
                            uint32_t spad_count,
                            uint32_t *lastSpad)
{
	uint32_t i;
	uint32_t index;
	int32_t next_good_spad = offset;
	uint32_t current_spad;
	uint8_t check_spad_array[6];

	/*
	 * This function takes in a spad array which may or may not have SPADS
	 * already enabled and appends from a given offset a requested number
	 * of new SPAD enables. The 'good spad map' is applied to
	 * determine the next SPADs to enable.
	 *
	 * This function applies to only aperture or only non-aperture spads.
	 * Checks are performed to ensure this.
	 */

	current_spad = offset;
	for (index = 0; index < spad_count; index++) {
		get_next_good_spad(good_spad_array, size, current_spad, &next_good_spad);

		if (next_good_spad == -1) {
			return 1;
		}

		/* Confirm that the next good SPAD is non-aperture */
		if (is_aperture(start + next_good_spad) != aperture_spads) {
			/* if we can't get the required number of good aperture
			 * spads from the current quadrant then this is an error
			 */
			return 1;
		}
		current_spad = (uint32_t)next_good_spad;
		enable_spad_bit(spad_array, size, current_spad);
		current_spad++;
	}
	*lastSpad = current_spad;

	set_ref_spad_map(dev, spad_array);
	get_ref_spad_map(dev, check_spad_array);

	i = 0;

	/* Compare spad maps. If not equal report error. */
	while (i < size) {
		if (spad_array[i] != check_spad_array[i]) {
			return 1;
		}
		i++;
	}

	return 0;
}

/*
 * Based on VL53L0X_set_reference_spads
 * Returns 0 if OK
 **/
static int set_reference_spads(vl53l0x_dev_t *dev,
                               uint32_t count,
                               uint8_t is_aperture_spads)
{
	uint32_t current_spad_index = 0;
	uint8_t start_select = 0xB4;
	uint32_t spad_array_size = 6;
	uint32_t max_spad_count = 44;
	uint32_t last_spad_index;
	uint32_t index;

	/*
	 * This function applies a requested number of reference spads, either
	 * aperture or non-aperture, as requested. The good spad map will be applied.
	 */

	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	dev->ll->i2c_write_reg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(GLOBAL_CONFIG_REF_EN_START_SELECT, start_select);

	for (index = 0; index < spad_array_size; index++) {
		dev->spad_data.ref_spad_enables[index] = 0;
	}

	if (is_aperture_spads) {
		/* Increment to the first APERTURE spad */
		while ((is_aperture(start_select + current_spad_index) == 0) &&
			   (current_spad_index < max_spad_count)) {
			current_spad_index++;
		}
	}

	if (enable_ref_spads(dev, is_aperture_spads,
				dev->spad_data.ref_good_spad_map,
				dev->spad_data.ref_spad_enables,
				spad_array_size, start_select,
				current_spad_index, count, &last_spad_index)) {
		return 1;
	}


	dev->spec_param.reference_spad_count = (uint8_t)(count);
	dev->spec_param.reference_spad_type = is_aperture_spads;
	return 0;
}

/* Return 0 if OK */
static int perform_ref_spad_management(vl53l0x_dev_t *dev,
									   uint32_t *ref_spad_count,
									   uint8_t *is_aperture_spads)
{
	return 1;
}

/* return 0 if OK */
static int load_tuning_settings(vl53l0x_dev_t *dev, uint8_t *settings_buf)
{
	int i = 0;
	int index = 0;
	uint8_t msb = 0;
	uint8_t lsb = 0;
	uint8_t select_param = 0;
	uint8_t number_of_writes = 0;
	uint8_t address = 0;
	uint8_t local_buffer[4] = { 0 };
	uint16_t temp16 = 0;

	while (*(settings_buf + index) != 0) {
		number_of_writes = *(settings_buf + index);
		index++;
		if (number_of_writes == 0xFF) {
			/* internal parameters */
			select_param = *(settings_buf + index);
			index++;
			switch (select_param) {
			case 0: /* uint16_t spec_param.sigma_est_ref_array -> 2 bytes */
				msb = *(settings_buf + index);
				index++;
				lsb = *(settings_buf + index);
				index++;
				temp16 = VL53L0X_MAKEUINT16(lsb, msb);
				dev->spec_param.sigma_est_ref_array = temp16;
				break;
			case 1: /* uint16_t spec_param.sigma_est_eff_pulse_width -> 2 bytes */
				msb = *(settings_buf + index);
				index++;
				lsb = *(settings_buf + index);
				index++;
				temp16 = VL53L0X_MAKEUINT16(lsb, msb);
				dev->spec_param.sigma_est_eff_pulse_width = temp16;
				break;
			case 2: /* uint16_t spec_param.sigma_est_eff_amb_width -> 2 bytes */
				msb = *(settings_buf + index);
				index++;
				lsb = *(settings_buf + index);
				index++;
				temp16 = VL53L0X_MAKEUINT16(lsb, msb);
				dev->spec_param.sigma_est_eff_amb_width = temp16;
				break;
			case 3: /* uint16_t spec_param.target_ref_rate -> 2 bytes */
				msb = *(settings_buf + index);
				index++;
				lsb = *(settings_buf + index);
				index++;
				temp16 = VL53L0X_MAKEUINT16(lsb, msb);
				dev->spec_param.target_ref_rate = temp16;
				break;
			default: /* invalid parameter */
				return 1;
			}
		} else if (number_of_writes <= 4) {
			address = *(settings_buf + index);
			index++;

			for (i = 0; i < number_of_writes; i++) {
				local_buffer[i] = *(settings_buf + index);
				index++;
			}

			dev->ll->i2c_write_reg_multi(address, local_buffer, number_of_writes);
		} else {
			return 1;
		}
	}

	return 0;
}

static int get_fraction_enable(vl53l0x_dev_t *dev, uint8_t *en)
{
	*en = dev->ll->i2c_read_reg(SYSTEM_RANGE_CONFIG);
	*en = (*en & 1);

	return 0;
}

/* Returns 0 if OK */
static int perform_vhv_calibration(vl53l0x_dev_t *dev)
{
	/* Run VHV */
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0x01);

	/* VL53L0X_perform_single_ref_calibration() */
	dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP | 0x40);
	while (get_measurement_data_ready(dev)) {
		dev->ll->delay_ms(10);
	}

	vl53l0x_clear_flag_gpio_interrupt(dev);
	dev->ll->i2c_write_reg(SYSRANGE_START, 0x00);
	return 0;
}

/* Returns 0 if OK */
static int perform_phase_calibration(vl53l0x_dev_t *dev)
{
	/* Run PhaseCal */
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);

	/* VL53L0X_perform_single_ref_calibration() */
	dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP | 0x40);
	while (get_measurement_data_ready(dev)) {
		dev->ll->delay_ms(10);
	}

	vl53l0x_clear_flag_gpio_interrupt(dev);
	dev->ll->i2c_write_reg(SYSRANGE_START, 0x00);
	return 0;
}
/* Returns 0 if OK */
static int perform_ref_calibration(vl53l0x_dev_t *dev,
                                   uint8_t *vhv_settings,
                                   uint8_t *phase_cal,
                                   uint8_t get_data_enable)
{
	uint8_t sequence_config = dev->sequence_config;

	/* In the following function we don't save the config to optimize
	 * writes on device. Config is saved and restored only once.
	 */
	if (perform_vhv_calibration(dev)) {
		return 1;
	}

	if (perform_phase_calibration(dev)) {
		return 1;
	}

	/* restore the previous Sequence Config */
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, sequence_config);
	dev->sequence_config = sequence_config;

	return 0;
}

/* Return 0 if OK */
static int static_init(vl53l0x_dev_t *dev)
{
	uint32_t ref_spad_count = 0;
	uint8_t aperture_spads = 0;

	get_dev_info_from_device(dev);

	/* NVM value invalid */
	if ((dev->spec_param.reference_spad_type > 1) ||
		((dev->spec_param.reference_spad_type == 1) && (dev->spec_param.reference_spad_count > 32)) ||
		((dev->spec_param.reference_spad_type == 0) && (dev->spec_param.reference_spad_count > 12))) {
		if (perform_ref_spad_management(dev, &ref_spad_count, &aperture_spads)) {
			return 1;
		}
	} else {
		if (set_reference_spads(dev,
			dev->spec_param.reference_spad_count,
			dev->spec_param.reference_spad_type)) {
			return 1;
		}
	}

	/* Setup tuning settings */
	if (load_tuning_settings(dev, default_tuning_settings)) {
		return 1;
	}

	/* Set interrupt config to new sample ready, low polarity */
	dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO,
						   VL53L0X_GPIO_FUNC_NEW_MEASURE_READY);
	dev->ll->i2c_write_reg(GPIO_HV_MUX_ACTIVE_HIGH,
			(dev->ll->i2c_read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & 0xEF) | 0);
	dev->spec_param.gpio_func = VL53L0X_GPIO_FUNC_NEW_MEASURE_READY;
	vl53l0x_clear_flag_gpio_interrupt(dev);

	/* Get internal freq */
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->spec_param.osc_frequency_MHz = VL53L0X_FIXPOINT412TOFIXPOINT1616(
						dev->ll->i2c_read_reg_16bit(0x84));
	dev->ll->i2c_write_reg(0xFF, 0x00);

	/* After static init, some device parameters may be changed, so update them */
	if (get_device_parameters(dev)) {
		return 1;
	}

	if (get_fraction_enable(dev, &(dev->range_fractional_enable))) {
		return 1;
	}

	/* read the sequence config and save it */
	dev->sequence_config = dev->ll->i2c_read_reg(SYSTEM_SEQUENCE_CONFIG);

	/* Disable MSRC and TCC by default */
	if (set_sequence_step_enable(dev, VL53L0X_SEQUENCESTEP_TCC, 0)) {
		return 1;
	}

	if (set_sequence_step_enable(dev, VL53L0X_SEQUENCESTEP_MSRC, 0)) {
		return 1;
	}

	/* ToDo: Store pre-range vcsel period  in dev structure */

	return 0;
}

vl53l0x_ret_t vl53l0x_init(vl53l0x_dev_t *dev)
{
    uint8_t vhv_settings;
    uint8_t phase_cal;

	if (check_args(dev)) {
		return VL53L0X_FAIL;
	}

	/* HW reset */
	dev->ll->delay_ms(100);
	vl53l0x_shutdown(dev);
	dev->ll->delay_ms(100);
	vl53l0x_power_up(dev);

	/* Wait 1.2 ms max (according to the spec) until vl53l0x fw boots */
	dev->ll->delay_ms(2);

	if (check_i2c_comm(dev)) {
		return VL53L0X_FAIL;
	}

	if (data_init(dev)) {
		return VL53L0X_FAIL;
	}

	if (static_init(dev)) {
		return VL53L0X_FAIL;
	}

	if (perform_ref_calibration(dev, &vhv_settings, &phase_cal, 1)) {
		return VL53L0X_FAIL;
	}

	if (set_measurement_timing_budget_micro_seconds(dev, 30000)) {
		return VL53L0X_FAIL;
	}

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

vl53l0x_ret_t vl53l0x_set_measurement_mode(vl53l0x_dev_t *dev,
								vl53l0x_measure_mode_t mode,
								uint32_t ms)
{
	switch (mode) {
	case VL53L0X_SINGLE_RANGING:
	case VL53L0X_CONTINUOUS_RANGING:
	case VL53L0X_CONTINUOUS_TIMED_RANGING:
		break;

	default:
		return VL53L0X_FAIL;
	}

	dev->cur_param.device_mode = mode;
	//dev->cur_param.inter_measurement_period_milliseconds = ms;

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_start_measurement(vl53l0x_dev_t *dev)
{
	uint8_t byte = 0xff;
	uint8_t start_stop_byte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
	int timeout_cycles = 0;

	dev->ll->i2c_write_reg(0x80, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x00);
	dev->ll->i2c_write_reg(0x91, dev->__stop_variable);
	dev->ll->i2c_write_reg(0x00, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x80, 0x00);

	switch (dev->cur_param.device_mode) {
	case VL53L0X_SINGLE_RANGING:
		dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP);

		while ((byte & start_stop_byte) == start_stop_byte) {
			if (timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP) {
				return VL53L0X_FAIL;
			}
			++timeout_cycles;

			byte = dev->ll->i2c_read_reg(SYSRANGE_START);
		}
		break;

	case VL53L0X_CONTINUOUS_RANGING:
		/* continuous back-to-back mode */
		dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);
		break;

	case VL53L0X_CONTINUOUS_TIMED_RANGING:
		dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_TIMED);
		break;

	default:
		return VL53L0X_FAIL;
	}

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_stop_measurement(vl53l0x_dev_t *dev)
{
	dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x00);
	dev->ll->i2c_write_reg(0x91, 0x00);
	dev->ll->i2c_write_reg(0x00, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_activate_gpio_interrupt(vl53l0x_dev_t *dev)
{
	/*
	 * Set interrupt config to new sample ready
	 * See VL53L0X_SetGpioConfig() ST API func.
	 **/
	dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO,
			VL53L0X_GPIO_FUNC_NEW_MEASURE_READY);
	dev->ll->i2c_write_reg(GPIO_HV_MUX_ACTIVE_HIGH,
			dev->ll->i2c_read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); /* polarity low */

	dev->spec_param.gpio_func = VL53L0X_GPIO_FUNC_NEW_MEASURE_READY;

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_deactivate_gpio_interrupt(vl53l0x_dev_t *dev)
{
	dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO,
				VL53L0X_GPIO_FUNC_OFF);
	dev->spec_param.gpio_func = VL53L0X_GPIO_FUNC_OFF;

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_clear_flag_gpio_interrupt(vl53l0x_dev_t *dev)
{
	uint8_t byte = 0xff;
	int cycles = 0;

	while ((byte & 0x07) != 0x00) {
		dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
		dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x00);
		byte = dev->ll->i2c_read_reg(RESULT_INTERRUPT_STATUS);

		if (cycles >= VL53L0X_DEFAULT_MAX_LOOP) {
			return VL53L0X_FAIL;
		}
		++cycles;
	}

	return VL53L0X_OK;
}

/* Based on VL53L0X_PerformSingleRangingMeasurement() */
vl53l0x_ret_t vl53l0x_get_range_mm_oneshot(vl53l0x_dev_t *dev, vl53l0x_range *range)
{
	/* VL53L0X_PerformSingleMeasurement */
	dev->cur_param.device_mode = VL53L0X_SINGLE_RANGING;
	vl53l0x_start_measurement(dev);

	/*
	 * Get data ready.
	 * Based on VL53L0X_measurement_poll_for_completion()
	 **/
	while (get_measurement_data_ready(dev)) {
		/*
		 * If user don't apply some settings regarding budget time, the
		 * range sensor will do one measurements per ±16 seconds.
		 * It was found out during experiments.
		 **/
		dev->ll->delay_ms(10);
	}

	/* Based on VL53L0X_GetRangingMeasurementData() */
	range->range_mm = dev->ll->i2c_read_reg_16bit(0x14 + 10);

	if (dev->spec_param.gpio_func == VL53L0X_GPIO_FUNC_NEW_MEASURE_READY) {
		vl53l0x_clear_flag_gpio_interrupt(dev);
	}

	return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_get_range_mm_continuous(vl53l0x_dev_t *dev, vl53l0x_range *range)
{
	range->range_mm = dev->ll->i2c_read_reg_16bit(0x14 + 10);
	return VL53L0X_OK;
}
