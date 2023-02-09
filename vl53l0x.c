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

/*
 * Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
 * based on VL53L0X_calc_macro_period_ps()
 * PLL_period_ps = 1655; macro_period_vclks = 2304
 **/
#define CALC_MACRO_PERIOD(vcsel_period_pclks)	((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/*
 * Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
 * from register value based on VL53L0X_decode_vcsel_period()
 **/
#define DECODE_VCSEL_PERIOD(reg_val)	(((reg_val) + 1) << 1)

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

/*
 * TCC: Target CentreCheck
 * MSRC: Minimum Signal Rate Check
 * DSS: Dynamic Spad Selection
 **/
typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
} sequence_step_enables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} sequence_step_timeouts;


typedef enum {
	vcsel_period_pre_range,
	vcsel_period_final_range
} vcsel_period_type;

// returns 0 if OK
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

/*
 * Get reference SPAD (single photon avalanche diode) count and type
 * based on VL53L0X_get_info_from_device(), but only gets reference
 * SPAD count and type.
 *
 * Returns 0 if OK
 **/
static int get_spad_info(vl53l0x_dev_t *dev,
					uint8_t *count, int *type_is_aperture)
{
	uint8_t tmp = 0;
	int timeout_cycles = 0;

	dev->ll->i2c_write_reg(0x80, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x00);
	dev->ll->i2c_write_reg(0xFF, 0x06);
	dev->ll->i2c_write_reg(0x83, dev->ll->i2c_read_reg(0x83) | 0x04);
	dev->ll->i2c_write_reg(0xFF, 0x07);
	dev->ll->i2c_write_reg(0x81, 0x01);
	dev->ll->i2c_write_reg(0x80, 0x01);
	dev->ll->i2c_write_reg(0x94, 0x6b);
	dev->ll->i2c_write_reg(0x83, 0x00);

	while (dev->ll->i2c_read_reg(0x83) == 0x00) {
		dev->ll->delay_ms(50);
		if (timeout_cycles >= 20) {
			return 1;
		}
		++timeout_cycles;
	}
	dev->ll->i2c_write_reg(0x83, 0x01);
	tmp = dev->ll->i2c_read_reg(0x92);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	dev->ll->i2c_write_reg(0x81, 0x00);
	dev->ll->i2c_write_reg(0xFF, 0x06);
	dev->ll->i2c_write_reg(0x83, dev->ll->i2c_read_reg(0x83) & ~0x04);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x80, 0x00);

	return 0;
}

/* Get sequence step enables based on VL53L0X_GetSequenceStepEnables() */
static void get_sequence_step_enables(vl53l0x_dev_t *dev, sequence_step_enables *enables)
{
	uint8_t sequence_config = dev->ll->i2c_read_reg(SYSTEM_SEQUENCE_CONFIG);

	enables->tcc          = (sequence_config >> 4) & 0x1;
	enables->dss          = (sequence_config >> 3) & 0x1;
	enables->msrc         = (sequence_config >> 2) & 0x1;
	enables->pre_range    = (sequence_config >> 6) & 0x1;
	enables->final_range  = (sequence_config >> 7) & 0x1;
}

/*
 * Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
 * based on VL53L0X_calc_timeout_us()
 **/
static uint32_t timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = CALC_MACRO_PERIOD(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

/*
 * Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
 * based on VL53L0X_calc_timeout_mclks()
 **/
static uint32_t timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = CALC_MACRO_PERIOD(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

/*
 * Encode sequence step timeout register value from timeout in MCLKs
 * based on VL53L0X_encode_timeout()
 * Note: the original function took a uint16_t, but the argument passed to it
 * is always a uint16_t.
 **/
static uint16_t encode_timeout(uint16_t timeout_mclks)
{
	/* format: (LSByte * 2^MSByte) + 1 */
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0)
		{
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);
	} else {
		return 0;
	}
}

/*
 * Decode sequence step timeout in MCLKs from register value
 * based on VL53L0X_decode_timeout()
 * Note: the original function returned a uint32_t, but the return value is
 * always stored in a uint16_t.
 **/
static uint16_t decode_timeout(uint16_t reg_val)
{
  /* format: (LSByte * 2^MSByte) + 1 */
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

/*
 * Get the VCSEL pulse period in PCLKs for the given period type.
 * based on VL53L0X_get_vcsel_pulse_period()
 **/
static uint8_t get_vcsel_pulse_period(vl53l0x_dev_t *dev, vcsel_period_type type)
{
	if (type == vcsel_period_pre_range) {
		return DECODE_VCSEL_PERIOD(dev->ll->i2c_read_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
	} else if (type == vcsel_period_final_range) {
		return DECODE_VCSEL_PERIOD(dev->ll->i2c_read_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	} else {
		return 255;
	}
}

/*
 * Get sequence step timeouts
 * based on get_sequence_step_timeout(),
 * but gets all timeouts instead of just the requested one, and also stores
 * intermediate values.
 **/
static void get_sequence_step_timeouts(vl53l0x_dev_t *dev,
					sequence_step_enables const *enables,
					sequence_step_timeouts *timeouts)
{
	timeouts->pre_range_vcsel_period_pclks = get_vcsel_pulse_period(dev,
							vcsel_period_pre_range);

	timeouts->msrc_dss_tcc_mclks = dev->ll->i2c_read_reg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	timeouts->msrc_dss_tcc_us =
	timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks,
								timeouts->pre_range_vcsel_period_pclks);

	timeouts->pre_range_mclks =
	decode_timeout(dev->ll->i2c_read_reg_16bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	timeouts->pre_range_us =
	timeout_mclks_to_microseconds(timeouts->pre_range_mclks,
								timeouts->pre_range_vcsel_period_pclks);

	timeouts->final_range_vcsel_period_pclks = get_vcsel_pulse_period(dev,
							vcsel_period_final_range);

	timeouts->final_range_mclks =
	decode_timeout(dev->ll->i2c_read_reg_16bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	if (enables->pre_range) {
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	}

	timeouts->final_range_us =
	timeout_mclks_to_microseconds(timeouts->final_range_mclks,
								timeouts->final_range_vcsel_period_pclks);
}

/*
 * Get the measurement timing budget in microseconds
 * based on VL53L0X_get_measurement_timing_budget_micro_seconds() in us.
 **/
static uint32_t get_measurement_timing_budget(vl53l0x_dev_t *dev)
{
	sequence_step_enables enables = { 0 };
	sequence_step_timeouts timeouts = { 0 };

	uint16_t const start_overhead       = 1910;
	uint16_t const end_overhead         = 960;
	uint16_t const msrc_overhead        = 660;
	uint16_t const tcc_overhead         = 590;
	uint16_t const dss_overhead         = 690;
	uint16_t const pre_range_overhead   = 660;
	uint16_t const final_range_overhead = 550;

	/* Start and end overhead times always present */
	uint32_t budget_us = start_overhead + end_overhead;

	get_sequence_step_enables(dev, &enables);
	get_sequence_step_timeouts(dev, &enables, &timeouts);

	if (enables.tcc) {
		budget_us += (timeouts.msrc_dss_tcc_us + tcc_overhead);
	}

	if (enables.dss) {
		budget_us += 2 * (timeouts.msrc_dss_tcc_us + dss_overhead);
	} else if (enables.msrc) {
		budget_us += (timeouts.msrc_dss_tcc_us + msrc_overhead);
	}

	if (enables.pre_range) {
		budget_us += (timeouts.pre_range_us + pre_range_overhead);
	}

	if (enables.final_range) {
		budget_us += (timeouts.final_range_us + final_range_overhead);
	}

	return budget_us;
}

/*
 * Set the measurement timing budget in microseconds, which is the time allowed
 * for one measurement; the ST API and this library take care of splitting the
 * timing budget among the sub-steps in the ranging sequence. A longer timing
 * budget allows for more accurate measurements. Increasing the budget by a
 * factor of N decreases the range measurement standard deviation by a factor of
 * sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
 * based on VL53L0X_set_measurement_timing_budget_micro_seconds()
 *
 * Returns 0 if OK
 **/
static int set_measurement_timing_budget(vl53l0x_dev_t *dev, uint32_t budget_us)
{
	sequence_step_enables enables = { 0 };
	sequence_step_timeouts timeouts = { 0 };

	uint16_t const start_overhead       = 1320;
	uint16_t const end_overhead         = 960;
	uint16_t const msrc_overhead        = 660;
	uint16_t const tcc_overhead         = 590;
	uint16_t const dss_overhead         = 690;
	uint16_t const pre_range_overhead   = 660;
	uint16_t const final_range_overhead = 550;

	if (budget_us < 20000) {
		return 1;
	}

	uint32_t used_budget_us = start_overhead + end_overhead;

	get_sequence_step_enables(dev, &enables);
	get_sequence_step_timeouts(dev, &enables, &timeouts);

	if (enables.tcc) {
		used_budget_us += (timeouts.msrc_dss_tcc_us + tcc_overhead);
	}

	if (enables.dss) {
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + dss_overhead);
	}
	else if (enables.msrc) {
		used_budget_us += (timeouts.msrc_dss_tcc_us + msrc_overhead);
	}

	if (enables.pre_range) {
		used_budget_us += (timeouts.pre_range_us + pre_range_overhead);
	}

	if (enables.final_range) {
		used_budget_us += final_range_overhead;

		/*
		 * "Note that the final range timeout is determined by the timing
		 * budget and the sum of all other timeouts within the sequence.
		 * If there is no room for the final range timeout, then an error
		 * will be set. Otherwise the remaining time will be applied to
		 * the final range."
		 **/
		if (used_budget_us > budget_us) {
			/* Requested timeout too big. */
			return 1;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;

		/*
		 * (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
		 * "For the final range timeout, the pre-range timeout
		 *  must be added. To do this both final and pre-range
		 *  timeouts must be expressed in macro periods MClks
		 *  because they have different vcsel periods."
		 **/
		uint16_t final_range_timeout_mclks =
			timeout_microseconds_to_mclks(final_range_timeout_us,
										timeouts.final_range_vcsel_period_pclks);

		if (enables.pre_range) {
			final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		dev->ll->i2c_write_reg_16bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
			encode_timeout(final_range_timeout_mclks));

		dev->__g_meas_time_bud_us = budget_us;
	}

	return 0;
}

/*
 * Based on VL53L0X_perform_single_ref_calibration()
 *
 * Returns 0 if OK
 **/
static int perform_single_ref_calibration(vl53l0x_dev_t *dev, uint8_t vhv_init_byte)
{
	int timeout_cycles = 0;
	dev->ll->i2c_write_reg(SYSRANGE_START, 0x01 | vhv_init_byte); /* VL53L0X_REG_SYSRANGE_MODE_START_STOP */

	while ((dev->ll->i2c_read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		dev->ll->delay_ms(50);
		if (timeout_cycles >= 20) {
			return 1;
		}
		++timeout_cycles;
	}

	dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	dev->ll->i2c_write_reg(SYSRANGE_START, 0x00);

	return 0;
}

vl53l0x_ret_t vl53l0x_init(vl53l0x_dev_t *dev)
{
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

	/*
	 * STM API VL53L0X_DataInit() function is called one time, and it performs
	 * the device initialization. To be called once and only once after
	 * device is brought out of reset.
	 *
	 * Make the same things here
	 **/
	dev->ll->i2c_write_reg(0x88, 0x00); /* Set I2C standard mode (400KHz) */
	dev->ll->i2c_write_reg(0x80, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x00);
	dev->__stop_variable = dev->ll->i2c_read_reg(0x91);
	dev->ll->i2c_write_reg(0x00, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x80, 0x00);

	/* Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks */
	dev->ll->i2c_write_reg(MSRC_CONFIG_CONTROL,
			dev->ll->i2c_read_reg(MSRC_CONFIG_CONTROL) | 0x12);

	/*
	* This represents the amplitude of the signal reflected from the
	* target and detected by the device"; setting this limit presumably determines
	* the minimum measurement necessary for the sensor to report a valid reading.
	* Setting a lower limit increases the potential range of the sensor but also
	* seems to increase the likelihood of getting an inaccurate reading because of
	* unwanted reflections from objects other than the intended target.
	* Defaults to 0.25 MCPS as initialized by the ST API and this library. (0.25 * (1 << 7))
	**/
	dev->ll->i2c_write_reg_16bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x20);
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

	/*
	 * STM API VL53L0X_StaticInit() function allows to load device settings
	 * specific for a given use case.
	 **/
	uint8_t spad_count;
	int spad_type_is_aperture;

	if (get_spad_info(dev, &spad_count, &spad_type_is_aperture)) {
		return VL53L0X_FAIL;
	}

	/*
	 * The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	 * the ST API, but the same data seems to be more easily readable from
	 * GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	 *
	 * https://github.com/yetifrisstlama/vl53l0x-non-arduino/blob/master/VL53L0X.c#L186
	 **/
	uint8_t ref_spad_map[6];
	dev->ll->i2c_read_reg_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	/* VL53L0X_set_reference_spads() begin (assume NVM values are valid) */
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	dev->ll->i2c_write_reg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	/* 12 is the first aperture spad */
	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++) {
		if (i < first_spad_to_enable || spads_enabled == spad_count) {
			/*
			 * This bit is lower than the first one that should be enabled, or
			 * (reference_spad_count) bits have already been enabled, so zero this bit
			 **/
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			spads_enabled++;
		}
	}
	dev->ll->i2c_write_reg_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	/*
	 * See ST API VL53L0X_load_tuning_settings() func.
	 * DefaultTuningSettings from vl53l0x_tuning.h
	 **/
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x00);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x09, 0x00);
	dev->ll->i2c_write_reg(0x10, 0x00);
	dev->ll->i2c_write_reg(0x11, 0x00);
	dev->ll->i2c_write_reg(0x24, 0x01);
	dev->ll->i2c_write_reg(0x25, 0xFF);
	dev->ll->i2c_write_reg(0x75, 0x00);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x4E, 0x2C);
	dev->ll->i2c_write_reg(0x48, 0x00);
	dev->ll->i2c_write_reg(0x30, 0x20);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x30, 0x09);
	dev->ll->i2c_write_reg(0x54, 0x00);
	dev->ll->i2c_write_reg(0x31, 0x04);
	dev->ll->i2c_write_reg(0x32, 0x03);
	dev->ll->i2c_write_reg(0x40, 0x83);
	dev->ll->i2c_write_reg(0x46, 0x25);
	dev->ll->i2c_write_reg(0x60, 0x00);
	dev->ll->i2c_write_reg(0x27, 0x00);
	dev->ll->i2c_write_reg(0x50, 0x06);
	dev->ll->i2c_write_reg(0x51, 0x00);
	dev->ll->i2c_write_reg(0x52, 0x96);
	dev->ll->i2c_write_reg(0x56, 0x08);
	dev->ll->i2c_write_reg(0x57, 0x30);
	dev->ll->i2c_write_reg(0x61, 0x00);
	dev->ll->i2c_write_reg(0x62, 0x00);
	dev->ll->i2c_write_reg(0x64, 0x00);
	dev->ll->i2c_write_reg(0x65, 0x00);
	dev->ll->i2c_write_reg(0x66, 0xA0);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x22, 0x32);
	dev->ll->i2c_write_reg(0x47, 0x14);
	dev->ll->i2c_write_reg(0x49, 0xFF);
	dev->ll->i2c_write_reg(0x4A, 0x00);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x7A, 0x0A);
	dev->ll->i2c_write_reg(0x7B, 0x00);
	dev->ll->i2c_write_reg(0x78, 0x21);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x23, 0x34);
	dev->ll->i2c_write_reg(0x42, 0x00);
	dev->ll->i2c_write_reg(0x44, 0xFF);
	dev->ll->i2c_write_reg(0x45, 0x26);
	dev->ll->i2c_write_reg(0x46, 0x05);
	dev->ll->i2c_write_reg(0x40, 0x40);
	dev->ll->i2c_write_reg(0x0E, 0x06);
	dev->ll->i2c_write_reg(0x20, 0x1A);
	dev->ll->i2c_write_reg(0x43, 0x40);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x34, 0x03);
	dev->ll->i2c_write_reg(0x35, 0x44);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x31, 0x04);
	dev->ll->i2c_write_reg(0x4B, 0x09);
	dev->ll->i2c_write_reg(0x4C, 0x05);
	dev->ll->i2c_write_reg(0x4D, 0x04);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x44, 0x00);
	dev->ll->i2c_write_reg(0x45, 0x20);
	dev->ll->i2c_write_reg(0x47, 0x08);
	dev->ll->i2c_write_reg(0x48, 0x28);
	dev->ll->i2c_write_reg(0x67, 0x00);
	dev->ll->i2c_write_reg(0x70, 0x04);
	dev->ll->i2c_write_reg(0x71, 0x01);
	dev->ll->i2c_write_reg(0x72, 0xFE);
	dev->ll->i2c_write_reg(0x76, 0x00);
	dev->ll->i2c_write_reg(0x77, 0x00);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x0D, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x80, 0x01);
	dev->ll->i2c_write_reg(0x01, 0xF8);
	dev->ll->i2c_write_reg(0xFF, 0x01);
	dev->ll->i2c_write_reg(0x8E, 0x01);
	dev->ll->i2c_write_reg(0x00, 0x01);
	dev->ll->i2c_write_reg(0xFF, 0x00);
	dev->ll->i2c_write_reg(0x80, 0x00);

	/*
	 * Set interrupt config to new sample ready
	 * See VL53L0X_SetGpioConfig() ST API func.
	 **/
	dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	dev->ll->i2c_write_reg(GPIO_HV_MUX_ACTIVE_HIGH,
			dev->ll->i2c_read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); /* active low */
	dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);


	dev->__g_meas_time_bud_us = get_measurement_timing_budget(dev);

	/*
	 * Disable MSRC and TCC by default.
	 * MSRC = Minimum Signal Rate Check
	 * TCC = Target CentreCheck
	 * See ST API func VL53L0X_SetSequenceStepEnable()
	 **/
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

	/* Recalculate timing budget */
	if (set_measurement_timing_budget(dev, dev->__g_meas_time_bud_us)) {
		return VL53L0X_FAIL;
	}

	/*
	 * See ST API func VL53L0X_PerformRefCalibration()
	 * (VL53L0X_perform_ref_calibration() --> VL53L0X_perform_vhv_calibration)
	 **/
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (perform_single_ref_calibration(dev, 0x40)) {
		return VL53L0X_FAIL;
	}

	/* See VL53L0X_perform_phase_calibration() */
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (perform_single_ref_calibration(dev, 0x00)) {
		return VL53L0X_FAIL;
	}

	/* Restore the previous Sequence Config */
	dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

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
								uint32_t time)
{
	switch (mode) {
	case VL53L0X_SINGLE:
	case VL53L0X_CONTINUOUS:
	case VL53L0X_TIMED:
		break;

	default:
		return VL53L0X_FAIL;
	}

	dev->__measurement_mode = mode;
	dev->__measurement_timeout = time;

	return VL53L0X_OK;
}
