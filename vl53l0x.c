#include "vl53l0x.h"

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
    The stop variable is used for initiating the stop sequence when doing range measurement.
    It's not obvious why this stop variable exists
*/
static uint8_t stop_variable = 0;

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

#define VL53L0X_VCSEL_PERIOD_PRE_RANGE	(0) /* Identifies the pre-range vcsel period. */
#define VL53L0X_VCSEL_PERIOD_FINAL_RANGE (1) /* Identifies the final range vcsel period. */

#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

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

typedef enum {
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;

typedef enum {
	VL53L0X_SINGLE_RANGING =  0,
	VL53L0X_CONTINUOUS_RANGING =  1,
	VL53L0X_SINGLE_HISTOGRAM =  2, /* not supported for now */
	VL53L0X_CONTINUOUS_TIMED_RANGING =  3, /* not supported for now */
	VL53L0X_SINGLE_ALS = 10, /* not supported for now */
	VL53L0X_GPIO_DRIVE = 20, /* not supported for now */
	VL53L0X_GPIO_OSC = 21, /* not supported for now */
} vl53l0x_measure_mode_t;

/* Returns 0 if OK */
static int check_args(vl53l0x_dev_t *dev)
{
    if (!dev || !dev->ll->delay_ms || \
        !dev->ll->i2c_write_reg || !dev->ll->i2c_write_reg_16bit || \
        !dev->ll->i2c_write_reg_32bit || !dev->ll->i2c_read_reg || \
        !dev->ll->i2c_read_reg_16bit || !dev->ll->i2c_read_reg_32bit) {
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

static int data_init(vl53l0x_dev_t *dev)
{
    /* USE_I2C_2V8 */
    dev->ll->i2c_write_reg(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
        (dev->ll->i2c_read_reg(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) & 0xFE) | 0x01);

    dev->ll->i2c_write_reg(0x88, 0x00); /* Set I2C standard mode (400KHz) */

    dev->ll->i2c_write_reg(0x80, 0x01);
    dev->ll->i2c_write_reg(0xFF, 0x01);
    dev->ll->i2c_write_reg(0x00, 0x00);
    stop_variable = dev->ll->i2c_read_reg(0x91);
    dev->ll->i2c_write_reg(0x00, 0x01);
    dev->ll->i2c_write_reg(0xFF, 0x00);
    dev->ll->i2c_write_reg(0x80, 0x00);

    return 0;
}

static void set_sequence_steps_enabled(vl53l0x_dev_t *dev, uint8_t sequence_step)
{
    dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, sequence_step);
}

static int perform_single_ref_calibration(vl53l0x_dev_t *dev, calibration_type_t calib_type)
{

    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;

    switch (calib_type) {
    case CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }

    dev->ll->i2c_write_reg(SYSTEM_SEQUENCE_CONFIG, sequence_config);
    dev->ll->i2c_write_reg(SYSRANGE_START, sysrange_start);

    /* Wait for interrupt */
    uint8_t interrupt_status = 0;
    do {
        interrupt_status = dev->ll->i2c_read_reg(RESULT_INTERRUPT_STATUS);
    } while ((interrupt_status & 0x07) == 0);


    dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    dev->ll->i2c_write_reg(SYSRANGE_START, 0x00);

    return 0;
}

static int perform_ref_calibration(vl53l0x_dev_t *dev)
{
    if (perform_single_ref_calibration(dev, CALIBRATION_TYPE_VHV)) {
        return 1;
    }

    if (perform_single_ref_calibration(dev, CALIBRATION_TYPE_PHASE)) {
        return 1;
    }

    /* restore the previous Sequence Config */
    set_sequence_steps_enabled(dev, RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE);

    return 0;
}

/* Return 0 if OK */
static int static_init(vl53l0x_dev_t *dev)
{
    /* Same as default tuning settings provided by ST api code */
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
    dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO,
            VL53L0X_GPIO_FUNC_NEW_MEASURE_READY);
    dev->ll->i2c_write_reg(GPIO_HV_MUX_ACTIVE_HIGH,
            dev->ll->i2c_read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); /* polarity low */
    dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->gpio_func = VL53L0X_GPIO_FUNC_NEW_MEASURE_READY;

    set_sequence_steps_enabled(dev, RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE);

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

    if (data_init(dev)) {
        return VL53L0X_FAIL;
    }

    if (static_init(dev)) {
        return VL53L0X_FAIL;
    }

    /**
     * Temperature calibration needs to be run again if the temperature changes by
     * more than 8 degrees according to the datasheet.
     */
    if (perform_ref_calibration(dev)) {
        return VL53L0X_FAIL;
    }

    return VL53L0X_OK;
}

void vl53l0x_shutdown(vl53l0x_dev_t *dev)
{
    if (!dev->ll->xshut_set || !dev->ll->xshut_reset) {
        return;
    }

    dev->ll->xshut_reset();
}

void vl53l0x_power_up(vl53l0x_dev_t *dev)
{
    if (!dev->ll->xshut_set || !dev->ll->xshut_reset) {
        return;
    }

    dev->ll->xshut_set();
}

vl53l0x_ret_t vl53l0x_do_measurement(vl53l0x_dev_t *dev, vl53l0x_measure_mode_t mode)
{
    uint8_t sysrange_start = 0;
    uint8_t interrupt_status = 0;
    int timeout_cycles = 0;

    dev->ll->i2c_write_reg(0x80, 0x01);
    dev->ll->i2c_write_reg(0xFF, 0x01);
    dev->ll->i2c_write_reg(0x00, 0x00);
    dev->ll->i2c_write_reg(0x91, stop_variable);
    dev->ll->i2c_write_reg(0x00, 0x01);
    dev->ll->i2c_write_reg(0xFF, 0x00);
    dev->ll->i2c_write_reg(0x80, 0x00);

    switch (mode) {
    case VL53L0X_SINGLE_RANGING:
        dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP);

        do {
            if (timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP) {
                return VL53L0X_FAIL;
            }
            ++timeout_cycles;
            sysrange_start = dev->ll->i2c_read_reg(SYSRANGE_START);
        } while (sysrange_start & 0x01);

        timeout_cycles = 0;

        do {
            if (timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP) {
                return VL53L0X_FAIL;
            }
            ++timeout_cycles;

            interrupt_status = dev->ll->i2c_read_reg(RESULT_INTERRUPT_STATUS);
        } while ((interrupt_status & 0x07) == 0);

        break;

    case VL53L0X_CONTINUOUS_RANGING:
        /* continuous back-to-back mode */
        dev->ll->i2c_write_reg(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);
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

vl53l0x_ret_t vl53l0x_activate_gpio_interrupt(vl53l0x_dev_t *dev, vl53l0x_gpio_func_t int_type)
{
    /*
     * Set interrupt config to new sample ready
     * See VL53L0X_SetGpioConfig() ST API func.
     **/
    dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO, int_type);
    dev->ll->i2c_write_reg(GPIO_HV_MUX_ACTIVE_HIGH,
            dev->ll->i2c_read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); /* polarity low */
    dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->gpio_func = int_type;

    return VL53L0X_OK;
}

vl53l0x_ret_t vl53l0x_deactivate_gpio_interrupt(vl53l0x_dev_t *dev)
{
    dev->ll->i2c_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO,
                VL53L0X_GPIO_FUNC_OFF);
    dev->gpio_func = VL53L0X_GPIO_FUNC_OFF;

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
vl53l0x_ret_t vl53l0x_read_in_oneshot_mode(vl53l0x_dev_t *dev, uint16_t *range)
{
    vl53l0x_ret_t ret = VL53L0X_FAIL;

    /* VL53L0X_PerformSingleMeasurement */
    ret = vl53l0x_do_measurement(dev, VL53L0X_SINGLE_RANGING);

    /* Based on VL53L0X_GetRangingMeasurementData() */
    *range = dev->ll->i2c_read_reg_16bit(RESULT_RANGE_STATUS + 10);

    /* clear IRQ flag in vl53l0 status register */
    ret |= vl53l0x_clear_flag_gpio_interrupt(dev);

    return ret;
}

vl53l0x_ret_t vl53l0x_start_continuous_measurements(vl53l0x_dev_t *dev)
{
	return vl53l0x_do_measurement(dev, VL53L0X_CONTINUOUS_RANGING);
}

vl53l0x_ret_t vl53l0x_get_range_mm_continuous(vl53l0x_dev_t *dev, uint16_t *range)
{
    *range = dev->ll->i2c_read_reg_16bit(0x14 + 10);
    return VL53L0X_OK;
}
