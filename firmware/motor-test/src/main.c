#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#define POT_STOP_MV 200
#define POT_FULL_MV 2800
#define MOTOR_MIN_DUTY 0
#define MOTOR_MAX_DUTY 50
#define OBJECT_STOP_MM 100

#define MOTOR_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec in3 = GPIO_DT_SPEC_GET(MOTOR_NODE, in3_gpios);
static const struct gpio_dt_spec in4 = GPIO_DT_SPEC_GET(MOTOR_NODE, in4_gpios);
static const struct pwm_dt_spec motor_enable = PWM_DT_SPEC_GET(MOTOR_NODE);
static const struct device *const distance_sensor = DEVICE_DT_GET(DT_NODELABEL(conveyor_sensor));

static bool conveyor_path_is_clear(void)
{
	static bool was_blocked;
	struct sensor_value distance;
	int64_t distance_mm = -1;

	if (device_is_ready(distance_sensor) &&
	    sensor_sample_fetch_chan(distance_sensor, SENSOR_CHAN_DISTANCE) == 0 &&
	    sensor_channel_get(distance_sensor, SENSOR_CHAN_DISTANCE, &distance) == 0) {
		distance_mm = sensor_value_to_milli(&distance);
	}

	bool blocked = distance_mm > 0 && distance_mm < OBJECT_STOP_MM;
	if (blocked != was_blocked) {
		if (blocked) {
			printk("Conveyor path blocked (distance %d mm)\n", (int)distance_mm);
		} else {
			printk("Conveyor path clear\n");
		}
		was_blocked = blocked;
	}

	return !blocked;
}

#ifdef MOTOR_STEP_TEST
#define STEP_LOW_DUTY 35U
#define STEP_HIGH_DUTY 60U
#define STEP_AT_MS 5000
#define STEP_END_MS 10000
#define SAMPLE_MS 100

static const struct gpio_dt_spec encoder = GPIO_DT_SPEC_GET(MOTOR_NODE, encoder_gpios);
static const struct gpio_dt_spec boot_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback encoder_callback;
static atomic_t encoder_count;

static void count_encoder_edge(const struct device *port, struct gpio_callback *callback,
			       gpio_port_pins_t pins)
{
	atomic_inc(&encoder_count);
}

static int set_step_duty(uint32_t duty)
{
	return pwm_set_pulse_dt(&motor_enable, motor_enable.period * duty / 100U);
}

static const char *record_step(void)
{
	if (!conveyor_path_is_clear()) {
		return "path blocked";
	}

	atomic_clear(&encoder_count);
	if (set_step_duty(STEP_LOW_DUTY) < 0) {
		return "could not apply 35% PWM";
	}

	int64_t start_ms = k_uptime_get();
	uint32_t duty = STEP_LOW_DUTY;
	atomic_val_t settling_count = 0;

	printk("STEP,0,%u,0\n", duty);
	for (int sample_ms = SAMPLE_MS; sample_ms <= STEP_END_MS; sample_ms += SAMPLE_MS) {
		k_sleep(K_TIMEOUT_ABS_MS(start_ms + sample_ms));
		if (!conveyor_path_is_clear()) {
			return "path blocked";
		}

		atomic_val_t count = atomic_get(&encoder_count);
		if (sample_ms == STEP_AT_MS - 1000) {
			settling_count = count;
		}
		if (sample_ms == STEP_AT_MS) {
			if (count == settling_count) {
				return "no encoder pulses during the last second at 35% PWM";
			}
			if (set_step_duty(STEP_HIGH_DUTY) < 0) {
				return "could not apply 60% PWM";
			}
			duty = STEP_HIGH_DUTY;
		}
		printk("STEP,%u,%u,%u\n", (unsigned int)(k_uptime_get() - start_ms),
		       duty, (unsigned int)count);
	}
	return NULL;
}

static void run_step_test(void)
{
	if (!device_is_ready(distance_sensor) || !gpio_is_ready_dt(&encoder) ||
	    !gpio_is_ready_dt(&boot_button) ||
	    gpio_pin_configure_dt(&encoder, GPIO_INPUT | GPIO_PULL_UP) < 0 ||
	    gpio_pin_configure_dt(&boot_button, GPIO_INPUT) < 0) {
		printk("STEP_ABORT,encoder, button or distance sensor setup failed\n");
		return;
	}

	gpio_init_callback(&encoder_callback, count_encoder_edge, BIT(encoder.pin));
	if (gpio_add_callback(encoder.port, &encoder_callback) < 0 ||
	    gpio_pin_interrupt_configure_dt(&encoder, GPIO_INT_EDGE_RISING) < 0) {
		printk("STEP_ABORT,encoder interrupt setup failed\n");
		return;
	}

	printk("Press BOOT for the 35%% to 60%% PWM step test.\n");
	int pressed;
	do {
		k_sleep(K_MSEC(20));
		pressed = gpio_pin_get_dt(&boot_button);
	} while (pressed == 0);

	const char *error = pressed < 0 ? "could not read BOOT button" : record_step();
	if (set_step_duty(0) < 0) {
		gpio_pin_set_dt(&in3, 0);
		gpio_pin_set_dt(&in4, 0);
		error = "could not set PWM to zero, motor inputs set low";
	}
	if (error != NULL) {
		printk("STEP_ABORT,%s\n", error);
	} else {
		printk("STEP_DONE\n");
	}
}
#else
static const struct adc_dt_spec speed_knob = ADC_DT_SPEC_GET(MOTOR_NODE);
static void update_motor_speed(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(speed_work, update_motor_speed);

static void update_motor_speed(struct k_work *work)
{
	static int last_duty;
	uint16_t raw;
	struct adc_sequence sequence = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};

	int ret = adc_sequence_init_dt(&speed_knob, &sequence);
	if (ret == 0) {
		ret = adc_read_dt(&speed_knob, &sequence);
	}
	if (ret < 0) {
		pwm_set_pulse_dt(&motor_enable, 0);
		printk("Speed knob ADC error (%d). Motor test stopped.\n", ret);
		return;
	}

	int32_t voltage_mv = raw;
	ret = adc_raw_to_millivolts_dt(&speed_knob, &voltage_mv);
	if (ret < 0) {
		pwm_set_pulse_dt(&motor_enable, 0);
		printk("Could not convert speed knob reading (%d).\n", ret);
		return;
	}

	int duty = 0;
	if (conveyor_path_is_clear() && voltage_mv > POT_STOP_MV) {
		int32_t travel = MIN(voltage_mv, POT_FULL_MV) - POT_STOP_MV;

		duty = MOTOR_MIN_DUTY + travel * (MOTOR_MAX_DUTY - MOTOR_MIN_DUTY) /
		       (POT_FULL_MV - POT_STOP_MV);
	}

	if (duty != last_duty) {
		ret = pwm_set_pulse_dt(&motor_enable, motor_enable.period * duty / 100U);
		if (ret < 0) {
			pwm_set_pulse_dt(&motor_enable, 0);
			printk("Motor PWM update failed (%d).\n", ret);
			return;
		}
		last_duty = duty;
		printk("Conveyor duty: %d%%\n", duty);
	}

	k_work_schedule(k_work_delayable_from_work(work), K_MSEC(100));
}
#endif

#ifdef MOTOR_TEST_ON_BOOT
void motor_test_start(void);
void motor_test_start(void)
#else
static void motor_test_start(void)
#endif
{
	if (!gpio_is_ready_dt(&in3) || !gpio_is_ready_dt(&in4) ||
	    !pwm_is_ready_dt(&motor_enable)) {
		printk("Motor GPIO or PWM device is not ready.\n");
		return;
	}
	if (pwm_set_pulse_dt(&motor_enable, 0) < 0) {
		printk("Could not set motor PWM to zero.\n");
		return;
	}
	if (gpio_pin_configure_dt(&in3, GPIO_OUTPUT_ACTIVE) < 0 ||
	    gpio_pin_configure_dt(&in4, GPIO_OUTPUT_INACTIVE) < 0) {
		printk("Motor direction GPIO setup failed.\n");
		return;
	}

#ifdef MOTOR_STEP_TEST
	run_step_test();
#else
	if (!adc_is_ready_dt(&speed_knob)) {
		printk("Speed knob ADC device is not ready.\n");
		return;
	}
	if (!device_is_ready(distance_sensor)) {
		printk("Distance sensor is not ready. Speed knob remains active.\n");
	}

	int ret = adc_channel_setup_dt(&speed_knob);
	if (ret < 0) {
		printk("Could not configure speed knob (%d).\n", ret);
		return;
	}

	printk("Conveyor speed knob ready. PWM at %u Hz.\n",
	       NSEC_PER_SEC / motor_enable.period);
	printk("Conveyor stops for readings below %d mm.\n", OBJECT_STOP_MM);
	k_work_schedule(&speed_work, K_NO_WAIT);
#endif
}

#ifndef MOTOR_TEST_ON_BOOT
int main(void)
{
	motor_test_start();
	k_sleep(K_FOREVER);
	return 0;
}
#endif
