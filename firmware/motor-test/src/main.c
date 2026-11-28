#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#ifdef UNOQ_SORT_STATION
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdlib.h>
#endif

#define POT_STOP_MV 200
#define POT_FULL_MV 2800
#define MOTOR_MIN_DUTY 0
#define MOTOR_MAX_DUTY 50
#define OBJECT_STOP_MM 100
#define STATION_STOP_MM 25

#define MOTOR_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec in3 = GPIO_DT_SPEC_GET(MOTOR_NODE, in3_gpios);
static const struct gpio_dt_spec in4 = GPIO_DT_SPEC_GET(MOTOR_NODE, in4_gpios);
static const struct pwm_dt_spec motor_enable = PWM_DT_SPEC_GET(MOTOR_NODE);

#ifndef UNOQ_AUTO_STEP_TEST
static const struct device *const distance_sensor = DEVICE_DT_GET(DT_NODELABEL(conveyor_sensor));

static bool conveyor_path_is_clear(int *measured_mm)
{
	static bool was_blocked;
	struct sensor_value distance;
	int64_t distance_mm = -1;

	if (device_is_ready(distance_sensor) &&
	    sensor_sample_fetch_chan(distance_sensor, SENSOR_CHAN_DISTANCE) == 0 &&
	    sensor_channel_get(distance_sensor, SENSOR_CHAN_DISTANCE, &distance) == 0) {
		distance_mm = sensor_value_to_milli(&distance);
	}
	if (measured_mm != NULL) {
		*measured_mm = (int)distance_mm;
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
#endif

#if defined(MOTOR_STEP_TEST) || defined(UNOQ_CONVEYOR)
static const struct gpio_dt_spec encoder = GPIO_DT_SPEC_GET(MOTOR_NODE, encoder_gpios);
static struct gpio_callback encoder_callback;
static atomic_t encoder_count;

static void count_encoder_edge(const struct device *port, struct gpio_callback *callback,
			       gpio_port_pins_t pins)
{
	atomic_inc(&encoder_count);
}

static int setup_encoder(void)
{
	if (!gpio_is_ready_dt(&encoder)) {
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&encoder, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&encoder_callback, count_encoder_edge, BIT(encoder.pin));
	ret = gpio_add_callback(encoder.port, &encoder_callback);
	if (ret < 0) {
		return ret;
	}
	return gpio_pin_interrupt_configure_dt(&encoder, GPIO_INT_EDGE_RISING);
}
#endif

#if !defined(MOTOR_STEP_TEST) || (defined(UNOQ_CONVEYOR) && !defined(UNOQ_AUTO_STEP_TEST))
static const struct adc_dt_spec speed_knob = ADC_DT_SPEC_GET(MOTOR_NODE);

static int read_knob_mv(int32_t *voltage_mv)
{
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
		return ret;
	}

	*voltage_mv = raw;
	return adc_raw_to_millivolts_dt(&speed_knob, voltage_mv);
}
#endif

#ifdef MOTOR_STEP_TEST
#ifdef UNOQ_AUTO_STEP_TEST
#define STEP_LOW_DUTY 60U
#define STEP_HIGH_DUTY 90U
#define STEP_AT_MS 10000
#define STEP_END_MS 20000
#else
#define STEP_LOW_DUTY 35U
#define STEP_HIGH_DUTY 60U
#define STEP_AT_MS 5000
#define STEP_END_MS 10000
#endif
#define SAMPLE_MS 100

#ifdef UNOQ_CONVEYOR
struct step_sample {
	uint32_t elapsed_ms;
	uint32_t duty;
	uint32_t pulses;
};

/* Keep a copy for SWD readout after the run, when Linux owns the serial port. */
static volatile struct {
	uint32_t finished;
	uint32_t count;
	const char *error;
	struct step_sample samples[1 + STEP_END_MS / SAMPLE_MS];
} unoq_step_log;
#endif

#ifdef UNOQ_AUTO_STEP_TEST
static int wait_for_step_start(void)
{
	printk("Automatic step test starts in 5 seconds.\n");
	k_sleep(K_SECONDS(5));
	return 0;
}
#elif defined(UNOQ_CONVEYOR)
static int wait_for_step_start(void)
{
	int32_t voltage_mv;
	bool knob_zero_seen = false;

	printk("Step test ready. Turn the knob fully down, then fully up to start.\n");
	for (;;) {
		int ret = read_knob_mv(&voltage_mv);
		if (ret < 0) {
			return ret;
		}
		if (voltage_mv <= POT_STOP_MV) {
			knob_zero_seen = true;
		}
		if (knob_zero_seen && voltage_mv >= POT_FULL_MV) {
			return 0;
		}
		k_sleep(K_MSEC(20));
	}
}
#else
static const struct gpio_dt_spec boot_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static int wait_for_step_start(void)
{
	printk("Press BOOT for the 35%% to 60%% PWM step test.\n");
	int pressed;
	do {
		k_sleep(K_MSEC(20));
		pressed = gpio_pin_get_dt(&boot_button);
	} while (pressed == 0);
	return pressed < 0 ? pressed : 0;
}
#endif

static void report_step_sample(uint32_t elapsed_ms, uint32_t duty, uint32_t pulses)
{
#ifdef UNOQ_CONVEYOR
	uint32_t index = unoq_step_log.count;

	unoq_step_log.samples[index] = (struct step_sample){elapsed_ms, duty, pulses};
	unoq_step_log.count = index + 1;
#endif
	printk("STEP,%u,%u,%u\n", elapsed_ms, duty, pulses);
}

static int set_step_duty(uint32_t duty)
{
	return pwm_set_pulse_dt(&motor_enable, motor_enable.period * duty / 100U);
}

static const char *record_step(void)
{
#ifndef UNOQ_AUTO_STEP_TEST
	if (!conveyor_path_is_clear(NULL)) {
		return "path blocked";
	}
#endif

	atomic_clear(&encoder_count);
	if (set_step_duty(STEP_LOW_DUTY) < 0) {
		return "could not apply the initial PWM duty";
	}

	int64_t start_ms = k_uptime_get();
	uint32_t duty = STEP_LOW_DUTY;
	atomic_val_t settling_count = 0;

	report_step_sample(0, duty, 0);
	for (int sample_ms = SAMPLE_MS; sample_ms <= STEP_END_MS; sample_ms += SAMPLE_MS) {
		k_sleep(K_TIMEOUT_ABS_MS(start_ms + sample_ms));
#if defined(UNOQ_CONVEYOR) && !defined(UNOQ_AUTO_STEP_TEST)
		int32_t voltage_mv;
		if (read_knob_mv(&voltage_mv) < 0 || voltage_mv <= POT_STOP_MV) {
			return "speed knob at zero or ADC error";
		}
#endif
#ifndef UNOQ_AUTO_STEP_TEST
		if (!conveyor_path_is_clear(NULL)) {
			return "path blocked";
		}
#endif

		atomic_val_t count = atomic_get(&encoder_count);
		if (sample_ms == STEP_AT_MS - 1000) {
			settling_count = count;
		}
		if (sample_ms == STEP_AT_MS) {
			if (count == settling_count) {
				return "no encoder pulses during the last second before the step";
			}
			if (set_step_duty(STEP_HIGH_DUTY) < 0) {
				return "could not apply the higher PWM duty";
			}
			duty = STEP_HIGH_DUTY;
		}
		report_step_sample((uint32_t)(k_uptime_get() - start_ms), duty, (uint32_t)count);
	}
	return NULL;
}

static const char *prepare_step_test(void)
{
#ifndef UNOQ_AUTO_STEP_TEST
	if (!device_is_ready(distance_sensor)) {
		return "distance sensor setup failed";
	}
#endif
#if defined(UNOQ_CONVEYOR) && !defined(UNOQ_AUTO_STEP_TEST)
	if (!adc_is_ready_dt(&speed_knob) || adc_channel_setup_dt(&speed_knob) < 0) {
		return "speed knob setup failed";
	}
#elif !defined(UNOQ_CONVEYOR)
	if (!gpio_is_ready_dt(&boot_button) ||
	    gpio_pin_configure_dt(&boot_button, GPIO_INPUT) < 0) {
		return "BOOT button setup failed";
	}
#endif

	if (setup_encoder() < 0) {
		return "encoder interrupt setup failed";
	}

	return wait_for_step_start() < 0 ? "could not read start control" : NULL;
}

static void run_step_test(void)
{
	const char *error = prepare_step_test();
	if (error == NULL) {
		error = record_step();
	}
	if (set_step_duty(0) < 0) {
		gpio_pin_set_dt(&in3, 0);
		gpio_pin_set_dt(&in4, 0);
		error = "could not set PWM to zero, motor inputs set low";
	}
#ifdef UNOQ_CONVEYOR
	unoq_step_log.error = error;
	unoq_step_log.finished = 1;
#endif
	if (error != NULL) {
		printk("STEP_ABORT,%s\n", error);
	} else {
		printk("STEP_DONE\n");
	}
}
#else
static void update_motor_speed(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(speed_work, update_motor_speed);

#ifdef UNOQ_SORT_STATION
static const struct device *const station_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
RING_BUF_DECLARE(station_rx, 128);
static uint32_t station_command;
static int64_t run_until_ms;
static int station_stop_mm = STATION_STOP_MM;
static int station_trip_mm;
static uint32_t jog_start_pulses;
static int64_t jog_until_ms;

#define STATION_JOG_DUTY 65
#define STATION_JOG_TIMEOUT_MS 450

static void station_uart_rx(const struct device *uart, void *user_data)
{
	uint8_t bytes[16];
	int count;

	uart_irq_update(uart);
	if (!uart_irq_rx_ready(uart)) {
		return;
	}
	while ((count = uart_fifo_read(uart, bytes, sizeof(bytes))) > 0) {
		ring_buf_put(&station_rx, bytes, count);
	}
}

static void read_station_commands(void)
{
	static char line[24];
	static size_t length;
	static bool overflow;
	unsigned char ch;

	for (int i = 0; i < 128 && ring_buf_get(&station_rx, &ch, 1) == 1; i++) {
		if (ch == '\n') {
			if (!overflow && length > 2 && line[1] == ' ' &&
			    (line[0] == 'R' || line[0] == 'S' || line[0] == 'J') &&
			    line[2] >= '1' && line[2] <= '9') {
				char *end;
				unsigned long stop_mm = STATION_STOP_MM;

				line[length] = '\0';
				unsigned long id = strtoul(line + 2, &end, 10);
				if (line[0] == 'R' && *end == ' ') {
					stop_mm = strtoul(end + 1, &end, 10);
				}
				if (*end == '\0' && id < 1000000000UL && stop_mm >= 10 && stop_mm <= 400) {
					station_command = id;
					int64_t now = k_uptime_get();
					if (line[0] == 'S') {
						run_until_ms = 0;
						jog_until_ms = 0;
						station_trip_mm = 0;
					} else if (line[0] == 'J' && now >= run_until_ms && !jog_until_ms) {
						jog_start_pulses = (uint32_t)atomic_get(&encoder_count);
						jog_until_ms = now + STATION_JOG_TIMEOUT_MS;
					} else if (line[0] == 'R' && !station_trip_mm && !jog_until_ms) {
						station_stop_mm = (int)stop_mm;
						run_until_ms = now + 1200;
					}
				}
			}
			length = 0;
			overflow = false;
		} else if (length < sizeof(line) - 1) {
			line[length++] = ch;
		} else {
			overflow = true;
		}
	}
}
#endif

#ifdef UNOQ_SPEED_PI
#define SPEED_MIN_RPM 105
#define SPEED_MAX_RPM 120
#define SPEED_PULSES_PER_REV 20U
#define SPEED_WINDOW_SAMPLES 5U
#define SPEED_MAX_DUTY 90.0f
#define SPEED_KP 0.234f
#define SPEED_KI 0.635f
#define SPEED_NO_PULSE_MS 1200

static struct {
	struct {
		int64_t time_ms;
		uint32_t pulses;
	} history[SPEED_WINDOW_SAMPLES];
	uint32_t index;
	uint32_t samples;
	uint32_t last_pulses;
	int64_t last_pulse_ms;
	int64_t last_update_ms;
	float integral;
	bool encoder_fault;
} speed_pi;

static int update_speed_pi(int target_rpm, int64_t now, uint32_t pulses, int *measured_rpm)
{
	uint32_t index = speed_pi.index;
	int64_t window_ms = now - speed_pi.history[index].time_ms;
	int64_t update_ms = MIN(now - speed_pi.last_update_ms, 250);
	float speed = 0.0f;

	if (speed_pi.samples == 0 || pulses != speed_pi.last_pulses) {
		speed_pi.last_pulse_ms = now;
	}
	speed_pi.last_pulses = pulses;
	if (now - speed_pi.last_pulse_ms >= SPEED_NO_PULSE_MS) {
		return -EIO;
	}

	bool window_ready = speed_pi.samples == SPEED_WINDOW_SAMPLES && window_ms > 0;
	if (window_ready) {
		uint32_t pulse_delta = pulses - speed_pi.history[index].pulses;

		speed = (float)pulse_delta * 60000.0f / ((float)window_ms * SPEED_PULSES_PER_REV);
		*measured_rpm = (int)(speed + 0.5f);
	}
	speed_pi.history[index].time_ms = now;
	speed_pi.history[index].pulses = pulses;
	speed_pi.index = (index + 1) % SPEED_WINDOW_SAMPLES;
	speed_pi.samples = MIN(speed_pi.samples + 1, SPEED_WINDOW_SAMPLES);
	speed_pi.last_update_ms = now;

	/* The 60% to 90% run gave about 103 to 124 RPM. */
	float feedforward = 60.0f + (target_rpm - 103.0f) / 0.8f;
	if (!window_ready) {
		return (int)MAX(feedforward, 65.0f);
	}

	float error = target_rpm - speed;
	float base_duty = feedforward + SPEED_KP * error;
	float duty = base_duty + speed_pi.integral;
	if ((error > 0 && duty < SPEED_MAX_DUTY) || (error < 0 && duty > 0)) {
		speed_pi.integral += SPEED_KI * error * (float)update_ms / 1000.0f;
		duty = base_duty + speed_pi.integral;
	}
	return (int)(CLAMP(duty, 0.0f, SPEED_MAX_DUTY) + 0.5f);
}
#endif

static void update_motor_speed(struct k_work *work)
{
	static int last_duty;
#ifdef UNOQ_CONVEYOR
	static bool knob_zero_seen;
#ifndef UNOQ_SORT_STATION
	static int64_t next_status_ms;
#endif
	int distance_mm;
#endif
#ifdef UNOQ_SORT_STATION
	read_station_commands();
#endif
	int32_t voltage_mv;
	int ret = read_knob_mv(&voltage_mv);
	if (ret < 0) {
		pwm_set_pulse_dt(&motor_enable, 0);
		printk("Speed knob ADC error (%d). Motor test stopped.\n", ret);
		return;
	}

	int duty = 0;
#ifdef UNOQ_CONVEYOR
#ifdef UNOQ_SORT_STATION
	int64_t now = k_uptime_get();
	bool jog_cycle = jog_until_ms != 0;
	if (jog_until_ms && (now >= jog_until_ms || voltage_mv <= POT_STOP_MV ||
	    (uint32_t)atomic_get(&encoder_count) != jog_start_pulses)) {
		jog_until_ms = 0;
	}
	bool jogging = jog_until_ms != 0;
	bool seeking = now < run_until_ms;
	distance_mm = -1;
	if (seeking) {
		conveyor_path_is_clear(&distance_mm);
		if (distance_mm > 0 && distance_mm < station_stop_mm) {
			station_trip_mm = distance_mm;
			run_until_ms = 0;
		}
	}
	bool path_clear = k_uptime_get() < run_until_ms;
	bool run_allowed = path_clear || jogging;
#else
	bool path_clear = conveyor_path_is_clear(&distance_mm);
#endif
	if (!knob_zero_seen && voltage_mv <= POT_STOP_MV) {
		knob_zero_seen = true;
		printk("Speed knob at zero. Conveyor ready.\n");
	}
	path_clear = path_clear && knob_zero_seen;
#else
	bool path_clear = conveyor_path_is_clear(NULL);
#endif
#ifdef UNOQ_SPEED_PI
	int target_rpm = 0;
	int measured_rpm = 0;
	int64_t update_time = k_uptime_get();
	uint32_t encoder_pulses = (uint32_t)atomic_get(&encoder_count);

	if (voltage_mv <= POT_STOP_MV) {
		speed_pi.encoder_fault = false;
	}
	if (path_clear && voltage_mv > POT_STOP_MV && !speed_pi.encoder_fault) {
		int32_t travel = MIN(voltage_mv, POT_FULL_MV) - POT_STOP_MV;

		target_rpm = SPEED_MIN_RPM + travel * (SPEED_MAX_RPM - SPEED_MIN_RPM) /
			     (POT_FULL_MV - POT_STOP_MV);
		duty = update_speed_pi(target_rpm, update_time, encoder_pulses, &measured_rpm);
		if (duty < 0) {
			speed_pi.encoder_fault = true;
			duty = 0;
			printk("No encoder pulses. Turn the knob to zero to rearm.\n");
		}
	}
	if (duty == 0 && (target_rpm == 0 || speed_pi.encoder_fault)) {
		speed_pi.index = 0;
		speed_pi.samples = 0;
		speed_pi.integral = 0.0f;
	}
#else
	if (path_clear && voltage_mv > POT_STOP_MV) {
		int32_t travel = MIN(voltage_mv, POT_FULL_MV) - POT_STOP_MV;

		duty = MOTOR_MIN_DUTY + travel * (MOTOR_MAX_DUTY - MOTOR_MIN_DUTY) /
		       (POT_FULL_MV - POT_STOP_MV);
	}
#endif

#ifdef UNOQ_SORT_STATION
	bool jog_motor_ready = knob_zero_seen && voltage_mv > POT_STOP_MV;
#ifdef UNOQ_SPEED_PI
	jog_motor_ready = jog_motor_ready && !speed_pi.encoder_fault;
#endif
	if (jogging && jog_motor_ready) {
		duty = STATION_JOG_DUTY;
	}
#endif
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

#ifdef UNOQ_CONVEYOR
#ifdef UNOQ_SORT_STATION
	if (station_trip_mm) {
		distance_mm = station_trip_mm;
	} else if (!seeking && !jog_cycle) {
		conveyor_path_is_clear(&distance_mm);
	}
	printk("SORT %u %d %d %u %u\n", station_command, distance_mm, duty,
	       (unsigned int)atomic_get(&encoder_count), (unsigned int)run_allowed);
#else
	int64_t now = k_uptime_get();
	if (now >= next_status_ms) {
#ifdef UNOQ_SPEED_PI
		printk("Speed: target=%d RPM, measured=%d RPM, PWM=%d%%, distance=%d mm\n",
		       target_rpm, measured_rpm, duty, distance_mm);
#else
		printk("Conveyor: knob=%d mV, distance=%d mm, encoder=%u, PWM=%d%%\n",
		       (int)voltage_mv, distance_mm, (unsigned int)atomic_get(&encoder_count), duty);
#endif
		next_status_ms = now + 1000;
	}
#endif
#endif
#ifdef UNOQ_SORT_STATION
	k_work_schedule(k_work_delayable_from_work(work), K_MSEC(jogging ? 15 : 100));
#else
	k_work_schedule(k_work_delayable_from_work(work), K_MSEC(100));
#endif
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
#ifdef UNOQ_CONVEYOR
		printk("Distance sensor is not ready. Motor stopped.\n");
		return;
#else
		printk("Distance sensor is not ready. Speed knob remains active.\n");
#endif
	}

	int ret = adc_channel_setup_dt(&speed_knob);
	if (ret < 0) {
		printk("Could not configure speed knob (%d).\n", ret);
		return;
	}

#ifdef UNOQ_CONVEYOR
	ret = setup_encoder();
	if (ret < 0) {
		printk("Encoder setup failed (%d). Motor stopped.\n", ret);
		return;
	}
	printk("UNO Q conveyor. Turn knob to zero before running. Distance -1 means no echo.\n");
#endif
	printk("Conveyor speed knob ready. PWM at %u Hz.\n",
	       NSEC_PER_SEC / motor_enable.period);
#ifdef UNOQ_SORT_STATION
	if (!device_is_ready(station_uart) ||
	    uart_irq_callback_user_data_set(station_uart, station_uart_rx, NULL) < 0) {
		printk("Sorting UART setup failed. Motor stopped.\n");
		return;
	}
	uart_irq_rx_enable(station_uart);
	printk("Sorting mode. Conveyor waits for the PC controller.\n");
#else
	printk("Conveyor stops for readings below %d mm.\n", OBJECT_STOP_MM);
#endif
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
