/* SPDX-License-Identifier: Apache-2.0 */

#ifndef VISION_OV3660_H_
#define VISION_OV3660_H_

#include <zephyr/drivers/gpio.h>

int ov3660_test_vsync_pin(const struct device *sensor, const struct device *gpio,
			 gpio_pin_t pin, int *low, int *high);

#endif /* VISION_OV3660_H_ */
