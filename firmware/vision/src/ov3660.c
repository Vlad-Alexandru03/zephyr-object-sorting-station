/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ovti_ov3660

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ov3660_regs.h"

LOG_MODULE_REGISTER(vision_ov3660, CONFIG_VIDEO_LOG_LEVEL);

#define OV3660_PID_HIGH_REG 0x300a
#define OV3660_PID_LOW_REG  0x300b
#define OV3660_SYSTEM_CTRL  0x3008
#define OV3660_STREAMING    0x02
#define OV3660_STANDBY      0x42
#define OV3660_PID          0x3660
#define OV3660_FRAME_SIZE   240
#define OV3660_STARTUP_MS   100
#define OV3660_PID_ATTEMPTS 3
#define OV3660_PID_RETRY_MS 20

static const struct video_format_cap ov3660_formats[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width_min = OV3660_FRAME_SIZE,
		.width_max = OV3660_FRAME_SIZE,
		.height_min = OV3660_FRAME_SIZE,
		.height_max = OV3660_FRAME_SIZE,
	},
	 {
        .pixelformat = VIDEO_PIX_FMT_JPEG,
        .width_min = 320,
        .width_max = 320,
        .height_min = 240,
        .height_max = 240,
    },
	  {
        .pixelformat = VIDEO_PIX_FMT_JPEG,
        .width_min = 2048,
        .width_max = 2048,
        .height_min = 1536,
        .height_max = 1536,
    },
	{},
};

struct ov3660_data {
    struct video_format format;
    bool format_valid;
};

static int ov3660_read(const struct i2c_dt_spec *bus, uint16_t reg, uint8_t *value)
{
	uint8_t address[2];

	sys_put_be16(reg, address);
	return i2c_write_read_dt(bus, address, sizeof(address), value, 1);
}

static int ov3660_write(const struct i2c_dt_spec *bus, uint16_t reg, uint8_t value)
{
	uint8_t data[3];

	sys_put_be16(reg, data);
	data[2] = value;
	return i2c_write_dt(bus, data, sizeof(data));
}

static int ov3660_write_table(const struct i2c_dt_spec *bus, const struct ov3660_reg *table,
			     size_t length)
{
	for (size_t i = 0; i < length; i++) {
		int ret;

		if (table[i].address == OV3660_DELAY_REG) {
			k_msleep(table[i].value);
			continue;
		}

		ret = ov3660_write(bus, table[i].address, table[i].value);
		if (ret < 0) {
			LOG_ERR("Register 0x%04x write failed (%d)", table[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int ov3660_init(const struct device *dev)
{
	const struct i2c_dt_spec *bus = dev->config;
	uint8_t high, low;
	uint16_t pid = 0;
	int ret = 0;

	if (!i2c_is_ready_dt(bus)) {
		LOG_ERR("Camera I2C bus is not ready");
		return -ENODEV;
	}

	/* XCLK is enabled during early boot. Wait for the sensor before probing it. */
	k_msleep(OV3660_STARTUP_MS);
	for (int attempt = 0; attempt < OV3660_PID_ATTEMPTS; attempt++) {
		ret = ov3660_read(bus, OV3660_PID_HIGH_REG, &high);
		if (ret == 0) {
			ret = ov3660_read(bus, OV3660_PID_LOW_REG, &low);
		}
		if (ret == 0) {
			pid = ((uint16_t)high << 8) | low;
			if (pid == OV3660_PID) {
				break;
			}
		}
		if (attempt + 1 < OV3660_PID_ATTEMPTS) {
			k_msleep(OV3660_PID_RETRY_MS);
		}
	}
	if (ret < 0) {
		LOG_ERR("Camera PID read failed (%d)", ret);
		return ret;
	}

	if (pid != OV3660_PID) {
		LOG_ERR("Unexpected camera PID 0x%04x", pid);
		return -ENODEV;
	}

	ret = ov3660_write_table(bus, ov3660_defaults, ARRAY_SIZE(ov3660_defaults));
	if (ret < 0) {
		return ret;
	}

	ret = ov3660_write_table(bus, ov3660_snapshot_mode, ARRAY_SIZE(ov3660_snapshot_mode));
	if (ret < 0) {
		return ret;
	}

	struct ov3660_data *data = dev->data;

    data->format = (struct video_format) {
        .type = VIDEO_BUF_TYPE_OUTPUT,
        .pixelformat = VIDEO_PIX_FMT_RGB565,
        .width = OV3660_FRAME_SIZE,
        .height = OV3660_FRAME_SIZE,
    };

    ret = video_estimate_fmt_size(&data->format);
    if (ret < 0) {
        return ret;
    }

    data->format_valid = true;

    k_msleep(10);
    LOG_INF("OV3660 initialized (PID 0x%04x)", pid);
    return 0;
}

static int ov3660_get_format(const struct device *dev,
                            struct video_format *fmt)
{
    struct ov3660_data *data = dev->data;

    if (fmt->type != VIDEO_BUF_TYPE_OUTPUT) {
        return -EINVAL;
    }

    if (!data->format_valid) {
        return -EIO;
    }

    *fmt = data->format;
    return 0;
}

static int ov3660_set_format(const struct device *dev,
                            struct video_format *fmt)
{
    const struct i2c_dt_spec *bus = dev->config;
    struct ov3660_data *data = dev->data;
    struct video_format requested = *fmt;

    if (requested.type != VIDEO_BUF_TYPE_OUTPUT) {
        return -EINVAL;
    }

    bool rgb565 = requested.pixelformat == VIDEO_PIX_FMT_RGB565 &&
                  requested.width == OV3660_FRAME_SIZE &&
                  requested.height == OV3660_FRAME_SIZE;

    bool qvga = requested.width == 320 && requested.height == 240;
    bool qxga = requested.width == 2048 && requested.height == 1536;

    bool jpeg = requested.pixelformat == VIDEO_PIX_FMT_JPEG &&
                (qvga || qxga);
    if (!rgb565 && !jpeg) {
        return -ENOTSUP;
    }

    int ret = video_estimate_fmt_size(&requested);
    if (ret < 0) {
        return ret;
    }
    if (jpeg && qxga) {
        requested.pitch = 0;
        requested.size = 1024U * 1024U;
    }

    if (data->format_valid &&
        requested.pixelformat == data->format.pixelformat &&
        requested.width == data->format.width &&
        requested.height == data->format.height) {
        *fmt = data->format;
        return 0;
    }

    /* A failed register write can leave the sensor partly configured. */
    data->format_valid = false;

    ret = ov3660_write_table(bus, ov3660_defaults,
                            ARRAY_SIZE(ov3660_defaults));
    if (ret < 0) {
        return ret;
    }

    ret = ov3660_write_table(bus, ov3660_snapshot_mode,
                            ARRAY_SIZE(ov3660_snapshot_mode));
    if (ret < 0) {
        return ret;
    }

    if (jpeg) {
        ret = ov3660_write_table(bus, ov3660_jpeg_qvga_mode,
                                ARRAY_SIZE(ov3660_jpeg_qvga_mode));
        if (ret < 0) {
            return ret;
        }

        if (qxga) {
            ret = ov3660_write_table(bus, ov3660_jpeg_qxga_mode,
                                    ARRAY_SIZE(ov3660_jpeg_qxga_mode));
            if (ret < 0) {
                return ret;
            }
        }
    }

    k_msleep(10);

    ret = ov3660_write(bus, OV3660_SYSTEM_CTRL, OV3660_STANDBY);
    if (ret < 0) {
        return ret;
    }

    data->format = requested;
    data->format_valid = true;
    *fmt = requested;

    return 0;
}

static int ov3660_get_caps(const struct device *dev, struct video_caps *caps)
{
	ARG_UNUSED(dev);

	if (caps->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	caps->format_caps = ov3660_formats;
	return 0;
}

static int ov3660_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
    const struct i2c_dt_spec *bus = dev->config;

    if (type != VIDEO_BUF_TYPE_OUTPUT) {
        return -EINVAL;
    }

    return ov3660_write(bus, OV3660_SYSTEM_CTRL, enable ? OV3660_STREAMING : OV3660_STANDBY);
}

static DEVICE_API(video, ov3660_api) = {
	.set_format = ov3660_set_format,
	.get_format = ov3660_get_format,
	.get_caps = ov3660_get_caps,
	.set_stream = ov3660_set_stream,
};

static const struct i2c_dt_spec ov3660_bus = I2C_DT_SPEC_INST_GET(0);
static struct ov3660_data ov3660_runtime;

DEVICE_DT_INST_DEFINE(0, ov3660_init, NULL,
                      &ov3660_runtime, &ov3660_bus,
                      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,
                      &ov3660_api);
