#include <errno.h>
#include <esp_cache.h>
#include <esp_memory_utils.h>
#include <hal/usb_serial_jtag_ll.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "ov3660.h"

static void report_capture_error(const struct device *sensor, int error)
{
    const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    /* ESP32-S3-EYE routes VSYNC to GPIO6 and HREF to GPIO7. */
    const gpio_pin_t vsync_pin = 6;
    const gpio_port_value_t vsync = BIT(vsync_pin);
    const gpio_port_value_t href = BIT(7);
    gpio_port_value_t previous, pins;
    unsigned int frames = 0, lines = 0;

    if (!device_is_ready(gpio) || gpio_port_get_raw(gpio, &previous) < 0) {
        printk("ERROR: waiting for frame (%d)\n", error);
        return;
    }

    int64_t deadline = k_uptime_get() + 150;
    while (k_uptime_get() < deadline && gpio_port_get_raw(gpio, &pins) == 0) {
        gpio_port_value_t rising = pins & ~previous;

        frames += (rising & vsync) != 0;
        lines += (rising & href) != 0;
        previous = pins;
        k_busy_wait(5);
    }

    if (frames == 0) {
        int low, high;
        int pin_test = ov3660_test_vsync_pin(sensor, gpio, vsync_pin, &low, &high);

        if (pin_test == 0) {
            printk("ERROR: waiting for frame (%d), VSYNC=%u HREF=%u, GPIO6 forced low/high=%d/%d\n",
                   error, frames, lines, low, high);
        } else {
            printk("ERROR: waiting for frame (%d), VSYNC=%u HREF=%u, GPIO6 pin test failed (%d)\n",
                   error, frames, lines, pin_test);
        }
        return;
    }

    printk("ERROR: waiting for frame (%d), VSYNC=%u HREF=%u rising edges in 150 ms\n",
           error, frames, lines);
}

static const struct gpio_dt_spec capture_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct device *const photo_usb = DEVICE_DT_GET(DT_NODELABEL(usb_serial));

static int wait_for_button_state(int state)
{
    for (;;) {
        int value = gpio_pin_get_dt(&capture_button);

        if (value < 0) {
            return value;
        }
        k_msleep(25);
        if (value == state) {
            value = gpio_pin_get_dt(&capture_button);
            if (value < 0 || value == state) {
                return value < 0 ? value : 0;
            }
        }
    }
}

static int send_usb_bytes(const void *data, size_t length)
{
    const uint8_t *bytes = data;

    while (length > 0) {
        int64_t deadline = k_uptime_get() + 1000;

        while (!usb_serial_jtag_ll_txfifo_writable()) {
            if (k_uptime_get() >= deadline) {
                return -ETIMEDOUT;
            }
            k_msleep(1);
        }

        size_t sent = usb_serial_jtag_ll_write_txfifo(bytes, MIN(length, 64));

        usb_serial_jtag_ll_txfifo_flush();
        bytes += sent;
        length -= sent;
    }

    return 0;
}

static int send_frame(const struct video_format *fmt, const struct video_buffer *frame)
{
    if (esp_ptr_external_ram(frame->buffer)) {
        int sync_ret = esp_cache_msync(frame->buffer, fmt->size,
                                      ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        if (sync_ret != 0) {
            printk("ERROR: synchronizing image memory (%d)\n", sync_ret);
            return -EIO;
        }
    }

    char header[64];
    int length = snprintk(header, sizeof(header), "\nJPEG %u %u %u\n",
                         fmt->width, fmt->height, frame->bytesused);

    if (length < 0 || (size_t)length >= sizeof(header)) {
        return -EINVAL;
    }

    int ret = send_usb_bytes(header, length);
    if (ret == 0) {
        ret = send_usb_bytes(frame->buffer, frame->bytesused);
    }
    if (ret == 0) {
        ret = send_usb_bytes("END\n", 4);
    }

    return ret;
}

int main(void)
{
    const struct device *camera = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
    const struct device *sensor = DEVICE_DT_GET(DT_PHANDLE(DT_CHOSEN(zephyr_camera), source));
    struct video_format fmt = {
        .type = VIDEO_BUF_TYPE_OUTPUT,
        .pixelformat = VIDEO_PIX_FMT_JPEG,
        .width = 2048,
        .height = 1536,
    };
    struct video_buffer *buffers[2];

    if (!device_is_ready(camera) || !device_is_ready(sensor) ||
        !device_is_ready(photo_usb) || !gpio_is_ready_dt(&capture_button)) {
        printk("ERROR: camera, native USB, or BOOT button is not ready\n");
        return 0;
    }

    int ret = gpio_pin_configure_dt(&capture_button, GPIO_INPUT);
    if (ret < 0) {
        printk("ERROR: configuring BOOT button (%d)\n", ret);
        return 0;
    }

    ret = video_set_format(camera, &fmt);
    if (ret < 0) {
        printk("ERROR: setting JPEG format (%d)\n", ret);
        return 0;
    }

    for (size_t i = 0; i < ARRAY_SIZE(buffers); i++) {
        buffers[i] = video_buffer_aligned_alloc(fmt.size, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
                                                K_NO_WAIT);
        if (buffers[i] == NULL) {
            printk("ERROR: allocating video buffer\n");
            return 0;
        }
        buffers[i]->type = VIDEO_BUF_TYPE_OUTPUT;
        if (!esp_ptr_external_ram(buffers[i]->buffer)) {
            printk("ERROR: image buffer is not in PSRAM\n");
            return 0;
        }
    }

    printk("JPEG mode %u x %u, buffer capacity %u bytes in PSRAM\n",
           fmt.width, fmt.height, fmt.size);

    for (;;) {
        struct video_buffer *frame = NULL;

        printk("Ready. Press BOOT to take a picture.\n");
        ret = wait_for_button_state(0);
        if (ret == 0) {
            ret = wait_for_button_state(1);
        }
        if (ret < 0) {
            printk("ERROR: reading BOOT button (%d)\n", ret);
            return 0;
        }

        printk("Capturing image.\n");
        for (size_t i = 0; i < ARRAY_SIZE(buffers); i++) {
            memset(buffers[i]->buffer, 0, fmt.size);
            ret = esp_cache_msync(buffers[i]->buffer, fmt.size,
                                 ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            if (ret != 0) {
                printk("ERROR: preparing capture memory (%d)\n", ret);
                return 0;
            }
            ret = video_enqueue(camera, buffers[i]);
            if (ret < 0) {
                printk("ERROR: queueing video buffer (%d)\n", ret);
                return 0;
            }
        }

        ret = video_stream_start(camera, VIDEO_BUF_TYPE_OUTPUT);
        if (ret < 0) {
            printk("ERROR: starting capture (%d)\n", ret);
            return 0;
        }

        /* The first DMA buffer can start in the middle of a sensor frame. */
        ret = video_dequeue(camera, &frame, K_SECONDS(10));
        if (ret == 0) {
            ret = video_dequeue(camera, &frame, K_SECONDS(10));
        }

        int stop_ret = video_stream_stop(camera, VIDEO_BUF_TYPE_OUTPUT);
        if (stop_ret < 0) {
            printk("ERROR: stopping capture (%d)\n", stop_ret);
            return 0;
        }

        if (ret < 0) {
            report_capture_error(sensor, ret);
        } else if (frame->bytesused == 0 ||
                   frame->bytesused > fmt.size ||
                   frame->bytesused > frame->size) {
            printk("ERROR: invalid JPEG length (%u, capacity %u)\n",
                   frame->bytesused, frame->size);
        } else {
            ret = send_frame(&fmt, frame);
            if (ret < 0) {
                printk("ERROR: sending image over native USB (%d)\n", ret);
            } else {
                printk("Image sent over native USB.\n");
            }
        }

        /* Stop returns any pending buffers to the output queue. */
        while (video_dequeue(camera, &frame, K_NO_WAIT) == 0) {
        }
    }
}
