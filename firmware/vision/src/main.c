#include <errno.h>
#include <esp_cache.h>
#include <esp_memory_utils.h>
#include <esp_wifi.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/net/http/client.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define QBOT_IP "192.168.0.121"
#define QBOT_PORT 8000
#define MAX_JPEG_BYTES (1024U * 1024U)
#define HTTP_TIMEOUT_MS 30000
#define CAPTURE_ATTEMPTS 3
#define WIFI_RECONNECT_TIMEOUT_MS 20000

static const struct gpio_dt_spec capture_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

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

static int reconnect_wifi(struct net_if *iface, wifi_config_t *config)
{
    esp_err_t ret = esp_wifi_start();

    if (ret == ESP_OK) {
        ret = esp_wifi_set_config(WIFI_IF_STA, config);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_connect();
    }
    if (ret != ESP_OK) {
        printk("ERROR: restarting Wi-Fi (%d)\n", (int)ret);
        return -EIO;
    }

    const int64_t deadline = k_uptime_get() + WIFI_RECONNECT_TIMEOUT_MS;
    wifi_ap_record_t ap;

    while (k_uptime_get() < deadline) {
        if (net_if_is_up(iface) && esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
            net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL) {
            return 0;
        }
        k_msleep(100);
    }

    printk("ERROR: Wi-Fi did not reconnect and get an IPv4 address\n");
    return -ETIMEDOUT;
}

static int upload_response(struct http_response *response,
                           enum http_final_call final_data, void *user_data)
{
    if (final_data == HTTP_DATA_FINAL) {
        uint16_t *status = user_data;

        *status = response->http_status_code;
    }

    return 0;
}

static int post_jpeg(const uint8_t *jpeg, size_t length)
{
    struct sockaddr_in destination = {
        .sin_family = AF_INET,
        .sin_port = net_htons(QBOT_PORT),
    };
    static uint8_t response_buffer[512];
    char checksum_header[32];
    const char *const headers[] = {
        "X-Camera-Orientation: rotate-180\r\n",
        checksum_header,
        "Connection: close\r\n",
        NULL,
    };
    uint16_t status = 0;

    snprintk(checksum_header, sizeof(checksum_header), "X-JPEG-CRC32: %08x\r\n",
             (unsigned int)crc32_ieee(jpeg, length));

    if (zsock_inet_pton(AF_INET, QBOT_IP, &destination.sin_addr) != 1) {
        return -EINVAL;
    }

    int sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        return -errno;
    }

    if (zsock_connect(sock, (const struct sockaddr *)&destination,
                      sizeof(destination)) < 0) {
        int error = -errno;

        zsock_close(sock);
        return error;
    }

    struct http_request request = {
        .method = HTTP_POST,
        .url = "/capture",
        .host = QBOT_IP,
        .port = STRINGIFY(QBOT_PORT),
        .protocol = "HTTP/1.1",
        .header_fields = headers,
        .content_type_value = "image/jpeg",
        .payload = (const char *)jpeg,
        .payload_len = length,
        .response = upload_response,
        .recv_buf = response_buffer,
        .recv_buf_len = sizeof(response_buffer),
    };

    int ret = http_client_req(sock, &request, HTTP_TIMEOUT_MS, &status);
    zsock_close(sock);

    if (ret < 0) {
        return ret;
    }

    printk("Qbot returned HTTP %u\n", (unsigned int)status);
    if (status == 422) {
        return -EBADMSG;
    }

    return status == 200 ? 0 : -EIO;
}

static int send_frame(const struct video_format *fmt, const struct video_buffer *frame)
{
    if (frame->bytesused == 0 || frame->bytesused > fmt->size ||
        frame->bytesused > frame->size) {
        return -EBADMSG;
    }

    if (esp_ptr_external_ram(frame->buffer)) {
        int ret = esp_cache_msync(frame->buffer, fmt->size,
                                  ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        if (ret != 0) {
            printk("ERROR: synchronizing image memory (%d)\n", ret);
            return -EIO;
        }
    }

    const uint8_t *jpeg = frame->buffer;
    size_t length = frame->bytesused;

    while (length > 0 && jpeg[length - 1] == 0) {
        length--;
    }

    if (length < 4 || jpeg[0] != 0xff || jpeg[1] != 0xd8 ||
        jpeg[length - 2] != 0xff || jpeg[length - 1] != 0xd9) {
        printk("ERROR: invalid JPEG markers\n");
        return -EBADMSG;
    }

    if (length > MAX_JPEG_BYTES) {
        printk("ERROR: JPEG exceeds Qbot's 1 MiB upload limit\n");
        return -EMSGSIZE;
    }

    printk("Uploading %u JPEG bytes to Qbot\n", (unsigned int)length);
    return post_jpeg(jpeg, length);
}

static int capture_frames(const struct device *camera, size_t size,
                          struct video_buffer *buffers[2],
                          struct video_buffer **first, struct video_buffer **second)
{
    struct video_buffer *pending;
    int ret;

    for (size_t i = 0; i < 2; i++) {
        memset(buffers[i]->buffer, 0, size);
        ret = esp_cache_msync(buffers[i]->buffer, size,
                              ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        if (ret != 0) {
            printk("ERROR: preparing capture memory (%d)\n", ret);
            return -EIO;
        }
        ret = video_enqueue(camera, buffers[i]);
        if (ret < 0) {
            printk("ERROR: queueing video buffer (%d)\n", ret);
            return ret;
        }
    }

    ret = video_stream_start(camera, VIDEO_BUF_TYPE_OUTPUT);
    if (ret < 0) {
        printk("ERROR: starting capture (%d)\n", ret);
        return ret;
    }

    ret = video_dequeue(camera, first, K_SECONDS(10));
    if (ret == 0) {
        ret = video_dequeue(camera, second, K_SECONDS(10));
    }

    int stop_ret = video_stream_stop(camera, VIDEO_BUF_TYPE_OUTPUT);
    if (stop_ret < 0) {
        printk("ERROR: stopping capture (%d)\n", stop_ret);
        return stop_ret;
    }

    while (video_dequeue(camera, &pending, K_NO_WAIT) == 0) {
    }

    if (ret < 0) {
        printk("ERROR: waiting for frame (%d)\n", ret);
    }

    return 0;
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
        !gpio_is_ready_dt(&capture_button)) {
        printk("ERROR: camera, sensor, or BOOT button is not ready\n");
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
        printk("Ready. Press BOOT to take a picture.\n");
        ret = wait_for_button_state(0);
        if (ret == 0) {
            ret = wait_for_button_state(1);
        }
        if (ret < 0) {
            printk("ERROR: reading BOOT button (%d)\n", ret);
            return 0;
        }

        struct net_if *iface = net_if_get_default();
        wifi_config_t wifi_config;
        wifi_ap_record_t ap;

        if (iface == NULL || esp_wifi_sta_get_ap_info(&ap) != ESP_OK ||
            esp_wifi_get_config(WIFI_IF_STA, &wifi_config) != ESP_OK) {
            printk("Connect to Wi-Fi in the shell, then press BOOT again.\n");
            continue;
        }

        for (int attempt = 1; attempt <= CAPTURE_ATTEMPTS; attempt++) {
            struct video_buffer *first = NULL;
            struct video_buffer *second = NULL;

            /* Wi-Fi activity during capture corrupted some JPEGs. */
            esp_err_t wifi_ret = esp_wifi_stop();
            if (wifi_ret != ESP_OK) {
                printk("ERROR: stopping Wi-Fi for capture (%d)\n", (int)wifi_ret);
                break;
            }
            net_dhcpv4_stop(iface);
            k_msleep(100);

            printk("Capturing image with Wi-Fi off (attempt %d/%d).\n",
                   attempt, CAPTURE_ATTEMPTS);
            ret = capture_frames(camera, fmt.size, buffers, &first, &second);
            if (ret < 0) {
                return 0;
            }

            printk("Reconnecting Wi-Fi for upload.\n");
            if (reconnect_wifi(iface, &wifi_config) < 0) {
                break;
            }

            ret = second ? send_frame(&fmt, second) : -EBADMSG;
            if (ret == -EBADMSG && first) {
                printk("Trying the other captured frame.\n");
                ret = send_frame(&fmt, first);
            }

            if (ret == 0) {
                printk("Image uploaded successfully.\n");
                break;
            }
            if (ret != -EBADMSG) {
                printk("ERROR: uploading image to Qbot (%d)\n", ret);
                break;
            }
            if (attempt == CAPTURE_ATTEMPTS) {
                printk("ERROR: no valid JPEG after %d attempts.\n", CAPTURE_ATTEMPTS);
            }
        }
    }
}
