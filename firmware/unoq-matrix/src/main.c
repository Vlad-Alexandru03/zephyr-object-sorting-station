#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>


#define MATRIX_WIDTH  13
#define MATRIX_HEIGHT 8
#define ROW_BYTES     2

#define GLYPH_WIDTH   3
#define GLYPH_HEIGHT  5
#define GLYPH_GAP     1

static const uint8_t font[128][GLYPH_HEIGHT] = {
    ['A'] = {0b010, 0b101, 0b111, 0b101, 0b101},
    ['B'] = {0b110, 0b101, 0b110, 0b101, 0b110},
    ['C'] = {0b110, 0b100, 0b100, 0b100, 0b110},
    ['D'] = {0b110, 0b101, 0b101, 0b101, 0b110},
    ['E'] = {0b111, 0b100, 0b110, 0b100, 0b111},
    ['F'] = {0b110, 0b100, 0b110, 0b100, 0b100},
    ['G'] = {0b111, 0b100, 0b101, 0b101, 0b111},
    ['H'] = {0b101, 0b101, 0b111, 0b101, 0b101},
    ['I'] = {0b111, 0b010, 0b010, 0b010, 0b111},
    ['J'] = {0b001, 0b001, 0b001, 0b101, 0b111},
    ['K'] = {0b101, 0b101, 0b110, 0b101, 0b101},
    ['L'] = {0b100, 0b100, 0b100, 0b100, 0b110},
    ['M'] = {0b101, 0b111, 0b111, 0b101, 0b101},
    ['N'] = {0b101, 0b111, 0b111, 0b111, 0b101},
    ['O'] = {0b010, 0b101, 0b101, 0b101, 0b010},
    ['P'] = {0b110, 0b101, 0b110, 0b100, 0b100},
    ['Q'] = {0b010, 0b101, 0b101, 0b111, 0b011},
    ['R'] = {0b110, 0b101, 0b110, 0b101, 0b101},
    ['S'] = {0b111, 0b100, 0b111, 0b001, 0b111},
    ['T'] = {0b111, 0b010, 0b010, 0b010, 0b010},
    ['U'] = {0b101, 0b101, 0b101, 0b101, 0b111},
    ['V'] = {0b101, 0b101, 0b101, 0b101, 0b010},
    ['W'] = {0b101, 0b101, 0b111, 0b111, 0b101},
    ['X'] = {0b101, 0b101, 0b010, 0b101, 0b101},
    ['Y'] = {0b101, 0b101, 0b010, 0b010, 0b010},
    ['Z'] = {0b111, 0b001, 0b010, 0b100, 0b111},
    ['0'] = {0b111, 0b101, 0b101, 0b101, 0b111},
    ['1'] = {0b010, 0b110, 0b010, 0b010, 0b110},
    ['2'] = {0b111, 0b001, 0b111, 0b100, 0b111},
    ['3'] = {0b111, 0b001, 0b111, 0b001, 0b111},
    ['4'] = {0b101, 0b101, 0b111, 0b001, 0b001},
    ['5'] = {0b111, 0b100, 0b111, 0b001, 0b111},
    ['6'] = {0b111, 0b100, 0b111, 0b101, 0b111},
    ['7'] = {0b111, 0b001, 0b010, 0b010, 0b010},
    ['8'] = {0b111, 0b101, 0b111, 0b101, 0b111},
    ['9'] = {0b111, 0b101, 0b111, 0b001, 0b111},
    ['%'] = {0b101, 0b001, 0b010, 0b100, 0b101},
    ['?'] = {0b110, 0b001, 0b010, 0b000, 0b010},
};

static const uint8_t *glyph_for(unsigned char letter)
{
    if ((letter >= 'A' && letter <= 'Z') ||
        (letter >= '0' && letter <= '9') || letter == '%' || letter == ' ') {
        return font[letter];
    }

    return font['?'];
}

static int glyph_width(const uint8_t *glyph)
{
    uint8_t columns = 0;

    for (int row = 0; row < GLYPH_HEIGHT; row++) {
        columns |= glyph[row];
    }

    int width = GLYPH_WIDTH;
    while (width > 0 && (columns & 1U) == 0) {
        columns >>= 1;
        width--;
    }

    return width;
}


static int draw_frame(const struct device *display, const char *message, int offset)
{
    const int top = (MATRIX_HEIGHT - GLYPH_HEIGHT) / 2;
    uint8_t pixels[MATRIX_HEIGHT * ROW_BYTES] = {0};

    const struct display_buffer_descriptor desc = {
        .width = MATRIX_WIDTH,
        .height = MATRIX_HEIGHT,
        .pitch = ROW_BYTES * 8,
        .buf_size = sizeof(pixels),
    };
    int start_x = -offset;
    for (const char *letter = message; *letter && start_x < MATRIX_WIDTH; letter++) {
        const uint8_t *glyph = glyph_for(*letter);
        int width = glyph_width(glyph);

        for (int col = 0; col < width; col++) {
            int x = start_x + col;
            if (x < 0 || x >= MATRIX_WIDTH) {
                continue;
            }

            uint8_t mask = 1U << (GLYPH_WIDTH - 1 - col);
            for (int row = 0; row < GLYPH_HEIGHT; row++) {
                if (glyph[row] & mask) {
                    int y = top + row;
                    pixels[y * ROW_BYTES + x / 8] |= 1U << (x % 8);
                }
            }
        }

        start_x += width + GLYPH_GAP;
    }


    return display_write(display, 0, 0, &desc, pixels);
}


#define MESSAGE_SIZE 40

struct display_message {
    char text[MESSAGE_SIZE];
};

K_MSGQ_DEFINE(display_queue, sizeof(struct display_message), 4, 1);

static void uart_receive(const struct device *uart, void *user_data)
{
    static struct display_message incoming;
    static size_t length;
    static bool overflow;
    uint8_t bytes[16];

    ARG_UNUSED(user_data);

    uart_irq_update(uart);
    if (!uart_irq_rx_ready(uart)) {
        return;
    }

    int count;
    while ((count = uart_fifo_read(uart, bytes, sizeof(bytes))) > 0) {
        for (int i = 0; i < count; i++) {
            uint8_t byte = bytes[i];

            if (byte == '\n') {
                if (length > 0 && !overflow) {
                    incoming.text[length] = '\0';
                    if (k_msgq_put(&display_queue, &incoming, K_NO_WAIT) != 0) {
                        k_msgq_purge(&display_queue);
                        k_msgq_put(&display_queue, &incoming, K_NO_WAIT);
                    }
                }

                length = 0;
                overflow = false;
            } else if (byte != '\r' && !overflow) {
                if (length < sizeof(incoming.text) - 1) {
                    incoming.text[length++] = byte;
                } else {
                    overflow = true;
                }
            }
        }
    }
}

static int message_width(const char *text)
{
    int width = 0;

    for (const char *letter = text; *letter; letter++) {
        width += glyph_width(glyph_for(*letter)) + GLYPH_GAP;
    }

    return width;
}

int main(void)
{
    const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(lpuart1));
    struct display_message active = { .text = "WAITING" };
    struct display_message next;
    int offset = -MATRIX_WIDTH;
    int width = message_width(active.text);

    if (!device_is_ready(display) || !device_is_ready(uart)) {
        return -ENODEV;
    }

    int ret = draw_frame(display, active.text, offset);
    if (ret < 0) {
        return ret;
    }

    ret = display_blanking_off(display);
    if (ret < 0) {
        return ret;
    }

    ret = uart_irq_callback_user_data_set(uart, uart_receive, NULL);
    if (ret < 0) {
        return ret;
    }

    uart_irq_rx_enable(uart);

    while (1) {
        while (k_msgq_get(&display_queue, &next, K_NO_WAIT) == 0) {
            active = next;
            width = message_width(active.text);
            offset = -MATRIX_WIDTH;
        }

        ret = draw_frame(display, active.text, offset);
        if (ret < 0) {
            return ret;
        }

        if (++offset >= width) {
            offset = -MATRIX_WIDTH;
        }

        k_msleep(120);
    }
}
