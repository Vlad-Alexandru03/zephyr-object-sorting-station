#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define OV3660_ADDRESS 0x3C

int main(void)
{
    const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    const uint8_t pid_high_reg[] = {0x30, 0x0A};
    const uint8_t pid_low_reg[] = {0x30, 0x0B};
    uint8_t high;
    uint8_t low;
    int ret;

    if (!device_is_ready(bus)) {
        printk("Camera I2C bus is not ready\n");
        return 0;
    }

    ret = i2c_write_read(bus, OV3660_ADDRESS, pid_high_reg,
                         sizeof(pid_high_reg), &high, sizeof(high));
    if (ret < 0) {
        printk("Read of register 0x300A failed (%d)\n", ret);
        return 0;
    }

    ret = i2c_write_read(bus, OV3660_ADDRESS, pid_low_reg,
                         sizeof(pid_low_reg), &low, sizeof(low));
    if (ret < 0) {
        printk("Read of register 0x300B failed (%d)\n", ret);
        return 0;
    }

    printk("Camera PID: 0x%04x\n", ((unsigned int)high << 8) | low);
    return 0;
}