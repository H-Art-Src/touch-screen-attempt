/*
Zephyr app for nRF52840 DK that detects touch on a 4-wire resistive
digitizer and blinks an LED while touch is present.

Uses DT_NODE_EXISTS / DEVICE_DT_GET guarded lookup so compilation won't
fail if DT_NODELABEL(gpio0) isn't present in the generated devicetree.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <dk_buttons_and_leds.h>

#define PIN_XP 2   /* P0.02 - X+ */
#define PIN_XM 3   /* P0.03 - X- */
#define PIN_YP 4   /* P0.04 - Y+ */
#define PIN_YM 5   /* P0.05 - Y- */
#define PIN_LED 13 /* P0.13 - LED (adjust if different) */

#define SLEEP_MS 50
#define DEBOUNCE_SAMPLES 5
#define DEBOUNCE_DELAY_MS 5

static const struct device *get_gpio0(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(gpio0))
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    /* DEVICE_DT_GET may build even if device isn't ready at runtime, so check readiness */
    if (!device_is_ready(dev)) {
        printk("DT gpio0 exists but device not ready\n");
        return NULL;
    }
    return dev;
#else
    /* Fallback to classic name if the devicetree node label isn't present */
    const struct device *dev = device_get_binding("GPIO_0");
    if (!dev) {
        printk("Fallback device_get_binding(\"GPIO_0\") failed\n");
    }
    return dev;
#endif
}

static int configure_drive_and_sense(const struct device *gpio)
{
    int rc;

    rc = gpio_pin_configure(gpio, PIN_XP, GPIO_OUTPUT_ACTIVE);
    if (rc) {
        printk("Failed to configure PIN_XP (%d)\n", rc);
        return rc;
    }

    rc = gpio_pin_configure(gpio, PIN_XM, GPIO_OUTPUT_INACTIVE);
    if (rc) {
        printk("Failed to configure PIN_XM (%d)\n", rc);
        return rc;
    }

    rc = gpio_pin_configure(gpio, PIN_YP, GPIO_INPUT | GPIO_PULL_DOWN);
    if (rc) {
        printk("Failed to configure PIN_YP (%d)\n", rc);
        return rc;
    }

    rc = gpio_pin_configure(gpio, PIN_YM, GPIO_INPUT | GPIO_PULL_DOWN);
    if (rc) {
        printk("Failed to configure PIN_YM (%d)\n", rc);
        return rc;
    }

    return 0;
}

static bool sample_touch(const struct device *gpio)
{
    int high_count = 0;

    for (int i = 0; i < DEBOUNCE_SAMPLES; i++) {
        int val_yp = gpio_pin_get(gpio, PIN_YP);
        int val_ym = gpio_pin_get(gpio, PIN_YM);

        if (val_yp < 0 || val_ym < 0) {
            printk("GPIO read error: yp=%d ym=%d\n", val_yp, val_ym);
            return false;
        }

        if (val_yp || val_ym) {
            high_count++;
        }

        k_msleep(DEBOUNCE_DELAY_MS);
    }

    return (high_count > (DEBOUNCE_SAMPLES / 2));
}

int main(void)
{
    const struct device *gpio0 = get_gpio0();
    if (!gpio0) {
        printk("Failed to obtain GPIO device (check devicetree/board)\n");
        return 1;
    }

    int rc = gpio_pin_configure(gpio0, PIN_LED, GPIO_OUTPUT_INACTIVE);
    if (rc) {
        printk("Failed to configure LED pin (%d)\n", rc);
        return 2;
    }

    rc = configure_drive_and_sense(gpio0);
    if (rc) {
        printk("Touch pins configuration failed (%d)\n", rc);
        return 3;
    }

    printk("Touch detector started (pins: X+=%d X-=%d Y+=%d Y-=%d LED=%d)\n",
           PIN_XP, PIN_XM, PIN_YP, PIN_YM, PIN_LED);

    while (1) {
        bool touched = sample_touch(gpio0);

        if (touched) {
			dk_set_led(DK_LED1, 1);
            k_msleep(150);
			dk_set_led(DK_LED1, 0);
            k_msleep(150);
        } else {
			dk_set_led(DK_LED1, 0);
            k_msleep(SLEEP_MS);
        }
    }
}