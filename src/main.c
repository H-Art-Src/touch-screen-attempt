/*
Zephyr app for nRF52840 DK that detects touch on a 4-wire resistive
digitizer and blinks an LED while touch is present.

This variant augments sample_touch so it measures and prints the current
attributes of the GPIO pins (value, raw PIN_CNF and decoded fields)
every time sampling runs. Useful when debugging pin configuration / wiring.

Notes:
- Reading PIN_CNF is nRF52-specific (direct MMIO). This code assumes P0,
  and the standard nRF52 PIN_CNF layout:
    - DIR bit:        bit 0
    - PULL field:     bits 2..3
    - DRIVE field:    bits 8..10
    - SENSE field:    bits 16..17
  We print the raw PIN_CNF and those decoded fields.
- Printing every sample can be chatty; change the call site if you want
  less frequent diagnostics.
- Keep voltages within 3.3V and use series resistors / level shifting if required.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <stdint.h>
#include <dk_buttons_and_leds.h>


/* Change these if you wire to different pins */
#define PIN_XP 2   /* P0.02 - X+ */
#define PIN_XM 3   /* P0.03 - X- */
#define PIN_YP 4   /* P0.04 - Y+ */
#define PIN_YM 5   /* P0.05 - Y- */
#define PIN_LED 13 /* P0.13 - LED (adjust if different) */

#define SLEEP_MS 50
#define DEBOUNCE_SAMPLES 5
#define DEBOUNCE_DELAY_MS 5

/* nRF52 P0 base and PIN_CNF register access (MMIO). This is SoC-specific. */
#define P0_BASE_ADDR 0x50000000UL
#define P0_PIN_CNF_ADDR(n) (P0_BASE_ADDR + 0x700U + ((n) * 4U))
static inline uint32_t read_pin_cnf(unsigned int pin)
{
    volatile uint32_t *reg = (volatile uint32_t *)P0_PIN_CNF_ADDR(pin);
    return *reg;
}

/* Decode a few common fields from PIN_CNF (per nRF52 spec). */
static inline const char *dir_str(uint32_t cnf)
{
    return (cnf & 0x1U) ? "OUT" : "IN";
}
static inline const char *pull_str(uint32_t cnf)
{
    uint32_t pull = (cnf >> 2) & 0x3U;
    switch (pull) {
    case 0: return "NONE";
    case 1: return "PULL-DOWN";
    case 3: return "PULL-UP";
    default: return "RESERVED";
    }
}
static inline uint32_t drive_field(uint32_t cnf)
{
    return (cnf >> 8) & 0x7U;
}
static inline uint32_t sense_field(uint32_t cnf)
{
    return (cnf >> 16) & 0x3U;
}

/* Friendly printer for a pin's attributes */
static void print_pin_attributes(const struct device *gpio, int pin, const char *name)
{
    int val = gpio_pin_get_raw(gpio, pin);
    if (val < 0) {
        printk("%s (P0.%02d): gpio_pin_get_raw error %d\n", name, pin, val);
        return;
    }

    uint32_t cnf = read_pin_cnf((unsigned int)pin);

    printk("%s (P0.%02d): value=%d  PIN_CNF=0x%08x  DIR=%s  PULL=%s  DRIVE=0x%x  SENSE=0x%x\n",
           name, pin, val, cnf, dir_str(cnf), pull_str(cnf),
           drive_field(cnf), sense_field(cnf));
}

/* Get GPIO_0 in a DT-safe manner (guarding DT_NODELABEL usage). */
static const struct device *get_gpio0(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(gpio0))
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(dev)) {
        printk("DT gpio0 exists but device not ready\n");
        return NULL;
    }
    return dev;
#else
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

    /* Drive X+ HIGH, X- LOW */
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

    /* Configure Y pins as inputs with pull-down so they read LOW when not touched.
       When the screen is pressed the Y pins will be pulled toward X+ (HIGH). */
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

/* sample_touch now prints pin attributes (value + PIN_CNF decode) and returns touch present */
static bool sample_touch(const struct device *gpio)
{
	printk("\033c");
    /* Print current attributes for all relevant pins first */
    print_pin_attributes(gpio, PIN_XP, "X+");
    print_pin_attributes(gpio, PIN_XM, "X-");
    print_pin_attributes(gpio, PIN_YP, "Y+");
    print_pin_attributes(gpio, PIN_YM, "Y-");

    int high_count = 0;

    for (int i = 0; i < DEBOUNCE_SAMPLES; i++) {
        int val_yp = gpio_pin_get(gpio, PIN_YP);
        int val_ym = gpio_pin_get(gpio, PIN_YM);

        if (val_yp < 0 || val_ym < 0) {
            printk("GPIO read error: yp=%d ym=%d\n", val_yp, val_ym);
            return false;
        }

        printk("sample %d: Y+=%d, Y-=%d\n", i, val_yp, val_ym);

        if (val_yp || val_ym) {
            high_count++;
        }

        k_msleep(DEBOUNCE_DELAY_MS);
    }

    bool touched = (high_count > (DEBOUNCE_SAMPLES / 2));
    printk("Touch sample result: touched=%d (high_count=%d)\n", touched, high_count);

    return touched;
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

    /* Main loop: sample and print attributes each cycle. Reduce frequency if too chatty. */
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