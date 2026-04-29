#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <stdint.h>
#include <dk_buttons_and_leds.h>

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)
#define SW2_NODE DT_ALIAS(sw2)
#define SW3_NODE DT_ALIAS(sw3)

static const struct gpio_dt_spec buttons[] = {
	GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(SW2_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios, {0}),
};

static struct gpio_callback button_cb_data[4];
int mode = 0;

static void flip_coins()
{
	int random = k_cycle_get_32() % 16;
	for (int i = 0; i < 4; i++) {
		dk_set_led(i, 1 & (random >> i));
	}
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	for (int i = 0; i < 4; i++) {
        const struct gpio_dt_spec *spec = &buttons[i];
        if (spec->port == NULL) {
            continue; /* alias not present / unused entry */
        }

        if (spec->port != dev) {
            continue; /* different GPIO port */
        }

        /* safe to compute mask now */
        const uint32_t mask = BIT(spec->pin);
        if ((pins & mask) == 0) {
            continue; /* this pin didn't trigger */
        }
        switch (i) {
            case 0:
                flip_coins();
                break;
            case 3:
                for (int j = 0; j < 4; j++) {
                    dk_set_led(j, j % 2);
                }
                break;
        }
        mode = i;
        return;
    }
}

int main(void)
{
	dk_leds_init();

	for (int i = 0; i < 4; i++) {
		gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(&buttons[i], GPIO_INT_EDGE_TO_ACTIVE);
		gpio_init_callback(&button_cb_data[i], button_pressed, BIT(buttons[i].pin));
		gpio_add_callback(buttons[i].port, &button_cb_data[i]);
	}

	while (1) {
        if (mode == 2)
            k_sleep(K_MSEC(1000 * (k_cycle_get_32() % 10)));
        else
		    k_sleep(K_MSEC(1000));
        if(mode == 1 || mode == 2)
            flip_coins();
	}
}