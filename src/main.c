#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <stdint.h>
#include <dk_buttons_and_leds.h>

int main(void)
{
	dk_leds_init();
    dk_set_led(DK_LED4, 1);
}