/*
 * Battery / charging indicator on the XIAO BLE onboard RGB LED.
 *
 * - Boot (and wake from deep sleep, which is a reboot): the battery level
 *   is shown for CONFIG_CYGNUS_BATTERY_LED_BOOT_MS.
 *     green  >= LEVEL_HIGH
 *     yellow >= LEVEL_LOW
 *     red    <  LEVEL_LOW
 * - USB power present:
 *     blinking cyan   = charging
 *     solid green     = charge complete
 * - Otherwise the LED stays off.
 *
 * Charging state comes from the BQ25101 charger's ~CHG output, which the
 * XIAO BLE routes to P0.17 (low while charging). USB power is detected via
 * the nRF52840 VBUS detector, so this also works on the split peripheral.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_power.h>

#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/battery_state_changed.h>

#define TICK_MS 250
#define IDLE_POLL_MS 2000
#define BATTERY_WAIT_MS 3000

#define CHG_PIN 17

enum color {
    COLOR_OFF = 0,
    COLOR_RED = BIT(0),
    COLOR_GREEN = BIT(1),
    COLOR_BLUE = BIT(2),
    COLOR_YELLOW = COLOR_RED | COLOR_GREEN,
    COLOR_CYAN = COLOR_GREEN | COLOR_BLUE,
};

static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct device *const chg_port = DEVICE_DT_GET(DT_NODELABEL(gpio0));

enum phase {
    PHASE_WAIT_BATTERY,
    PHASE_BOOT,
    PHASE_STATUS,
    PHASE_SLEEP,
};

static enum phase phase = PHASE_WAIT_BATTERY;
static int64_t phase_start;
static uint32_t tick_count;
static bool ready;

static void set_color(enum color c) {
    gpio_pin_set_dt(&led_red, (c & COLOR_RED) ? 1 : 0);
    gpio_pin_set_dt(&led_green, (c & COLOR_GREEN) ? 1 : 0);
    gpio_pin_set_dt(&led_blue, (c & COLOR_BLUE) ? 1 : 0);
}

static enum color battery_color(uint8_t soc) {
    if (soc >= CONFIG_CYGNUS_BATTERY_LED_LEVEL_HIGH) {
        return COLOR_GREEN;
    }
    if (soc >= CONFIG_CYGNUS_BATTERY_LED_LEVEL_LOW) {
        return COLOR_YELLOW;
    }
    return COLOR_RED;
}

static bool usb_powered(void) { return nrf_power_usbregstatus_vbusdet_get(NRF_POWER); }

static bool charging(void) { return gpio_pin_get(chg_port, CHG_PIN) == 0; }

static void set_phase(enum phase p) {
    phase = p;
    phase_start = k_uptime_get();
}

static void led_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led_work, led_work_handler);

static void led_work_handler(struct k_work *work) {
    int64_t elapsed = k_uptime_get() - phase_start;

    switch (phase) {
    case PHASE_SLEEP:
        set_color(COLOR_OFF);
        return;

    case PHASE_WAIT_BATTERY:
        /* No reading yet - skip the boot indication rather than hang */
        if (elapsed >= BATTERY_WAIT_MS) {
            set_phase(PHASE_STATUS);
        }
        break;

    case PHASE_BOOT:
        if (elapsed < CONFIG_CYGNUS_BATTERY_LED_BOOT_MS) {
            set_color(battery_color(zmk_battery_state_of_charge()));
            break;
        }
        set_phase(PHASE_STATUS);
        __fallthrough;

    case PHASE_STATUS:
        if (!usb_powered()) {
            set_color(COLOR_OFF);
            k_work_schedule(&led_work, K_MSEC(IDLE_POLL_MS));
            return;
        }
        if (charging()) {
            /* 1 s period, 50% duty */
            set_color((tick_count++ / 2) % 2 == 0 ? COLOR_CYAN : COLOR_OFF);
        } else {
            tick_count = 0;
            set_color(COLOR_GREEN);
        }
        break;
    }

    k_work_schedule(&led_work, K_MSEC(TICK_MS));
}

static int battery_led_listener(const zmk_event_t *eh) {
    if (as_zmk_battery_state_changed(eh) != NULL) {
        if (phase == PHASE_WAIT_BATTERY) {
            set_phase(PHASE_BOOT);
            if (ready) {
                k_work_reschedule(&led_work, K_NO_WAIT);
            }
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev != NULL && ev->state == ZMK_ACTIVITY_SLEEP) {
        /* GPIO outputs are retained in System OFF, so make sure the LED is dark */
        set_phase(PHASE_SLEEP);
        if (ready) {
            k_work_cancel_delayable(&led_work);
            set_color(COLOR_OFF);
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_led, battery_led_listener);
ZMK_SUBSCRIPTION(battery_led, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(battery_led, zmk_activity_state_changed);

static int battery_led_init(void) {
    if (!gpio_is_ready_dt(&led_red) || !gpio_is_ready_dt(&led_green) ||
        !gpio_is_ready_dt(&led_blue) || !device_is_ready(chg_port)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(chg_port, CHG_PIN, GPIO_INPUT | GPIO_PULL_UP);

    /* The first battery reading may already have moved us to PHASE_BOOT */
    ready = true;
    k_work_schedule(&led_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(battery_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
