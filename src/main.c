/* 2026-04-22 - Zigbee LED candle, Sleepy End Device by Claude
 * 2026-04-22 - Button now triggers network steering when not joined (Claude)
 * GPIO 1.11: candle LED (PWM0 ch0)
 * GPIO 0.31: pairing/identify button (active low)
 * Short press: enter identify/pairing mode
 * Hold 5s: factory reset
 * LED blinks 2Hz until joined to Zigbee network, then switches to flicker.
 * Reports as ZHA dimmable light - auto-discovered by Home Assistant
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/random/random.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_zcl_scenes.h>
#include <zb_nrf_platform.h>
#include "zb_dimmable_light.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define CANDLE_ENDPOINT         10
#define PWM_PERIOD_US           10000U  /* 100 Hz */
#define FLICKER_INTERVAL_MS     50      /* flicker update rate */
#define FACTORY_RESET_MS        5000    /* hold button this long to reset */

#define MANUF_NAME      "DIY"
#define MODEL_ID        "LED_Candle_v1"
#define DATE_CODE       "20260421"
#define LOCATION_DESC   "Home"

#define BULB_INIT_BASIC_APP_VERSION     01
#define BULB_INIT_BASIC_STACK_VERSION   10
#define BULB_INIT_BASIC_HW_VERSION      11
#define BULB_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_BATTERY
#define BULB_INIT_BASIC_PH_ENV          ZB_ZCL_BASIC_ENV_UNSPECIFIED

/* Hardware bindings from devicetree */
static const struct pwm_dt_spec led_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(candle_pwm));
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(pairing_button), gpios);

/* Candle state - written from Zigbee callbacks, read from flicker timer */
static volatile uint8_t candle_base_level = 255;
static volatile bool candle_on = false;
static volatile bool zigbee_joined = false;

/* Flicker state - only accessed in system workqueue */
static uint8_t flicker_current;
static uint8_t flicker_target;

/* Button state */
static int64_t btn_press_uptime_ms;
static struct gpio_callback btn_cb_data;

/* ---------- Zigbee attribute storage ---------- */

typedef struct {
	zb_zcl_basic_attrs_ext_t      basic_attr;
	zb_zcl_identify_attrs_t       identify_attr;
	zb_zcl_scenes_attrs_t         scenes_attr;
	zb_zcl_groups_attrs_t         groups_attr;
	zb_zcl_on_off_attrs_t         on_off_attr;
	zb_zcl_level_control_attrs_t  level_control_attr;
} bulb_device_ctx_t;

static bulb_device_ctx_t dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(groups_attr_list,
	&dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(scenes_attr_list,
	&dev_ctx.scenes_attr.scene_count,
	&dev_ctx.scenes_attr.current_scene,
	&dev_ctx.scenes_attr.current_group,
	&dev_ctx.scenes_attr.scene_valid,
	&dev_ctx.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.app_version,
	&dev_ctx.basic_attr.stack_version,
	&dev_ctx.basic_attr.hw_version,
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id,
	dev_ctx.basic_attr.date_code,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.location_id,
	&dev_ctx.basic_attr.ph_env,
	dev_ctx.basic_attr.sw_ver);

ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(on_off_attr_list,
	&dev_ctx.on_off_attr.on_off);

ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST(level_control_attr_list,
	&dev_ctx.level_control_attr.current_level,
	&dev_ctx.level_control_attr.remaining_time);

ZB_DECLARE_DIMMABLE_LIGHT_CLUSTER_LIST(dimmable_light_clusters,
	basic_attr_list, identify_attr_list, groups_attr_list,
	scenes_attr_list, on_off_attr_list, level_control_attr_list);

ZB_DECLARE_DIMMABLE_LIGHT_EP(dimmable_light_ep, CANDLE_ENDPOINT,
	dimmable_light_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(dimmable_light_ctx, dimmable_light_ep);

/* ---------- PWM / flicker ---------- */

static void set_pwm_level(uint8_t level)
{
	uint32_t pulse = (uint32_t)level * PWM_PERIOD_US / 255U;

	pwm_set_dt(&led_pwm, PWM_USEC(PWM_PERIOD_US), PWM_USEC(pulse));
}

static void flicker_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!candle_on) {
		set_pwm_level(0);
		return;
	}

	uint8_t base = candle_base_level;
	uint32_t rnd = sys_rand32_get();

	/* New target on every tick with 70% probability, or when reached */
	if (flicker_current == flicker_target || (rnd & 0xF) < 11) {
		/* Occasional deep dip (10% chance) simulates gust */
		if ((sys_rand32_get() & 0x1F) == 0) {
			flicker_target = (uint8_t)CLAMP((int32_t)base * 2 / 10,
							8, base);
		} else {
			/* Normal flicker: ±45% of base */
			int32_t offset = (int32_t)(sys_rand32_get() & 0xFF) - 128;
			offset = offset * 9 / 10 * base / 255;
			flicker_target = (uint8_t)CLAMP((int32_t)base + offset,
							base * 3 / 10, base);
		}
	}

	/* Fast drop, slower rise — like a real flame */
	if (flicker_current > flicker_target) {
		flicker_current -= MAX(2, (flicker_current - flicker_target) >> 1);
	} else if (flicker_current < flicker_target) {
		flicker_current += MAX(1, (flicker_target - flicker_current) >> 2);
	}

	set_pwm_level(flicker_current);
}

static K_WORK_DEFINE(flicker_work, flicker_work_handler);

static void flicker_timer_cb(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&flicker_work);
}

static K_TIMER_DEFINE(flicker_timer, flicker_timer_cb, NULL);

/* ---------- Pairing blink (runs until Zigbee joins) ---------- */

static void pairing_blink_cb(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	static bool state;

	state = !state;
	set_pwm_level(state ? 255U : 0U);
}

static K_TIMER_DEFINE(pairing_blink_timer, pairing_blink_cb, NULL);

static void on_joined(void)
{
	if (zigbee_joined) {
		return;
	}
	zigbee_joined = true;
	k_timer_stop(&pairing_blink_timer);

	/* Restore candle state and start flicker */
	candle_on = (bool)dev_ctx.on_off_attr.on_off;
	candle_base_level = dev_ctx.level_control_attr.current_level;
	flicker_current = candle_base_level;
	flicker_target  = candle_base_level;
	k_timer_start(&flicker_timer, K_MSEC(FLICKER_INTERVAL_MS),
		      K_MSEC(FLICKER_INTERVAL_MS));
	zb_ieee_addr_t ieee;
	zb_get_long_address(ieee);
	LOG_INF("joined network, NWK: 0x%04x IEEE: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		zb_get_short_address(),
		ieee[7], ieee[6], ieee[5], ieee[4],
		ieee[3], ieee[2], ieee[1], ieee[0]);
	LOG_INF("joined network, flicker started");
}

/* ---------- Zigbee level/on-off control ---------- */

static void light_set_brightness(zb_uint8_t level)
{
	candle_base_level = level;
	/* Flicker timer applies it on next tick */
}

static void level_control_set_value(zb_uint16_t new_level)
{
	LOG_INF("level -> %d", new_level);
	ZB_ZCL_SET_ATTRIBUTE(CANDLE_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
		(zb_uint8_t *)&new_level, ZB_FALSE);
	light_set_brightness((zb_uint8_t)new_level);
}

static void on_off_set_value(zb_bool_t on)
{
	LOG_INF("on/off -> %d", on);
	ZB_ZCL_SET_ATTRIBUTE(CANDLE_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&on, ZB_FALSE);

	candle_on = (bool)on;
	if (!on) {
		set_pwm_level(0);
	} else {
		light_set_brightness(dev_ctx.level_control_attr.current_level);
	}
}

/* ---------- Zigbee identify ---------- */

static void toggle_identify_led(zb_bufid_t bufid)
{
	static bool blink;

	blink = !blink;
	set_pwm_level(blink ? 255U : 0U);
	ZB_SCHEDULE_APP_ALARM(toggle_identify_led, bufid,
		ZB_MILLISECONDS_TO_BEACON_INTERVAL(200));
}

static void identify_cb(zb_bufid_t bufid)
{
	if (bufid) {
		ZB_SCHEDULE_APP_CALLBACK(toggle_identify_led, bufid);
	} else {
		zb_ret_t err = ZB_SCHEDULE_APP_ALARM_CANCEL(
			toggle_identify_led, ZB_ALARM_ANY_PARAM);
		ZVUNUSED(err);
		on_off_set_value((zb_bool_t)dev_ctx.on_off_attr.on_off);
	}
}

static void start_identifying(zb_bufid_t bufid)
{
	ZVUNUSED(bufid);
	if (!ZB_JOINED()) {
		LOG_WRN("not joined - cannot identify");
		return;
	}
	ZB_ERROR_CHECK(zb_bdb_finding_binding_target(CANDLE_ENDPOINT));
	LOG_INF("identify mode started");
}

/* ---------- Button ---------- */

static void btn_isr(const struct device *port, struct gpio_callback *cb,
		    gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	bool pressed = gpio_pin_get_dt(&btn);

	if (pressed) {
		btn_press_uptime_ms = k_uptime_get();
	} else {
		int64_t held_ms = k_uptime_get() - btn_press_uptime_ms;

		if (held_ms >= FACTORY_RESET_MS) {
			ZB_SCHEDULE_APP_CALLBACK(zb_bdb_reset_via_local_action, 0);
			LOG_INF("factory reset triggered");
		} else if (held_ms >= 50) {
			if (!ZB_JOINED()) {
				/* Clear credentials and reboot into fresh pairing */
				ZB_SCHEDULE_APP_CALLBACK(zb_bdb_reset_via_local_action, 0);
				LOG_INF("credentials cleared, repairing");
			} else {
				ZB_SCHEDULE_APP_CALLBACK(start_identifying, 0);
			}
		}
	}
}

/* ---------- ZCL callback ---------- */

static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_zcl_device_callback_param_t *p =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

	p->status = RET_OK;

	switch (p->device_cb_id) {
	case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
		level_control_set_value(
			p->cb_param.level_control_set_value_param.new_value);
		break;

	case ZB_ZCL_SET_ATTR_VALUE_CB_ID: {
		uint8_t cluster = p->cb_param.set_attr_value_param.cluster_id;
		uint8_t attr    = p->cb_param.set_attr_value_param.attr_id;

		if (cluster == ZB_ZCL_CLUSTER_ID_ON_OFF &&
		    attr == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			on_off_set_value((zb_bool_t)
				p->cb_param.set_attr_value_param.values.data8);
		} else if (cluster == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
			   attr == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID) {
			level_control_set_value(
				p->cb_param.set_attr_value_param.values.data16);
		} else {
			p->status = RET_NOT_IMPLEMENTED;
		}
		break;
	}

	default:
		if (zcl_scenes_cb(bufid) == ZB_FALSE) {
			p->status = RET_NOT_IMPLEMENTED;
		}
		break;
	}
}

/* ---------- Zigbee signal handler ---------- */

void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *hdr = NULL;
	zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &hdr);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

	switch (sig) {
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
	case ZB_BDB_SIGNAL_STEERING:
		if (status == RET_OK) {
			on_joined();
		}
		break;
	default:
		break;
	}

	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
	if (bufid) {
		zb_buf_free(bufid);
	}
}

/* ---------- Attribute init ---------- */

static void clusters_attr_init(void)
{
	dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.app_version   = BULB_INIT_BASIC_APP_VERSION;
	dev_ctx.basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
	dev_ctx.basic_attr.hw_version    = BULB_INIT_BASIC_HW_VERSION;
	dev_ctx.basic_attr.power_source  = BULB_INIT_BASIC_POWER_SOURCE;
	dev_ctx.basic_attr.ph_env        = BULB_INIT_BASIC_PH_ENV;

	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.mf_name,
		MANUF_NAME, ZB_ZCL_STRING_CONST_SIZE(MANUF_NAME));
	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.model_id,
		MODEL_ID, ZB_ZCL_STRING_CONST_SIZE(MODEL_ID));
	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.date_code,
		DATE_CODE, ZB_ZCL_STRING_CONST_SIZE(DATE_CODE));
	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.location_id,
		LOCATION_DESC, ZB_ZCL_STRING_CONST_SIZE(LOCATION_DESC));

	dev_ctx.identify_attr.identify_time =
		ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	dev_ctx.on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;
	dev_ctx.level_control_attr.current_level =
		ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE;
	dev_ctx.level_control_attr.remaining_time =
		ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;

	ZB_ZCL_SET_ATTRIBUTE(CANDLE_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&dev_ctx.on_off_attr.on_off, ZB_FALSE);

	ZB_ZCL_SET_ATTRIBUTE(CANDLE_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
		(zb_uint8_t *)&dev_ctx.level_control_attr.current_level,
		ZB_FALSE);
}

/* ---------- main ---------- */

int main(void)
{
	printk("\r\n\r\n*** LED Candle starting ***\r\n");
	LOG_INF("LED candle starting");

	/* PWM init */
	if (!device_is_ready(led_pwm.dev)) {
		LOG_ERR("PWM device not ready");
		return -1;
	}
	set_pwm_level(0);

	/* Button init */
	if (!device_is_ready(btn.port)) {
		LOG_ERR("Button GPIO not ready");
		return -1;
	}
	gpio_pin_configure_dt(&btn, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&btn_cb_data, btn_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb_data);

	/* Settings */
	int err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings init failed: %d", err);
	}

	/* Zigbee */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);
	ZB_AF_REGISTER_DEVICE_CTX(&dimmable_light_ctx);
	clusters_attr_init();

	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(CANDLE_ENDPOINT, identify_cb);
	zcl_scenes_init();

	err = settings_load();
	if (err) {
		LOG_ERR("settings load failed: %d", err);
	}

	/* Delay before enabling Zigbee/sleep so bootloader double-tap RST still works */
	k_msleep(2000);

	zigbee_configure_sleepy_behavior(true);
	zigbee_enable();

	/* Blink at 2Hz until we join a network */
	k_timer_start(&pairing_blink_timer, K_MSEC(250), K_MSEC(250));

	LOG_INF("LED candle ready - waiting for Zigbee network");

	/* Nothing left for main - Zigbee runs in its own thread */
	return 0;
}
