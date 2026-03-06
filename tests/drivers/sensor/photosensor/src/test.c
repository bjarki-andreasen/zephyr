/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/adc/adc_fake.h>
#include <zephyr/drivers/gpio/gpio_fake.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define TEST_SENSOR_NODE DT_NODELABEL(test_photosensor)
#define TEST_ADC_NODE DT_NODELABEL(test_adc)
#define TEST_SETTLE_TIME_NS DT_PROP_OR(DT_NODELABEL(test_photosensor), settle_time_ns, 0)
#define TEST_SETTLE_TIME_MS (TEST_SETTLE_TIME_NS / NSEC_PER_MSEC)
#define TEST_RX_BUF_SIZE 32
#define TEST_ADC_READ_TIMEOUT_MS 1
#define TEST_ADC_READ_TIMEOUT K_MSEC(TEST_ADC_READ_TIMEOUT_MS)

DEFINE_FFF_GLOBALS;

static uint16_t test_adc_read_values[] = DT_PROP(DT_PATH(zephyr_user), adc_read_values);
static uint32_t test_lux_values[] = DT_PROP(DT_PATH(zephyr_user), lux_values);
static size_t test_values_index;

SENSOR_DT_READ_IODEV(
	test_iodev,
	TEST_SENSOR_NODE,
	{SENSOR_CHAN_AMBIENT_LIGHT, 0}
);

static const struct sensor_decoder_api *test_decoder =
	SENSOR_DECODER_DT_GET(TEST_SENSOR_NODE);

RTIO_DEFINE(test_rtio, 1, 1);

ZTEST_SUITE(sensor_photosensor, NULL, NULL, NULL, NULL, NULL);

static const struct device *test_adc_dev = DEVICE_DT_GET(TEST_ADC_NODE);
static adc_sequence_callback test_adc_read_done_callback;
static const struct adc_sequence *test_adc_read_done_sequence;
static int64_t test_adc_read_uptime;

static void test_adc_read_timer_handler(struct k_timer *timer)
{
	enum adc_action action;
	uint16_t *value;

	value = (uint16_t *)test_adc_read_done_sequence->buffer;
	*value = test_adc_read_values[test_values_index];

	action = test_adc_read_done_callback(test_adc_dev, test_adc_read_done_sequence, 0);
	zassert_equal(action, ADC_ACTION_CONTINUE);
}

static K_TIMER_DEFINE(test_adc_read_timer, test_adc_read_timer_handler, NULL);

static int test_adc_read_fake_async(const struct device *dev,
				    const struct adc_sequence *sequence,
				    struct k_poll_signal *async)
{
	zassert_equal(dev, test_adc_dev);
	zassert_not_null(sequence);
	zassert_is_null(async);

	test_adc_read_done_callback = sequence->options->callback;
	test_adc_read_done_sequence = sequence;
	test_adc_read_uptime = k_uptime_get();
	k_timer_start(&test_adc_read_timer, TEST_ADC_READ_TIMEOUT, K_FOREVER);
	return 0;
}

static int64_t test_gpio_pin_configure_fake_0_uptime;

static int test_gpio_pin_configure_fake_0(const struct device *port,
					  gpio_pin_t pin,
					  gpio_flags_t flags)
{
	zassert_true(flags & GPIO_OUTPUT);
	test_gpio_pin_configure_fake_0_uptime = k_uptime_get();
	return 0;
}

static int64_t test_gpio_pin_configure_fake_1_uptime;

static int test_gpio_pin_configure_fake_1(const struct device *port,
					  gpio_pin_t pin,
					  gpio_flags_t flags)
{
	zassert_false(flags & GPIO_OUTPUT);
	test_gpio_pin_configure_fake_1_uptime = k_uptime_get();
	return 0;
}

typedef int (*test_gpio_pin_configure_fake)(const struct device *port,
					    gpio_pin_t pin,
					    gpio_flags_t flags);

static test_gpio_pin_configure_fake test_gpio_pin_configure_fake_seq_0[] = {
	test_gpio_pin_configure_fake_0,
	test_gpio_pin_configure_fake_1,
};

static uint32_t test_lux_q31_to_u(q31_t light)
{
	return light / (INT32_MAX >> 14);
}

ZTEST(sensor_photosensor, test_single_read)
{
	int ret;
	uint8_t rx_buf[TEST_RX_BUF_SIZE];
	struct sensor_q31_data data;
	struct sensor_decode_context decoder_ctx =
		SENSOR_DECODE_CONTEXT_INIT(test_decoder, rx_buf, SENSOR_CHAN_AMBIENT_LIGHT, 0);
	int64_t ms;

	adc_fake_read_async_fake.custom_fake = test_adc_read_fake_async;
	gpio_fake_pin_configure_fake.custom_fake_seq = test_gpio_pin_configure_fake_seq_0;
	gpio_fake_pin_configure_fake.custom_fake_seq_len =
		ARRAY_SIZE(test_gpio_pin_configure_fake_seq_0);

	ret = sensor_read(&test_iodev, &test_rtio, rx_buf, sizeof(rx_buf));
	zassert_ok(ret);
	ms = test_adc_read_uptime - test_gpio_pin_configure_fake_0_uptime;
	zassert_true(ms >= TEST_SETTLE_TIME_MS);
	zassert_equal(sensor_decode(&decoder_ctx, &data, 1), 1);
	zassert_equal(data.shift, 14);
	zassert_equal(test_lux_q31_to_u(data.readings->light),
		      test_lux_values[test_values_index]);
}
