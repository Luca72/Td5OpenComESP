#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "ds3231.h"

#define CHECK_ARG(ARG) do { if (!ARG) return ESP_ERR_INVALID_ARG; } while (0)


/* Private */
#define TAG "I2CDEV"

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
	return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size)
{
	return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}

esp_err_t i2c_dev_init(i2c_port_t port, int sda, int scl)
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda,
		.scl_io_num = scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		//.master.clk_speed = 1000000
		.master.clk_speed = I2C_FREQ_HZ
	};
	//i2c_param_config(I2C_NUM_0, &i2c_config);
	//i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	i2c_param_config(port, &i2c_config);
	return i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
	if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (out_data && out_size)
	{
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write(cmd, (void *)out_data, out_size, true);
	}
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
	i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

	esp_err_t res = i2c_master_cmd_begin(dev->port, cmd, I2CDEV_TIMEOUT / portTICK_PERIOD_MS);
	if (res != ESP_OK)
		ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", dev->addr, dev->port, res);
	i2c_cmd_link_delete(cmd);

	return res;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
	if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
	if (out_reg && out_reg_size)
		i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
	i2c_master_write(cmd, (void *)out_data, out_size, true);
	i2c_master_stop(cmd);
	esp_err_t res = i2c_master_cmd_begin(dev->port, cmd, I2CDEV_TIMEOUT / portTICK_PERIOD_MS);
	if (res != ESP_OK)
		ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d", dev->addr, dev->port, res);
	i2c_cmd_link_delete(cmd);

	return res;
}



/* Public */
uint8_t bcd2dec(uint8_t val)
{
	return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val)
{
	return ((val / 10) << 4) + (val % 10);
}

static uint8_t conv2d(const char *p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

void buildtime2tm(const char *date, const char *time, struct tm *datetime) {
  datetime->tm_year = conv2d(date + 9) + 2000;
  // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
  switch (date[0]) {
	  case 'J':
		datetime->tm_mon = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7);
		break;
	  case 'F':
		datetime->tm_mon = 2;
		break;
	  case 'A':
		datetime->tm_mon = date[2] == 'r' ? 4 : 8;
		break;
	  case 'M':
		datetime->tm_mon = date[2] == 'r' ? 3 : 5;
		break;
	  case 'S':
		datetime->tm_mon = 9;
		break;
	  case 'O':
		datetime->tm_mon = 10;
		break;
	  case 'N':
		datetime->tm_mon = 11;
		break;
	  case 'D':
		datetime->tm_mon = 12;
		break;
	}
	datetime->tm_mday = conv2d(date + 4);
	datetime->tm_hour = conv2d(time);
	datetime->tm_min = conv2d(time + 3);
	datetime->tm_sec = conv2d(time + 6);
}


esp_err_t ds3231_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
	CHECK_ARG(dev);

	dev->port = port;
	dev->addr = DS3231_ADDR;
	dev->sda_io_num = sda_gpio;
	dev->scl_io_num = scl_gpio;
	dev->clk_speed = I2C_FREQ_HZ;
	return i2c_dev_init(port, sda_gpio, scl_gpio);
}

esp_err_t ds3231_set_time(i2c_dev_t *dev, struct tm *time)
{
	CHECK_ARG(dev);
	CHECK_ARG(time);

	uint8_t data[7];

	/* time/date data */
	data[0] = dec2bcd(time->tm_sec);
	data[1] = dec2bcd(time->tm_min);
	data[2] = dec2bcd(time->tm_hour);
	/* The week data must be in the range 1 to 7, and to keep the start on the
	 * same day as for tm_wday have it start at 1 on Sunday. */
	data[3] = dec2bcd(time->tm_wday + 1);
	data[4] = dec2bcd(time->tm_mday);
	data[5] = dec2bcd(time->tm_mon + 1);
	data[6] = dec2bcd(time->tm_year - 2000);

	return i2c_dev_write_reg(dev, DS3231_ADDR_TIME, data, 7);
}

esp_err_t ds3231_set_build_time(i2c_dev_t *dev, const char *date, const char *time)
{
	CHECK_ARG(dev);

	struct tm datetime;
	buildtime2tm(date, time, &datetime);
	return ds3231_set_time(dev, &datetime);
}

esp_err_t ds3231_get_raw_temp(i2c_dev_t *dev, int16_t *temp)
{
	CHECK_ARG(dev);
	CHECK_ARG(temp);

	uint8_t data[2];

	esp_err_t res = i2c_dev_read_reg(dev, DS3231_ADDR_TEMP, data, sizeof(data));
	if (res == ESP_OK)
		*temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;

	return res;
}

esp_err_t ds3231_get_temp_integer(i2c_dev_t *dev, int8_t *temp)
{
	CHECK_ARG(temp);

	int16_t t_int;

	esp_err_t res = ds3231_get_raw_temp(dev, &t_int);
	if (res == ESP_OK)
		*temp = t_int >> 2;

	return res;
}

esp_err_t ds3231_get_lost_power(i2c_dev_t *dev, bool *lostpower)
{
	CHECK_ARG(dev);

	uint8_t data[1];

	esp_err_t res = i2c_dev_read_reg(dev, DS3231_ADDR_STATUS, data, sizeof(data));
	if (res == ESP_OK)
		*lostpower = data[0] >> 7;

	return res;
}

esp_err_t ds3231_get_status(i2c_dev_t *dev, uint8_t *status)
{
	CHECK_ARG(dev);

	uint8_t data[1];

	esp_err_t res = i2c_dev_read_reg(dev, DS3231_ADDR_STATUS, data, sizeof(data));
	if (res == ESP_OK)
		*status = data[0];

	return res;
}

esp_err_t ds3231_reset_lost_power(i2c_dev_t *dev)
{
    uint8_t statreg;
    ds3231_get_status(dev, &statreg);
    statreg &= ~0x80; // flip OSF bit

    esp_err_t res = i2c_dev_write_reg(dev, DS3231_ADDR_STATUS, &statreg, 1);

	return res;
}

esp_err_t ds3231_get_temp_float(i2c_dev_t *dev, float *temp)
{
	CHECK_ARG(temp);

	int16_t t_int;

	esp_err_t res = ds3231_get_raw_temp(dev, &t_int);
	if (res == ESP_OK)
		*temp = t_int * 0.25;

	return res;
}

esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time)
{
	CHECK_ARG(dev);
	CHECK_ARG(time);

	uint8_t data[7];

	/* read time */
	esp_err_t res = i2c_dev_read_reg(dev, DS3231_ADDR_TIME, data, 7);
		if (res != ESP_OK) return res;

	/* convert to unix time structure */
	time->tm_sec = bcd2dec(data[0]);
	time->tm_min = bcd2dec(data[1]);
	if (data[2] & DS3231_12HOUR_FLAG)
	{
		/* 12H */
		time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
		/* AM/PM? */
		if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
	}
	else time->tm_hour = bcd2dec(data[2]); /* 24H */
	time->tm_wday = bcd2dec(data[3]) - 1;
	time->tm_mday = bcd2dec(data[4]);
	time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
	time->tm_year = bcd2dec(data[6]) + 2000;
	time->tm_isdst = 0;

	// apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
	//applyTZ(time);

	return ESP_OK;
}


