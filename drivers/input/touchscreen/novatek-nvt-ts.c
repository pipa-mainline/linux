// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Novatek NT11205 i2c touchscreen controller as found
 * on the Acer Iconia One 7 B1-750 tablet.
 *
 * Copyright (c) 2023 Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>

#include <asm/unaligned.h>

#define DUMMY_BYTES (1)

#define NVT_TS_TOUCH_START		0x00
#define NVT_TS_TOUCH_SIZE		6

#define NVT_TS_PARAMETERS_START		0x78
/* These are offsets from NVT_TS_PARAMETERS_START */
#define NVT_TS_PARAMS_WIDTH		0x04
#define NVT_TS_PARAMS_HEIGHT		0x06
#define NVT_TS_PARAMS_MAX_TOUCH		0x09
#define NVT_TS_PARAMS_MAX_BUTTONS	0x0a
#define NVT_TS_PARAMS_IRQ_TYPE		0x0b
#define NVT_TS_PARAMS_WAKE_TYPE		0x0c
#define NVT_TS_PARAMS_CHIP_ID		0x0e
#define NVT_TS_PARAMS_SIZE		0x0f

#define NVT_TS_MAX_TOUCHES		10
#define NVT_TS_MAX_SIZE			4096

#define NVT_TS_TOUCH_INVALID		0xff
#define NVT_TS_TOUCH_SLOT_SHIFT		3
#define NVT_TS_TOUCH_TYPE_MASK		GENMASK(2, 0)
#define NVT_TS_TOUCH_NEW		1
#define NVT_TS_TOUCH_UPDATE		2
#define NVT_TS_TOUCH_RELEASE		3
#define NVT_TS_SPI_WRITE_MASK(a) (a | 0x80)
#define NVT_TS_SPI_READ_MASK(a) (a & 0x7F)
#define NVT_TS_TRANSFER_LEN	(63*1024)
#define NVT_TS_READ_LEN		(2*1024)
#define NVT_TS_RESET_REG		0x7FFF80
#define NVT_TS_SWRST_N8_ADDR  0x03F0FE
#define NVT_TS_EVENT_BUF_ADDR 0x125800
#define NVT_TS_EVENT_MAP_RESET_COMPLETE 0x60

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

static const int nvt_ts_irq_type[4] = {
	IRQF_TRIGGER_RISING,
	IRQF_TRIGGER_FALLING,
	IRQF_TRIGGER_LOW,
	IRQF_TRIGGER_HIGH
};

struct nvt_ts_data {
	struct device dev;
	struct i2c_client *client;
	struct spi_device *device;
	int irq;
	struct regmap *regmap;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data regulators[2];
	struct touchscreen_properties prop;
	int max_touches;
	u8 buf[NVT_TS_TOUCH_SIZE * NVT_TS_MAX_TOUCHES];
	uint8_t *xbuf;  // buffer for transmit
	uint8_t *rbuf;  // buffer for receive
	struct mutex xbuf_lock; // mutex for xbuf
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
};

static const struct regmap_config nvt_ts_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = NVT_TS_MAX_SIZE,
};

static irqreturn_t nvt_ts_irq(int irq, void *dev_id)
{
	struct nvt_ts_data *data = dev_id;
	struct device *dev = &data->dev;
	int i, error, slot, x, y;
	bool active;
	u8 *touch;

	error = regmap_bulk_read(data->regmap, NVT_TS_TOUCH_START, data->buf,
				 data->max_touches * NVT_TS_TOUCH_SIZE);
	if (error)
		return IRQ_HANDLED;

	for (i = 0; i < data->max_touches; i++) {
		touch = &data->buf[i * NVT_TS_TOUCH_SIZE];

		if (touch[0] == NVT_TS_TOUCH_INVALID)
			continue;

		slot = touch[0] >> NVT_TS_TOUCH_SLOT_SHIFT;
		if (slot < 1 || slot > data->max_touches) {
			dev_warn(dev, "slot %d out of range, ignoring\n", slot);
			continue;
		}

		switch (touch[0] & NVT_TS_TOUCH_TYPE_MASK) {
		case NVT_TS_TOUCH_NEW:
		case NVT_TS_TOUCH_UPDATE:
			active = true;
			break;
		case NVT_TS_TOUCH_RELEASE:
			active = false;
			break;
		default:
			dev_warn(dev, "slot %d unknown state %d\n", slot, touch[0] & 7);
			continue;
		}

		slot--;
		x = (touch[1] << 4) | (touch[3] >> 4);
		y = (touch[2] << 4) | (touch[3] & 0x0f);

		input_mt_slot(data->input, slot);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, active);
		touchscreen_report_pos(data->input, &data->prop, x, y, true);
	}

	input_mt_sync_frame(data->input);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static void nvt_ts_disable_regulators(void *_data)
{
	struct nvt_ts_data *data = _data;

	regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
}

static int nvt_ts_start(struct input_dev *dev)
{
	struct nvt_ts_data *data = input_get_drvdata(dev);
	int error;

	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(&data->dev, "failed to enable regulators\n");
		return error;
	}

	enable_irq(data->irq);
	gpiod_set_value_cansleep(data->reset_gpio, 0);

	return 0;
}

static void nvt_ts_stop(struct input_dev *dev)
{
	struct nvt_ts_data *data = input_get_drvdata(dev);

	disable_irq(data->irq);
	gpiod_set_value_cansleep(data->reset_gpio, 1);
	nvt_ts_disable_regulators(data);
}

static int nvt_ts_suspend(struct device *dev)
{
	struct nvt_ts_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		nvt_ts_stop(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int nvt_ts_resume(struct device *dev)
{
	struct nvt_ts_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		nvt_ts_start(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvt_ts_pm_ops, nvt_ts_suspend, nvt_ts_resume);

static int nvt_ts_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int error, width, height, irq_type;
	struct nvt_ts_data *data;
	struct input_dev *input;

	if (!client->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);

	data->dev = client->dev;
	data->irq = client->irq;

	// Create regmap
	data->regmap = devm_regmap_init_i2c(client, &nvt_ts_regmap_config);
	if (IS_ERR(data->regmap)) {
		error = PTR_ERR(data->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", error);
		return error;
	}

	/*
	 * VCC is the analog voltage supply
	 * IOVCC is the digital voltage supply
	 */
	data->regulators[0].supply = "vcc";
	data->regulators[1].supply = "iovcc";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(dev, "cannot get regulators: %d\n", error);
		return error;
	}

	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(dev, "failed to enable regulators\n");
		return error;
	}

	error = devm_add_action_or_reset(dev, nvt_ts_disable_regulators, data);
	if (error) {
		dev_err(dev, "failed to install regulator disable handler\n");
		return error;
	}

	data->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	error = PTR_ERR_OR_ZERO(data->reset_gpio);
	if (error) {
		dev_err(dev, "failed to request reset GPIO: %d\n", error);
		return error;
	}

	/* Wait for controller to come out of reset before params read */
	msleep(100);
	error = regmap_bulk_read(data->regmap, NVT_TS_PARAMETERS_START,
				 data->buf, NVT_TS_PARAMS_SIZE);
	gpiod_set_value_cansleep(data->reset_gpio, 1); /* Put back in reset */
	if (error)
		return error;

	width  = get_unaligned_be16(&data->buf[NVT_TS_PARAMS_WIDTH]);
	height = get_unaligned_be16(&data->buf[NVT_TS_PARAMS_HEIGHT]);
	data->max_touches = data->buf[NVT_TS_PARAMS_MAX_TOUCH];
	irq_type = data->buf[NVT_TS_PARAMS_IRQ_TYPE];

	if (width > NVT_TS_MAX_SIZE || height >= NVT_TS_MAX_SIZE ||
	    data->max_touches > NVT_TS_MAX_TOUCHES ||
	    irq_type >= ARRAY_SIZE(nvt_ts_irq_type)) {
		dev_err(dev, "Unsupported touchscreen parameters: %*ph\n",
			NVT_TS_PARAMS_SIZE, data->buf);
		return -EIO;
	}

	dev_dbg(dev, "Detected %dx%d touchscreen with %d max touches\n",
		width, height, data->max_touches);

	if (data->buf[NVT_TS_PARAMS_MAX_BUTTONS])
		dev_warn(dev, "Touchscreen buttons are not supported\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->open = nvt_ts_start;
	input->close = nvt_ts_stop;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, width - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, height - 1, 0, 0);
	touchscreen_parse_properties(input, true, &data->prop);

	error = input_mt_init_slots(input, data->max_touches,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	data->input = input;
	input_set_drvdata(input, data);

	error = devm_request_threaded_irq(dev, client->irq, NULL, nvt_ts_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN |
						nvt_ts_irq_type[irq_type],
					  client->name, data);
	if (error) {
		dev_err(dev, "failed to request irq: %d\n", error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

int32_t nvt_ts_spi_read_write(struct nvt_ts_data *ts, uint8_t *buf, size_t len, NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(ts->device, &m);
}

int32_t nvt_ts_spi_read(struct nvt_ts_data *ts, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

//	mutex_lock(&ts->xbuf_lock);

	buf[0] = NVT_TS_SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = nvt_ts_spi_read_write(ts, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		dev_err(&ts->dev,"read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

//	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

int32_t nvt_ts_spi_write(struct nvt_ts_data *ts, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

//	mutex_lock(&ts->xbuf_lock);

	buf[0] = NVT_TS_SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = nvt_ts_spi_read_write(ts, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		dev_err(&ts->dev,"error, ret=%d\n", ret);
		ret = -EIO;
	}

//	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

int32_t nvt_ts_set_page(struct nvt_ts_data *ts, uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return nvt_ts_spi_read_write(ts, buf, 3, NVTWRITE);
}

int32_t nvt_ts_get_fw_info(struct nvt_ts_data *ts)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_ts_set_page(ts, NVT_TS_EVENT_BUF_ADDR | 0x78);

	//---read fw info---
	buf[0] = 0x78;
	nvt_ts_spi_read(ts, buf, 39);
	int fw_ver = buf[1];
	int x_num = buf[3];
	int y_num = buf[4];
	int abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	int abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	int max_button_num = buf[11];
	int cascade = buf[34] & 0x01;

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		dev_err(&ts->dev, "FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		fw_ver = 0;
		x_num = 18;
		y_num = 32;
		abs_x_max = 101;
		abs_y_max = 101;
		max_button_num = 101;

		if(retry_count < 3) {
			retry_count++;
			dev_err(&ts->dev, "retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			dev_err(&ts->dev, "Set default fw_ver=%d, x_num=%d, y_num=%d, "
					"abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					fw_ver, x_num, y_num,
					abs_x_max, abs_y_max, max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	dev_err(&ts->dev, "fw_ver = 0x%02X, fw_type = 0x%02X, x_num=%d, y_num=%d\n", fw_ver, buf[14], x_num, y_num);

	return ret;
}


int32_t nvt_ts_read_addr(struct nvt_ts_data *ts, uint32_t addr, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;

	if (!ts || !buf) {
		// handle error, invalid argument
		return -EINVAL;
	}

	//mutex_lock(&ts->xbuf_lock);

	// Prepare command buffer
	ts->xbuf[0] = NVT_TS_SPI_READ_MASK(addr & 0x7F);

	ret = nvt_ts_spi_read(ts, ts->xbuf, len);
	if (ret < 0) {
		dev_err(&ts->dev, "read from addr 0x%06X failed, ret = %d\n", addr, ret);
	} else {
		memcpy(buf, ts->rbuf + 2, len); // copy the data to the provided buffer
	}

	//mutex_unlock(&ts->xbuf_lock);

	return ret;
}

int32_t nvt_ts_write_addr(struct nvt_ts_data *ts, uint32_t addr, uint8_t *data, uint16_t len)
{int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = nvt_ts_spi_write(ts, buf, 3);
	if (ret) {
		dev_err(&ts->dev,"set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = nvt_ts_spi_write(ts, buf, 2);
	if (ret) {
		dev_err(&ts->dev, "write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

#define tmp111 0x1FB50D
static int32_t nvt_ts_write_sram(struct nvt_ts_data *ts, const u8 *fwdata,
		uint32_t SRAM_addr, uint32_t size, uint32_t BIN_addr)
{
    uint8_t *fwbuf = (uint8_t *)kzalloc((NVT_TS_TRANSFER_LEN + 1 + DUMMY_BYTES), GFP_KERNEL);
    memset(fwbuf, 0, (NVT_TS_TRANSFER_LEN+1+ DUMMY_BYTES));
	int32_t ret = 0;
	uint32_t i = 0;
	uint16_t len = 0;
	int32_t count = 0;
    dev_err(&ts->dev, "sram write: called\n");
	if (size % NVT_TS_TRANSFER_LEN)
		count = (size / NVT_TS_TRANSFER_LEN) + 1;
	else
		count = (size / NVT_TS_TRANSFER_LEN);

	for (i = 0 ; i < count ; i++) {
		len = (size < NVT_TS_TRANSFER_LEN) ? size : NVT_TS_TRANSFER_LEN;
		//---set xdata index to start address of SRAM---
		ret = nvt_ts_set_page(ts, SRAM_addr);
		if (ret) {
			dev_err(&ts->dev, "set page failed, ret = %d\n", ret);
			return ret;
		}
		//---write data into SRAM---
		fwbuf[0] = SRAM_addr & 0x7F;	//offset
		memcpy(fwbuf+1, &fwdata[BIN_addr], len);	//payload
		ret = nvt_ts_spi_read_write(ts, fwbuf, len+1, NVTWRITE);

		if (ret) {
			dev_err(&ts->dev, "write to sram failed, ret = %d\n", ret);
			return ret;
		}

		SRAM_addr += NVT_TS_TRANSFER_LEN;
		BIN_addr += NVT_TS_TRANSFER_LEN;
		size -= NVT_TS_TRANSFER_LEN;
	}
    kfree(fwbuf);


	return ret;
}

int32_t nvt_check_fw_reset_state(struct nvt_ts_data *ts)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = 10;

	//---set xdata index to EVENT BUF ADDR---
	nvt_ts_set_page(ts, NVT_TS_EVENT_BUF_ADDR | NVT_TS_EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = 0x60;
		buf[1] = 0x00;
		nvt_ts_spi_read(ts, buf, 6);

        dev_err(&ts->dev,"error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
		//if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
		//	ret = 0;
		//	break;
		//}

		retry++;
		if(unlikely(retry > retry_max)) {
			dev_err(&ts->dev,"error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}

	return ret;
}
int nvt_ts_chip_id(struct nvt_ts_data *ts)
{
    uint8_t buf[8] = {0};
    nvt_ts_set_page(ts, 0x1fb104);
    buf[0] = 0x1fb104 & 0x7F;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;
    nvt_ts_spi_write(ts, buf, 7);

    buf[0] = 0x1fb104 & 0x7F;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;
	nvt_ts_spi_read(ts, buf, 7);
	dev_err(&ts->dev,"buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
    return 0;
}

int32_t nvt_ts_check_tx_auto_copy(struct nvt_ts_data *ts)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 200;

	for (i = 0; i < retry; i++) {
		//---set xdata index to SPI_MST_AUTO_COPY---
		nvt_ts_set_page(ts, 0x1FC925);

		//---read auto copy status---
		buf[0] = 0x1FC925 & 0x7F;
		buf[1] = 0xFF;
		nvt_ts_spi_read(ts, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1000);
	}

	if (i >= retry) {
		dev_err(&ts->dev, "failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

void nvt_fw_crc_enable(struct nvt_ts_data *ts)
{
	uint8_t buf[4] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_ts_set_page(ts, NVT_TS_EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = NVT_TS_EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	nvt_ts_spi_write(ts, buf, 2);

	//---enable fw crc---
	buf[0] = 0x50 & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	nvt_ts_spi_write(ts, buf, 2);
}

static void nvt_ts_set_bld_crc_bank(struct nvt_ts_data *ts, uint32_t DES_ADDR, uint32_t SRAM_ADDR,
		uint32_t LENGTH_ADDR, uint32_t size,
		uint32_t G_CHECKSUM_ADDR, uint32_t crc)
{
    uint8_t fwbuf[4] = {0};
	/* write destination address */
	nvt_ts_set_page(ts, DES_ADDR);
	fwbuf[0] = DES_ADDR & 0x7F;
	fwbuf[1] = (SRAM_ADDR) & 0xFF;
	fwbuf[2] = (SRAM_ADDR >> 8) & 0xFF;
	fwbuf[3] = (SRAM_ADDR >> 16) & 0xFF;
	nvt_ts_spi_write(ts, fwbuf, 4);

	/* write length */
	//nvt_set_page(LENGTH_ADDR);
	fwbuf[0] = LENGTH_ADDR & 0x7F;
	fwbuf[1] = (size) & 0xFF;
	fwbuf[2] = (size >> 8) & 0xFF;
	fwbuf[3] = (size >> 16) & 0xFF;
//	if (ts->hw_crc == HWCRC_LEN_2Bytes) {
		nvt_ts_spi_write(ts, fwbuf, 3);
//	} else if (ts->hw_crc >= HWCRC_LEN_3Bytes) {
//		nvt_ts_spi_write(ts, fwbuf, 4);
//	}

	/* write golden dlm checksum */
	//nvt_set_page(G_CHECKSUM_ADDR);
	fwbuf[0] = G_CHECKSUM_ADDR & 0x7F;
	fwbuf[1] = (crc) & 0xFF;
	fwbuf[2] = (crc >> 8) & 0xFF;
	fwbuf[3] = (crc >> 16) & 0xFF;
	fwbuf[4] = (crc >> 24) & 0xFF;
	nvt_ts_spi_write(ts, fwbuf, 5);

	return;
}

static void nvt_ts_fw_upload(const struct firmware *fw, void *ctx){
	struct nvt_ts_data *ts = ctx;
	int32_t ret = -1;
    char to_write=0;
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
    dev_err(&ts->dev, "fw upload: enter\n");
    if(!ts || !fw){
        dev_err(&ts->dev, "fw upload: no fw\n");
        return;
    }

	// Reboot to "eng" mode
    to_write=0x5a;
	nvt_ts_write_addr(ts, NVT_TS_RESET_REG, &to_write, 1);
    dev_err(&ts->dev, "fw upload: trigger eng reboot\n");
	// Reboot to bootloader
    to_write=0x69;
	nvt_ts_write_addr(ts, NVT_TS_SWRST_N8_ADDR, &to_write, 1);
    dev_err(&ts->dev, "fw upload: rebooted to bl\n");
	mdelay(5);	//wait tBRST2FR after Bootload RST

  //  nvt_ts_chip_id(ts);

    to_write=0x00;
	/* clear fw reset status */
	nvt_ts_write_addr(ts, NVT_TS_EVENT_BUF_ADDR | NVT_TS_EVENT_MAP_RESET_COMPLETE, &to_write, 1);
    dev_err(&ts->dev, "fw upload: cleared fw state\n");
	// Parse elf firmware and flash it
	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(ehdr + 1);


    /* [0] ILM */
	/* write register bank */
	nvt_ts_set_bld_crc_bank(ts, 0x1FB528, phdrs[0].p_paddr,
			0x1FB518, phdrs[0].p_filesz,
			0x1FB500, 0xEC1B47FE);

	/* [1] DLM */
	/* write register bank */
	nvt_ts_set_bld_crc_bank(ts, 0x1FB52C, phdrs[1].p_paddr,
			0x1FB530, phdrs[1].p_filesz,
			0x1FB504, 0x62FD8F84);

    to_write=0x56;
    nvt_ts_write_addr(ts, 0x1FC925, &to_write, 1);
	for (int i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];
		if(phdr->p_filesz==0)
			continue;
        dev_err(&ts->dev, "fw upload: going to write part: %d from: %llx to: %llx size: %d\n", i, phdr->p_offset, phdr->p_paddr, phdr->p_filesz);
		nvt_ts_write_sram(ts, fw->data, phdr->p_paddr, phdr->p_filesz, phdr->p_offset);
        dev_err(&ts->dev, "fw upload: part written to: 0x%llx\n", phdr->p_paddr);
	}

	nvt_ts_check_tx_auto_copy(ts);
    mdelay(1000);
    nvt_fw_crc_enable(ts);
	to_write=0x01;
	nvt_ts_write_addr(ts, tmp111, &to_write, 1);
    to_write=0x00;
	nvt_ts_write_addr(ts, tmp111, &to_write, 1);
    to_write=0xA0;
    nvt_ts_write_addr(ts, 0x1F61C, &to_write, 1);

	mdelay(1000);
   // nvt_check_fw_reset_state(ts);
    nvt_ts_get_fw_info(ts);
}
static irqreturn_t nvt_ts_irq_spi(int irq, void *dev_id)
{
    printk("NVT_TS irq");
    return IRQ_HANDLED;
}

static int nvt_ts_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int error, width, height, irq_type;
	struct nvt_ts_data *data;
	struct input_dev *input;
	const char *firmware_name;

	//if (!spi->irq) {
	//	dev_err(dev, "Error no irq specified\n");
	//	return -EINVAL;
	//}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
    dev_err(dev, "spi probe: enter\n");
	data->device = spi;
    spi_set_drvdata(spi, data);
	dev_set_drvdata(dev, data);

	data->dev = spi->dev;
	//data->irq = spi->irq;
    int irq_gpio = of_get_named_gpio(spi->dev.of_node, "novatek,irq-gpio", 0);
    dev_err(dev, "novatek,irq-gpio=%d\n", irq_gpio);
    error = gpio_request_one(irq_gpio, GPIOF_IN, "NVT-int");
    if(error){
		dev_err(dev, "failed to get irq gpio: %d\n", error);
		return error;
	}

    data->device->bits_per_word = 8;
	data->device->mode = SPI_MODE_0;

	error = spi_setup(data->device);

    // Allocate memory for SPI buffers
	data->xbuf = devm_kzalloc(&spi->dev, (NVT_TS_TRANSFER_LEN+1+DUMMY_BYTES), GFP_KERNEL); // Define XFER_BUFFER_SIZE as appropriate
	if (!data->xbuf)
		return -ENOMEM;

	data->rbuf = devm_kzalloc(&spi->dev, (NVT_TS_TRANSFER_LEN+1+DUMMY_BYTES), GFP_KERNEL);
	if (!data->rbuf)
		return -ENOMEM;
    dev_err(dev, "spi bufs allocated\n");
	if(error){
		dev_err(dev, "failed to setup spi: %d\n", error);
		return error;
	}

	dev_err(dev, "mode=%d, max_speed_hz=%d\n", data->device->mode, data->device->max_speed_hz);

	// Create regmap
//	data->regmap = devm_regmap_init_spi(spi, &nvt_ts_regmap_config);
//	if (IS_ERR(data->regmap)) {
//		error = PTR_ERR(data->regmap);
	//	dev_err(dev, "Failed to allocate register map: %d\n", error);
	////	return error;
//	}

	error = device_property_read_string(&spi->dev, "firmware-name", &firmware_name);
	if (error) {
		dev_err(dev, "Failed to read firmware-name property: %d\n", error);
    }

	/*
	 * VCC is the analog voltage supply
	 * IOVCC is the digital voltage supply
	 */
	data->regulators[0].supply = "vcc";
	data->regulators[1].supply = "iovcc";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(dev, "cannot get regulators: %d\n", error);
		return error;
	}

	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(dev, "failed to enable regulators\n");
		return error;
	}

	error = devm_add_action_or_reset(dev, nvt_ts_disable_regulators, data);
	if (error) {
		dev_err(dev, "failed to install regulator disable handler\n");
		return error;
	}

	data->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	error = PTR_ERR_OR_ZERO(data->reset_gpio);
	if (error) {
		dev_err(dev, "failed to request reset GPIO: %d, continuing without\n", error);
	}

	char to_write=0x5a;
	nvt_ts_write_addr(data, NVT_TS_RESET_REG, &to_write, 1);
	msleep(10);

	dev_err(dev, "spi probe: going to upload fw\n");
    error = request_firmware_nowait(THIS_MODULE, true, firmware_name,
						dev, GFP_KERNEL, data,
						nvt_ts_fw_upload);
	if (error < 0) {
		dev_err(dev, "unable to load %s\n", firmware_name);
		return error;
	}

    if (error) {
		dev_err(dev, "fw not found\n");
		return error;
	}
    dev_err(dev, "Goinf to read ts params\n");
	return 0;
    nvt_ts_read_addr(data, 0x3F004,
				 data->buf, NVT_TS_TOUCH_SIZE * NVT_TS_MAX_TOUCHES);
    dev_err(dev, "Read ts param done err: %d\n", error);
     print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 1, data->buf, NVT_TS_TOUCH_SIZE * NVT_TS_MAX_TOUCHES, true);
	if(data->reset_gpio!=NULL)
		gpiod_set_value_cansleep(data->reset_gpio, 1); /* Put back in reset */
	if (error){
        dev_err(dev, "failed to read ts params: %d\n", error);
		return error;
    }
	width  = get_unaligned_be16(&data->buf[NVT_TS_PARAMS_WIDTH]);
	height = get_unaligned_be16(&data->buf[NVT_TS_PARAMS_HEIGHT]);
	data->max_touches = data->buf[NVT_TS_PARAMS_MAX_TOUCH];
	irq_type = data->buf[NVT_TS_PARAMS_IRQ_TYPE];

	if (width > NVT_TS_MAX_SIZE || height >= NVT_TS_MAX_SIZE ||
	    data->max_touches > NVT_TS_MAX_TOUCHES ||
	    irq_type >= ARRAY_SIZE(nvt_ts_irq_type)) {
		dev_err(dev, "Unsupported touchscreen parameters: %*ph\n",
			NVT_TS_PARAMS_SIZE, data->buf);
		return -EIO;
	}

	dev_dbg(dev, "Detected %dx%d touchscreen with %d max touches\n",
		width, height, data->max_touches);

	if (data->buf[NVT_TS_PARAMS_MAX_BUTTONS])
		dev_warn(dev, "Touchscreen buttons are not supported\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "test";
	input->id.bustype = BUS_SPI;
	input->open = nvt_ts_start;
	input->close = nvt_ts_stop;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, width - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, height - 1, 0, 0);
	touchscreen_parse_properties(input, true, &data->prop);

	error = input_mt_init_slots(input, data->max_touches,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	data->input = input;
	input_set_drvdata(input, data);

	error = devm_request_threaded_irq(dev, spi->irq, NULL, nvt_ts_irq_spi,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN |
						nvt_ts_irq_type[irq_type],
					  "test", data);
	if (error) {
		dev_err(dev, "failed to request irq: %d\n", error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

static const struct of_device_id nvt_ts_of_match[] = {
	{ .compatible = "novatek,nvt-ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, nvt_ts_of_match);

static const struct i2c_device_id nvt_ts_i2c_id[] = {
	{ "NVT-ts" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nvt_ts_i2c_id);

static struct i2c_driver nvt_ts_i2c_driver = {
	.driver = {
		.name	= "novatek-nvt-ts-i2c",
		.pm	= pm_sleep_ptr(&nvt_ts_pm_ops),
		.of_match_table = nvt_ts_of_match,
	},
	.probe = nvt_ts_i2c_probe,
	.id_table = nvt_ts_i2c_id,
};

module_i2c_driver(nvt_ts_i2c_driver);

static struct spi_driver nvt_ts_spi_driver = {
	.driver = {
		.name	= "novatek-nvt-ts-spi",
		.pm	= pm_sleep_ptr(&nvt_ts_pm_ops),
		.of_match_table = nvt_ts_of_match,
	},
	.probe = nvt_ts_spi_probe,
};

module_spi_driver(nvt_ts_spi_driver);

MODULE_DESCRIPTION("Novatek NT11205 touchscreen driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
