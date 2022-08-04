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

#include <asm/unaligned.h>

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

#define PINCTRL_STATE_ACTIVE "pmx_ts_active"

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;
#define DUMMY_BYTES (1)
#define SPI_WRITE_MASK(a) (a | 0x80)
#define SPI_READ_MASK(a) (a & 0x7F)
#define NVT_TRANSFER_LEN	(63*1024)
#define NVT_READ_LEN		(2*1024)

int32_t spi_read_write(struct nvt_ts_data *ts, uint8_t *buf, size_t len, NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,  // this will be overwritten for reads
	};
printk("spi read write called\n");
	if (!ts || !buf) {
		printk("Invalid ts or buf pointer\n");
		return -EINVAL;
	}

	memset(ts->xbuf, 0, len + DUMMY_BYTES); // Assuming DUMMY_BYTES is defined
	memcpy(ts->xbuf, buf, len);
printk("spi read write memcpy done\n");
	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = len + DUMMY_BYTES;  // include dummy bytes for read operations
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			t.rx_buf = NULL;  // not receiving any data
			t.len    = len;   // no dummy bytes for write
			break;

		default:
			printk("Invalid SPI operation\n");
			return -EINVAL;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(ts->device, &m);  // using the SPI device from your structure
}

int32_t CTP_SPI_READ(struct nvt_ts_data *ts, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	if (!ts || !buf) {
		printk("Invalid ts or buf pointer\n");
		return -EINVAL;
	}

	//mutex_lock(&ts->xbuf_lock);

	// Assuming the buf[0] has been prepared by the caller function
	while (retries < 5) {
		ret = spi_read_write(ts, buf, len, NVTREAD); // Here, we use ts->client
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		printk("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		// Assuming the actual read data starts from the third byte in rbuf
		memcpy(buf, ts->rbuf + 2, len); // Adjust based on your protocol
	}

	//mutex_unlock(&ts->xbuf_lock);

	return ret;
}

int32_t CTP_SPI_WRITE(struct nvt_ts_data *ts, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;
printk("enter ctp\n");
	if (!ts || !buf) {
		printk("Invalid ts or buf pointer\n");
		return -EINVAL;
	}

	//mutex_lock(&ts->xbuf_lock);
printk("ctp mux locked\n");
	// Assuming the buf[0] has been prepared by the caller function
	while (retries < 5) {
        printk("ctp calling wite\n");
		ret = spi_read_write(ts, buf, len, NVTWRITE); // Here, we use ts->client
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		printk("write error, ret=%d\n", ret);
		ret = -EIO;
	}

	//mutex_unlock(&ts->xbuf_lock);

	return ret;
}


int32_t nvt_read_addr(struct nvt_ts_data *ts, uint32_t addr, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;

	if (!ts || !buf) {
		// handle error, invalid argument
		return -EINVAL;
	}

	mutex_lock(&ts->xbuf_lock);

	// Prepare command buffer
	ts->xbuf[0] = SPI_READ_MASK(addr & 0x7F); // assuming SPI_READ_MASK is defined somewhere

	ret = CTP_SPI_READ(ts, ts->xbuf, len);
	if (ret < 0) {
		printk("read from addr 0x%06X failed, ret = %d\n", addr, ret);
	} else {
		memcpy(buf, ts->rbuf + 2, len); // copy the data to the provided buffer
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

int32_t nvt_write_addr(struct nvt_ts_data *ts, uint32_t addr, uint8_t *data, uint16_t len)
{
	int32_t ret = -1;
    printk("nvt write called");
	if (!ts || !data) {
		// handle error, invalid argument
		return -EINVAL;
	}
printk("nvt write lock mutex");
	mutex_lock(&ts->xbuf_lock);
printk("nvt write locked mutex");
	// Prepare command buffer
	ts->xbuf[0] = SPI_WRITE_MASK(addr & 0x7F); // assuming SPI_WRITE_MASK is defined somewhere
    printk("nvt write got mask");
	memcpy(ts->xbuf + 1, data, len); // copy data to be written
printk("data copied");
	ret = CTP_SPI_WRITE(ts, ts->xbuf, len + 1); // write data
	if (ret < 0) {
		printk("write to addr 0x%06X failed, ret = %d\n", addr, ret);
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

static int nvt_ts_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int error, width, height, irq_type;
	struct nvt_ts_data *data;
	struct input_dev *input;

	if (!spi->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->device = spi;
    spi_set_drvdata(spi, data);
	dev_set_drvdata(dev, data);

	data->dev = spi->dev;
	data->irq = spi->irq;

    data->device->bits_per_word = 8;
	data->device->mode = SPI_MODE_0;

	error = spi_setup(data->device);

    // Allocate memory for SPI buffers
	data->xbuf = devm_kzalloc(&spi->dev, (NVT_TRANSFER_LEN+1+DUMMY_BYTES), GFP_KERNEL); // Define XFER_BUFFER_SIZE as appropriate
	if (!data->xbuf)
		return -ENOMEM;

	data->rbuf = devm_kzalloc(&spi->dev, (NVT_TRANSFER_LEN+1+DUMMY_BYTES), GFP_KERNEL);
	if (!data->rbuf)
		return -ENOMEM;

	if(error){
		dev_err(dev, "failed to setup spi: %d\n", error);
		return error;
	}

	// Create regmap
	data->regmap = devm_regmap_init_spi(spi, &nvt_ts_regmap_config);
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
		dev_err(dev, "failed to request reset GPIO: %d, continuing without\n", error);
	}

	uint8_t data_to_write = 0x5A; // The data you want to write
nvt_write_addr(data, 0x7FFF80, &data_to_write, 1); // 'ts' is your struct nvt_ts_data *ts

	/* Wait for controller to come out of reset before params read */
	msleep(100);
    dev_err(dev, "Goinf to read ts params\n");
	nvt_read_addr(data, 0x3F004,
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

	error = devm_request_threaded_irq(dev, spi->irq, NULL, nvt_ts_irq,
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
