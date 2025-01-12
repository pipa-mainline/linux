// SPDX-License-Identifier: GPL-2.0
/*
 *  Nanosic 803 keyboard controller
 *
 *  Copyright (C) 2024 Luka Panio <lukapanio@gmail.com>
 *
 *  Base on nano_driver driver by:
 *  Bin yuan <bin.yuan@nanosic.com>
 *  Copyright (C) 2010, Nanosic, Inc
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mutex.h>

#define I2C_DATA_LENGTH_READ (68)
#define I2C_DATA_LENGTH_WRITE (66)

#define TOUCH_TIMEOUT_MS 75

static const unsigned int hid_to_linux_keycode[] = {
	[0x04] = KEY_A,
	[0x05] = KEY_B,
	[0x06] = KEY_C,
	[0x07] = KEY_D,
	[0x08] = KEY_E,
	[0x09] = KEY_F,
	[0x0A] = KEY_G,
	[0x0B] = KEY_H,
	[0x0C] = KEY_I,
	[0x0D] = KEY_J,
	[0x0E] = KEY_K,
	[0x0F] = KEY_L,
	[0x10] = KEY_M,
	[0x11] = KEY_N,
	[0x12] = KEY_O,
	[0x13] = KEY_P,
	[0x14] = KEY_Q,
	[0x15] = KEY_R,
	[0x16] = KEY_S,
	[0x17] = KEY_T,
	[0x18] = KEY_U,
	[0x19] = KEY_V,
	[0x1A] = KEY_W,
	[0x1B] = KEY_X,
	[0x1C] = KEY_Y,
	[0x1D] = KEY_Z,
	[0x1E] = KEY_1,
	[0x1F] = KEY_2,
	[0x20] = KEY_3,
	[0x21] = KEY_4,
	[0x22] = KEY_5,
	[0x23] = KEY_6,
	[0x24] = KEY_7,
	[0x25] = KEY_8,
	[0x26] = KEY_9,
	[0x27] = KEY_0,
	[0x28] = KEY_ENTER,
	[0x29] = KEY_ESC,
	[0x2A] = KEY_BACKSPACE,
	[0x2B] = KEY_TAB,
	[0x2C] = KEY_SPACE,
	[0x2D] = KEY_MINUS,
	[0x2E] = KEY_EQUAL,
	[0x2F] = KEY_LEFTBRACE,
	[0x30] = KEY_RIGHTBRACE,
	[0x31] = KEY_BACKSLASH,
	[0x32] = KEY_GRAVE,
	[0x33] = KEY_SEMICOLON,
	[0x34] = KEY_APOSTROPHE,
	[0x35] = KEY_GRAVE,
	[0x36] = KEY_COMMA,
	[0x37] = KEY_DOT,
	[0x38] = KEY_SLASH,
	[0x39] = KEY_CAPSLOCK,
	[0x4f] = KEY_RIGHT,
	[0x50] = KEY_LEFT,
	[0x51] = KEY_DOWN,
	[0x52] = KEY_UP,
	[0xe0] = KEY_LEFTCTRL,
	[0xe1] = KEY_LEFTSHIFT,
	[0xe2] = KEY_LEFTALT,
	[0xe3] = KEY_LEFTMETA,
	[0xe4] = KEY_RIGHTCTRL,
	[0xe5] = KEY_RIGHTSHIFT,
	[0xe6] = KEY_RIGHTALT,
};

static const uint16_t hid_modifier_to_linux_keycode[8] = {
    KEY_LEFTCTRL,
    KEY_LEFTSHIFT,
    KEY_LEFTALT,
    KEY_LEFTMETA,
    KEY_RIGHTCTRL,
    KEY_RIGHTSHIFT,
    KEY_RIGHTALT,
    KEY_RIGHTMETA
};


static const struct regmap_config nanosic_803_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

struct nanosic_803_priv {
	struct device 		*dev;
	struct i2c_client	*client;
	struct input_dev	*keyboard_input_dev;
	struct input_dev	*touchpad_input_dev;
	struct regmap		*regmap;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*sleep_gpio;
	struct gpio_desc	*vdd_gpio;
	struct gpio_desc	*irq_gpio;
	unsigned int 		irq_number;
	struct mutex 		read_mutex;
	char			last_pressed_key[5];
	char			last_modifier_state;
	int 			slot_mapping[3];
	struct timer_list	finger_timer;
	bool 			finger_down;
	unsigned long		last_touch_time;
	int 			last_x, last_y;
	struct regulator	*vdd_1v8;
	struct regulator	*vdd_3v3;
};

void nanosic_803_wakeup(struct nanosic_803_priv *nanosic_dev)
{
	int level = 1;
	int retry = 3;
	level = gpiod_get_value(nanosic_dev->irq_gpio);

	if (level <= 0) {
		mdelay(25);
		while (retry--) {
			/*try three times*/
			if (gpiod_get_value(nanosic_dev->irq_gpio)) {
				break;
			}
			/*reset wn8030*/
			if (retry == 0) {
				gpiod_set_value(nanosic_dev->reset_gpio, 0);
				mdelay(100);
				gpiod_set_value(nanosic_dev->reset_gpio, 1);

				// delay 500ms for iic ready
				mdelay(500);
			}
			/*irq low level duration is 1ms*/
			mdelay(1);
		}
	}
}

int nanosic_803_read_version(struct nanosic_803_priv *nanosic_dev)
{
	char rsp[I2C_DATA_LENGTH_READ] = { 0 };
	char cmd[I2C_DATA_LENGTH_WRITE] = { 0x32, 0x00, 0x4F, 0x30, 0x80,
						0x18, 0x01, 0x00, 0x18 };
	char hex_dump[3 * I2C_DATA_LENGTH_READ + 1] = { 0 };
	u8 retry = 0;
	int ret = -1;

	while (retry++ < 30) {
		ret = regmap_raw_write(nanosic_dev->regmap, 0, cmd, sizeof(cmd));
		if (ret < 0) {
			dev_err(nanosic_dev->dev, "regmap write cmd failed time %d\n", retry);
			msleep(100);
			continue;
		}
		msleep(2);

		ret = regmap_raw_read(nanosic_dev->regmap, 0, rsp, sizeof(rsp));
		if (ret == 0) {
			for (int i = 0; i < sizeof(rsp); i++) {
				snprintf(&hex_dump[i * 3], 4, "%02x ", rsp[i]);
			}
			dev_err(nanosic_dev->dev, "nanosic chip version: %s\n", hex_dump);
			dev_err(nanosic_dev->dev, "Version read OK\n");
			break;
		}
		msleep(2);
	}

	return ret;
}

#define INT_ADDR_MAX_BYTES 4

int nanosic_i2c_read(struct nanosic_803_priv *nanosic_dev, void *buf, size_t len)
{
	struct i2c_adapter *adap;
	unsigned char addr[INT_ADDR_MAX_BYTES];
	struct i2c_msg msg[2];
	int ret;

	addr[0] = nanosic_dev->client->addr;

	adap = i2c_get_adapter(nanosic_dev->client->adapter->nr);

	msg[0].len = 1;
	msg[0].addr = nanosic_dev->client->addr;
	msg[0].flags = 0;
	msg[0].buf = addr;

	msg[1].addr = nanosic_dev->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	ret = i2c_transfer(adap, msg, 2);
	if (ret < 0) {
		printk("i2c_transfer read error\n");
		len = -1;
	}

	//mutex_unlock(&i2c_client->read_mutex);

	return len;
}

int nanosic_i2c_write(struct nanosic_803_priv *nanosic_dev, void *buf, size_t len)
{
	struct i2c_msg msg;
	struct i2c_adapter *adap;
	unsigned char tmp_buf[128] = { 0 };
	int ret;

	adap = i2c_get_adapter(nanosic_dev->client->adapter->nr);

	tmp_buf[0] = nanosic_dev->client->addr;
	memcpy(tmp_buf + 1, buf, len);

	msg.addr = nanosic_dev->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = tmp_buf;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret < 0) {
		printk("i2c_transfer write error\n");
		len = -1;
	}

	return len;
}

void handle_modifiers(struct nanosic_803_priv *nanosic_dev, char modifiers) {
	char last_modifiers = nanosic_dev->last_modifier_state;

	for (int i = 0; i < 8; i++) {
		uint8_t current_bit = (modifiers >> i) & 1;
		uint8_t last_bit = (last_modifiers >> i) & 1;

		if (current_bit && !last_bit) {
			input_report_key(nanosic_dev->keyboard_input_dev, hid_modifier_to_linux_keycode[i], 1);
			printk("Modifier pressed: %d\n", hid_modifier_to_linux_keycode[i]);
		} else if (!current_bit && last_bit) {
			input_report_key(nanosic_dev->keyboard_input_dev, hid_modifier_to_linux_keycode[i], 0);
			printk("Modifier released: %d\n", hid_modifier_to_linux_keycode[i]);
		}
	}

	input_sync(nanosic_dev->keyboard_input_dev);

	nanosic_dev->last_modifier_state = modifiers;
}

void nanosic_handle_keyboard(struct nanosic_803_priv *nanosic_dev, char *buf) {
	int i, j;
	int found;

	handle_modifiers(nanosic_dev, buf[4]);

	for (i = 0; i < 5 && buf[6+i] != 0x00; ++i) {
		found = 0;
		for (j = 0; j < 5 && nanosic_dev->last_pressed_key[j] != 0x00; ++j) {
			if (buf[6+i] == nanosic_dev->last_pressed_key[j]) {
				found = 1;
				break;
			}
		}
		if (!found) {
			input_report_key(nanosic_dev->keyboard_input_dev, hid_to_linux_keycode[buf[6+i]], 1);
			input_sync(nanosic_dev->keyboard_input_dev);
			printk("Key pressed: 0x%02X\n", buf[6+i]);
		}
	}

	for (i = 0; i < 5 && nanosic_dev->last_pressed_key[i] != 0x00; ++i) {
		found = 0;
		for (j = 0; j < 5 && buf[6+j] != 0x00; ++j) {
			if (nanosic_dev->last_pressed_key[i] == buf[6+j]) {
				found = 1;
				break;
			}
		}
		if (!found) {
			printk("Key released: 0x%02X\n", nanosic_dev->last_pressed_key[i]);
			input_report_key(nanosic_dev->keyboard_input_dev, hid_to_linux_keycode[nanosic_dev->last_pressed_key[i]], 0);
			input_sync(nanosic_dev->keyboard_input_dev);
		}
	}
	memcpy(nanosic_dev->last_pressed_key, &buf[6], sizeof(nanosic_dev->last_pressed_key));
}

void nanosic_touch_timer_callback(struct timer_list *t) {
	struct nanosic_803_priv *nanosic_dev = from_timer(nanosic_dev, t, finger_timer);

	if (nanosic_dev->finger_down) {
		for(int i = 0; i<3; i++) {
			input_mt_slot(nanosic_dev->touchpad_input_dev, nanosic_dev->slot_mapping[i]);
			input_mt_report_slot_state(nanosic_dev->touchpad_input_dev, MT_TOOL_FINGER, 0);
			input_report_abs(nanosic_dev->touchpad_input_dev, ABS_MT_TRACKING_ID, -1);
			input_sync(nanosic_dev->touchpad_input_dev);
		}
        nanosic_dev->finger_down = false;
    }
}

void nanosic_handle_touchpad_mt(struct nanosic_803_priv *nanosic_dev, char *buf) {
	int finger_id, x, y;

	for (int i = 0; i < 3; i++) {
		int offset = 6 + (i * 6);
		finger_id = buf[offset];
		x = (uint16_t)(buf[offset + 1] | (buf[offset + 2] << 8));
		y = (uint16_t)(buf[offset + 3] | (buf[offset + 4] << 8));

		if (finger_id == 0 && x == 0 && y == 0) {
			if (nanosic_dev->slot_mapping[i] != -1) {
				input_mt_slot(nanosic_dev->touchpad_input_dev, nanosic_dev->slot_mapping[i]);
				input_mt_report_slot_state(nanosic_dev->touchpad_input_dev, MT_TOOL_FINGER, 0);
				nanosic_dev->slot_mapping[i] = -1;
			}
			continue;
		}

		int slot = nanosic_dev->slot_mapping[i];
		if (slot == -1) {
			for (int j = 0; j < 3; j++) {
				if (nanosic_dev->slot_mapping[j] == -1) {
					slot = j;
					nanosic_dev->slot_mapping[i] = slot;
					break;
				}
			}
			if (slot == -1) {
				dev_err(nanosic_dev->dev, "No free slots available for finger %d\n", finger_id);
				continue;
			}
		}

		input_mt_slot(nanosic_dev->touchpad_input_dev, slot);
		input_mt_report_slot_state(nanosic_dev->touchpad_input_dev, MT_TOOL_FINGER, 1);
		input_report_abs(nanosic_dev->touchpad_input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(nanosic_dev->touchpad_input_dev, ABS_MT_POSITION_Y, y);
	}

	input_mt_sync_frame(nanosic_dev->touchpad_input_dev);
	input_sync(nanosic_dev->touchpad_input_dev);

	nanosic_dev->finger_down = true;
	nanosic_dev->last_touch_time = jiffies;
	mod_timer(&nanosic_dev->finger_timer, jiffies + msecs_to_jiffies(TOUCH_TIMEOUT_MS));
}

static irqreturn_t nanosic_irq_handler(int irq, void *dev_id) {
	struct nanosic_803_priv *nanosic_dev = dev_id;
	int ret;
	char buf[I2C_DATA_LENGTH_READ] = { 0 };
	char hex_dump[3 * I2C_DATA_LENGTH_READ + 1] = { 0 };

	mutex_lock(&nanosic_dev->read_mutex);
	ret = nanosic_i2c_read(nanosic_dev, buf, sizeof(buf));
	mutex_unlock(&nanosic_dev->read_mutex);
	if(ret == 0) {
		dev_err(nanosic_dev->dev, "Failed to read data on interrupt: %d\n", ret);
		return IRQ_HANDLED;
	}
	for (int i = 0; i < ret; i++) {
		snprintf(&hex_dump[i * 3], 4, "%02x ", buf[i]);
	}
	dev_err(nanosic_dev->dev, "nanosic message: %s\n", hex_dump);

	if(buf[0] != 0x57 || buf[2] == 0) {
		dev_err(nanosic_dev->dev, "Malformed message\n");
		// Our hardware might randomly return no meesage/random data, so assume we hanfled irq correctly
		return IRQ_HANDLED;
	}

	// Keyboard event
	if(buf[3] == 0x05) {
		nanosic_handle_keyboard(nanosic_dev, buf);
	}
	// Touchpad event
	else if(buf[3] == 0x19) {
		nanosic_handle_touchpad_mt(nanosic_dev, buf);
	}
	return IRQ_HANDLED;
}

int nanosic_set_caps_led(struct nanosic_803_priv *nanosic_dev, bool enable)
{
	int i = 0, ret = 0;
	dev_err(nanosic_dev->dev, "Going to set caps led: %d\n", enable);
	char cmd[I2C_DATA_LENGTH_WRITE] = { 0x32, 0x00, 0x4E, 0x31,
					    0x80, 0x38, 0x26, 0x01, enable};

	for (i = 2; i < 9; i++) {
		cmd[9] += cmd[i];
	} /*cal sum*/
	ret = nanosic_i2c_write(nanosic_dev, cmd, sizeof(cmd));

	return ret;
}

static int nanosic_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	struct nanosic_803_priv *nanosic_dev = input_get_drvdata(dev);
	dev_err(nanosic_dev->dev, "nanosic_event type: %ld, code: %ld, value: %ld\n", type, code, value);
	if (!nanosic_dev)
		return -EINVAL;

	if (type == EV_LED) {
		switch (code) {
			case LED_CAPSL:
				nanosic_set_caps_led(nanosic_dev ,value);
				break;
			default:
				break;
		}
	}
	return 0;
}

void nanosic_803_reset(struct nanosic_803_priv *nanosic_dev)
{
	gpiod_set_value(nanosic_dev->reset_gpio, 0);
	gpiod_set_value(nanosic_dev->sleep_gpio, 0);
	gpiod_set_value(nanosic_dev->vdd_gpio, 0);
	msleep(100);
	gpiod_set_value(nanosic_dev->vdd_gpio, 1);
	msleep(2);
	gpiod_set_value(nanosic_dev->reset_gpio, 1);
	gpiod_set_value(nanosic_dev->sleep_gpio, 1);
}

static int nanosic_803_probe(struct i2c_client *client)
{
	struct nanosic_803_priv *nanosic_dev;
	struct input_dev *keyboard_input_dev;
	struct input_dev *touchpad_dev;
	struct device *dev = &client->dev;
	struct regmap *map;
	unsigned int ret, i;
	unsigned int touchpad_resolution_x, touchpad_resolution_y;

	nanosic_dev = devm_kzalloc(dev, sizeof(*nanosic_dev), GFP_KERNEL);
	if (!nanosic_dev)
		return -ENOMEM;

	nanosic_dev->dev = dev;
	dev_set_drvdata(dev, nanosic_dev);

	// Get GPIOs
	nanosic_dev->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(nanosic_dev->reset_gpio)) {
		dev_err(dev, "Failed to get reset GPIO\n");
		return PTR_ERR(nanosic_dev->reset_gpio);
	}

	nanosic_dev->sleep_gpio = devm_gpiod_get(dev, "sleep", GPIOD_OUT_HIGH);
	if (IS_ERR(nanosic_dev->sleep_gpio)) {
		dev_err(dev, "Failed to get sleep GPIO\n");
		return PTR_ERR(nanosic_dev->sleep_gpio);
	}

	nanosic_dev->vdd_gpio = devm_gpiod_get(dev, "vdd", GPIOD_OUT_HIGH);
	if (IS_ERR(nanosic_dev->vdd_gpio)) {
		dev_err(dev, "Failed to get vdd GPIO\n");
		return PTR_ERR(nanosic_dev->vdd_gpio);
	}

	nanosic_dev->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(nanosic_dev->irq_gpio)) {
		dev_err(dev, "Failed to get irq GPIO\n");
		return PTR_ERR(nanosic_dev->vdd_gpio);
	}

	// Get regulators
	nanosic_dev->vdd_1v8 = devm_regulator_get(dev, "vdd_1v8");
	if (IS_ERR(nanosic_dev->vdd_1v8)) {
		dev_err(dev, "Failed to get 1.8V regulator\n");
		return PTR_ERR(nanosic_dev->vdd_1v8);
	}

	nanosic_dev->vdd_3v3 = devm_regulator_get(dev, "vdd_3v3");
	if (IS_ERR(nanosic_dev->vdd_3v3)) {
		dev_err(dev, "Failed to get 3.3V regulator\n");
		return PTR_ERR(nanosic_dev->vdd_3v3);
	}

	// Get touchpad resolution
	if (of_property_read_u32(dev->of_node, "touchpad-resolution-x", &touchpad_resolution_x)) {
		dev_err(dev, "Failed to read touchpad-resolution-x from DT\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "touchpad-resolution-y", &touchpad_resolution_y)) {
		dev_err(dev, "Failed to read touchpad-resolution-y from DT\n");
		return -EINVAL;
	}

	// Enable regulators
	ret = regulator_enable(nanosic_dev->vdd_1v8);
	if (ret) {
		dev_err(dev, "Failed to enable 1.8V regulator\n");
		return ret;
	}

	ret = regulator_enable(nanosic_dev->vdd_3v3);
	if (ret) {
		dev_err(dev, "Failed to enable 3.3V regulator\n");
		return ret;
	}

	// Reset the chip
	nanosic_803_reset(nanosic_dev);

	// Wake up the chip
	nanosic_803_wakeup(nanosic_dev);

	// Setup keyboard input
	keyboard_input_dev = devm_input_allocate_device(dev);
	if (!keyboard_input_dev)
		return -ENOMEM;

	keyboard_input_dev->name = "Nanosic 803 keyboard";
	keyboard_input_dev->phys = "input/keyboard";
	keyboard_input_dev->id.bustype = BUS_I2C;
	keyboard_input_dev->id.vendor = 0x1234;
	keyboard_input_dev->id.product = 0x5678;
	keyboard_input_dev->id.version = 0x0100;

	set_bit(EV_KEY, keyboard_input_dev->evbit);
	set_bit(EV_REP, keyboard_input_dev->evbit);

	keyboard_input_dev->evbit[0] |= BIT_MASK(EV_LED) |  BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
	keyboard_input_dev->ledbit[0] = BIT_MASK(LED_CAPSL);
	keyboard_input_dev->event = nanosic_event;

	input_set_drvdata(keyboard_input_dev, nanosic_dev);

	for (int i = 0; i < KEY_MAX; i++) {
		set_bit(i, keyboard_input_dev->keybit);
	}

	ret = input_register_device(keyboard_input_dev);
	if (ret) {
		pr_err("Failed to register keyboard_input_dev device: %d\n", ret);
		return ret;
	}

	// Set up touhpad input device
	touchpad_dev = devm_input_allocate_device(dev);
	if (!touchpad_dev) {
		pr_err("Failed to allocate touchpad input device\n");
		return -ENOMEM;
	}

	touchpad_dev->name = "Nanosic 803 touchpad";
	touchpad_dev->phys = "input/touchpad";
	touchpad_dev->id.bustype = BUS_I2C;
	touchpad_dev->id.vendor = 0x1234;
	touchpad_dev->id.product = 0x5678;
	touchpad_dev->id.version = 0x0001;

	ret = input_mt_init_slots(touchpad_dev, 3, INPUT_MT_POINTER);
	if (ret) {
		dev_err(dev, "Failed to initialize MT slots: %d\n", ret);
		return ret;
	}
	set_bit(INPUT_PROP_POINTER, touchpad_dev->propbit);
	set_bit(EV_ABS, touchpad_dev->evbit);
	input_set_abs_params(touchpad_dev, ABS_MT_POSITION_X, 0, touchpad_resolution_x, 0, 0);
	input_set_abs_params(touchpad_dev, ABS_MT_POSITION_Y, 0, touchpad_resolution_y, 0, 0);
	input_set_abs_params(touchpad_dev, ABS_MT_TRACKING_ID, 0, 3 - 1, 0, 0);

	set_bit(EV_KEY, touchpad_dev->evbit);
	set_bit(BTN_LEFT, touchpad_dev->keybit);
	set_bit(BTN_RIGHT, touchpad_dev->keybit);

	input_set_drvdata(touchpad_dev, nanosic_dev);

	ret = input_register_device(touchpad_dev);
	if (ret) {
		pr_err("Failed to register touchpad input device\n");
		input_free_device(touchpad_dev);
		return ret;
	}

	// Set up regmap
	map = devm_regmap_init_i2c(client, &nanosic_803_regmap_config);
	if (IS_ERR(map))
		return PTR_ERR(map);

	nanosic_dev->client = client;
	nanosic_dev->keyboard_input_dev = keyboard_input_dev;
	nanosic_dev->touchpad_input_dev = touchpad_dev;
	nanosic_dev->regmap = map;


	mutex_init(&nanosic_dev->read_mutex);

	i2c_set_clientdata(client, nanosic_dev);

	// Try to identify chip
	if(nanosic_803_read_version(nanosic_dev)) {
		dev_err(dev, "Nanosic 803 not found\n");
		return -ENODEV;
	}

	// Set up irq
	nanosic_dev->irq_number = gpiod_to_irq(nanosic_dev->irq_gpio);
	if (nanosic_dev->irq_number < 0) {
		dev_err(nanosic_dev->dev, "Failed to get IRQ for GPIO %d\n", nanosic_dev->irq_gpio);
		return nanosic_dev->irq_number;
	}

	ret = request_irq(nanosic_dev->irq_number, nanosic_irq_handler,
				0x6001, "nanosic_irq", nanosic_dev);

	if (ret) {
		dev_err(nanosic_dev->dev, "Failed to request IRQ %d: %d\n", nanosic_dev->irq_number, ret);
		return ret;
	}

	// Set up touchpad in-activity tracking
	for (int i = 0; i < 3; i++) {
		nanosic_dev->slot_mapping[i] = -1;
	}

	timer_setup(&nanosic_dev->finger_timer, nanosic_touch_timer_callback, 0);
	nanosic_dev->finger_down = false;
	return 0;
}

static int nanosic_803_suspend(struct device *dev)
{
	struct nanosic_803_priv *nanosic_dev = dev_get_drvdata(dev);
	int ret;

	// Actually de-init device
	gpiod_set_value(nanosic_dev->reset_gpio, 0);
	gpiod_set_value(nanosic_dev->sleep_gpio, 0);
	gpiod_set_value(nanosic_dev->vdd_gpio, 0);

	// Turn the regulators off
	ret = regulator_disable(nanosic_dev->vdd_1v8);
	if (ret) {
		dev_err(dev, "Failed to disable 1.8V regulator\n");
		return ret;
	}

	ret = regulator_disable(nanosic_dev->vdd_3v3);
	if (ret) {
		dev_err(dev, "Failed to disable 3.3V regulator\n");
		return ret;
	}
	return 0;
}

static int nanosic_803_resume(struct device *dev)
{
	struct nanosic_803_priv *nanosic_dev = dev_get_drvdata(dev);
	int ret;

	// Turn the regulators on
	ret = regulator_enable(nanosic_dev->vdd_1v8);
	if (ret) {
		dev_err(dev, "Failed to disable 1.8V regulator\n");
		return ret;
	}

	ret = regulator_enable(nanosic_dev->vdd_3v3);
	if (ret) {
		dev_err(dev, "Failed to disable 3.3V regulator\n");
		return ret;
	}

	// Reset the chip
	nanosic_803_reset(nanosic_dev);

	// Wake up the chip
	nanosic_803_wakeup(nanosic_dev);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nanosic_803_pm_ops, nanosic_803_suspend, nanosic_803_resume);

static const struct of_device_id __maybe_unused nanosic_803_of_match[] = {
	{ .compatible = "nanosic,803", },
	{ },
};
MODULE_DEVICE_TABLE(of, nanosic_803_of_match);

static struct i2c_driver nanosic_803_driver = {
	.driver	= {
		.name = "nanosic_803",
		.of_match_table = of_match_ptr(nanosic_803_of_match),
		.pm = pm_sleep_ptr(&nanosic_803_pm_ops),
	},
	.probe = nanosic_803_probe,
};

module_i2c_driver(nanosic_803_driver);

MODULE_AUTHOR("Luka Panio <lukapanio@gmail.com>");
MODULE_DESCRIPTION("Driver for nanosic 803 keyboard controller");
MODULE_LICENSE("GPL v2");
