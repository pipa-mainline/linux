// SPDX-License-Identifier: GPL-2.0
/*
 *  Nanosic 803 keyboard controller driver
 *
 *  Copyright (C) 2024 Luka Panio <lukapanio@gmail.com>
 *
 *  Based on nano_driver by:
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
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>

#define I2C_DATA_LENGTH_READ (68)
#define I2C_DATA_LENGTH_WRITE (66)

#define TOUCH_TIMEOUT_MS 75

static char *command_name[] = {
	[0x01] = "Get version ()",
	[0x23] = "Set backlight (0-100)",
	[0x25] = "Set power state (off/on)",
	[0x26] = "Set caps LED (off/on)",
	[0x30] = "Get value (value_id)",
	[0x31] = "MIAUTH (???)",
	[0x36] = "NFC? (???)",
	[0x52] = "Request G-Sensor data ()",
	[0xA1] = "Request hall info ()",

	// will be shown for every other command
	[0xFF] = "Unknown"
};

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
	[0x6f] = KEY_BRIGHTNESSUP,
	[0x70] = KEY_BRIGHTNESSDOWN,
	[0xb5] = KEY_NEXTSONG,
	[0xb6] = KEY_PREVIOUSSONG,
	[0xcd] = KEY_PLAYPAUSE,
	[0xe2] = KEY_MUTE,
	[0xe9] = KEY_VOLUMEUP,
	[0xea] = KEY_VOLUMEDOWN,
};

static const unsigned int second_layer_to_linux_keycode[] = {
	[0x04] = KEY_A,
	[0x05] = KEY_KP1,
	[0x06] = KEY_C,
	[0x07] = KEY_D,
	[0x08] = KEY_E,
	[0x09] = KEY_F,
	[0x0A] = KEY_KP4,
	[0x0B] = KEY_KP5,
	[0x0C] = KEY_INSERT,
	[0x0D] = KEY_KP6,
	[0x0E] = KEY_KPASTERISK,
	[0x0F] = KEY_KPPLUS,
	[0x10] = KEY_KP3,
	[0x11] = KEY_KP2,
	[0x12] = KEY_KP0,
	[0x13] = KEY_SYSRQ,
	[0x14] = KEY_Q,
	[0x15] = KEY_R,
	[0x16] = KEY_S,
	[0x17] = KEY_KP7,
	[0x18] = KEY_KP9,
	[0x19] = KEY_V,
	[0x1A] = KEY_W,
	[0x1B] = KEY_X,
	[0x1C] = KEY_KP8,
	[0x1D] = KEY_Z,
	[0x1E] = KEY_F1,
	[0x1F] = KEY_F2,
	[0x20] = KEY_F3,
	[0x21] = KEY_F4,
	[0x22] = KEY_F5,
	[0x23] = KEY_F6,
	[0x24] = KEY_F7,
	[0x25] = KEY_F8,
	[0x26] = KEY_F9,
	[0x27] = KEY_F10,
	[0x28] = KEY_KPENTER,
	[0x29] = KEY_GRAVE,
	[0x2A] = KEY_DELETE,
	[0x2B] = KEY_NUMLOCK,
	[0x2C] = KEY_SPACE,
	[0x2D] = KEY_F11,
	[0x2E] = KEY_F12,
	[0x2F] = KEY_ROTATE_DISPLAY,
	[0x30] = KEY_BREAK,
	[0x31] = KEY_BACKSLASH,
	[0x32] = KEY_GRAVE,
	[0x33] = KEY_KPMINUS,
	[0x34] = KEY_COMPOSE,
	[0x35] = KEY_GRAVE,
	[0x36] = KEY_KPCOMMA,
	[0x37] = KEY_KPDOT,
	[0x38] = KEY_KPSLASH,
	[0x39] = KEY_CAPSLOCK,
	[0x4f] = KEY_END,
	[0x50] = KEY_HOME,
	[0x51] = KEY_PAGEDOWN,
	[0x52] = KEY_PAGEUP,
	[0x6f] = KEY_BRIGHTNESSUP,
	[0x70] = KEY_BRIGHTNESSDOWN,
	[0xb5] = KEY_NEXTSONG,
	[0xb6] = KEY_PREVIOUSSONG,
	[0xcd] = KEY_PLAYPAUSE,
	[0xe2] = KEY_MUTE,
	[0xe9] = KEY_VOLUMEUP,
	[0xea] = KEY_VOLUMEDOWN,
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
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *keyboard_input_dev;
	struct input_dev *touchpad_input_dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *sleep_gpio;
	struct gpio_desc *vdd_gpio;
	struct gpio_desc *irq_gpio;
	struct regulator *vdd_1v8;
	struct regulator *vdd_3v3;
	struct workqueue_struct	*wq;
	struct work_struct led_work;
	struct timer_list finger_timer;
	struct mutex i2c_mutex;
	unsigned int irq_number;
	char last_pressed_key[5];
	char last_modifier_state;
	char last_fn_key;
	int slot_mapping[3];
	bool finger_down;
	bool caps_led_on;
	bool caps_as_second_layer_key;
	bool second_layer_active;
	unsigned long last_touch_time;
	int last_x, last_y;
	struct dentry *debugfs_root;
};

struct nanosic_message_header {
	u8 header; // 0x32 for outgoing messages, ignored by checksum
	u8 counter; // always 0 for outgoing messages
	u8 unknown0; // 0x4E or 0x4F
	u8 message_type; // 30 or 31
	u8 src; // sender
	u8 dst; // recipient
	u8 cmd_id; // command id
	u8 data_length; // data length in bytes
};

static void nanosic_print_cmd(struct nanosic_803_priv *nanosic_dev, char *prefix, char *buf, int len)
{
	print_hex_dump_debug(prefix, DUMP_PREFIX_NONE, 32, 1, buf, len, true);
}

/*
 * Nanosic command format:
 *  byte 0: Header (0x32 for outgoing messages, ignored by checksum)
 *  byte 1: Counter (always 0 for outgoing messages)
 *  byte 2: Unknown (0x4E or 0x4F)
 *  byte 3: Message Type
 *  byte 4: Source Address
 *  byte 5: Destination Address
 *  byte 6: Command ID
 *  byte 7: Data Length (N)
 *  byte 8..8+N-1: Data payload
 *  byte 8+N: Checksum
 */
static int nanosic_send_command(struct nanosic_803_priv *nanosic_dev, char *cmd, size_t cmd_size)
{
	char cmd_buf[I2C_DATA_LENGTH_WRITE];
	char *cmd_name;
	int checksum = 0;
	int i = 0;
	int length_total;

	struct nanosic_message_header *message_header = (struct nanosic_message_header *)cmd;

	if (cmd_size < 8) {
		dev_err(nanosic_dev->dev, "Command is too short: %zu < 8", cmd_size);
		return -EINVAL;
	}

	length_total = 8 + message_header->data_length + 1;

	cmd_name = command_name[message_header->cmd_id];
	if (!cmd_name)
		cmd_name = command_name[0xFF];

	dev_dbg(nanosic_dev->dev, "Sending %s command_id=%X %X->%X data_length=%d\n",
		 cmd_name,
		 message_header->cmd_id,
		 message_header->src,
		 message_header->dst,
		 message_header->data_length);

	if (cmd_size > I2C_DATA_LENGTH_WRITE) {
		dev_err(nanosic_dev->dev, "Command is too large: %zu bytes (max is %d)\n",
			cmd_size, I2C_DATA_LENGTH_WRITE);
		return -EINVAL;
	}

	if (length_total > I2C_DATA_LENGTH_WRITE) {
		dev_err(nanosic_dev->dev, "Calculated total length (%d bytes) exceeds buffer size (%d bytes)\n",
			length_total, I2C_DATA_LENGTH_WRITE);
		return -EINVAL;
	}

	memcpy(cmd_buf, cmd, cmd_size);

	for (i = 2; i < length_total - 1; i++)
		checksum += cmd[i];

	cmd_buf[length_total - 1] = checksum;

	nanosic_print_cmd(nanosic_dev, "nanosic: sending command: ", cmd_buf, length_total);

	return regmap_raw_write(nanosic_dev->regmap, 0, cmd_buf, sizeof(cmd_buf));
}

static void nanosic_803_wakeup(struct nanosic_803_priv *nanosic_dev)
{
	int level;
	int retry = 2;

	level = gpiod_get_value_cansleep(nanosic_dev->irq_gpio);

	dev_dbg(nanosic_dev->dev, "setting sleep pin to 0\n");
	gpiod_set_value_cansleep(nanosic_dev->sleep_gpio, 0);

	if (level > 0) {
		dev_dbg(nanosic_dev->dev, "Chip is asleep (IRQ level=%d), waking up...\n", level);
		mdelay(25);
		while (retry--) {
			dev_dbg(nanosic_dev->dev, "sleep pin: %d\n", gpiod_get_value(nanosic_dev->sleep_gpio));
			if (gpiod_get_value_cansleep(nanosic_dev->irq_gpio) <= 0) {
				dev_dbg(nanosic_dev->dev, "Wake up successful.\n");
				return;
			}
			mdelay(1);
		}

		dev_err(nanosic_dev->dev, "Failed to wake up chip, resetting...\n");
		gpiod_set_value_cansleep(nanosic_dev->reset_gpio, 1);
		mdelay(100);
		gpiod_set_value_cansleep(nanosic_dev->reset_gpio, 0);
		mdelay(500);
		dev_dbg(nanosic_dev->dev, "reset pin: %d, sleep pin: %d\n", gpiod_get_value(nanosic_dev->reset_gpio), gpiod_get_value(nanosic_dev->sleep_gpio));
	}
}

static int nanosic_i2c_read(struct nanosic_803_priv *nanosic_dev, void *buf, size_t len)
{
	int ret = i2c_master_recv(nanosic_dev->client, buf, len);

	// nanosic_803_wakeup(nanosic_dev);

	if (ret < 0) {
		dev_err(nanosic_dev->dev, "i2c_master_recv error: %d\n", ret);
		return ret;
	}
	if (ret != len) {
		dev_err(nanosic_dev->dev, "i2c_master_recv incomplete read: got %d, expected %zu\n", ret, len);
		return -EIO;
	}
	return ret;
}

static int nanosic_803_read_version(struct nanosic_803_priv *nanosic_dev)
{
	char rsp[I2C_DATA_LENGTH_READ] = {0};
	char cmd[] = {
		0x32, 0x00, 0x4F, 0x30, 0x80,
		0x18, 0x01, 0x00
	};
	u8 retry = 0;
	int ret = -1;

	while (retry++ < 30) {
		ret = nanosic_send_command(nanosic_dev, cmd, sizeof(cmd));
		if (ret < 0) {
			dev_err(nanosic_dev->dev, "regmap write cmd failed time %d\n", retry);
			msleep(100);
			continue;
		}
		msleep(2);

		ret = nanosic_i2c_read(nanosic_dev, rsp, sizeof(rsp));
		if (ret > 0) {
			nanosic_print_cmd(nanosic_dev, "nanosic chip version: ", rsp, sizeof(rsp));
			dev_err(nanosic_dev->dev, "Version read OK\n");
			break;
		}
		msleep(2);
	}

	return ret;
}

static void nanosic_sync_caps_led(struct work_struct *work)
{
	struct nanosic_803_priv *nanosic_dev = container_of(work, struct nanosic_803_priv, led_work);
	int ret = 0;

	dev_dbg(nanosic_dev->dev, "setting caps led: %d\n", nanosic_dev->caps_led_on);
	char cmd[] = {
		0x32, 0x00, 0x4E, 0x31,
		0x80, 0x38, 0x26, 0x01, nanosic_dev->caps_led_on
	};

	mutex_lock(&nanosic_dev->i2c_mutex);
	nanosic_803_wakeup(nanosic_dev);
	ret = nanosic_send_command(nanosic_dev, cmd, sizeof(cmd));
	mutex_unlock(&nanosic_dev->i2c_mutex);
	if (ret < 0)
		dev_err(nanosic_dev->dev, "could not set caps led");
}

static int nanosic_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	struct nanosic_803_priv *nanosic_dev = input_get_drvdata(dev);
	bool prev_value = nanosic_dev->caps_led_on;

	dev_dbg(nanosic_dev->dev, "nanosic event type: %d, code: %u, value: %d\n", type, code, value);
	if (!nanosic_dev)
		return -EINVAL;

	if (type == EV_LED) {
		switch (code) {
			case LED_CAPSL:
				nanosic_dev->caps_led_on = value;
				if (value != prev_value)
					queue_work(nanosic_dev->wq, &nanosic_dev->led_work);
				break;
			default:
				break;
		}
	}
	return 0;
}

static int nanosic_register_keyboard(struct nanosic_803_priv *nanosic_dev)
{
	struct input_dev *keyboard_input_dev;
	int ret;

	if (nanosic_dev->keyboard_input_dev) {
		dev_dbg(nanosic_dev->dev, "register_keyboard: keyboard input device is already registered\n");
		return 0;
	}

	// Allocating keyboard device
	keyboard_input_dev = devm_input_allocate_device(nanosic_dev->dev);
	if (!keyboard_input_dev) {
		ret = -ENOMEM;
		dev_err(nanosic_dev->dev, "could not allocate keyboard input device: %d\n", ret);
		return ret;
	}
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

	for (int i = 0; i < KEY_MAX; i++)
		set_bit(i, keyboard_input_dev->keybit);

	// Registering keyboard device
	nanosic_dev->keyboard_input_dev = keyboard_input_dev;
	ret = input_register_device(nanosic_dev->keyboard_input_dev);
	if (ret)
		dev_err(nanosic_dev->dev, "failed to register input device: %d\n", ret);
	return ret;
}

static int nanosic_register_touchpad(struct nanosic_803_priv *nanosic_dev)
{
	struct input_dev *touchpad_input_dev;
	int ret;
	unsigned int touchpad_resolution_x, touchpad_resolution_y;

	if (nanosic_dev->touchpad_input_dev) {
		dev_dbg(nanosic_dev->dev, "register_touchpad: touchpad input device is already registered\n");
		return 0;
	}

	// Get touchpad resolution
	if (of_property_read_u32(nanosic_dev->dev->of_node, "touchpad-resolution-x", &touchpad_resolution_x)) {
		dev_err(nanosic_dev->dev, "Failed to read touchpad-resolution-x from DT\n");
		return -EINVAL;
	}

	if (of_property_read_u32(nanosic_dev->dev->of_node, "touchpad-resolution-y", &touchpad_resolution_y)) {
		dev_err(nanosic_dev->dev, "Failed to read touchpad-resolution-y from DT\n");
		return -EINVAL;
	}

	// Allocating touchpad device
	touchpad_input_dev = devm_input_allocate_device(nanosic_dev->dev);
	if (!touchpad_input_dev) {
		pr_err("Failed to allocate touchpad input device\n");
		return -ENOMEM;
	}

	touchpad_input_dev->name = "Nanosic 803 touchpad";
	touchpad_input_dev->phys = "input/touchpad";
	touchpad_input_dev->id.bustype = BUS_I2C;
	touchpad_input_dev->id.vendor = 0x1234;
	touchpad_input_dev->id.product = 0x5678;
	touchpad_input_dev->id.version = 0x0001;

	ret = input_mt_init_slots(touchpad_input_dev, 3, INPUT_MT_POINTER);
	if (ret) {
		dev_err(nanosic_dev->dev, "Failed to initialize MT slots: %d\n", ret);
		return ret;
	}

	set_bit(INPUT_PROP_POINTER, touchpad_input_dev->propbit);
	set_bit(EV_ABS, touchpad_input_dev->evbit);
	input_set_abs_params(touchpad_input_dev, ABS_MT_POSITION_X, 0, touchpad_resolution_x, 0, 0);
	input_set_abs_params(touchpad_input_dev, ABS_MT_POSITION_Y, 0, touchpad_resolution_y, 0, 0);
	input_set_abs_params(touchpad_input_dev, ABS_MT_TRACKING_ID, 0, 3 - 1, 0, 0);

	set_bit(EV_KEY, touchpad_input_dev->evbit);
	set_bit(BTN_LEFT, touchpad_input_dev->keybit);
	set_bit(BTN_RIGHT, touchpad_input_dev->keybit);

	input_set_drvdata(touchpad_input_dev, nanosic_dev);

	// Registering touchpad device
	nanosic_dev->touchpad_input_dev = touchpad_input_dev;
	ret = input_register_device(nanosic_dev->touchpad_input_dev);
	if (ret)
		dev_err(nanosic_dev->dev, "failed to register input device: %d\n", ret);
	return ret;
}


static void nanosic_handle_hall(struct nanosic_803_priv *nanosic_dev, char *buf)
{
	if (buf[5] == 0x38 && buf[6] == 0x80 && buf[7] == 0xA2) {
		if (buf[12] == 0x23) {
			dev_dbg(nanosic_dev->dev, "registering input devices\n");
			if (!nanosic_dev->keyboard_input_dev)
				nanosic_register_keyboard(nanosic_dev);
			if (!nanosic_dev->touchpad_input_dev)
				nanosic_register_touchpad(nanosic_dev);
		} else if (buf[12] == 0x0) {
			dev_dbg(nanosic_dev->dev, "unregistering input devices\n");
			if (nanosic_dev->keyboard_input_dev) {
				input_unregister_device(nanosic_dev->keyboard_input_dev);
				nanosic_dev->keyboard_input_dev = NULL;
			}
			if (nanosic_dev->touchpad_input_dev) {
				input_unregister_device(nanosic_dev->touchpad_input_dev);
				nanosic_dev->touchpad_input_dev = NULL;
			}
		}
	}
}

static void nanosic_handle_modifiers(struct nanosic_803_priv *nanosic_dev, char modifiers)
{
	char last_modifiers = nanosic_dev->last_modifier_state;

	for (int i = 0; i < 8; i++) {
		uint8_t current_bit = (modifiers >> i) & 1;
		uint8_t last_bit = (last_modifiers >> i) & 1;

		if (current_bit && !last_bit) {
			input_report_key(nanosic_dev->keyboard_input_dev, hid_modifier_to_linux_keycode[i], 1);
			dev_dbg(nanosic_dev->dev, "Modifier pressed: %d\n", hid_modifier_to_linux_keycode[i]);
		} else if (!current_bit && last_bit) {
			input_report_key(nanosic_dev->keyboard_input_dev, hid_modifier_to_linux_keycode[i], 0);
			dev_dbg(nanosic_dev->dev, "Modifier released: %d\n", hid_modifier_to_linux_keycode[i]);
		}
	}

	input_sync(nanosic_dev->keyboard_input_dev);

	nanosic_dev->last_modifier_state = modifiers;
}

static void nanosic_handle_keyboard(struct nanosic_803_priv *nanosic_dev, char *buf)
{
	int i, j;
	int found;
	int keycode;

	if (!nanosic_dev->keyboard_input_dev) {
		nanosic_register_keyboard(nanosic_dev);
		msleep(100);
	}

	nanosic_handle_modifiers(nanosic_dev, buf[4]);

	for (i = 0; i < 5 && buf[6+i] != 0x00; ++i) {
		found = 0;
		for (j = 0; j < 5 && nanosic_dev->last_pressed_key[j] != 0x00; ++j) {
			if (buf[6+i] == nanosic_dev->last_pressed_key[j]) {
				found = 1;
				break;
			}
		}
		if (!found) {
			dev_dbg(nanosic_dev->dev, "Key pressed: 0x%02X\n", buf[6+i]);
			keycode = hid_to_linux_keycode[(unsigned char)buf[6+i]];
			if (nanosic_dev->caps_as_second_layer_key && keycode == KEY_CAPSLOCK) {
				dev_dbg(nanosic_dev->dev, "Second layer");
				nanosic_dev->second_layer_active = true;
				continue;
			}
			if (nanosic_dev->second_layer_active)
				keycode = second_layer_to_linux_keycode[(unsigned char)buf[6+i]];
			input_report_key(nanosic_dev->keyboard_input_dev, keycode, 1);
			input_sync(nanosic_dev->keyboard_input_dev);
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
			dev_dbg(nanosic_dev->dev, "Key released: 0x%02X\n", nanosic_dev->last_pressed_key[i]);
			keycode = hid_to_linux_keycode[(unsigned char)nanosic_dev->last_pressed_key[i]];
			if (nanosic_dev->caps_as_second_layer_key && keycode == KEY_CAPSLOCK) {
				dev_dbg(nanosic_dev->dev, "First layer");
				nanosic_dev->second_layer_active = false;
				continue;
			}

			// release the key from both layers
			input_report_key(nanosic_dev->keyboard_input_dev, keycode, 0);
			input_report_key(nanosic_dev->keyboard_input_dev, second_layer_to_linux_keycode[(unsigned char)nanosic_dev->last_pressed_key[i]], 0);
			input_sync(nanosic_dev->keyboard_input_dev);
		}
	}
	memcpy(nanosic_dev->last_pressed_key, &buf[6], sizeof(nanosic_dev->last_pressed_key));
}

static void nanosic_handle_fn_key(struct nanosic_803_priv *nanosic_dev, char *buf)
{
	if (!nanosic_dev->keyboard_input_dev) {
		nanosic_register_keyboard(nanosic_dev);
		msleep(100);
	}

	if (buf[4] != nanosic_dev->last_fn_key) {
		if (nanosic_dev->last_fn_key != 0x00) {
			dev_dbg(nanosic_dev->dev, "Key released: 0x%02X\n", nanosic_dev->last_fn_key);
			input_report_key(nanosic_dev->keyboard_input_dev, hid_to_linux_keycode[(unsigned char)nanosic_dev->last_fn_key], 0);
			input_sync(nanosic_dev->keyboard_input_dev);
		}
		if (buf[4] != 0x00) {
			dev_dbg(nanosic_dev->dev, "Key pressed: 0x%02X\n", buf[4]);
			input_report_key(nanosic_dev->keyboard_input_dev, hid_to_linux_keycode[(unsigned char)buf[4]], 1);
			input_sync(nanosic_dev->keyboard_input_dev);
		} else if (buf[8] == 0x05 && hid_to_linux_keycode[(unsigned char)buf[11]] == KEY_2 && hid_to_linux_keycode[(unsigned char)buf[12]] == KEY_1) {
			/*
			 * Use Caps Lock as a second layer key if fn+1+2 was pressed:
			 * 57  03  4a  06  00  00  00  00  05  00  00  1f  1e  00  00  00  00  00  00  00  00  <...>
			 *              ^      (key event)--^           ^   ^---(KEY_1)
			 *              |                               |
			 *               \-(empty fn key event)          \-(KEY_2)
			 */
			nanosic_dev->caps_as_second_layer_key = !nanosic_dev->caps_as_second_layer_key;
			dev_dbg(nanosic_dev->dev, "Caps Lock as second layer key: %d\n", nanosic_dev->caps_as_second_layer_key);
		}
		nanosic_dev->last_fn_key = buf[4];
	}
}

static void nanosic_touch_timer_callback(struct timer_list *t)
{
	struct nanosic_803_priv *nanosic_dev = from_timer(nanosic_dev, t, finger_timer);

	if (nanosic_dev->finger_down) {
		for (int i = 0; i<3; i++) {
			input_mt_slot(nanosic_dev->touchpad_input_dev, nanosic_dev->slot_mapping[i]);
			input_mt_report_slot_state(nanosic_dev->touchpad_input_dev, MT_TOOL_FINGER, 0);
			input_report_abs(nanosic_dev->touchpad_input_dev, ABS_MT_TRACKING_ID, -1);
			input_sync(nanosic_dev->touchpad_input_dev);
		}
        nanosic_dev->finger_down = false;
    }
}

static void nanosic_handle_touchpad_mt(struct nanosic_803_priv *nanosic_dev, char *buf)
{
	int finger_id, x, y;

	if (!nanosic_dev->touchpad_input_dev) {
		nanosic_register_touchpad(nanosic_dev);
		msleep(100);
	}

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

static irqreturn_t nanosic_irq_handler(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t nanosic_interrupt_thread_fn(int irq, void *dev_id)
{
	struct nanosic_803_priv *nanosic_dev = dev_id;
	int ret;
	char buf[I2C_DATA_LENGTH_READ] = {0};

	usleep_range(1000, 5000);

	mutex_lock(&nanosic_dev->i2c_mutex);
	ret = nanosic_i2c_read(nanosic_dev, buf, sizeof(buf));
	mutex_unlock(&nanosic_dev->i2c_mutex);
	if (ret <= 0) {
		dev_err(nanosic_dev->dev, "Failed to read data on interrupt: %d\n", ret);
		return IRQ_HANDLED;
	}

	nanosic_print_cmd(nanosic_dev, "nanosic message: ", buf, ret);

	if (buf[0] != 0x57 || buf[2] == 0) {
		dev_dbg(nanosic_dev->dev, "Malformed message\n");
		/*
		 * Our hardware might randomly return no message/random data,
		 * so assume we handled IRQ correctly
		 */
		return IRQ_HANDLED;
	}

	switch (buf[3]) {
		case 0x05:
			// Handle keyboard event
			nanosic_handle_keyboard(nanosic_dev, buf);
			break;
		case 0x06:
			// Handle function keys
			nanosic_handle_fn_key(nanosic_dev, buf);
			break;
		case 0x19:
			// Handle touchpad event
			nanosic_handle_touchpad_mt(nanosic_dev, buf);
			break;
		case 0x23:
			// Handle hall event
			nanosic_handle_hall(nanosic_dev, buf);
			break;
	}
	return IRQ_HANDLED;
}

static void nanosic_803_reset(struct nanosic_803_priv *nanosic_dev)
{
	gpiod_set_value(nanosic_dev->reset_gpio, 1);
	gpiod_set_value(nanosic_dev->sleep_gpio, 1);

	gpiod_set_value(nanosic_dev->vdd_gpio, 0);

	msleep(100);

	gpiod_set_value(nanosic_dev->vdd_gpio, 1);

	msleep(2);

	gpiod_set_value(nanosic_dev->reset_gpio, 0);
	gpiod_set_value(nanosic_dev->sleep_gpio, 0);
}

static int hex_str_to_bin(u8 *bin, const char *hex, size_t *bin_len)
{
	size_t hex_len = strlen(hex);
	size_t max_len = *bin_len;
	size_t count = 0;
	int i;
	int nibble = -1; // -1: look for high nibble, 1: look for low nibble

	for (i = 0; i < hex_len; i++) {
		char c = hex[i];
		int val;

		if (isspace(c))
			continue;

		val = hex_to_bin(c);
		if (val < 0) {
			pr_err("Invalid character '%c' in hex string\n", c);
			return -EINVAL;
		}

		if (nibble < 0) {
			if (count >= max_len)
				return -ENOSPC; // Output buffer is full
			bin[count] = val << 4;
			nibble = 1;
		} else {
			bin[count] |= val;
			count++;
			nibble = -1;
		}
	}

	if (nibble > 0) {
		pr_err("Odd number of hex digits in string\n");
		return -EINVAL;
	}

	*bin_len = count;
	return 0;
}


static ssize_t nanosic_debugfs_cmd_write(struct file *file, const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct nanosic_803_priv *nanosic_dev = file->private_data;
	char *user_buf;
	u8 *cmd_buf;
	int ret;
	size_t cmd_len = I2C_DATA_LENGTH_WRITE;

	if (!nanosic_dev) {
		pr_err("debugfs write called with NULL private data\n");
		return -EIO;
	}

	user_buf = memdup_user_nul(ubuf, count);
	if (IS_ERR(user_buf))
		return PTR_ERR(user_buf);

	cmd_buf = kmalloc(cmd_len, GFP_KERNEL);
	if (!cmd_buf) {
		kfree(user_buf);
		return -ENOMEM;
	}

	ret = hex_str_to_bin(cmd_buf, user_buf, &cmd_len);
	if (ret) {
		dev_err(nanosic_dev->dev, "Failed to parse hex string: %d\n", ret);
		goto out;
	}

	if (cmd_len == 0) {
		dev_warn(nanosic_dev->dev, "Empty command after parsing\n");
		ret = count;
		goto out;
	}

	dev_dbg(nanosic_dev->dev, "Sending command of length %zu\n", cmd_len);

	nanosic_print_cmd(nanosic_dev, "nanosic: debugfs_write cmd: ", cmd_buf, cmd_len);

	mutex_lock(&nanosic_dev->i2c_mutex);
	nanosic_803_wakeup(nanosic_dev);
	dev_dbg(nanosic_dev->dev, "reset pin: %d, sleep pin: %d\n", gpiod_get_value(nanosic_dev->reset_gpio), gpiod_get_value(nanosic_dev->sleep_gpio));
	ret = nanosic_send_command(nanosic_dev, cmd_buf, cmd_len);
	mutex_unlock(&nanosic_dev->i2c_mutex);

	if (ret < 0) {
		dev_err(nanosic_dev->dev, "Failed to write command via regmap: %d\n", ret);
	} else {
		ret = count;
	}

out:
	kfree(cmd_buf);
	kfree(user_buf);
	return ret;
}

static ssize_t nanosic_debugfs_sleep_read(struct file *file, char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct nanosic_803_priv *nanosic_dev = file->private_data;
	char buf[4];
	int level;
	size_t len;

	if (*ppos > 0)
		return 0;

	level = gpiod_get_value_cansleep(nanosic_dev->sleep_gpio);
	if (level < 0)
		return level;

	len = scnprintf(buf, sizeof(buf), "%d\n", level);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t nanosic_debugfs_sleep_write(struct file *file, const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct nanosic_803_priv *nanosic_dev = file->private_data;
	long val;
	int ret;

	ret = kstrtol_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	if (val != 0 && val != 1)
		return -EINVAL;

	dev_dbg(nanosic_dev->dev, "debugfs: setting sleep_gpio to %ld\n", val);
	gpiod_set_value_cansleep(nanosic_dev->sleep_gpio, val);

	return count;
}

static const struct file_operations nanosic_debugfs_cmd_fops = {
	.owner   = THIS_MODULE,
	.open    = simple_open,
	.write   = nanosic_debugfs_cmd_write,
	.llseek  = noop_llseek,
};
static const struct file_operations nanosic_debugfs_sleep_fops = {
	.owner   = THIS_MODULE,
	.open    = simple_open,
	.read    = nanosic_debugfs_sleep_read,
	.write   = nanosic_debugfs_sleep_write,
	.llseek  = default_llseek,
};
static void nanosic_debugfs_init(struct nanosic_803_priv *nanosic_dev)
{
	nanosic_dev->debugfs_root = debugfs_create_dir("nanosic_803", NULL);
	if (IS_ERR_OR_NULL(nanosic_dev->debugfs_root)) {
		dev_err(nanosic_dev->dev, "Failed to create debugfs directory\n");
		nanosic_dev->debugfs_root = NULL;
		return;
	}

	debugfs_create_file("send_command", 0200, nanosic_dev->debugfs_root, nanosic_dev, &nanosic_debugfs_cmd_fops);
	debugfs_create_file("sleep_pin", 0644, nanosic_dev->debugfs_root, nanosic_dev, &nanosic_debugfs_sleep_fops);
}

static void nanosic_debugfs_remove(struct nanosic_803_priv *nanosic_dev)
{
	debugfs_remove_recursive(nanosic_dev->debugfs_root);
}

static int nanosic_803_probe(struct i2c_client *client)
{
	struct nanosic_803_priv *nanosic_dev;
	struct device *dev = &client->dev;
	struct regmap *map;
	int ret;

	nanosic_dev = devm_kzalloc(dev, sizeof(*nanosic_dev), GFP_KERNEL);
	if (!nanosic_dev)
		return -ENOMEM;

	nanosic_dev->dev = dev;
	dev_set_drvdata(dev, nanosic_dev);

	nanosic_dev->wq = create_singlethread_workqueue("nanosic_wq");
	INIT_WORK(&nanosic_dev->led_work, nanosic_sync_caps_led);

	// Get GPIOs
	nanosic_dev->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(nanosic_dev->reset_gpio)) {
		dev_err(dev, "Failed to get reset GPIO\n");
		return PTR_ERR(nanosic_dev->reset_gpio);
	}

	nanosic_dev->sleep_gpio = devm_gpiod_get(dev, "sleep", GPIOD_OUT_LOW);
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
		return PTR_ERR(nanosic_dev->irq_gpio);
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
		regulator_disable(nanosic_dev->vdd_1v8);
		return PTR_ERR(nanosic_dev->vdd_3v3);
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

	// Set up regmap
	map = devm_regmap_init_i2c(client, &nanosic_803_regmap_config);
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		goto err_regulator_disable;
	}

	nanosic_dev->client = client;
	nanosic_dev->regmap = map;

	mutex_init(&nanosic_dev->i2c_mutex);

	i2c_set_clientdata(client, nanosic_dev);

	// Try to identify the chip
	if (nanosic_803_read_version(nanosic_dev) <= 0) {
		dev_err(dev, "Nanosic 803 not found\n");
		return -ENODEV;
	}

	// Set up IRQ
	nanosic_dev->irq_number = gpiod_to_irq(nanosic_dev->irq_gpio);
	if (nanosic_dev->irq_number < 0) {
		dev_err(nanosic_dev->dev, "Failed to get IRQ for GPIO %d\n", desc_to_gpio(nanosic_dev->irq_gpio));
		ret = nanosic_dev->irq_number;
		goto err_regulator_disable;
	}

	ret = request_threaded_irq(nanosic_dev->irq_number,
		nanosic_irq_handler, nanosic_interrupt_thread_fn,
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

	nanosic_debugfs_init(nanosic_dev);

	return 0;

err_regulator_disable:
	regulator_disable(nanosic_dev->vdd_3v3);
	regulator_disable(nanosic_dev->vdd_1v8);
	return ret;
}

static void nanosic_803_remove(struct i2c_client *client)
{
	struct nanosic_803_priv *nanosic_dev = i2c_get_clientdata(client);

	free_irq(nanosic_dev->irq_number, nanosic_dev);

	nanosic_debugfs_remove(nanosic_dev);

	cancel_work_sync(&nanosic_dev->led_work);
	destroy_workqueue(nanosic_dev->wq);

	gpiod_set_value(nanosic_dev->reset_gpio, 1);
	gpiod_set_value(nanosic_dev->sleep_gpio, 1);

	gpiod_set_value(nanosic_dev->vdd_gpio, 0);

	if (regulator_disable(nanosic_dev->vdd_1v8))
		dev_err(nanosic_dev->dev, "Failed to disable 1.8V regulator\n");

	if (regulator_disable(nanosic_dev->vdd_3v3))
		dev_err(nanosic_dev->dev, "Failed to disable 3.3V regulator\n");

	if (nanosic_dev->keyboard_input_dev)
		input_unregister_device(nanosic_dev->keyboard_input_dev);
	if (nanosic_dev->touchpad_input_dev)
		input_unregister_device(nanosic_dev->touchpad_input_dev);
}

static int nanosic_803_suspend(struct device *dev)
{
	struct nanosic_803_priv *nanosic_dev = dev_get_drvdata(dev);
	int ret;

	// Actually de-init device
	gpiod_set_value(nanosic_dev->reset_gpio, 1);
	gpiod_set_value(nanosic_dev->sleep_gpio, 1);

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
		dev_err(dev, "Failed to enable 1.8V regulator\n");
		return ret;
	}

	ret = regulator_enable(nanosic_dev->vdd_3v3);
	if (ret) {
		dev_err(dev, "Failed to enable 3.3V regulator\n");
		regulator_disable(nanosic_dev->vdd_1v8);
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
	.remove = nanosic_803_remove,
};

module_i2c_driver(nanosic_803_driver);

MODULE_AUTHOR("Luka Panio <lukapanio@gmail.com>");
MODULE_DESCRIPTION("Driver for nanosic 803 keyboard controller");
MODULE_LICENSE("GPL v2");
