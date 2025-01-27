// SPDX-License-Identifier: GPL-2.0-only
/*
 * A V4L2 driver for Sony OV13B10I2C cameras.
 *
 * Based on Sony imx412 camera driver
 * Copyright (C) 2021 Intel Corporation
 */
#include <linux/unaligned.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Streaming Mode */
#define OV13B10I2C_REG_MODE_SELECT	0x0100
#define OV13B10I2C_MODE_STANDBY		0x00
#define OV13B10I2C_MODE_STREAMING	0x01

/* Lines per frame */
#define OV13B10I2C_REG_LPFR		0x0340

/* Chip ID */
#define OV13B10I2C_REG_ID		0x300B
#define OV13B10I2C_ID			0xD42

/* Exposure control */
#define OV13B10I2C_REG_EXPOSURE_CIT	0x0202
#define OV13B10I2C_EXPOSURE_MIN		8
#define OV13B10I2C_EXPOSURE_OFFSET	22
#define OV13B10I2C_EXPOSURE_STEP	1
#define OV13B10I2C_EXPOSURE_DEFAULT	0x0648

/* Analog gain control */
#define OV13B10I2C_REG_AGAIN		0x0204
#define OV13B10I2C_AGAIN_MIN		0
#define OV13B10I2C_AGAIN_MAX		978
#define OV13B10I2C_AGAIN_STEP		1
#define OV13B10I2C_AGAIN_DEFAULT	0

/* Group hold register */
#define OV13B10I2C_REG_HOLD		0x0104

/* Input clock rate */
#define OV13B10I2C_INCLK_RATE_19Mhz	19200000
#define OV13B10I2C_INCLK_RATE_24Mhz	24000000

/* CSI2 HW configuration */
#define OV13B10I2C_LINK_FREQ		560000000
#define OV13B10I2C_NUM_DATA_LANES	4

#define OV13B10I2C_REG_MIN		0x00
#define OV13B10I2C_REG_MAX		0xffff

/**
 * struct ov13b10i2c_reg - ov13b10i2c sensor register
 * @address: Register address
 * @val: Register value
 */
struct ov13b10i2c_reg {
	u16 address;
	u8 val;
};

/**
 * struct ov13b10i2c_reg_list - ov13b10i2c sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct ov13b10i2c_reg_list {
	u32 num_of_regs;
	const struct ov13b10i2c_reg *regs;
};

static const int ov13b10i2c_inclk_list[] = {
	OV13B10I2C_INCLK_RATE_19Mhz,
	OV13B10I2C_INCLK_RATE_24Mhz,
};

/**
 * struct ov13b10i2c_mode - ov13b10i2c sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @hblank: Horizontal blanking in lines
 * @vblank: Vertical blanking in lines
 * @vblank_min: Minimum vertical blanking in lines
 * @vblank_max: Maximum vertical blanking in lines
 * @pclk: Sensor pixel clock
 * @link_freq_idx: Link frequency index
 * @reg_list: Register list for sensor mode
 */
struct ov13b10i2c_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 hblank;
	u32 vblank;
	u32 vblank_min;
	u32 vblank_max;
	u64 pclk;
	u32 link_freq_idx;
	struct ov13b10i2c_reg_list reg_list;
};

static const char * const ov13b10i2c_supply_names[] = {
	"vddio",	/* 1.8V I/O Power */
	"vddd",		/* 1.05V Digital Power */
	"vdda",		/* 2.8V Analog Power */
};

/**
 * struct ov13b10i2c - ov13b10i2c sensor device structure
 * @dev: Pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @supplies: Regulator supplies
 * @ctrl_handler: V4L2 control handler
 * @link_freq_ctrl: Pointer to link frequency control
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @vblank: Vertical blanking in lines
 * @cur_mode: Pointer to current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 */
struct ov13b10i2c {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct clk *inclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ov13b10i2c_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	u32 vblank;
	const struct ov13b10i2c_mode *cur_mode;
	struct mutex mutex;
};

static const s64 link_freq[] = {
	OV13B10I2C_LINK_FREQ,
};

/* Sensor mode registers */
static const struct ov13b10i2c_reg mode_2104x1560_regs[] = {
	{0x0103, 0x01},
	{0x0303, 0x04},
	{0x0305, 0xaf},
	{0x0321, 0x00},
	{0x0323, 0x04},
	{0x0324, 0x01},
	{0x0325, 0xa4},
	{0x0326, 0x81},
	{0x0327, 0x04},
	{0x3012, 0x07},
	{0x3013, 0x32},
	{0x3107, 0x23},
	{0x3501, 0x0c},
	{0x3502, 0x10},
	{0x3504, 0x08},
	{0x3508, 0x07},
	{0x3509, 0xc0},
	{0x3600, 0x16},
	{0x3601, 0x54},
	{0x3612, 0x4e},
	{0x3620, 0x00},
	{0x3621, 0x68},
	{0x3622, 0x66},
	{0x3623, 0x03},
	{0x3662, 0x92},
	{0x3666, 0xbb},
	{0x3667, 0x44},
	{0x366e, 0xff},
	{0x366f, 0xf3},
	{0x3675, 0x44},
	{0x3676, 0x00},
	{0x367f, 0xe9},
	{0x3681, 0x32},
	{0x3682, 0x1f},
	{0x3683, 0x0b},
	{0x3684, 0x0b},
	{0x3704, 0x0f},
	{0x3706, 0x40},
	{0x3708, 0x3b},
	{0x3709, 0x72},
	{0x370b, 0xa2},
	{0x3714, 0x24},
	{0x371a, 0x3e},
	{0x3725, 0x42},
	{0x3739, 0x12},
	{0x3767, 0x00},
	{0x377a, 0x0d},
	{0x3789, 0x18},
	{0x3790, 0x40},
	{0x3791, 0xa2},
	{0x37c2, 0x04},
	{0x37c3, 0xf1},
	{0x37d9, 0x0c},
	{0x37da, 0x02},
	{0x37dc, 0x02},
	{0x37e1, 0x04},
	{0x37e2, 0x0a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x10},
	{0x3809, 0x70},
	{0x380a, 0x0c},
	{0x380b, 0x30},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x0c},
	{0x380f, 0x7c},
	{0x3811, 0x0f},
	{0x3813, 0x09},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3816, 0x01},
	{0x3817, 0x01},
	{0x381f, 0x08},
	{0x3820, 0x88},
	{0x3821, 0x00},
	{0x3822, 0x14},
	{0x382e, 0xe6},
	{0x3c80, 0x00},
	{0x3c87, 0x01},
	{0x3c8c, 0x19},
	{0x3c8d, 0x1c},
	{0x3ca0, 0x00},
	{0x3ca1, 0x00},
	{0x3ca2, 0x00},
	{0x3ca3, 0x00},
	{0x3ca4, 0x50},
	{0x3ca5, 0x11},
	{0x3ca6, 0x01},
	{0x3ca7, 0x00},
	{0x3ca8, 0x00},
	{0x4008, 0x02},
	{0x4009, 0x0f},
	{0x400a, 0x01},
	{0x400b, 0x19},
	{0x4011, 0x21},
	{0x4017, 0x08},
	{0x4019, 0x04},
	{0x401a, 0x58},
	{0x4032, 0x1e},
	{0x4050, 0x02},
	{0x4051, 0x09},
	{0x405e, 0x00},
	{0x4066, 0x02},
	{0x4501, 0x00},
	{0x4502, 0x10},
	{0x4505, 0x00},
	{0x4800, 0x64},
	{0x481b, 0x3e},
	{0x481f, 0x30},
	{0x4825, 0x34},
	{0x4837, 0x0e},
	{0x484b, 0x01},
	{0x4883, 0x02},
	{0x5000, 0xff},
	{0x5001, 0x0f},
	{0x5045, 0x20},
	{0x5046, 0x20},
	{0x5047, 0xa4},
	{0x5048, 0x20},
	{0x5049, 0xa4},

	{0x0305, 0xaf},
	{0x3501, 0x06},
	{0x3662, 0x88},
	{0x3714, 0x28},
	{0x3739, 0x10},
	{0x37c2, 0x14},
	{0x37d9, 0x06},
	{0x37e2, 0x0c},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x08},
	{0x3809, 0x38},
	{0x380a, 0x06},
	{0x380b, 0x18},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x06},
	{0x380f, 0x3e},
	{0x3810, 0x00},
	{0x3811, 0x07},
	{0x3812, 0x00},
	{0x3813, 0x05},
	{0x3814, 0x03},
	{0x3816, 0x03},
	{0x3820, 0x8b},
	{0x3c8c, 0x18},
	{0x4008, 0x00},
	{0x4009, 0x05},
	{0x4050, 0x00},
	{0x4051, 0x05},
	{0x4501, 0x08},
	{0x4505, 0x00},
	{0x4837, 0x0e},
	{0x5000, 0xfd},
	{0x5001, 0x0d},
};
/* Supported sensor mode configurations */
static const struct ov13b10i2c_mode supported_mode = {
	.width = 2104,
	.height = 1560,
	.hblank = 456, // FIXME
	.vblank = 506, // FIXME
	.vblank_min = 506, // FIXME
	.vblank_max = 32420, // FIXME
	.pclk = 480000000, // FIXME
	.link_freq_idx = 0,
	.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_2104x1560_regs),
		.regs = mode_2104x1560_regs,
	},
};

/**
 * to_ov13b10i2c() - ov13b10i2c V4L2 sub-device to ov13b10i2c device.
 * @subdev: pointer to ov13b10i2c V4L2 sub-device
 *
 * Return: pointer to ov13b10i2c device
 */
static inline struct ov13b10i2c *to_ov13b10i2c(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ov13b10i2c, sd);
}

/**
 * ov13b10i2c_read_reg() - Read registers.
 * @ov13b10i2c: pointer to ov13b10i2c device
 * @reg: register address
 * @len: length of bytes to read. Max supported bytes is 4
 * @val: pointer to register value to be filled.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_read_reg(struct ov13b10i2c *ov13b10i2c, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov13b10i2c->sd);
	struct i2c_msg msgs[2] = {0};
	u8 addr_buf[2] = {0};
	u8 data_buf[4] = {0};
	int ret;

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/**
 * ov13b10i2c_write_reg() - Write register
 * @ov13b10i2c: pointer to ov13b10i2c device
 * @reg: register address
 * @len: length of bytes. Max supported bytes is 4
 * @val: register value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_write_reg(struct ov13b10i2c *ov13b10i2c, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov13b10i2c->sd);
	u8 buf[6] = {0};

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/**
 * ov13b10i2c_write_regs() - Write a list of registers
 * @ov13b10i2c: pointer to ov13b10i2c device
 * @regs: list of registers to be written
 * @len: length of registers array
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_write_regs(struct ov13b10i2c *ov13b10i2c,
			     const struct ov13b10i2c_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = ov13b10i2c_write_reg(ov13b10i2c, regs[i].address, 1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * ov13b10i2c_update_controls() - Update control ranges based on streaming mode
 * @ov13b10i2c: pointer to ov13b10i2c device
 * @mode: pointer to ov13b10i2c_mode sensor mode
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_update_controls(struct ov13b10i2c *ov13b10i2c,
				  const struct ov13b10i2c_mode *mode)
{
	int ret;

	ret = __v4l2_ctrl_s_ctrl(ov13b10i2c->link_freq_ctrl, mode->link_freq_idx);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(ov13b10i2c->hblank_ctrl, mode->hblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_modify_range(ov13b10i2c->vblank_ctrl, mode->vblank_min,
					mode->vblank_max, 1, mode->vblank);
}

/**
 * ov13b10i2c_update_exp_gain() - Set updated exposure and gain
 * @ov13b10i2c: pointer to ov13b10i2c device
 * @exposure: updated exposure value
 * @gain: updated analog gain value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_update_exp_gain(struct ov13b10i2c *ov13b10i2c, u32 exposure, u32 gain)
{
	u32 lpfr;
	int ret;

	lpfr = ov13b10i2c->vblank + ov13b10i2c->cur_mode->height;

	dev_dbg(ov13b10i2c->dev, "Set exp %u, analog gain %u, lpfr %u\n",
		exposure, gain, lpfr);

	ret = ov13b10i2c_write_reg(ov13b10i2c, OV13B10I2C_REG_HOLD, 1, 1);
	if (ret)
		return ret;

	ret = ov13b10i2c_write_reg(ov13b10i2c, OV13B10I2C_REG_LPFR, 2, lpfr);
	if (ret)
		goto error_release_group_hold;

	ret = ov13b10i2c_write_reg(ov13b10i2c, OV13B10I2C_REG_EXPOSURE_CIT, 2, exposure);
	if (ret)
		goto error_release_group_hold;

	ret = ov13b10i2c_write_reg(ov13b10i2c, OV13B10I2C_REG_AGAIN, 2, gain);

error_release_group_hold:
	ov13b10i2c_write_reg(ov13b10i2c, OV13B10I2C_REG_HOLD, 1, 0);

	return ret;
}

/**
 * ov13b10i2c_set_ctrl() - Set subdevice control
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Supported controls:
 * - V4L2_CID_VBLANK
 * - cluster controls:
 *   - V4L2_CID_ANALOGUE_GAIN
 *   - V4L2_CID_EXPOSURE
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov13b10i2c *ov13b10i2c =
		container_of(ctrl->handler, struct ov13b10i2c, ctrl_handler);
	u32 analog_gain;
	u32 exposure;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ov13b10i2c->vblank = ov13b10i2c->vblank_ctrl->val;

		dev_dbg(ov13b10i2c->dev, "Received vblank %u, new lpfr %u\n",
			ov13b10i2c->vblank,
			ov13b10i2c->vblank + ov13b10i2c->cur_mode->height);

		ret = __v4l2_ctrl_modify_range(ov13b10i2c->exp_ctrl,
					       OV13B10I2C_EXPOSURE_MIN,
					       ov13b10i2c->vblank +
					       ov13b10i2c->cur_mode->height -
					       OV13B10I2C_EXPOSURE_OFFSET,
					       1, OV13B10I2C_EXPOSURE_DEFAULT);
		break;
	case V4L2_CID_EXPOSURE:
		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(ov13b10i2c->dev))
			return 0;

		exposure = ctrl->val;
		analog_gain = ov13b10i2c->again_ctrl->val;

		dev_dbg(ov13b10i2c->dev, "Received exp %u, analog gain %u\n",
			exposure, analog_gain);

		ret = ov13b10i2c_update_exp_gain(ov13b10i2c, exposure, analog_gain);

		pm_runtime_put(ov13b10i2c->dev);

		break;
	default:
		dev_err(ov13b10i2c->dev, "Invalid control %d\n", ctrl->id);
		ret = -EINVAL;
	}

	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops ov13b10i2c_ctrl_ops = {
	.s_ctrl = ov13b10i2c_set_ctrl,
};

/**
 * ov13b10i2c_enum_mbus_code() - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to ov13b10i2c V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @code: V4L2 sub-device code enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = supported_mode.code;

	return 0;
}

/**
 * ov13b10i2c_enum_frame_size() - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to ov13b10i2c V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->index > 0)
		return -EINVAL;

	if (fsize->code != supported_mode.code)
		return -EINVAL;

	fsize->min_width = supported_mode.width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_mode.height;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * ov13b10i2c_fill_pad_format() - Fill subdevice pad format
 *                            from selected sensor mode
 * @ov13b10i2c: pointer to ov13b10i2c device
 * @mode: pointer to ov13b10i2c_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 */
static void ov13b10i2c_fill_pad_format(struct ov13b10i2c *ov13b10i2c,
				   const struct ov13b10i2c_mode *mode,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

/**
 * ov13b10i2c_get_pad_format() - Get subdevice pad format
 * @sd: pointer to ov13b10i2c V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ov13b10i2c *ov13b10i2c = to_ov13b10i2c(sd);

	mutex_lock(&ov13b10i2c->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		ov13b10i2c_fill_pad_format(ov13b10i2c, ov13b10i2c->cur_mode, fmt);
	}

	mutex_unlock(&ov13b10i2c->mutex);

	return 0;
}

/**
 * ov13b10i2c_set_pad_format() - Set subdevice pad format
 * @sd: pointer to ov13b10i2c V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ov13b10i2c *ov13b10i2c = to_ov13b10i2c(sd);
	const struct ov13b10i2c_mode *mode;
	int ret = 0;

	mutex_lock(&ov13b10i2c->mutex);

	mode = &supported_mode;
	ov13b10i2c_fill_pad_format(ov13b10i2c, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		ret = ov13b10i2c_update_controls(ov13b10i2c, mode);
		if (!ret)
			ov13b10i2c->cur_mode = mode;
	}

	mutex_unlock(&ov13b10i2c->mutex);

	return ret;
}

/**
 * ov13b10i2c_init_state() - Initialize sub-device state
 * @sd: pointer to ov13b10i2c V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	struct ov13b10i2c *ov13b10i2c = to_ov13b10i2c(sd);
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	ov13b10i2c_fill_pad_format(ov13b10i2c, &supported_mode, &fmt);

	return ov13b10i2c_set_pad_format(sd, sd_state, &fmt);
}

/**
 * ov13b10i2c_start_streaming() - Start sensor stream
 * @ov13b10i2c: pointer to ov13b10i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_start_streaming(struct ov13b10i2c *ov13b10i2c)
{
	const struct ov13b10i2c_reg_list *reg_list;
	int ret;

	/* Write sensor mode registers */
	reg_list = &ov13b10i2c->cur_mode->reg_list;
	ret = ov13b10i2c_write_regs(ov13b10i2c, reg_list->regs,
				reg_list->num_of_regs);
	if (ret) {
		dev_err(ov13b10i2c->dev, "fail to write initial registers\n");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(ov13b10i2c->sd.ctrl_handler);
	if (ret) {
		dev_err(ov13b10i2c->dev, "fail to setup handler\n");
		return ret;
	}

	/* Delay is required before streaming*/
	usleep_range(7400, 8000);

	/* Start streaming */
	ret = ov13b10i2c_write_reg(ov13b10i2c, OV13B10I2C_REG_MODE_SELECT,
			       1, OV13B10I2C_MODE_STREAMING);
	if (ret) {
		dev_err(ov13b10i2c->dev, "fail to start streaming\n");
		return ret;
	}

	return 0;
}

/**
 * ov13b10i2c_stop_streaming() - Stop sensor stream
 * @ov13b10i2c: pointer to ov13b10i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_stop_streaming(struct ov13b10i2c *ov13b10i2c)
{
	return ov13b10i2c_write_reg(ov13b10i2c, OV13B10I2C_REG_MODE_SELECT,
				1, OV13B10I2C_MODE_STANDBY);
}

/**
 * ov13b10i2c_set_stream() - Enable sensor streaming
 * @sd: pointer to ov13b10i2c subdevice
 * @enable: set to enable sensor streaming
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov13b10i2c *ov13b10i2c = to_ov13b10i2c(sd);
	int ret;

	mutex_lock(&ov13b10i2c->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(ov13b10i2c->dev);
		if (ret)
			goto error_unlock;

		ret = ov13b10i2c_start_streaming(ov13b10i2c);
		if (ret)
			goto error_power_off;
	} else {
		ov13b10i2c_stop_streaming(ov13b10i2c);
		pm_runtime_put(ov13b10i2c->dev);
	}

	mutex_unlock(&ov13b10i2c->mutex);

	return 0;

error_power_off:
	pm_runtime_put(ov13b10i2c->dev);
error_unlock:
	mutex_unlock(&ov13b10i2c->mutex);

	return ret;
}

/**
 * ov13b10i2c_detect() - Detect ov13b10i2c sensor
 * @ov13b10i2c: pointer to ov13b10i2c device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int ov13b10i2c_detect(struct ov13b10i2c *ov13b10i2c)
{
	int ret;
	u32 val;

	ret = ov13b10i2c_read_reg(ov13b10i2c, OV13B10I2C_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (val != OV13B10I2C_ID) {
		dev_err(ov13b10i2c->dev, "chip id mismatch: %x!=%x\n",
			OV13B10I2C_ID, val);
		return -ENXIO;
	}

	return 0;
}

/**
 * ov13b10i2c_check_inclk_freq() - Сheck inclk frequency
 * @rate: inclk frequency from clk_get_rate()
 *
 * Return: 0 if successful, -ENODEV if inclk freq does not match
 */
static int ov13b10i2c_check_inclk_freq(const int rate)
{
	for (int i = 0; i < ARRAY_SIZE(ov13b10i2c_inclk_list); i++) {
		if (rate == ov13b10i2c_inclk_list[i])
			return 0;
	}

	return -ENODEV;
}

/**
 * ov13b10i2c_parse_hw_config() - Parse HW configuration and check if supported
 * @ov13b10i2c: pointer to ov13b10i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_parse_hw_config(struct ov13b10i2c *ov13b10i2c)
{
	struct fwnode_handle *fwnode = dev_fwnode(ov13b10i2c->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	if (!fwnode)
		return -ENXIO;

	/* Request optional reset pin */
	ov13b10i2c->reset_gpio = devm_gpiod_get_optional(ov13b10i2c->dev, "reset",
						     GPIOD_OUT_LOW);
	if (IS_ERR(ov13b10i2c->reset_gpio)) {
		dev_err(ov13b10i2c->dev, "failed to get reset gpio %ld\n",
			PTR_ERR(ov13b10i2c->reset_gpio));
		return PTR_ERR(ov13b10i2c->reset_gpio);
	}

	/* Get sensor input clock */
	ov13b10i2c->inclk = devm_clk_get(ov13b10i2c->dev, NULL);
	if (IS_ERR(ov13b10i2c->inclk)) {
		dev_err(ov13b10i2c->dev, "could not get inclk\n");
		return PTR_ERR(ov13b10i2c->inclk);
	}

	ret = ov13b10i2c_check_inclk_freq(clk_get_rate(ov13b10i2c->inclk));
	if (ret) {
		dev_err(ov13b10i2c->dev, "inclk frequency mismatch\n");
		return ret;
	}

	/* Get optional DT defined regulators */
	for (i = 0; i < ARRAY_SIZE(ov13b10i2c_supply_names); i++)
		ov13b10i2c->supplies[i].supply = ov13b10i2c_supply_names[i];

	ret = devm_regulator_bulk_get(ov13b10i2c->dev,
				      ARRAY_SIZE(ov13b10i2c_supply_names),
				      ov13b10i2c->supplies);
	if (ret)
		return ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV13B10I2C_NUM_DATA_LANES) {
		dev_err(ov13b10i2c->dev,
			"number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(ov13b10i2c->dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == OV13B10I2C_LINK_FREQ)
			goto done_endpoint_free;

	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops ov13b10i2c_video_ops = {
	.s_stream = ov13b10i2c_set_stream,
};

static const struct v4l2_subdev_pad_ops ov13b10i2c_pad_ops = {
	.enum_mbus_code = ov13b10i2c_enum_mbus_code,
	.enum_frame_size = ov13b10i2c_enum_frame_size,
	.get_fmt = ov13b10i2c_get_pad_format,
	.set_fmt = ov13b10i2c_set_pad_format,
};

static const struct v4l2_subdev_ops ov13b10i2c_subdev_ops = {
	.video = &ov13b10i2c_video_ops,
	.pad = &ov13b10i2c_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov13b10i2c_internal_ops = {
	.init_state = ov13b10i2c_init_state,
};

/**
 * ov13b10i2c_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov13b10i2c *ov13b10i2c = to_ov13b10i2c(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ov13b10i2c_supply_names),
				    ov13b10i2c->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	gpiod_set_value_cansleep(ov13b10i2c->reset_gpio, 0);

	ret = clk_prepare_enable(ov13b10i2c->inclk);
	if (ret) {
		dev_err(ov13b10i2c->dev, "fail to enable inclk\n");
		goto error_reset;
	}

	/* At least 45000 MCLK cycles */
	usleep_range(10000, 10200);

	return 0;

error_reset:
	gpiod_set_value_cansleep(ov13b10i2c->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ov13b10i2c_supply_names),
			       ov13b10i2c->supplies);

	return ret;
}

/**
 * ov13b10i2c_power_off() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov13b10i2c *ov13b10i2c = to_ov13b10i2c(sd);

	clk_disable_unprepare(ov13b10i2c->inclk);

	gpiod_set_value_cansleep(ov13b10i2c->reset_gpio, 1);

	regulator_bulk_disable(ARRAY_SIZE(ov13b10i2c_supply_names),
			       ov13b10i2c->supplies);

	return 0;
}

/**
 * ov13b10i2c_init_controls() - Initialize sensor subdevice controls
 * @ov13b10i2c: pointer to ov13b10i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_init_controls(struct ov13b10i2c *ov13b10i2c)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr = &ov13b10i2c->ctrl_handler;
	const struct ov13b10i2c_mode *mode = ov13b10i2c->cur_mode;
	u32 lpfr;
	int ret;

	/* set properties from fwnode (e.g. rotation, orientation) */
	ret = v4l2_fwnode_device_parse(ov13b10i2c->dev, &props);
	if (ret)
		return ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &ov13b10i2c->mutex;

	/* Initialize exposure and gain */
	lpfr = mode->vblank + mode->height;

	ov13b10i2c->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					     &ov13b10i2c_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     OV13B10I2C_EXPOSURE_MIN,
					     lpfr - OV13B10I2C_EXPOSURE_OFFSET,
					     OV13B10I2C_EXPOSURE_STEP,
					     OV13B10I2C_EXPOSURE_DEFAULT);

	ov13b10i2c->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					       &ov13b10i2c_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN,
					       OV13B10I2C_AGAIN_MIN,
					       OV13B10I2C_AGAIN_MAX,
					       OV13B10I2C_AGAIN_STEP,
					       OV13B10I2C_AGAIN_DEFAULT);

	v4l2_ctrl_cluster(2, &ov13b10i2c->exp_ctrl);

	ov13b10i2c->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&ov13b10i2c_ctrl_ops,
						V4L2_CID_VBLANK,
						mode->vblank_min,
						mode->vblank_max,
						1, mode->vblank);

	/* Read only controls */
	ov13b10i2c->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					      &ov13b10i2c_ctrl_ops,
					      V4L2_CID_PIXEL_RATE,
					      mode->pclk, mode->pclk,
					      1, mode->pclk);

	ov13b10i2c->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
							&ov13b10i2c_ctrl_ops,
							V4L2_CID_LINK_FREQ,
							ARRAY_SIZE(link_freq) -
							1,
							mode->link_freq_idx,
							link_freq);
	if (ov13b10i2c->link_freq_ctrl)
		ov13b10i2c->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov13b10i2c->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&ov13b10i2c_ctrl_ops,
						V4L2_CID_HBLANK,
						OV13B10I2C_REG_MIN,
						OV13B10I2C_REG_MAX,
						1, mode->hblank);
	if (ov13b10i2c->hblank_ctrl)
		ov13b10i2c->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ov13b10i2c_ctrl_ops, &props);

	if (ctrl_hdlr->error) {
		dev_err(ov13b10i2c->dev, "control init failed: %d\n",
			ctrl_hdlr->error);
		ret = ctrl_hdlr->error;
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	ov13b10i2c->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

/**
 * ov13b10i2c_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int ov13b10i2c_probe(struct i2c_client *client)
{
	struct ov13b10i2c *ov13b10i2c;
	int ret;

	ov13b10i2c = devm_kzalloc(&client->dev, sizeof(*ov13b10i2c), GFP_KERNEL);
	if (!ov13b10i2c)
		return -ENOMEM;

	ov13b10i2c->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&ov13b10i2c->sd, client, &ov13b10i2c_subdev_ops);
	ov13b10i2c->sd.internal_ops = &ov13b10i2c_internal_ops;

	ret = ov13b10i2c_parse_hw_config(ov13b10i2c);
	if (ret)
		return dev_err_probe(ov13b10i2c->dev, ret,
				     "HW configuration is not supported\n");

	mutex_init(&ov13b10i2c->mutex);

	ret = ov13b10i2c_power_on(ov13b10i2c->dev);
	if (ret) {
		dev_err(ov13b10i2c->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = ov13b10i2c_detect(ov13b10i2c);
	if (ret) {
		dev_err(ov13b10i2c->dev, "failed to find sensor: %d\n", ret);
		goto error_power_off;
	}

	/* Set default mode to max resolution */
	ov13b10i2c->cur_mode = &supported_mode;
	ov13b10i2c->vblank = ov13b10i2c->cur_mode->vblank;

	ret = ov13b10i2c_init_controls(ov13b10i2c);
	if (ret) {
		dev_err(ov13b10i2c->dev, "failed to init controls: %d\n", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	ov13b10i2c->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov13b10i2c->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	ov13b10i2c->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ov13b10i2c->sd.entity, 1, &ov13b10i2c->pad);
	if (ret) {
		dev_err(ov13b10i2c->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&ov13b10i2c->sd);
	if (ret < 0) {
		dev_err(ov13b10i2c->dev,
			"failed to register async subdev: %d\n", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(ov13b10i2c->dev);
	pm_runtime_enable(ov13b10i2c->dev);
	pm_runtime_idle(ov13b10i2c->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&ov13b10i2c->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(ov13b10i2c->sd.ctrl_handler);
error_power_off:
	ov13b10i2c_power_off(ov13b10i2c->dev);
error_mutex_destroy:
	mutex_destroy(&ov13b10i2c->mutex);

	return ret;
}

static void ov13b10i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov13b10i2c *ov13b10i2c = to_ov13b10i2c(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ov13b10i2c_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&ov13b10i2c->mutex);
}

static const struct dev_pm_ops ov13b10i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(ov13b10i2c_power_off, ov13b10i2c_power_on, NULL)
};

static const struct of_device_id ov13b10i2c_of_match[] = {
	{ .compatible = "ovti,ov13b10i2c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov13b10i2c_of_match);

static struct i2c_driver ov13b10i2c_driver = {
	.probe = ov13b10i2c_probe,
	.remove = ov13b10i2c_remove,
	.driver = {
		.name = "ov13b10i2c",
		.pm = &ov13b10i2c_pm_ops,
		.of_match_table = ov13b10i2c_of_match,
	},
};

module_i2c_driver(ov13b10i2c_driver);

MODULE_DESCRIPTION("OV OV13B10I2C sensor driver");
MODULE_LICENSE("GPL");
