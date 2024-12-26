// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 Luka Panio <lukapanio@gmail.com>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

struct nt36532 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	struct drm_dsc_config dsc;
	struct regulator_bulk_data supplies[3];
	struct gpio_desc *reset_gpio;
	u8 display_maker;
};

static inline struct nt36532 *
to_nt36532(struct drm_panel *panel)
{
	return container_of(panel, struct nt36532, panel);
}

struct panel_desc {
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct nt36532 *ctx);
};

static void nt36532_reset(struct nt36532 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(12000, 13000);
}


static int nt36532_read_display_maker(struct nt36532 *ctx)
{
	struct mipi_dsi_device *dsi= ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;
	u8 lockdown_info[8];
	ctx->display_maker = 0x42;

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x22);

	ret = mipi_dsi_dcs_read(dsi, 0x0, &lockdown_info, 8);
	if (ret < 0) {
		dev_err(dev, "could not read lockdown info\n");
		return ret;
	}

	ctx->display_maker = lockdown_info[1];
	dev_info(dev, "Got display maker: 0x%02x\n", ctx->display_maker);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x10);
	return 0;
}

static int pipa_init_sequence(struct nt36532 *ctx)
{
	struct mipi_dsi_device *dsi= ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->dsi[0])
		ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x27);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xd0, 0x31);
	mipi_dsi_dcs_write_seq(dsi, 0xd1, 0x20);
	mipi_dsi_dcs_write_seq(dsi, 0xd2, 0x38);
	mipi_dsi_dcs_write_seq(dsi, 0xde, 0x43);
	mipi_dsi_dcs_write_seq(dsi, 0xdf, 0x02);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x23);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_NOP, 0x80);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SOFT_RESET, 0x84);
	mipi_dsi_dcs_write_seq(dsi, 0x05, 0x2d);
	mipi_dsi_dcs_write_seq(dsi, 0x06, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x11, 0x03);
	mipi_dsi_dcs_write_seq(dsi, 0x12, 0x2a);
	mipi_dsi_dcs_write_seq(dsi, 0x15, 0xd0);
	mipi_dsi_dcs_write_seq(dsi, 0x16, 0x16);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_DISPLAY_ON, 0x0a);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_ROWS, 0xff);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_COLUMNS, 0xfe);
	mipi_dsi_dcs_write_seq(dsi, 0x32, 0xfd);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_SCROLL_AREA, 0xfb);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_TEAR_OFF, 0xf8);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_TEAR_ON, 0xf5);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_ADDRESS_MODE, 0xf3);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_SCROLL_START, 0xf2);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_EXIT_IDLE_MODE, 0xf2);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_ENTER_IDLE_MODE, 0xf2);

	ret = mipi_dsi_dcs_set_pixel_format(dsi, 0xef);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, 0x3b, 0xec);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_3D_CONTROL, 0xe9);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_GET_3D_CONTROL, 0xe5);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_VSYNC_TIMING, 0xe5);
	mipi_dsi_dcs_write_seq(dsi, 0x41, 0xe5);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_COLUMN_ADDRESS, 0x13);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_GET_SCANLINE, 0xff);
	mipi_dsi_dcs_write_seq(dsi, 0x46, 0xf4);
	mipi_dsi_dcs_write_seq(dsi, 0x47, 0xe7);
	mipi_dsi_dcs_write_seq(dsi, 0x48, 0xda);
	mipi_dsi_dcs_write_seq(dsi, 0x49, 0xcd);
	mipi_dsi_dcs_write_seq(dsi, 0x4a, 0xc0);
	mipi_dsi_dcs_write_seq(dsi, 0x4b, 0xb3);
	mipi_dsi_dcs_write_seq(dsi, 0x4c, 0xb1);
	mipi_dsi_dcs_write_seq(dsi, 0x4d, 0xb1);
	mipi_dsi_dcs_write_seq(dsi, 0x4e, 0xb1);
	mipi_dsi_dcs_write_seq(dsi, 0x4f, 0x95);
	mipi_dsi_dcs_write_seq(dsi, 0x50, 0x79);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x5c);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_GET_DISPLAY_BRIGHTNESS, 0x58);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x58);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_GET_CONTROL_DISPLAY, 0x58);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PAGE_ADDRESS, 0x0e);
	mipi_dsi_dcs_write_seq(dsi, 0x58, 0xff);
	mipi_dsi_dcs_write_seq(dsi, 0x59, 0xfb);
	mipi_dsi_dcs_write_seq(dsi, 0x5a, 0xf7);
	mipi_dsi_dcs_write_seq(dsi, 0x5b, 0xf3);
	mipi_dsi_dcs_write_seq(dsi, 0x5c, 0xef);
	mipi_dsi_dcs_write_seq(dsi, 0x5d, 0xe3);
	mipi_dsi_dcs_write_seq(dsi, 0x5e, 0xd8);
	mipi_dsi_dcs_write_seq(dsi, 0x5f, 0xd6);
	mipi_dsi_dcs_write_seq(dsi, 0x60, 0xd6);
	mipi_dsi_dcs_write_seq(dsi, 0x61, 0xd6);
	mipi_dsi_dcs_write_seq(dsi, 0x62, 0xc8);
	mipi_dsi_dcs_write_seq(dsi, 0x63, 0xb7);
	mipi_dsi_dcs_write_seq(dsi, 0x64, 0xaa);
	mipi_dsi_dcs_write_seq(dsi, 0x65, 0xa8);
	mipi_dsi_dcs_write_seq(dsi, 0x66, 0xa8);
	mipi_dsi_dcs_write_seq(dsi, 0x67, 0xa8);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x20);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x17, 0x02);
	mipi_dsi_dcs_write_seq(dsi, 0x32, 0x72);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x22);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x9f, 0x57);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x1f, 0x1f, 0x1f, 0x1f);
	mipi_dsi_dcs_write_seq(dsi, 0xb1, 0x4d, 0x4d, 0x4d, 0x4d);
	mipi_dsi_dcs_write_seq(dsi, 0xb2, 0x1e, 0x1e, 0x1e, 0x1e);
	mipi_dsi_dcs_write_seq(dsi, 0xb3, 0x6c, 0x6c, 0x6c, 0x6c);
	mipi_dsi_dcs_write_seq(dsi, 0xb4, 0x1f, 0x1f, 0x1f, 0x1f);
	mipi_dsi_dcs_write_seq(dsi, 0xb5, 0x44, 0x44, 0x44, 0x44);
	mipi_dsi_dcs_write_seq(dsi, 0xb8, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xb9, 0x6e);
	mipi_dsi_dcs_write_seq(dsi, 0xba, 0x6e);
	mipi_dsi_dcs_write_seq(dsi, 0xbb, 0x6e);
	mipi_dsi_dcs_write_seq(dsi, 0xbe, 0x0b);
	mipi_dsi_dcs_write_seq(dsi, 0xbf, 0x6e);
	mipi_dsi_dcs_write_seq(dsi, 0xc1, 0x6e);
	mipi_dsi_dcs_write_seq(dsi, 0xc3, 0x6e);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x23);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xba, 0x7a, 0x5d);
	mipi_dsi_dcs_write_seq(dsi, 0xbb, 0x77, 0x60);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x24);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x1c, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0x92, 0x45, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq(dsi, 0xdb, 0x33);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x25);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_GET_ERROR_COUNT_ON_DSI, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x23, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0x24, 0x16);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_COLUMN_ADDRESS, 0x09);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PAGE_ADDRESS, 0x16);
	mipi_dsi_dcs_write_seq(dsi, 0x42, 0x0b);
	mipi_dsi_dcs_write_seq(dsi, 0xc5, 0x1e);
	mipi_dsi_dcs_write_seq(dsi, 0xf6, 0x02);
	mipi_dsi_dcs_write_seq(dsi, 0xf7, 0x48);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x26);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x04, 0x75);
	mipi_dsi_dcs_write_seq(dsi, 0x19, 0x10, 0x12, 0x12, 0x12);
	mipi_dsi_dcs_write_seq(dsi, 0x1a, 0xc5, 0xa3, 0xa3, 0xa3);
	mipi_dsi_dcs_write_seq(dsi, 0x1b, 0x0f, 0x11, 0x11, 0x11);
	mipi_dsi_dcs_write_seq(dsi, 0x1c, 0xe8, 0xc6, 0xc6, 0xc6);
	mipi_dsi_dcs_write_seq(dsi, 0x1e, 0x45);
	mipi_dsi_dcs_write_seq(dsi, 0x1f, 0x45);
	mipi_dsi_dcs_write_seq(dsi, 0x2a, 0x10, 0x12, 0x12, 0x12);
	mipi_dsi_dcs_write_seq(dsi, 0x2b, 0xc0, 0x9e, 0x9e, 0x9e);
	mipi_dsi_dcs_write_seq(dsi, 0x2f, 0x0b);
	mipi_dsi_dcs_write_seq(dsi, 0x30, 0x45);
	mipi_dsi_dcs_write_seq(dsi, 0x32, 0x45);
	mipi_dsi_dcs_write_seq(dsi, 0x33, 0x22);
	mipi_dsi_dcs_write_seq(dsi, 0x34, 0x92);
	mipi_dsi_dcs_write_seq(dsi, 0x35, 0x78);
	mipi_dsi_dcs_write_seq(dsi, 0x36, 0x96);
	mipi_dsi_dcs_write_seq(dsi, 0x37, 0x78);
	mipi_dsi_dcs_write_seq(dsi, 0x38, 0x06);
	mipi_dsi_dcs_write_seq(dsi, 0x3a, 0x45);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_VSYNC_TIMING, 0x47);
	mipi_dsi_dcs_write_seq(dsi, 0x41, 0x47);
	mipi_dsi_dcs_write_seq(dsi, 0x42, 0x47);
	mipi_dsi_dcs_write_seq(dsi, 0x45, 0x0b);
	mipi_dsi_dcs_write_seq(dsi, 0x46, 0x47);
	mipi_dsi_dcs_write_seq(dsi, 0x48, 0x47);
	mipi_dsi_dcs_write_seq(dsi, 0x4a, 0x47);
	mipi_dsi_dcs_write_seq(dsi, 0x84, 0x16, 0x16, 0x16);
	mipi_dsi_dcs_write_seq(dsi, 0x85, 0x26, 0x26, 0x26);
	mipi_dsi_dcs_write_seq(dsi, 0x8b, 0xaa);
	mipi_dsi_dcs_write_seq(dsi, 0x99, 0x26, 0x26, 0x26, 0x26);
	mipi_dsi_dcs_write_seq(dsi, 0x9a, 0xb6, 0xb6, 0xb6, 0xb6);
	mipi_dsi_dcs_write_seq(dsi, 0x9b, 0x25, 0x25, 0x25, 0x25);
	mipi_dsi_dcs_write_seq(dsi, 0x9c, 0xd4, 0xd4, 0xd4, 0xd4);
	mipi_dsi_dcs_write_seq(dsi, 0x9d, 0x26, 0x26, 0x26, 0x26);
	mipi_dsi_dcs_write_seq(dsi, 0x9e, 0xac, 0xac, 0xac, 0xac);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x27);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x01, 0xc1);
	mipi_dsi_dcs_write_seq(dsi, 0x07, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x08, 0x3c, 0x0d);
	mipi_dsi_dcs_write_seq(dsi, 0x0a, 0xeb, 0x40);
	mipi_dsi_dcs_write_seq(dsi, 0x0c, 0x4b, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x0e, 0xeb, 0x40);
	mipi_dsi_dcs_write_seq(dsi, 0x10, 0x4b, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x14, 0x11);
	mipi_dsi_dcs_write_seq(dsi, 0x76, 0xf0);
	mipi_dsi_dcs_write_seq(dsi, 0x77, 0x02);
	mipi_dsi_dcs_write_seq(dsi, 0x79, 0x29);
	mipi_dsi_dcs_write_seq(dsi, 0x7e, 0x4e);
	mipi_dsi_dcs_write_seq(dsi, 0x80, 0xc9, 0x0c);
	mipi_dsi_dcs_write_seq(dsi, 0x81, 0x3d, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x82, 0xcb, 0xc0);
	mipi_dsi_dcs_write_seq(dsi, 0x84, 0x92, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x85, 0x36, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x86, 0xcb, 0xc0);
	mipi_dsi_dcs_write_seq(dsi, 0x88, 0x92, 0x00);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x2a);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x14, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0x1e, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0x1f, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0xa3, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc5, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0xc6, 0x16);

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x10);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xb3, 0x40);
	mipi_dsi_dcs_write_seq(dsi, 0xb2, 0x91);
	mipi_dsi_dcs_write_seq(dsi, 0x90, 0x03);
	mipi_dsi_dcs_write_seq(dsi, 0x91,
			       0x89, 0x28, 0x00, 0x14, 0xd2, 0x00, 0x01, 0xf4,
			       0x01, 0xab, 0x00, 0x06, 0x05, 0x7a, 0x06, 0x1a);
	mipi_dsi_dcs_write_seq(dsi, 0x92, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq(dsi, 0x35, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x3b, 0x03, 0xd8, 0x1a, 0x0a, 0x0a, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x51, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq(dsi, 0x53, 0x24);
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(70);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	msleep(40);

	return 0;
}

static const struct drm_display_mode nt36532_mode_120 = {
	.clock = (1800 + 200 + 4 + 92) * (2880 + 26 + 2 + 214) * 120 / 1000,
	.hdisplay = 1800,
	.hsync_start = 1800 + 200,
	.hsync_end = 1800 + 200 + 4,
	.htotal = 1800 + 200 + 4 + 92,
	.vdisplay = 2880,
	.vsync_start = 2880 + 26,
	.vsync_end = 2880 + 26 + 2,
	.vtotal = 2880 + 26 + 2 + 214,
	.width_mm = 1480,
	.height_mm = 2367,
	.type = DRM_MODE_TYPE_DRIVER,
};
static const struct panel_desc pipa_desc = {
	.dsi_info = {
		.type = "pipa",
		.channel = 0,
		.node = NULL,
	},
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.init_sequence = pipa_init_sequence,
};

static int nt36532_off(struct nt36532 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(20);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);


	return 0;
}

static int nt36532_prepare(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	msleep(120);

	nt36532_reset(ctx);
	
	msleep(120);
		
	ret = ctx->desc->init_sequence(ctx);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}

	if (!ctx->display_maker) {
		nt36532_read_display_maker(ctx);
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}


	msleep(120);

	return 0;

fail:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	return ret;
}

static int nt36532_enable(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);
	struct drm_dsc_picture_parameter_set pps;
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	usleep_range(10000, 11000);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	print_hex_dump(KERN_INFO, "DSC:", DUMP_PREFIX_NONE, 16,
	       1, (void *)&pps, sizeof(pps), false);

	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	msleep(28);

	return ret;
}

static int nt36532_disable(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	ret = nt36532_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	return ret;
}

static int nt36532_unprepare(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	return 0;
}

static int nt36532_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	const struct drm_display_mode *mode;
	mode = &nt36532_mode_120;
	return drm_connector_helper_get_modes_fixed(connector, mode);
}

static const struct drm_panel_funcs nt36532_panel_funcs = {
	.prepare = nt36532_prepare,
	.enable = nt36532_enable,
	.disable = nt36532_disable,
	.unprepare = nt36532_unprepare,
	.get_modes = nt36532_get_modes,
};

static int nt36532_probe(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_host *dsi_sec_host;
	struct nt36532 *ctx;
	struct device *dev = &dsi->dev;
	struct device_node *dsi_sec;
	int ret, i;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supplies[0].supply = "vddio";
	ctx->supplies[1].supply = "dvddbuck";
	ctx->supplies[2].supply = "dvddldo";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	dsi_sec = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);

	if (dsi_sec) {
		const struct mipi_dsi_device_info info = { "nt36532", 0,
							   dsi_sec };

		dev_notice(dev, "Using Dual-DSI with OF node `%s`\n", dsi_sec->name);

		dsi_sec_host = of_find_mipi_dsi_host_by_node(dsi_sec);
		of_node_put(dsi_sec);
		if (!dsi_sec_host) {
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "Cannot get secondary DSI host\n");
		}

		ctx->dsi[1] =
			devm_mipi_dsi_device_register_full(dev, dsi_sec_host, &info);
		if (IS_ERR(ctx->dsi[1])) {
			return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
					     "Cannot get secondary DSI node\n");
		}

		mipi_dsi_set_drvdata(ctx->dsi[1], ctx);
	} else {
		dev_notice(dev, "Using Single-DSI\n");
	}

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	drm_panel_init(&ctx->panel, dev, &nt36532_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	ctx->dsc.slice_height = 20;
	/* Downstream sets this while parsing DT */
	ctx->dsc.slice_count = 2;

	ctx->dsc.slice_width = 450;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;

		/* This panel only supports DSC; unconditionally enable it */
		ctx->dsi[i]->dsc = &ctx->dsc;
		ctx->dsi[i]->dsc_slice_per_pkt = 2;

		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS;

		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret < 0) {
			drm_panel_remove(&ctx->panel);
			return dev_err_probe(dev, ret,
					     "Failed to attach to DSI%d\n", i);
		}
	}

	return 0;
}

static void nt36532_remove(struct mipi_dsi_device *dsi)
{
	struct nt36532 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt36532_of_match[] = {
	{
		.compatible = "xiaomi,pipa-nt36532",
		.data = &pipa_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, nt36532_of_match);

static struct mipi_dsi_driver nt36532_driver = {
	.probe = nt36532_probe,
	.remove = nt36532_remove,
	.driver = {
		.name = "panel-nt36532",
		.of_match_table = nt36532_of_match,
	},
};
module_mipi_dsi_driver(nt36532_driver);

MODULE_AUTHOR("Luka Panio <luakapnio@gmail.com>");
MODULE_DESCRIPTION("DRM panel driver for the Novatek NT36532 Driver-IC");
MODULE_LICENSE("GPL");
