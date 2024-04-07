// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct rm69380_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline
struct rm69380_panel *to_rm69380_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rm69380_panel, panel);
}

static void rm69380_reset(struct rm69380_panel *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(15000, 16000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(30);
}

static int rm69380_on(struct rm69380_panel *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq(dsi, 0xfe, 0xd4);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0xfe, 0xd0);
	mipi_dsi_dcs_write_seq(dsi, 0x48, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xfe, 0x26);
	mipi_dsi_dcs_write_seq(dsi, 0x75, 0x3f);
	mipi_dsi_dcs_write_seq(dsi, 0x1d, 0x1a);
	mipi_dsi_dcs_write_seq(dsi, 0xfe, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x53, 0x28);
	mipi_dsi_dcs_write_seq(dsi, 0xc2, 0x08);
	mipi_dsi_dcs_write_seq(dsi, 0x35, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x51, 0x07, 0xff);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(20);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	msleep(36);

	return 0;
}

static int rm69380_off(struct rm69380_panel *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(35);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(20);

	return 0;
}

static int rm69380_prepare(struct drm_panel *panel)
{
	struct rm69380_panel *ctx = to_rm69380_panel(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	rm69380_reset(ctx);

	ret = rm69380_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int rm69380_unprepare(struct drm_panel *panel)
{
	struct rm69380_panel *ctx = to_rm69380_panel(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = rm69380_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode rm69380_mode = {
	.clock = (2560 + 32 + 12 + 38) * (1600 + 20 + 4 + 8) * 90 / 1000,
	.hdisplay = 2560,
	.hsync_start = 2560 + 32,
	.hsync_end = 2560 + 32 + 12,
	.htotal = 2560 + 32 + 12 + 38,
	.vdisplay = 1600,
	.vsync_start = 1600 + 20,
	.vsync_end = 1600 + 20 + 4,
	.vtotal = 1600 + 20 + 4 + 8,
	.width_mm = 0,
	.height_mm = 0,
};

static int rm69380_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &rm69380_panel_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs rm69380_panel_funcs = {
	.prepare = rm69380_prepare,
	.unprepare = rm69380_unprepare,
	.get_modes = rm69380_get_modes,
};

static int rm69380_probe(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_host *dsi_sec_host;
	struct rm69380_panel *ctx;
	struct device *dev = &dsi->dev;
	struct device_node *dsi_sec;
	int ret, i;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supplies[0].supply = "vddio";
	ctx->supplies[1].supply = "avdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	dsi_sec = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);

	if (dsi_sec) {
		dev_notice(dev, "Using Dual-DSI\n");

		const struct mipi_dsi_device_info info = { "RM69380", 0,
							   dsi_sec };

		dev_notice(dev, "Found second DSI `%s`\n", dsi_sec->name);

		dsi_sec_host = of_find_mipi_dsi_host_by_node(dsi_sec);
		of_node_put(dsi_sec);
		if (!dsi_sec_host) {
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "Cannot get secondary DSI host\n");
		}

		ctx->dsi[1] =
			mipi_dsi_device_register_full(dsi_sec_host, &info);
		if (IS_ERR(ctx->dsi[1])) {
			return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
					     "Cannot get secondary DSI node\n");
		}

		dev_notice(dev, "Second DSI name `%s`\n", ctx->dsi[1]->name);
		mipi_dsi_set_drvdata(ctx->dsi[1], ctx);
	} else {
		dev_notice(dev, "Using Single-DSI\n");
	}

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	drm_panel_init(&ctx->panel, dev, &rm69380_panel_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	drm_panel_add(&ctx->panel);

	for (i = 0; i <ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;

		dev_notice(&ctx->dsi[i]->dev, "Binding DSI %d\n", i);

		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
					  MIPI_DSI_CLOCK_NON_CONTINUOUS;

		// TODO: devm_mipi_dsi_attach?
		ret = mipi_dsi_attach(ctx->dsi[i]);
		if (ret < 0) {
			drm_panel_remove(&ctx->panel);
			return dev_err_probe(dev, ret,
					     "Failed to attach to DSI%d\n", i);
		}
	}

	return 0;
}

static void rm69380_remove(struct mipi_dsi_device *dsi)
{
	struct rm69380_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id rm69380_panel_of_match[] = {
	{ .compatible = "lenovo,rm69380-edo" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rm69380_panel_of_match);

static struct mipi_dsi_driver rm69380_panel_driver = {
	.probe = rm69380_panel_probe,
	.remove = rm69380_panel_remove,
	.driver = {
		.name = "panel-rm69380-edo-amoled",
		.of_match_table = rm69380_panel_of_match,
	},
};
module_mipi_dsi_driver(rm69380_panel_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for Raydium RM69380-equipped DSI panels");
MODULE_LICENSE("GPL");
