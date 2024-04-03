// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36532 DriverIC panels driver
 *
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define DSI_NUM_MIN 1

struct panel_info {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	enum drm_panel_orientation orientation;

	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;
	struct regulator *vddio;
};

struct panel_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int bpc;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct panel_info *pinfo);

	bool is_dual_dsi;

	struct drm_dsc_config dsc;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int pipa_csot_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi0 = pinfo->dsi[0];
	struct device *dev = &dsi0->dev;
	int ret;

	ret = mipi_dsi_compression_mode(dsi0, true);
	if (ret < 0) {
		dev_err(dev, "Failed to set compression mode: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x27);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xd0, 0x31);
	mipi_dsi_dcs_write_seq(dsi0, 0xd1, 0x20);
	mipi_dsi_dcs_write_seq(dsi0, 0xd2, 0x38);
	mipi_dsi_dcs_write_seq(dsi0, 0xde, 0x43);
	mipi_dsi_dcs_write_seq(dsi0, 0xdf, 0x02);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x23);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x00, 0x80);
	mipi_dsi_dcs_write_seq(dsi0, 0x01, 0x84);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0x2d);
	mipi_dsi_dcs_write_seq(dsi0, 0x06, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x11, 0x03);
	mipi_dsi_dcs_write_seq(dsi0, 0x12, 0x2a);
	mipi_dsi_dcs_write_seq(dsi0, 0x15, 0xd0);
	mipi_dsi_dcs_write_seq(dsi0, 0x16, 0x16);
	mipi_dsi_dcs_write_seq(dsi0, 0x29, 0x0a);
	mipi_dsi_dcs_write_seq(dsi0, 0x30, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x31, 0xfe);
	mipi_dsi_dcs_write_seq(dsi0, 0x32, 0xfd);
	mipi_dsi_dcs_write_seq(dsi0, 0x33, 0xfb);
	mipi_dsi_dcs_write_seq(dsi0, 0x34, 0xf8);
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0xf5);
	mipi_dsi_dcs_write_seq(dsi0, 0x36, 0xf3);
	mipi_dsi_dcs_write_seq(dsi0, 0x37, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x38, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x39, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x3a, 0xef);
	mipi_dsi_dcs_write_seq(dsi0, 0x3b, 0xec);
	mipi_dsi_dcs_write_seq(dsi0, 0x3d, 0xe9);
	mipi_dsi_dcs_write_seq(dsi0, 0x3f, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x40, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x41, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x2a, 0x13);
	mipi_dsi_dcs_write_seq(dsi0, 0x45, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x46, 0xf4);
	mipi_dsi_dcs_write_seq(dsi0, 0x47, 0xe7);
	mipi_dsi_dcs_write_seq(dsi0, 0x48, 0xda);
	mipi_dsi_dcs_write_seq(dsi0, 0x49, 0xcd);
	mipi_dsi_dcs_write_seq(dsi0, 0x4a, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x4b, 0xb3);
	mipi_dsi_dcs_write_seq(dsi0, 0x4c, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4d, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4e, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4f, 0x95);
	mipi_dsi_dcs_write_seq(dsi0, 0x50, 0x79);
	mipi_dsi_dcs_write_seq(dsi0, 0x51, 0x5c);
	mipi_dsi_dcs_write_seq(dsi0, 0x52, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x53, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x54, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x2b, 0x0e);
	mipi_dsi_dcs_write_seq(dsi0, 0x58, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x59, 0xfb);
	mipi_dsi_dcs_write_seq(dsi0, 0x5a, 0xf7);
	mipi_dsi_dcs_write_seq(dsi0, 0x5b, 0xf3);
	mipi_dsi_dcs_write_seq(dsi0, 0x5c, 0xef);
	mipi_dsi_dcs_write_seq(dsi0, 0x5d, 0xe3);
	mipi_dsi_dcs_write_seq(dsi0, 0x5e, 0xd8);
	mipi_dsi_dcs_write_seq(dsi0, 0x5f, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x60, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x61, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x62, 0xc8);
	mipi_dsi_dcs_write_seq(dsi0, 0x63, 0xb7);
	mipi_dsi_dcs_write_seq(dsi0, 0x64, 0xaa);
	mipi_dsi_dcs_write_seq(dsi0, 0x65, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0x66, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0x67, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x20);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x17, 0x02);
	mipi_dsi_dcs_write_seq(dsi0, 0x32, 0x72);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x22);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x9f, 0x57);
	mipi_dsi_dcs_write_seq(dsi0, 0xb0, 0x1f, 0x1f, 0x1f, 0x1f);
	mipi_dsi_dcs_write_seq(dsi0, 0xb1, 0x4d, 0x4d, 0x4d, 0x4d);
	mipi_dsi_dcs_write_seq(dsi0, 0xb2, 0x1e, 0x1e, 0x1e, 0x1e);
	mipi_dsi_dcs_write_seq(dsi0, 0xb3, 0x6c, 0x6c, 0x6c, 0x6c);
	mipi_dsi_dcs_write_seq(dsi0, 0xb4, 0x1f, 0x1f, 0x1f, 0x1f);
	mipi_dsi_dcs_write_seq(dsi0, 0xb5, 0x44, 0x44, 0x44, 0x44);
	mipi_dsi_dcs_write_seq(dsi0, 0xb8, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0xb9, 0x6e);
	mipi_dsi_dcs_write_seq(dsi0, 0xba, 0x6e);
	mipi_dsi_dcs_write_seq(dsi0, 0xbb, 0x6e);
	mipi_dsi_dcs_write_seq(dsi0, 0xbe, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xbf, 0x6e);
	mipi_dsi_dcs_write_seq(dsi0, 0xc1, 0x6e);
	mipi_dsi_dcs_write_seq(dsi0, 0xc3, 0x6e);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x23);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xba, 0x7a, 0x5d);
	mipi_dsi_dcs_write_seq(dsi0, 0xbb, 0x77, 0x60);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x24);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x1c, 0x80);
	mipi_dsi_dcs_write_seq(dsi0, 0x92, 0x45, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0xdb, 0x33);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x25);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x23, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0x24, 0x16);
	mipi_dsi_dcs_write_seq(dsi0, 0x2a, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0x2b, 0x16);
	mipi_dsi_dcs_write_seq(dsi0, 0x42, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xc5, 0x1e);
	mipi_dsi_dcs_write_seq(dsi0, 0xf6, 0x02);
	mipi_dsi_dcs_write_seq(dsi0, 0xf7, 0x48);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x04, 0x75);
	mipi_dsi_dcs_write_seq(dsi0, 0x19, 0x10, 0x12, 0x12, 0x12);
	mipi_dsi_dcs_write_seq(dsi0, 0x1a, 0xc5, 0xa3, 0xa3, 0xa3);
	mipi_dsi_dcs_write_seq(dsi0, 0x1b, 0x0f, 0x11, 0x11, 0x11);
	mipi_dsi_dcs_write_seq(dsi0, 0x1c, 0xe8, 0xc6, 0xc6, 0xc6);
	mipi_dsi_dcs_write_seq(dsi0, 0x1e, 0x45);
	mipi_dsi_dcs_write_seq(dsi0, 0x1f, 0x45);
	mipi_dsi_dcs_write_seq(dsi0, 0x2a, 0x10, 0x12, 0x12, 0x12);
	mipi_dsi_dcs_write_seq(dsi0, 0x2b, 0xc0, 0x9e, 0x9e, 0x9e);
	mipi_dsi_dcs_write_seq(dsi0, 0x2f, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0x30, 0x45);
	mipi_dsi_dcs_write_seq(dsi0, 0x32, 0x45);
	mipi_dsi_dcs_write_seq(dsi0, 0x33, 0x22);
	mipi_dsi_dcs_write_seq(dsi0, 0x34, 0x92);
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0x78);
	mipi_dsi_dcs_write_seq(dsi0, 0x36, 0x96);
	mipi_dsi_dcs_write_seq(dsi0, 0x37, 0x78);
	mipi_dsi_dcs_write_seq(dsi0, 0x38, 0x06);
	mipi_dsi_dcs_write_seq(dsi0, 0x3a, 0x45);
	mipi_dsi_dcs_write_seq(dsi0, 0x40, 0x47);
	mipi_dsi_dcs_write_seq(dsi0, 0x41, 0x47);
	mipi_dsi_dcs_write_seq(dsi0, 0x42, 0x47);
	mipi_dsi_dcs_write_seq(dsi0, 0x45, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0x46, 0x47);
	mipi_dsi_dcs_write_seq(dsi0, 0x48, 0x47);
	mipi_dsi_dcs_write_seq(dsi0, 0x4a, 0x47);
	mipi_dsi_dcs_write_seq(dsi0, 0x84, 0x16, 0x16, 0x16);
	mipi_dsi_dcs_write_seq(dsi0, 0x85, 0x26, 0x26, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x8b, 0xaa);
	mipi_dsi_dcs_write_seq(dsi0, 0x99, 0x26, 0x26, 0x26, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x9a, 0xb6, 0xb6, 0xb6, 0xb6);
	mipi_dsi_dcs_write_seq(dsi0, 0x9b, 0x25, 0x25, 0x25, 0x25);
	mipi_dsi_dcs_write_seq(dsi0, 0x9c, 0xd4, 0xd4, 0xd4, 0xd4);
	mipi_dsi_dcs_write_seq(dsi0, 0x9d, 0x26, 0x26, 0x26, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x9e, 0xac, 0xac, 0xac, 0xac);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x27);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x01, 0xc1);
	mipi_dsi_dcs_write_seq(dsi0, 0x07, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x08, 0x3c, 0x0d);
	mipi_dsi_dcs_write_seq(dsi0, 0x0a, 0xeb, 0x40);
	mipi_dsi_dcs_write_seq(dsi0, 0x0c, 0x4b, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x0e, 0xeb, 0x40);
	mipi_dsi_dcs_write_seq(dsi0, 0x10, 0x4b, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x14, 0x11);
	mipi_dsi_dcs_write_seq(dsi0, 0x76, 0xf0);
	mipi_dsi_dcs_write_seq(dsi0, 0x77, 0x02);
	mipi_dsi_dcs_write_seq(dsi0, 0x79, 0x29);
	mipi_dsi_dcs_write_seq(dsi0, 0x7e, 0x4e);
	mipi_dsi_dcs_write_seq(dsi0, 0x80, 0xc9, 0x0c);
	mipi_dsi_dcs_write_seq(dsi0, 0x81, 0x3d, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x82, 0xcb, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x84, 0x92, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x85, 0x36, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x86, 0xcb, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x88, 0x92, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x2a);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x14, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0x1e, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0x1f, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0xa3, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0xc5, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0xc6, 0x16);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x10);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xb3, 0x40);
	mipi_dsi_dcs_write_seq(dsi0, 0xb2, 0x91);
	mipi_dsi_dcs_write_seq(dsi0, 0x90, 0x03);
	mipi_dsi_dcs_write_seq(dsi0, 0x91,
			       0x89, 0x28, 0x00, 0x14, 0xd2, 0x00, 0x01, 0xf4,
			       0x01, 0xab, 0x00, 0x06, 0x05, 0x7a, 0x06, 0x1a);
	mipi_dsi_dcs_write_seq(dsi0, 0x92, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x3b, 0x03, 0xd8, 0x1a, 0x0a, 0x0a, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x51, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x53, 0x24);
	mipi_dsi_dcs_write_seq(dsi0, 0x11);
	msleep(70);
	mipi_dsi_dcs_write_seq(dsi0, 0x29);

	return 0;
}

static int pipa_tianma_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi0 = pinfo->dsi[0];
	struct device *dev = &dsi0->dev;
	int ret;

	ret = mipi_dsi_compression_mode(dsi0, true);
	if (ret < 0) {
		dev_err(dev, "Failed to set compression mode: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x27);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xd0, 0x31);
	mipi_dsi_dcs_write_seq(dsi0, 0xd1, 0x20);
	mipi_dsi_dcs_write_seq(dsi0, 0xd2, 0x38);
	mipi_dsi_dcs_write_seq(dsi0, 0xde, 0x43);
	mipi_dsi_dcs_write_seq(dsi0, 0xdf, 0x02);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x23);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x00, 0x80);
	mipi_dsi_dcs_write_seq(dsi0, 0x01, 0x84);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0x2d);
	mipi_dsi_dcs_write_seq(dsi0, 0x06, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x11, 0x03);
	mipi_dsi_dcs_write_seq(dsi0, 0x12, 0x2a);
	mipi_dsi_dcs_write_seq(dsi0, 0x15, 0xd0);
	mipi_dsi_dcs_write_seq(dsi0, 0x16, 0x16);
	mipi_dsi_dcs_write_seq(dsi0, 0x29, 0x0a);
	mipi_dsi_dcs_write_seq(dsi0, 0x30, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x31, 0xfe);
	mipi_dsi_dcs_write_seq(dsi0, 0x32, 0xfd);
	mipi_dsi_dcs_write_seq(dsi0, 0x33, 0xfb);
	mipi_dsi_dcs_write_seq(dsi0, 0x34, 0xf8);
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0xf5);
	mipi_dsi_dcs_write_seq(dsi0, 0x36, 0xf3);
	mipi_dsi_dcs_write_seq(dsi0, 0x37, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x38, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x39, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x3a, 0xef);
	mipi_dsi_dcs_write_seq(dsi0, 0x3b, 0xec);
	mipi_dsi_dcs_write_seq(dsi0, 0x3d, 0xe9);
	mipi_dsi_dcs_write_seq(dsi0, 0x3f, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x40, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x41, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x2a, 0x13);
	mipi_dsi_dcs_write_seq(dsi0, 0x45, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x46, 0xf4);
	mipi_dsi_dcs_write_seq(dsi0, 0x47, 0xe7);
	mipi_dsi_dcs_write_seq(dsi0, 0x48, 0xda);
	mipi_dsi_dcs_write_seq(dsi0, 0x49, 0xcd);
	mipi_dsi_dcs_write_seq(dsi0, 0x4a, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x4b, 0xb3);
	mipi_dsi_dcs_write_seq(dsi0, 0x4c, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4d, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4e, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4f, 0x95);
	mipi_dsi_dcs_write_seq(dsi0, 0x50, 0x79);
	mipi_dsi_dcs_write_seq(dsi0, 0x51, 0x5c);
	mipi_dsi_dcs_write_seq(dsi0, 0x52, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x53, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x54, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x2b, 0x0e);
	mipi_dsi_dcs_write_seq(dsi0, 0x58, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x59, 0xfb);
	mipi_dsi_dcs_write_seq(dsi0, 0x5a, 0xf7);
	mipi_dsi_dcs_write_seq(dsi0, 0x5b, 0xf3);
	mipi_dsi_dcs_write_seq(dsi0, 0x5c, 0xef);
	mipi_dsi_dcs_write_seq(dsi0, 0x5d, 0xe3);
	mipi_dsi_dcs_write_seq(dsi0, 0x5e, 0xd8);
	mipi_dsi_dcs_write_seq(dsi0, 0x5f, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x60, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x61, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x62, 0xc8);
	mipi_dsi_dcs_write_seq(dsi0, 0x63, 0xb7);
	mipi_dsi_dcs_write_seq(dsi0, 0x64, 0xaa);
	mipi_dsi_dcs_write_seq(dsi0, 0x65, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0x66, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0x67, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x20);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x65, 0xcc);
	mipi_dsi_dcs_write_seq(dsi0, 0x6d, 0xcc);
	mipi_dsi_dcs_write_seq(dsi0, 0x31, 0x02);
	mipi_dsi_dcs_write_seq(dsi0, 0x32, 0x72);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0x07, 0x37);
	mipi_dsi_dcs_write_seq(dsi0, 0x08, 0x19);
	mipi_dsi_dcs_write_seq(dsi0, 0x0f, 0x6e);
	mipi_dsi_dcs_write_seq(dsi0, 0x9d, 0x0a);
	mipi_dsi_dcs_write_seq(dsi0, 0x9e, 0x0a);
	mipi_dsi_dcs_write_seq(dsi0, 0x17, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x22);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x9f, 0x57);
	mipi_dsi_dcs_write_seq(dsi0, 0xb0, 0x1d, 0x1d, 0x1d, 0x1d);
	mipi_dsi_dcs_write_seq(dsi0, 0xb1, 0xce, 0xce, 0xce, 0xce);
	mipi_dsi_dcs_write_seq(dsi0, 0xb2, 0x1c, 0x1c, 0x1c, 0x1c);
	mipi_dsi_dcs_write_seq(dsi0, 0xb3, 0xee, 0xee, 0xee, 0xee);
	mipi_dsi_dcs_write_seq(dsi0, 0xb4, 0x1d, 0x1d, 0x1d, 0x1d);
	mipi_dsi_dcs_write_seq(dsi0, 0xb5, 0xc6, 0xc6, 0xc6, 0xc6);
	mipi_dsi_dcs_write_seq(dsi0, 0xb8, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0xf8, 0xcc);
	mipi_dsi_dcs_write_seq(dsi0, 0xf9, 0xcc);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x23);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xba, 0x68, 0x5d);
	mipi_dsi_dcs_write_seq(dsi0, 0xbb, 0x65, 0x60);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x24);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x1c, 0x80);
	mipi_dsi_dcs_write_seq(dsi0, 0x92, 0x47, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x94, 0xd8, 0x00, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x95, 0x09);
	mipi_dsi_dcs_write_seq(dsi0, 0x98, 0x80);
	mipi_dsi_dcs_write_seq(dsi0, 0xc3, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0xc7, 0x07);
	mipi_dsi_dcs_write_seq(dsi0, 0xda, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xdb, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xdf, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xe1, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xe3, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xe5, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xe9, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xeb, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xef, 0x0b);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x25);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x14, 0x80);
	mipi_dsi_dcs_write_seq(dsi0, 0x16, 0x40);
	mipi_dsi_dcs_write_seq(dsi0, 0x20, 0x3b);
	mipi_dsi_dcs_write_seq(dsi0, 0x23, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0x24, 0x18);
	mipi_dsi_dcs_write_seq(dsi0, 0x26, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0x27, 0x3d);
	mipi_dsi_dcs_write_seq(dsi0, 0x2a, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0x2b, 0x18);
	mipi_dsi_dcs_write_seq(dsi0, 0x42, 0x05);
	mipi_dsi_dcs_write_seq(dsi0, 0x4c, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0x65, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0xc5, 0x1e);
	mipi_dsi_dcs_write_seq(dsi0, 0xdc, 0xb6);
	mipi_dsi_dcs_write_seq(dsi0, 0xde, 0x6d);
	mipi_dsi_dcs_write_seq(dsi0, 0xf7, 0x48);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x04, 0x75);
	mipi_dsi_dcs_write_seq(dsi0, 0x13, 0xcc);
	mipi_dsi_dcs_write_seq(dsi0, 0x14, 0xdf);
	mipi_dsi_dcs_write_seq(dsi0, 0x19, 0x11, 0x12, 0x12, 0x12);
	mipi_dsi_dcs_write_seq(dsi0, 0x1a, 0x0f, 0xc7, 0xc7, 0xc7);
	mipi_dsi_dcs_write_seq(dsi0, 0x1b, 0x10, 0x11, 0x11, 0x11);
	mipi_dsi_dcs_write_seq(dsi0, 0x1c, 0x32, 0xea, 0xea, 0xea);
	mipi_dsi_dcs_write_seq(dsi0, 0x2a, 0x11, 0x12, 0x12, 0x12);
	mipi_dsi_dcs_write_seq(dsi0, 0x2b, 0x0a, 0xc2, 0xc2, 0xc2);
	mipi_dsi_dcs_write_seq(dsi0, 0x33, 0x91);
	mipi_dsi_dcs_write_seq(dsi0, 0x34, 0x78);
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0x16);
	mipi_dsi_dcs_write_seq(dsi0, 0x80, 0xcc);
	mipi_dsi_dcs_write_seq(dsi0, 0x81, 0xcc);
	mipi_dsi_dcs_write_seq(dsi0, 0x8b, 0x8c);
	mipi_dsi_dcs_write_seq(dsi0, 0x97, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x99, 0x1d, 0x1d, 0x1d, 0x1d);
	mipi_dsi_dcs_write_seq(dsi0, 0x9a, 0x66, 0x66, 0x66, 0x66);
	mipi_dsi_dcs_write_seq(dsi0, 0x9b, 0x1c, 0x1c, 0x1c, 0x1c);
	mipi_dsi_dcs_write_seq(dsi0, 0x9c, 0x86, 0x86, 0x86, 0x86);
	mipi_dsi_dcs_write_seq(dsi0, 0x9d, 0x1d, 0x1d, 0x1d, 0x1d);
	mipi_dsi_dcs_write_seq(dsi0, 0x9e, 0x5e, 0x5e, 0x5e, 0x5e);
	mipi_dsi_dcs_write_seq(dsi0, 0xcd, 0x5d, 0x68);
	mipi_dsi_dcs_write_seq(dsi0, 0xce, 0x5a, 0x6a);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x27);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x00, 0x81);
	mipi_dsi_dcs_write_seq(dsi0, 0x01, 0xba);
	mipi_dsi_dcs_write_seq(dsi0, 0x03, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0x04);
	mipi_dsi_dcs_write_seq(dsi0, 0x06, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0x07, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x08, 0xf4, 0x0c);
	mipi_dsi_dcs_write_seq(dsi0, 0x09, 0x29, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x0a, 0xe4, 0x40);
	mipi_dsi_dcs_write_seq(dsi0, 0x0b, 0x0c, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x0c, 0x46, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x0d, 0x24, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x0e, 0xe4, 0x40);
	mipi_dsi_dcs_write_seq(dsi0, 0x0f, 0x0c, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x10, 0x46, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x13, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x14, 0x33);
	mipi_dsi_dcs_write_seq(dsi0, 0x5b, 0xc7);
	mipi_dsi_dcs_write_seq(dsi0, 0x76, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x78, 0x83);
	mipi_dsi_dcs_write_seq(dsi0, 0x79, 0x23);
	mipi_dsi_dcs_write_seq(dsi0, 0x7b, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x7d, 0x04);
	mipi_dsi_dcs_write_seq(dsi0, 0x7e, 0x4d);
	mipi_dsi_dcs_write_seq(dsi0, 0x7f, 0x04);
	mipi_dsi_dcs_write_seq(dsi0, 0x80, 0x40, 0x0c);
	mipi_dsi_dcs_write_seq(dsi0, 0x81, 0x3c, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x82, 0xc4, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x83, 0x0c, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x84, 0x6b, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x85, 0x35, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x86, 0xc4, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x87, 0x0c, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x88, 0x6b, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x2a);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x33, 0x70);
	mipi_dsi_dcs_write_seq(dsi0, 0x64, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x6a, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x79, 0xae);
	mipi_dsi_dcs_write_seq(dsi0, 0x7c, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x7f, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x82, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0x85, 0x2e);
	mipi_dsi_dcs_write_seq(dsi0, 0x91, 0x2e);
	mipi_dsi_dcs_write_seq(dsi0, 0xc5, 0x08);
	mipi_dsi_dcs_write_seq(dsi0, 0xc6, 0x18);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0xd0);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x09, 0x77);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0xe0);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x14, 0x60);
	mipi_dsi_dcs_write_seq(dsi0, 0x16, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x10);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xb3, 0x40);
	mipi_dsi_dcs_write_seq(dsi0, 0xb2, 0x91);
	mipi_dsi_dcs_write_seq(dsi0, 0x90, 0x03);
	mipi_dsi_dcs_write_seq(dsi0, 0x91,
			       0x89, 0x28, 0x00, 0x14, 0xd2, 0x00, 0x01, 0xf4,
			       0x01, 0xab, 0x00, 0x06, 0x05, 0x7a, 0x06, 0x1a);
	mipi_dsi_dcs_write_seq(dsi0, 0x92, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x3b, 0x03, 0xd8, 0x1a, 0x0a, 0x0a, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x51, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x53, 0x24);
	mipi_dsi_dcs_write_seq(dsi0, 0x11);
	msleep(120);
	mipi_dsi_dcs_write_seq(dsi0, 0x29);

	return 0;
}

// CSOT and TIANMA shares the same timings
static const struct drm_display_mode pipa_modes[] = {
	{
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
	},
};

static const struct panel_desc pipa_csot_desc = {
	.modes = pipa_modes,
	.num_modes = ARRAY_SIZE(pipa_modes),
	.dsi_info = {
		.type = "CSOT-pipa",
		.channel = 0,
		.node = NULL,
	},
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = pipa_csot_init_sequence,
	.is_dual_dsi = true,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 20,
		.slice_width = 450,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
		.pic_width = 900,
		.pic_height = 2880,
	},
};

static const struct panel_desc pipa_tianma_desc = {
	.modes = pipa_modes,
	.num_modes = ARRAY_SIZE(pipa_modes),
	.dsi_info = {
		.type = "TIANMA-pipa",
		.channel = 0,
		.node = NULL,
	},
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = pipa_tianma_init_sequence,
	.is_dual_dsi = true,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 20,
		.slice_width = 450,
		.slice_count = 4,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
		.pic_width = 1800,
		.pic_height = 2880,
	},
};

static void nt36532_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(12000, 13000);
}

static int nt36532_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	// ret = regulator_enable(pinfo->vddio);
	// if (ret) {
	// 	dev_err(panel->dev, "failed to enable vddio regulator: %d\n", ret);
	// 	return ret;
	// }

	nt36532_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		// regulator_disable(pinfo->vddio);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);

	ret = mipi_dsi_picture_parameter_set(pinfo->dsi[0], &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	return 0;
}

static int nt36532_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i, ret;

	ret = mipi_dsi_dcs_set_display_off(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&pinfo->dsi[i]->dev, "failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&pinfo->dsi[i]->dev, "failed to enter sleep mode: %d\n", ret);

	msleep(70);

	return 0;
}

static int nt36532_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	// gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	// regulator_disable(pinfo->vddio);

	return 0;
}

static void nt36532_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI0 host: %d\n", ret);

	if (pinfo->desc->is_dual_dsi) {
		ret = mipi_dsi_detach(pinfo->dsi[1]);
		if (ret < 0)
			dev_err(&pinfo->dsi[1]->dev, "failed to detach from DSI1 host: %d\n", ret);
		mipi_dsi_device_unregister(pinfo->dsi[1]);
	}

	drm_panel_remove(&pinfo->panel);
}

static int nt36532_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i;

	for (i = 0; i < pinfo->desc->num_modes; i++) {
		const struct drm_display_mode *m = &pinfo->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
			return -ENOMEM;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;

	return pinfo->desc->num_modes;
}

static enum drm_panel_orientation nt36532_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

static const struct drm_panel_funcs nt36532_panel_funcs = {
	.disable = nt36532_disable,
	.prepare = nt36532_prepare,
	.unprepare = nt36532_unprepare,
	.get_modes = nt36532_get_modes,
	.get_orientation = nt36532_get_orientation,
};

static int nt36532_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int nt36532_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops nt36532_bl_ops = {
	.update_status = nt36532_bl_update_status,
	.get_brightness = nt36532_bl_get_brightness,
};

static struct backlight_device *nt36532_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 512,
		.max_brightness = 4095,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &nt36532_bl_ops, &props);
}

static int nt36532_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct panel_info *pinfo;
	const struct mipi_dsi_device_info *info;
	int i, ret;

	pinfo = devm_kzalloc(dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	// pinfo->vddio = devm_regulator_get(dev, "vddio");
	// if (IS_ERR(pinfo->vddio))
	// 	return dev_err_probe(dev, PTR_ERR(pinfo->vddio), "failed to get vddio regulator\n");

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");

	pinfo->desc = of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	/* If the panel is dual dsi, register DSI1 */
	if (pinfo->desc->is_dual_dsi) {
		info = &pinfo->desc->dsi_info;

		dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi1) {
			dev_err(dev, "cannot get secondary DSI node.\n");
			return -ENODEV;
		}

		dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
		of_node_put(dsi1);
		if (!dsi1_host)
			return dev_err_probe(dev, -EPROBE_DEFER, "cannot get secondary DSI host\n");

		pinfo->dsi[1] = mipi_dsi_device_register_full(dsi1_host, info);
		if (!pinfo->dsi[1]) {
			dev_err(dev, "cannot get secondary DSI device\n");
			return -ENODEV;
		}
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	pinfo->panel.prepare_prev_first = true;

	drm_panel_init(&pinfo->panel, dev, &nt36532_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	ret = of_drm_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	drm_panel_add(&pinfo->panel);

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;
		pinfo->dsi[i]->dsc = &pinfo->desc->dsc;
		pinfo->dsi[i]->dsc_slice_per_pkt = 2;

		ret = mipi_dsi_attach(pinfo->dsi[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot attach to DSI%d host.\n", i);
	}

	return 0;
}

static const struct of_device_id nt36532_of_match[] = {
	{
		.compatible = "xiaomi,pipa-csot-nt36532",
		.data = &pipa_csot_desc,
	},
	{
		.compatible = "xiaomi,pipa-tianma-nt36532",
		.data = &pipa_tianma_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, nt36532_of_match);

static struct mipi_dsi_driver nt36532_driver = {
	.probe = nt36532_probe,
	.remove = nt36532_remove,
	.driver = {
		.name = "panel-novatek-nt36532",
		.of_match_table = nt36532_of_match,
	},
};
module_mipi_dsi_driver(nt36532_driver);

MODULE_AUTHOR("Molly Sophia <mollysophia379@gmail.com>");
MODULE_AUTHOR("Arseniy Velikanov <me@adomerle.xyz>");
MODULE_DESCRIPTION("DRM driver for Novatek nt36532 based MIPI DSI panels");
MODULE_LICENSE("GPL");
