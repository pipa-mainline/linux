/* Copyright (c) 2022 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* this driver is compatible for nuvolta wireless charge ic */

#include <linux/i2c.h>
#include <linux/alarmtimer.h>
#include <linux/ktime.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <asm/uaccess.h>
#include <linux/irq.h>

#include "nu1665.h"

static struct nuvolta_1665_chg *g_chip;
static int last_valid_pen_soc = -1;
static int pen_soc_count = 0;
static u8 sram_buffer[256];
static int curr_count = 0;

static int nuvolta_1665_set_enable_mode(struct nuvolta_1665_chg *chip,
					bool enable);
static int nuvolta_1665_set_reverse_chg_mode(struct nuvolta_1665_chg *chip,
					     int enable);
static int tx_info_update(struct nuvolta_1665_chg *chip, u8 *buff);
static int fw_crc_chk(struct nuvolta_1665_chg *chip);
static int read_fw_version(struct nuvolta_1665_chg *chip, u8 *version);
static int nuvolta_1665_download_fw(struct nuvolta_1665_chg *chip,
				    bool power_on, bool force);
static int nuvolta_1665_get_reverse_soc(struct nuvolta_1665_chg *chip);

int pen_charge_state_notifier_register_client(struct notifier_block *nb);
int pen_charge_state_notifier_unregister_client(struct notifier_block *nb);
void pen_charge_state_notifier_call_chain(unsigned long val);

static struct regmap_config nuvolta_1665_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

struct delayed_work *pen_notifier_work;

static BLOCKING_NOTIFIER_HEAD(pen_charge_state_notifier_list);

static void pen_charge_notifier_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, pen_notifier_work.work);
	blocking_notifier_call_chain(&pen_charge_state_notifier_list,
				     chip->pen_val, chip->pen_v);
	return;
}

int pen_charge_state_notifier_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pen_charge_state_notifier_list,
						nb);
}
EXPORT_SYMBOL(pen_charge_state_notifier_register_client);

int pen_charge_state_notifier_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(
		&pen_charge_state_notifier_list, nb);
}
EXPORT_SYMBOL(pen_charge_state_notifier_unregister_client);

void pen_charge_state_notifier_call_chain(unsigned long val)
{
	struct nuvolta_1665_chg *chip = container_of(
		pen_notifier_work, struct nuvolta_1665_chg, pen_notifier_work);

	chip->pen_v = NULL;
	chip->pen_val = val;
	schedule_delayed_work(&chip->pen_notifier_work, msecs_to_jiffies(0));
}
EXPORT_SYMBOL(pen_charge_state_notifier_call_chain);

static int rx1665_read(struct nuvolta_1665_chg *chip, u8 *val, u16 addr)
{
	unsigned int temp;
	int rc = 0;

	rc = regmap_read(chip->regmap, addr, &temp);
	if (rc < 0)
		dev_err(chip->dev, "i2c read error: %d, address:%x\n", rc, addr);
	else
		*val = (u8)temp;

	return rc;
}

static int rx1665_read_buffer(struct nuvolta_1665_chg *chip, u8 *buf, u16 addr,
			      int size)
{
	int rc = 0;

	while (size--) {
		rc = rx1665_read(chip, buf++, addr++);
		if (rc < 0) {
			dev_err(chip->dev, "[%s]i2c read error: %d\n", __func__, rc);
			return rc;
		}
	}

	return rc;
}

static int rx1665_write(struct nuvolta_1665_chg *chip, u8 val, u16 addr)
{
	int rc = 0;

	rc = regmap_write(chip->regmap, addr, val);
	if (rc < 0)
		dev_err(chip->dev, "i2c write error: %d, address:%x\n", rc, addr);

	return rc;
}

static bool nuvolta_1665_check_cmd_free(struct nuvolta_1665_chg *chip, u16 reg)
{
	u8 rx_cmd_busy = 0, retry = 0;

	while (retry++ < 100) {
		rx1665_read(chip, &rx_cmd_busy, reg);
		if (rx_cmd_busy != 0x55)
			return true;
	}

	dev_info(chip->dev, "%s reg: %x always busy\n", __func__, reg);
	return false;
}

static bool nuvolta_1665_check_buffer_ready(struct nuvolta_1665_chg *chip)
{
	return nuvolta_1665_check_cmd_free(chip, 0x0024);
}

static bool nuvolta_1665_check_rx_ready(struct nuvolta_1665_chg *chip)
{
	return nuvolta_1665_check_cmd_free(chip, 0x0025);
}

static int nuvolta_1665_start_tx_function(struct nuvolta_1665_chg *chip,
					  bool en)
{
	int ret = 0;

	dev_err(chip->dev, "%s enable:%d\n", __func__, en);
	if (!chip->fw_update) {
		if (en) {
			ret = rx1665_write(chip, 0x01, TRX_MODE_EN);
			if (ret >= 0) {
				dev_info(chip->dev,
					"ic work on rtx mode,start reverse charging,ret:%d\n",
					ret);
				chip->is_reverse_mode = 1;
				return 1;
			}
		} else
			ret = rx1665_write(chip, 0x00, TRX_MODE_EN);

		dev_info(chip->dev, "[%s] ret = %d\n", __func__, ret);
	}

	dev_info(chip->dev, "Not open reverse charging start:%d\n", en);
	ret = nuvolta_1665_set_reverse_chg_mode(chip, false);
	return 0;
}

static int rx_set_reverse_boost_enable_gpio(struct nuvolta_1665_chg *chip,
					    int enable)
{
	int ret = 0;
	if (gpio_is_valid(chip->reverse_boost_gpio)) {
		ret = gpio_request(chip->reverse_boost_gpio,
				   "reverse-boost-enable-gpio");
		if (ret) {
			dev_err(chip->dev,
				"%s: unable to reverse_boost_enable_gpio [%d]\n",
				__func__, chip->reverse_boost_gpio);
		}

		ret = gpio_direction_output(chip->reverse_boost_gpio, !!enable);
		if (ret) {
			dev_err(chip->dev,
				"%s: cannot set direction for reverse_boost_enable_gpio  gpio [%d]\n",
				__func__, chip->reverse_boost_gpio);
		}
		gpio_free(chip->reverse_boost_gpio);
	} else
		dev_err(chip->dev, "%s: unable to set reverse_boost_enable_gpio\n",
			    __func__);

	return ret;
}

static int nuvolta_1665_set_reverse_gpio(struct nuvolta_1665_chg *chip,
					 int enable)
{
	int ret = 0;
	union power_supply_propval val = {
		0,
	};

	val.intval = !!enable;

	if (gpio_is_valid(chip->tx_on_gpio)) {
		if (!enable) {
			rx_set_reverse_boost_enable_gpio(chip, enable);
			msleep(100);
		}

		ret = gpio_request(chip->tx_on_gpio, "tx-on-gpio");
		if (ret) {
			dev_err(chip->dev, "%s: unable to request tx_on gpio\n",
				    __func__);
		}
		ret = gpio_direction_output(chip->tx_on_gpio, enable);
		if (ret) {
			dev_err(chip->dev, "%s: cannot set direction for tx_on gpio\n",
				    __func__);
		}

		ret = gpio_get_value(chip->tx_on_gpio);
		dev_info(chip->dev, "txon gpio: %d\n", ret);
		dev_err(chip->dev, "%s-2 chip->tx_on_gpio:%d, gpio is valid:%d,\n",
			    __func__, chip->tx_on_gpio,
			    gpio_is_valid(chip->tx_on_gpio));
		gpio_free(chip->tx_on_gpio);
		if (enable) {
			msleep(100);
			rx_set_reverse_boost_enable_gpio(chip, enable);
		}
	} else
		dev_err(chip->dev, "%s: unable to set tx_on gpio\n", __func__);

	return ret;
}

static int nuvolta_1665_set_reverse_chg_mode(struct nuvolta_1665_chg *chip,
					     int enable)
{
	int rc = 0;

	if (chip->fw_update) {
		goto out;
	}

	nuvolta_1665_set_reverse_gpio(chip, enable);

	if (enable) {
		chip->is_boost_mode = 1;
		dev_info(chip->dev, "enable reverse charging\n");
		chip->reverse_chg_en = true;

		msleep(100);
		rc = nuvolta_1665_start_tx_function(chip, true);

	} else {
		chip->is_boost_mode = 0;
		dev_info(chip->dev, "disable reverse charging\n");
		chip->reverse_chg_en = false;
		chip->alarm_flag = false;
		chip->is_reverse_mode = 0;
		chip->reverse_pen_soc = 255;
		pen_soc_count = 0;
		curr_count = 0;

		msleep(100);
		cancel_delayed_work(&chip->reverse_chg_state_work);
		cancel_delayed_work(&chip->reverse_dping_state_work);
		cancel_delayed_work(&chip->reverse_chg_work);
		cancel_delayed_work(&chip->pen_check_work);
		pm_relax(chip->dev);
	}

	schedule_delayed_work(&chip->pen_check_work, msecs_to_jiffies(1500));

out:
	return 0;
}

static int nuvolta_1665_get_cep(struct nuvolta_1665_chg *chip, int *cep)
{
	int ret = 0;
	bool status = true;
	u8 read_buf[128];

	if (!chip->power_good_flag) {
		*cep = 0;
		return ret;
	}

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_write(chip, 0x88, 0x0062);
	if (ret < 0)
		return ret;

	status = nuvolta_1665_check_buffer_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_read_buffer(chip, read_buf, RX_DATA_INFO, 30);
	if (ret < 0)
		return ret;

	*cep = read_buf[10];
	dev_info(chip->dev, "get rx cep: %d\n", *cep);

	return ret;
}

static int nuvolta_1665_set_vout(struct nuvolta_1665_chg *chip, int vout)
{
	int ret = 0;
	bool status = true;
	u8 vout_l = 0, vout_h = 0;
	int max_vol = 19500;
	int cep = 0;

	if (!chip->power_good_flag) {
		dev_info(chip->dev, "power good disonline, don't set vout\n");
		return 0;
	}

	if (chip->parallel_charge) {
		ret = nuvolta_1665_get_cep(chip, &cep);
		if (ret < 0) {
			dev_info(chip->dev, "get cep failed : %d\n", ret);
			return ret;
		} else if (ABS(cep) > ABS_CEP_VALUE) {
			dev_info(chip->dev, "[%s] vout: %d, cep: %d, not set vout\n",
				     __func__, vout, cep);
			return 0;
		}
	}

	if (vout < 4000) {
		vout = 6000;
	} else if (vout > max_vol) {
		vout = max_vol;
	}

	vout_h = (u8)(vout >> 8);
	vout_l = (u8)(vout & 0xFF);

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_write(chip, 0x31, 0x0000);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, 0x02, 0x0001);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, vout_l, 0x0002);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, vout_h, 0x0003);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, 0x04, 0x0060);
	if (ret < 0)
		return ret;

	chip->vout_setted = vout;
	dev_info(chip->dev, "set rx vout: %d\n", vout);

	return ret;
}

static int nuvolta_1665_get_vrect(struct nuvolta_1665_chg *chip, int *vrect)
{
	int ret = 0;
	bool status = true;
	u8 read_buf[128];

	if (!chip->power_good_flag) {
		*vrect = 0;
		return ret;
	}

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_write(chip, 0x88, 0x0062);
	if (ret < 0)
		return ret;

	status = nuvolta_1665_check_buffer_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_read_buffer(chip, read_buf, RX_DATA_INFO, 30);
	if (ret < 0)
		return ret;

	*vrect = read_buf[19] * 256 + read_buf[18];
	dev_info(chip->dev, "get rx vrect: %d\n", *vrect);

	return ret;
}

static int nuvolta_1665_get_vout(struct nuvolta_1665_chg *chip, int *vout)
{
	int ret = 0;
	bool status = true;
	u8 read_buf[128];
	s8 cep = 0;

	if (!chip->power_good_flag) {
		*vout = 0;
		return ret;
	}

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_write(chip, 0x88, 0x0062);
	if (ret < 0)
		return ret;

	status = nuvolta_1665_check_buffer_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_read_buffer(chip, read_buf, RX_DATA_INFO, 30);
	if (ret < 0)
		return ret;

	*vout = read_buf[21] * 256 + read_buf[20];
	cep = read_buf[10];
	dev_info(chip->dev, "get rx vout: %d, cep: %d\n", *vout, cep);
	chip->reverse_vout = *vout;

	return ret;
}

static int nuvolta_1665_get_iout(struct nuvolta_1665_chg *chip, int *iout)
{
	int ret = 0;
	bool status = true;
	u8 read_buf[128];

	if (!chip->power_good_flag) {
		*iout = 0;
		return ret;
	}

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_write(chip, 0x88, 0x0062);
	if (ret < 0)
		return ret;

	status = nuvolta_1665_check_buffer_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_read_buffer(chip, read_buf, RX_DATA_INFO, 30);
	if (ret < 0)
		return ret;

	*iout = read_buf[17] * 256 + read_buf[16];

	dev_info(chip->dev, "get rx iout: %d\n", *iout);
	chip->reverse_iout = *iout;
	return ret;
}

static int nuvolta_1665_get_temp(struct nuvolta_1665_chg *chip, int *temp)
{
	int ret = 0;
	bool status = true;
	u8 read_buf[128];

	if (!chip->power_good_flag) {
		*temp = 0;
		return ret;
	}

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_write(chip, 0x88, 0x0062);
	if (ret < 0)
		return 0;

	status = nuvolta_1665_check_buffer_ready(chip);
	if (!status)
		return 0;

	ret = rx1665_read_buffer(chip, read_buf, RX_DATA_INFO, 30);
	if (ret < 0)
		return ret;

	*temp = read_buf[15] * 256 + read_buf[14];
	dev_info(chip->dev, "get rx temp: %d\n", *temp);

	return ret;
}

static int tx_info_update(struct nuvolta_1665_chg *chip, u8 *buff)
{
	int ret = 0;

	ret = rx1665_write(chip, 0x88, 0x0000);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, 0x01, 0x0060);
	if (ret < 0)
		return ret;

	msleep(20);

	ret = rx1665_read_buffer(chip, buff, 0x1200, 256);
	if (ret < 0) {
		dev_err(chip->dev, "Update %s failed!\n", __func__);
		return ret;
	}

	return ret;
}

static int nuvolta_1665_get_reverse_vout(struct nuvolta_1665_chg *chip,
					 int *vout)
{
	int ret = 0;

	*vout = 256 * sram_buffer[19] + sram_buffer[18];

	dev_info(chip->dev, "get tx reverse vout: %d\n", *vout);
	chip->reverse_vout = *vout;

	return ret;
}

static int nuvolta_1665_get_reverse_iout(struct nuvolta_1665_chg *chip,
					 int *iout)
{
	int ret = 0;

	*iout = 256 * sram_buffer[17] + sram_buffer[16];

	dev_info(chip->dev, "get tx reverse iout: %d\n", *iout);
	chip->reverse_iout = *iout;

	if (chip->reverse_iout > 500 || chip->reverse_iout < 50) {
		curr_count++;
	}
	if (curr_count >= 5) {
		curr_count = 0;
		dev_info(chip->dev, "The pen position is out of the right place.\n");
		nuvolta_1665_set_reverse_chg_mode(chip, false);
		chip->is_reverse_mode = 0;
		chip->is_reverse_chg = 2;
	}
	return ret;
}

static int nuvolta_1665_get_reverse_temp(struct nuvolta_1665_chg *chip,
					 int *temp)
{
	int ret = 0;

	*temp = 256 * sram_buffer[15] + sram_buffer[14];

	dev_info(chip->dev, "get tx reverse temperature: %d\n", *temp);
	chip->reverse_temp = *temp;

	return ret;
}

#define SOC_100_RETRY 20
static int soc_count;
static int nuvolta_1665_get_reverse_soc(struct nuvolta_1665_chg *chip)
{
	int ret = 0;
	u8 soc = 0xFF;
	static int last_soc;

	soc = sram_buffer[9];

	if ((soc < 0) || (soc > 0x64)) {
		if (soc == 0xFF) {
			dev_info(chip->dev, "[reverse] soc is default 0xFF\n");
			chip->reverse_pen_soc = 0xFF;
			return ret;
		} else {
			dev_info(chip->dev, "[reverse] soc illegal: %d\n", soc);
			return ret;
		}
	}

	chip->reverse_pen_soc = soc + 1;
	if (chip->reverse_pen_soc > 100)
		chip->reverse_pen_soc = 100;
	dev_info(chip->dev, "get tx reverse raw_soc: %d, UI_soc:%d\n", soc,
		     chip->reverse_pen_soc);

	if ((soc == 100) && (pen_soc_count < SOC_100_RETRY)) {
		dev_info(chip->dev, "[reverse] soc is 100 count: %d\n", pen_soc_count);
		pen_soc_count++;
	} else {
		pen_soc_count = 0;
	}

	if ((soc - last_soc) == 0)
		soc_count++;
	else {
		dev_info(chip->dev, "pen soc is change soc_count=%d!\n", soc_count);
		soc_count = 0;
	}

	if (pen_soc_count == SOC_100_RETRY) {
		dev_info(chip->dev,
			"[reverse] soc is 100 exceed 6 times, disable reverse chg!\n");
		nuvolta_1665_set_reverse_chg_mode(chip, false);
		chip->is_reverse_mode = 0;
		chip->is_reverse_chg = 2;
		pen_soc_count = 0;
	}

	if (soc_count >= 180) {
		dev_info(chip->dev,
			"Happen pen lock, need disable/enable reverse chg!\n");
		soc_count = 0;
		ret = nuvolta_1665_set_reverse_chg_mode(chip, false);
		chip->is_reverse_mode = 0;
		chip->is_reverse_chg = 2;
		msleep(30);
		ret = nuvolta_1665_set_reverse_chg_mode(chip, true);
	}

	last_soc = soc;

	return ret;
}

static u8 nuvolta_1665_get_fastchg_result(struct nuvolta_1665_chg *chip)
{
	u8 fastchg_result = 0;
	bool status = true;
	int ret = 0;

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return fastchg_result;

	ret = rx1665_write(chip, 0x88, 0x0062);
	if (ret < 0)
		return fastchg_result;

	status = nuvolta_1665_check_buffer_ready(chip);
	if (!status)
		return fastchg_result;

	ret = rx1665_read(chip, &fastchg_result, RX_FASTCHG_RESULT);
	if (ret < 0)
		return fastchg_result;

	dev_info(chip->dev, "[%s] fastch result: %d\n", __func__, fastchg_result);

	return fastchg_result;
}

static void nuvolta_1665_power_off_err(struct nuvolta_1665_chg *chip)
{
	int ret = 0;
	u8 err_code = 0;

	ret = rx1665_read(chip, &err_code, RX_POWER_OFF_ERR);
	if (ret < 0)
		return;

	/*unknown:0x00 otp:0x03 ovp:0x04 ocp:0x05 sc:0x06 hop:0x10
	 *sop:0x11 sleep:0x0B ovl:0x13 vup:0x14 rect_err:0x15
	 */
	dev_info(chip->dev, "[%s] power off err = 0x%x\n", __func__, err_code);
	return;
}

static void nuvolta_1665_adapter_handle(struct nuvolta_1665_chg *chip)
{
	dev_info(chip->dev, "[%s] adapter: %d, epp: %d\n", __func__,
		     chip->adapter_type, chip->epp);

	dev_info(chip->dev, "set icl for SDP/CDP/DCP/QC2 adapter\n");
	chip->pre_curr = 750;

	schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(1000));
	return;
}

static void nuvolta_1665_clear_int(struct nuvolta_1665_chg *chip)
{
	int ret = 0;

	ret = rx1665_write(chip, 0x68, 0x0000);
	ret = rx1665_write(chip, 0x01, 0x0060);

	dev_info(chip->dev, "[%s] ret: %d\n", __func__, ret);
	return;
}

static void reverse_chg_state_set_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, reverse_chg_state_work.work);
	int ret;

	dev_info(chip->dev, "no rx found and disable reverse charging\n");
	mutex_lock(&chip->reverse_op_lock);
	ret = nuvolta_1665_set_reverse_chg_mode(chip, false);
	chip->is_reverse_chg = REVERSE_STATE_TIMEOUT;
	chip->is_reverse_mode = 0;
	mutex_unlock(&chip->reverse_op_lock);

	return;
}

static void reverse_dping_state_set_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, reverse_dping_state_work.work);
	int ret;

	dev_info(chip->dev, "tx mode fault and disable reverse charging\n");
	mutex_lock(&chip->reverse_op_lock);
	ret = nuvolta_1665_set_reverse_chg_mode(chip, false);
	chip->is_reverse_mode = 0;
	chip->is_reverse_chg = REVERSE_STATE_ENDTRANS;
	mutex_unlock(&chip->reverse_op_lock);

	return;
}

static void pen_check_worker(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(work,
			struct nuvolta_1665_chg, pen_check_work.work);

	static bool pen_charge_en = false;
	bool enable = (chip->reverse_pen_soc >= 0 && chip->reverse_pen_soc <= 100);

	if (pen_charge_en != enable)
		pen_charge_state_notifier_call_chain(enable);

	if (chip->reverse_chg_en != enable)
		nuvolta_1665_set_reverse_chg_mode(chip, enable);

	pen_charge_en = enable;
}

static int nuvolta_1665_reverse_enable_fod(struct nuvolta_1665_chg *chip,
					   bool enable)
{
	int ret = 0;
	bool status = true;
	u8 gain = REVERSE_FOD_GAIN;
	u8 offset = REVERSE_FOD_OFFSET;

	if (!enable)
		return ret;

	status = nuvolta_1665_check_rx_ready(chip);
	if (!status)
		return -1;

	ret = rx1665_write(chip, 0x23, 0x0000);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, 0x01, 0x0001);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, gain, 0x0002);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, offset, 0x0003);
	if (ret < 0)
		return ret;

	ret = rx1665_write(chip, 0x04, 0x0060);
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "[%s] gain: %d, offset: %d\n", __func__, gain, offset);
	return ret;
}

static void nuvolta_1665_reverse_chg_handler(struct nuvolta_1665_chg *chip,
					     u16 int_flag)
{
	if (int_flag & RTX_INT_EPT) {
		if (tx_info_update(chip, sram_buffer) >= 0)
			dev_info(chip->dev, "tx mode ept. the code:0x%02x\n",
				     sram_buffer[10]);
		goto out;
	}

	if (int_flag & INT_GET_DPING) {
		dev_info(chip->dev, "TRX get dping and disable reverse charging \n");
		nuvolta_1665_set_reverse_chg_mode(chip, false);
		chip->is_reverse_mode = 0;
		chip->is_reverse_chg = 2;
		goto out;
	}

	if (int_flag & RTX_INT_START_DPING) {
		pm_relax(chip->dev);
		dev_info(chip->dev, "tx mode ping\n");
	}

	if (int_flag & RTX_INT_GET_CFG) {
		pm_stay_awake(chip->dev);

		/* set reverse charging state to started*/
		nuvolta_1665_reverse_enable_fod(chip, true);
		msleep(50);

		//start reverse chg infor work
		cancel_delayed_work_sync(&chip->reverse_chg_work);
		msleep(10);
		schedule_delayed_work(&chip->reverse_chg_work, 0);
		/* set reverse charging state to started */
		if (chip->is_reverse_mode || chip->is_boost_mode) {
			chip->is_reverse_chg = 4;
		}
		dev_info(chip->dev, "tx mode get rx\n");
	}

	if (int_flag & INT_GET_SS)
		dev_info(chip->dev, "TRX get ss\n");
	if (int_flag & INT_GET_ID)
		dev_info(chip->dev, "TRX get id\n");
	if (int_flag & INT_INIT_TX)
		dev_info(chip->dev, "TRX reset done\n");

	if (int_flag & INT_GET_PPP) {
		dev_info(chip->dev, "INT_GET_PPP.\n");
		if (tx_info_update(chip, sram_buffer) >= 0) {
			dev_info(chip->dev,
				"receive dates: 0x%02x: ,0x%02x: ,0x%02x: ,0x%02x: ,0x%02x: ,0x%02x: ,0x%02x: ,0x%02x:\n",
				sram_buffer[42], sram_buffer[43],
				sram_buffer[44], sram_buffer[45],
				sram_buffer[46], sram_buffer[47],
				sram_buffer[48], sram_buffer[49]);
		}
	}
out:
	return;
}

static void nuvolta_1665_chg_handler(struct nuvolta_1665_chg *chip,
				     u16 int_flag)
{
	switch (int_flag) {
	case RX_INT_FAST_CHARGE:
		dev_info(chip->dev, "[%s] fastchg finish!\n", __func__);
		chip->fc_flag = nuvolta_1665_get_fastchg_result(chip);
		if (chip->fc_flag) {
			nuvolta_1665_adapter_handle(chip);
		} else if (chip->set_fastcharge_vout_cnt++ < 3) {
			dev_info(chip->dev, "set fastchg vol failed, retry %d\n",
				     chip->set_fastcharge_vout_cnt);
			msleep(2000);
		} else {
			dev_info(chip->dev, "set fastchg vol failed finally\n");
			nuvolta_1665_adapter_handle(chip);
		}
		break;
	case RX_INT_OCP_OTP_ALARM:
		dev_info(chip->dev, "[%s] OCP OR OTP trigger\n", __func__);
		schedule_delayed_work(&chip->rx_alarm_work,
				      msecs_to_jiffies(500));
		break;
	case RX_INT_POWER_OFF:
		dev_info(chip->dev, "[%s] POWER OFF INT trigger\n", __func__);
		nuvolta_1665_power_off_err(chip);
		break;
	default:
		break;
	}

	return;
}

static void nu1665_dump_regs(struct nuvolta_1665_chg *chip)
{
	u8 int_l = 0;
	int ret = 0;

	ret = rx1665_read(chip, &int_l, REG_RX_REV_CMD); //0x0020
	if (ret < 0) {
		dev_err(chip->dev, "%s read int 0x20 error\n", __func__);
		goto exit;
	}

	ret = rx1665_read(chip, &int_l, 0x0021); //0x0021
	if (ret < 0) {
		dev_err(chip->dev, "%s read int 0x21 error\n", __func__);
		goto exit;
	}

	ret = rx1665_read(chip, &int_l, 0x0022); //0x0022
	if (ret < 0) {
		dev_err(chip->dev, "%s read int 0x22 error\n", __func__);
		goto exit;
	}

	ret = rx1665_read(chip, &int_l, 0x0023); //0x0022
	if (ret < 0) {
		dev_err(chip->dev, "%s read int 0x23 error\n", __func__);
		goto exit;
	}
exit:
	return;
}

static void nuvolta_1665_wireless_int_work(struct work_struct *work)
{
	u16 int_flags = 0;
	u8 tmp = 0; //int_l = 0, int_h = 0,
	u8 int_trx_mode = RTX_MODE;
	int ret = 0;
	int irq_level;

	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, wireless_int_work.work);

	if (gpio_is_valid(chip->irq_gpio))
		irq_level = gpio_get_value(chip->irq_gpio);
	else {
		dev_err(chip->dev, "%s: irq gpio not provided\n", __func__);
		irq_level = -1;
		pm_relax(chip->dev);
		return;
	}
	dev_info(chip->dev, "irq gpio status: %d\n", irq_level);
	if (irq_level) {
		dev_info(chip->dev, "irq is high level, ignore%d\n", irq_level);
		pm_relax(chip->dev);
		return;
	}
	mutex_lock(&chip->wireless_chg_int_lock);

	ret = rx1665_read(chip, &tmp, REG_RX_REV_CMD); //0x0020
	if (ret < 0) {
		dev_err(chip->dev, "%s read int 0x20 error\n", __func__);
		goto exit;
	}
	int_flags |= tmp;
	tmp = 0;

	ret = rx1665_read(chip, &tmp, REG_RX_REV_DATA1); //0x0021
	if (ret < 0) {
		dev_err(chip->dev, "%s read int 0x21 error\n", __func__);
		goto exit;
	}
	int_flags |= (tmp << 8);
	tmp = 0;

	if (rx1665_read(chip, &tmp, 0x0022) < 0) {
		dev_err(chip->dev, "%s read int 0x22 error\n", __func__);
		goto exit;
	}
	int_flags |= (tmp << 16);
	tmp = 0;

	if (rx1665_read(chip, &tmp, 0x0023) < 0) {
		dev_err(chip->dev, "%s read int 0x23 error\n", __func__);
		goto exit;
	}
	int_flags |= (tmp << 24);

	nu1665_dump_regs(chip);
	dev_info(chip->dev, "int_flag: 0x%x\n", int_flags);
	nuvolta_1665_clear_int(chip);

	dev_info(chip->dev, "ic work only rtx mode\n");
	if (int_trx_mode == RTX_MODE) {
		dev_info(chip->dev, "ic work on rtx mode\n");
		nuvolta_1665_reverse_chg_handler(chip, int_flags);
	} else {
		dev_info(chip->dev, "ic work on rx mode\n");
		nuvolta_1665_chg_handler(chip, int_flags);
	}

exit:
	mutex_unlock(&chip->wireless_chg_int_lock);
	return;
}

static irqreturn_t nuvolta_1665_interrupt_handler(int irq, void *dev_id)
{
	struct nuvolta_1665_chg *chip = dev_id;

	dev_info(chip->dev, "[%s]\n", __func__);
	pm_stay_awake(chip->dev);
	schedule_delayed_work(&chip->wireless_int_work, 0);

	return IRQ_HANDLED;
}

static void nuvolta_1665_reset_parameters(struct nuvolta_1665_chg *chip)
{
	dev_info(chip->dev, "%s\n", __func__);

	chip->power_good_flag = 0;
	chip->ss = 2;
	chip->epp = 0;
	chip->qc_enable = false;
	chip->chg_phase = NORMAL_MODE;
	chip->adapter_type = 0;
	chip->fc_flag = 0;
	chip->set_fastcharge_vout_cnt = 0;
	chip->parallel_charge = false;
	chip->reverse_chg_en = false;
	chip->alarm_flag = false;

	return;
}

static void nuvolta_1665_pg_det_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, wireless_pg_det_work.work);
	int ret = 0, wls_switch_usb = 0;

	if (gpio_is_valid(chip->power_good_gpio)) {
		ret = gpio_get_value(chip->power_good_gpio);
		if (ret) {
			dev_info(chip->dev, "power_good high, wireless attached\n");
			chip->power_good_flag = 1;
			chip->adapter_type = ADAPTER_SDP;
			cancel_delayed_work(&chip->delay_report_status_work);
			cancel_delayed_work(&chip->rx_enable_usb_work);
		} else {
			dev_info(chip->dev, "power_good low, wireless detached\n");
			nuvolta_1665_reset_parameters(chip);

			cancel_delayed_work(&chip->chg_monitor_work);
			cancel_delayed_work(&chip->max_power_control_work);
			cancel_delayed_work(&chip->rx_alarm_work);
			schedule_delayed_work(&chip->rx_enable_usb_work,
					      msecs_to_jiffies(500));
			schedule_delayed_work(&chip->delay_report_status_work,
					      msecs_to_jiffies(2000));

			dev_info(chip->dev, "wireless switch to usb: %d\n",
				     wls_switch_usb);
		}
	}
}

static void nuvolta_1665_get_charge_phase(struct nuvolta_1665_chg *chip,
					  int *chg_phase)
{
	switch (*chg_phase) {
	case NORMAL_MODE:
		if (chip->batt_soc == 100) {
			*chg_phase = TAPER_MODE;
			dev_info(chip->dev, "change normal mode to tapter mode");
		}
		break;
	case TAPER_MODE:
		if ((chip->batt_soc == 100) &&
		    (chip->chg_status == POWER_SUPPLY_STATUS_FULL)) {
			*chg_phase = FULL_MODE;
			dev_info(chip->dev, "change taper mode to full mode");
		} else if (chip->batt_soc < 99) {
			*chg_phase = NORMAL_MODE;
			dev_info(chip->dev, "change taper mode to normal mode");
		}
		break;
	case FULL_MODE:
		if ((chip->chg_status == POWER_SUPPLY_STATUS_CHARGING) &&
		    (chip->batt_soc < 100)) {
			*chg_phase = RECHG_MODE;
			dev_info(chip->dev, "change full mode to recharge mode");
		}
		break;
	case RECHG_MODE:
		if (chip->chg_status == POWER_SUPPLY_STATUS_FULL) {
			*chg_phase = FULL_MODE;
			dev_info(chip->dev, "change recharge mode to full mode");
		}
		break;
	default:
		break;
	}
	return;
}

static void nuvolta_1665_get_charging_info(struct nuvolta_1665_chg *chip)
{
	int vout, iout, vrect;
	union power_supply_propval val = {
		0,
	};
	int ret = 0;

	if (!chip)
		return;

	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy)
		dev_err(chip->dev, "failed to get batt_psy\n");
	else {
		power_supply_get_property(chip->batt_psy,
					  POWER_SUPPLY_PROP_CAPACITY, &val);
		chip->batt_soc = val.intval;
		power_supply_get_property(chip->batt_psy,
					  POWER_SUPPLY_PROP_STATUS, &val);
		chip->chg_status = val.intval;
		nuvolta_1665_get_charge_phase(chip, &chip->chg_phase);
	}

	ret = nuvolta_1665_get_iout(chip, &iout);
	if (ret < 0) {
		dev_err(chip->dev, "get iout failed\n");
		iout = 0;
	}
	ret = nuvolta_1665_get_vout(chip, &vout);
	if (ret < 0) {
		dev_err(chip->dev, "get vout failed\n");
		vout = 0;
	}
	ret = nuvolta_1665_get_vrect(chip, &vrect);
	if (ret < 0) {
		dev_err(chip->dev, "get vrect failed\n");
		vrect = 0;
	}

	dev_info(chip->dev,
		"%s:Vout:%d, Iout:%d, Vrect:%d, soc: %d, status: %d, chg_phase: %d\n",
		__func__, vout, iout, vrect, chip->batt_soc, chip->chg_status,
		chip->chg_phase);
}

static void nuvolta_1665_standard_epp_work(struct nuvolta_1665_chg *chip)
{
	dev_info(chip->dev, "run standard epp work\n");
	if (chip->chg_phase == FULL_MODE)
		chip->target_curr = 250;
	if (chip->chg_phase == RECHG_MODE)
		chip->target_curr = 550;

	if (chip->target_vol != chip->pre_vol) {
		dev_info(chip->dev, "set new vout: %d, pre vout: %d\n",
			     chip->target_vol, chip->pre_vol);
		nuvolta_1665_set_vout(chip, chip->target_vol);
		chip->pre_vol = chip->target_vol;
	}

	if (chip->target_curr != chip->pre_curr) {
		dev_info(chip->dev, "set new icl: %d, pre icl: %d\n",
			     chip->target_curr, chip->pre_curr);
		chip->pre_curr = chip->target_curr;
	}

	return;
}

static void nuvolta_1665_monitor_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, chg_monitor_work.work);
	nuvolta_1665_get_charging_info(chip);

	nuvolta_1665_standard_epp_work(chip);

	schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(5000));
}

static void nuvolta_1665_delay_report_status_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, delay_report_status_work.work);

	//TODO:delay report discharging
	dev_info(chip->dev, "just for use chip: %d\n", chip->power_good_flag);
	return;
}

static void nuvolta_1665_check_rx_alarm(struct nuvolta_1665_chg *chip,
					bool *ocp_flag, bool *otp_flag)
{
	int iout = 0, temp = 0;
	int ret = 0;

	ret = nuvolta_1665_get_iout(chip, &iout);
	if (ret < 0)
		*ocp_flag = false;
	else
		*ocp_flag = (iout >= RX_MAX_IOUT);

	ret = nuvolta_1665_get_temp(chip, &temp);
	if (ret < 0)
		*otp_flag = false;
	else
		*otp_flag = (temp >= RX_MAX_TEMP);

	return;
}

static void nuvolta_1665_rx_alarm_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip =
		container_of(work, struct nuvolta_1665_chg, rx_alarm_work.work);

	bool ocp_flag = false, otp_flag = false;

	nuvolta_1665_check_rx_alarm(chip, &ocp_flag, &otp_flag);
	if ((!ocp_flag) && (!otp_flag))
		return;

	schedule_delayed_work(&chip->rx_alarm_work, msecs_to_jiffies(4000));
	return;
}

static void nu1665_hall3_irq_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, hall3_irq_work.work);

	if (chip->fw_update) {
		dev_info(chip->dev, "[hall3] fw updating, don't enable reverse chg\n");
		return;
	}

	if (chip->hall3_online)
		nuvolta_1665_set_reverse_chg_mode(chip, true);
	else if (!chip->hall4_online) {
		nuvolta_1665_set_reverse_chg_mode(chip, false);
		chip->is_reverse_mode = 0;
		chip->is_reverse_chg = 2;
	} else
		dev_info(chip->dev,
			"[hall3] hall4 online, don't disable reverse charge\n");

	return;
}

static void nu1665_hall4_irq_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, hall4_irq_work.work);

	if (chip->fw_update) {
		dev_info(chip->dev, "[hall4] fw updating, don't enable reverse chg\n");
		return;
	}

	if (chip->hall4_online)
		nuvolta_1665_set_reverse_chg_mode(chip, true);
	else if (!chip->hall3_online) {
		nuvolta_1665_set_reverse_chg_mode(chip, false);
		chip->is_reverse_mode = 0;
		chip->is_reverse_chg = 2;
	} else
		dev_info(chip->dev,
			"[hall4] hall3 online, don't disable reverse charge\n");

	return;
}

static void nu_reverse_chg_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, reverse_chg_work.work);
	int temp = 0, rc = 0;

	if (chip->is_reverse_mode || chip->is_boost_mode) {
		rc = tx_info_update(chip, sram_buffer);
		if (rc < 0)
			goto exit;
		nuvolta_1665_get_reverse_vout(chip, &temp);
		nuvolta_1665_get_reverse_iout(chip, &temp);
		nuvolta_1665_get_reverse_temp(chip, &temp);
		nuvolta_1665_get_reverse_soc(chip);
	} else {
		dev_info(chip->dev, "reverse chg closed, return\n");
		chip->reverse_pen_soc = 255;
		chip->reverse_vout = 0;
		chip->reverse_iout = 0;
		return;
	}
exit:
	if (chip->reverse_pen_soc == 255)
		schedule_delayed_work(&chip->reverse_chg_work, 100);
	else
		schedule_delayed_work(&chip->reverse_chg_work, 10 * HZ);

	return;
}

static void nu1665_probe_fw_download_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, probe_fw_download_work.work);
	bool crc_ok = false;
	int rc = 0;
	bool fw_ok = false;
	u8 fw_version = 0;

	dev_info(chip->dev, "[nuvo] enter %s\n", __func__);

	if (chip->fw_update) {
		dev_info(chip->dev, "[nu1665] [%s] FW Update is on going!\n",
			     __func__);
		return;
	}

	pm_stay_awake(chip->dev);
	chip->fw_update = true;
	rc = nuvolta_1665_set_reverse_gpio(chip, true);
	msleep(100);

	rc = fw_crc_chk(chip);
	if (rc < 0) {
		dev_err(chip->dev, "update crc verify failed.\n");
		crc_ok = false;
	} else {
		dev_info(chip->dev, "update crc verify success.\n");
		crc_ok = true;
	}

	rc = read_fw_version(chip, &fw_version);
	dev_info(chip->dev, "%s, FW Version:0x%02x\n", __func__, fw_version);
	chip->fw_version = fw_version;

	if (rc < 0)
		chip->chip_ok = 0;
	else
		chip->chip_ok = 1;

	rc = nuvolta_1665_set_reverse_gpio(chip, false);
	msleep(1000);

	if (fw_version >= FW_VERSION)
		fw_ok = true;
	if (crc_ok && fw_ok)
		dev_info(chip->dev, "Don't need update FW,so skip.\n");
	else {
		rc = nuvolta_1665_set_reverse_gpio(chip, true);
		msleep(100);

		dev_info(chip->dev, "%s: FW download start\n", __func__);
		rc = nuvolta_1665_download_fw(chip, false, true);
		if (rc < 0) {
			dev_err(chip->dev, "[%s] fw download failed!\n", __func__);
		} else {
			dev_info(chip->dev, "%s: FW download end\n", __func__);
		}

		rc = nuvolta_1665_set_reverse_gpio(chip, false);
		msleep(1000);

		// start crc verify
		rc = nuvolta_1665_set_reverse_gpio(chip, true);
		msleep(100);

		if (fw_crc_chk(chip) < 0)
			dev_err(chip->dev, "[%s] fw crc failed!\n", __func__);

		rc = nuvolta_1665_set_reverse_gpio(chip, false);
	}

	chip->fw_update = false;
	pm_relax(chip->dev);
	if (chip->hall3_online)
		schedule_delayed_work(&chip->hall3_irq_work,
				      msecs_to_jiffies(2000));
	else if (chip->hall4_online)
		schedule_delayed_work(&chip->hall4_irq_work,
				      msecs_to_jiffies(2000));
	else
		return;
}

static irqreturn_t nuvolta_1665_power_good_handler(int irq, void *dev_id)
{
	struct nuvolta_1665_chg *chip = dev_id;

	if (chip->fw_update)
		return IRQ_HANDLED;
	schedule_delayed_work(&chip->wireless_pg_det_work, msecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static irqreturn_t nuvolta_1665_hall3_irq_handler(int irq, void *dev_id)
{
	struct nuvolta_1665_chg *chip = dev_id;

	if (gpio_is_valid(chip->hall3_gpio)) {
		if (gpio_get_value(chip->hall3_gpio)) {
			dev_err(chip->dev, "hall3_irq_handler: pen detach\n");
			chip->hall3_online = 0;
			if (chip->hall4_online) {
				dev_err(chip->dev,
					"hall3_irq_handler: hall4 online, return\n");
				return IRQ_HANDLED;
			}
			schedule_delayed_work(&chip->hall3_irq_work,
					      msecs_to_jiffies(0));
			return IRQ_HANDLED;
		} else {
			dev_err(chip->dev, "hall3_irq_handler: pen attach\n");
			chip->hall3_online = 1;
		}
	}

	if (chip->hall4_online) {
		dev_err(chip->dev,
			"[hall3] reverse charging already running, return\n");
		return IRQ_HANDLED;
	} else
		schedule_delayed_work(&chip->hall3_irq_work,
				      msecs_to_jiffies(10));

	return IRQ_HANDLED;
}

static irqreturn_t nuvolta_1665_hall4_irq_handler(int irq, void *dev_id)
{
	struct nuvolta_1665_chg *chip = dev_id;

	if (gpio_is_valid(chip->hall4_gpio)) {
		if (gpio_get_value(chip->hall4_gpio)) {
			dev_err(chip->dev, "hall4_irq_handler: pen detach\n");
			chip->hall4_online = 0;
			if (chip->hall3_online) {
				dev_err(chip->dev,
					"hall4_irq_handler: hall3 online, return\n");
				return IRQ_HANDLED;
			}
			schedule_delayed_work(&chip->hall4_irq_work,
					      msecs_to_jiffies(0));
			return IRQ_HANDLED;
		} else {
			dev_err(chip->dev, "hall4_irq_handler: pen attach\n");
			chip->hall4_online = 1;
		}
	}

	if (chip->hall3_online) {
		dev_err(chip->dev,
			"[hall4] reverse charging already running, return\n");
		return IRQ_HANDLED;
	} else
		schedule_delayed_work(&chip->hall4_irq_work,
				      msecs_to_jiffies(10));

	return IRQ_HANDLED;
}

static int nuvolta_1665_parse_dt(struct nuvolta_1665_chg *chip)
{
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "%s:No DT data Failing Probe\n", __func__);
		return -EINVAL;
	}

	chip->tx_on_gpio = of_get_named_gpio(node, "reverse_chg_ovp_gpio", 0);
	dev_err(chip->dev, "[%s] print tx_on gpio %d\n", __func__, chip->tx_on_gpio);
	if (!gpio_is_valid(chip->tx_on_gpio)) {
		dev_err(chip->dev, "[%s] fail_tx_on gpio %d\n", __func__,
			    chip->tx_on_gpio);
		return -EINVAL;
	}

	chip->irq_gpio = of_get_named_gpio(node, "rx_irq_gpio", 0);
	dev_err(chip->dev, "[%s] print irq_gpio %d\n", __func__, chip->irq_gpio);
	if (!gpio_is_valid(chip->irq_gpio)) {
		dev_err(chip->dev, "[%s] fail_irq_gpio %d\n", __func__,
			    chip->irq_gpio);
		return -EINVAL;
	}

	chip->reverse_boost_gpio =
		of_get_named_gpio(node, "reverse_boost_gpio", 0);
	dev_err(chip->dev, "[%s] print reverse_boost_gpio %d\n", __func__,
		    chip->reverse_boost_gpio);
	if (!gpio_is_valid(chip->reverse_boost_gpio)) {
		dev_err(chip->dev, "[%s] fail reverse_boost_gpio %d\n", __func__,
			    chip->reverse_boost_gpio);
		return -EINVAL;
	}

	chip->hall3_gpio = of_get_named_gpio(node, "hall,int3", 0);
	dev_err(chip->dev, "[%s] print chip->hall3_gpio %d\n", __func__,
		    chip->hall3_gpio);
	if ((!gpio_is_valid(chip->hall3_gpio))) {
		dev_err(chip->dev, "[%s] chip->hall3_gpio %d\n", __func__,
			    chip->hall3_gpio);
		return -EINVAL;
	}

	chip->hall4_gpio = of_get_named_gpio(node, "hall,int4", 0);
	dev_err(chip->dev, "[%s] print chip->hall4_gpio %d\n", __func__,
		    chip->hall4_gpio);
	if ((!gpio_is_valid(chip->hall4_gpio))) {
		dev_err(chip->dev, "[%s] chip->hall4_gpio %d\n", __func__,
			    chip->hall4_gpio);
		return -EINVAL;
	}

	return 0;
}

static int nuvolta_rx1665_gpio_init(struct nuvolta_1665_chg *chip)
{
	int ret = 0;
	int irqn = 0;

	chip->idt_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR(chip->idt_pinctrl)) {
		ret = PTR_ERR(chip->idt_pinctrl);
		dev_err(chip->dev, "Failed to get pinctrl config: %d\n", ret);
		return ret;
	}

	chip->idt_gpio_active = pinctrl_lookup_state(chip->idt_pinctrl, "idt_active");
	if (IS_ERR(chip->idt_gpio_active)) {
		ret = PTR_ERR(chip->idt_gpio_active);
		dev_err(chip->dev, "Failed to get active pinctrl state: %d\n", ret);
		return ret;
	}

	chip->idt_gpio_suspend = pinctrl_lookup_state(chip->idt_pinctrl, "idt_suspend");
	if (IS_ERR(chip->idt_gpio_suspend)) {
		ret = PTR_ERR(chip->idt_gpio_suspend);
		dev_err(chip->dev, "Failed to get suspend pinctrl state: %d\n", ret);
		return ret;
	}

	/* Set the pinctrl state to active initially */
	ret = pinctrl_select_state(chip->idt_pinctrl, chip->idt_gpio_active);
	if (ret) {
		dev_err(chip->dev, "Failed to select active pinctrl state: %d\n", ret);
		return ret;
	}

	if (gpio_is_valid(chip->irq_gpio)) {
		irqn = gpio_to_irq(chip->irq_gpio);
		if (irqn < 0) {
			dev_err(chip->dev, "[%s] gpio_to_irq Fail!, irq_gpio:%d \n",
				    __func__, chip->irq_gpio);
			ret = -1;
			goto fail_irq_gpio;
		}
		chip->irq = irqn;
	} else {
		dev_err(chip->dev, "%s: irq gpio not provided\n", __func__);
		ret = -1;
		goto fail_irq_gpio;
	}

	if (gpio_is_valid(chip->hall3_gpio)) {
		irqn = gpio_to_irq(chip->hall3_gpio);
		if (irqn < 0) {
			ret = irqn;
			dev_err(chip->dev, "%s:hall3 irq gpio failed\n", __func__);
			goto fail_irq_gpio;
		}
		chip->hall3_irq = irqn;
	} else {
		dev_err(chip->dev, "%s:hall3 irq gpio not provided\n", __func__);
		goto err_hall3_irq_gpio;
	}

	if (gpio_is_valid(chip->hall4_gpio)) {
		irqn = gpio_to_irq(chip->hall4_gpio);
		if (irqn < 0) {
			ret = irqn;
			dev_err(chip->dev, "%s:hall4 irq gpio failed\n", __func__);
			goto fail_irq_gpio;
		}
		chip->hall4_irq = irqn;
	} else {
		dev_err(chip->dev, "%s:hall4 irq gpio not provided\n", __func__);
		goto err_hall4_irq_gpio;
	}

	return ret;

fail_irq_gpio:
	gpio_free(chip->irq_gpio);
err_hall4_irq_gpio:
	gpio_free(chip->hall4_gpio);
err_hall3_irq_gpio:
	gpio_free(chip->hall3_gpio);
	return ret;
}

static int nuvolta_rx1665_irq_request(struct nuvolta_1665_chg *chip)
{
	int ret;

	// config irq
	if (!chip->irq) {
		dev_err(chip->dev, "irq is wrong = %s \n", __func__);
		return -EINVAL;
	}

	ret = request_irq(chip->irq, nuvolta_1665_interrupt_handler,
			  (IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
			  "nuvolta_1665_chg_stat_irq", chip);
	if (ret) {
		dev_err(chip->dev, "Failed irq = %d ret = %d\n", chip->irq, ret);
		return ret;
	}
	ret = enable_irq_wake(chip->irq);
	if (ret) {
		dev_err(chip->dev, "%s: enable request irq is failed\n", __func__);
		return ret;
	}

	// config hall3 irq
	if (!chip->hall3_irq) {
		dev_err(chip->dev, "hall3 irq is wrong = %s \n", __func__);
		return -EINVAL;
	}

	ret = request_irq(chip->hall3_irq, nuvolta_1665_hall3_irq_handler,
			  (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
			  "hall3_irq", chip);
	if (ret) {
		dev_err(chip->dev, "Failed hall3-irq = %d ret = %d\n", chip->hall3_irq,
			    ret);
		return ret;
	}

	enable_irq_wake(chip->hall3_irq);
	if (ret) {
		dev_err(chip->dev, "%s: enable request hall3 irq is failed\n",
			    __func__);
		return ret;
	}
	// config hall4 irq
	if (!chip->hall4_irq) {
		dev_err(chip->dev, "hall4 irq is wrong = %s \n", __func__);
		return -EINVAL;
	}

	ret = request_irq(chip->hall4_irq, nuvolta_1665_hall4_irq_handler,
			  (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
			  "hall4_irq", chip);
	if (ret) {
		dev_err(chip->dev, "Failed hall4-irq = %d ret = %d\n", chip->hall4_irq,
			    ret);
		return ret;
	}

	enable_irq_wake(chip->hall4_irq);
	if (ret) {
		dev_err(chip->dev, "%s: enable request hall4 irq is failed\n",
			    __func__);
		return ret;
	}

	return 0;
	// config power good irq
	if (!chip->power_good_irq) {
		dev_err(chip->dev, "power good irq is wrong = %s \n", __func__);
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(
		&chip->client->dev, chip->power_good_irq, NULL,
		nuvolta_1665_power_good_handler,
		(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT),
		"nuvolta_1665_power_good_irq", chip);
	if (ret) {
		dev_err(chip->dev, "Failed irq = %d ret = %d\n", chip->power_good_irq,
			    ret);
		return ret;
	}

	enable_irq_wake(chip->power_good_irq);
	if (ret) {
		dev_err(chip->dev, "%s: enable request power good irq is failed\n",
			    __func__);
		return ret;
	}
	return -EINVAL;
}

static ssize_t chip_vrect_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int vrect = 0, ret = 0;

	ret = nuvolta_1665_get_vrect(g_chip, &vrect);
	if (ret < 0) {
		dev_err(dev, "get vrect failed\n");
		vrect = 0;
	}

	return scnprintf(buf, PAGE_SIZE, "rx1665 Vrect : %d mV\n", vrect);
}

static ssize_t chip_iout_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int iout = 0, ret = 0;

	ret = nuvolta_1665_get_iout(g_chip, &iout);
	if (ret < 0) {
		dev_err(dev, "get iout failed\n");
		iout = 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", iout);
}

static ssize_t chip_vout_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int vout = 0, ret = 0;

	ret = nuvolta_1665_get_vout(g_chip, &vout);
	if (ret < 0) {
		dev_err(dev, "get vout failed\n");
		vout = 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", vout);
}

static ssize_t chip_vout_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int index;

	index = (int)simple_strtoul(buf, NULL, 10);
	dev_info(dev, "[rx1665] [%s] --Store output_voltage = %d\n", __func__,
		     index);
	if ((index < 4000) || (index > 21000)) {
		dev_err(dev, "[rx1665] [%s] Store Voltage %s is invalid\n",
			    __func__, buf);
		nuvolta_1665_set_vout(g_chip, 0);
		return count;
	}

	nuvolta_1665_set_vout(g_chip, index);

	return count;
}

static int nuvolta_1665_check_i2c(struct nuvolta_1665_chg *chip)
{
	int ret = 0;
	u8 data = 0;

	ret = rx1665_write(chip, 0x88, 0x0000);
	if (ret < 0)
		return ret;
	msleep(10);

	ret = rx1665_read(chip, &data, 0x0000);
	if (ret < 0)
		return ret;

	if (data == 0x88) {
		dev_info(chip->dev, "[%s] i2c check ok!\n", __func__);
		return 1;
	} else {
		dev_info(chip->dev, "[%s] i2c check failed!\n", __func__);
		return -1;
	}

	return ret;
}

static int nuvolta_1665_download_fw_data(struct nuvolta_1665_chg *chip,
					 const unsigned char *fw_data,
					 int fw_data_length)
{
	int ret = 0;
	u8 read_data = 0; //, wrfail = 0, busy = 0;
	int i = 0, j = 0;
	u8 __1st_word = 1;

	dev_info(chip->dev, "[%s] start\n", __func__);

	//ret = nuvolta_1665_enter_dtm_mode(chip);
	if (rx1665_write(chip, 0x41, 0x0090) < 0)
		goto exit;
	if (rx1665_write(chip, 0xC0, 0x1000) < 0)
		goto exit;
	if (rx1665_write(chip, 0xFF, 0x0012) < 0)
		goto exit;
	if (ret < 0) {
		dev_err(chip->dev, "[%s] failed to enter dtm mode\n", __func__);
		return ret;
	}

	for (i = 0; i < fw_data_length; i += 4) {
		if (__1st_word) {
			__1st_word = 0;

			if (rx1665_write(chip, 0x01, 0x0017) < 0)
				goto exit;

			if (rx1665_write(chip, fw_data[i + 3], 0x001C) < 0)
				goto exit;
			if (rx1665_write(chip, fw_data[i + 2], 0x001D) < 0)
				goto exit;
			if (rx1665_write(chip, fw_data[i + 1], 0x001E) < 0)
				goto exit;
			if (rx1665_write(chip, fw_data[i + 0], 0x001F) < 0)
				goto exit;

			if (rx1665_write(chip, 0x01, 0x0019) < 0)
				goto exit;
			if (rx1665_write(chip, 0x00, 0x0019) < 0)
				goto exit;

			if (rx1665_write(chip, 0x5A, 0x001A) < 0)
				goto exit;
			msleep(20);
		}

		if (rx1665_write(chip, fw_data[i + 3], 0x001C) < 0)
			goto exit;
		if (rx1665_write(chip, fw_data[i + 2], 0x001D) < 0)
			goto exit;
		if (rx1665_write(chip, fw_data[i + 1], 0x001E) < 0)
			goto exit;
		if (rx1665_write(chip, fw_data[i + 0], 0x001F) < 0)
			goto exit;

		for (j = 0; j < 250; j++) {
			if (rx1665_read(chip, &read_data, 0x001b) < 0)
				goto exit;
			if (!(read_data & (1 << 7)))
				break;
			if (read_data & (1 << 6)) {
				dev_err(chip->dev, "[%s] write failed \n", __func__);
				goto exit;
			}
			msleep(1);
		}

		if (j == 250) {
			dev_err(chip->dev, "[%s] write timeout \n", __func__);
			goto exit;
		}
	}
	if (rx1665_write(chip, 0x00, 0x001A) < 0)
		goto exit;

	return ret;

exit:
	msleep(100);
	dev_err(chip->dev, "[%s] wrfail, MTP error\n", __func__);
	return -1;
}

static int key_open(struct nuvolta_1665_chg *chip)
{
	if (rx1665_write(chip, 0x00, 0x2017) < 0)
		goto exit;
	if (rx1665_write(chip, 0x2D, 0x2017) < 0)
		goto exit;
	if (rx1665_write(chip, 0xD2, 0x2017) < 0)
		goto exit;
	if (rx1665_write(chip, 0x22, 0x2017) < 0)
		goto exit;
	if (rx1665_write(chip, 0xDD, 0x2017) < 0)
		goto exit;
	return 0;
exit:
	dev_err(chip->dev, "[%s] failed \n", __func__);
	return -1;
}

static int write_key0(struct nuvolta_1665_chg *chip)
{
	if (rx1665_write(chip, 0x00, 0x2018) < 0) {
		dev_err(chip->dev, "[%s] failed \n", __func__);
		return -1;
	}
	return 0;
}

static int write_key1(struct nuvolta_1665_chg *chip)
{
	if (rx1665_write(chip, 0x00, 0x2019) < 0) {
		dev_err(chip->dev, "[%s] failed \n", __func__);
		return -1;
	}
	return 0;
}

static int exit_key0(struct nuvolta_1665_chg *chip)
{
	if (rx1665_write(chip, 0xFF, 0x2018) < 0) {
		dev_err(chip->dev, "[%s] failed \n", __func__);
		return -1;
	}
	return 0;
}

static int exit_key1(struct nuvolta_1665_chg *chip)
{
	if (rx1665_write(chip, 0xFF, 0x2019) < 0) {
		dev_err(chip->dev, "[%s] failed \n", __func__);
		return -1;
	}
	return 0;
}

static int exit_key_open(struct nuvolta_1665_chg *chip)
{
	if (rx1665_write(chip, 0x00, 0x2017) < 0) {
		dev_err(chip->dev, "[%s] failed \n", __func__);
		return -1;
	}
	return 0;
}

static int nuvolta_1665_download_fw(struct nuvolta_1665_chg *chip,
				    bool power_on, bool force)
{
	int ret = 0;
	const unsigned char *fw_data = NULL;
	int fw_data_length = 0;
	const struct firmware *fw;

	if (power_on) {
		dev_info(chip->dev, "[%s]auto update when power on\n", __func__);
	}

	ret = request_firmware(&fw, "nuvolta/rx1665.bin", chip->dev);
	if (ret) {
		dev_err(chip->dev, "Failed to load firmware: %d\n", ret);
		return ret;
	}

	fw_data = fw->data;
	fw_data_length = fw->size;

	dev_info(chip->dev, "Firmware loaded successfully: %zu bytes\n", fw->size);

	dev_info(chip->dev, "[%s] fw data length: %d\n", __func__, fw_data_length);

	if (key_open(chip) < 0)
		return -1;

	if (write_key0(chip) < 0)
		return -1;

	if (write_key1(chip) < 0)
		return -1;

	ret = nuvolta_1665_download_fw_data(chip, fw_data, fw_data_length);
	release_firmware(fw);
	if (ret < 0) {
		dev_err(chip->dev, "[] nuvolta_1665_download_fw_data failed \n");
		return -1;
	}

	if (exit_key0(chip) < 0)
		return -1;

	if (exit_key1(chip) < 0)
		return -1;

	if (exit_key_open(chip) < 0)
		return -1;

	return 0;
}

static int fw_crc_chk(struct nuvolta_1665_chg *chip)
{
	u8 read_data = 0;

	if (rx1665_write(chip, 0x02, 0x0063) < 0)
		goto exit;
	msleep(100);
	if (rx1665_read(chip, &read_data, 0x0028) < 0)
		goto exit;
	if (read_data == 0x66) {
		dev_info(chip->dev, "fw crc chk good \n");
		return 0;
	}
	dev_info(chip->dev, "fw crc chk res %x \n", read_data);
exit:
	dev_err(chip->dev, "[%s] failed \n", __func__);
	return -1;
}

static int read_fw_version(struct nuvolta_1665_chg *chip, u8 *version)
{
	if (!version)
		return -1;

	*version = 0xFF;

	if (rx1665_read(chip, version, 0x002C) < 0) {
		dev_err(chip->dev, "[%s] failed \n", __func__);
		return -1;
	}
	dev_info(chip->dev, "fw chk version %x \n", *version);
	return 0;
}

static int nuvolta_1665_firmware_update_func(struct nuvolta_1665_chg *chip,
					     u8 cmd)
{
	int ret = 0;
	u8 fw_version = 0;

	//TODO1 disable reverse charge if it run
	chip->fw_update = true;
	//TODO2 sleep rx before start download and resume after
	nuvolta_1665_set_reverse_gpio(chip, true);
	msleep(100);

	ret = nuvolta_1665_check_i2c(chip);
	if (ret < 0)
		goto exit;

	switch (cmd) {
	case FW_UPDATE_CHECK:
		ret = nuvolta_1665_download_fw(chip, false, false);
		if (ret < 0) {
			dev_err(chip->dev, "[%s] fw download failed! cmd: %d\n",
				    __func__, cmd);
			goto exit;
		}
		break;
	case FW_UPDATE_FORCE:
		ret = nuvolta_1665_download_fw(chip, false, true);
		if (ret < 0) {
			dev_err(chip->dev, "[%s] fw download failed! cmd: %d\n",
				    __func__, cmd);
			goto exit;
		}
		break;
	case FW_UPDATE_FROM_BIN:
		//TODO add fw download from bin
		break;
	case FW_UPDATE_ERASE:
		//TODO add erase func
		break;
	case FW_UPDATE_AUTO:
		ret = nuvolta_1665_download_fw(chip, true, false);
		if (ret < 0) {
			dev_err(chip->dev, "[%s] fw download failed! cmd: %d\n",
				    __func__, cmd);
			goto exit;
		}
		break;
	default:
		dev_err(chip->dev, "[%s] unknown cmd: %d\n", __func__, cmd);
		break;
	}

	nuvolta_1665_set_reverse_gpio(chip, false);
	msleep(1000);
	nuvolta_1665_set_reverse_gpio(chip, true);

	msleep(100);

	if (fw_crc_chk(chip) < 0)
		ret = -1;
	else {
		if (read_fw_version(chip, &fw_version) < 0)
			ret = -1;
		else
			chip->fw_version = fw_version;
	}
	dev_info(chip->dev, "check fw version %x \n", chip->fw_version);
exit:
	chip->fw_update = false;
	nuvolta_1665_set_reverse_gpio(chip, false);
	return ret;
}

static ssize_t chip_firmware_update_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int cmd = 0, ret = 0;

	if (g_chip->fw_update) {
		dev_info(dev, "[%s] Firmware Update is on going!\n", __func__);
		return count;
	}

	cmd = (int)simple_strtoul(buf, NULL, 10);
	dev_info(dev, "[%s] value %d\n", __func__, cmd);

	if ((cmd > FW_UPDATE_NONE) && (cmd < FW_UPDATE_MAX)) {
		ret = nuvolta_1665_firmware_update_func(g_chip, cmd);
		if (ret < 0) {
			dev_err(dev, "[%s] Firmware Update:failed!\n", __func__);
			return count;
		} else {
			dev_info(dev, "[%s] Firmware Update:Success!\n",
				     __func__);
			return count;
		}
	} else {
		dev_err(dev, "[%s] Firmware Update:invalid cmd\n", __func__);
	}

	return count;
}

static ssize_t chip_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int check_result = 0;
	u8 default_FW_Ver = 0xFE;

	if (g_chip->fw_update) {
		dev_info(dev, "[%s] fw update going, can not show version\n",
			     __func__);
		return scnprintf(buf, PAGE_SIZE, "updating\n");
	} else {
		nuvolta_1665_set_reverse_gpio(g_chip, true);
		msleep(100);

		check_result = fw_crc_chk(g_chip);
		if (check_result >= 0) {
			read_fw_version(g_chip, &default_FW_Ver);
		}
		nuvolta_1665_set_reverse_gpio(g_chip, false);

		return scnprintf(buf, PAGE_SIZE, "fw_ver:%02x\n",
				 default_FW_Ver);
	}
}

static ssize_t chip_fw_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int ret = 0;

	if (g_chip->fw_update) {
		dev_info(dev, "[%s] Firmware Update is on going!\n", __func__);
		return snprintf(buf, PAGE_SIZE,
				"Firmware Update is on going!\n");
	}

	dev_info(dev, "[%s] Start fireware update process\n", __func__);
	ret = nuvolta_1665_firmware_update_func(g_chip, 2);
	if (ret < 0) {
		dev_err(dev, "[%s] Firmware Update:failed!\n", __func__);
		return snprintf(buf, PAGE_SIZE, "Firmware Update:Failed\n");
	} else {
		dev_info(dev, "[%s] Firmware Update:Success!\n", __func__);
		return snprintf(buf, PAGE_SIZE, "Firmware Update:Success\n");
	}
}

static DEVICE_ATTR(chip_vrect, S_IRUGO, chip_vrect_show, NULL);
static DEVICE_ATTR(chip_firmware_update, S_IWUSR, NULL,
		   chip_firmware_update_store);
static DEVICE_ATTR(chip_version, S_IRUGO, chip_version_show, NULL);
static DEVICE_ATTR(chip_vout, S_IWUSR | S_IRUGO, chip_vout_show,
		   chip_vout_store);
static DEVICE_ATTR(chip_iout, S_IRUGO, chip_iout_show, NULL);
static DEVICE_ATTR(chip_fw, S_IWUSR | S_IRUGO, chip_fw_show, NULL);

static struct attribute *rx1665_sysfs_attrs[] = {
	&dev_attr_chip_vrect.attr,
	&dev_attr_chip_version.attr,
	&dev_attr_chip_vout.attr,
	&dev_attr_chip_iout.attr,
	&dev_attr_chip_firmware_update.attr,
	&dev_attr_chip_fw.attr,
	NULL,
};

static const struct attribute_group rx1665_sysfs_group_attrs = {
	.attrs = rx1665_sysfs_attrs,
};

static int nuvolta_1665_set_enable_mode(struct nuvolta_1665_chg *chip,
					bool enable)
{
	int ret = 0;
	int gpio_enable_val = 0;
	int en = !!enable;

	if (gpio_is_valid(chip->enable_gpio)) {
		ret = gpio_request(chip->enable_gpio, "rx-enable-gpio");
		if (ret) {
			dev_err(chip->dev, "%s: unable to request enable gpio [%d]\n",
				    __func__, chip->enable_gpio);
		}

		ret = gpio_direction_output(chip->enable_gpio, !en);
		if (ret) {
			dev_err(chip->dev,
				"%s: cannot set direction for idt enable gpio [%d]\n",
				__func__, chip->enable_gpio);
		}
		gpio_enable_val = gpio_get_value(chip->enable_gpio);
		dev_info(chip->dev, "nuvolta enable gpio val is :%d\n",
			     gpio_enable_val);
		gpio_free(chip->enable_gpio);
	}

	return ret;
}
static enum power_supply_property nu1665_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int nu1665_get_prop(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct nuvolta_1665_chg *chip = power_supply_get_drvdata(psy);
	int temp = 0;
	int rc = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->is_reverse_mode || chip->is_boost_mode) {
			if (chip->reverse_pen_soc != 255) {
				last_valid_pen_soc = chip->reverse_pen_soc;
				val->intval = chip->reverse_pen_soc;
			} else {
				val->intval = last_valid_pen_soc;
				dev_err(chip->dev, "print last valid pen soc: %d\n",
					    val->intval);
			}
		} else {
			val->intval = last_valid_pen_soc;
			if (val->intval)
				dev_err(chip->dev, "report last valid pen soc: %d\n",
					    val->intval);
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (chip->is_reverse_mode || chip->is_boost_mode)
			val->intval = chip->reverse_vout;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (chip->is_reverse_mode || chip->is_boost_mode)
			val->intval = chip->reverse_iout;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (!chip->power_good_flag) {
			val->intval = 0;
			break;
		}
		rc = nuvolta_1665_get_temp(chip, &temp);
		if (rc > 0)
			val->intval = temp;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int nu1665_set_prop(struct power_supply *psy,
			   enum power_supply_property psp,
			   const union power_supply_propval *val)
{
	switch (psp) {
	default:
		return -EINVAL;
	}

	return 0;
}

static int nu1665_prop_is_writeable(struct power_supply *psy,
				    enum power_supply_property psp)
{
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		return 1;
	default:
		rc = 0;
		break;
	}

	return rc;
}

static const struct power_supply_desc nuvo_psy_desc = {
	.name = "fuda",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = nu1665_props,
	.num_properties = ARRAY_SIZE(nu1665_props),
	.get_property = nu1665_get_prop,
	.set_property = nu1665_set_prop,
	.property_is_writeable = nu1665_prop_is_writeable,
};

static void nuvolta_1665_init_detect_work(struct work_struct *work)
{
	struct nuvolta_1665_chg *chip = container_of(
		work, struct nuvolta_1665_chg, init_detect_work.work);
	int ret = 0;

	if (gpio_is_valid(chip->power_good_gpio)) {
		ret = gpio_get_value(chip->power_good_gpio);
		dev_info(chip->dev, "[%s]init power good: %d\n", __func__, ret);
		if (ret) {
			nuvolta_1665_set_enable_mode(chip, false);
			usleep_range(20000, 25000);
			nuvolta_1665_set_enable_mode(chip, true);
		}
	}
	return;
}

extern char *saved_command_line;


static const struct i2c_device_id nuvolta_1665_id[] = {
	{ "nuvolta_1665", 0 },
	{},
};

static int nuvolta_1665_probe(struct i2c_client *client)
{

	int ret = 0;
	int hall3_val = 1, hall4_val = 1;
	struct nuvolta_1665_chg *chip;

	struct power_supply_config nuvo_cfg = {};

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(chip->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->regmap =
		devm_regmap_init_i2c(client, &nuvolta_1665_regmap_config);
	if (IS_ERR(chip->regmap)) {
		dev_err(chip->dev, "failed to allocate register map\n");
		return PTR_ERR(chip->regmap);
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->fw_update = false;
	chip->fw_version = 0;
	chip->ss = 2;
	chip->wlsdev_name = NUVOLTA_1665_DRIVER_NAME;
	chip->chg_phase = NORMAL_MODE;
	g_chip = chip;
	chip->reverse_pen_soc = 255;

	device_init_wakeup(&client->dev, true);
	i2c_set_clientdata(client, chip);

	mutex_init(&chip->wireless_chg_int_lock);
	mutex_init(&chip->reverse_op_lock);

	INIT_DELAYED_WORK(&chip->wireless_int_work,
			  nuvolta_1665_wireless_int_work);
	INIT_DELAYED_WORK(&chip->wireless_pg_det_work,
			  nuvolta_1665_pg_det_work);
	INIT_DELAYED_WORK(&chip->chg_monitor_work, nuvolta_1665_monitor_work);
	INIT_DELAYED_WORK(&chip->reverse_chg_state_work,
			  reverse_chg_state_set_work);
	INIT_DELAYED_WORK(&chip->reverse_dping_state_work,
			  reverse_dping_state_set_work);
	INIT_DELAYED_WORK(&chip->init_detect_work,
			  nuvolta_1665_init_detect_work);
	INIT_DELAYED_WORK(&chip->delay_report_status_work,
			  nuvolta_1665_delay_report_status_work);
	INIT_DELAYED_WORK(&chip->rx_alarm_work, nuvolta_1665_rx_alarm_work);
	INIT_DELAYED_WORK(&chip->hall3_irq_work, nu1665_hall3_irq_work);
	INIT_DELAYED_WORK(&chip->hall4_irq_work, nu1665_hall4_irq_work);
	INIT_DELAYED_WORK(&chip->pen_notifier_work, pen_charge_notifier_work);
	pen_notifier_work = &chip->pen_notifier_work;
	INIT_DELAYED_WORK(&chip->reverse_chg_work, nu_reverse_chg_work);
	INIT_DELAYED_WORK(&chip->probe_fw_download_work,
			  nu1665_probe_fw_download_work);
	INIT_DELAYED_WORK(&chip->pen_check_work, pen_check_worker);

	ret = nuvolta_1665_parse_dt(chip);
	if (ret < 0) {
		dev_err(chip->dev, "device tree init is failed = %s\n", __func__);
		goto error_sysfs;
	}

	ret = nuvolta_rx1665_gpio_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "gpio init is failed = %s\n", __func__);
		goto error_sysfs;
	}

	ret = nuvolta_rx1665_irq_request(chip);
	if (ret < 0) {
		dev_err(chip->dev, "irq init is failed = %s\n", __func__);
		goto error_sysfs;
	}

	schedule_delayed_work(&chip->reverse_dping_state_work, 0);
	schedule_delayed_work(&chip->reverse_chg_state_work, 5000);

	ret = sysfs_create_group(&chip->dev->kobj, &rx1665_sysfs_group_attrs);
	if (ret < 0) {
		dev_err(chip->dev, "sysfs_create_group fail %d\n", ret);
		goto error_sysfs;
	}
	nuvo_cfg.drv_data = chip;
	chip->nuvo_psy =
		power_supply_register(chip->dev, &nuvo_psy_desc, &nuvo_cfg);

	/* reset wls charge when power good online */
	schedule_delayed_work(&chip->init_detect_work, msecs_to_jiffies(20000));

	if (gpio_is_valid(chip->hall3_gpio)) {
		hall3_val = gpio_get_value(chip->hall3_gpio);
		if (!hall3_val) {
			dev_info(chip->dev, "pen online, start reverse charge\n");
			chip->hall3_online = 1;
			schedule_delayed_work(&chip->hall3_irq_work,
					      msecs_to_jiffies(6000));
		}
	} else
		dev_err(chip->dev, "%s: hall3 gpio not provided\n", __func__);

	if (gpio_is_valid(chip->hall4_gpio)) {
		hall4_val = gpio_get_value(chip->hall4_gpio);
		if (!hall4_val) {
			dev_info(chip->dev, "pen online, start reverse charge\n");
			chip->hall4_online = 1;
			schedule_delayed_work(&chip->hall4_irq_work,
					      msecs_to_jiffies(6000));
		}
	} else
		dev_err(chip->dev, "%s: hall4 gpio not provided\n", __func__);

	if (!chip->power_off_mode)
		schedule_delayed_work(&chip->probe_fw_download_work, 10 * HZ);

	return 0;

error_sysfs:
	sysfs_remove_group(&chip->dev->kobj, &rx1665_sysfs_group_attrs);
	if (chip->irq_gpio > 0)
		gpio_free(chip->irq_gpio);
	if (chip->power_good_gpio > 0)
		gpio_free(chip->power_good_gpio);
	cancel_delayed_work_sync(&chip->hall3_irq_work);
	cancel_delayed_work_sync(&chip->hall4_irq_work);
	return 0;
}

static void nuvolta_1665_remove(struct i2c_client *client)
{
	struct nuvolta_1665_chg *chip = i2c_get_clientdata(client);

	if (chip->irq_gpio > 0)
		gpio_free(chip->irq_gpio);
	if (chip->power_good_gpio > 0)
		gpio_free(chip->power_good_gpio);
}

static int nuvolta_1665_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nuvolta_1665_chg *chip = i2c_get_clientdata(client);

	return enable_irq_wake(chip->irq);
}

static int nuvolta_1665_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nuvolta_1665_chg *chip = i2c_get_clientdata(client);

	return disable_irq_wake(chip->irq);
}

static const struct dev_pm_ops nuvolta_1665_pm_ops = {
	.suspend = nuvolta_1665_suspend,
	.resume = nuvolta_1665_resume,
};

static void nuvolta_1665_shutdown(struct i2c_client *client)
{
	struct nuvolta_1665_chg *chip = i2c_get_clientdata(client);

	if (chip->power_good_flag) {
		nuvolta_1665_set_enable_mode(chip, false);
		usleep_range(20000, 25000);
		nuvolta_1665_set_enable_mode(chip, true);
	}

	dev_info(chip->dev, "%s: shutdown: %s\n", __func__, chip->wlsdev_name);
	return;
}

static const struct of_device_id nuvolta_1665_match_table[] = {
	{
		.compatible = "nuvolta,rx1665",
	},
	{},
};

MODULE_DEVICE_TABLE(i2c, nuvolta_1665_id);

static struct i2c_driver nuvolta_1665_driver = {
	.driver		= {
		.name		= "nuvolta_1665",
		.owner		= THIS_MODULE,
		.of_match_table	= nuvolta_1665_match_table,
		.pm		= &nuvolta_1665_pm_ops,
	},
	.probe		= nuvolta_1665_probe,
	.remove		= nuvolta_1665_remove,
	.id_table	= nuvolta_1665_id,
	.shutdown	= nuvolta_1665_shutdown,
};

module_i2c_driver(nuvolta_1665_driver);

MODULE_AUTHOR("anxufeng <anxufeng@xiaomi.com>");
MODULE_DESCRIPTION("nuvolta wireless charge driver");
MODULE_LICENSE("GPL");
