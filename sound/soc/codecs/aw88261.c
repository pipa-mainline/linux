// SPDX-License-Identifier: GPL-2.0-only
//
// aw88261.c  --  AW88261 ALSA SoC Audio driver
//
// Copyright (c) 2023 awinic Technology CO., LTD
//
// Author: Jimmy Zhang <zhangjianming@awinic.com>
// Author: Weidong Wang <wangweidong.a@awinic.com>
//

#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include "aw88261.h"
#include "aw88395/aw88395_data_type.h"
#include "aw88395/aw88395_device.h"
#include <sound/pcm_params.h>

static const struct regmap_config aw88261_remap_config = {
	.val_bits = 16,
	.reg_bits = 8,
	.max_register = AW88261_REG_MAX,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static void aw88261_dev_set_volume(struct aw_device *aw_dev, unsigned int value)
{
	struct aw_volume_desc *vol_desc = &aw_dev->volume_desc;
	unsigned int real_value, volume;
	unsigned int reg_value;
	unsigned int volume_boost = 3; // Apply a 3dB boost to all volume settings

	/* Apply boosted volume - increase output level for Xiaomi Pad 6 */
	value = (value > volume_boost) ? (value - volume_boost) : 0;

	volume = min((value + vol_desc->init_volume), (unsigned int)AW88261_MUTE_VOL);
	real_value = DB_TO_REG_VAL(volume);

	regmap_read(aw_dev->regmap, AW88261_SYSCTRL2_REG, &reg_value);

	real_value = (real_value | (reg_value & AW88261_VOL_START_MASK));

	dev_dbg(aw_dev->dev, "value 0x%x , real_value:0x%x (boosted)", value, real_value);

	regmap_write(aw_dev->regmap, AW88261_SYSCTRL2_REG, real_value);
}

static void aw88261_dev_fade_in(struct aw_device *aw_dev)
{
	struct aw_volume_desc *desc = &aw_dev->volume_desc;
	int fade_in_vol = desc->ctl_volume;
	int fade_step = aw_dev->fade_step;
	int i;

	if (fade_step == 0 || aw_dev->fade_in_time == 0) {
		aw88261_dev_set_volume(aw_dev, fade_in_vol);
		return;
	}

	for (i = AW88261_MUTE_VOL; i >= fade_in_vol; i -= fade_step) {
		aw88261_dev_set_volume(aw_dev, i);
		usleep_range(aw_dev->fade_in_time,
					aw_dev->fade_in_time + 10);
	}

	if (i != fade_in_vol)
		aw88261_dev_set_volume(aw_dev, fade_in_vol);
}

static void aw88261_dev_fade_out(struct aw_device *aw_dev)
{
	struct aw_volume_desc *desc = &aw_dev->volume_desc;
	int fade_step = aw_dev->fade_step;
	int i;

	if (fade_step == 0 || aw_dev->fade_out_time == 0) {
		aw88261_dev_set_volume(aw_dev, AW88261_MUTE_VOL);
		return;
	}

	for (i = desc->ctl_volume; i <= AW88261_MUTE_VOL; i += fade_step) {
		aw88261_dev_set_volume(aw_dev, i);
		usleep_range(aw_dev->fade_out_time, aw_dev->fade_out_time + 10);
	}

	if (i != AW88261_MUTE_VOL) {
		aw88261_dev_set_volume(aw_dev, AW88261_MUTE_VOL);
		usleep_range(aw_dev->fade_out_time, aw_dev->fade_out_time + 10);
	}
}

static void aw88261_dev_i2s_tx_enable(struct aw_device *aw_dev, bool flag)
{
	// if (flag)
	//	regmap_update_bits(aw_dev->regmap, AW88261_I2SCFG1_REG,
	//		~AW88261_I2STXEN_MASK, AW88261_I2STXEN_ENABLE_VALUE);
	// else
	//	regmap_update_bits(aw_dev->regmap, AW88261_I2SCFG1_REG,
	//		~AW88261_I2STXEN_MASK, AW88261_I2STXEN_DISABLE_VALUE);
}

static void aw88261_dev_pwd(struct aw_device *aw_dev, bool pwd)
{
	if (pwd)
		regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_PWDN_MASK, AW88261_PWDN_POWER_DOWN_VALUE);
	else
		regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_PWDN_MASK, AW88261_PWDN_WORKING_VALUE);
}

static void aw88261_dev_amppd(struct aw_device *aw_dev, bool amppd)
{
	int ret;
	if (amppd)
		ret = regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_AMPPD_MASK, AW88261_AMPPD_POWER_DOWN_VALUE);
	else
		ret = regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_AMPPD_MASK, AW88261_AMPPD_WORKING_VALUE);
}

static void aw88261_dev_mute(struct aw_device *aw_dev, bool is_mute)
{
	if (is_mute) {
		aw88261_dev_fade_out(aw_dev);
		regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_HMUTE_MASK, AW88261_HMUTE_ENABLE_VALUE);
	} else {
		regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_HMUTE_MASK, AW88261_HMUTE_DISABLE_VALUE);
		aw88261_dev_fade_in(aw_dev);
	}
}

static void aw88261_dev_clear_int_status(struct aw_device *aw_dev)
{
	unsigned int int_status;

	/* read int status and clear */
	regmap_read(aw_dev->regmap, AW88261_SYSINT_REG, &int_status);
	/* make sure int status is clear */
	regmap_read(aw_dev->regmap, AW88261_SYSINT_REG, &int_status);

	dev_dbg(aw_dev->dev, "read interrupt reg = 0x%04x", int_status);
}

static int aw88261_dev_get_iis_status(struct aw_device *aw_dev)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(aw_dev->regmap, AW88261_SYSST_REG, &reg_val);
	if (ret)
		return ret;

	bool pll_lock = (reg_val & (1 << 0)) == 0x0001;
	bool clks_available = (reg_val & (1 << 5)) == 0x0000;

	if (pll_lock && clks_available) {
		dev_dbg(aw_dev->dev, "IIS signal is OK");
		ret = 0;
	} else {
		dev_err(aw_dev->dev,
			"IIS signal is not OK, pll_lock:%d, clks_available:%d",
			pll_lock, clks_available);
		ret = -EINVAL;
	}
	return ret;
}

static int aw88261_dev_check_mode1_pll(struct aw_device *aw_dev)
{
	int ret, i;

	for (i = 0; i < AW88261_DEV_SYSST_CHECK_MAX; i++) {
		ret = aw88261_dev_get_iis_status(aw_dev);
		if (ret) {
			dev_err(aw_dev->dev, "mode1 iis signal check error");
			usleep_range(AW88261_2000_US, AW88261_2000_US + 10);
		} else {
			return ret;
		}
	}

	return -EPERM;
}

static int aw88261_dev_check_syspll(struct aw_device *aw_dev)
{
	int ret;
	int retry_count = 0;
	const int max_retries = 20; // Increase retries for PLL stabilization

	while (retry_count < max_retries) {
		ret = aw88261_dev_check_mode1_pll(aw_dev);
		if (ret != 0)
			retry_count++;
		else
			break;
	}

	if (retry_count == max_retries) {
		dev_err(aw_dev->dev,
			"Failed to stabilize PLL after %d attempts, we're cooked", max_retries);
		return -ETIMEDOUT;
	}
	if (ret == 0) {
		dev_info(aw_dev->dev, "PLL stabilized successfully");
	}

	return ret;
}

static int aw88261_dev_check_sysst(struct aw_device *aw_dev)
{
	unsigned int check_val;
	unsigned int reg_val;
	int ret, i;

	for (i = 0; i < AW88261_DEV_SYSST_CHECK_MAX; i++) {
		ret = regmap_read(aw_dev->regmap, AW88261_SYSST_REG, &reg_val);
		if (ret)
			return ret;

		check_val = reg_val & (~AW88261_BIT_SYSST_CHECK_MASK)
							& AW88261_BIT_SYSST_CHECK;
		if (check_val != AW88261_BIT_SYSST_CHECK) {
			dev_err(aw_dev->dev, "check sysst fail, reg_val=0x%04x, check:0x%x",
				reg_val, AW88261_BIT_SYSST_CHECK);
			usleep_range(AW88261_2000_US, AW88261_2000_US + 10);
		} else {
			dev_err(aw_dev->dev, "check sysst ok, reg_val=0x%04x",
				reg_val, AW88261_BIT_SYSST_CHECK);
			return 0;
		}
	}

	return -EPERM;
}

static void aw88261_dev_uls_hmute(struct aw_device *aw_dev, bool uls_hmute)
{
	if (uls_hmute)
		regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_ULS_HMUTE_MASK,
				AW88261_ULS_HMUTE_ENABLE_VALUE);
	else
		regmap_update_bits(aw_dev->regmap, AW88261_SYSCTRL_REG,
				~AW88261_ULS_HMUTE_MASK,
				AW88261_ULS_HMUTE_DISABLE_VALUE);
}

static int aw88261_dev_reg_update(struct aw88261 *aw88261, unsigned char *data, unsigned int len)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	struct aw_volume_desc *vol_desc = &aw_dev->volume_desc;
	unsigned int read_val, efcheck_val, read_vol;
	struct device_node *np = aw_dev->dev->of_node;

	int data_len, i, ret;
	int16_t *reg_data;
	u16 reg_val;
	u8 reg_addr;

	if (!len || !data) {
		dev_err(aw_dev->dev, "reg data is null or len is 0");
		return -EINVAL;
	}

	reg_data = (int16_t *)data;
	data_len = len >> 1;

	if (data_len & 0x1) {
		dev_err(aw_dev->dev, "data len:%d unsupported",	data_len);
		return -EINVAL;
	}

	for (i = 0; i < data_len; i += 2) {
		reg_addr = reg_data[i];
		reg_val = reg_data[i + 1];

		switch (reg_addr) {
		case AW88261_ID_REG:
		case AW88261_SYSST_REG:
		case AW88261_SYSINT_REG:
		case AW88261_SYSINTM_REG:
		case AW88261_SYSCTRL_REG:
		case AW88261_SYSCTRL2_REG:
		case AW88261_I2SCTRL1_REG:
		case AW88261_I2SCTRL2_REG:
		case AW88261_I2SCTRL3_REG:
		case AW88261_DACCFG1_REG:
		case AW88261_DACCFG2_REG:
		case AW88261_DACCFG3_REG:
		case AW88261_DACCFG4_REG:
		case AW88261_DACST_REG:
		case AW88261_VBAT_REG:
		case AW88261_TEMP_REG:
		case AW88261_PVDD_REG:
		case AW88261_BSTCTRL1_REG:
		case AW88261_BSTCTRL2_REG:
			break;
		default:
			continue;
		}

		if (reg_addr == AW88261_SYSCTRL_REG) {
			aw88261->amppd_st = reg_val & (~AW88261_AMPPD_MASK);
			ret = regmap_read(aw_dev->regmap, reg_addr, &read_val);
			if (ret)
				break;

			read_val &= (~AW88261_AMPPD_MASK) | (~AW88261_PWDN_MASK) |
								(~AW88261_HMUTE_MASK);
			reg_val &= (AW88261_AMPPD_MASK | AW88261_PWDN_MASK | AW88261_HMUTE_MASK);
			reg_val |= read_val;

			/* enable uls hmute */
			reg_val &= AW88261_ULS_HMUTE_MASK;
			reg_val |= AW88261_ULS_HMUTE_ENABLE_VALUE;

			reg_val = 0b0011001001000000;
		}

		/* Special handling for I2SCTRL registers for Xiaomi Pad 6 */
		if (reg_addr == AW88261_I2SCTRL1_REG &&
			(of_machine_is_compatible("xiaomi,pipa") ||
			of_machine_is_compatible("qcom,sm8250-mtp"))) {
			//reg_val &= ~(0x3 << 0); /* Clear I2S format bits */
			//reg_val |= (0x1 << 0); /* Set TDM format */

			reg_val = 0b000010011101000; // 0b000_01_00_11_10_1000;
			dev_dbg(aw_dev->dev, "Setting I2S mode ");
		}

		if (reg_addr == AW88261_I2SCTRL2_REG &&
			(of_machine_is_compatible("xiaomi,pipa") ||
			of_machine_is_compatible("qcom,sm8250-mtp"))) {
			/* Force TDM mode */
			//reg_val &= ~(0x3 << 0); /* Clear I2S format bits */
			//reg_val |= (0x1 << 0); /* Set TDM format */

			u32 slot_num = 0;

			of_property_read_u32(np, "rx_slot", &slot_num);
			// slot_num = 0;
			reg_val = 0b0101000000000000 | (slot_num << 4) | (slot_num); // 0b0101_0000_0000_0000;
			dev_info(
				aw_dev->dev,
				"Setting TDM mode for Xiaomi Pad 6, channel: %d",
				slot_num);
		}

		/* i2stxen */
		if (reg_addr == AW88261_I2SCTRL3_REG) {
			/* close tx */
			// reg_val &= AW88261_I2STXEN_MASK;
			// reg_val |= AW88261_I2STXEN_DISABLE_VALUE;
			reg_val = 0b0000000011110110; // 0b00000000_0_0_0_1_0_0_1_0;
		}

		if (reg_addr == AW88261_SYSCTRL2_REG) {
			read_vol = (reg_val & (~AW88261_VOL_MASK)) >>
				AW88261_VOL_START_BIT;
			aw_dev->volume_desc.init_volume =
				REG_VAL_TO_DB(read_vol);
		}

		ret = regmap_write(aw_dev->regmap, reg_addr, reg_val);
		if (ret)
			break;
	}

	if (aw_dev->prof_cur != aw_dev->prof_index)
		vol_desc->ctl_volume = 0;

	/* keep min volume */
	aw88261_dev_set_volume(aw_dev, vol_desc->mute_volume);

	return ret;
}

static int aw88261_dev_get_prof_name(struct aw_device *aw_dev, int index, char **prof_name)
{
	struct aw_prof_info *prof_info = &aw_dev->prof_info;
	struct aw_prof_desc *prof_desc;

	if ((index >= aw_dev->prof_info.count) || (index < 0)) {
		dev_err(aw_dev->dev, "index[%d] overflow count[%d]",
			index, aw_dev->prof_info.count);
		return -EINVAL;
	}

	prof_desc = &aw_dev->prof_info.prof_desc[index];

	*prof_name = prof_info->prof_name_list[prof_desc->id];

	return 0;
}

static int aw88261_dev_get_prof_data(struct aw_device *aw_dev, int index,
			struct aw_prof_desc **prof_desc)
{
	if ((index >= aw_dev->prof_info.count) || (index < 0)) {
		dev_err(aw_dev->dev, "%s: index[%d] overflow count[%d]\n",
				__func__, index, aw_dev->prof_info.count);
		return -EINVAL;
	}

	*prof_desc = &aw_dev->prof_info.prof_desc[index];

	return 0;
}

static int aw88261_dev_fw_update(struct aw88261 *aw88261)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	struct aw_prof_desc *prof_index_desc;
	struct aw_sec_data_desc *sec_desc;
	char *prof_name;
	int ret;

	ret = aw88261_dev_get_prof_name(aw_dev, aw_dev->prof_index, &prof_name);
	if (ret) {
		dev_err(aw_dev->dev, "get prof name failed");
		return -EINVAL;
	}

	dev_dbg(aw_dev->dev, "start update %s", prof_name);

	ret = aw88261_dev_get_prof_data(aw_dev, aw_dev->prof_index, &prof_index_desc);
	if (ret)
		return ret;

	/* update reg */
	sec_desc = prof_index_desc->sec_desc;
	ret = aw88261_dev_reg_update(aw88261, sec_desc[AW88395_DATA_TYPE_REG].data,
					sec_desc[AW88395_DATA_TYPE_REG].len);
	if (ret) {
		dev_err(aw_dev->dev, "update reg failed");
		return ret;
	}

	aw_dev->prof_cur = aw_dev->prof_index;

	return ret;
}

static int aw88261_dev_start(struct aw88261 *aw88261)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	int ret;

	if (aw_dev->status == AW88261_DEV_PW_ON) {
		dev_info(aw_dev->dev, "already power on");
		return 0;
	}

	/* power on */
	aw88261_dev_pwd(aw_dev, false);
	usleep_range(AW88261_2000_US, AW88261_2000_US + 10);

	ret = aw88261_dev_check_syspll(aw_dev);
	if (ret) {
		dev_err(aw_dev->dev, "pll check failed cannot start");
		goto pll_check_fail;
	}

	/* amppd on */
	aw88261_dev_amppd(aw_dev, false);
	usleep_range(AW88261_1000_US, AW88261_1000_US + 50);

	/* check i2s status */
	ret = aw88261_dev_check_sysst(aw_dev);
	if (ret) {
		dev_err(aw_dev->dev, "sysst check failed");
		goto sysst_check_fail;
	}

	/* enable tx feedback */
	aw88261_dev_i2s_tx_enable(aw_dev, true);

	if (aw88261->amppd_st)
		aw88261_dev_amppd(aw_dev, true);

	/* close uls mute */
	aw88261_dev_uls_hmute(aw_dev, false);

	/* close mute */
	if (!aw88261->mute_st)
		aw88261_dev_mute(aw_dev, false);

	/* clear inturrupt */
	aw88261_dev_clear_int_status(aw_dev);
	aw_dev->status = AW88261_DEV_PW_ON;

	return 0;

sysst_check_fail:
	aw88261_dev_i2s_tx_enable(aw_dev, false);
	aw88261_dev_clear_int_status(aw_dev);
	aw88261_dev_amppd(aw_dev, true);
pll_check_fail:
	aw88261_dev_pwd(aw_dev, true);
	aw_dev->status = AW88261_DEV_PW_OFF;

	return ret;
}

static int aw88261_dev_stop(struct aw_device *aw_dev)
{
	if (aw_dev->status == AW88261_DEV_PW_OFF) {
		dev_dbg(aw_dev->dev, "already power off");
		return 0;
	}

	aw_dev->status = AW88261_DEV_PW_OFF;

	/* clear inturrupt */
	aw88261_dev_clear_int_status(aw_dev);

	aw88261_dev_uls_hmute(aw_dev, true);
	/* set mute */
	aw88261_dev_mute(aw_dev, true);

	/* close tx feedback */
	aw88261_dev_i2s_tx_enable(aw_dev, false);
	usleep_range(AW88261_1000_US, AW88261_1000_US + 100);

	/* enable amppd */
	aw88261_dev_amppd(aw_dev, true);

	/* set power down */
	aw88261_dev_pwd(aw_dev, true);

	return 0;
}

static int aw88261_reg_update(struct aw88261 *aw88261, bool force)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	int ret;

	if (force) {
		ret = regmap_write(aw_dev->regmap,
					AW88261_ID_REG, AW88261_SOFT_RESET_VALUE);
		if (ret)
			return ret;

		ret = aw88261_dev_fw_update(aw88261);
		if (ret)
			return ret;
	} else {
		if (aw_dev->prof_cur != aw_dev->prof_index) {
			ret = aw88261_dev_fw_update(aw88261);
			if (ret)
				return ret;
		} else {
			ret = 0;
		}
	}

	aw_dev->prof_cur = aw_dev->prof_index;

	return ret;
}

static void aw88261_start_pa(struct aw88261 *aw88261)
{
	int ret, i;

	for (i = 0; i < AW88261_START_RETRIES; i++) {
		ret = aw88261_reg_update(aw88261, aw88261->phase_sync);
		if (ret) {
			dev_err(aw88261->aw_pa->dev, "fw update failed, cnt:%d\n", i);
			continue;
		}
		ret = aw88261_dev_start(aw88261);
		if (ret) {
			dev_err(aw88261->aw_pa->dev, "aw88261 device start failed. retry = %d", i);
			continue;
		} else {
			dev_info(aw88261->aw_pa->dev, "start success\n");
			break;
		}
	}
}

static void aw88261_startup_work(struct work_struct *work)
{
	struct aw88261 *aw88261 =
		container_of(work, struct aw88261, start_work.work);

	mutex_lock(&aw88261->lock);
	aw88261_start_pa(aw88261);
	mutex_unlock(&aw88261->lock);
}

static void aw88261_start(struct aw88261 *aw88261, bool sync_start)
{
	if (aw88261->aw_pa->fw_status != AW88261_DEV_FW_OK)
		return;

	if (aw88261->aw_pa->status == AW88261_DEV_PW_ON)
		return;

	if (sync_start == AW88261_SYNC_START)
		aw88261_start_pa(aw88261);
	else
		queue_delayed_work(system_wq,
			&aw88261->start_work,
			AW88261_START_WORK_DELAY_MS);
}

static int aw88261_dai_set_stream(struct snd_soc_dai *dai, void *sdw_stream, int direction)
{
	snd_soc_dai_dma_data_set(dai, direction, sdw_stream);

	return 0;
}

static int aw88261_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(dai->component);

	dev_info(aw88261->aw_pa->dev, "sysclk = %d\n", freq);

	aw88261->sysclk = freq;

	return 0;
}

static int aw88261_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(dai->component);

	dev_info(aw88261->aw_pa->dev, "fmt = 0x%x\n", fmt);

	return 0;
}

static int aw88261_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(component);
	unsigned int mclk = aw88261->sysclk;
	unsigned int rate = params_rate(params);
	unsigned int fmt = params_format(params);

	dev_info(aw88261->aw_pa->dev, "hw_params: mclk=%d, rate=%d, format=%x\n",
		mclk, rate, fmt);

	/* No specific I2S format or rate configurations are applied here.
	 * The driver relies on default configurations or configurations
	 * applied elsewhere in the system.
	 *
	 * If specific configurations are needed, they should be implemented here,
	 * setting up the DAI based on the parameters.
	 */

	dev_info(aw88261->aw_pa->dev, "Stream direction: %s",
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ? "Capture" : "Playback");

	dev_info(aw88261->aw_pa->dev, "Requested rate: %d Hz, sample size: %d bits",
		params_rate(params), params_width(params));

	return 0;
}

static int aw88261_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	// aw_snd_soc_codec_t *codec = dai->component;

	// struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		// handle playback
	} else {
		// handle capture
	}

	return 0;
}

static const struct snd_soc_dai_ops aw88261_dai_ops = {
	.startup = aw88261_startup,
	.set_stream = aw88261_dai_set_stream,
	.set_sysclk = aw88261_dai_set_sysclk,
	.set_fmt = aw88261_dai_set_fmt,
	.hw_params = aw88261_hw_params,
};

static struct snd_soc_dai_driver aw88261_dai[] = {
	{
		.name = "aw88261-aif",
		.id = 1,
		.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = AW88261_RATES,
			.formats = AW88261_FORMATS,
		},
		.capture = {
			.stream_name = "Speaker_Capture",
			.channels_min = 1,
			.channels_max = 1,
			.rates = AW88261_RATES,
			.formats = AW88261_FORMATS,
		},
		.ops = &aw88261_dai_ops
	},
};

static int aw88261_get_fade_in_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(component);
	struct aw_device *aw_dev = aw88261->aw_pa;

	ucontrol->value.integer.value[0] = aw_dev->fade_in_time;

	return 0;
}

static int aw88261_set_fade_in_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct aw_device *aw_dev = aw88261->aw_pa;
	int time;

	time = ucontrol->value.integer.value[0];

	if (time < mc->min || time > mc->max)
		return -EINVAL;

	if (time != aw_dev->fade_in_time) {
		aw_dev->fade_in_time = time;
		return 1;
	}

	return 0;
}

static int aw88261_get_fade_out_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(component);
	struct aw_device *aw_dev = aw88261->aw_pa;

	ucontrol->value.integer.value[0] = aw_dev->fade_out_time;

	return 0;
}

static int aw88261_set_fade_out_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct aw_device *aw_dev = aw88261->aw_pa;
	int time;

	time = ucontrol->value.integer.value[0];
	if (time < mc->min || time > mc->max)
		return -EINVAL;

	if (time != aw_dev->fade_out_time) {
		aw_dev->fade_out_time = time;
		return 1;
	}

	return 0;
}

static int aw88261_dev_set_profile_index(struct aw_device *aw_dev, int index)
{
	/* check the index whether is valid */
	if ((index >= aw_dev->prof_info.count) || (index < 0))
		return -EINVAL;
	/* check the index whether change */
	if (aw_dev->prof_index == index)
		return -EPERM;

	aw_dev->prof_index = index;

	return 0;
}

static int aw88261_profile_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);
	char *prof_name;
	int count, ret;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	count = aw88261->aw_pa->prof_info.count;
	if (count <= 0) {
		uinfo->value.enumerated.items = 0;
		return 0;
	}

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	count = uinfo->value.enumerated.item;

	ret = aw88261_dev_get_prof_name(aw88261->aw_pa, count, &prof_name);
	if (ret) {
		strscpy(uinfo->value.enumerated.name, "null");
		return 0;
	}

	strscpy(uinfo->value.enumerated.name, prof_name);

	return 0;
}

static int aw88261_profile_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw88261->aw_pa->prof_index;

	return 0;
}

static int aw88261_profile_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);
	int ret;

	/* pa stop or stopping just set profile */
	mutex_lock(&aw88261->lock);
	ret = aw88261_dev_set_profile_index(aw88261->aw_pa, ucontrol->value.integer.value[0]);
	if (ret) {
		dev_dbg(codec->dev, "profile index does not change");
		mutex_unlock(&aw88261->lock);
		return 0;
	}

	if (aw88261->aw_pa->status) {
		aw88261_dev_stop(aw88261->aw_pa);
		aw88261_start(aw88261, AW88261_SYNC_START);
	}

	mutex_unlock(&aw88261->lock);

	return 1;
}

static int aw88261_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);
	struct aw_volume_desc *vol_desc = &aw88261->aw_pa->volume_desc;

	ucontrol->value.integer.value[0] = vol_desc->ctl_volume;

	return 0;
}

static int aw88261_volume_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);
	struct aw_volume_desc *vol_desc = &aw88261->aw_pa->volume_desc;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int value;

	value = ucontrol->value.integer.value[0];

	if (value < mc->min || value > mc->max)
		return -EINVAL;

	if (vol_desc->ctl_volume != value) {
		vol_desc->ctl_volume = value;
		aw88261_dev_set_volume(aw88261->aw_pa, vol_desc->ctl_volume);

		return 1;
	}

	return 0;
}

static int aw88261_get_fade_step(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw88261->aw_pa->fade_step;

	return 0;
}

static int aw88261_set_fade_step(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int value;

	value = ucontrol->value.integer.value[0];
	if (value < mc->min || value > mc->max)
		return -EINVAL;

	if (aw88261->aw_pa->fade_step != value) {
		aw88261->aw_pa->fade_step = value;
		return 1;
	}

	return 0;
}

/* Add unique suffix to control names based on codec channel */
static char *aw88261_append_suffix(struct aw88261 *aw88261, const char *name)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	char *new_name;
	int len;

	if (!name)
		return NULL;

	len = strlen(name) + 8; /* Extra space for "-chX" suffix */
	new_name = devm_kzalloc(aw_dev->dev, len, GFP_KERNEL);
	if (!new_name)
		return (char *)name; /* Fall back to original name */

	snprintf(new_name, len, "%s-ch%d", name, aw_dev->channel);
	return new_name;
}

static const struct snd_kcontrol_new aw88261_controls[] = {
	SOC_SINGLE_EXT("PCM Playback Volume", AW88261_SYSCTRL2_REG,
		6, AW88261_MUTE_VOL, 0, aw88261_volume_get,
		aw88261_volume_set),
	SOC_SINGLE_EXT("Fade Step", 0, 0, AW88261_MUTE_VOL, 0,
		aw88261_get_fade_step, aw88261_set_fade_step),
	SOC_SINGLE_EXT("Volume Ramp Up Step", 0, 0, FADE_TIME_MAX, FADE_TIME_MIN,
		aw88261_get_fade_in_time, aw88261_set_fade_in_time),
	SOC_SINGLE_EXT("Volume Ramp Down Step", 0, 0, FADE_TIME_MAX, FADE_TIME_MIN,
		aw88261_get_fade_out_time, aw88261_set_fade_out_time),
	AW88261_PROFILE_EXT("Profile Set", aw88261_profile_info,
		aw88261_profile_get, aw88261_profile_set),
};

static int aw88261_playback_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(component);

	mutex_lock(&aw88261->lock);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		aw88261_start(aw88261, AW88261_ASYNC_START);
		break;
	case SND_SOC_DAPM_POST_PMD:
		aw88261_dev_stop(aw88261->aw_pa);
		break;
	default:
		break;
	}
	mutex_unlock(&aw88261->lock);

	return 0;
}

static const struct snd_soc_dapm_widget aw88261_dapm_widgets[] = {
	 /* playback */
	SND_SOC_DAPM_AIF_IN_E("AIF_RX", "Speaker_Playback", 0, 0, 0, 0,
					aw88261_playback_event,
					SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("DAC Output"),

	/* capture */
	SND_SOC_DAPM_AIF_OUT("AIF_TX", "Speaker_Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("ADC Input"),
};

static const struct snd_soc_dapm_route aw88261_audio_map[] = {
	{"DAC Output", NULL, "AIF_RX"},
	{"AIF_TX", NULL, "ADC Input"},
};

static int aw88261_dev_init(struct aw88261 *aw88261, struct aw_container *aw_cfg)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	int ret;

	ret = aw88395_dev_cfg_load(aw_dev, aw_cfg);
	if (ret) {
		dev_err(aw_dev->dev, "aw_dev acf parse failed");
		return -EINVAL;
	}

	ret = regmap_write(aw_dev->regmap, AW88261_ID_REG, AW88261_SOFT_RESET_VALUE);
	if (ret)
		return ret;

	aw_dev->fade_in_time = AW88261_500_US;
	aw_dev->fade_out_time = AW88261_500_US;
	aw_dev->prof_cur = AW88261_INIT_PROFILE;
	aw_dev->prof_index = AW88261_INIT_PROFILE;

	ret = aw88261_dev_fw_update(aw88261);
	if (ret) {
		dev_err(aw_dev->dev, "fw update failed ret = %d\n", ret);
		return ret;
	}

	aw88261_dev_clear_int_status(aw_dev);

	aw88261_dev_uls_hmute(aw_dev, true);

	aw88261_dev_mute(aw_dev, true);

	aw88261_dev_i2s_tx_enable(aw_dev, false);

	usleep_range(AW88261_1000_US, AW88261_1000_US + 100);

	aw88261_dev_amppd(aw_dev, true);

	aw88261_dev_pwd(aw_dev, true);

	return 0;
}

static int aw88261_request_firmware_file(struct aw88261 *aw88261)
{
	const struct firmware *cont = NULL;
	int ret;

	aw88261->aw_pa->fw_status = AW88261_DEV_FW_FAILED;

	ret = request_firmware(&cont, aw88261->fw_name, aw88261->aw_pa->dev);
	if (ret)
		return dev_err_probe(aw88261->aw_pa->dev, ret,
					"load [%s] failed!", aw88261->fw_name);

	dev_info(aw88261->aw_pa->dev, "loaded %s - size: %zu\n",
			aw88261->fw_name, cont ? cont->size : 0);

	aw88261->aw_cfg = devm_kzalloc(aw88261->aw_pa->dev, cont->size + sizeof(int), GFP_KERNEL);
	if (!aw88261->aw_cfg) {
		release_firmware(cont);
		return -ENOMEM;
	}
	aw88261->aw_cfg->len = (int)cont->size;
	memcpy(aw88261->aw_cfg->data, cont->data, cont->size);
	release_firmware(cont);

	ret = aw88395_dev_load_acf_check(aw88261->aw_pa, aw88261->aw_cfg);
	if (ret) {
		dev_err(aw88261->aw_pa->dev, "load [%s] failed !", aw88261->fw_name);
		return ret;
	}

	mutex_lock(&aw88261->lock);
	/* aw device init */
	ret = aw88261_dev_init(aw88261, aw88261->aw_cfg);
	if (ret)
		dev_err(aw88261->aw_pa->dev, "dev init failed");
	mutex_unlock(&aw88261->lock);

	return ret;
}

static int aw88261_codec_probe(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(component);
	int ret;
	int i;
	struct snd_kcontrol_new *controls_copy;
	char *new_name;
	int num_controls = ARRAY_SIZE(aw88261_controls);

	INIT_DELAYED_WORK(&aw88261->start_work, aw88261_startup_work);

	/* Log device info for debugging */
	dev_info(component->dev,
		 "Probing AW88261 codec for Xiaomi Pad 6 (channel %d)",
		 aw88261->aw_pa->channel);

	ret = aw88261_request_firmware_file(aw88261);
	if (ret)
		return dev_err_probe(aw88261->aw_pa->dev, ret,
				"aw88261_request_firmware_file failed\n");

	/* add widgets with unique names for each channel */
	ret = snd_soc_dapm_new_controls(dapm, aw88261_dapm_widgets,
							ARRAY_SIZE(aw88261_dapm_widgets));
	dev_info(component->dev, "Widget return status: %d\n", ret);
	if (ret)
		return ret;

	/* add route with unique widget names */

	ret = snd_soc_dapm_add_routes(dapm, aw88261_audio_map,
							ARRAY_SIZE(aw88261_audio_map));
	dev_info(component->dev, "Route return status: %d\n", ret);
	if (ret)
		return ret;

	/* Create channel-specific control names to avoid conflicts */
	controls_copy = devm_kmemdup(component->dev, aw88261_controls,
					sizeof(aw88261_controls), GFP_KERNEL);
	if (!controls_copy)
		return -ENOMEM;

	/* Add channel suffix to each control name */
	for (i = 0; i < num_controls; i++) {
		new_name =
			aw88261_append_suffix(aw88261, controls_copy[i].name);
		controls_copy[i].name = new_name;
	}

	ret = snd_soc_add_component_controls(component, controls_copy, num_controls);
	if (ret) {
		dev_err(component->dev, "Failed to add controls: %d\n", ret);
		return ret;
	}

	/* Apply Xiaomi Pad 6 specific optimizations */
	if (of_machine_is_compatible("xiaomi,pipa") ||
		of_machine_is_compatible("qcom,sm8250-mtp")) {
		dev_info(
			component->dev,
			"Applying Xiaomi Pad 6 specific optimizations for channel %d",
			aw88261->aw_pa->channel);
	}

	return ret;
}

static void aw88261_codec_remove(struct snd_soc_component *aw_codec)
{
	struct aw88261 *aw88261 = snd_soc_component_get_drvdata(aw_codec);

	cancel_delayed_work_sync(&aw88261->start_work);
}

static const struct snd_soc_component_driver soc_codec_dev_aw88261 = {
	.probe = aw88261_codec_probe,
	.remove = aw88261_codec_remove,
};

static void aw88261_hw_reset(struct aw88261 *aw88261)
{
	gpiod_set_value_cansleep(aw88261->reset_gpio, 0);
	usleep_range(AW88261_1000_US, AW88261_1000_US + 10);
	gpiod_set_value_cansleep(aw88261->reset_gpio, 1);
	usleep_range(AW88261_1000_US, AW88261_1000_US + 10);
}

static void aw88261_parse_channel_dt(struct aw88261 *aw88261)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	struct device_node *np = aw_dev->dev->of_node;
	u32 channel_value = AW88261_DEV_DEFAULT_CH;

	of_property_read_u32(np, "awinic,audio-channel", &channel_value);
	aw88261->phase_sync = of_property_read_bool(np, "awinic,sync-flag");

	aw_dev->channel = channel_value;
}

static void aw88261_parse_firmware_name_dt(struct aw88261 *aw88261)
{
	struct aw_device *aw_dev = aw88261->aw_pa;
	struct device_node *np = aw_dev->dev->of_node;
	int ret;

	ret = of_property_read_string(np, "firmware-name", &aw88261->fw_name);
	if (ret) {
		dev_info(aw_dev->dev, "firmware-name is not defined, failing back to default value\n");
		aw88261->fw_name = AW88261_ACF_FILE;
	}
}

static int aw88261_init(struct aw88261 **aw88261, struct i2c_client *i2c, struct regmap *regmap)
{
	struct aw_device *aw_dev;
	unsigned int chip_id;
	int ret;

	/* read chip id */
	ret = regmap_read(regmap, AW88261_ID_REG, &chip_id);
	if (ret) {
		dev_err(&i2c->dev, "%s read chipid error. ret = %d", __func__, ret);
		return ret;
	}
	if (chip_id != AW88261_CHIP_ID) {
		dev_err(&i2c->dev, "unsupported device");
		return -ENXIO;
	}

	dev_info(&i2c->dev, "chip id = %x\n", chip_id);

	aw_dev = devm_kzalloc(&i2c->dev, sizeof(*aw_dev), GFP_KERNEL);
	if (!aw_dev)
		return -ENOMEM;

	(*aw88261)->aw_pa = aw_dev;
	aw_dev->i2c = i2c;
	aw_dev->regmap = regmap;
	aw_dev->dev = &i2c->dev;
	aw_dev->chip_id = AW88261_CHIP_ID;
	aw_dev->acf = NULL;
	aw_dev->prof_info.prof_desc = NULL;
	aw_dev->prof_info.count = 0;
	aw_dev->prof_info.prof_type = AW88395_DEV_NONE_TYPE_ID;
	aw_dev->channel = 0;
	aw_dev->fw_status = AW88261_DEV_FW_OK;
	aw_dev->fade_step = AW88261_VOLUME_STEP_DB;
	aw_dev->volume_desc.ctl_volume = AW88261_VOL_DEFAULT_VALUE;
	aw_dev->volume_desc.mute_volume = AW88261_MUTE_VOL;
	aw88261_parse_channel_dt(*aw88261);
	aw88261_parse_firmware_name_dt(*aw88261);

	return ret;
}

static int aw88261_i2c_probe(struct i2c_client *i2c)
{
	struct aw88261 *aw88261;
	int ret;

	dev_info(&i2c->dev, "Probing AW88261 at address 0x%x, name: %s\n",
		 i2c->addr, dev_name(&i2c->dev));
	ret = i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C);
	if (!ret)
		return dev_err_probe(&i2c->dev, -ENXIO, "check_functionality failed");

	aw88261 = devm_kzalloc(&i2c->dev, sizeof(*aw88261), GFP_KERNEL);
	if (!aw88261)
		return -ENOMEM;

	mutex_init(&aw88261->lock);

	i2c_set_clientdata(i2c, aw88261);

	aw88261->reset_gpio =
		devm_gpiod_get_optional(&i2c->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(aw88261->reset_gpio))
		dev_info(&i2c->dev, "reset gpio not defined\n");
	else
		aw88261_hw_reset(aw88261);

	aw88261->regmap = devm_regmap_init_i2c(i2c, &aw88261_remap_config);
	if (IS_ERR(aw88261->regmap)) {
		ret = PTR_ERR(aw88261->regmap);
		return dev_err_probe(&i2c->dev, ret, "failed to init regmap: %d\n", ret);
	}

	/* aw pa init */
	ret = aw88261_init(&aw88261, i2c, aw88261->regmap);
	if (ret)
		return ret;

	ret = devm_snd_soc_register_component(&i2c->dev,
			&soc_codec_dev_aw88261,
			aw88261_dai, ARRAY_SIZE(aw88261_dai));
	if (ret)
		dev_err(&i2c->dev, "failed to register aw88261: %d", ret);

	return ret;
}

static const struct i2c_device_id aw88261_i2c_id[] = {
	{ AW88261_I2C_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw88261_i2c_id);

static const struct of_device_id aw88261_match_table[] = {
	{ .compatible = "awinic,aw88261" },
	{},
};
MODULE_DEVICE_TABLE(of, aw88261_match_table);

static struct i2c_driver aw88261_i2c_driver = {
	.driver = {
		.name = AW88261_I2C_NAME,
		.of_match_table = aw88261_match_table,
	},
	.probe = aw88261_i2c_probe,
	.id_table = aw88261_i2c_id,
};
module_i2c_driver(aw88261_i2c_driver);

MODULE_DESCRIPTION("ASoC AW88261 Smart PA Driver");
MODULE_LICENSE("GPL v2");
