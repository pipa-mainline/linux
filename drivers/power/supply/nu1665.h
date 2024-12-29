/**
 * @file nuvolta_1665.h
 * @author <anxufeng@xiaomi.com>
 * @data April 6 2022
 * @brief
 *  nuvolta 1665 wireless charge!
*/
#ifndef __NU1665_HEADER__
#define __NU1665_HEADER__

#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>

//registers define
#define REG_RX_REV_CMD 0x0020
#define REG_RX_REV_DATA1 0x0021
#define RX_POWER_OFF_ERR 0x0028
#define TRX_MODE_EN 0x0063
#define RX_RTX_MODE 0x002f
#define RX_DATA_INFO 0x1200
#define RX_POWER_MODE 0x1203
#define RX_FASTCHG_RESULT 0x1209
#define TX_MANU_ID_L 0x1224
#define TX_MANU_ID_H 0x1225

//vout definition
#define BPP_DEFAULT_VOUT 6000
#define BPP_QC2_VOUT 6500
#define BPP_PLUS_VOUT 9000
#define EPP_DEFAULT_VOUT 11000
#define EPP_PLUS_VOUT 15000

//protect threshold definition
#define RX_MAX_IOUT 2650
#define RX_MAX_TEMP 84

// interrupts foward definition
#define RX_INT_LDO_ON 0x0001
#define RX_INT_FAST_CHARGE 0x0002
#define RX_INT_AUTHEN_FINISH 0x0004
#define RX_INT_RENEGO_DONE 0x0008
#define RX_INT_ALARM_SUCCESS 0x0010
#define RX_INT_ALARM_FAIL 0x0020
#define RX_INT_OOB_GOOD 0x0040
#define RX_INT_RPP 0x0080
#define RX_INT_TRANSPARENT_SUCCESS 0x0100
#define RX_INT_TRANSPARENT_FAIL 0x0200
#define RX_INT_FACTORY_TEST 0x0400
#define RX_INT_OCP_OTP_ALARM 0x1000
#define RX_INT_POWER_OFF 0x4000
#define RX_INT_POWER_ON 0x8000

#define RTX_INT_EPT (1 << 0)
#define RTX_INT_START_DPING (1 << 1)
#define INT_GET_SS (1 << 2)
#define INT_GET_ID (1 << 3)
#define RTX_INT_GET_CFG (1 << 4)
#define INT_GET_PPP (1 << 5)
#define INT_GET_DPING (1 << 6)
#define INT_INIT_TX (1 << 7)
#define INT_GET_BLE_ADDR (1 << 8)
#define FW_VERSION 0x11

//reverse charge timer
#define REVERSE_CHG_CHECK_DELAY_MS 100000
#define REVERSE_DPING_CHECK_DELAY_MS 10000

//reverse charge fod setting
#define REVERSE_FOD_GAIN 94
#define REVERSE_FOD_OFFSET 0

//driver name definition
#define NUVOLTA_1665_DRIVER_NAME "nuvolta_1665"
#define WLS_CHG_VOTER "WLS_CHG_VOTER"

//firmware check result
#define RX_CHECK_SUCCESS (1 << 0)
#define TX_CHECK_SUCCESS (1 << 1)
#define BOOT_CHECK_SUCCESS (1 << 2)

#ifndef ABS
#define ABS(x) ((x) > 0 ? (x) : (-x))
#endif
#define ABS_CEP_VALUE 1

enum FW_UPDATE_CMD {
	FW_UPDATE_NONE,
	FW_UPDATE_CHECK,
	FW_UPDATE_FORCE,
	FW_UPDATE_FROM_BIN,
	FW_UPDATE_ERASE,
	FW_UPDATE_AUTO,
	FW_UPDATE_MAX,
};

enum reverse_chg_state {
	REVERSE_STATE_OPEN,
	REVERSE_STATE_TIMEOUT,
	REVERSE_STATE_ENDTRANS,
	REVERSE_STATE_FORWARD,
	REVERSE_STATE_TRANSFER,
	REVERSE_STATE_WAITPING,
};

enum fod_param_id {
	FOD_PARAM_20V,
	FOD_PARAM_27V,
	FOD_PARAM_BPP_PLUS,
	FOD_PARAM_MAX,
};

struct params_t {
	s8 gain;
	s8 offset;
};

enum wls_chg_stage {
	NORMAL_MODE = 1,
	TAPER_MODE,
	FULL_MODE,
	RECHG_MODE,
};

enum wls_work_mode {
	RX_MODE,
	RTX_MODE,
};

enum wls_adapter_type {
	ADAPTER_NONE,
	ADAPTER_SDP,
};

struct nuvolta_1665_chg {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	// irq and gpio
	unsigned int tx_on_gpio;
	unsigned int reverse_boost_gpio;
	unsigned int irq_gpio;
	unsigned int power_good_gpio;
	unsigned int power_good_irq;
	unsigned int irq;
	unsigned int enable_gpio;
	unsigned int hall3_irq;
	unsigned int hall4_irq;
	unsigned int hall3_gpio;
	unsigned int hall4_gpio;
	int hall3_online;
	int hall4_online;
	unsigned long pen_val;
	void *pen_v;
	int is_reverse_mode;
	int is_boost_mode;
	int reverse_vout;
	int reverse_iout;
	int reverse_temp;
	int reverse_pen_soc;
	u8 pen_mac_data[6];
	int power_off_mode;
	int chip_ok;
	struct pinctrl *idt_pinctrl;
	struct pinctrl_state *idt_gpio_active;
	struct pinctrl_state *idt_gpio_suspend;
	// delay works
	struct delayed_work wireless_int_work;
	struct delayed_work wireless_pg_det_work;
	struct delayed_work chg_monitor_work;
	struct delayed_work reverse_chg_state_work;
	struct delayed_work reverse_dping_state_work;
	struct delayed_work init_detect_work;
	struct delayed_work factory_reverse_start_work;
	struct delayed_work factory_reverse_stop_work;
	struct delayed_work delay_report_status_work;
	struct delayed_work rx_alarm_work;
	struct delayed_work rx_enable_usb_work;
	struct delayed_work max_power_control_work;
	struct delayed_work fw_state_work;
	struct delayed_work hall3_irq_work;
	struct delayed_work hall4_irq_work;
	struct delayed_work pen_notifier_work;
	struct delayed_work reverse_sent_state_work;
	struct delayed_work reverse_chg_work;
	struct delayed_work probe_fw_download_work;
	struct delayed_work pen_check_work;
	// lock
	struct mutex wireless_chg_int_lock;
	struct mutex reverse_op_lock;
	// alarm
	struct alarm reverse_dping_alarm;
	struct alarm reverse_chg_alarm;
	//vote
	struct votable *fcc_votable;
	struct votable *icl_votable;
	// wireless charge device
	struct wireless_charger_device *wlschgdev;
	struct charger_device *master_cp_dev;
	const char *wlsdev_name;
	// charger device
	struct charger_device *cp_master_dev;
	// power supply
	struct power_supply *batt_psy;
	struct regulator *pmic_boost;
	struct power_supply *nuvo_psy;
	// driver parameters
	u8 epp;
	u8 epp_tx_id_h;
	u8 epp_tx_id_l;
	u8 tx_manu_id_l;
	u8 tx_manu_id_h;
	u8 fc_flag;
	u8 uuid[4];
	u8 wait_for_reverse_test_status;
	u8 power_good_flag;
	u8 set_fastcharge_vout_cnt;
	u8 ss;
	u16 adapter_type;
	int batt_soc;
	int target_vol;
	int target_curr;
	int pre_curr;
	int pre_vol;
	int vout_setted;
	int chg_status;
	int chg_phase;
	int is_reverse_chg;
	bool fw_update;
	int fw_version;
	bool parallel_charge;
	bool wait_for_reverse_test;
	bool qc_enable;
	bool reverse_chg_en;
	bool alarm_flag;
};

struct wls_fw_parameters {
	u8 fw_rx_id;
	u8 fw_tx_id;
	u8 fw_boot_id;
	u8 hw_id_h;
	u8 hw_id_l;
};

#endif
