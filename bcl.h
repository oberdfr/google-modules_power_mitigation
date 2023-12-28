/* SPDX-License-Identifier: GPL-2.0 */

// TODO: merge bcl-whi.h
#if ! IS_ENABLED(CONFIG_SOC_ZUMA)
#include "whi/bcl.h"
#else

#ifndef __BCL_H
#define __BCL_H

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>
#include <soc/google/exynos_pm_qos.h>
#include <dt-bindings/power/s2mpg1x-power.h>
#include <dt-bindings/soc/google/zumapro-bcl.h>
#include "uapi/brownout_stats.h"

#define bcl_cb_get_irq(bcl, v) (((bcl)->ifpmic == MAX77759) ? \
        max77759_get_irq(bcl, v) : max77779_get_irq(bcl, v))
#define bcl_cb_clr_irq(bcl, v) (((bcl)->ifpmic == MAX77759) ? \
        max77759_clr_irq(bcl, v) : max77779_clr_irq(bcl, v))

/* This driver determines if HW was throttled due to SMPL/OCP */

#define DELTA_10MS			(10 * NSEC_PER_MSEC)
#define DELTA_50MS			(50 * NSEC_PER_MSEC)
#define VSHUNT_MULTIPLIER		10000
#define MILLI_TO_MICRO			1000
#define IRQ_ENABLE_DELAY_MS		50
#define NOT_USED 			-1
#define TIMEOUT_10MS			10
#define TIMEOUT_5MS			5
#define TIMEOUT_1MS			1
#define DATA_LOGGING_TIME_MS		48
#define DATA_LOGGING_NUM		50
#define HEAVY_MITIGATION_MODULES_NUM	3
#define MITIGATION_INPUT_DELIM		","
#define MITIGATION_PRINT_BUF_SIZE  	256
#define MITIGATION_TMP_BUF_SIZE	16

enum CPU_CLUSTER {
	LITTLE_CLUSTER,
	MID_CLUSTER,
	BIG_CLUSTER,
	CPU_CLUSTER_MAX,
};

enum SUBSYSTEM_SOURCE {
	SUBSYSTEM_CPU0,
	SUBSYSTEM_CPU1,
	SUBSYSTEM_CPU2,
	SUBSYSTEM_TPU,
	SUBSYSTEM_GPU,
	SUBSYSTEM_AUR,
	SUBSYSTEM_SOURCE_MAX,
};

enum CONCURRENT_PWRWARN_IRQ {
	NONE_BCL_BIN,
	MMWAVE_BCL_BIN,
	RFFE_BCL_BIN,
	MAX_CONCURRENT_PWRWARN_IRQ,
};

enum BCL_BATT_IRQ {
	UVLO1_IRQ_BIN,
	UVLO2_IRQ_BIN,
	BATOILO_IRQ_BIN,
	BATOILO2_IRQ_BIN,
	MAX_BCL_BATT_IRQ,
};

enum MITIGATION_MODE {
	START,
	LIGHT,
	MEDIUM,
	HEAVY,
	MAX_MITIGATION_MODE,
};

enum IRQ_DURATION_BIN {
	LT_5MS,
	BT_5MS_10MS,
	GT_10MS,
};

enum IRQ_TYPE {
	CORE_MAIN_PMIC,
	CORE_SUB_PMIC,
	IF_PMIC,
};

enum IFPMIC {
	MAX77759,
	MAX77779
};

struct irq_duration_stats {
	atomic_t lt_5ms_count;
	atomic_t bt_5ms_10ms_count;
	atomic_t gt_10ms_count;
	ktime_t start_time;
};

extern const unsigned int subsystem_pmu[];
extern const unsigned int clk_stats_offset[];

struct ocpsmpl_stats {
	ktime_t _time;
	int capacity;
	int voltage;
};

enum RATIO_SOURCE {
	CPU0_CON,
	CPU1_HEAVY,
	CPU2_HEAVY,
	TPU_HEAVY,
	GPU_HEAVY,
	CPU1_LIGHT,
	CPU2_LIGHT,
	TPU_LIGHT,
	GPU_LIGHT
};

enum MPMM_SOURCE {
	LITTLE,
	MID,
	BIG,
	MPMMEN
};

struct qos_throttle_limit {
	struct freq_qos_request cpu0_max_qos_req;
	struct freq_qos_request cpu1_max_qos_req;
	struct freq_qos_request cpu2_max_qos_req;
	struct exynos_pm_qos_request gpu_qos_max;
	struct exynos_pm_qos_request tpu_qos_max;
	int cpu0_limit;
	int cpu1_limit;
	int cpu2_limit;
	int gpu_limit;
	int tpu_limit;
	bool throttle;
};

struct zone_triggered_stats {
	atomic_t triggered_cnt[MAX_MITIGATION_MODE];
	ktime_t triggered_time[MAX_MITIGATION_MODE];
};

struct bcl_zone {
	struct device *device;
	struct mutex req_lock;
	struct completion deassert;
	struct workqueue_struct *triggered_wq;
	struct workqueue_struct *warn_wq;
	struct work_struct irq_triggered_work;
	struct delayed_work irq_untriggered_work;
	struct work_struct warn_work;
	struct delayed_work enable_irq_work;
	struct thermal_zone_device *tz;
	struct thermal_zone_device_ops tz_ops;
	struct qos_throttle_limit *bcl_qos;
	struct ocpsmpl_stats bcl_stats;
	struct zone_triggered_stats last_triggered;
	atomic_t bcl_cnt;
	int bcl_prev_lvl;
	int bcl_cur_lvl;
	int bcl_lvl;
	int bcl_pin;
	int bcl_irq;
	int irq_type;
	int polarity;
	void *parent;
	int idx;
	bool disabled;
	bool irq_reg;
	bool conf_qos;
	u32 current_state;
};

struct bcl_core_conf {
	unsigned int con_heavy;
	unsigned int con_light;
	unsigned int clkdivstep;
	unsigned int vdroop_flt;
	unsigned int clk_stats;
	unsigned int clk_out;
	void __iomem *base_mem;
};

struct bcl_batt_irq_conf {
	int batoilo_lower_limit;
	int batoilo_upper_limit;
	u8 batoilo_trig_lvl;
	u8 batoilo_wlc_trig_lvl;
	u8 batoilo_bat_open_to;
	u8 batoilo_rel;
	u8 batoilo_det;
	u8 uvlo_rel;
	u8 uvlo_det;
};

struct bcl_evt_count {
	unsigned int uvlo1;
	unsigned int uvlo2;
	unsigned int batoilo1;
	unsigned int batoilo2;
	u8 enable;
	u8 rate;
};

struct bcl_mitigation_conf {
	u32 module_id;
	u32 threshold;
};

struct bcl_device {
	struct device *device;
	struct device *main_dev;
	struct device *sub_dev;
	struct device *mitigation_dev;
	struct odpm_info *main_odpm;
	struct odpm_info *sub_odpm;
	void __iomem *sysreg_cpucl0;
	struct power_supply *batt_psy;

	struct notifier_block psy_nb;
	struct bcl_zone *zone[TRIGGERED_SOURCE_MAX];
	struct delayed_work soc_work;
	struct thermal_zone_device *soc_tz;
	struct thermal_zone_device_ops soc_tz_ops;

	int trip_high_temp;
	int trip_low_temp;
	int trip_val;
	struct mutex state_trans_lock;
	struct mutex sysreg_lock;

	struct i2c_client *main_pmic_i2c;
	struct i2c_client *sub_pmic_i2c;
	struct i2c_client *main_meter_i2c;
	struct i2c_client *sub_meter_i2c;
	struct device *intf_pmic_dev;
	struct device *irq_pmic_dev;

	struct mutex cpu_ratio_lock;
	struct mutex tpu_ratio_lock;
	struct mutex gpu_ratio_lock;
	struct mutex aur_ratio_lock;
	struct bcl_core_conf core_conf[SUBSYSTEM_SOURCE_MAX];

	bool batt_psy_initialized;
	bool enabled;

	unsigned int main_offsrc1;
	unsigned int main_offsrc2;
	unsigned int sub_offsrc1;
	unsigned int sub_offsrc2;
	unsigned int pwronsrc;
	unsigned int irq_delay;

	unsigned int vdroop1_pin;
	unsigned int vdroop2_pin;
	unsigned int modem_gpio1_pin;
	unsigned int modem_gpio2_pin;
	unsigned int rffe_channel;

	/* debug */
	struct dentry *debug_entry;
	unsigned int gpu_clk_out;
	unsigned int tpu_clk_out;
	unsigned int aur_clk_out;
	u8 add_perph;
	u64 add_addr;
	u64 add_data;
	void __iomem *base_add_mem[SUBSYSTEM_SOURCE_MAX];

	int main_irq_base, sub_irq_base;
	u8 main_setting[METER_CHANNEL_MAX];
	u8 sub_setting[METER_CHANNEL_MAX];
	u64 main_limit[METER_CHANNEL_MAX];
	u64 sub_limit[METER_CHANNEL_MAX];
	int main_pwr_warn_irq[METER_CHANNEL_MAX];
	int sub_pwr_warn_irq[METER_CHANNEL_MAX];
	bool main_pwr_warn_triggered[METER_CHANNEL_MAX];
	bool sub_pwr_warn_triggered[METER_CHANNEL_MAX];
	struct delayed_work main_pwr_irq_work;
	struct delayed_work sub_pwr_irq_work;
	struct irq_duration_stats ifpmic_irq_bins[MAX_BCL_BATT_IRQ][MAX_CONCURRENT_PWRWARN_IRQ];
	struct irq_duration_stats pwrwarn_main_irq_bins[METER_CHANNEL_MAX];
	struct irq_duration_stats pwrwarn_sub_irq_bins[METER_CHANNEL_MAX];
	const char *main_rail_names[METER_CHANNEL_MAX];
	const char *sub_rail_names[METER_CHANNEL_MAX];

	int cpu0_cluster;
	int cpu1_cluster;
	int cpu2_cluster;

	bool cpu0_cluster_on;
	bool cpu1_cluster_on;
	bool cpu2_cluster_on;

	struct bcl_batt_irq_conf batt_irq_conf1;
	struct bcl_batt_irq_conf batt_irq_conf2;
	int pmic_irq;

	enum IFPMIC ifpmic;

	struct gvotable_election *toggle_wlc;

	struct bcl_evt_count evt_cnt;

	bool enabled_br_stats;
	bool data_logging_initialized;
	bool is_data_logging_running;
	unsigned int triggered_idx;
	ssize_t br_stats_size;
	struct brownout_stats *br_stats;
	struct mutex data_logging_lock;
	struct delayed_work data_logging_complete_work;
	struct task_struct *main_task;
	struct task_struct *sub_task;
	struct kthread_worker main_meter_worker;
	struct kthread_worker sub_meter_worker;
	struct kthread_work main_meter_work;
	struct kthread_work sub_meter_work;
	bool main_thread_running;
	bool sub_thread_running;
	/* module id */
	struct bcl_mitigation_conf main_mitigation_conf[METER_CHANNEL_MAX];
	struct bcl_mitigation_conf sub_mitigation_conf[METER_CHANNEL_MAX];
	u32 *non_monitored_module_ids;
	u32 non_monitored_mitigation_module_ids;
	atomic_t mitigation_module_ids;

	bool config_modem;
};

extern void google_bcl_irq_update_lvl(struct bcl_device *bcl_dev, int index, unsigned int lvl);
extern int google_set_db(struct bcl_device *data, unsigned int value, enum MPMM_SOURCE index);
extern unsigned int google_get_db(struct bcl_device *data, enum MPMM_SOURCE index);
extern struct bcl_device *google_retrieve_bcl_handle(void);
extern int google_init_gpu_ratio(struct bcl_device *data);
extern int google_init_tpu_ratio(struct bcl_device *data);
extern int google_init_aur_ratio(struct bcl_device *data);
bool bcl_is_subsystem_on(struct bcl_device *bcl_dev, unsigned int addr);
int cpu_sfr_read(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int *reg);
int cpu_sfr_write(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int value);
bool bcl_disable_power(struct bcl_device *bcl_dev, int cluster);
bool bcl_enable_power(struct bcl_device *bcl_dev, int cluster);
bool bcl_is_cluster_on(struct bcl_device *bcl_dev, int cluster);
int pmic_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value);
int pmic_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value);
int meter_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value);
int meter_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value);
u64 settings_to_current(struct bcl_device *bcl_dev, int pmic, int idx, u32 setting);
void google_bcl_qos_update(struct bcl_zone *zone, bool throttle);
int google_bcl_setup_qos(struct bcl_device *bcl_dev);
void google_bcl_remove_qos(struct bcl_device *bcl_dev);
void google_init_debugfs(struct bcl_device *bcl_dev);
int uvlo_reg_read(struct device *dev, enum IFPMIC ifpmic, int triggered, unsigned int *val);
int batoilo_reg_read(struct device *dev, enum IFPMIC ifpmic, int oilo, unsigned int *val);
int max77759_get_irq(struct bcl_device *bcl_dev, u8 *irq_val);
int max77759_clr_irq(struct bcl_device *bcl_dev, int idx);
int max77779_get_irq(struct bcl_device *bcl_dev, u8 *irq_val);
int max77779_clr_irq(struct bcl_device *bcl_dev, int idx);
int max77779_adjust_batoilo_lvl(struct bcl_device *bcl_dev, u8 wlc_tx_enable);
int google_bcl_setup_votable(struct bcl_device *bcl_dev);
void google_bcl_remove_votable(struct bcl_device *bcl_dev);
int google_bcl_init_data_logging(struct bcl_device *bcl_dev);
void google_bcl_start_data_logging(struct bcl_device *bcl_dev, int idx);
void google_bcl_remove_data_logging(struct bcl_device *bcl_dev);
void google_bcl_upstream_state(struct bcl_zone *zone, enum MITIGATION_MODE state);

#endif /* __BCL_H */

#endif // ! IS_ENABLED(CONFIG_SOC_ZUMA)
