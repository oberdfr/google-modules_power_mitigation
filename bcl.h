/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __BCL_H
#define __BCL_H

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>
#if IS_ENABLED(CONFIG_SOC_GS101)
#include <dt-bindings/soc/google/gs101-bcl.h>
#elif IS_ENABLED(CONFIG_SOC_GS201)
#include <dt-bindings/soc/google/gs201-bcl.h>
#endif

#define bcl_cb_get_irq(bcl, v) max77759_get_irq(bcl, v)
#define bcl_cb_clr_irq(bcl) max77759_clr_irq(bcl)

#if IS_ENABLED(CONFIG_SOC_GS101)
#define MAIN_OFFSRC1 S2MPG10_PM_OFFSRC
#define MAIN_OFFSRC2 S2MPG10_PM_OFFSRC
#define SUB_OFFSRC1 S2MPG11_PM_OFFSRC
#define SUB_OFFSRC2 S2MPG11_PM_OFFSRC
#define MAIN_PWRONSRC S2MPG10_PM_PWRONSRC
#elif IS_ENABLED(CONFIG_SOC_GS201)
#define MAIN_OFFSRC1 S2MPG12_PM_OFFSRC1
#define MAIN_OFFSRC2 S2MPG12_PM_OFFSRC2
#define SUB_OFFSRC1 S2MPG13_PM_OFFSRC
#define SUB_OFFSRC2 S2MPG13_PM_OFFSRC
#define MAIN_PWRONSRC S2MPG12_PM_PWRONSRC
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#define MAIN_OFFSRC1 S2MPG14_PM_OFFSRC1
#define MAIN_OFFSRC2 S2MPG14_PM_OFFSRC2
#define SUB_OFFSRC1 S2MPG15_PM_OFFSRC1
#define SUB_OFFSRC2 S2MPG15_PM_OFFSRC2
#define MAIN_PWRONSRC S2MPG14_PM_PWRONSRC
#endif
/* This driver determines if HW was throttled due to SMPL/OCP */

#define DELTA_10MS		(10 * NSEC_PER_MSEC)
#define DELTA_50MS		(50 * NSEC_PER_MSEC)
#define VSHUNT_MULTIPLIER	10000
#define MILLI_TO_MICRO		1000
#define IRQ_ENABLE_DELAY_MS	50

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
	MAX_BCL_BATT_IRQ,
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

enum IFPMIC {
	MAX77759,
	MAX77779
};

enum MITIGATION_SEVERITY {
	NOTHING,
	LIMIT_CAP,
	POWER_REDUCTION,
	SHUTDOWN
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

struct bcl_zone {
	struct device *device;
	struct delayed_work irq_triggered_work;
	struct delayed_work irq_untriggered_work;
	struct delayed_work irq_work;
	struct delayed_work enable_irq_work;
	struct thermal_zone_device *tz;
	struct thermal_zone_device_ops tz_ops;
	struct ocpsmpl_stats bcl_stats;
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
	int triggered_count;
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
	struct i2c_client *intf_pmic_i2c;
	struct i2c_client *irq_pmic_i2c;

	struct mutex ratio_lock;
	struct bcl_core_conf core_conf[SUBSYSTEM_SOURCE_MAX];

	bool batt_psy_initialized;
	bool enabled;
	bool ready;

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

	int batoilo_lower_limit;
	int batoilo_upper_limit;
	int pmic_irq;

	enum IFPMIC ifpmic;
};

extern void google_bcl_irq_update_lvl(struct bcl_device *bcl_dev, int index, unsigned int lvl);
extern struct bcl_device *google_retrieve_bcl_handle(void);
extern int google_init_gpu_ratio(struct bcl_device *data);
extern int google_init_tpu_ratio(struct bcl_device *data);
extern int google_init_aur_ratio(struct bcl_device *data);
bool bcl_is_subsystem_on(unsigned int addr);
bool bcl_disable_power(int cluster);
bool bcl_enable_power(int cluster);
bool bcl_is_cluster_on(int cluster);
int pmic_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value);
int pmic_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value);
int meter_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value);
int meter_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value);
u64 settings_to_current(struct bcl_device *bcl_dev, int pmic, int idx, u32 setting);
void google_init_debugfs(struct bcl_device *bcl_dev);
int uvlo_reg_read(struct i2c_client *client, enum IFPMIC ifpmic, int triggered, unsigned int *val);
int batoilo_reg_read(struct i2c_client *client, enum IFPMIC ifpmic, int oilo, unsigned int *val);
int max77759_get_irq(struct bcl_device *bcl_dev, u8 *irq_val);
int max77759_clr_irq(struct bcl_device *bcl_dev);

#endif /* __BCL_H */
