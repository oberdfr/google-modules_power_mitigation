// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_core.c Google bcl core driver
 *
 * Copyright (c) 2022, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/completion.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <dt-bindings/interrupt-controller/zuma.h>
#include <linux/regulator/pmic_class.h>
#include <soc/google/odpm.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include "bcl.h"
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include <max77759_regs.h>
#include <max77779.h>
#include <max777x9_bcl.h>

static const struct platform_device_id google_id_table[] = {
	{.name = "google_mitigation",},
	{},
};

static const unsigned int xclkout_source[] = {
	XCLKOUT_SOURCE_CPU0,
	XCLKOUT_SOURCE_CPU1,
	XCLKOUT_SOURCE_CPU2,
	XCLKOUT_SOURCE_TPU,
	XCLKOUT_SOURCE_GPU
};

void update_irq_start_times(struct bcl_device *bcl_dev, int id);
void update_irq_end_times(struct bcl_device *bcl_dev, int id);
void pwrwarn_update_start_time(struct bcl_device *bcl_dev,
				int id, struct irq_duration_stats *bins,
				bool *pwr_warn_triggered,
				enum CONCURRENT_PWRWARN_IRQ bin_ind);
void pwrwarn_update_end_time(struct bcl_device *bcl_dev, int id,
				struct irq_duration_stats *bins,
				enum CONCURRENT_PWRWARN_IRQ bin_ind);

static int zone_read_temp(struct thermal_zone_device *tz, int *val)
{
	struct bcl_zone *zone = tz->devdata;

	*val = zone->bcl_cur_lvl;
	zone->bcl_prev_lvl = *val;
	return 0;
}

static struct power_supply *google_get_power_supply(struct bcl_device *bcl_dev)
{
	static struct power_supply *psy[2];
	static struct power_supply *batt_psy;
	int err = 0;

	batt_psy = NULL;
	err = power_supply_get_by_phandle_array(bcl_dev->device->of_node, "google,power-supply",
						psy, ARRAY_SIZE(psy));
	if (err > 0)
		batt_psy = psy[0];
	return batt_psy;
}

static void ocpsmpl_read_stats(struct bcl_device *bcl_dev,
			       struct ocpsmpl_stats *dst, struct power_supply *psy)
{
	union power_supply_propval ret = {0};
	int err = 0;

	if (!psy)
		return;
	dst->_time = ktime_to_ms(ktime_get());
	err = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &ret);
	if (err < 0)
		dst->capacity = -1;
	else {
		dst->capacity = ret.intval;
		bcl_dev->batt_psy_initialized = true;
	}
	err = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	if (err < 0)
		dst->voltage = -1;
	else {
		dst->voltage = ret.intval;
		bcl_dev->batt_psy_initialized = true;
	}

}

static void update_tz(struct bcl_zone *zone, int idx, bool triggered)
{
	if (triggered)
		zone->bcl_cur_lvl = zone->bcl_lvl + THERMAL_HYST_LEVEL;
	else
		zone->bcl_cur_lvl = 0;
	if (zone->tz && (zone->bcl_prev_lvl != zone->bcl_cur_lvl))
		thermal_zone_device_update(zone->tz, THERMAL_EVENT_UNSPECIFIED);
}

static int evt_cnt_rd_and_clr(struct bcl_device *bcl_dev, int idx, unsigned int *val)
{
	int ret;
	unsigned int reg;

	switch (idx) {
	case UVLO1:
		reg = MAX77779_PMIC_EVENT_CNT_UVLO0;
		break;
	case UVLO2:
		reg = MAX77779_PMIC_EVENT_CNT_UVLO1;
		break;
	case BATOILO1:
		reg = MAX77779_PMIC_EVENT_CNT_OILO0;
		break;
	case BATOILO2:
		reg = MAX77779_PMIC_EVENT_CNT_OILO1;
		break;
	}

	/* Read to clear register */
	ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_i2c, reg, val);
	if (ret < 0) {
		dev_err(bcl_dev->device, "evt_cnt_rd_and_clr: %d, fail\n", reg);
		return -ENODEV;
	}
	return 0;
}

static irqreturn_t deassert_irq_handler(int irq, void *data)
{
	struct bcl_zone *zone = data;
	struct bcl_device *bcl_dev;
	u8 idx;
	u8 irq_val = 0;

	if (!zone || !zone->parent)
		return IRQ_HANDLED;

	idx = zone->idx;
	bcl_dev = zone->parent;
	if (!bcl_dev->enabled)
		return IRQ_HANDLED;

	if (bcl_dev->ifpmic == MAX77759 && idx >= UVLO2 && idx <= BATOILO2) {
		bcl_cb_get_irq(bcl_dev, &irq_val);
		if (irq_val == 0)
			goto exit;
		idx = irq_val;
		zone = bcl_dev->zone[idx];
	}
	mutex_lock(&zone->req_lock);
	complete(&zone->deassert);
	mutex_unlock(&zone->req_lock);
	mod_delayed_work(system_highpri_wq, &zone->irq_untriggered_work, 0);
exit:
	return IRQ_HANDLED;
}

static int google_bcl_wait_for_response_locked(struct bcl_zone *zone, int timeout_ms)
{
	mutex_lock(&zone->req_lock);
	if (wait_for_completion_timeout(&zone->deassert, msecs_to_jiffies(timeout_ms)) == 0) {
		mutex_unlock(&zone->req_lock);
		return -EINVAL;
	}
	mutex_unlock(&zone->req_lock);
	return 0;
}

static irqreturn_t latched_irq_handler(int irq, void *data)
{
	struct bcl_zone *zone = data;
	struct bcl_device *bcl_dev;
	u8 idx;
	u8 irq_val = 0;

	if (!zone || !zone->parent)
		return IRQ_HANDLED;

	idx = zone->idx;
	bcl_dev = zone->parent;
	if (!bcl_dev->enabled)
		return IRQ_HANDLED;

	if (idx >= UVLO1 && idx <= BATOILO2) {
		atomic_inc(&zone->last_triggered.triggered_cnt[START]);
		zone->last_triggered.triggered_time[START] = ktime_to_ms(ktime_get());
	}

	if (bcl_dev->ifpmic == MAX77759 && idx >= UVLO2 && idx <= BATOILO2) {
		bcl_cb_get_irq(bcl_dev, &irq_val);
		if (irq_val == 0)
			goto exit;
		idx = irq_val;
		zone = bcl_dev->zone[idx];
	}
	queue_work(zone->triggered_wq, &zone->irq_triggered_work);
exit:
	return IRQ_HANDLED;
}

static bool google_warn_check(struct bcl_zone *zone)
{
	struct bcl_device *bcl_dev;
	int gpio_level, assert = 0, ret;
	int idx = zone->idx;
	unsigned int regval;

	bcl_dev = zone->parent;
	if (zone->bcl_pin != NOT_USED) {
		gpio_level = gpio_get_value(zone->bcl_pin);
		if (zone->irq_type == IF_PMIC)
			bcl_cb_clr_irq(bcl_dev, zone->idx);
		return (gpio_level == zone->polarity);
	}
	if (bcl_dev->ifpmic == MAX77779) {
		ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_i2c,
						      MAX77779_PMIC_VDROOP_INT, &regval);
		if (ret < 0) {
			dev_err(bcl_dev->device, "IRQ read: %d, fail\n", regval);
			return false;
		}
		if (idx == BATOILO2)
			assert = _max77779_pmic_vdroop_int_bat_oilo2_int_get(regval);
		else if (idx == UVLO2)
			assert = _max77779_pmic_vdroop_int_sys_uvlo2_int_get(regval);
		if (assert) {
			ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_i2c,
							       MAX77779_PMIC_VDROOP_INT,
							       regval);
			if (ret < 0) {
				dev_err(bcl_dev->device, "IRQ clear: %d, fail\n", regval);
				return false;
			}
			return true;
		}
	}
	return false;
}

static void google_bcl_release_throttling(struct bcl_zone *zone)
{
	struct bcl_device *bcl_dev;
	unsigned int reg;

	bcl_dev = zone->parent;
	zone->bcl_cur_lvl = 0;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	google_bcl_upstream_state(zone, START);
	if (zone->bcl_qos)
		google_bcl_qos_update(zone, false);
#endif
	mutex_lock(&zone->req_lock);
	complete(&zone->deassert);
	mutex_unlock(&zone->req_lock);
	if (zone->irq_type == IF_PMIC) {
		update_irq_end_times(bcl_dev, zone->idx);
		if (zone->idx >= UVLO1 && zone->idx <= BATOILO2 && bcl_dev->ifpmic == MAX77779)
			evt_cnt_rd_and_clr(bcl_dev, zone->idx, &reg);
	}
	if (zone->idx == BATOILO && bcl_dev->config_modem)
		gpio_set_value(bcl_dev->modem_gpio2_pin, 0);
	update_tz(zone, zone->idx, false);
}

static void google_warn_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, warn_work);
	struct bcl_device *bcl_dev;

	bcl_dev = zone->parent;
	if (!bcl_dev)
		return;
	if (!google_warn_check(zone)) {
		google_bcl_release_throttling(zone);
	} else {
		zone->bcl_cur_lvl = zone->bcl_lvl + THERMAL_HYST_LEVEL;
		/* ODPM Read to kick off LIGHT module throttling */
		queue_work(zone->warn_wq, &zone->warn_work);
	}
	if (zone->tz)
		thermal_zone_device_update(zone->tz, THERMAL_EVENT_UNSPECIFIED);
}

static int google_bcl_set_soc(struct bcl_device *bcl_dev, int low, int high)
{
	if (high == bcl_dev->trip_high_temp)
		return 0;

	mutex_lock(&bcl_dev->state_trans_lock);
	bcl_dev->trip_low_temp = low;
	bcl_dev->trip_high_temp = high;
	schedule_delayed_work(&bcl_dev->soc_work, 0);

	mutex_unlock(&bcl_dev->state_trans_lock);
	return 0;
}

static int tz_bcl_set_soc(struct thermal_zone_device *tz, int low, int high)
{
	return google_bcl_set_soc(tz->devdata, low, high);
}

static int google_bcl_read_soc(struct bcl_device *bcl_dev, int *val)
{
	union power_supply_propval ret = {
		0,
	};
	int err = 0;

	*val = 100;
	if (!bcl_dev->batt_psy)
		bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	if (bcl_dev->batt_psy) {
		err = power_supply_get_property(bcl_dev->batt_psy,
						POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (err < 0) {
			dev_err(bcl_dev->device, "battery percentage read error:%d\n", err);
			return err;
		}
		bcl_dev->batt_psy_initialized = true;
		*val = 100 - ret.intval;
	}
	pr_debug("soc:%d\n", *val);

	return err;
}

static int tz_bcl_read_soc(struct thermal_zone_device *tz, int *val)
{
	return google_bcl_read_soc(tz->devdata, val);
}

static void google_bcl_evaluate_soc(struct work_struct *work)
{
	int battery_percentage_reverse;
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  soc_work.work);

	if (google_bcl_read_soc(bcl_dev, &battery_percentage_reverse))
		return;

	mutex_lock(&bcl_dev->state_trans_lock);
	if ((battery_percentage_reverse < bcl_dev->trip_high_temp) &&
		(battery_percentage_reverse > bcl_dev->trip_low_temp))
		goto eval_exit;

	bcl_dev->trip_val = battery_percentage_reverse;
	mutex_unlock(&bcl_dev->state_trans_lock);
	if (!bcl_dev->soc_tz) {
		bcl_dev->soc_tz = devm_thermal_of_zone_register(bcl_dev->device,
								PMIC_SOC, bcl_dev,
								&bcl_dev->soc_tz_ops);
		if (IS_ERR(bcl_dev->soc_tz)) {
			dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
				PTR_ERR(bcl_dev->soc_tz));
			return;
		}
	}
	if (!IS_ERR(bcl_dev->soc_tz))
		thermal_zone_device_update(bcl_dev->soc_tz, THERMAL_EVENT_UNSPECIFIED);
	return;
eval_exit:
	mutex_unlock(&bcl_dev->state_trans_lock);
}

static int battery_supply_callback(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct bcl_device *bcl_dev = container_of(nb, struct bcl_device, psy_nb);
	struct power_supply *bcl_psy;

	if (!bcl_dev)
		return NOTIFY_OK;

	bcl_psy = bcl_dev->batt_psy;

	if (!bcl_psy || event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!strcmp(psy->desc->name, bcl_psy->desc->name))
		schedule_delayed_work(&bcl_dev->soc_work, 0);

	return NOTIFY_OK;
}

static int google_bcl_remove_thermal(struct bcl_device *bcl_dev)
{
	int i = 0;
	struct bcl_zone *zone;

	power_supply_unreg_notifier(&bcl_dev->psy_nb);
	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (!bcl_dev->zone[i])
			continue;
		zone = bcl_dev->zone[i];
		if (zone->tz)
			devm_thermal_of_zone_unregister(bcl_dev->device, zone->tz);
		mutex_destroy(&zone->req_lock);
		destroy_workqueue(zone->triggered_wq);
		destroy_workqueue(zone->warn_wq);
		cancel_delayed_work(&zone->irq_untriggered_work);
		cancel_delayed_work(&zone->enable_irq_work);
	}
	mutex_destroy(&bcl_dev->data_logging_lock);
	mutex_destroy(&bcl_dev->state_trans_lock);
	mutex_destroy(&bcl_dev->cpu_ratio_lock);
	mutex_destroy(&bcl_dev->gpu_ratio_lock);
	mutex_destroy(&bcl_dev->tpu_ratio_lock);
	mutex_destroy(&bcl_dev->aur_ratio_lock);
	mutex_destroy(&bcl_dev->sysreg_lock);

	return 0;
}

static int google_bcl_init_clk_div(struct bcl_device *bcl_dev, int idx,
				   unsigned int value)
{
	void __iomem *addr;
	int ret;

	if (!bcl_dev)
		return -EIO;
	switch (idx) {
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_GPU:
	case SUBSYSTEM_AUR:
		return -EIO;
	case SUBSYSTEM_CPU0:
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		addr = bcl_dev->core_conf[idx].base_mem + CLKDIVSTEP;
		break;
	}
	ret = cpu_sfr_write(bcl_dev, idx, addr, value);

	return ret;
}

struct bcl_device *google_retrieve_bcl_handle(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct bcl_device *bcl_dev;

	np = of_find_node_by_name(NULL, "google,mitigation");
	if (!np || !virt_addr_valid(np) || !of_device_is_available(np))
		return NULL;
	pdev = of_find_device_by_node(np);
	if (!pdev)
		return NULL;
	bcl_dev = platform_get_drvdata(pdev);
	if (!bcl_dev)
		return NULL;

	return bcl_dev;
}
EXPORT_SYMBOL_GPL(google_retrieve_bcl_handle);

static int google_init_ratio(struct bcl_device *data, enum SUBSYSTEM_SOURCE idx, struct mutex lock)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;

	if (!bcl_is_subsystem_on(data, subsystem_pmu[idx]))
		return -EIO;

	if (idx < SUBSYSTEM_TPU)
		return -EIO;

	mutex_lock(&lock);
	if (idx != SUBSYSTEM_AUR) {
		addr = data->core_conf[idx].base_mem + CLKDIVSTEP_CON_HEAVY;
		__raw_writel(data->core_conf[idx].con_heavy, addr);
		addr = data->core_conf[idx].base_mem + CLKDIVSTEP_CON_LIGHT;
		__raw_writel(data->core_conf[idx].con_light, addr);
		addr = data->core_conf[idx].base_mem + VDROOP_FLT;
		__raw_writel(data->core_conf[idx].vdroop_flt, addr);
	}
	addr = data->core_conf[idx].base_mem + CLKDIVSTEP;
	__raw_writel(data->core_conf[idx].clkdivstep, addr);
	addr = data->core_conf[idx].base_mem + CLKOUT;
	__raw_writel(data->core_conf[idx].clk_out, addr);
	data->core_conf[idx].clk_stats = __raw_readl(data->core_conf[idx].base_mem +
						     clk_stats_offset[idx]);
	mutex_unlock(&lock);

	return 0;
}

int google_init_tpu_ratio(struct bcl_device *data)
{
	return google_init_ratio(data, SUBSYSTEM_TPU, data->tpu_ratio_lock);
}
EXPORT_SYMBOL_GPL(google_init_tpu_ratio);

int google_init_gpu_ratio(struct bcl_device *data)
{
	return google_init_ratio(data, SUBSYSTEM_GPU, data->gpu_ratio_lock);
}
EXPORT_SYMBOL_GPL(google_init_gpu_ratio);

int google_init_aur_ratio(struct bcl_device *data)
{
	return google_init_ratio(data, SUBSYSTEM_AUR, data->aur_ratio_lock);
}
EXPORT_SYMBOL_GPL(google_init_aur_ratio);

unsigned int google_get_db(struct bcl_device *data, enum MPMM_SOURCE index)
{
	void __iomem *addr;
	unsigned int reg;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		dev_err(data->device, "Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	if (index == MID)
		addr = data->sysreg_cpucl0 + CLUSTER0_MID_DISPBLOCK;
	else if (index == BIG)
		addr = data->sysreg_cpucl0 + CLUSTER0_BIG_DISPBLOCK;
	else
		return -EINVAL;

	mutex_lock(&data->sysreg_lock);
	reg = __raw_readl(addr);
	mutex_unlock(&data->sysreg_lock);

	return reg;
}
EXPORT_SYMBOL_GPL(google_get_db);

int google_set_db(struct bcl_device *data, unsigned int value, enum MPMM_SOURCE index)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		dev_err(data->device, "Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	if (index == MID)
		addr = data->sysreg_cpucl0 + CLUSTER0_MID_DISPBLOCK;
	else if (index == BIG)
		addr = data->sysreg_cpucl0 + CLUSTER0_BIG_DISPBLOCK;
	else
		return -EINVAL;

	mutex_lock(&data->sysreg_lock);
	__raw_writel(value, addr);
	mutex_unlock(&data->sysreg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_set_db);

static void google_enable_irq_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, enable_irq_work.work);

	if (!zone)
		return;

	zone->disabled = false;
	enable_irq(zone->bcl_irq);
}

static void google_irq_triggered_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, irq_triggered_work);
	struct bcl_device *bcl_dev;
	int idx, ret;

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	if (zone->bcl_qos)
		google_bcl_qos_update(zone, true);
#endif

	idx = zone->idx;
	bcl_dev = zone->parent;
	/* LIGHT phase */
	google_bcl_upstream_state(zone, LIGHT);

	if (bcl_dev->batt_psy_initialized) {
		atomic_inc(&zone->bcl_cnt);
		ocpsmpl_read_stats(bcl_dev, &zone->bcl_stats, bcl_dev->batt_psy);
		update_tz(zone, idx, true);
	}
	queue_work(zone->warn_wq, &zone->warn_work);

	if (zone->irq_type == IF_PMIC)
		update_irq_start_times(bcl_dev, idx);

	if (idx == BATOILO && bcl_dev->config_modem)
		gpio_set_value(bcl_dev->modem_gpio2_pin, 1);

	if (zone->irq_type != IF_PMIC && bcl_dev->irq_delay != 0) {
		if (!zone->disabled) {
			zone->disabled = true;
			disable_irq(zone->bcl_irq);
			mod_delayed_work(system_highpri_wq, &zone->enable_irq_work,
					 msecs_to_jiffies(bcl_dev->irq_delay));
		}
	}
	ret = google_bcl_wait_for_response_locked(zone, TIMEOUT_10MS);
	google_bcl_upstream_state(zone, MEDIUM);

	/* MEDIUM phase: b/300504518 */
	ret = google_bcl_wait_for_response_locked(zone, TIMEOUT_10MS);
	google_bcl_upstream_state(zone, HEAVY);
	/* We most likely have to shutdown after this */
	/* HEAVY phase */
	/* IRQ deasserted */
	if (ret == 0)
		return;
}

static void google_irq_untriggered_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, irq_untriggered_work.work);
	struct bcl_device *bcl_dev;

	if (!zone || !zone->parent)
		return;

	bcl_dev = zone->parent;

	google_bcl_release_throttling(zone);
	/* IRQ falling edge */
	if (zone->irq_type == IF_PMIC)
		bcl_cb_clr_irq(bcl_dev, zone->idx);
}

static irqreturn_t vdroop_irq_thread_fn(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	struct bcl_zone *zone = NULL;
	int ret;
	unsigned int regval;

	if (!bcl_dev->enabled)
		return IRQ_HANDLED;
	/* This can one of BATOILO2 or SYS_UVLO2 or EVENT_CNT IRQ */
	ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_i2c,
					      MAX77779_PMIC_VDROOP_INT, &regval);
	if (ret < 0) {
		dev_err(bcl_dev->device, "irq_thread: read: %d, fail\n", irq);
		return IRQ_HANDLED;
	}
	if (_max77779_pmic_vdroop_int_bat_oilo2_int_get(regval))
		zone = bcl_dev->zone[BATOILO2];
	else if	(_max77779_pmic_vdroop_int_sys_uvlo2_int_get(regval))
		zone = bcl_dev->zone[UVLO2];
	if (zone) {
		atomic_inc(&zone->last_triggered.triggered_cnt[START]);
		zone->last_triggered.triggered_time[START] = ktime_to_ms(ktime_get());
		queue_work(zone->triggered_wq, &zone->irq_triggered_work);
	}

	ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_i2c,
					       MAX77779_PMIC_VDROOP_INT, regval);
	if (ret < 0) {
		dev_err(bcl_dev->device, "irq_thread: clear: %d, fail\n", regval);
		return IRQ_HANDLED;
	}
	return IRQ_HANDLED;
}

static int google_bcl_register_zone(struct bcl_device *bcl_dev, int idx, const char *devname,
				    int pin, int lvl, int irq, int type)
{
	int ret = 0;
	struct bcl_zone *zone;
	bool to_conf = true;
	u32 default_intr_flag, latched_intr_flag, deassert_intr_flag;

	if (!bcl_dev)
		return -ENOMEM;

	zone = devm_kzalloc(bcl_dev->device, sizeof(struct bcl_zone), GFP_KERNEL);

	if (!zone)
		return -ENOMEM;

	default_intr_flag = IRQF_ONESHOT | IRQF_SHARED;
	init_completion(&zone->deassert);
	mutex_init(&zone->req_lock);
	zone->idx = idx;
	zone->bcl_pin = pin;
	zone->bcl_irq = irq;
	zone->bcl_cur_lvl = 0;
	zone->bcl_prev_lvl = 0;
	zone->bcl_lvl = lvl;
	zone->parent = bcl_dev;
	zone->irq_type = type;
	atomic_set(&zone->bcl_cnt, 0);
	atomic_set(&zone->last_triggered.triggered_cnt[START], 0);
	atomic_set(&zone->last_triggered.triggered_cnt[LIGHT], 0);
	atomic_set(&zone->last_triggered.triggered_cnt[MEDIUM], 0);
	atomic_set(&zone->last_triggered.triggered_cnt[HEAVY], 0);
	if (idx == SMPL_WARN) {
		latched_intr_flag = IRQF_TRIGGER_FALLING;
		deassert_intr_flag = IRQF_TRIGGER_RISING;
		irq_set_status_flags(zone->bcl_irq, IRQ_DISABLE_UNLAZY);
		zone->polarity = 0;
	} else {
		latched_intr_flag = IRQF_TRIGGER_RISING;
		deassert_intr_flag = IRQF_TRIGGER_FALLING;
		zone->polarity = 1;
	}
	latched_intr_flag = latched_intr_flag | default_intr_flag;
	deassert_intr_flag = deassert_intr_flag | default_intr_flag;
	if ((bcl_dev->ifpmic == MAX77779) && (idx == BATOILO2 || idx == UVLO2)) {
		zone->bcl_pin = NOT_USED;
		if (!zone->irq_reg) {
			ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->pmic_irq, NULL,
							vdroop_irq_thread_fn,
							IRQF_TRIGGER_FALLING | IRQF_SHARED |
							IRQF_ONESHOT | IRQF_NO_THREAD, devname,
							bcl_dev);
			if (ret < 0) {
				dev_err(zone->device, "Failed to request l-IRQ: %d: %d\n",
					irq, ret);
				devm_kfree(bcl_dev->device, zone);
				return ret;
			}
			zone->irq_reg = true;
		}
		to_conf = false;
	}
	if ((bcl_dev->ifpmic == MAX77759) && (idx == BATOILO))
		to_conf = false;
	if (to_conf) {
		ret = devm_request_threaded_irq(bcl_dev->device, zone->bcl_irq, NULL,
						latched_irq_handler,
						latched_intr_flag, devname, zone);

		if (ret < 0) {
			dev_err(zone->device, "Failed to request l-IRQ: %d: %d\n", irq, ret);
			devm_kfree(bcl_dev->device, zone);
			return ret;
		}
		ret = devm_request_threaded_irq(bcl_dev->device, zone->bcl_irq, NULL,
						deassert_irq_handler,
						deassert_intr_flag, devname, zone);

		if (ret < 0) {
			dev_err(zone->device, "Failed to request d-IRQ: %d: %d\n", irq, ret);
			devm_kfree(bcl_dev->device, zone);
			return ret;
		}
		zone->irq_reg = true;
		zone->disabled = true;
		disable_irq(zone->bcl_irq);
	}
	zone->triggered_wq = alloc_workqueue(devname, WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!zone->triggered_wq) {
		dev_err(zone->device, "%s: ERR! fail to create triggered_wq\n", devname);
		return -ENOMEM;
	}
	zone->warn_wq = alloc_workqueue(devname, WQ_HIGHPRI, 1);
	if (!zone->warn_wq) {
		dev_err(zone->device, "%s: ERR! fail to create warn_wq\n", devname);
		destroy_workqueue(zone->triggered_wq);
		return -ENOMEM;
	}
	INIT_WORK(&zone->irq_triggered_work, google_irq_triggered_work);
	INIT_WORK(&zone->warn_work, google_warn_work);
	INIT_DELAYED_WORK(&zone->irq_untriggered_work, google_irq_untriggered_work);
	INIT_DELAYED_WORK(&zone->enable_irq_work, google_enable_irq_work);
	zone->tz_ops.get_temp = zone_read_temp;
	zone->tz = devm_thermal_of_zone_register(bcl_dev->device, idx, zone, &zone->tz_ops);
	if (IS_ERR(zone->tz))
		dev_err(zone->device, "TZ register failed. %d, err:%ld\n", idx, PTR_ERR(zone->tz));
	else {
		thermal_zone_device_enable(zone->tz);
		thermal_zone_device_update(zone->tz, THERMAL_DEVICE_UP);
	}
	bcl_dev->zone[idx] = zone;
	return ret;
}

static void main_pwrwarn_irq_work(struct work_struct *work)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  main_pwr_irq_work.work);
	bool revisit_needed = false;
	int i;
	u32 micro_unit[ODPM_CHANNEL_MAX];
	u32 measurement;

	mutex_lock(&bcl_dev->main_odpm->lock);

	odpm_get_raw_lpf_values(bcl_dev->main_odpm, S2MPG1415_METER_CURRENT, micro_unit);
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		measurement = micro_unit[i] >> LPF_CURRENT_SHIFT;
		bcl_dev->main_pwr_warn_triggered[i] = (measurement > bcl_dev->main_setting[i]);
		if (!revisit_needed)
			revisit_needed = bcl_dev->main_pwr_warn_triggered[i];
		if ((!revisit_needed) && (i == bcl_dev->rffe_channel) && bcl_dev->config_modem)
			gpio_set_value(bcl_dev->modem_gpio1_pin, 0);
		if (!bcl_dev->main_pwr_warn_triggered[i])
			pwrwarn_update_end_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
						RFFE_BCL_BIN);
		else
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
							bcl_dev->main_pwr_warn_triggered,
							RFFE_BCL_BIN);
	}

	mutex_unlock(&bcl_dev->main_odpm->lock);

	if (revisit_needed)
		mod_delayed_work(system_unbound_wq, &bcl_dev->main_pwr_irq_work,
				 msecs_to_jiffies(PWRWARN_DELAY_MS));
#endif
}

static void sub_pwrwarn_irq_work(struct work_struct *work)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  sub_pwr_irq_work.work);
	bool revisit_needed = false;
	int i;
	u32 micro_unit[ODPM_CHANNEL_MAX];
	u32 measurement;

	mutex_lock(&bcl_dev->sub_odpm->lock);

	odpm_get_raw_lpf_values(bcl_dev->sub_odpm, S2MPG1415_METER_CURRENT, micro_unit);
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		measurement = micro_unit[i] >> LPF_CURRENT_SHIFT;
		bcl_dev->sub_pwr_warn_triggered[i] = (measurement > bcl_dev->sub_setting[i]);
		if (!revisit_needed)
			revisit_needed = bcl_dev->sub_pwr_warn_triggered[i];
		if ((!revisit_needed) && (i == bcl_dev->rffe_channel) && bcl_dev->config_modem)
			gpio_set_value(bcl_dev->modem_gpio1_pin, 0);
		if (!bcl_dev->sub_pwr_warn_triggered[i])
			pwrwarn_update_end_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
						MMWAVE_BCL_BIN);
		else
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
							bcl_dev->sub_pwr_warn_triggered,
							MMWAVE_BCL_BIN);
	}

	mutex_unlock(&bcl_dev->sub_odpm->lock);

	if (revisit_needed)
		mod_delayed_work(system_unbound_wq, &bcl_dev->sub_pwr_irq_work,
				 msecs_to_jiffies(PWRWARN_DELAY_MS));
#endif
}

static irqreturn_t sub_pwr_warn_irq_handler(int irq, void *data)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct bcl_device *bcl_dev = data;
	int i;

	if (!bcl_dev->enabled)
		return IRQ_HANDLED;
	mutex_lock(&bcl_dev->sub_odpm->lock);

	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		if (bcl_dev->sub_pwr_warn_irq[i] == irq) {
			bcl_dev->sub_pwr_warn_triggered[i] = 1;
			/* Check for Modem MMWAVE */
			if (i == bcl_dev->rffe_channel && bcl_dev->config_modem)
				gpio_set_value(bcl_dev->modem_gpio1_pin, 1);

			/* Setup Timer to clear the triggered */
			mod_delayed_work(system_unbound_wq, &bcl_dev->sub_pwr_irq_work,
					 msecs_to_jiffies(PWRWARN_DELAY_MS));
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
							bcl_dev->sub_pwr_warn_triggered,
							MMWAVE_BCL_BIN);
			break;
		}
	}

	mutex_unlock(&bcl_dev->sub_odpm->lock);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t main_pwr_warn_irq_handler(int irq, void *data)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct bcl_device *bcl_dev = data;
	int i;

	if (!bcl_dev->enabled)
		return IRQ_HANDLED;
	mutex_lock(&bcl_dev->main_odpm->lock);

	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		if (bcl_dev->main_pwr_warn_irq[i] == irq) {
			bcl_dev->main_pwr_warn_triggered[i] = 1;
			/* Check for Modem RFFE */
			if (i == bcl_dev->rffe_channel && bcl_dev->config_modem)
				gpio_set_value(bcl_dev->modem_gpio1_pin, 1);

			/* Setup Timer to clear the triggered */
			mod_delayed_work(system_unbound_wq, &bcl_dev->main_pwr_irq_work,
					 msecs_to_jiffies(PWRWARN_DELAY_MS));
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
							bcl_dev->main_pwr_warn_triggered,
							RFFE_BCL_BIN);
			break;
		}
	}

	mutex_unlock(&bcl_dev->main_odpm->lock);
#endif
	return IRQ_HANDLED;
}

static int google_set_sub_pmic(struct bcl_device *bcl_dev)
{
	struct s2mpg15_platform_data *pdata_sub;
	struct s2mpg15_dev *sub_dev = NULL;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	u8 val = 0;
	int ret, i, rail_i;

	INIT_DELAYED_WORK(&bcl_dev->sub_pwr_irq_work, sub_pwrwarn_irq_work);

	p_np = of_parse_phandle(np, "google,sub-power", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find sub-power I2C\n");
			return -ENODEV;
		}
		sub_dev = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!sub_dev) {
		dev_err(bcl_dev->device, "SUB PMIC device not found\n");
		return -ENODEV;
	}
	pdata_sub = dev_get_platdata(sub_dev->dev);
	bcl_dev->sub_odpm = pdata_sub->meter;
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		rail_i = bcl_dev->sub_odpm->channels[i].rail_i;
		bcl_dev->sub_rail_names[i] = bcl_dev->sub_odpm->chip.rails[rail_i].schematic_name;
	}
	bcl_dev->sub_irq_base = pdata_sub->irq_base;
	bcl_dev->sub_pmic_i2c = sub_dev->pmic;
	bcl_dev->sub_meter_i2c = sub_dev->meter;
	bcl_dev->sub_dev = sub_dev->dev;
	if (pmic_read(CORE_PMIC_SUB, bcl_dev, SUB_CHIPID, &val)) {
		dev_err(bcl_dev->device, "Failed to read PMIC chipid.\n");
		return -ENODEV;
	}
	pmic_read(CORE_PMIC_SUB, bcl_dev, S2MPG15_PM_OFFSRC1, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC1 : %#x\n", val);
	bcl_dev->sub_offsrc1 = val;
	pmic_read(CORE_PMIC_SUB, bcl_dev, S2MPG15_PM_OFFSRC2, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC2 : %#x\n", val);
	bcl_dev->sub_offsrc2 = val;
	pmic_write(CORE_PMIC_SUB, bcl_dev, S2MPG15_PM_OFFSRC1, 0);
	pmic_write(CORE_PMIC_SUB, bcl_dev, S2MPG15_PM_OFFSRC2, 0);

	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_GPU, "GPU_OCP_IRQ",
				       pdata_sub->b2_ocp_warn_pin,
				       GPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_sub->b2_ocp_warn_lvl * GPU_STEP),
				       gpio_to_irq(pdata_sub->b2_ocp_warn_pin),
				       CORE_SUB_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: GPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_GPU, "SOFT_GPU_OCP_IRQ",
				       pdata_sub->b2_soft_ocp_warn_pin,
				       GPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_sub->b2_soft_ocp_warn_lvl * GPU_STEP),
				       gpio_to_irq(pdata_sub->b2_soft_ocp_warn_pin),
				       CORE_SUB_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_GPU\n");
		return -ENODEV;
	}
	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		bcl_dev->sub_pwr_warn_irq[i] =
				bcl_dev->sub_irq_base + S2MPG15_IRQ_PWR_WARN_CH0_INT5 + i;
		ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->sub_pwr_warn_irq[i],
						NULL, sub_pwr_warn_irq_handler, 0,
						bcl_dev->sub_rail_names[i], bcl_dev);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to request PWR_WARN_CH%d IRQ: %d: %d\n",
				i, bcl_dev->sub_pwr_warn_irq[i], ret);
		}
	}

	return 0;
}

static int get_idx_from_tz(struct bcl_device *bcl_dev, const char *name)
{
	int i;
	struct bcl_zone *zone;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		zone = bcl_dev->zone[i];
		if (!zone)
			continue;
		if (!strcmp(name, zone->tz->type))
			return i;
	}
	return -EINVAL;
}

static void google_bcl_parse_qos(struct bcl_device *bcl_dev)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct device_node *np = bcl_dev->device->of_node;
	struct device_node *child;
	struct device_node *p_np;
	int idx;

	/* parse qos */
	p_np = of_get_child_by_name(np, "freq_qos");
	if (!p_np)
		return;
	for_each_child_of_node(p_np, child) {
		idx = get_idx_from_tz(bcl_dev, child->name);
		if (idx < 0)
			continue;
		bcl_dev->zone[idx]->bcl_qos = devm_kzalloc(bcl_dev->device,
		                                           sizeof(struct qos_throttle_limit),
		                                           GFP_KERNEL);
		bcl_dev->zone[idx]->bcl_qos->throttle = false;
		if (of_property_read_u32(child, "cpucl0",
					 &bcl_dev->zone[idx]->bcl_qos->cpu0_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->cpu0_limit = INT_MAX;
		if (of_property_read_u32(child, "cpucl1",
					 &bcl_dev->zone[idx]->bcl_qos->cpu1_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->cpu1_limit = INT_MAX;
		if (of_property_read_u32(child, "cpucl2",
					 &bcl_dev->zone[idx]->bcl_qos->cpu2_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->cpu2_limit = INT_MAX;
		if (of_property_read_u32(child, "gpu",
					 &bcl_dev->zone[idx]->bcl_qos->gpu_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->gpu_limit = INT_MAX;
		if (of_property_read_u32(child, "tpu",
					 &bcl_dev->zone[idx]->bcl_qos->tpu_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->tpu_limit = INT_MAX;
	}
#endif
}

static int intf_pmic_init(struct bcl_device *bcl_dev)
{
	int ret;
	u8 val;
	u32 retval;
	unsigned int uvlo1_lvl, uvlo2_lvl, batoilo_lvl, batoilo2_lvl, lvl, regval;

	bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	batoilo_reg_read(bcl_dev->intf_pmic_i2c, bcl_dev->ifpmic, BATOILO2, &lvl);
	batoilo2_lvl = BO_STEP * lvl + bcl_dev->batt_irq_conf1.batoilo_lower_limit;
	batoilo_reg_read(bcl_dev->intf_pmic_i2c, bcl_dev->ifpmic, BATOILO1, &lvl);
	batoilo_lvl = BO_STEP * lvl + bcl_dev->batt_irq_conf1.batoilo_lower_limit;
	uvlo_reg_read(bcl_dev->intf_pmic_i2c, bcl_dev->ifpmic, UVLO1, &uvlo1_lvl);
	uvlo_reg_read(bcl_dev->intf_pmic_i2c, bcl_dev->ifpmic, UVLO2, &uvlo2_lvl);

	ret = google_bcl_register_zone(bcl_dev, UVLO1, "UVLO1", bcl_dev->vdroop1_pin,
				       VD_BATTERY_VOLTAGE - uvlo1_lvl - THERMAL_HYST_LEVEL,
				       gpio_to_irq(bcl_dev->vdroop1_pin), IF_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: UVLO1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, UVLO2, "UVLO2", bcl_dev->vdroop2_pin,
				       VD_BATTERY_VOLTAGE - uvlo2_lvl - THERMAL_HYST_LEVEL,
				       gpio_to_irq(bcl_dev->vdroop2_pin), IF_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: UVLO2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, BATOILO1, "BATOILO1", bcl_dev->vdroop2_pin,
				       batoilo_lvl - THERMAL_HYST_LEVEL,
				       gpio_to_irq(bcl_dev->vdroop2_pin), IF_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: BATOILO\n");
		return -ENODEV;
	}
	if (bcl_dev->ifpmic == MAX77779) {
		ret = google_bcl_register_zone(bcl_dev, BATOILO2, "BATOILO2", bcl_dev->vdroop2_pin,
					       batoilo2_lvl - THERMAL_HYST_LEVEL,
					       gpio_to_irq(bcl_dev->vdroop2_pin), IF_PMIC);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: BATOILO2\n");
			return -ENODEV;
		}
		/* Setup mitigation IRQ */
		ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_i2c,
		                                       MAX77779_PMIC_VDROOP_INT_MASK, 0x0);
		ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_i2c,
		                                      MAX77779_PMIC_INTB_MASK, &retval);
		val = 0;
		retval = _max77779_pmic_intb_mask_vdroop_int_m_set(retval, val);
		ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_i2c,
		                                       MAX77779_PMIC_INTB_MASK, retval);

		/* UVLO2 no VDROOP2 */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_SYS_UVLO2_CNFG_1, &val);
		val = _max77779_sys_uvlo2_cnfg_1_sys_uvlo2_vdrp2_en_set(val, 0);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
						      MAX77779_SYS_UVLO2_CNFG_1, val);
		val = _max77779_sys_uvlo2_cnfg_0_sys_uvlo2_set(val, 0xc);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_SYS_UVLO2_CNFG_0, val);
		/* UVLO1 = VDROOP1, 3.1V */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_SYS_UVLO1_CNFG_1, &val);
		val = _max77779_sys_uvlo1_cnfg_1_sys_uvlo1_vdrp1_en_set(val, 1);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_SYS_UVLO1_CNFG_1, val);
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_SYS_UVLO1_CNFG_0, &val);
		val = _max77779_sys_uvlo1_cnfg_0_sys_uvlo1_set(val, 0xa);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_SYS_UVLO1_CNFG_0, val);

		/* BATOILO1 = VDROOP2, 36ms BATOILO1 BAT_OPEN */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO1_CNFG_3, &val);
		val = _max77779_bat_oilo1_cnfg_3_bat_oilo1_vdrp1_en_set(val, 0);
		val = _max77779_bat_oilo1_cnfg_3_bat_oilo1_vdrp2_en_set(val, 1);
		val = _max77779_bat_oilo1_cnfg_3_bat_open_to_1_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_bat_open_to);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_BAT_OILO1_CNFG_3, val);

		/* BATOILO2 no VDROOP1/2, 12ms BATOILO2 BAT_OPEN */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO2_CNFG_3, &val);
		val = _max77779_bat_oilo2_cnfg_3_bat_oilo2_vdrp1_en_set(val, 0);
		val = _max77779_bat_oilo2_cnfg_3_bat_oilo2_vdrp2_en_set(val, 0);
		val = _max77779_bat_oilo2_cnfg_3_bat_open_to_2_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_bat_open_to);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_BAT_OILO2_CNFG_3, val);

		/* BATOILO1 5A THRESHOLD */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO1_CNFG_0, &val);
		val = _max77779_bat_oilo1_cnfg_0_bat_oilo1_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_trig_lvl);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_BAT_OILO1_CNFG_0, val);

		/* BATOILO2 8A THRESHOLD */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO2_CNFG_0, &val);
		val = _max77779_bat_oilo2_cnfg_0_bat_oilo2_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_trig_lvl);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
						      MAX77779_BAT_OILO2_CNFG_0, val);

		/* BATOILO INT and VDROOP1 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO1_CNFG_1, &val);
		val = _max77779_bat_oilo1_cnfg_1_bat_oilo1_rel_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_rel);
		val = _max77779_bat_oilo1_cnfg_1_bat_oilo1_det_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_BAT_OILO1_CNFG_1, val);

		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO1_CNFG_2, &val);
		val = _max77779_bat_oilo1_cnfg_2_bat_oilo1_int_rel_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_rel);
		val = _max77779_bat_oilo1_cnfg_2_bat_oilo1_int_det_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_BAT_OILO1_CNFG_2, val);

		/* BATOILO2 INT and VDROOP2 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO2_CNFG_1, &val);
		val = _max77779_bat_oilo2_cnfg_1_bat_oilo2_rel_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_rel);
		val = _max77779_bat_oilo2_cnfg_1_bat_oilo2_det_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_BAT_OILO2_CNFG_1, val);

		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_BAT_OILO2_CNFG_2, &val);
		val = _max77779_bat_oilo2_cnfg_2_bat_oilo2_int_rel_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_rel);
		val = _max77779_bat_oilo2_cnfg_2_bat_oilo2_int_det_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_BAT_OILO2_CNFG_2, val);

		/* UVLO1 INT and VDROOP1 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_SYS_UVLO1_CNFG_1, &val);
		val = _max77779_sys_uvlo1_cnfg_1_sys_uvlo1_rel_set(
						 val, bcl_dev->batt_irq_conf1.uvlo_rel);
		val = _max77779_sys_uvlo1_cnfg_1_sys_uvlo1_det_set(
						 val, bcl_dev->batt_irq_conf1.uvlo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_SYS_UVLO1_CNFG_1, val);

		/* UVLO2 INT and VDROOP1 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_i2c,
						     MAX77779_SYS_UVLO2_CNFG_1, &val);
		val = _max77779_sys_uvlo2_cnfg_1_sys_uvlo2_rel_set(
						 val, bcl_dev->batt_irq_conf2.uvlo_rel);
		val = _max77779_sys_uvlo2_cnfg_1_sys_uvlo2_det_set(
						 val, bcl_dev->batt_irq_conf2.uvlo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_i2c,
		                                      MAX77779_SYS_UVLO2_CNFG_1, val);

		/* Read, save, and clear event counters */
		ret = evt_cnt_rd_and_clr(bcl_dev, UVLO1, &regval);
		if (ret == 0)
			bcl_dev->evt_cnt.uvlo1 = regval;

		ret = evt_cnt_rd_and_clr(bcl_dev, UVLO2, &regval);
		if (ret == 0)
			bcl_dev->evt_cnt.uvlo2 = regval;

		ret = evt_cnt_rd_and_clr(bcl_dev, BATOILO1, &regval);
		if (ret == 0)
			bcl_dev->evt_cnt.batoilo1 = regval;

		ret = evt_cnt_rd_and_clr(bcl_dev, BATOILO2, &regval);
		if (ret == 0)
			bcl_dev->evt_cnt.batoilo2 = regval;

		/* Enable event counter if it is not enabled */
		ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_i2c,
						 MAX77779_PMIC_EVENT_CNT_CFG, &retval);
		retval = _max77779_pmic_event_cnt_cfg_enable_set(
						 retval, bcl_dev->evt_cnt.enable);
		retval = _max77779_pmic_event_cnt_cfg_sample_rate_set(
						 retval, bcl_dev->evt_cnt.rate);
		ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_i2c,
						  MAX77779_PMIC_EVENT_CNT_CFG, retval);
	}
	return ret;
}

static int google_set_intf_pmic(struct bcl_device *bcl_dev, struct platform_device *pdev)
{
	int i, ret = 0;
	u32 retval;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;

	p_np = of_parse_phandle(np, "google,charger", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find Charger I2C\n");
			return -ENODEV;
		}
		if (!strcmp(i2c->name, "max77779chrg") ||
		    !strcmp(i2c->name, "max77779chrg_i2c"))
			bcl_dev->ifpmic = MAX77779;
		else
			bcl_dev->ifpmic = MAX77759;
		bcl_dev->intf_pmic_i2c = i2c;
		bcl_dev->irq_pmic_i2c = i2c;
	}
	of_node_put(p_np);

	if (bcl_dev->ifpmic == MAX77779) {
		google_bcl_setup_votable(bcl_dev);
		ret = platform_get_irq(pdev, 0);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to get irq: %d\n", ret);
			return -ENODEV;
		}
		bcl_dev->pmic_irq = ret;
	}

	if (np) {
		ret = of_property_read_u32(np, "batoilo_lower", &retval);
		bcl_dev->batt_irq_conf1.batoilo_lower_limit = ret ? BO_LOWER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo_upper", &retval);
		bcl_dev->batt_irq_conf1.batoilo_upper_limit = ret ? BO_UPPER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo2_lower", &retval);
		bcl_dev->batt_irq_conf2.batoilo_lower_limit = ret ? BO_LOWER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo2_upper", &retval);
		bcl_dev->batt_irq_conf2.batoilo_upper_limit = ret ? BO_UPPER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo_trig_lvl", &retval);
		retval = ret ? BO_LIMIT : retval;
		bcl_dev->batt_irq_conf1.batoilo_trig_lvl =
				(retval - bcl_dev->batt_irq_conf1.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo2_trig_lvl", &retval);
		retval = ret ? BO_LIMIT : retval;
		bcl_dev->batt_irq_conf2.batoilo_trig_lvl =
				(retval - bcl_dev->batt_irq_conf2.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo_wlc_trig_lvl", &retval);
		bcl_dev->batt_irq_conf1.batoilo_wlc_trig_lvl = ret ?
				bcl_dev->batt_irq_conf1.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf1.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo2_wlc_trig_lvl", &retval);
		bcl_dev->batt_irq_conf2.batoilo_wlc_trig_lvl = ret ?
				bcl_dev->batt_irq_conf2.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf2.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo_bat_open_to", &retval);
		bcl_dev->batt_irq_conf1.batoilo_bat_open_to = ret ? BO_BAT_OPEN_TO_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_bat_open_to", &retval);
		bcl_dev->batt_irq_conf2.batoilo_bat_open_to = ret ? BO_BAT_OPEN_TO_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo_rel", &retval);
		bcl_dev->batt_irq_conf1.batoilo_rel = ret ? BO_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_rel", &retval);
		bcl_dev->batt_irq_conf2.batoilo_rel = ret ? BO_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo_det", &retval);
		bcl_dev->batt_irq_conf1.batoilo_det = ret ? BO_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_det", &retval);
		bcl_dev->batt_irq_conf2.batoilo_det = ret ? BO_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo1_det", &retval);
		bcl_dev->batt_irq_conf1.uvlo_det = ret ? UV_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo2_det", &retval);
		bcl_dev->batt_irq_conf2.uvlo_det = ret ? UV_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo1_rel", &retval);
		bcl_dev->batt_irq_conf1.uvlo_rel = ret ? UV_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo2_rel", &retval);
		bcl_dev->batt_irq_conf2.uvlo_rel = ret ? UV_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "evt_cnt_enable", &retval);
		bcl_dev->evt_cnt.enable = ret ? EVT_CNT_ENABLE_DEFAULT : retval;
		ret = of_property_read_u32(np, "evt_cnt_rate", &retval);
		bcl_dev->evt_cnt.rate = ret ? EVT_CNT_RATE_DEFAULT : retval;
	}

	if (!bcl_dev->intf_pmic_i2c) {
		dev_err(bcl_dev->device, "Interface PMIC device not found\n");
		return -ENODEV;
	}
	if (bcl_dev->ifpmic == MAX77779) {
		p_np = of_parse_phandle(np, "google,pmic", 0);
		if (p_np) {
			i2c = of_find_i2c_device_by_node(p_np);
			if (!i2c) {
				dev_err(bcl_dev->device, "Cannot find PMIC I2C\n");
				return -ENODEV;
			}
			bcl_dev->irq_pmic_i2c = i2c;
		}
		of_node_put(p_np);
	}

	INIT_DELAYED_WORK(&bcl_dev->soc_work, google_bcl_evaluate_soc);
	bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	bcl_dev->soc_tz_ops.get_temp = tz_bcl_read_soc;
	bcl_dev->soc_tz_ops.set_trips = tz_bcl_set_soc;
	bcl_dev->soc_tz = devm_thermal_of_zone_register(bcl_dev->device, PMIC_SOC, bcl_dev,
							&bcl_dev->soc_tz_ops);
	if (IS_ERR(bcl_dev->soc_tz)) {
		dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
			PTR_ERR(bcl_dev->soc_tz));
		ret = PTR_ERR(bcl_dev->soc_tz);
		bcl_dev->soc_tz = NULL;
	} else {
		bcl_dev->psy_nb.notifier_call = battery_supply_callback;
		ret = power_supply_reg_notifier(&bcl_dev->psy_nb);
		if (ret < 0)
			dev_err(bcl_dev->device,
				"soc notifier registration error. defer. err:%d\n", ret);
		thermal_zone_device_update(bcl_dev->soc_tz, THERMAL_DEVICE_UP);
	}
	bcl_dev->batt_psy_initialized = false;

	ret = intf_pmic_init(bcl_dev);
	if (ret < 0) {
		dev_err(bcl_dev->device, "Interface PMIC initialization err:%d\n", ret);
		return ret;
	}

	google_bcl_parse_qos(bcl_dev);
	if (google_bcl_setup_qos(bcl_dev) != 0) {
#if IS_ENABLED(CONFIG_SOC_ZUMA)
		dev_err(bcl_dev->device, "Cannot Initiate QOS\n");
		return -ENODEV;
#endif
	}

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (bcl_dev->ifpmic == MAX77779) {
			if (bcl_dev->zone[i] && (i != BATOILO2) && (i != UVLO2)) {
				bcl_dev->zone[i]->disabled = false;
				enable_irq(bcl_dev->zone[i]->bcl_irq);
			}
		} else {
			if (bcl_dev->zone[i] && (i != BATOILO)) {
				bcl_dev->zone[i]->disabled = false;
				enable_irq(bcl_dev->zone[i]->bcl_irq);
			}
		}
	}

	return 0;
}

static int google_set_main_pmic(struct bcl_device *bcl_dev)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct s2mpg14_platform_data *pdata_main;
	struct s2mpg14_dev *main_dev = NULL;
#elif IS_ENABLED(CONFIG_SOC_GS101)
	struct s2mpg10_platform_data *pdata_main;
	struct s2mpg10_dev *main_dev = NULL;
#endif
	u8 val;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	int ret, i, rail_i;

	INIT_DELAYED_WORK(&bcl_dev->main_pwr_irq_work, main_pwrwarn_irq_work);

	p_np = of_parse_phandle(np, "google,main-power", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find main-power I2C\n");
			return -ENODEV;
		}
		main_dev = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!main_dev) {
		dev_err(bcl_dev->device, "Main PMIC device not found\n");
		return -ENODEV;
	}
	pdata_main = dev_get_platdata(main_dev->dev);
	bcl_dev->main_odpm = pdata_main->meter;
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		rail_i = bcl_dev->main_odpm->channels[i].rail_i;
		bcl_dev->main_rail_names[i] = bcl_dev->main_odpm->chip.rails[rail_i].schematic_name;
	}
	bcl_dev->main_irq_base = pdata_main->irq_base;
	bcl_dev->main_pmic_i2c = main_dev->pmic;
	bcl_dev->main_meter_i2c = main_dev->meter;
	bcl_dev->main_dev = main_dev->dev;
	/* clear MAIN information every boot */
	/* see b/215371539 */
	pmic_read(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_OFFSRC1, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC1 : %#x\n", val);
	bcl_dev->main_offsrc1 = val;
	pmic_read(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_OFFSRC2, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC2 : %#x\n", val);
	bcl_dev->main_offsrc2 = val;
	pmic_read(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_PWRONSRC, &val);
	dev_info(bcl_dev->device, "MAIN PWRONSRC: %#x\n", val);
	bcl_dev->pwronsrc = val;
	pmic_write(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_OFFSRC1, 0);
	pmic_write(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_OFFSRC2, 0);
	pmic_write(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_PWRONSRC, 0);
	/* SMPL_WARN = 3.0V */
	pmic_write(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_SMPL_WARN_CTRL, 0x8b);

	ret = google_bcl_register_zone(bcl_dev, SMPL_WARN, "SMPL_WARN_IRQ",
				       pdata_main->smpl_warn_pin, SMPL_BATTERY_VOLTAGE -
				       (pdata_main->smpl_warn_lvl * SMPL_STEP + SMPL_LOWER_LIMIT),
				       gpio_to_irq(pdata_main->smpl_warn_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SMPL_WARN\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_CPUCL1, "CPU1_OCP_IRQ",
				       pdata_main->b3_ocp_warn_pin,
				       CPU1_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_main->b3_ocp_warn_lvl * CPU1_STEP),
				       gpio_to_irq(pdata_main->b3_ocp_warn_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_CPUCL2, "CPU2_OCP_IRQ",
				       pdata_main->b2_ocp_warn_pin,
				       CPU2_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_main->b2_ocp_warn_lvl * CPU2_STEP),
				       gpio_to_irq(pdata_main->b2_ocp_warn_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_CPUCL1, "SOFT_CPU1_OCP_IRQ",
				       pdata_main->b3_soft_ocp_warn_pin,
				       CPU1_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_main->b3_soft_ocp_warn_lvl * CPU1_STEP),
				       gpio_to_irq(pdata_main->b3_soft_ocp_warn_pin),
				       CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_CPUCL2, "SOFT_CPU2_OCP_IRQ",
				       pdata_main->b2_soft_ocp_warn_pin,
				       CPU2_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_main->b2_soft_ocp_warn_lvl * CPU2_STEP),
				       gpio_to_irq(pdata_main->b2_soft_ocp_warn_pin),
				       CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_TPU, "TPU_OCP_IRQ",
				       pdata_main->b7_ocp_warn_pin,
				       TPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_main->b7_ocp_warn_lvl * TPU_STEP),
				       gpio_to_irq(pdata_main->b7_ocp_warn_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: TPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_TPU, "SOFT_TPU_OCP_IRQ",
				       pdata_main->b7_soft_ocp_warn_pin,
				       TPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata_main->b7_soft_ocp_warn_lvl * TPU_STEP),
				       gpio_to_irq(pdata_main->b7_soft_ocp_warn_pin),
				       CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_TPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, PMIC_120C, "PMIC_120C", 0,
				       PMIC_120C_UPPER_LIMIT - THERMAL_HYST_LEVEL,
				       pdata_main->irq_base + INT3_120C, CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_120C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, PMIC_140C, "PMIC_140C", 0,
				       PMIC_140C_UPPER_LIMIT - THERMAL_HYST_LEVEL,
				       pdata_main->irq_base + INT3_140C, CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_140C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, PMIC_OVERHEAT, "PMIC_OVERHEAT", 0,
				       PMIC_OVERHEAT_UPPER_LIMIT - THERMAL_HYST_LEVEL,
				       pdata_main->irq_base + INT3_TSD, CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_OVERHEAT\n");
		return -ENODEV;
	}
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		bcl_dev->main_pwr_warn_irq[i] = bcl_dev->main_irq_base
				+ S2MPG14_IRQ_PWR_WARN_CH0_INT6 + i;
		ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->main_pwr_warn_irq[i],
						NULL, main_pwr_warn_irq_handler, 0,
						bcl_dev->main_rail_names[i], bcl_dev);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to request PWR_WARN_CH%d IRQ: %d: %d\n",
				i, bcl_dev->main_pwr_warn_irq[i], ret);
		}
	}
#endif


	return 0;

}

extern const struct attribute_group *mitigation_mw_groups[];
extern const struct attribute_group *mitigation_sq_groups[];

static int google_init_fs(struct bcl_device *bcl_dev)
{
	if (bcl_dev->ifpmic == MAX77759)
		bcl_dev->mitigation_dev = pmic_subdevice_create(NULL, mitigation_mw_groups,
								bcl_dev, "mitigation");
	else
		bcl_dev->mitigation_dev = pmic_subdevice_create(NULL, mitigation_sq_groups,
								bcl_dev, "mitigation");
	if (IS_ERR(bcl_dev->mitigation_dev))
		return -ENODEV;

	return 0;
}

static void google_bcl_enable_vdroop_irq(struct bcl_device *bcl_dev)
{
	void __iomem *gpio_alive;
	unsigned int reg;

	gpio_alive = ioremap(GPIO_ALIVE_BASE, SZ_4K);
	reg = __raw_readl(gpio_alive + GPA9_CON);
	reg |= 0xFF0000;
	__raw_writel(0xFFFFF22, gpio_alive + GPA9_CON);
}

static int google_bcl_init_instruction(struct bcl_device *bcl_dev)
{
	if (!bcl_dev)
		return -EIO;

	bcl_dev->core_conf[SUBSYSTEM_CPU0].base_mem = devm_ioremap(bcl_dev->device,
	                                                           CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_CPU0].base_mem) {
		dev_err(bcl_dev->device, "cpu0_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_CPU1].base_mem = devm_ioremap(bcl_dev->device,
	                                                           CPUCL1_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_CPU1].base_mem) {
		dev_err(bcl_dev->device, "cpu1_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_CPU2].base_mem = devm_ioremap(bcl_dev->device,
	                                                           CPUCL2_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_CPU2].base_mem) {
		dev_err(bcl_dev->device, "cpu2_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_TPU].base_mem = devm_ioremap(bcl_dev->device,
	                                                          TPU_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_TPU].base_mem) {
		dev_err(bcl_dev->device, "tpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_GPU].base_mem = devm_ioremap(bcl_dev->device,
	                                                          G3D_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_GPU].base_mem) {
		dev_err(bcl_dev->device, "gpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_AUR].base_mem = devm_ioremap(bcl_dev->device,
	                                                          AUR_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_AUR].base_mem) {
		dev_err(bcl_dev->device, "aur_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->sysreg_cpucl0 = devm_ioremap(bcl_dev->device, SYSREG_CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->sysreg_cpucl0) {
		dev_err(bcl_dev->device, "sysreg_cpucl0 ioremap failed\n");
		return -EIO;
	}

	mutex_init(&bcl_dev->state_trans_lock);
	mutex_init(&bcl_dev->cpu_ratio_lock);
	mutex_init(&bcl_dev->tpu_ratio_lock);
	mutex_init(&bcl_dev->gpu_ratio_lock);
	mutex_init(&bcl_dev->aur_ratio_lock);
	google_bcl_enable_vdroop_irq(bcl_dev);

	bcl_dev->base_add_mem[SUBSYSTEM_CPU0] = devm_ioremap(bcl_dev->device, ADD_CPUCL0, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_CPU0]) {
		dev_err(bcl_dev->device, "cpu0_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_CPU1] = devm_ioremap(bcl_dev->device, ADD_CPUCL1, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_CPU1]) {
		dev_err(bcl_dev->device, "cpu1_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_CPU2] = devm_ioremap(bcl_dev->device, ADD_CPUCL2, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_CPU2]) {
		dev_err(bcl_dev->device, "cpu2_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_TPU] = devm_ioremap(bcl_dev->device, ADD_TPU, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_TPU]) {
		dev_err(bcl_dev->device, "tpu_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_GPU] = devm_ioremap(bcl_dev->device, ADD_G3D, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_GPU]) {
		dev_err(bcl_dev->device, "gpu_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_AUR] = devm_ioremap(bcl_dev->device, ADD_AUR, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_AUR]) {
		dev_err(bcl_dev->device, "aur_add_mem ioremap failed\n");
		return -EIO;
	}
	return 0;
}

u64 settings_to_current(struct bcl_device *bcl_dev, int pmic, int idx, u32 setting)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	int rail_i;
	enum s2mpg1415_meter_muxsel muxsel;
	struct odpm_info *info;
	u64 raw_unit;
	u32 resolution;

	if (!bcl_dev)
		return 0;
	if (pmic == CORE_PMIC_MAIN)
		info = bcl_dev->main_odpm;
	else
		info = bcl_dev->sub_odpm;

	if (!info)
		return 0;

	rail_i = info->channels[idx].rail_i;
	muxsel = info->chip.rails[rail_i].mux_select;
	if ((strstr(bcl_dev->main_rail_names[idx], "VSYS") != NULL) ||
		(strstr(bcl_dev->sub_rail_names[idx], "VSYS") != NULL)) {
			resolution = (u32) VSHUNT_MULTIPLIER * ((u64)EXTERNAL_RESOLUTION_VSHUNT) /
					info->chip.rails[rail_i].shunt_uohms;
	} else {
		if (pmic == CORE_PMIC_MAIN)
			resolution = s2mpg14_muxsel_to_current_resolution(muxsel);
		else
			resolution = s2mpg15_muxsel_to_current_resolution(muxsel);
	}
	raw_unit = (u64)setting * resolution;
	raw_unit = raw_unit * MILLI_TO_MICRO;
	return (u32)_IQ30_to_int(raw_unit);
#endif
        return 0;
}

static void google_bcl_parse_dtree(struct bcl_device *bcl_dev)
{
	int ret, i = 0;
	struct device_node *np = bcl_dev->device->of_node;
	struct device_node *child;
	struct device_node *p_np;
	u32 val;
	int read;

	if (!bcl_dev) {
		dev_err(bcl_dev->device, "Cannot parse device tree\n");
		return;
	}
	ret = of_property_read_u32(np, "tpu_con_heavy", &val);
	bcl_dev->core_conf[SUBSYSTEM_TPU].con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_con_light", &val);
	bcl_dev->core_conf[SUBSYSTEM_TPU].con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_heavy", &val);
	bcl_dev->core_conf[SUBSYSTEM_GPU].con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_light", &val);
	bcl_dev->core_conf[SUBSYSTEM_GPU].con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_GPU].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_TPU].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "aur_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_AUR].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu2_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_CPU2].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu1_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_CPU1].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu0_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_CPU0].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "irq_enable_delay", &val);
	bcl_dev->irq_delay = ret ? IRQ_ENABLE_DELAY_MS : val;
	bcl_dev->vdroop1_pin = of_get_gpio(np, 0);
	bcl_dev->vdroop2_pin = of_get_gpio(np, 1);
	bcl_dev->modem_gpio1_pin = of_get_gpio(np, 2);
	bcl_dev->modem_gpio2_pin = of_get_gpio(np, 3);
	ret = of_property_read_u32(np, "rffe_channel", &val);
	bcl_dev->rffe_channel = ret ? 11 : val;
	ret = of_property_read_u32(np, "cpu0_cluster", &val);
	bcl_dev->cpu0_cluster = ret ? CPU0_CLUSTER_MIN : val;
	ret = of_property_read_u32(np, "cpu1_cluster", &val);
	bcl_dev->cpu1_cluster = ret ? CPU1_CLUSTER_MIN : val;
	ret = of_property_read_u32(np, "cpu2_cluster", &val);
	bcl_dev->cpu2_cluster = ret ? CPU2_CLUSTER_MIN : val;

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	/* parse ODPM main limit */
	p_np = of_get_child_by_name(np, "main_limit");
	if (p_np) {
		for_each_child_of_node(p_np, child) {
			of_property_read_u32(child, "setting", &read);
			if (i < METER_CHANNEL_MAX) {
				bcl_dev->main_setting[i] = read;
				meter_write(CORE_PMIC_MAIN, bcl_dev,
				            S2MPG14_METER_PWR_WARN0 + i, read);
				bcl_dev->main_limit[i] =
				    settings_to_current(bcl_dev, CORE_PMIC_MAIN, i,
				                        read << LPF_CURRENT_SHIFT);
				i++;
			}
		}
	}

	/* parse ODPM sub limit */
	p_np = of_get_child_by_name(np, "sub_limit");
	i = 0;
	if (p_np) {
		for_each_child_of_node(p_np, child) {
			of_property_read_u32(child, "setting", &read);
			if (i < METER_CHANNEL_MAX) {
				bcl_dev->sub_setting[i] = read;
				meter_write(CORE_PMIC_SUB, bcl_dev,
				            S2MPG15_METER_PWR_WARN0 + i, read);
				bcl_dev->sub_limit[i] =
				    settings_to_current(bcl_dev, CORE_PMIC_SUB, i,
				                        read << LPF_CURRENT_SHIFT);
				i++;
			}
		}
	}
#endif

	if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU2,
				    bcl_dev->core_conf[SUBSYSTEM_CPU2].clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU2 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU1,
				    bcl_dev->core_conf[SUBSYSTEM_CPU1].clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU1 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU0,
	                            bcl_dev->core_conf[SUBSYSTEM_CPU0].clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU0 Address is NULL\n");
}

static int google_bcl_configure_modem(struct bcl_device *bcl_dev)
{
	struct pinctrl *modem_pinctrl;
	struct pinctrl_state *batoilo_pinctrl_state, *rffe_pinctrl_state;
	int ret;

	modem_pinctrl = devm_pinctrl_get(bcl_dev->device);
	if (IS_ERR_OR_NULL(modem_pinctrl)) {
		dev_err(bcl_dev->device, "Cannot find modem_pinctrl!\n");
		return -EINVAL;
	}
	batoilo_pinctrl_state = pinctrl_lookup_state(modem_pinctrl, "bcl-batoilo-modem");
	if (IS_ERR_OR_NULL(batoilo_pinctrl_state)) {
		dev_err(bcl_dev->device, "batoilo: pinctrl lookup state failed!\n");
		return -EINVAL;
	}
	rffe_pinctrl_state = pinctrl_lookup_state(modem_pinctrl, "bcl-rffe-modem");
	if (IS_ERR_OR_NULL(rffe_pinctrl_state)) {
		dev_err(bcl_dev->device, "rffe: pinctrl lookup state failed!\n");
		return -EINVAL;
	}
	ret = pinctrl_select_state(modem_pinctrl, batoilo_pinctrl_state);
	if (ret < 0) {
		dev_err(bcl_dev->device, "batoilo: pinctrl select state failed!!\n");
		return -EINVAL;
	}
	ret = pinctrl_select_state(modem_pinctrl, rffe_pinctrl_state);
	if (ret < 0) {
		dev_err(bcl_dev->device, "rffe: pinctrl select state failed!!\n");
		return -EINVAL;
	}
	bcl_dev->config_modem = true;
	return 0;
}

static int google_bcl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bcl_device *bcl_dev;

	bcl_dev = devm_kzalloc(&pdev->dev, sizeof(*bcl_dev), GFP_KERNEL);
	if (!bcl_dev)
		return -ENOMEM;
	bcl_dev->device = &pdev->dev;

	mutex_init(&bcl_dev->sysreg_lock);
	platform_set_drvdata(pdev, bcl_dev);

	ret = google_bcl_init_data_logging(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	ret = google_bcl_init_instruction(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	if (google_set_main_pmic(bcl_dev) < 0)
		goto bcl_soc_probe_exit;
	if (google_set_sub_pmic(bcl_dev) < 0)
		goto bcl_soc_probe_exit;
	google_bcl_parse_dtree(bcl_dev);
	google_bcl_configure_modem(bcl_dev);

	if (google_set_intf_pmic(bcl_dev, pdev) < 0)
		goto bcl_soc_probe_exit;
	google_init_debugfs(bcl_dev);

	bcl_dev->triggered_idx = TRIGGERED_SOURCE_MAX;

	ret = google_init_fs(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	bcl_dev->enabled = true;

	return 0;

bcl_soc_probe_exit:
	dev_err(bcl_dev->device, "Retrying BCL probe\n");
	google_bcl_remove_thermal(bcl_dev);
	return -EPROBE_DEFER;
}

static int google_bcl_remove(struct platform_device *pdev)
{
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	pmic_device_destroy(bcl_dev->mitigation_dev->devt);
	debugfs_remove_recursive(bcl_dev->debug_entry);
	google_bcl_remove_thermal(bcl_dev);
	if (bcl_dev->enabled)
		google_bcl_remove_qos(bcl_dev);
	google_bcl_remove_votable(bcl_dev);
	google_bcl_remove_data_logging(bcl_dev);

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "google,google-bcl"},
	{},
};

static struct platform_driver google_bcl_driver = {
	.probe  = google_bcl_probe,
	.remove = google_bcl_remove,
	.id_table = google_id_table,
	.driver = {
		.name           = "google_mitigation",
		.owner          = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_platform_driver(google_bcl_driver);

MODULE_SOFTDEP("pre: i2c-acpm");
MODULE_DESCRIPTION("Google Battery Current Limiter");
MODULE_AUTHOR("George Lee <geolee@google.com>");
MODULE_LICENSE("GPL");
