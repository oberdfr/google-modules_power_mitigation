// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_core.c Google bcl core driver
 *
 * Copyright (c) 2022, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

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
#include <dt-bindings/interrupt-controller/gs201.h>
#include <linux/regulator/pmic_class.h>
#include <soc/google/odpm-whi.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include "bcl.h"
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include <max77759_regs.h>
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

static irqreturn_t irq_handler(int irq, void *data)
{
	struct bcl_zone *zone = data;
	struct bcl_device *bcl_dev;
	u8 idx;
	int gpio_level;
	u8 irq_val = 0;

	if (!zone || !zone->parent)
		return IRQ_HANDLED;

	idx = zone->idx;
	bcl_dev = zone->parent;
	if (!bcl_dev->ready)
		return IRQ_HANDLED;

	gpio_level = gpio_get_value(zone->bcl_pin);

	if (idx >= UVLO2 && idx <= BATOILO2) {
		bcl_cb_get_irq(bcl_dev, &irq_val);
		if (irq_val == 0)
			goto exit;
		idx = irq_val;
		zone = bcl_dev->zone[idx];
	}
	if (gpio_level == zone->polarity)
		mod_delayed_work(system_highpri_wq, &zone->irq_triggered_work, 0);
	else
		mod_delayed_work(system_highpri_wq, &zone->irq_untriggered_work, 0);
exit:
	return IRQ_HANDLED;
}
static bool google_warn_check(struct bcl_zone *zone)
{
	struct bcl_device *bcl_dev;
	int gpio_level;

	bcl_dev = zone->parent;
	gpio_level = gpio_get_value(zone->bcl_pin);
	return (gpio_level == zone->polarity);
}

static void google_warn_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, irq_work.work);
	struct bcl_device *bcl_dev;
	int idx;

	idx = zone->idx;
	bcl_dev = zone->parent;

	if (!google_warn_check(zone)) {
		zone->bcl_cur_lvl = 0;
		if (zone->irq_type == IF_PMIC)
			bcl_cb_clr_irq(bcl_dev);
	} else {
		zone->bcl_cur_lvl = zone->bcl_lvl + THERMAL_HYST_LEVEL;
		mod_delayed_work(system_unbound_wq, &zone->irq_work,
				 msecs_to_jiffies(THRESHOLD_DELAY_MS));
	}
	if (zone->tz)
		thermal_zone_device_update(zone->tz, THERMAL_EVENT_UNSPECIFIED);
	if (zone->irq_type != IF_PMIC && bcl_dev->irq_delay != 0) {
		if (!zone->disabled) {
			zone->disabled = true;
			disable_irq(zone->bcl_irq);
			mod_delayed_work(system_unbound_wq, &zone->enable_irq_work,
					 msecs_to_jiffies(bcl_dev->irq_delay));
		}
	}
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
	struct device *dev;

	power_supply_unreg_notifier(&bcl_dev->psy_nb);
	dev = bcl_dev->main_dev;
	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (i > SOFT_OCP_WARN_TPU)
			dev = bcl_dev->sub_dev;
		if (!bcl_dev->zone[i])
			continue;
		if (bcl_dev->zone[i]->tz)
			devm_thermal_of_zone_unregister(dev, bcl_dev->zone[i]->tz);
		cancel_delayed_work(&bcl_dev->zone[i]->irq_work);
		cancel_delayed_work(&bcl_dev->zone[i]->irq_triggered_work);
		cancel_delayed_work(&bcl_dev->zone[i]->irq_untriggered_work);
		cancel_delayed_work(&bcl_dev->zone[i]->enable_irq_work);
	}

	return 0;
}

static int google_bcl_init_clk_div(struct bcl_device *bcl_dev, int idx,
				   unsigned int value)
{
	void __iomem *addr;

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
	mutex_lock(&bcl_dev->ratio_lock);
	__raw_writel(value, addr);
	mutex_unlock(&bcl_dev->ratio_lock);

	return 0;
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

static int google_init_ratio(struct bcl_device *data, enum SUBSYSTEM_SOURCE idx)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;

	if (!bcl_is_subsystem_on(subsystem_pmu[idx]))
		return -EIO;

	if (idx < SUBSYSTEM_TPU)
		return -EIO;

	mutex_lock(&data->ratio_lock);
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
	mutex_unlock(&data->ratio_lock);

	return 0;
}

int google_init_tpu_ratio(struct bcl_device *data)
{
	return google_init_ratio(data, SUBSYSTEM_TPU);
}
EXPORT_SYMBOL_GPL(google_init_tpu_ratio);

int google_init_gpu_ratio(struct bcl_device *data)
{
	return google_init_ratio(data, SUBSYSTEM_GPU);
}
EXPORT_SYMBOL_GPL(google_init_gpu_ratio);

int google_init_aur_ratio(struct bcl_device *data)
{
	return google_init_ratio(data, SUBSYSTEM_AUR);
}
EXPORT_SYMBOL_GPL(google_init_aur_ratio);

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
	struct bcl_zone *zone = container_of(work, struct bcl_zone, irq_triggered_work.work);
	struct bcl_device *bcl_dev;
	int idx;

	idx = zone->idx;
	bcl_dev = zone->parent;

	if (idx == BATOILO)
		gpio_set_value(bcl_dev->modem_gpio2_pin, 1);

	if (bcl_dev->batt_psy_initialized) {
		atomic_inc(&zone->bcl_cnt);
		ocpsmpl_read_stats(bcl_dev, &zone->bcl_stats, bcl_dev->batt_psy);
		update_tz(zone, idx, true);
	}
	mod_delayed_work(system_unbound_wq, &zone->irq_work, msecs_to_jiffies(THRESHOLD_DELAY_MS));
	if (zone->irq_type == IF_PMIC)
		bcl_cb_clr_irq(bcl_dev);
}

static void google_irq_untriggered_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, irq_untriggered_work.work);
	struct bcl_device *bcl_dev;
	int idx;

	if (!zone || !zone->parent)
		return;

	idx = zone->idx;
	bcl_dev = zone->parent;

	/* IRQ falling edge */
	if (zone->irq_type == IF_PMIC)
		bcl_cb_clr_irq(bcl_dev);
	if (idx == BATOILO)
		gpio_set_value(bcl_dev->modem_gpio2_pin, 0);

	update_tz(zone, idx, false);
}

static int google_bcl_register_zone(struct bcl_device *bcl_dev, int idx, const char *devname,
				    u32 intr_flag, int pin, int lvl, int irq, int type)
{
	int ret = 0;
	struct bcl_zone *zone;
	bool to_conf = true;

	if (!bcl_dev)
		return -ENOMEM;

	zone = devm_kzalloc(bcl_dev->device, sizeof(struct bcl_zone), GFP_KERNEL);

	if (!zone)
		return -ENOMEM;

	zone->idx = idx;
	zone->bcl_pin = pin;
	zone->bcl_irq = irq;
	zone->bcl_cur_lvl = 0;
	zone->bcl_prev_lvl = 0;
	zone->bcl_lvl = lvl;
	zone->parent = bcl_dev;
	zone->irq_type = type;
	atomic_set(&zone->bcl_cnt, 0);
	if (idx == SMPL_WARN) {
		irq_set_status_flags(zone->bcl_irq, IRQ_DISABLE_UNLAZY);
		zone->polarity = 0;
	} else
		zone->polarity = 1;
	if ((bcl_dev->ifpmic == MAX77759) && (idx == BATOILO))
		to_conf = false;
	if (to_conf) {
		ret = devm_request_threaded_irq(bcl_dev->device, zone->bcl_irq, NULL,
						irq_handler, intr_flag | IRQF_ONESHOT,
						devname, zone);

		if (ret < 0) {
			dev_err(zone->device, "Failed to request IRQ: %d: %d\n", irq, ret);
			devm_kfree(bcl_dev->device, zone);
			return ret;
		}
		zone->irq_reg = true;
		zone->disabled = true;
		disable_irq(zone->bcl_irq);
	}
	INIT_DELAYED_WORK(&zone->irq_work, google_warn_work);
	INIT_DELAYED_WORK(&zone->irq_triggered_work, google_irq_triggered_work);
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

static int google_set_sub_pmic(struct bcl_device *bcl_dev)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct s2mpg15_platform_data *pdata_sub;
	struct s2mpg15_dev *sub_dev = NULL;
	int i, rail_i;
#elif IS_ENABLED(CONFIG_SOC_GS201)
	struct s2mpg13_platform_data *pdata_sub;
	struct s2mpg13_dev *sub_dev = NULL;
#elif IS_ENABLED(CONFIG_SOC_GS101)
	struct s2mpg11_platform_data *pdata_sub;
	struct s2mpg11_dev *sub_dev = NULL;
#endif
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	u8 val = 0;
	int ret;

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
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	bcl_dev->sub_odpm = pdata_sub->meter;
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		rail_i = bcl_dev->sub_odpm->channels[i].rail_i;
		bcl_dev->sub_rail_names[i] = bcl_dev->sub_odpm->chip.rails[rail_i].schematic_name;
	}
	bcl_dev->sub_meter_i2c = sub_dev->meter;
#endif
	bcl_dev->sub_irq_base = pdata_sub->irq_base;
	bcl_dev->sub_pmic_i2c = sub_dev->pmic;
	bcl_dev->sub_dev = sub_dev->dev;
	if (pmic_read(CORE_PMIC_SUB, bcl_dev, SUB_CHIPID, &val)) {
		dev_err(bcl_dev->device, "Failed to read PMIC chipid.\n");
		return -ENODEV;
	}
	pmic_read(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC1, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC1 : %#x\n", val);
	bcl_dev->sub_offsrc1 = val;
	pmic_write(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC1, 0);
#if IS_ENABLED(CONFIG_SOC_ZUMA) || IS_ENABLED(CONFIG_SOC_GS201)
	pmic_read(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC2, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC2 : %#x\n", val);
	bcl_dev->sub_offsrc2 = val;
	pmic_write(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC2, 0);
#endif

	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_GPU, "GPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
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
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      pdata_sub->b2_soft_ocp_warn_pin,
				      GPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				      (pdata_sub->b2_soft_ocp_warn_lvl * GPU_STEP),
				      gpio_to_irq(pdata_sub->b2_soft_ocp_warn_pin),
				      CORE_SUB_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_GPU\n");
		return -ENODEV;
	}

	return 0;
}

static int intf_pmic_init(struct bcl_device *bcl_dev)
{
	int ret;
	unsigned int uvlo1_lvl, uvlo2_lvl, batoilo_lvl, lvl;

	bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	batoilo_reg_read(bcl_dev->intf_pmic_i2c, bcl_dev->ifpmic, BATOILO1, &lvl);
	batoilo_lvl = BO_STEP * lvl + bcl_dev->batoilo_lower_limit;
	uvlo_reg_read(bcl_dev->intf_pmic_i2c, bcl_dev->ifpmic, UVLO1, &uvlo1_lvl);
	uvlo_reg_read(bcl_dev->intf_pmic_i2c, bcl_dev->ifpmic, UVLO2, &uvlo2_lvl);

	ret = google_bcl_register_zone(bcl_dev, UVLO1, "UVLO1",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      bcl_dev->vdroop1_pin,
				      VD_BATTERY_VOLTAGE - uvlo1_lvl - THERMAL_HYST_LEVEL,
				      gpio_to_irq(bcl_dev->vdroop1_pin),
				      IF_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: UVLO1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, UVLO2, "UVLO2",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      bcl_dev->vdroop2_pin,
				      VD_BATTERY_VOLTAGE - uvlo2_lvl - THERMAL_HYST_LEVEL,
				      gpio_to_irq(bcl_dev->vdroop2_pin),
				      IF_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: UVLO2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, BATOILO1, "BATOILO1",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      bcl_dev->vdroop2_pin,
				      batoilo_lvl - THERMAL_HYST_LEVEL,
				      gpio_to_irq(bcl_dev->vdroop2_pin),
				      IF_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: BATOILO\n");
		return -ENODEV;
	}
	return ret;
}

static int google_set_intf_pmic(struct bcl_device *bcl_dev)
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
		if (!strcmp(i2c->name, "max77779chrg"))
			bcl_dev->ifpmic = MAX77779;
		else
			bcl_dev->ifpmic = MAX77759;
		ret = of_property_read_u32(p_np, "batoilo_lower", &retval);
		bcl_dev->batoilo_lower_limit = ret ? BO_LOWER_LIMIT : retval;
		ret = of_property_read_u32(p_np, "batoilo_upper", &retval);
		bcl_dev->batoilo_upper_limit = ret ? BO_UPPER_LIMIT : retval;
		bcl_dev->intf_pmic_i2c = i2c;
		bcl_dev->irq_pmic_i2c = i2c;
	}
	of_node_put(p_np);
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

	intf_pmic_init(bcl_dev);

	bcl_dev->ready = true;

	if (!bcl_dev->ready)
		return -ENODEV;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (bcl_dev->ifpmic == MAX77779) {
			if (bcl_dev->zone[i] && (i != BATOILO1) && (i != UVLO1)) {
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
	int i, rail_i;
#elif IS_ENABLED(CONFIG_SOC_GS201)
	struct s2mpg12_platform_data *pdata_main;
	struct s2mpg12_dev *main_dev = NULL;
#elif IS_ENABLED(CONFIG_SOC_GS101)
	struct s2mpg10_platform_data *pdata_main;
	struct s2mpg10_dev *main_dev = NULL;
#endif
	u8 val;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	int ret;
	unsigned int ocp_cpu2_pin, ocp_cpu2_lvl;
	unsigned int ocp_cpu1_pin, ocp_cpu1_lvl;
	unsigned int ocp_tpu_pin, ocp_tpu_lvl;
	unsigned int soft_ocp_cpu2_pin, soft_ocp_cpu2_lvl;
	unsigned int soft_ocp_cpu1_pin, soft_ocp_cpu1_lvl;
	unsigned int soft_ocp_tpu_pin, soft_ocp_tpu_lvl;

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
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	ocp_cpu2_pin = pdata_main->b2_ocp_warn_pin;
	ocp_cpu2_lvl = pdata_main->b2_ocp_warn_lvl;
	ocp_cpu1_pin = pdata_main->b3_ocp_warn_pin;
	ocp_cpu1_lvl = pdata_main->b3_ocp_warn_lvl;
	ocp_tpu_pin = pdata_main->b7_ocp_warn_pin;
	ocp_tpu_lvl = pdata_main->b7_ocp_warn_lvl;
	soft_ocp_cpu2_pin = pdata_main->soft_b2_ocp_warn_pin;
	soft_ocp_cpu2_lvl = pdata_main->soft_b2_ocp_warn_lvl;
	soft_ocp_cpu1_pin = pdata_main->soft_b3_ocp_warn_pin;
	soft_ocp_cpu1_lvl = pdata_main->soft_b3_ocp_warn_lvl;
	soft_ocp_tpu_pin = pdata_main->soft_b7_ocp_warn_pin;
	soft_ocp_tpu_lvl = pdata_main->soft_b7_ocp_warn_lvl;
#elif IS_ENABLED(CONFIG_SOC_GS201) || IS_ENABLED(CONFIG_SOC_GS101)
	ocp_cpu2_pin = pdata_main->b2_ocp_warn_pin;
	ocp_cpu2_lvl = pdata_main->b2_ocp_warn_lvl;
	ocp_cpu1_pin = pdata_main->b3_ocp_warn_pin;
	ocp_cpu1_lvl = pdata_main->b3_ocp_warn_lvl;
	ocp_tpu_pin = pdata_main->b10_ocp_warn_pin;
	ocp_tpu_lvl = pdata_main->b10_ocp_warn_lvl;
	soft_ocp_cpu2_pin = pdata_main->b2_soft_ocp_warn_pin;
	soft_ocp_cpu2_lvl = pdata_main->b2_soft_ocp_warn_lvl;
	soft_ocp_cpu1_pin = pdata_main->b3_soft_ocp_warn_pin;
	soft_ocp_cpu1_lvl = pdata_main->b3_soft_ocp_warn_lvl;
	soft_ocp_tpu_pin = pdata_main->b10_soft_ocp_warn_pin;
	soft_ocp_tpu_lvl = pdata_main->b10_soft_ocp_warn_lvl;
#endif
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	bcl_dev->main_odpm = pdata_main->meter;
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		rail_i = bcl_dev->main_odpm->channels[i].rail_i;
		bcl_dev->main_rail_names[i] = bcl_dev->main_odpm->chip.rails[rail_i].schematic_name;
	}
	bcl_dev->main_meter_i2c = main_dev->meter;
#endif
	bcl_dev->main_irq_base = pdata_main->irq_base;
	bcl_dev->main_pmic_i2c = main_dev->pmic;
	bcl_dev->main_dev = main_dev->dev;
	/* clear MAIN information every boot */
	/* see b/215371539 */
	pmic_read(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC1, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC1 : %#x\n", val);
	bcl_dev->main_offsrc1 = val;
#if IS_ENABLED(CONFIG_SOC_ZUMA) || IS_ENABLED(CONFIG_SOC_GS201)
	pmic_read(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC2, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC2 : %#x\n", val);
	bcl_dev->main_offsrc2 = val;
#endif
	pmic_read(CORE_PMIC_MAIN, bcl_dev, MAIN_PWRONSRC, &val);
	dev_info(bcl_dev->device, "MAIN PWRONSRC: %#x\n", val);
	bcl_dev->pwronsrc = val;
	pmic_write(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC1, 0);
#if IS_ENABLED(CONFIG_SOC_ZUMA) || IS_ENABLED(CONFIG_SOC_GS201)
	pmic_write(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC2, 0);
#endif
	pmic_write(CORE_PMIC_MAIN, bcl_dev, MAIN_PWRONSRC, 0);

	ret = google_bcl_register_zone(bcl_dev, SMPL_WARN, "SMPL_WARN_IRQ",
				      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				      pdata_main->smpl_warn_pin,
				      SMPL_BATTERY_VOLTAGE -
				      (pdata_main->smpl_warn_lvl *
				       SMPL_STEP + SMPL_LOWER_LIMIT),
				      gpio_to_irq(pdata_main->smpl_warn_pin),
				      CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SMPL_WARN\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_CPUCL1, "CPU1_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      ocp_cpu1_pin,
				      CPU1_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				      (ocp_cpu1_lvl * CPU1_STEP),
				      gpio_to_irq(ocp_cpu1_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_CPUCL2, "CPU2_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      ocp_cpu2_pin,
				      CPU2_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				      (ocp_cpu2_lvl * CPU2_STEP),
				      gpio_to_irq(ocp_cpu2_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_CPUCL1, "SOFT_CPU1_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      soft_ocp_cpu1_pin,
				      CPU1_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				      (soft_ocp_cpu1_lvl * CPU1_STEP),
				      gpio_to_irq(soft_ocp_cpu1_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_CPUCL2, "SOFT_CPU2_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      soft_ocp_cpu2_pin,
				      CPU2_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				      (soft_ocp_cpu2_lvl * CPU2_STEP),
				      gpio_to_irq(soft_ocp_cpu2_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_TPU, "TPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      ocp_tpu_pin,
				      TPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				      (ocp_tpu_lvl * TPU_STEP),
				      gpio_to_irq(ocp_tpu_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: TPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_TPU, "SOFT_TPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      soft_ocp_tpu_pin,
				      TPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				      (soft_ocp_tpu_lvl * TPU_STEP),
				      gpio_to_irq(soft_ocp_tpu_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_TPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, PMIC_120C, "PMIC_120C",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      0,
				      PMIC_120C_UPPER_LIMIT - THERMAL_HYST_LEVEL,
				      pdata_main->irq_base + INT3_120C,
				      CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_120C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, PMIC_140C, "PMIC_140C",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      0,
				      PMIC_140C_UPPER_LIMIT - THERMAL_HYST_LEVEL,
				      pdata_main->irq_base + INT3_140C,
				      CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_140C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_zone(bcl_dev, PMIC_OVERHEAT, "PMIC_OVERHEAT",
				      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				      0,
				      PMIC_OVERHEAT_UPPER_LIMIT - THERMAL_HYST_LEVEL,
				      pdata_main->irq_base + INT3_TSD,
				      CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_OVERHEAT\n");
		return -ENODEV;
	}


	return 0;

}

extern const struct attribute_group *mitigation_groups[];

static int google_init_fs(struct bcl_device *bcl_dev)
{
	bcl_dev->mitigation_dev = pmic_subdevice_create(NULL, mitigation_groups,
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
	reg = __raw_readl(gpio_alive + GPA5_CON);
	reg |= 0xFF0000;
	__raw_writel(0xFFFFF22, gpio_alive + GPA5_CON);
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
	mutex_init(&bcl_dev->ratio_lock);
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

static void google_bcl_parse_dtree(struct bcl_device *bcl_dev)
{
	int ret;
	u32 val;
	struct device_node *np = bcl_dev->device->of_node;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct device_node *child;
	struct device_node *p_np;
	int read, i = 0;
#endif

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

	if (bcl_disable_power(SUBSYSTEM_CPU2)) {
		if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU2,
					    bcl_dev->core_conf[SUBSYSTEM_CPU2].clkdivstep) != 0)
			dev_err(bcl_dev->device, "CPU2 Address is NULL\n");
		bcl_enable_power(SUBSYSTEM_CPU2);
	}
	if (bcl_disable_power(SUBSYSTEM_CPU1)) {
		if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU1,
					    bcl_dev->core_conf[SUBSYSTEM_CPU1].clkdivstep) != 0)
			dev_err(bcl_dev->device, "CPU1 Address is NULL\n");
		bcl_enable_power(SUBSYSTEM_CPU1);
	}
	if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU0,
	                            bcl_dev->core_conf[SUBSYSTEM_CPU0].clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU0 Address is NULL\n");
}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
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
	return 0;
}
#endif

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

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	bcl_dev->pmic_irq = platform_get_irq(pdev, 0);
#endif
	bcl_dev->ready = false;
	ret = google_bcl_init_instruction(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	ret = google_set_main_pmic(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;
	ret = google_set_sub_pmic(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	google_bcl_parse_dtree(bcl_dev);

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	google_bcl_configure_modem(bcl_dev);
#endif
	ret = google_init_fs(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;
	if (google_set_intf_pmic(bcl_dev) < 0)
		goto bcl_soc_probe_exit;
	bcl_dev->enabled = true;
	google_init_debugfs(bcl_dev);
	return 0;

bcl_soc_probe_exit:
	google_bcl_remove_thermal(bcl_dev);
	return ret;
}

static int google_bcl_remove(struct platform_device *pdev)
{
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	pmic_device_destroy(bcl_dev->mitigation_dev->devt);
	debugfs_remove_recursive(bcl_dev->debug_entry);
	google_bcl_remove_thermal(bcl_dev);

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
