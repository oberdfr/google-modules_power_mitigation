// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_core.c Google bcl driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_SOC_GS101)
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#include <linux/mfd/samsung/s2mpg1415.h>
#include <linux/mfd/samsung/s2mpg1415-register.h>
#endif

#include <soc/google/cal-if.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include "bcl.h"

const unsigned int subsystem_pmu[] = {
	PMU_ALIVE_CPU0_STATES,
	PMU_ALIVE_CPU1_STATES,
	PMU_ALIVE_CPU2_STATES,
	PMU_ALIVE_TPU_STATES,
	PMU_ALIVE_GPU_STATES,
	PMU_ALIVE_AUR_STATES
};

#if IS_ENABLED(CONFIG_SOC_GS101)
#define PMIC_MAIN_WRITE_REG(i2c, reg, val) s2mpg10_write_reg(i2c, reg, val)
#define PMIC_SUB_WRITE_REG(i2c, reg, val) s2mpg11_write_reg(i2c, reg, val)
#define PMIC_MAIN_READ_REG(i2c, reg, val) s2mpg10_read_reg(i2c, reg, val)
#define PMIC_SUB_READ_REG(i2c, reg, val) s2mpg11_read_reg(i2c, reg, val)
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#define PMIC_MAIN_WRITE_REG(i2c, reg, val) s2mpg14_write_reg(i2c, reg, val)
#define PMIC_SUB_WRITE_REG(i2c, reg, val) s2mpg15_write_reg(i2c, reg, val)
#define PMIC_MAIN_READ_REG(i2c, reg, val) s2mpg14_read_reg(i2c, reg, val)
#define PMIC_SUB_READ_REG(i2c, reg, val) s2mpg15_read_reg(i2c, reg, val)
#endif

int meter_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_WRITE_REG((bcl_dev)->sub_meter_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_WRITE_REG((bcl_dev)->main_meter_i2c, reg, value);
	}
	return 0;
}

int cpu_sfr_write(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int value)
{
	mutex_lock(&bcl_dev->cpu_ratio_lock);
	if (!bcl_disable_power(bcl_dev, idx)) {
		mutex_unlock(&bcl_dev->cpu_ratio_lock);
		return -EIO;
	}
	__raw_writel(value, addr);
	bcl_enable_power(bcl_dev, idx);
	mutex_unlock(&bcl_dev->cpu_ratio_lock);
	return 0;
}

int cpu_sfr_read(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int *reg)
{
	mutex_lock(&bcl_dev->cpu_ratio_lock);
	if (!bcl_disable_power(bcl_dev, idx)) {
		mutex_unlock(&bcl_dev->cpu_ratio_lock);
		return -EIO;
	}
	*reg = __raw_readl(addr);
	bcl_enable_power(bcl_dev, idx);
	mutex_unlock(&bcl_dev->cpu_ratio_lock);

	return 0;
}

int meter_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_READ_REG((bcl_dev)->sub_meter_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_READ_REG((bcl_dev)->main_meter_i2c, reg, value);
	}
	return 0;
}

int pmic_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_WRITE_REG((bcl_dev)->sub_pmic_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_WRITE_REG((bcl_dev)->main_pmic_i2c, reg, value);
	}
	return 0;
}

int pmic_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_READ_REG((bcl_dev)->sub_pmic_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_READ_REG((bcl_dev)->main_pmic_i2c, reg, value);
	}
	return 0;
}

bool bcl_is_cluster_on(struct bcl_device *bcl_dev, int cluster)
{
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	unsigned int addr, value = 0;

	if (cluster < bcl_dev->cpu2_cluster) {
		addr = CLUSTER1_NONCPU_STATES;
		exynos_pmu_read(addr, &value);
		return value & BIT(4);
	}
	if (cluster == bcl_dev->cpu2_cluster) {
		addr = CLUSTER2_NONCPU_STATES;
		exynos_pmu_read(addr, &value);
		return value & BIT(4);
	}
	return false;
#endif
	return true;
}

bool bcl_is_subsystem_on(struct bcl_device *bcl_dev, unsigned int addr)
{
	unsigned int value;

	switch (addr) {
	case PMU_ALIVE_TPU_STATES:
	case PMU_ALIVE_GPU_STATES:
	case PMU_ALIVE_AUR_STATES:
		exynos_pmu_read(addr, &value);
		return !(value & BIT(7));
	case PMU_ALIVE_CPU0_STATES:
		return true;
	case PMU_ALIVE_CPU1_STATES:
		return bcl_is_cluster_on(bcl_dev, bcl_dev->cpu1_cluster);
	case PMU_ALIVE_CPU2_STATES:
		return bcl_is_cluster_on(bcl_dev, bcl_dev->cpu2_cluster);
	}
	return false;
}

bool bcl_disable_power(struct bcl_device *bcl_dev, int cluster)
{
	if (IS_ENABLED(CONFIG_SOC_ZUMA) || IS_ENABLED(CONFIG_SOC_GS201)) {
		int i;
		if (cluster == SUBSYSTEM_CPU1) {
			if (!bcl_is_cluster_on(bcl_dev, bcl_dev->cpu1_cluster))
				return false;
			for (i = bcl_dev->cpu1_cluster; i < bcl_dev->cpu2_cluster; i++) {
				disable_power_mode(i, POWERMODE_TYPE_CLUSTER);
			}
		}
		else if (cluster == SUBSYSTEM_CPU2) {
			if (!bcl_is_cluster_on(bcl_dev, bcl_dev->cpu2_cluster))
				return false;
			disable_power_mode(bcl_dev->cpu2_cluster, POWERMODE_TYPE_CLUSTER);
		}
	}
	return true;
}

bool bcl_enable_power(struct bcl_device *bcl_dev, int cluster)
{
	if (IS_ENABLED(CONFIG_SOC_ZUMA) || IS_ENABLED(CONFIG_SOC_GS201)) {
		int i;
		if (cluster == SUBSYSTEM_CPU1) {
			if (!bcl_is_cluster_on(bcl_dev, bcl_dev->cpu1_cluster))
				return false;
			for (i = bcl_dev->cpu1_cluster; i < bcl_dev->cpu2_cluster; i++) {
				enable_power_mode(i, POWERMODE_TYPE_CLUSTER);
			}
		}
		else if (cluster == SUBSYSTEM_CPU2) {
			if (!bcl_is_cluster_on(bcl_dev, bcl_dev->cpu2_cluster))
				return false;
			enable_power_mode(bcl_dev->cpu2_cluster, POWERMODE_TYPE_CLUSTER);
		}
	}
	return true;
}
