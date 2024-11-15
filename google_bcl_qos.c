// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_qos.c Google bcl PMQOS driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
#include <linux/mfd/samsung/s2mpg1415.h>
#include <linux/mfd/samsung/s2mpg1415-register.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
#include <linux/mfd/samsung/s2mpg1x.h>
#include <linux/mfd/samsung/s2mpg1x-register.h>
#endif
#include "bcl.h"

#define CREATE_TRACE_POINTS
#include <trace/events/bcl_exynos.h>

static void trace_qos(bool throttle, const char *devname)
{
	char buf[64];
	if (!trace_clock_set_rate_enabled())
		return;
	snprintf(buf, sizeof(buf), "BCL_ZONE_%s_QOS", devname);
	trace_clock_set_rate(buf, throttle ? 1 : 0, raw_smp_processor_id());
}

void google_bcl_qos_update(struct bcl_zone *zone, bool throttle)
{
	struct bcl_device *bcl_dev;
	if (!zone->bcl_qos)
		return;
	bcl_dev = zone->parent;

	mutex_lock(&bcl_dev->qos_update_lock);
	if (bcl_dev->throttle && throttle) {
		mutex_unlock(&bcl_dev->qos_update_lock);
		return;
	}
	if (throttle)
		bcl_dev->throttle = true;

	if (bcl_dev->cpu0_cluster_on)
		freq_qos_update_request(&zone->bcl_qos->cpu0_max_qos_req,
					throttle ? zone->bcl_qos->cpu0_limit : INT_MAX);
	if (bcl_dev->cpu1_cluster_on)
		freq_qos_update_request(&zone->bcl_qos->cpu1_max_qos_req,
					throttle ? zone->bcl_qos->cpu1_limit : INT_MAX);
	if (bcl_dev->cpu2_cluster_on)
		freq_qos_update_request(&zone->bcl_qos->cpu2_max_qos_req,
					throttle ? zone->bcl_qos->cpu2_limit : INT_MAX);

	exynos_pm_qos_update_request_async(&zone->bcl_qos->tpu_qos_max,
					   throttle ? zone->bcl_qos->tpu_limit : INT_MAX);
	exynos_pm_qos_update_request_async(&zone->bcl_qos->gpu_qos_max,
					   throttle ? zone->bcl_qos->gpu_limit : INT_MAX);

	if (!throttle)
		bcl_dev->throttle = false;
	mutex_unlock(&bcl_dev->qos_update_lock);

	trace_bcl_irq_trigger(zone->idx, throttle, throttle ? zone->bcl_qos->cpu0_limit : INT_MAX,
	                      throttle ? zone->bcl_qos->cpu1_limit : INT_MAX,
	                      throttle ? zone->bcl_qos->cpu2_limit : INT_MAX,
	                      throttle ? zone->bcl_qos->tpu_limit : INT_MAX,
	                      throttle ? zone->bcl_qos->gpu_limit : INT_MAX,
	                      zone->bcl_stats.voltage, zone->bcl_stats.capacity);
	trace_qos(throttle, zone->devname);
}

static int init_freq_qos(struct bcl_device *bcl_dev, struct qos_throttle_limit *throttle)
{
	struct cpufreq_policy *policy = NULL;
	int ret;

	policy = cpufreq_cpu_get(bcl_dev->cpu0_cluster);
	if (!policy)
		return -EINVAL;

	bcl_dev->cpu0_cluster_on = true;
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu0_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		return ret;

	policy = cpufreq_cpu_get(bcl_dev->cpu1_cluster);
	if (!policy)
		return ret;

	bcl_dev->cpu1_cluster_on = true;
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu1_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		goto fail1;

	policy = cpufreq_cpu_get(bcl_dev->cpu2_cluster);
	if (!policy)
		return ret;

	bcl_dev->cpu2_cluster_on = true;
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu2_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		goto fail2;

	return 0;
fail2:
	freq_qos_remove_request(&throttle->cpu1_max_qos_req);
fail1:
	freq_qos_remove_request(&throttle->cpu0_max_qos_req);
	return ret;
}

int google_bcl_setup_qos(struct bcl_device *bcl_dev)
{
	int ret = 0;
	int i;
	struct bcl_zone *zone;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		zone = bcl_dev->zone[i];
		if ((!zone) || (!zone->bcl_qos))
			continue;

		ret = init_freq_qos(bcl_dev, zone->bcl_qos);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Cannot init pm qos on %d for cpu\n",
				zone->idx);
			goto fail;
		}
		exynos_pm_qos_add_request(&zone->bcl_qos->tpu_qos_max, PM_QOS_TPU_FREQ_MAX,
				  	  INT_MAX);
		exynos_pm_qos_add_request(&zone->bcl_qos->gpu_qos_max, PM_QOS_GPU_FREQ_MAX,
				  	  INT_MAX);
		zone->conf_qos = true;
	}
	mutex_init(&bcl_dev->qos_update_lock);
	return 0;
fail:
	google_bcl_remove_qos(bcl_dev);
	return ret;
}

void google_bcl_remove_qos(struct bcl_device *bcl_dev)
{
	int i;
	struct bcl_zone *zone;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		zone = bcl_dev->zone[i];
		if ((!zone) || (!zone->bcl_qos))
			continue;
		if (zone->conf_qos) {
			if (bcl_dev->cpu0_cluster_on)
				freq_qos_remove_request(&zone->bcl_qos->cpu0_max_qos_req);
			if (bcl_dev->cpu1_cluster_on)
				freq_qos_remove_request(&zone->bcl_qos->cpu1_max_qos_req);
			if (bcl_dev->cpu2_cluster_on)
				freq_qos_remove_request(&zone->bcl_qos->cpu2_max_qos_req);
			exynos_pm_qos_remove_request(&zone->bcl_qos->tpu_qos_max);
			exynos_pm_qos_remove_request(&zone->bcl_qos->gpu_qos_max);
			zone->bcl_qos = NULL;
			zone->conf_qos = false;
		}
	}
	mutex_destroy(&bcl_dev->qos_update_lock);
}
