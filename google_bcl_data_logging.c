// SPDX-License-Identifier: GPL-2.0-only
/*
 * google_bcl_data_logging.c Google bcl Data Logging driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/threads.h>
#include <linux/time.h>
#include <soc/google/odpm.h>
#include <uapi/linux/sched/types.h>
#include "bcl.h"

void compute_mitigation_modules(struct bcl_device *bcl_dev,
						struct bcl_mitigation_conf *mitigation_conf,
						u32 *odpm_lpf_value) {
	int i;
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		if (odpm_lpf_value[i] >= mitigation_conf[i].threshold) {
			atomic_or(BIT(mitigation_conf[i].module_id),
					  &bcl_dev->mitigation_module_ids);
		}
	}
}

static void data_logging_main_odpm_lpf_task(struct bcl_device *bcl_dev)
{
	struct odpm_info *info = bcl_dev->main_odpm;
	/* select lpf power mode */
	s2mpg1415_meter_set_lpf_mode(info->chip.hw_id, info->i2c, S2MPG1415_METER_POWER);
	/* the acquisition time of lpf_data is around 1ms */
	s2mpg1415_meter_read_lpf_data_reg(info->chip.hw_id, info->i2c,
					  (u32 *)bcl_dev->br_stats->main_odpm_lpf.value);
	ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->main_odpm_lpf.time);
	compute_mitigation_modules(bcl_dev,
				   bcl_dev->main_mitigation_conf,
				   bcl_dev->br_stats->main_odpm_lpf.value);
}

static void data_logging_sub_odpm_lpf_task(struct bcl_device *bcl_dev)
{
	struct odpm_info *info = bcl_dev->sub_odpm;
	/* select lpf power mode */
	s2mpg1415_meter_set_lpf_mode(info->chip.hw_id, info->i2c, S2MPG1415_METER_POWER);
	/* the acquisition time of lpf_data is around 1ms */
	s2mpg1415_meter_read_lpf_data_reg(info->chip.hw_id, info->i2c,
					  (u32 *)bcl_dev->br_stats->sub_odpm_lpf.value);
	ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->sub_odpm_lpf.time);
	compute_mitigation_modules(bcl_dev,
				   bcl_dev->sub_mitigation_conf,
				   bcl_dev->br_stats->sub_odpm_lpf.value);
}

static void google_bcl_write_irq_triggered_event(struct bcl_device *bcl_dev, int idx)
{
	ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->triggered_time);
	bcl_dev->br_stats->triggered_idx = idx;
}

static void google_bcl_init_brownout_stats(struct bcl_device *bcl_dev)
{
	memset((void *)bcl_dev->br_stats, 0, bcl_dev->br_stats_size);
	bcl_dev->br_stats->triggered_idx = TRIGGERED_SOURCE_MAX;
}

void google_bcl_upstream_state(struct bcl_zone *zone, enum MITIGATION_MODE state)
{
	struct bcl_device *bcl_dev = zone->parent;
	int idx = zone->idx;

	if (!bcl_dev->enabled_br_stats)
		return;

	atomic_inc(&zone->last_triggered.triggered_cnt[state]);
	zone->last_triggered.triggered_time[state] = ktime_to_ms(ktime_get());
	zone->current_state = state;
	if (idx == UVLO1)
		sysfs_notify(&bcl_dev->mitigation_dev->kobj, "triggered_state", "uvlo1_triggered");
	else if (idx == UVLO2)
		sysfs_notify(&bcl_dev->mitigation_dev->kobj, "triggered_state", "uvlo2_triggered");
	else if (idx == BATOILO1)
		sysfs_notify(&bcl_dev->mitigation_dev->kobj, "triggered_state", "oilo1_triggered");
	else if (idx == BATOILO2)
		sysfs_notify(&bcl_dev->mitigation_dev->kobj, "triggered_state", "oilo2_triggered");
	else if (idx == SMPL_WARN)
		sysfs_notify(&bcl_dev->mitigation_dev->kobj, "triggered_state", "smpl_triggered");
	else
		return;
}

void google_bcl_start_data_logging(struct bcl_device *bcl_dev, int idx)
{
	if (!bcl_dev->enabled_br_stats)
		return;

	if (!bcl_dev->data_logging_initialized)
		return;

	mutex_trylock(&bcl_dev->data_logging_lock);
	if (bcl_dev->is_data_logging_running) {
		mutex_unlock(&bcl_dev->data_logging_lock);
		return;
	}
	bcl_dev->is_data_logging_running = true;
	google_bcl_init_brownout_stats(bcl_dev);

	google_bcl_write_irq_triggered_event(bcl_dev, idx);
	if (IS_ENABLED(CONFIG_REGULATOR_S2MPG14)) {
		bcl_dev->br_stats->triggered_state =
				bcl_dev->zone[bcl_dev->br_stats->triggered_idx]->current_state;
		data_logging_main_odpm_lpf_task(bcl_dev);
		data_logging_sub_odpm_lpf_task(bcl_dev);
	}

	bcl_dev->triggered_idx = idx;
	sysfs_notify(&bcl_dev->mitigation_dev->kobj, "br_stats", "triggered_idx");
	mutex_unlock(&bcl_dev->data_logging_lock);
}

void google_bcl_remove_data_logging(struct bcl_device *bcl_dev)
{
	bcl_dev->data_logging_initialized = false;
	mutex_lock(&bcl_dev->data_logging_lock);
	bcl_dev->is_data_logging_running = false;
	mutex_unlock(&bcl_dev->data_logging_lock);
	mutex_destroy(&bcl_dev->data_logging_lock);
	kfree(bcl_dev->br_stats);
}

int google_bcl_init_data_logging(struct bcl_device *bcl_dev)
{
	bcl_dev->triggered_idx = TRIGGERED_SOURCE_MAX;
	mutex_init(&bcl_dev->data_logging_lock);
	bcl_dev->br_stats_size = sizeof(struct brownout_stats);
	bcl_dev->br_stats = kmalloc(bcl_dev->br_stats_size, GFP_KERNEL);
	if (!bcl_dev->br_stats)
		return -ENOMEM;
	google_bcl_init_brownout_stats(bcl_dev);
	bcl_dev->is_data_logging_running = false;
	bcl_dev->data_logging_initialized = true;

	return 0;
}
