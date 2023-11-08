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

static void data_logging_main_odpm_lpf_task(struct kthread_work *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device, main_meter_work);
	struct odpm_info *info = bcl_dev->main_odpm;
	int i = 0;
	int j = 0;

	/* the acquisition time of lpf_data is around 1ms */
	for (i = 0; i < DATA_LOGGING_NUM && bcl_dev->main_thread_running; i++) {
		s2mpg1415_meter_read_lpf_data_reg(info->chip.hw_id, info->i2c,
						  (u32 *)bcl_dev->br_stats->main_odpm_lpf[j].value);
		ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->main_odpm_lpf[j].time);
		bcl_dev->br_stats->triggered_state[j] =
				bcl_dev->zone[bcl_dev->br_stats->triggered_idx]->current_state;
		/* b/299700579 decides which module should be throttled */
		if (++j == DATA_LOGGING_LEN)
			j = 0;
	}
}

static void data_logging_sub_odpm_lpf_task(struct kthread_work *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device, sub_meter_work);
	struct odpm_info *info = bcl_dev->sub_odpm;
	int i = 0;
	int j = 0;

	/* the acquisition time of lpf_data is around 1ms */
	for (i = 0; i < DATA_LOGGING_NUM && bcl_dev->sub_thread_running; i++) {
		s2mpg1415_meter_read_lpf_data_reg(info->chip.hw_id, info->i2c,
						  (u32 *)bcl_dev->br_stats->sub_odpm_lpf[j].value);
		ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->sub_odpm_lpf[j].time);
		if (++j == DATA_LOGGING_LEN)
			j = 0;
	}
}

static int google_bcl_create_thread(struct bcl_device *bcl_dev)
{
	int ret;
	struct task_struct *main_task, *sub_task;

	kthread_init_worker(&bcl_dev->main_meter_worker);
	main_task = kthread_create(kthread_worker_fn, &bcl_dev->main_meter_worker,
				   "bcl_meter_main");

	if (IS_ERR_OR_NULL(main_task)) {
		ret = PTR_ERR(main_task);
		dev_err(bcl_dev->device, "failed to create logging thread: %d\n", ret);
		return ret;
	}
	sched_set_normal(main_task, -10);
	kthread_init_work(&bcl_dev->main_meter_work, data_logging_main_odpm_lpf_task);
	kthread_init_worker(&bcl_dev->sub_meter_worker);
	sub_task = kthread_create(kthread_worker_fn, &bcl_dev->sub_meter_worker,
				  "bcl_meter_sub");

	if (IS_ERR_OR_NULL(sub_task)) {
		ret = PTR_ERR(sub_task);
		dev_err(bcl_dev->device, "failed to create logging thread: %d\n", ret);
		kthread_stop(main_task);
		return ret;
	}
	sched_set_normal(sub_task, -10);
	bcl_dev->main_task = main_task;
	bcl_dev->sub_task = sub_task;
	kthread_init_work(&bcl_dev->sub_meter_work, data_logging_sub_odpm_lpf_task);
	wake_up_process(bcl_dev->main_task);
	wake_up_process(bcl_dev->sub_task);

	return 0;
}

static void google_bcl_wakeup_logging_threads(struct bcl_device *bcl_dev)
{
	bcl_dev->main_thread_running = true;
	bcl_dev->sub_thread_running = true;
	kthread_queue_work(&bcl_dev->main_meter_worker, &bcl_dev->main_meter_work);
	kthread_queue_work(&bcl_dev->sub_meter_worker, &bcl_dev->sub_meter_work);
}

static void google_bcl_pause_logging_threads(struct bcl_device *bcl_dev)
{
	bcl_dev->main_thread_running = false;
	bcl_dev->sub_thread_running = false;
	kthread_flush_work(&bcl_dev->main_meter_work);
	kthread_flush_work(&bcl_dev->sub_meter_work);
}

static void google_bcl_stop_logging_threads(struct bcl_device *bcl_dev)
{
	google_bcl_pause_logging_threads(bcl_dev);
	kthread_destroy_worker(&bcl_dev->main_meter_worker);
	kthread_destroy_worker(&bcl_dev->sub_meter_worker);
}

static void google_bcl_write_irq_triggered_event(struct bcl_device *bcl_dev, int idx)
{
	ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->triggered_time);
	bcl_dev->br_stats->triggered_idx = idx;
}

static void data_logging_complete_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  data_logging_complete_work.work);

	google_bcl_pause_logging_threads(bcl_dev);

	mutex_lock(&bcl_dev->data_logging_lock);
	bcl_dev->is_data_logging_running = false;
	mutex_unlock(&bcl_dev->data_logging_lock);
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

	if (state == LIGHT)
		google_bcl_start_data_logging(bcl_dev, idx);
}

void google_bcl_start_data_logging(struct bcl_device *bcl_dev, int idx)
{
	if (!bcl_dev->data_logging_enabled)
		return;

	mutex_trylock(&bcl_dev->data_logging_lock);
	if (!bcl_dev->is_data_logging_running) {
		bcl_dev->is_data_logging_running = true;
		google_bcl_init_brownout_stats(bcl_dev);

		google_bcl_write_irq_triggered_event(bcl_dev, idx);

		bcl_dev->triggered_idx = idx;
		sysfs_notify(&bcl_dev->mitigation_dev->kobj, "br_stats", "triggered_idx");

		google_bcl_wakeup_logging_threads(bcl_dev);

		schedule_delayed_work(&bcl_dev->data_logging_complete_work,
		                      msecs_to_jiffies(DATA_LOGGING_TIME_MS));
	}
	mutex_unlock(&bcl_dev->data_logging_lock);
}

void google_bcl_remove_data_logging(struct bcl_device *bcl_dev)
{
	google_bcl_stop_logging_threads(bcl_dev);
	bcl_dev->data_logging_enabled = false;
	mutex_lock(&bcl_dev->data_logging_lock);
	bcl_dev->is_data_logging_running = false;
	mutex_unlock(&bcl_dev->data_logging_lock);
	mutex_destroy(&bcl_dev->data_logging_lock);
	cancel_delayed_work(&bcl_dev->data_logging_complete_work);
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
	if (google_bcl_create_thread(bcl_dev) != 0) {
		kfree(bcl_dev->br_stats);
		return -EINVAL;
	}
	bcl_dev->is_data_logging_running = false;
	bcl_dev->data_logging_enabled = true;
	INIT_DELAYED_WORK(&bcl_dev->data_logging_complete_work, data_logging_complete_work);

	return 0;
}
