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

static int data_logging_main_odpm_lpf_task(void *context)
{
	struct bcl_device *bcl_dev = context;
	struct odpm_info *info = bcl_dev->main_odpm;
	int i = 0;

	for (;;) {
		if (kthread_should_stop())
			return 0;
		if (kthread_should_park())
			kthread_parkme();
		s2mpg1415_meter_read_lpf_data_reg(info->chip.hw_id, info->i2c,
						  (u32 *)bcl_dev->br_stats->main_odpm_lpf[i].value);
		ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->main_odpm_lpf[i].time);
		if (++i == DATA_LOGGING_LEN)
			i = 0;
	}

	return 0;
}

static int data_logging_sub_odpm_lpf_task(void *context)
{
	struct bcl_device *bcl_dev = context;
	struct odpm_info *info = bcl_dev->sub_odpm;
	int i = 0;

	for (;;) {
		if (kthread_should_stop())
			return 0;
		if (kthread_should_park())
			kthread_parkme();
		s2mpg1415_meter_read_lpf_data_reg(info->chip.hw_id, info->i2c,
						  (u32 *)bcl_dev->br_stats->sub_odpm_lpf[i].value);
		ktime_get_real_ts64((struct timespec64 *)&bcl_dev->br_stats->sub_odpm_lpf[i].time);
		if (++i == DATA_LOGGING_LEN)
			i = 0;
	}

	return 0;
}

static int google_bcl_create_thread(struct bcl_device *bcl_dev)
{
	int ret;
	struct task_struct *main_task, *sub_task;
	const struct sched_param param = {.sched_priority = (MAX_RT_PRIO / 2)};

	main_task = kthread_create(data_logging_main_odpm_lpf_task, bcl_dev,
				   "bcl_data_logging_thread_main");

	if (IS_ERR_OR_NULL(main_task)) {
		ret = PTR_ERR(main_task);
		dev_err(bcl_dev->device, "failed to create logging thread: %ld\n", ret);
		return ret;
	}
	ret = sched_setscheduler_nocheck(main_task, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(main_task);
		main_task = NULL;
		return ret;
	}
	sub_task = kthread_create(data_logging_sub_odpm_lpf_task, bcl_dev,
				  "bcl_data_logging_thread_sub");

	if (IS_ERR_OR_NULL(sub_task)) {
		ret = PTR_ERR(sub_task);
		dev_err(bcl_dev->device, "failed to create logging thread: %ld\n", ret);
		kthread_stop(main_task);
		return ret;
	}
	ret = sched_setscheduler_nocheck(sub_task, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(main_task);
		kthread_stop(sub_task);
		sub_task = NULL;
		return ret;
	}
	bcl_dev->main_task = main_task;
	bcl_dev->sub_task = sub_task;
	wake_up_process(bcl_dev->main_task);
	wake_up_process(bcl_dev->sub_task);

	return ret;
}

static void google_bcl_wakeup_logging_threads(struct bcl_device *bcl_dev)
{
	if (!IS_ERR_OR_NULL(bcl_dev->main_task))
		kthread_unpark(bcl_dev->main_task);
	if (!IS_ERR_OR_NULL(bcl_dev->sub_task))
		kthread_unpark(bcl_dev->sub_task);
}

static void google_bcl_stop_logging_threads(struct bcl_device *bcl_dev)
{
	if (!IS_ERR_OR_NULL(bcl_dev->main_task))
		kthread_stop(bcl_dev->main_task);
	if (!IS_ERR_OR_NULL(bcl_dev->sub_task))
		kthread_stop(bcl_dev->sub_task);
}

static void google_bcl_pause_logging_threads(struct bcl_device *bcl_dev)
{
	if (!IS_ERR_OR_NULL(bcl_dev->main_task))
		kthread_park(bcl_dev->main_task);
	if (!IS_ERR_OR_NULL(bcl_dev->sub_task))
		kthread_park(bcl_dev->sub_task);
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

void google_bcl_start_data_logging(struct bcl_device *bcl_dev, int idx)
{
	if (!bcl_dev->data_logging_enabled)
		return;

	mutex_lock(&bcl_dev->data_logging_lock);
	if (!bcl_dev->is_data_logging_running) {
		bcl_dev->is_data_logging_running = true;
		google_bcl_init_brownout_stats(bcl_dev);

		google_bcl_write_irq_triggered_event(bcl_dev, idx);

		bcl_dev->triggered_idx = idx;
		sysfs_notify(&bcl_dev->mitigation_dev->kobj, "br_stats", "triggered_idx");

		google_bcl_wakeup_logging_threads(bcl_dev);

		schedule_delayed_work(&bcl_dev->data_logging_complete_work,
		                      msecs_to_jiffies(DATA_LOGGING_TIME));
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
