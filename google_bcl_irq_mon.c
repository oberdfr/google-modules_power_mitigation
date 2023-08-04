// SPDX-License-Identifier: GPL-2.0-only
/*
 * google_bcl_irq_mon.c Google bcl IRQ monitor driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include "bcl.h"

static enum BCL_BATT_IRQ id_to_ind(int id)
{
	switch (id) {
	case UVLO1:
		return UVLO1_IRQ_BIN;
	case UVLO2:
		return UVLO2_IRQ_BIN;
	case BATOILO1:
	case BATOILO2:
		return BATOILO_IRQ_BIN;
	}
	return MAX_BCL_BATT_IRQ;
}

static void bin_incr_ifpmic(struct bcl_device *bcl_dev, enum BCL_BATT_IRQ batt,
				enum CONCURRENT_PWRWARN_IRQ pwrwarn, ktime_t end_time)
{
	ktime_t time_delta;
	if (bcl_dev->ifpmic_irq_bins[batt][pwrwarn].start_time == 0)
		return;

	time_delta = ktime_sub(end_time, bcl_dev->ifpmic_irq_bins[batt][pwrwarn].start_time);
	if (ktime_compare(time_delta, DELTA_10MS) < 0)
		atomic_inc(&bcl_dev->ifpmic_irq_bins[batt][pwrwarn].lt_5ms_count);
	else if (ktime_compare(time_delta, DELTA_50MS) < 0)
		atomic_inc(&bcl_dev->ifpmic_irq_bins[batt][pwrwarn].bt_5ms_10ms_count);
	else
		atomic_inc(&bcl_dev->ifpmic_irq_bins[batt][pwrwarn].gt_10ms_count);

	bcl_dev->ifpmic_irq_bins[batt][pwrwarn].start_time = 0;
}

void update_irq_end_times(struct bcl_device *bcl_dev, int id)
{
	ktime_t end_time;
	int irq_ind = -1;
	int i;
	bool pwrwarn_irq_triggered;

	irq_ind = id_to_ind(id);
	if (irq_ind == MAX_BCL_BATT_IRQ)
		return;

	end_time = ktime_get();
	for (i = 0; i < MAX_CONCURRENT_PWRWARN_IRQ; i++) {
		switch (i) {
		case NONE_BCL_BIN:
			pwrwarn_irq_triggered = true;
			break;
		case MMWAVE_BCL_BIN:
			pwrwarn_irq_triggered =
				bcl_dev->sub_pwr_warn_triggered[bcl_dev->rffe_channel];
			break;
		case RFFE_BCL_BIN:
			pwrwarn_irq_triggered =
				bcl_dev->main_pwr_warn_triggered[bcl_dev->rffe_channel];
			break;
		}
		if (pwrwarn_irq_triggered)
			bin_incr_ifpmic(bcl_dev, irq_ind, i, end_time);
	}
}

/*
 * Track UVLO1/UVLO2/BATOILO IRQ starting times, and any PWRWARN events
 * happening at the same time as the UVLO1/UVLO2/BATOILO IRQ.
 */
void update_irq_start_times(struct bcl_device *bcl_dev, int id)
{
	ktime_t start_time;

	/* Check if it is a input IRQ */
	enum BCL_BATT_IRQ irq_ind = id_to_ind(id);

	if (irq_ind == MAX_BCL_BATT_IRQ)
		return;

	if (bcl_dev->ifpmic_irq_bins[irq_ind][NONE_BCL_BIN].start_time != 0)
		update_irq_end_times(bcl_dev, id);

	start_time = ktime_get();
	bcl_dev->ifpmic_irq_bins[irq_ind][NONE_BCL_BIN].start_time = start_time;
	if (bcl_dev->sub_pwr_warn_triggered[bcl_dev->rffe_channel])
		bcl_dev->ifpmic_irq_bins[irq_ind][MMWAVE_BCL_BIN].start_time = start_time;
	if (bcl_dev->main_pwr_warn_triggered[bcl_dev->rffe_channel])
		bcl_dev->ifpmic_irq_bins[irq_ind][RFFE_BCL_BIN].start_time = start_time;
}

void pwrwarn_update_start_time(struct bcl_device *bcl_dev,
					int id, struct irq_duration_stats *bins,
					bool *pwr_warn_triggered,
					enum CONCURRENT_PWRWARN_IRQ bin_ind)
{
	ktime_t start_time;
	int i;
	bool is_rf = bcl_dev->rffe_channel == id;

	if (bins[id].start_time != 0)
		return;

	start_time = ktime_get();
	if (is_rf && pwr_warn_triggered[id]) {
		for (i = 0; i < MAX_BCL_BATT_IRQ; i++) {
			if (bcl_dev->ifpmic_irq_bins[i][NONE_BCL_BIN].start_time != 0)
				bcl_dev->ifpmic_irq_bins[i][bin_ind].start_time = start_time;
		}
	}
	bins[id].start_time = start_time;
}

void pwrwarn_update_end_time(struct bcl_device *bcl_dev, int id, struct irq_duration_stats *bins,
				enum CONCURRENT_PWRWARN_IRQ bin_ind)
{
	ktime_t end_time;
	ktime_t time_delta;
	int i;
	bool is_rf = bcl_dev->rffe_channel == id;
	end_time = ktime_get();
	if (is_rf) {
		for (i = 0; i < MAX_BCL_BATT_IRQ; i++)
			if (bcl_dev->ifpmic_irq_bins[i][bin_ind].start_time != 0)
				bin_incr_ifpmic(bcl_dev, i, bin_ind, end_time);
	}

	if (bins[id].start_time == 0)
		return;

	time_delta = ktime_sub(end_time, bins[id].start_time);
	if (ktime_compare(time_delta, DELTA_10MS) < 0)
		atomic_inc(&(bins[id].lt_5ms_count));
	else if (ktime_compare(time_delta, DELTA_50MS) < 0)
		atomic_inc(&(bins[id].bt_5ms_10ms_count));
	else
		atomic_inc(&(bins[id].gt_10ms_count));
	bins[id].start_time = 0;
}

