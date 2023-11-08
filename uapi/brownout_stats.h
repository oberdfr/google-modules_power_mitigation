/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __BROWNOUT_STATS_H
#define __BROWNOUT_STATS_H

#define METER_CHANNEL_MAX	12
#define DATA_LOGGING_LEN	20

/* Indexing */
#define SMPL_WARN	0
#define OCP_WARN_CPUCL1	1
#define OCP_WARN_CPUCL2	2
#define SOFT_OCP_WARN_CPUCL1	3
#define SOFT_OCP_WARN_CPUCL2	4
#define OCP_WARN_TPU	5
#define SOFT_OCP_WARN_TPU	6
#define OCP_WARN_GPU	7
#define SOFT_OCP_WARN_GPU	8
#define PMIC_SOC	9
#define UVLO1	10
#define UVLO2	11
#define BATOILO1	12
#define BATOILO2	13
#define PMIC_120C	14
#define PMIC_140C	15
#define PMIC_OVERHEAT	16
#define BATOILO	BATOILO1
#define TRIGGERED_SOURCE_MAX	17

struct odpm_lpf {
	struct timespec64 time;
	u32 value[METER_CHANNEL_MAX];
};

/* Notice: sysfs only allocates a buffer of PAGE_SIZE
 * so the sizeof brownout_stats should be smaller than that
 */
struct brownout_stats {
	struct timespec64 triggered_time;
	u32 triggered_idx;

	struct odpm_lpf main_odpm_lpf[DATA_LOGGING_LEN];
	struct odpm_lpf sub_odpm_lpf[DATA_LOGGING_LEN];
	u32 triggered_state[DATA_LOGGING_LEN];
};
static_assert(sizeof(struct brownout_stats) <= PAGE_SIZE);

#endif /* __BROWNOUT_STATS_H */
