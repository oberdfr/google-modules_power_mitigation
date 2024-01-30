// SPDX-License-Identifier: GPL-2.0-only
/*
 * google_bcl_votable.c Google bcl votable driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <misc/gvotable.h>
#include "bcl.h"

#define BCL_WLC "BCL_WLC"

enum {
	WLC_ENABLED_TX,
	WLC_DISABLED_TX,
};

static int google_bcl_wlc_votable_callback(struct gvotable_election *el,
					   const char *reason, void *value)
{
	struct bcl_device *bcl_dev = gvotable_get_data(el);
	int ret;
	u8 wlc_tx_enable = (long)value ? WLC_ENABLED_TX : WLC_DISABLED_TX;


	ret = max77779_adjust_batoilo_lvl(bcl_dev, wlc_tx_enable);
	if (ret < 0) {
		dev_err(bcl_dev->device, "BATOILO cannot be adjusted\n");
		return ret;
	}

	return 0;
}

int google_bcl_setup_votable(struct bcl_device *bcl_dev)
{
	int ret;

	bcl_dev->toggle_wlc = gvotable_create_bool_election(NULL, google_bcl_wlc_votable_callback,
							    bcl_dev);
	if (IS_ERR_OR_NULL(bcl_dev->toggle_wlc)) {
		ret = PTR_ERR(bcl_dev->toggle_wlc);
		dev_err(bcl_dev->device, "no toggle_wlc votable (%d)\n", ret);
		return ret;
	}
	gvotable_set_vote2str(bcl_dev->toggle_wlc, gvotable_v2s_int);
	gvotable_election_set_name(bcl_dev->toggle_wlc, BCL_WLC);
	return 0;
}

void google_bcl_remove_votable(struct bcl_device *bcl_dev)
{
	gvotable_destroy_election(bcl_dev->toggle_wlc);
}
