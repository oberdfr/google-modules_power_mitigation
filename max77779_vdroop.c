/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <max77779_regs.h>
#include <max777x9_bcl.h>
#include "bcl.h"

int max77779_get_irq(struct bcl_device *bcl_dev, u8 *irq_val)
{
	unsigned int vdroop_int;
	u8 ret;
	u8 clr_bcl_irq_mask;

	clr_bcl_irq_mask = (MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK |
			    MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK |
			    MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_MASK |
			    MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK);
	ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_i2c,
		                              MAX77779_PMIC_VDROOP_INT,
					      &vdroop_int);
	if (ret < 0)
		return IRQ_NONE;
	if ((vdroop_int & clr_bcl_irq_mask) == 0)
		return IRQ_NONE;

	/* UVLO2 has the highest priority and then BATOILO, then UVLO1 */
	if (vdroop_int & MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK)
		*irq_val = UVLO2;
	else if (vdroop_int & MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK)
		*irq_val = BATOILO2;
	else if (vdroop_int & MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK)
		*irq_val = BATOILO1;
	else if (vdroop_int & MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_MASK)
		*irq_val = UVLO1;

	return ret;
}

int max77779_clr_irq(struct bcl_device *bcl_dev)
{
	u8 irq_val = 0;
	unsigned int chg_int = 0;
	int ret;

	if (max77779_get_irq(bcl_dev, &irq_val) != 0)
		return IRQ_NONE;

	if (irq_val == UVLO2)
		chg_int = MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK;
	else if (irq_val == UVLO1)
		chg_int = MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_MASK;
	else if (irq_val == BATOILO1)
		chg_int = MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK;
	else if (irq_val == BATOILO2)
		chg_int = MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK;

	ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_i2c,
		                               MAX77779_PMIC_VDROOP_INT, chg_int);
	if (ret < 0)
		return IRQ_NONE;
	return ret;
}

