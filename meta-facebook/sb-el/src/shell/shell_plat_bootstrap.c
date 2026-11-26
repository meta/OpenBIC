/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <shell/shell.h>
#include <stdlib.h>
#include <logging/log.h>
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_cpld.h"
#include "plat_ioexp.h"
#include "plat_class.h"

#define AEGIS_CPLD_ADDR (0x4C >> 1)

LOG_MODULE_REGISTER(plat_bootstrap_shell, LOG_LEVEL_DBG);

/* DBB bootstrap preset. Keep the values indexed by enum PLAT_STRAP_INDEX_E. */
static const uint8_t dbb_bootstrap_settings[STRAP_INDEX_EXCEPT_EVB_MAX] = {
	[STRAP_INDEX_HAMSA_TEST_STRAP_R] = 0x00,
	[STRAP_INDEX_HAMSA_LS_STRAP_0] = 0x01,
	[STRAP_INDEX_HAMSA_LS_STRAP_1] = 0x00,
	[STRAP_INDEX_HAMSA_CRM_STRAP_0] = 0x00,
	[STRAP_INDEX_HAMSA_CRM_STRAP_1] = 0x00,
	[STRAP_INDEX_HAMSA_MFIO7] = 0x01,
	[STRAP_INDEX_HAMSA_MFIO9] = 0x00,
	[STRAP_INDEX_HAMSA_MFIO11] = 0x00,
	[STRAP_INDEX_HAMSA_MFIO17] = 0x00,
	[STRAP_INDEX_HAMSA_MFIO18] = 0x01,
	[STRAP_INDEX_HAMSA_CORE_TAP_CTRL_L] = 0x01,
	[STRAP_INDEX_HAMSA_TRI_L] = 0x01,
	[STRAP_INDEX_HAMSA_ATPG_MODE_L] = 0x01,
	[STRAP_INDEX_HAMSA_DFT_TAP_EN_L] = 0x01,
	[STRAP_INDEX_FM_JTAG_HAMSA_JTCE_0_3] = 0x01,
	[STRAP_INDEX_NUWA0_TEST_STRAP] = 0x00,
	[STRAP_INDEX_NUWA0_CRM_STRAP_0] = 0x00,
	[STRAP_INDEX_NUWA0_CRM_STRAP_1] = 0x00,
	[STRAP_INDEX_NUWA0_CHIP_STRAP_0] = 0x01,
	[STRAP_INDEX_NUWA0_CHIP_STRAP_1] = 0x00,
	[STRAP_INDEX_NUWA0_CORE_TAP_CTRL_PLD_L] = 0x01,
	[STRAP_INDEX_NUWA0_TRI_L] = 0x01,
	[STRAP_INDEX_NUWA0_ATPG_MODE_L] = 0x01,
	[STRAP_INDEX_NUWA0_DFT_TAP_EN_PLD_L] = 0x01,
	[STRAP_INDEX_NUWA1_TEST_STRAP] = 0x00,
	[STRAP_INDEX_NUWA1_CRM_STRAP_0] = 0x00,
	[STRAP_INDEX_NUWA1_CRM_STRAP_1] = 0x00,
	[STRAP_INDEX_NUWA1_CHIP_STRAP_0] = 0x01,
	[STRAP_INDEX_NUWA1_CHIP_STRAP_1] = 0x00,
	[STRAP_INDEX_NUWA1_CORE_TAP_CTRL_PLD_L] = 0x01,
	[STRAP_INDEX_NUWA1_TRI_L] = 0x01,
	[STRAP_INDEX_NUWA1_ATPG_MODE_L] = 0x01,
	[STRAP_INDEX_NUWA1_DFT_TAP_EN_PLD_L] = 0x01,
	[STRAP_INDEX_FM_JTAG_NUWA0_JTCE_0_2] = 0x01,
	[STRAP_INDEX_FM_JTAG_NUWA1_JTCE_0_2] = 0x01,
	[STRAP_INDEX_PLD_OWL_E_DFT_TAP_EN_L] = 0x01,
	[STRAP_INDEX_PLD_OWL_E_CORE_TAP_CTRL_L] = 0x01,
	[STRAP_INDEX_PLD_OWL_E_PAD_TRI_L] = 0x01,
	[STRAP_INDEX_PLD_OWL_E_ATPG_MODE_L] = 0x01,
	[STRAP_INDEX_PLD_OWL_W_DFT_TAP_EN_L] = 0x01,
	[STRAP_INDEX_PLD_OWL_W_CORE_TAP_CTRL_L] = 0x01,
	[STRAP_INDEX_PLD_OWL_W_PAD_TRI_L] = 0x01,
	[STRAP_INDEX_PLD_OWL_W_ATPG_MODE_L] = 0x01,
	[STRAP_INDEX_OWL_E_JTAG_MUX_PLD_SEL_00] = 0x00,
	[STRAP_INDEX_OWL_E_JTAG_MUX_PLD_SEL_01] = 0x00,
	[STRAP_INDEX_OWL_E_JTAG_MUX_PLD_SEL_02] = 0x00,
	[STRAP_INDEX_OWL_E_JTAG_MUX_PLD_SEL_03] = 0x00,
	[STRAP_INDEX_OWL_W_JTAG_MUX_PLD_SEL_0_3] = 0x00,
	[STRAP_INDEX_OWL_E_UART_MUX_PLD_SEL_0_2] = 0x00,
	[STRAP_INDEX_OWL_W_UART_MUX_PLD_SEL_0_2] = 0x00,
	[STRAP_INDEX_OWL_E_DVT_ENABLE] = 0x00,
	[STRAP_INDEX_OWL_W_DVT_ENABLE] = 0x00,
	[STRAP_INDEX_OWL_E_BOOT_SOURCE_0_7] = 0x00,
	[STRAP_INDEX_OWL_W_BOOT_SOURCE_0_7] = 0x00,
};
// Read bits from an 8-bit value in the range [start_bit .. end_bit] (inclusive).
// If reverse = true, the extracted bits will be reversed (bit order flipped).
uint8_t read_bits(uint8_t data, uint8_t start_bit, uint8_t end_bit, bool reverse)
{
	if (start_bit > 7 || end_bit > 7 || start_bit > end_bit) {
		return 0; // invalid parameters
	}

	// 1. Create mask and extract the bits
	uint8_t mask = ((1 << (end_bit - start_bit + 1)) - 1) << start_bit;
	uint8_t result = (data & mask) >> start_bit;

	// 2. Reverse bit order if requested
	if (reverse) {
		uint8_t rev = 0;
		uint8_t length = end_bit - start_bit + 1;
		for (uint8_t i = 0; i < length; i++) {
			rev <<= 1;
			rev |= (result >> i) & 0x1;
		}
		result = rev;
	}

	return result;
}
static int cmd_bootstrap_get_all(const struct shell *shell, size_t argc, char **argv)
{
	if (!strcmp(argv[1], "all")) {
		shell_print(shell, "%-4s|%-40s|%-25s", "id", "strap name", "hex-value");
		for (int i = 0; i < get_strap_index_max(); i++) {
			uint8_t *rail_name = NULL;
			if (!strap_name_get((uint8_t)i, &rail_name)) {
				LOG_ERR("Can't find strap_rail_name by rail index: %x", i);
				continue;
			}

			int drive_level = -1;

			if (!get_bootstrap_change_drive_level(i, &drive_level)) {
				LOG_ERR("Can't get_bootstrap_change_drive_level by rail index: %x",
					i);
				continue;
			}

			shell_print(shell, "%-4d|%-40s|0x%-23.2x", i, rail_name, drive_level);
		}
	} else {
		/* covert string to enum */
		enum PLAT_STRAP_INDEX_E rail;
		if (strap_enum_get(argv[1], &rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return -1;
		}

		int drive_level = -1;
		if (!get_bootstrap_change_drive_level(rail, &drive_level)) {
			LOG_ERR("Can't get level by index: %x", rail);
			return -1;
		}
		shell_print(shell, "%-4s|%-40s|%-25s", "id", "strap name", "hex-value");
		shell_print(shell, "%-4d|%-40s|0x%-23.2x", rail, argv[1], drive_level);
	}

	return 0;
}

static int bootstrap_set_all_default(const struct shell *shell)
{
	uint8_t change_setting_value;
	uint8_t drive_index_level = 0;
	bootstrap_mapping_register bootstrap_item;
	bool all_success = true;

	for (int i = 0; i < get_strap_index_max(); i++) {
		if (!set_bootstrap_table_and_user_settings(i, &change_setting_value,
							   drive_index_level, false, true)) {
			shell_print(shell, "plat bootstrap[%2d] set failed", i);
			all_success = false;
		}
		if (!find_bootstrap_by_rail((uint8_t)i, &bootstrap_item)) {
			shell_print(shell, "Can't find bootstrap_item by rail index: %d", i);
			continue;
		}
		// write change_setting_value to cpld or io-exp
		if (!set_bootstrap_val_to_device(i, change_setting_value))
			shell_print(shell, "Can't set bootstrap[0x%02x] to default", i);
	}

	if (all_success) {
		shell_print(shell, "Set all bootstrap to default");
	}

	return 0;
}

static int bootstrap_set_dbb(const struct shell *shell, bool is_perm)
{
	for (int i = 0; i < ARRAY_SIZE(dbb_bootstrap_settings); i++) {
		uint8_t change_setting_value = 0;

		if (!set_bootstrap_table_and_user_settings(
			    i, &change_setting_value, dbb_bootstrap_settings[i], is_perm, false)) {
			shell_error(shell, "Failed to set DBB bootstrap[%d]", i);
			return -1;
		}

		if (!set_bootstrap_val_to_device(i, change_setting_value)) {
			shell_error(shell, "Failed to write DBB bootstrap[%d]", i);
			return -1;
		}
	}

	shell_print(shell, "Set DBB bootstrap preset, %svolatile", is_perm ? "non-" : "");
	return 0;
}

static int cmd_bootstrap_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;
	uint8_t change_setting_value;
	uint8_t drive_index_level = 0;
	bootstrap_mapping_register bootstrap_item;

	if (!strcmp(argv[1], "DBB")) {
		if ((argc > 3) || ((argc == 3) && strcmp(argv[2], "perm"))) {
			shell_error(shell, "Usage: bootstrap set DBB [perm]");
			return -1;
		}
		return bootstrap_set_dbb(shell, argc == 3);
	}

	if (argc < 3) {
		shell_error(shell,
			    "Usage: bootstrap set <strap-name>|all <hex-value>|default [perm]");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		if (!strcmp(argv[2], "default")) {
			bootstrap_set_all_default(shell);
			return 0;
		} else {
			shell_error(shell, "Only support bootstrap set all default");
			return -1;
		}
	}

	if (argc == 4) {
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}
	/* covert string to enum */
	enum PLAT_STRAP_INDEX_E rail;
	if (strap_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}
	// check TEST_STRAP for MFIO 6 8 10
	int drive_level = -1;

	switch (rail) {
	case STRAP_INDEX_HAMSA_MFIO6:
	case STRAP_INDEX_HAMSA_MFIO8:
	case STRAP_INDEX_HAMSA_MFIO10:
		get_bootstrap_change_drive_level(STRAP_INDEX_HAMSA_TEST_STRAP_R, &drive_level);
		if (drive_level == 0) {
			shell_error(shell, "Can't change due to HAMSA_TEST_STRAP_R is 0x00 ");
			return -1;
		}
		break;
	case STRAP_INDEX_NUWA0_MFIO6:
	case STRAP_INDEX_NUWA0_MFIO8:
	case STRAP_INDEX_NUWA0_MFIO10:
		get_bootstrap_change_drive_level(STRAP_INDEX_NUWA0_TEST_STRAP, &drive_level);
		if (drive_level == 0) {
			shell_error(shell, "Can't change due to NUWA0_TEST_STRAP is 0x00");
			return -1;
		}
		break;
	case STRAP_INDEX_NUWA1_MFIO6:
	case STRAP_INDEX_NUWA1_MFIO8:
	case STRAP_INDEX_NUWA1_MFIO10:
		get_bootstrap_change_drive_level(STRAP_INDEX_NUWA1_TEST_STRAP, &drive_level);
		if (drive_level == 0) {
			shell_error(shell, "Can't change due to NUWA1_TEST_STRAP is 0x00 ");
			return -1;
		}
		break;
	default:
		break;
	}

	bool is_default = false;
	if (!strcmp(argv[2], "default"))
		is_default = true;
	else
		drive_index_level = strtol(argv[2], NULL, 16);

	if (!set_bootstrap_table_and_user_settings(rail, &change_setting_value, drive_index_level,
						   is_perm, is_default)) {
		shell_error(shell, "plat bootstrap set failed");
		return -1;
	}
	if (!find_bootstrap_by_rail((uint8_t)rail, &bootstrap_item)) {
		shell_error(shell, "Can't find bootstrap_item by rail index: %d", rail);
		return -1;
	}

	// write change_setting_value to cpld or io-exp
	if (!set_bootstrap_val_to_device(bootstrap_item.index, change_setting_value)) {
		LOG_ERR("Can't set bootstrap[%2d]=%02x", bootstrap_item.index,
			change_setting_value);
		return -1;
	}

	shell_print(shell, "Set %s %s, %svolatile\n", argv[1], argv[2], (argc == 4) ? "non-" : "");
	return 0;
}

static void strap_rname_get_(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	strap_name_get((uint8_t)idx, &name);

	if (idx == get_strap_index_max())
		name = (uint8_t *)"all";
	else if (idx == (get_strap_index_max() + 1))
		name = (uint8_t *)"DBB";

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(strap_name, strap_rname_get_);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_bootstrap_cmds, SHELL_CMD(get, &strap_name, "get <strap-name>", cmd_bootstrap_get_all),
	SHELL_CMD_ARG(set, &strap_name,
		      "set <strap-name>|all <hex-value>|default [perm], or set DBB [perm]",
		      cmd_bootstrap_set, 2, 2),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(bootstrap, &sub_bootstrap_cmds, "bootstrap set/get commands", NULL);
