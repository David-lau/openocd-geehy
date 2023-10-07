// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by KaiLi                                           *
 *   kayli.wu@foxmail.com                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

/*
 * This code is referred to stm32f1x.c
 */

/* apm32x register locations */

#define FMC_REG_BASE 0x40022000

#define APM32_FMC_CTRL1			0x00
#define APM32_FMC_KEY			0x04
#define APM32_FMC_OBKEY			0x08
#define APM32_FMC_STS			0x0C
#define APM32_FMC_CTRL2			0x10
#define APM32_FMC_ADDR		0x14
#define APM32_FMC_OBCS			0x1C
#define APM32_FMC_WRTPROT		0x20

/* TODO: Check if code using these really should be hard coded to bank 0.
 * There are valid cases, on dual flash devices the protection of the
 * second bank is done on the bank0 reg's. */
#define APM32_FMC_CTRL1_B0		0x40022000
#define APM32_FMC_KEY_B0		0x40022004
#define APM32_FMC_OBKEY_B0		0x40022008
#define APM32_FMC_STS_B0		0x4002200C
#define APM32_FMC_CTRL2_B0		0x40022010
#define APM32_FMC_ADDR_B0		0x40022014
#define APM32_FMC_OBCS_B0		0x4002201C
#define APM32_FMC_WRTPROT_B0    0x40022020

/* option byte location */

#define APM32_OB_RDP		0x1FFFF800
#define APM32_OB_USER		0x1FFFF802
#define APM32_OB_DATA0		0x1FFFF804
#define APM32_OB_DATA1		0x1FFFF806
#define APM32_OB_WRP0		0x1FFFF808
#define APM32_OB_WRP1		0x1FFFF80A
#define APM32_OB_WRP2		0x1FFFF80C
#define APM32_OB_WRP3		0x1FFFF80E

/* FMC_CTRL2 register bits */

#define FMC_PG			BIT(0)
#define FMC_PAGEERA		BIT(1)
#define FMC_MASSERA		BIT(2)
#define FMC_OBP			BIT(4)
#define FMC_OBE			BIT(5)
#define FMC_STA			BIT(6)
#define FMC_LOCK		BIT(7)
#define FMC_OBWEN		BIT(9)
#define FMC_OBLOAD		BIT(13)	/* apm32f0xx series */

/* FMC_STS register bits */

#define FMC_BUSYF		BIT(0)
#define FMC_PEF			BIT(2)
#define FMC_WPEF		BIT(4)
#define FMC_OCF			BIT(5)

/* APM32_FMC_OBCS bit definitions (reading) */

#define OBCS_OBE		0
#define OBCS_READPROT	1
#define OBCS_WDTSEL		2
#define OBCS_RSTSTOP	3
#define OBCS_RSTSTDB	4

/* register unlock keys */

#define KEY1			0x45670123
#define KEY2			0xCDEF89AB

/* timeout values */

#define FLASH_WRITE_TIMEOUT 10
#define FLASH_ERASE_TIMEOUT 100

struct apm32x_options {
	uint8_t rdp;
	uint8_t user;
	uint16_t data;
	uint32_t protection;
};

struct apm32x_flash_bank {
	struct apm32x_options option_bytes;
	int ppage_size;
	bool probed;

	bool can_load_options;
	uint32_t register_base;
	uint8_t default_rdp;
	int user_data_offset;
	int option_offset;
	uint32_t user_bank_size;
};

static int apm32x_mass_erase(struct flash_bank *bank);
static int apm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t hwords_count);

/* flash bank apm32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(apm32x_flash_bank_command)
{
	struct apm32x_flash_bank *apm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	apm32x_info = malloc(sizeof(struct apm32x_flash_bank));

	bank->driver_priv = apm32x_info;
	apm32x_info->probed = false;
	apm32x_info->can_load_options = false;
	apm32x_info->register_base = FMC_REG_BASE;
	apm32x_info->user_bank_size = bank->size;

	/* The flash write must be aligned to a halfword boundary */
	bank->write_start_alignment = 2;
	bank->write_end_alignment = 2;

	return ERROR_OK;
}

static inline int apm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	return reg + apm32x_info->register_base;
}

static inline int apm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_STS), status);
}

static int apm32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = apm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FMC_BUSYF) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FLASH_BUSY;
		}
		alive_sleep(1);
	}

	if (status & FMC_WPEF) {
		LOG_ERROR("apm32x device protected");
		retval = ERROR_FLASH_PROTECTED;
	}

	if (status & FMC_PEF) {
		LOG_ERROR("apm32x device programming failed / flash not erased");
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

	/* Clear but report errors */
	if (status & (FMC_WPEF | FMC_PEF)) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_STS),
				FMC_WPEF | FMC_PEF);
	}
	return retval;
}

static int apm32x_check_operation_supported(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;

	/* if we have a dual flash bank device then
	 * we need to perform option byte stuff on bank0 only */
	if (apm32x_info->register_base != FMC_REG_BASE) {
		LOG_ERROR("Option byte operations must use bank 0");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int apm32x_read_options(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t option_bytes;
	int retval;

	/* read user and read protection option bytes, user data option bytes */
	retval = target_read_u32(target, APM32_FMC_OBCS_B0, &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	apm32x_info->option_bytes.rdp = (option_bytes & (1 << OBCS_READPROT)) ? 0 : apm32x_info->default_rdp;
	apm32x_info->option_bytes.user = (option_bytes >> apm32x_info->option_offset >> 2) & 0xff;
	apm32x_info->option_bytes.data = (option_bytes >> apm32x_info->user_data_offset) & 0xffff;

	/* read write protection option bytes */
	retval = target_read_u32(target, APM32_FMC_WRTPROT_B0, &apm32x_info->option_bytes.protection);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int apm32x_erase_options(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	struct target *target = bank->target;

	/* read current options */
	apm32x_read_options(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, APM32_FMC_KEY_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, APM32_FMC_KEY_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* unlock option flash registers */
	retval = target_write_u32(target, APM32_FMC_OBKEY_B0, KEY1);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, APM32_FMC_OBKEY_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* erase option bytes */
	retval = target_write_u32(target, APM32_FMC_CTRL2_B0, FMC_OBE | FMC_OBWEN);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, APM32_FMC_CTRL2_B0, FMC_OBE | FMC_STA | FMC_OBWEN);
	if (retval != ERROR_OK)
		goto flash_lock;

	retval = apm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* clear read protection option byte
	 * this will also force a device unlock if set */
	apm32x_info->option_bytes.rdp = apm32x_info->default_rdp;

	return ERROR_OK;

flash_lock:
	target_write_u32(target, APM32_FMC_CTRL2_B0, FMC_LOCK);
	return retval;
}

static int apm32x_write_options(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = NULL;
	struct target *target = bank->target;

	apm32x_info = bank->driver_priv;

	/* unlock flash registers */
	int retval = target_write_u32(target, APM32_FMC_KEY_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, APM32_FMC_KEY_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* unlock option flash registers */
	retval = target_write_u32(target, APM32_FMC_OBKEY_B0, KEY1);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, APM32_FMC_OBKEY_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* program option bytes */
	retval = target_write_u32(target, APM32_FMC_CTRL2_B0, FMC_OBP | FMC_OBWEN);
	if (retval != ERROR_OK)
		goto flash_lock;

	uint8_t opt_bytes[16];

	target_buffer_set_u16(target, opt_bytes, apm32x_info->option_bytes.rdp);
	target_buffer_set_u16(target, opt_bytes + 2, apm32x_info->option_bytes.user);
	target_buffer_set_u16(target, opt_bytes + 4, apm32x_info->option_bytes.data & 0xff);
	target_buffer_set_u16(target, opt_bytes + 6, (apm32x_info->option_bytes.data >> 8) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 8, apm32x_info->option_bytes.protection & 0xff);
	target_buffer_set_u16(target, opt_bytes + 10, (apm32x_info->option_bytes.protection >> 8) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 12, (apm32x_info->option_bytes.protection >> 16) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 14, (apm32x_info->option_bytes.protection >> 24) & 0xff);

	/* Block write is preferred in favour of operation with ancient ST-Link
	 * firmwares without 16-bit memory access. See
	 * 480: flash: apm32f1x: write option bytes using the loader
	 * https://review.openocd.org/c/openocd/+/480
	 */
	retval = apm32x_write_block(bank, opt_bytes, APM32_OB_RDP, sizeof(opt_bytes) / 2);

flash_lock:
	{
		int retval2 = target_write_u32(target, APM32_FMC_CTRL2_B0, FMC_LOCK);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int apm32x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t protection;

	int retval = apm32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	/* medium density - each bit refers to a 4 sector protection block
	 * high density - each bit refers to a 2 sector protection block
	 * bit 31 refers to all remaining sectors in a bank */
	retval = target_read_u32(target, APM32_FMC_WRTPROT_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

	return ERROR_OK;
}

static int apm32x_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (first == 0 && last == (bank->num_sectors - 1))
		return apm32x_mass_erase(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	for (unsigned int i = first; i <= last; i++) {
		retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_PAGEERA);
		if (retval != ERROR_OK)
			goto flash_lock;
		retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_ADDR),
				bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			goto flash_lock;
		retval = target_write_u32(target,
				apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_PAGEERA | FMC_STA);
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = apm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			goto flash_lock;
	}

flash_lock:
	{
		int retval2 = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_LOCK);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int apm32x_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = apm32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = apm32x_erase_options(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("apm32x failed to erase options");
		return retval;
	}

	for (unsigned int i = first; i <= last; i++) {
		if (set)
			apm32x_info->option_bytes.protection &= ~(1 << i);
		else
			apm32x_info->option_bytes.protection |= (1 << i);
	}

	return apm32x_write_options(bank);
}

static int apm32x_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t hwords_count)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct armv7m_algorithm armv7m_info;
	int retval;

	static const uint8_t apm32x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/apm32/apm32f1x.inc"
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(apm32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(apm32x_flash_write_code), apm32x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN(hwords_count * 2, MAX(buffer_size, 256));
	/* Normally we allocate all available working area.
	 * MIN shrinks buffer_size if the size of the written block is smaller.
	 * MAX prevents using async algo if the available working area is smaller
	 * than 256, the following allocation fails with
	 * ERROR_TARGET_RESOURCE_NOT_AVAILABLE and slow flashing takes place.
	 */

	retval = target_alloc_working_area(target, buffer_size, &source);
	/* Allocated size is always 32-bit word aligned */
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		/* target_alloc_working_area() may return ERROR_FAIL if area backup fails:
		 * convert any error to ERROR_TARGET_RESOURCE_NOT_AVAILABLE
		 */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[5];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, apm32x_info->register_base);
	buf_set_u32(reg_params[1].value, 0, 32, hwords_count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, hwords_count, 2,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		/* Actually we just need to check for programming errors
		 * apm32x_wait_status_busy also reports error and clears status bits.
		 *
		 * Target algo returns flash status in r0 only if properly finished.
		 * It is safer to re-read status register.
		 */
		int retval2 = apm32x_wait_status_busy(bank, 5);
		if (retval2 != ERROR_OK)
			retval = retval2;

		LOG_ERROR("flash write failed just before address 0x%" PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	return retval;
}

/** Writes a block to flash either using target algorithm
 *  or use fallback, host controlled halfword-by-halfword access.
 *  Flash controller must be unlocked before this call.
 */
static int apm32x_write_block(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t address, uint32_t hwords_count)
{
	struct target *target = bank->target;

	/* The flash write must be aligned to a halfword boundary.
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(address % 2 == 0);

	int retval;

    /* try using a block write - on ARM architecture or... */
	retval = apm32x_write_block_async(bank, buffer, address, hwords_count);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (hwords_count > 0) {
			retval = target_write_memory(target, address, 2, 1, buffer);
			if (retval != ERROR_OK)
				return retval;

			retval = apm32x_wait_status_busy(bank, 5);
			if (retval != ERROR_OK)
				return retval;

			hwords_count--;
			buffer += 2;
			address += 2;
		}
	}
	return retval;
}

static int apm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The flash write must be aligned to a halfword boundary.
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(offset % 2 == 0);
	assert(count % 2 == 0);

	int retval, retval2;

	/* unlock flash registers */
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY2);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* enable flash programming */
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_PG);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* write to flash */
	retval = apm32x_write_block(bank, buffer, bank->base + offset, count / 2);

reset_pg_and_lock:
	retval2 = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_LOCK);
	if (retval == ERROR_OK)
		retval = retval2;

	return retval;
}

struct apm32x_property_addr {
	uint32_t device_id;
	uint32_t flash_size;
};

static int apm32x_get_property_addr(struct target *target, struct apm32x_property_addr *addr)
{
	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	switch (cortex_m_get_partno_safe(target)) {
	case CORTEX_M0P_PARTNO: /* APM32F0xx devices */
		addr->device_id = 0x40015800;
		addr->flash_size = 0x1FFFF7CC;
		return ERROR_OK;
	case CORTEX_M3_PARTNO: /* APM32F1x devices */
		addr->device_id = 0xE0042000;
		addr->flash_size = 0x1FFFF7E0;
		return ERROR_OK;
	default:
		LOG_ERROR("Cannot identify target as a apm32x");
		return ERROR_FAIL;
	}
}

static int apm32x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	struct target *target = bank->target;
	struct apm32x_property_addr addr;

	int retval = apm32x_get_property_addr(target, &addr);
	if (retval != ERROR_OK)
		return retval;

	return target_read_u32(target, addr.device_id, device_id);
}

static int apm32x_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
	struct target *target = bank->target;
	struct apm32x_property_addr addr;

	int retval = apm32x_get_property_addr(target, &addr);
	if (retval != ERROR_OK)
		return retval;

	return target_read_u16(target, addr.flash_size, flash_size_in_kb);
}

static int apm32x_probe(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t dbgmcu_idcode;
	int page_size;
	uint32_t base_address = 0x08000000;

	apm32x_info->probed = false;
	apm32x_info->register_base = FMC_REG_BASE;
	apm32x_info->user_data_offset = 10;
	apm32x_info->option_offset = 0;

	/* default factory read protection level 0 */
	apm32x_info->default_rdp = 0xA5;

	/* read apm32 device id register */
	int retval = apm32x_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", dbgmcu_idcode);

	uint16_t device_id = dbgmcu_idcode & 0xfff;

	/* set page size, protection granularity and max flash size depending on family */
	switch (device_id) {
	case 0x440: /* apm32f05x */
		page_size = 1024;
		apm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 64;
		apm32x_info->user_data_offset = 16;
		apm32x_info->option_offset = 6;
		apm32x_info->default_rdp = 0xAA;
		apm32x_info->can_load_options = true;
		break;
	case 0x448: /* apm32f07x */
		page_size = 2048;
		apm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 128;
		apm32x_info->user_data_offset = 16;
		apm32x_info->option_offset = 6;
		apm32x_info->default_rdp = 0xAA;
		apm32x_info->can_load_options = true;
		break;
	case 0x442: /* apm32f09x */
		page_size = 2048;
		apm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 256;
		apm32x_info->user_data_offset = 16;
		apm32x_info->option_offset = 6;
		apm32x_info->default_rdp = 0xAA;
		apm32x_info->can_load_options = true;
		break;
	case 0x410: /* apm32f1x medium-density */
		page_size = 1024;
		apm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 128;
		break;
	case 0x412: /* apm32f1x low-density */
		page_size = 1024;
		apm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 32;
		break;
	case 0x414: /* apm32f1x high-density */
		page_size = 2048;
		apm32x_info->ppage_size = 2;
		max_flash_size_in_kb = 512;
		break;
	case 0x418: /* apm32f1x connectivity */
		page_size = 2048;
		apm32x_info->ppage_size = 2;
		max_flash_size_in_kb = 256;
		break;
	default:
		LOG_WARNING("Cannot identify target as a APM32 family.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = apm32x_get_flash_size(bank, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("APM32 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (apm32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = apm32x_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %d KiB", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->base = base_address;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* calculate number of write protection blocks */
	int num_prot_blocks = num_pages / apm32x_info->ppage_size;
	if (num_prot_blocks > 32)
		num_prot_blocks = 32;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, apm32x_info->ppage_size * page_size, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	if (num_prot_blocks == 32)
		bank->prot_blocks[31].size = (num_pages - (31 * apm32x_info->ppage_size)) * page_size;

	apm32x_info->probed = true;

	return ERROR_OK;
}

static int apm32x_auto_probe(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	if (apm32x_info->probed)
		return ERROR_OK;
	return apm32x_probe(bank);
}

static const char *get_apm32f0_revision(uint16_t rev_id)
{
	const char *rev_str = NULL;

	switch (rev_id) {
	case 0x1000:
		rev_str = "1.0";
		break;
	case 0x2000:
		rev_str = "2.0";
		break;
	}
	return rev_str;
}

static int get_apm32x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	uint32_t dbgmcu_idcode;

	/* read apm32 device id register */
	int retval = apm32x_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint16_t rev_id = dbgmcu_idcode >> 16;
	const char *device_str;
	const char *rev_str = NULL;

	switch (device_id) {
	case 0x410:
		device_str = "APM32F10x (Medium Density)";

		switch (rev_id) {
		case 0x0000:
			rev_str = "A";
			break;

		case 0x2000:
			rev_str = "B";
			break;

		case 0x2001:
			rev_str = "Z";
			break;

		case 0x2003:
			rev_str = "Y";
			break;
		}
		break;

	case 0x412:
		device_str = "APM32F10x (Low Density)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;
		}
		break;

	case 0x414:
		device_str = "APM32F10x (High Density)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;

		case 0x1003:
			rev_str = "Y";
			break;
		}
		break;

	case 0x418:
		device_str = "APM32F10x (Connectivity)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;
		}
		break;

	case 0x440:
		device_str = "APM32F05x";
		rev_str = get_apm32f0_revision(rev_id);
		break;

	case 0x445:
		device_str = "APM32F04x";
		rev_str = get_apm32f0_revision(rev_id);
		break;

	case 0x448:
		device_str = "APM32F07x";
		rev_str = get_apm32f0_revision(rev_id);
		break;

	case 0x442:
		device_str = "APM32F09x";
		rev_str = get_apm32f0_revision(rev_id);
		break;

	default:
		command_print_sameline(cmd, "Cannot identify target as a APM32F0/1/3\n");
		return ERROR_FAIL;
	}

	if (rev_str)
		command_print_sameline(cmd, "%s - Rev: %s", device_str, rev_str);
	else
		command_print_sameline(cmd, "%s - Rev: unknown (0x%04x)", device_str, rev_id);

	return ERROR_OK;
}

COMMAND_HANDLER(apm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct apm32x_flash_bank *apm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	apm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = apm32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	if (apm32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32x failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	apm32x_info->option_bytes.rdp = 0;

	if (apm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "apm32x locked");

	return ERROR_OK;
}

COMMAND_HANDLER(apm32x_handle_unlock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = apm32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	if (apm32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32x failed to erase options");
		return ERROR_OK;
	}

	if (apm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32x failed to unlock device");
		return ERROR_OK;
	}

	command_print(CMD, "apm32x unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(apm32x_handle_options_read_command)
{
	uint32_t optionbyte, protection;
	struct target *target = NULL;
	struct apm32x_flash_bank *apm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	apm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = apm32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, APM32_FMC_OBCS_B0, &optionbyte);
	if (retval != ERROR_OK)
		return retval;

	uint16_t user_data = optionbyte >> apm32x_info->user_data_offset;

	retval = target_read_u32(target, APM32_FMC_WRTPROT_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	if (optionbyte & (1 << OBCS_OBE))
		command_print(CMD, "option byte complement error");

	command_print(CMD, "option byte register = 0x%" PRIx32 "", optionbyte);
	command_print(CMD, "write protection register = 0x%" PRIx32 "", protection);

	command_print(CMD, "read protection: %s",
				(optionbyte & (1 << OBCS_READPROT)) ? "on" : "off");

	/* user option bytes are offset depending on variant */
	optionbyte >>= apm32x_info->option_offset;

	command_print(CMD, "watchdog: %sware",
				(optionbyte & (1 << OBCS_WDTSEL)) ? "soft" : "hard");

	command_print(CMD, "stop mode: %sreset generated upon entry",
				(optionbyte & (1 << OBCS_RSTSTOP)) ? "no " : "");

	command_print(CMD, "standby mode: %sreset generated upon entry",
				(optionbyte & (1 << OBCS_RSTSTDB)) ? "no " : "");

	command_print(CMD, "user data = 0x%02" PRIx16 "", user_data);

	return ERROR_OK;
}

COMMAND_HANDLER(apm32x_handle_options_write_command)
{
	struct target *target = NULL;
	struct apm32x_flash_bank *apm32x_info = NULL;
	uint8_t optionbyte;
	uint16_t useropt;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	apm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = apm32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = apm32x_read_options(bank);
	if (retval != ERROR_OK)
		return retval;

	/* start with current options */
	optionbyte = apm32x_info->option_bytes.user;
	useropt = apm32x_info->option_bytes.data;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	while (CMD_ARGC) {
		if (strcmp("SWWDG", CMD_ARGV[0]) == 0) {
			optionbyte |= (1 << 0);
		} else if (strcmp("HWWDG", CMD_ARGV[0]) == 0) {
			optionbyte &= ~(1 << 0);
		} else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0) {
			optionbyte |= (1 << 1);
		} else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0) {
			optionbyte &= ~(1 << 1);
		} else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0) {
			optionbyte |= (1 << 2);
		} else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0) {
			optionbyte &= ~(1 << 2);
		} else if (strcmp("USEROPT", CMD_ARGV[0]) == 0) {
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], useropt);
			CMD_ARGC--;
			CMD_ARGV++;
		} else {
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		CMD_ARGC--;
		CMD_ARGV++;
	}

	if (apm32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32x failed to erase options");
		return ERROR_OK;
	}

	apm32x_info->option_bytes.user = optionbyte;
	apm32x_info->option_bytes.data = useropt;

	if (apm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32x failed to write options");
		return ERROR_OK;
	}

	command_print(CMD, "apm32x write options complete.\n"
				"INFO: %spower cycle is required "
				"for the new settings to take effect.",
				apm32x_info->can_load_options
					? "'apm32f1x options_load' command or " : "");

	return ERROR_OK;
}

COMMAND_HANDLER(apm32x_handle_options_load_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;

	if (!apm32x_info->can_load_options) {
		LOG_ERROR("Command not applicable to apm32f1x devices - power cycle is "
			"required instead.");
		return ERROR_FAIL;
	}

	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = apm32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY2);
	if (retval != ERROR_OK) {
		(void)target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_LOCK);
		return retval;
	}

	/* force re-load of option bytes - generates software reset */
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_OBLOAD);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int apm32x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock option flash registers */
	int retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_KEY), KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* mass erase flash memory */
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_MASSERA);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2),
			FMC_MASSERA | FMC_STA);
	if (retval != ERROR_OK)
		goto flash_lock;

	retval = apm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

flash_lock:
	{
		int retval2 = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL2), FMC_LOCK);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

COMMAND_HANDLER(apm32x_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = apm32x_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "apm32x mass erase complete");
	else
		command_print(CMD, "apm32x mass erase failed");

	return retval;
}

static const struct command_registration apm32f1x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = apm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = apm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = apm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = apm32x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "options_write",
		.handler = apm32x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP') ('USEROPT' user_data)",
		.help = "Replace bits in device option bytes.",
	},
	{
		.name = "options_load",
		.handler = apm32x_handle_options_load_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Force re-load of device option bytes.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration apm32f1x_command_handlers[] = {
	{
		.name = "apm32f1x",
		.mode = COMMAND_ANY,
		.help = "apm32f1x flash command group",
		.usage = "",
		.chain = apm32f1x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver apm32f1x_flash = {
	.name = "apm32f1x",
	.commands = apm32f1x_command_handlers,
	.flash_bank_command = apm32x_flash_bank_command,
	.erase = apm32x_erase,
	.protect = apm32x_protect,
	.write = apm32x_write,
	.read = default_flash_read,
	.probe = apm32x_probe,
	.auto_probe = apm32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = apm32x_protect_check,
	.info = get_apm32x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
