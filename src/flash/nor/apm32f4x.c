// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by KaiLi                                           *
 *   kayli.wu@foxmail.com                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

/*
 * This code is referred to stm32f2x.c
 */

/* Erase time can be as high as 1000ms, 10x this and it's toast... */
#define FLASH_ERASE_TIMEOUT 10000
#define FLASH_WRITE_TIMEOUT 5

/* Mass erase time can be as high as 32 s in x8 mode. */
#define FLASH_MASS_ERASE_TIMEOUT 33000

#define FLASH_BANK_BASE     0x80000000

/* "One-time programmable bytes" */
#define APM32F4_OTP_SIZE			512
#define APM32F4_OTP_SECTOR_SIZE	32
#define APM32F4_OTP_BANK_BASE       0x1fff7800
#define APM32F4_OTP_LOCK_BASE       ((APM32F4_OTP_BANK_BASE) + (APM32F4_OTP_SIZE))

#define APM32_FMC_BASE		0x40023c00
#define APM32_FMC_ACCTRL	0x40023c00
#define APM32_FMC_KEY		0x40023c04
#define APM32_FMC_OPTKEY	0x40023c08
#define APM32_FMC_STS		0x40023c0C
#define APM32_FMC_CTRL      0x40023c10
#define APM32_FMC_OPTCTRL   0x40023c14

/* FMC_CTRL register bits */
#define FMC_CTRL		BIT(0)
#define FMC_SERS		BIT(1)
#define FMC_MERS		BIT(2)
#define FMC_START		BIT(16)
#define FMC_PGSIZE_8	(0 << 8)
#define FMC_PGSIZE_16	BIT(8)
#define FMC_PGSIZE_32	(2 << 8)
#define FMC_PGSIZE_64	(3 << 8)
/* The sector number encoding is not straight binary for dual bank flash. */
#define FMC_SNUM(a)	 ((a) << 3)
#define FMC_LOCK		BIT(31)

/* FMC_STS register bits */
#define FMC_BUSY		BIT(16)
#define FMC_PGSEQERR    BIT(7) /* Programming sequence error */
#define FMC_PGPRLERR	BIT(6) /* Programming parallelism error */
#define FMC_PGALGERR	BIT(5) /* Programming alignment error */
#define FMC_WPROTERR	BIT(4) /* Write protection error */
#define FMC_OPRCMP		BIT(1) /* Operation error */

#define FLASH_ERROR (FMC_PGSEQERR  | FMC_PGPRLERR | FMC_PGALGERR | FMC_WPROTERR | FMC_OPRCMP)

/* APM32_FMC_OPTCTRL register bits */
#define OPTCTRL_OPTLOCK     BIT(0)
#define OPTCTRL_OPTSTART    BIT(1)

/* register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

struct apm32x_options {
	uint8_t RDP;
	uint16_t user_options;	/* bit 0-7 usual options, bit 8-11 extra options */
	uint32_t protection;
	uint32_t boot_addr;
	uint32_t optcr2_pcrop;
};

struct apm32x_flash_bank {
	struct apm32x_options option_bytes;
	bool probed;
	bool otp_unlocked;
	uint32_t user_bank_size;
};

static bool apm32x_is_otp(struct flash_bank *bank)
{
	return bank->base == APM32F4_OTP_BANK_BASE;
}

static int apm32x_is_otp_unlocked(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;

	return apm32x_info->otp_unlocked;
}

static int apm32x_otp_disable(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;

	LOG_INFO("OTP memory bank #%u is disabled for write commands.",
		 bank->bank_number);
	apm32x_info->otp_unlocked = false;
	return ERROR_OK;
}

static int apm32x_otp_enable(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;

	if (!apm32x_info->otp_unlocked) {
		LOG_INFO("OTP memory bank #%u is enabled for write commands.",
			 bank->bank_number);
		apm32x_info->otp_unlocked = true;
	} else {
		LOG_WARNING("OTP memory bank #%u is already enabled for write commands.",
			    bank->bank_number);
	}
	return ERROR_OK;
}

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
	apm32x_info->otp_unlocked = false;
	apm32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline int apm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	return reg;
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
		LOG_DEBUG("status: 0x%" PRIx32, status);
		if ((status & FMC_BUSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}


	if (status & FMC_WPROTERR) {
		LOG_ERROR("apm32x device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR) {
		if (retval == ERROR_OK)
			retval = ERROR_FAIL;
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_STS),
				status & FLASH_ERROR);
	}
	return retval;
}

static int apm32x_unlock_reg(struct target *target)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on APM32_FMC_KEY will fail
	 */
	int retval = target_read_u32(target, APM32_FMC_CTRL, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FMC_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = target_write_u32(target, APM32_FMC_KEY, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, APM32_FMC_KEY, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, APM32_FMC_CTRL, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FMC_LOCK) {
		LOG_ERROR("flash not unlocked APM32_FMC_CTRL: 0x%" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int apm32x_unlock_option_reg(struct target *target)
{
	uint32_t ctrl;

	int retval = target_read_u32(target, APM32_FMC_OPTCTRL, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & OPTCTRL_OPTLOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, APM32_FMC_OPTKEY, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, APM32_FMC_OPTKEY, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, APM32_FMC_OPTCTRL, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & OPTCTRL_OPTLOCK) {
		LOG_ERROR("options not unlocked APM32_FMC_OPTCTRL: 0x%" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int apm32x_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct apm32x_flash_bank *apm32x_info = NULL;
	struct target *target = bank->target;

	apm32x_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, APM32_FMC_OPTCTRL, &optiondata);
	if (retval != ERROR_OK)
		return retval;

    /* caution: F4 implements 5 bits (WDG_SW) */
	apm32x_info->option_bytes.user_options = optiondata & 0xfc;
	apm32x_info->option_bytes.RDP = (optiondata >> 8) & 0xff;
	apm32x_info->option_bytes.protection = (optiondata >> 16);

	apm32x_info->option_bytes.optcr2_pcrop = 0x0;

	if (apm32x_info->option_bytes.RDP != 0xAA)
		LOG_INFO("Device Security Bit Set");

	return ERROR_OK;
}

static int apm32x_write_options(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = NULL;
	struct target *target = bank->target;
	uint32_t optiondata;

	apm32x_info = bank->driver_priv;

	int retval = apm32x_unlock_option_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* rebuild option data */
	optiondata = apm32x_info->option_bytes.user_options & 0xfc;
	optiondata |= apm32x_info->option_bytes.RDP << 8;
	optiondata |= apm32x_info->option_bytes.protection;

	/* program options */
	retval = target_write_u32(target, APM32_FMC_OPTCTRL, optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* start programming cycle */
	retval = target_write_u32(target, APM32_FMC_OPTCTRL, optiondata | OPTCTRL_OPTSTART);
	if (retval != ERROR_OK)
		return retval;

	/* wait for completion, this might trigger a security erase and take a while */
	retval = apm32x_wait_status_busy(bank, FLASH_MASS_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* relock registers */
	retval = target_write_u32(target, APM32_FMC_OPTCTRL, optiondata | OPTCTRL_OPTLOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int apm32x_otp_read_protect(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t lock_base;
	int retval;
	uint8_t lock;

	lock_base = APM32F4_OTP_LOCK_BASE;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		retval = target_read_u8(target, lock_base + i, &lock);
		if (retval != ERROR_OK)
			return retval;
		bank->sectors[i].is_protected = !lock;
	}

	return ERROR_OK;
}

static int apm32x_otp_protect(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	uint32_t lock_base;
	int i, retval;
	uint8_t lock;

	assert((first <= last) && (last < bank->num_sectors));

	lock_base = APM32F4_OTP_LOCK_BASE;

	for (i = first; first <= last; i++) {
		retval = target_read_u8(target, lock_base + i, &lock);
		if (retval != ERROR_OK)
			return retval;
		if (lock)
			continue;

		lock = 0xff;
		retval = target_write_u8(target, lock_base + i, lock);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int apm32x_protect_check(struct flash_bank *bank)
{
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	struct flash_sector *prot_blocks;
	unsigned int num_prot_blocks;
	int retval;

	/* if it's the OTP bank, look at the lock bits there */
	if (apm32x_is_otp(bank))
		return apm32x_otp_read_protect(bank);

	/* read write protection settings */
	retval = apm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	if (bank->prot_blocks) {
		num_prot_blocks = bank->num_prot_blocks;
		prot_blocks = bank->prot_blocks;
	} else {
		num_prot_blocks = bank->num_sectors;
		prot_blocks = bank->sectors;
	}

	for (unsigned int i = 0; i < num_prot_blocks; i++)
		prot_blocks[i].is_protected =
			~(apm32x_info->option_bytes.protection >> i) & 1;

	return ERROR_OK;
}

static int apm32x_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (apm32x_is_otp(bank)) {
		LOG_ERROR("Cannot erase OTP memory");
		return ERROR_FAIL;
	}

	assert((first <= last) && (last < bank->num_sectors));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = apm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register
	2. Set the SER bit and select the sector
	  you wish to erase (SNB) in the FLASH_CR register
	3. Set the STRT bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	 */

	for (unsigned int i = first; i <= last; i++) {
		unsigned int snb;

		snb = i;

		retval = target_write_u32(target,
				apm32x_get_flash_reg(bank, APM32_FMC_CTRL), FMC_SERS | FMC_SNUM(snb) | FMC_START);
		if (retval != ERROR_OK)
			return retval;

		retval = apm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL), FMC_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
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

	if (apm32x_is_otp(bank)) {
		if (!set)
			return ERROR_COMMAND_ARGUMENT_INVALID;

		return apm32x_otp_protect(bank, first, last);
	}

	/* read protection settings */
	int retval = apm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (unsigned int i = first; i <= last; i++) {
		if (set)
			apm32x_info->option_bytes.protection &= ~(1 << i);
		else
			apm32x_info->option_bytes.protection |= (1 << i);
	}

	retval = apm32x_write_options(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int apm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t apm32x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/apm32/apm32f4x.inc"
	};

	if (apm32x_is_otp(bank) && !apm32x_is_otp_unlocked(bank)) {
		LOG_ERROR("OTP memory bank is disabled for write commands.");
		return ERROR_FAIL;
	}

	if (target_alloc_working_area(target, sizeof(apm32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(apm32x_flash_write_code),
			apm32x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count (halfword-16bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);		/* flash base */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, APM32_FMC_BASE);

	retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing apm32x flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR;

		if (error & FMC_WPROTERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = 0x%08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, APM32_FMC_STS, error);
			retval = ERROR_FAIL;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int apm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t words_remaining = (count / 2);
	uint32_t bytes_remaining = (count & 0x00000001);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = apm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* multiple half words (2-byte) to be programmed? */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = apm32x_write_block(bank, buffer, offset, words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
		} else {
			buffer += words_remaining * 2;
			address += words_remaining * 2;
			words_remaining = 0;
		}
	}

	if (retval != ERROR_OK && retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		return retval;

	/*
	Standard programming
	The Flash memory programming sequence is as follows:
	1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register.
	2. Set the PG bit in the FLASH_CR register
	3. Perform the data write operation(s) to the desired memory address (inside main
	  memory block or OTP area):
	– – Half-word access in case of x16 parallelism
	– Word access in case of x32 parallelism
	–
	4.
	Byte access in case of x8 parallelism
	Double word access in case of x64 parallelism
	Wait for the BSY bit to be cleared
	*/
	while (words_remaining > 0) {
		retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL),
				FMC_CTRL | FMC_PGSIZE_16);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_memory(target, address, 2, 1, buffer + bytes_written);
		if (retval != ERROR_OK)
			return retval;

		retval = apm32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 2;
		words_remaining--;
		address += 2;
	}

	if (bytes_remaining) {
		retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL),
				FMC_CTRL | FMC_PGSIZE_8);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u8(target, address, buffer[bytes_written]);
		if (retval != ERROR_OK)
			return retval;

		retval = apm32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	return target_write_u32(target, APM32_FMC_CTRL, FMC_LOCK);
}

static void setup_sector(struct flash_bank *bank, unsigned int i,
		unsigned int size)
{
	assert(i < bank->num_sectors);
	bank->sectors[i].offset = bank->size;
	bank->sectors[i].size = size;
	bank->size += bank->sectors[i].size;
	LOG_DEBUG("sector %u: %ukBytes", i, size >> 10);
}

static uint16_t sector_size_in_kb(unsigned int i, uint16_t max_sector_size_in_kb)
{
	if (i < 4)
		return max_sector_size_in_kb / 8;
	if (i == 4)
		return max_sector_size_in_kb / 2;
	return max_sector_size_in_kb;
}

static unsigned int calculate_number_of_sectors(struct flash_bank *bank,
		uint16_t flash_size_in_kb,
		uint16_t max_sector_size_in_kb)
{
	uint16_t remaining_flash_size_in_kb = flash_size_in_kb;
	unsigned int nr_sectors;

	for (nr_sectors = 0; remaining_flash_size_in_kb > 0; nr_sectors++) {
		uint16_t size_in_kb = sector_size_in_kb(nr_sectors, max_sector_size_in_kb);
		if (size_in_kb > remaining_flash_size_in_kb) {
			LOG_INFO("%" PRIu16 " kiB final sector clipped to %" PRIu16 " kiB",
				 flash_size_in_kb, remaining_flash_size_in_kb);
			remaining_flash_size_in_kb = 0;
		} else {
			remaining_flash_size_in_kb -= size_in_kb;
		}
	}

	return nr_sectors;
}

static void setup_bank(struct flash_bank *bank, unsigned int start,
	uint16_t flash_size_in_kb, uint16_t max_sector_size_in_kb)
{
	uint16_t remaining_flash_size_in_kb = flash_size_in_kb;
	unsigned int sector_index = 0;
	while (remaining_flash_size_in_kb > 0) {
		uint16_t size_in_kb = sector_size_in_kb(sector_index, max_sector_size_in_kb);
		if (size_in_kb > remaining_flash_size_in_kb) {
			/* Clip last sector. Already warned in
			 * calculate_number_of_sectors. */
			size_in_kb = remaining_flash_size_in_kb;
		}
		setup_sector(bank, start + sector_index, size_in_kb * 1024);
		remaining_flash_size_in_kb -= size_in_kb;
		sector_index++;
	}
}

static int apm32x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	/* this checks for a apm32f4x errata issue where a
	 * apm32f4x DBGMCU_IDCODE is incorrectly returned.
	 * If the issue is detected target is forced to apm32f4x Rev A.
	 * Only effects Rev A silicon */

	struct target *target = bank->target;

	/* read apm32 device id register */
	int retval = target_read_u32(target, 0xE0042000, device_id);
	if (retval != ERROR_OK)
		return retval;

	if ((*device_id & 0xfff) == 0x411
			&& cortex_m_get_partno_safe(target) == CORTEX_M4_PARTNO) {
		*device_id &= ~((0xFFFF << 16) | 0xfff);
		*device_id |= (0x1000 << 16) | 0x413;
		LOG_INFO("apm32f4x errata detected - fixing incorrect MCU_IDCODE");
	}
	return retval;
}

static int apm32x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct apm32x_flash_bank *apm32x_info = bank->driver_priv;
	unsigned int num_prot_blocks, num_sectors;
	uint16_t flash_size_in_kb;
	uint16_t otp_size_in_b;
	uint16_t otp_sector_size;
	uint32_t flash_size_reg = 0x1FFF7A22;
	uint16_t max_sector_size_in_kb = 128;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	uint32_t base_address = 0x08000000;

	apm32x_info->probed = false;
	num_prot_blocks = 0;

	free(bank->sectors);
	bank->num_sectors = 0;
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->num_prot_blocks = 0;
	bank->prot_blocks = NULL;

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	/* if explicitly called out as OTP bank, short circuit probe */
	if (apm32x_is_otp(bank)) {
		otp_size_in_b = APM32F4_OTP_SIZE;
		otp_sector_size = APM32F4_OTP_SECTOR_SIZE;

		num_sectors = otp_size_in_b / otp_sector_size;
		LOG_INFO("flash size = %" PRIu16 " bytes", otp_size_in_b);

		assert(num_sectors > 0);

		bank->num_sectors = num_sectors;
		bank->sectors = calloc(sizeof(struct flash_sector), num_sectors);

		bank->size = APM32F4_OTP_SIZE;

		for (unsigned int i = 0; i < num_sectors; i++) {
			bank->sectors[i].offset = i * otp_sector_size;
			bank->sectors[i].size = otp_sector_size;
			bank->sectors[i].is_erased = 1;
			bank->sectors[i].is_protected = 0;
		}

		apm32x_info->probed = true;
		return ERROR_OK;
	}

	/* read apm32 device id register */
	int retval = apm32x_get_device_id(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;
	LOG_INFO("device id = 0x%08" PRIx32, device_id);
	device_id &= 0xfff;		/* only bits 0-11 are used further on */

	/* set max flash size depending on family, id taken from AN2606 */
	switch (device_id) {
	case 0x413: /* F40x/41x */
		max_flash_size_in_kb = 1024;
		break;

	case 0x431: /* F411 */
	case 0x433: /* F401xD/E */
		max_flash_size_in_kb = 512;
		break;

	default:
		LOG_WARNING("Cannot identify target as a APM32 family.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = target_read_u16(target, flash_size_reg, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("APM32 flash size failed, probe inaccurate - assuming %" PRIu16 "k flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (apm32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = apm32x_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %" PRIu16 " KiB", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	unsigned int num_pages = calculate_number_of_sectors(bank,
			flash_size_in_kb, max_sector_size_in_kb);

	bank->base = base_address;
	bank->num_sectors = num_pages;
	bank->sectors = calloc(num_pages, sizeof(struct flash_sector));
	for (unsigned int i = 0; i < num_pages; i++) {
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}
	bank->size = 0;
	LOG_DEBUG("allocated %u sectors", num_pages);

	if (num_prot_blocks) {
		bank->prot_blocks = malloc(sizeof(struct flash_sector) * num_prot_blocks);
		for (unsigned int i = 0; i < num_prot_blocks; i++)
			bank->prot_blocks[i].is_protected = 0;
		LOG_DEBUG("allocated %u prot blocks", num_prot_blocks);
	}

	setup_bank(bank, 0, flash_size_in_kb, max_sector_size_in_kb);

	bank->num_prot_blocks = num_prot_blocks;
	assert((bank->size >> 10) == flash_size_in_kb);

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
	case 0x413:
		device_str = "APM32F4xx";

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

		case 0x1007:
			rev_str = "1";
			break;

		case 0x2001:
			rev_str = "3";
			break;
		}
		break;

	case 0x431:
	case 0x433:
		device_str = "APM32F4xx (Low Power)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;

		case 0x2000:
			rev_str = "B";
			break;

		case 0x3000:
			rev_str = "C";
			break;
		}
		break;

	default:
		command_print_sameline(cmd, "Cannot identify target as a APM32F4\n");
		return ERROR_FAIL;
	}

	if (rev_str)
		command_print_sameline(cmd, "%s - Rev: %s", device_str, rev_str);
	else
		command_print_sameline(cmd, "%s - Rev: unknown (0x%04" PRIx16 ")", device_str, rev_id);

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
		LOG_INFO("Target not halted");
		/* return ERROR_TARGET_NOT_HALTED; */
	}

	if (apm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* set readout protection */
	apm32x_info->option_bytes.RDP = 0;

	if (apm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to lock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD, "%s locked", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(apm32x_handle_unlock_command)
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
		LOG_INFO("Target not halted");
		/* return ERROR_TARGET_NOT_HALTED; */
	}

	if (apm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	apm32x_info->option_bytes.RDP = 0xAA;

	if (apm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD, "%s unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.", bank->driver->name);

	return ERROR_OK;
}

static int apm32x_mass_erase(struct flash_bank *bank)
{
	int retval;
	uint32_t flash_mer  = FMC_MERS;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = apm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL), flash_mer);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL),
		flash_mer | FMC_START);
	if (retval != ERROR_OK)
		return retval;

	retval = apm32x_wait_status_busy(bank, FLASH_MASS_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, apm32x_get_flash_reg(bank, APM32_FMC_CTRL), FMC_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(apm32x_handle_mass_erase_command)
{
	if (CMD_ARGC < 1) {
		command_print(CMD, "apm32x mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

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

COMMAND_HANDLER(apm32f4x_handle_options_read_command)
{
	int retval;
	struct flash_bank *bank;
	struct apm32x_flash_bank *apm32x_info = NULL;

	if (CMD_ARGC != 1) {
		command_print(CMD, "apm32f4x options_read <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = apm32x_read_options(bank);
	if (retval != ERROR_OK)
		return retval;

	apm32x_info = bank->driver_priv;
	command_print(CMD, "apm32f4x user_options 0x%02" PRIX16,
		apm32x_info->option_bytes.user_options);

	return retval;
}

COMMAND_HANDLER(apm32f4x_handle_options_write_command)
{
	int retval;
	struct flash_bank *bank;
	struct apm32x_flash_bank *apm32x_info = NULL;
	uint16_t user_options;

	if (CMD_ARGC < 1) {
		command_print(CMD, "apm32f4x options_write <bank> ...");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = apm32x_read_options(bank);
	if (retval != ERROR_OK)
		return retval;

	apm32x_info = bank->driver_priv;

	if (CMD_ARGC != 2) {
		command_print(CMD, "apm32f4x options_write <bank> <user_options>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], user_options);
	if (user_options) {
		command_print(CMD, "apm32f4x invalid user_options");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	apm32x_info->option_bytes.user_options = user_options;

	if (apm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32f4x failed to write options");
		return ERROR_OK;
	}

	/* switching between single- and dual-bank modes requires re-probe */
	/* ... and reprogramming of whole flash */
	apm32x_info->probed = false;

	command_print(CMD, "apm32f4x write options complete.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.");
	return retval;
}

COMMAND_HANDLER(apm32f4x_handle_optcr2_write_command)
{
	int retval;
	struct flash_bank *bank;
	struct apm32x_flash_bank *apm32x_info = NULL;
	uint32_t optcr2_pcrop;

	if (CMD_ARGC != 2) {
		command_print(CMD, "apm32f4x optcr2_write <bank> <optcr2_value>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	apm32x_info = bank->driver_priv;

	command_print(CMD, "INFO: To disable PCROP, set PCROP_RDP"
				" with PCROPi bits STILL SET, then\nlock device and"
				" finally unlock it. Clears PCROP and mass erases flash.");

	retval = apm32x_read_options(bank);
	if (retval != ERROR_OK)
		return retval;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], optcr2_pcrop);
	apm32x_info->option_bytes.optcr2_pcrop = optcr2_pcrop;

	if (apm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "apm32f4x failed to write options");
		return ERROR_OK;
	}

	command_print(CMD, "apm32f4x optcr2_write complete.");
	return retval;
}

COMMAND_HANDLER(apm32x_handle_otp_command)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "apm32x otp <bank> (enable|disable|show)");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	if (apm32x_is_otp(bank)) {
		if (strcmp(CMD_ARGV[1], "enable") == 0) {
			apm32x_otp_enable(bank);
		} else if (strcmp(CMD_ARGV[1], "disable") == 0) {
			apm32x_otp_disable(bank);
		} else if (strcmp(CMD_ARGV[1], "show") == 0) {
			command_print(CMD,
				"OTP memory bank #%u is %s for write commands.",
				bank->bank_number,
				apm32x_is_otp_unlocked(bank) ? "enabled" : "disabled");
		} else {
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else {
		command_print(CMD, "Failed: not an OTP bank.");
	}

	return retval;
}

static const struct command_registration apm32f4x_exec_command_handlers[] = {
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
		.handler = apm32f4x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "options_write",
		.handler = apm32f4x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id user_options [ boot_add0 boot_add1 ]",
		.help = "Write option bytes",
	},
	{
		.name = "optcr2_write",
		.handler = apm32f4x_handle_optcr2_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id optcr2",
		.help = "Write optcr2 word",
	},
	{
		.name = "otp",
		.handler = apm32x_handle_otp_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "OTP (One Time Programmable) memory write enable/disable.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration apm32f4x_command_handlers[] = {
	{
		.name = "apm32f4x",
		.mode = COMMAND_ANY,
		.help = "apm32f4x flash command group",
		.usage = "",
		.chain = apm32f4x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver apm32f4x_flash = {
	.name = "apm32f4x",
	.commands = apm32f4x_command_handlers,
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
