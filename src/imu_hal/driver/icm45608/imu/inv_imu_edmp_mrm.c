/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "imu/inv_imu_edmp_mrm.h"
#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_defs.h"
#include "imu/inv_imu_edmp_patches_defs.h"

#define GAF_MODE_BITMASK_MRM_ENABLE 0x4

int inv_imu_edmp_mrm_init(inv_imu_device_t *s, inv_imu_edmp_mrm_init_t usecase)
{
	int            status    = INV_IMU_OK;
	static uint8_t ram_img[] = {
#include "imu/edmp_ram_mrm_image.h"
	};
	uint32_t ram_mrm_img_start_addr;

	(void)usecase;

	memcpy(&ram_mrm_img_start_addr, &ram_img[RAM_MRM_IMG_PRGM_BASE - RAM_MRM_IMG_DATA_BASE],
	       sizeof(ram_mrm_img_start_addr));
	status |= inv_imu_write_sram(s, RAM_MRM_IMG_DATA_BASE, sizeof(ram_img), ram_img);
	status |= inv_imu_write_sram(s, EDMP_INVN_ALGO_MRM_PATCH_POINT_DISPATCH, 4,
	                             (uint8_t *)&ram_mrm_img_start_addr);

	return status;
}

int inv_imu_edmp_mrm_enable_auto(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	s->edmp_gaf_mode |= GAF_MODE_BITMASK_MRM_ENABLE;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_MODE, &s->edmp_gaf_mode);

	return status;
}

int inv_imu_edmp_mrm_disable_auto(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	s->edmp_gaf_mode &= ~GAF_MODE_BITMASK_MRM_ENABLE;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_MODE, &s->edmp_gaf_mode);

	return status;
}

int inv_imu_edmp_mrm_request(inv_imu_device_t *s)
{
	uint8_t cmd = 1;
	return INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_ONDEMAND_MRM_REQUEST, (uint8_t *)&cmd);
}
