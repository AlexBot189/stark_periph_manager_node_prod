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

#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_wearable.h"
#include "imu/inv_imu_edmp_defs.h"
#include "imu/edmp_data_ram_b2s_map.h"
#include "imu/inv_imu_edmp_patches_defs.h"

int inv_imu_edmp_b2s_init_over_sif(inv_imu_device_t *s, inv_imu_edmp_b2s_init_t *usecase)
{
	int status = INV_IMU_OK;

	/* Load B2S RAM image over SIF data area, part of APEX set of default features */
	uint32_t patch_key = (uint32_t)0x000070f1 | ((uint32_t)(RAM_B2S_IMG_PRGM_BASE & 0xFF00) << 8) |
	                     ((uint32_t)(RAM_B2S_IMG_PRGM_BASE & 0x00FF)
	                      << 24); // equivalent to jmp al,RAM_B2S_IMG_PRGM_BASE;
	static uint8_t ram_img[] = {
#include "imu/edmp_ram_b2s_image.h"
	};

	status |= inv_imu_edmp_init(s);
	status |= inv_imu_write_sram(s, RAM_B2S_IMG_DATA_BASE, sizeof(ram_img), ram_img);
	status |= inv_imu_write_sram(s, EDMP_RAM_FEATURE_PRGM_RAM_BASE, 4, (uint8_t *)&patch_key);

	*usecase = INV_IMU_EDMP_B2S_INIT_OVER_SIF;

	return status;
}

int inv_imu_edmp_b2s_init_over_gaf(inv_imu_device_t *s, inv_imu_edmp_b2s_init_t *usecase)
{
	int status = INV_IMU_OK;

	status |= inv_imu_edmp_init(s);

	/* Load B2S RAM image over GAF data area, part of APEX set of default features */
	uint32_t patch_key = (uint32_t)0x000070f1 |
	                     ((uint32_t)(RAM_B2S_OVER_GAF_IMG_PRGM_BASE & 0xFF00) << 8) |
	                     ((uint32_t)(RAM_B2S_OVER_GAF_IMG_PRGM_BASE & 0x00FF)
	                      << 24); // equivalent to jmp al,RAM_B2S_OVER_GAF_IMG_PRGM_BASE;
	static uint8_t ram_img[] = {
#include "imu/edmp_ram_b2s_over_gaf_image.h"
	};
	status |= inv_imu_write_sram(s, RAM_B2S_OVER_GAF_IMG_DATA_BASE, sizeof(ram_img), ram_img);
	status |= inv_imu_write_sram(s, EDMP_RAM_FEATURE_PRGM_RAM_BASE, 4, (uint8_t *)&patch_key);

	*usecase = INV_IMU_EDMP_B2S_INIT_OVER_GAF;

	return status;
}

int inv_imu_edmp_b2s_get_parameters(inv_imu_device_t *s, inv_imu_edmp_b2s_parameters_t *b2s_params,
                                    inv_imu_edmp_b2s_init_t usecase)
{
	int                                       status = INV_IMU_OK;
	uint8_t                                   b2s_edmp_mmatrix;
	struct inv_imu_edmp_b2s_algo_parameters_s b2s_edmp_params = { 0 };
	int16_t                                   ram_offset;

	if (usecase == INV_IMU_EDMP_B2S_INIT_OVER_SIF)
		ram_offset = 0;
	else
		ram_offset = RAM_B2S_OVER_GAF_IMG_DATA_BASE - RAM_B2S_IMG_DATA_BASE;

	status |= inv_imu_read_sram(s, EDMP_ram_b2s_mounting_matrix + ram_offset,
	                            EDMP_ram_b2s_mounting_matrix_size, &b2s_edmp_mmatrix);
	b2s_params->b2s_mounting_matrix = b2s_edmp_mmatrix;
	status |= inv_imu_read_sram(s, EDMP_ram_b2s_setting_Bts + ram_offset,
	                            EDMP_ram_b2s_setting_Bts_size, (uint8_t *)&b2s_edmp_params);
	/* Only extract the exposed parameters from edmp data */
	(void)memcpy(&b2s_params->b2s_algo_params, &b2s_edmp_params,
	             sizeof(b2s_params->b2s_algo_params));

	return status;
}

int inv_imu_edmp_b2s_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_b2s_parameters_t *b2s_params,
                                    inv_imu_edmp_b2s_init_t              usecase)
{
	int                                       status = INV_IMU_OK;
	int32_t                                   one_g_value;
	uint8_t                                   b2s_edmp_mmatrix;
	struct inv_imu_edmp_b2s_algo_parameters_s b2s_edmp_params = { 0 };
	int16_t                                   ram_offset;

	if (usecase == INV_IMU_EDMP_B2S_INIT_OVER_SIF)
		ram_offset = 0;
	else
		ram_offset = RAM_B2S_OVER_GAF_IMG_DATA_BASE - RAM_B2S_IMG_DATA_BASE;

	/* Get internal parameter from edmp */
	status |= inv_imu_read_sram(s, EDMP_ram_b2s_setting_lOneGValue + ram_offset,
	                            EDMP_ram_b2s_setting_lOneGValue_size, (uint8_t *)&one_g_value);

	/* Copy exposed parameter into internal structure */
	(void)memcpy(&b2s_edmp_params, &b2s_params->b2s_algo_params,
	             sizeof(b2s_params->b2s_algo_params));

	/* Derive internal parameters */
	b2s_edmp_params.LimitInf =
	    (one_g_value - b2s_edmp_params.dev_norm_max) * (one_g_value - b2s_edmp_params.dev_norm_max);
	b2s_edmp_params.LimitSup =
	    (one_g_value + b2s_edmp_params.dev_norm_max) * (one_g_value + b2s_edmp_params.dev_norm_max);

	/* Update all b2s parameters */
	status |= inv_imu_write_sram(s, EDMP_ram_b2s_setting_Bts + ram_offset,
	                             EDMP_ram_b2s_setting_Bts_size, (uint8_t *)&b2s_edmp_params);
	/* Adjust mounting matrix */
	b2s_edmp_mmatrix = (uint8_t)b2s_params->b2s_mounting_matrix;
	status |= inv_imu_write_sram(s, EDMP_ram_b2s_mounting_matrix + ram_offset,
	                             EDMP_ram_b2s_mounting_matrix_size, &b2s_edmp_mmatrix);

	return status;
}

int inv_imu_edmp_b2s_enable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	fifo_config0_t  fifo_config0;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_edmp_check_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&fifo_config0);
	if (fifo_config0.fifo_depth > (uint8_t)FIFO_CONFIG0_FIFO_DEPTH_APEX)
		return INV_IMU_ERROR;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_b2s_disable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}
