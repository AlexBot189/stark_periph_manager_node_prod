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

/** @defgroup WEARABLE EDMP Wearable
 *  @brief High-level functions to drive eDMP Wearable features
 *  @{
 */

/** @file inv_imu_edmp_wearable.h */

#ifndef _INV_IMU_WEARABLE_H_
#define _INV_IMU_WEARABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

/** @brief IMU B2S parameters definition
 */
typedef struct {
	/* B2S */
	uint32_t b2s_mounting_matrix; /**< Specifies mounting matrix to be applied to B2S raw data
                                       Set bit 2 : swap X/Y ; flip Z
                                       Set bit 1 : flip X ; flip Z
                                       Set bit 0 : flip Y ; flip Z */
	struct inv_imu_edmp_b2s_algo_parameters_s {
		int32_t
		         dev_norm_max; /**< Hysteresis added or removed to norm estimate and Y axis constrains value for RevB2S

                                             Unit in LSB, with 1 LBS = 1g / 2^12
                                             Default value 700 (corresponding to 0.1709g)
                                             Recommended range value [1 - 2048] */
		int32_t  sin_limit; /**< Maximum threshold on absolute value of X axis in b2s position
                                             link to the sinus value of inclination angle on X axis
                                             Unit in LSB, with 1 LBS = 1g / 2^12
                                             Default value 2048 (corresponding to 30deg)
                                             Recommended range value [300 - 3000] */
		int32_t  fast_limit; /**< Threshold of minimal motion to be detected as "Fast motion"
                                             Unit is no unit, filtered data
                                             Default value 200 */
		uint16_t fast_motion_age_limit; /**< Time limit between last "Fast motion" and b2s position
                                             Unit in sample number - ODR dependent
                                             Default value 20 (corresponding to 400ms at 50Hz) */
		uint16_t
		         fast_motion_time_limit; /**< Minimum time where the criterion is above the threshold  to be classified as "Fast motion"

                                             Unit in sample number - ODR dependent
                                             Default value 4 (corresponding to 80ms at 50Hz) */
		uint16_t age_limit; /**< Minimum time between 2 event b2s
                                             Unit in sample number - ODR dependent
                                             Default value 50 (corresponding to 1s at 50Hz) */
		uint16_t
		    rev_latency_th; /**< Condition of RevB2S should be maintained at least during RevB2sLatencyTh

                                             Unit in sample number - ODR dependent
                                             Default value 25 (corresponding to 0.5s at 50Hz) */
		int32_t
		    static_limit; /**< Threshold to determine static phase required after the gesture B2S to validate it

                                             Unit is no unit, filtered data
                                             Default value 1400 */
		int32_t
		    thr_cos_ang; /**< RevB2S threshold, condition satisfied when moving away from bring2see orientation position

                                             by more than threshold angle corresponding to the cosine angle
                                             Unit is cosine value of angle in q30
                                             Default value 1057429273 corresponding to 10deg */
		int32_t
		    rev_x_limit; /**< Condition of RevB2S on X axis value to ensure Rev-B2S when arm point down

                                             * Unit in LSB, with 1 LBS = 1g / 2^12
                                             * Default value 3680 (corresponding to 0.8984g)
                                             */
		int32_t LimitInf;
		int32_t LimitSup;
		uint32_t
		    sin_flat_angle; /**< Condition to detect the flat position and reject B2S interrupt at return to rest position

                                             Unit in LSB, with 1 LBS = 1g / 2^12
                                             Default value 2048 (corresponding to 30°) */
		uint32_t
		    timer_flat_reject; /**< A timer to disable flat rejection when age of the last no-flat B2S detection is over the timer

                                             Unit in sample number - ODR dependent
                                             Default value 350 (corresponding to 7s at 50Hz) */
	} b2s_algo_params;
} inv_imu_edmp_b2s_parameters_t;

/** @brief System usecase definition for IMU EDMP B2S initialization */
typedef enum {
	INV_IMU_EDMP_B2S_INIT_OVER_SIF,
	INV_IMU_EDMP_B2S_INIT_OVER_GAF
} inv_imu_edmp_b2s_init_t;

/** @brief  Initialize B2S algorithm for usecase where no SIF is needed. 
 *  @param[in]  s        Pointer to device.
 *  @param[out] usecase  System usecase for which B2S image was initialized.
 *  @return     0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_init_over_sif(inv_imu_device_t *s, inv_imu_edmp_b2s_init_t *usecase);

/** @brief  Initialize B2S algorithm for usecase where no GAF is needed. 
 *  @param[in]  s        Pointer to device.
 *  @param[out] usecase  System usecase for which B2S image was initialized.
 *  @return     0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_init_over_gaf(inv_imu_device_t *s, inv_imu_edmp_b2s_init_t *usecase);

/** @brief  Get current B2S configuration settings. 
 *  @param[in]  s           Pointer to device.
 *  @param[out] b2s_params  Pointer to B2S configuration structure, which will hold current B2S configuration.
 *  @param[in]  usecase     System usecase for which B2S image was initialized.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_get_parameters(inv_imu_device_t *s, inv_imu_edmp_b2s_parameters_t *b2s_params,
                                    inv_imu_edmp_b2s_init_t usecase);

/** @brief  Set new B2S configuration settings. 
 *  @param[in]  s           Pointer to device.
 *  @param[in]  b2s_params  Pointer to B2S configuration structure, which contains new B2S configuration.
 *  @param[in]  usecase     System usecase for which B2S image was initialized.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_b2s_parameters_t *b2s_params,
                                    inv_imu_edmp_b2s_init_t              usecase);

/** @brief  Enable APEX algorithm B2S. 
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_enable(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm B2S.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_disable(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_WEARABLE_H_ */

/** @} */
