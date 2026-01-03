// MIT License

// Copyright (c) 2025 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __ICM42688_H__
#define __ICM42688_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define ICM42688_I2C_ADDR_0			0x68
#define ICM42688_I2C_ADDR_1			0x69

typedef enum {
	ICM42688_STATUS_SUCCESS = 0,
	ICM42688_STATUS_FAILED,
	ICM42688_STATUS_INVALID_ARG
} icm42688_status_t;

typedef icm42688_status_t (*icm42688_func_i2c_send)(uint8_t reg_addr, uint8_t *buf_send, uint16_t len);
typedef icm42688_status_t (*icm42688_func_i2c_recv)(uint8_t reg_addr, uint8_t *buf_recv, uint16_t len);
typedef icm42688_status_t (*icm42688_func_spi_send)(uint8_t *buf_send, uint16_t len);
typedef icm42688_status_t (*icm42688_func_spi_recv)(uint8_t *buf_recv, uint16_t len);
typedef icm42688_status_t (*icm42688_func_set_gpio)(uint8_t level);
typedef void (*icm42688_func_delay)(uint32_t ms);

/**
 * @brief   Handle structure.
 */
typedef struct icm42688 *icm42688_handle_t;

/**
 * @brief   Gyro mode.
 */
typedef enum {
	ICM42688_GYRO_MODE_OFF = 0,
	ICM42688_GYRO_MODE_STANDBY = 1,
	ICM42688_GYRO_MODE_LOW_NOISE = 3
} icm42688_gyro_mode_t;

/**
 * @brief   Gyroscope full scale.
 */
typedef enum {
	ICM42688_GFS_SEL_2000dps = 0,
	ICM42688_GFS_SEL_1000dps = 1,
	ICM42688_GFS_SEL_500dps = 2,
	ICM42688_GFS_SEL_250dps = 3,
	ICM42688_GFS_SEL_125dps = 4,
	ICM42688_GFS_SEL_62_5dps = 5,
	ICM42688_GFS_SEL_31_25dps = 6,
	ICM42688_GFS_SEL_15_625dps = 7
} icm42688_gyro_fs_sel_t;

/**
 * @brief   Gyro oversample data rate.
 */
typedef enum {
	ICM42688_GYRO_ODR_32kHz = 1,
	ICM42688_GYRO_ODR_16kHz = 2,
	ICM42688_GYRO_ODR_8kHz = 3,
	ICM42688_GYRO_ODR_4kHz = 4,
	ICM42688_GYRO_ODR_2kHz = 5,
	ICM42688_GYRO_ODR_1kHz = 6,
	ICM42688_GYRO_ODR_200Hz = 7,
	ICM42688_GYRO_ODR_100Hz = 8,
	ICM42688_GYRO_ODR_50Hz = 9,
	ICM42688_GYRO_ODR_25Hz = 10,
	ICM42688_GYRO_ODR_12_5Hz = 11,
	ICM42688_GYRO_ODR_500Hz = 15
} icm42688_gyro_odr_t;

/**
 * @brief   Accel mode.
 */
typedef enum {
	ICM42688_ACCEL_MODE_OFF = 0,
	ICM42688_ACCEL_MODE_LOW_POWER = 2,
	ICM42688_ACCEL_MODE_LOW_NOISE = 3
} icm42688_accel_mode_t;

/**
 * @brief   Accel full scale.
 */
typedef enum {
	ICM42688_ACCEL_FS_SEL_16G = 0,
	ICM42688_ACCEL_FS_SEL_8G = 1,
	ICM42688_ACCEL_FS_SEL_4G = 2,
	ICM42688_ACCEL_FS_SEL_2G = 3
} icm42688_accel_fs_sel_t;

/**
 * @brief   Accel oversample data rate.
 */
typedef enum {
	ICM42688_ACCEL_ODR_32kHz = 1,
	ICM42688_ACCEL_ODR_16kHz = 2,
	ICM42688_ACCEL_ODR_8kHz = 3,
	ICM42688_ACCEL_ODR_4kHz = 4,
	ICM42688_ACCEL_ODR_2kHz = 5,
	ICM42688_ACCEL_ODR_1kHz = 6,
	ICM42688_ACCEL_ODR_200Hz = 7,
	ICM42688_ACCEL_ODR_100Hz = 8,
	ICM42688_ACCEL_ODR_50Hz = 9,
	ICM42688_ACCEL_ODR_25Hz = 10,
	ICM42688_ACCEL_ODR_12_5Hz = 11,
	ICM42688_ACCEL_ODR_6_25Hz = 12,
	ICM42688_ACCEL_ODR_3_125Hz = 13,
	ICM42688_ACCEL_ODR_1_5625Hz = 14,
	ICM42688_ACCEL_ODR_500Hz = 15
} icm42688_accel_odr_t;

/**
 * @brief   Communication mode.
 */
typedef enum {
	ICM42688_COMM_MODE_I2C = 0,
	ICM42688_COMM_MODE_SPI
} icm42688_comm_mode_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	icm42688_gyro_mode_t  			gyro_mode;					/*!< Gyro mode */
	icm42688_gyro_fs_sel_t  		gyro_fs_sel;				/*!< Gyro full scale */
	icm42688_gyro_odr_t  			gyro_odr;					/*!< Gyro oversample data rate */
	icm42688_accel_mode_t   		accel_mode;					/*!< Accel mode */
	icm42688_accel_odr_t   			accel_odr;					/*!< Accel oversample data rate */
	icm42688_accel_fs_sel_t   		accel_fs_sel;				/*!< Accel full scale */
	int16_t                     	accel_bias_x;               /*!< Accelerometer bias of x axis */
	int16_t                     	accel_bias_y;               /*!< Accelerometer bias of y axis */
	int16_t                     	accel_bias_z;               /*!< Accelerometer bias of z axis */
	int16_t                     	gyro_bias_x;                /*!< Gyroscope bias of x axis */
	int16_t                     	gyro_bias_y;                /*!< Gyroscope bias of y axis */
	int16_t                     	gyro_bias_z;                /*!< Gyroscope bias of z axis */
	icm42688_comm_mode_t   			comm_mode;					/*!< Comminication mode */
	icm42688_func_i2c_send       	i2c_send;        			/*!< Function I2C send */
	icm42688_func_i2c_recv       	i2c_recv;         			/*!< Function I2C receive */
	icm42688_func_spi_send  		spi_send;					/*!< Function SPI send */
	icm42688_func_spi_recv  		spi_recv;					/*!< Function SPI receive */
	icm42688_func_set_gpio   		set_cs;  					/*!< Function set pin CS */
	icm42688_func_delay          	delay;                 		/*!< Function delay function */
} icm42688_cfg_t;

/*
 * @brief   Initialize ICM42688 with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others: Failed.
 */
icm42688_handle_t icm42688_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_set_config(icm42688_handle_t handle, icm42688_cfg_t config);

/*
 * @brief   Configure ICM42688 to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_config(icm42688_handle_t handle);

/*
 * @brief   Reset ICM42688 by software.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_reset(icm42688_handle_t handle);

/*
 * @brief   Get accelerometer raw value.
 *
 * @param   handle Handle structure.
 * @param   raw_x Raw value x axis.
 * @param   raw_y Raw value y axis.
 * @param   raw_z Raw value z axis.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_get_accel_raw(icm42688_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/*
 * @brief   Get accelerometer calibrated data.
 *
 * @param   handle Handle structure.
 * @param   calib_x Calibrated data x axis.
 * @param   calib_y Calibrated data y axis.
 * @param   calib_z Calibrated data z axis.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_get_accel_calib(icm42688_handle_t handle, int16_t *calib_x, int16_t *calib_y, int16_t *calib_z);

/*
 * @brief   Get accelerometer scaled data.
 *
 * @param   handle Handle structure.
 * @param   scale_x Scaled data x axis.
 * @param   scale_y Scaled data y axis.
 * @param   scale_z Scaled data z axis.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_get_accel_scale(icm42688_handle_t handle, float *scale_x, float *scale_y, float *scale_z);

/*
 * @brief   Get gyroscope raw value.
 *
 * @param   handle Handle structure.
 * @param   raw_x Raw value x axis.
 * @param   raw_y Raw value y axis.
 * @param   raw_z Raw value z axis.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_get_gyro_raw(icm42688_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/*
 * @brief   Get gyroscope calibrated data.
 *
 * @param   handle Handle structure.
 * @param   calib_x Calibrated data x axis.
 * @param   calib_y Calibrated data y axis.
 * @param   calib_z Calibrated data z axis.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_get_gyro_calib(icm42688_handle_t handle, int16_t *calib_x, int16_t *calib_y, int16_t *calib_z);

/*
 * @brief   Get gyroscope scaled data.
 *
 * @param   handle Handle structure.
 * @param   scale_x Scaled data x axis.
 * @param   scale_y Scaled data y axis.
 * @param   scale_z Scaled data z axis.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_get_gyro_scale(icm42688_handle_t handle, float *scale_x, float *scale_y, float *scale_z);

/*
 * @brief   Auto calibrate all acceleromter and gyroscope bias value.
 *
 * @param   handle Handle structure.
 * @param   reverse_z Set to 1 to reverse the Z axis direction.
 *
 * @return
 *      - ICM42688_STATUS_SUCCESS: Success.
 *      - Others: Failed.
 */
icm42688_status_t icm42688_auto_calib(icm42688_handle_t handle, uint8_t reverse_z);


#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H__ */
