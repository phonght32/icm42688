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

#include "err_code.h"

#define ICM42688_I2C_ADDR_0			0x68
#define ICM42688_I2C_ADDR_1			0x69

typedef err_code_t (*icm42688_func_i2c_send)(uint8_t reg_addr, uint8_t *buf_send, uint16_t len);
typedef err_code_t (*icm42688_func_i2c_recv)(uint8_t reg_addr, uint8_t *buf_recv, uint16_t len);
typedef err_code_t (*icm42688_func_spi_send)(uint8_t *buf_send, uint16_t len);
typedef err_code_t (*icm42688_func_spi_recv)(uint8_t *buf_recv, uint16_t len);
typedef err_code_t (*icm42688_func_set_gpio)(uint8_t level);
typedef void (*icm42688_func_delay)(uint32_t ms);

/**
 * @brief   Handle structure.
 */
typedef struct icm42688 *icm42688_handle_t;

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
 *      - Others:           Fail.
 */
icm42688_handle_t icm42688_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t icm42688_set_config(icm42688_handle_t handle, icm42688_cfg_t config);

/*
 * @brief   Configure ICM42688 to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t icm42688_config(icm42688_handle_t handle);

/*
 * @brief   Reset ICM42688 by software.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t icm42688_reset(icm42688_handle_t handle);


#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H__ */