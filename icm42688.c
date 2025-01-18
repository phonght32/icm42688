#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "icm42688.h"

/**
 * @brief chip register definition
 */


typedef struct icm42688 {
	icm42688_comm_mode_t   			comm_mode;					/*!< Comminication mode */
	icm42688_func_i2c_send       	i2c_send;        			/*!< Function I2C send */
	icm42688_func_i2c_recv       	i2c_recv;         			/*!< Function I2C receive */
	icm42688_func_spi_send  		spi_send;					/*!< Function SPI send */
	icm42688_func_spi_recv  		spi_recv;					/*!< Function SPI receive */
	icm42688_func_set_gpio   		set_cs;  					/*!< Function set pin CS */
	icm42688_func_delay          	delay;                 		/*!< Function delay function */
} icm42688_t;

icm42688_handle_t icm42688_init(void)
{
	icm42688_handle_t handle = calloc(1, sizeof(icm42688_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

err_code_t icm42688_set_config(icm42688_handle_t handle, icm42688_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->comm_mode 					= config.comm_mode;
	handle->i2c_send 					= config.i2c_send;
	handle->i2c_recv 					= config.i2c_recv;
	handle->spi_send 					= config.spi_send;
	handle->spi_recv 					= config.spi_recv;
	handle->set_cs 						= config.set_cs;
	handle->delay 						= config.delay;

	return ERR_CODE_SUCCESS;
}

err_code_t icm42688_config(icm42688_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}


	return ERR_CODE_SUCCESS;
}