#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "icm42688.h"

/**
 * @brief chip register definition
 */
#define ICM42688_REG_DEVICE_CONFIG 		0x11
#define ICM42688_REG_WHO_AM_I 			0x75
#define ICM42688_REG_BANK_SEL 			0x76



#define ICM42688_CS_ACTIVE 			0
#define ICM42688_CS_UNACTIVE 		1

typedef struct icm42688 {
	icm42688_comm_mode_t   			comm_mode;					/*!< Comminication mode */
	icm42688_func_i2c_send       	i2c_send;        			/*!< Function I2C send */
	icm42688_func_i2c_recv       	i2c_recv;         			/*!< Function I2C receive */
	icm42688_func_spi_send  		spi_send;					/*!< Function SPI send */
	icm42688_func_spi_recv  		spi_recv;					/*!< Function SPI receive */
	icm42688_func_set_gpio   		set_cs;  					/*!< Function set pin CS */
	icm42688_func_delay          	delay;                 		/*!< Function delay function */
	uint8_t 						bank;
} icm42688_t;

static err_code_t icm42688_send(icm42688_handle_t handle, uint8_t reg_addr, uint8_t *buf_send, uint16_t len)
{
	if (handle->comm_mode == ICM42688_COMM_MODE_I2C)
	{
		handle->i2c_send(reg_addr, buf_send, len);
	}
	else
	{
		if (handle->set_cs != NULL)
		{
			handle->set_cs(ICM42688_CS_ACTIVE);
		}

		uint8_t buf[len + 1];
		buf[0] = reg_addr;
		memcpy(&buf[1], buf_send, len);

		handle->spi_send(buf, len + 1);

		if (handle->set_cs != NULL)
		{
			handle->set_cs(ICM42688_CS_UNACTIVE);
		}
	}

	return ERR_CODE_SUCCESS;
}

static err_code_t icm42688_recv(icm42688_handle_t handle, uint8_t reg_addr, uint8_t *buf_recv, uint16_t len)
{
	if (handle->comm_mode == ICM42688_COMM_MODE_I2C)
	{
		handle->i2c_send(reg_addr, buf_recv, len);
	}
	else
	{
		if (handle->set_cs != NULL)
		{
			handle->set_cs(ICM42688_CS_ACTIVE);
		}

		uint8_t buf = reg_addr | 0x80;

		handle->spi_send(&buf, 1);
		handle->spi_recv(buf_recv, len);

		if (handle->set_cs != NULL)
		{
			handle->set_cs(ICM42688_CS_UNACTIVE);
		}
	}

	return ERR_CODE_SUCCESS;
}

static err_code_t icm42688_set_bank(icm42688_handle_t handle, uint8_t bank)
{
	if (handle->bank == bank)
	{
		return ERR_CODE_SUCCESS;
	}

	uint8_t buf_send = bank;

	icm42688_send(handle, ICM42688_REG_BANK_SEL, &buf_send, 1);

	handle->bank = bank;

	return ERR_CODE_SUCCESS;
}

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

	icm42688_reset(handle);

	uint8_t who_am_i;
	icm42688_recv(handle, ICM42688_REG_WHO_AM_I, &who_am_i, 1);

	if (who_am_i != 0x47)
	{
		return ERR_CODE_FAIL;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t icm42688_reset(icm42688_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	icm42688_set_bank(handle, 0);

	uint8_t buf_send = 0x01;
	icm42688_send(handle, ICM42688_REG_DEVICE_CONFIG, &buf_send, 1);

	if (handle->delay != NULL)
	{
		handle->delay(100);
	}

	return ERR_CODE_SUCCESS;
}
