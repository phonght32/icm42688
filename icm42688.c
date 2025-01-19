#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "icm42688.h"


/**
 * @brief Registers bank 0
 */
#define ICM42688_REG_BANK0_DEVICE_CONFIG  			0x11
#define ICM42688_REG_BANK0_DRIVE_CONFIG  			0x13
#define ICM42688_REG_BANK0_INT_CONFIG      			0x14
#define ICM42688_REG_BANK0_FIFO_CONFIG    			0x16
#define ICM42688_REG_BANK0_TEMP_DATA1       		0x1D
#define ICM42688_REG_BANK0_TEMP_DATA0            	0x1E
#define ICM42688_REG_BANK0_ACCEL_DATA_X1         	0x1F
#define ICM42688_REG_BANK0_ACCEL_DATA_X0           	0x20
#define ICM42688_REG_BANK0_ACCEL_DATA_Y1           	0x21
#define ICM42688_REG_BANK0_ACCEL_DATA_Y0           	0x22
#define ICM42688_REG_BANK0_ACCEL_DATA_Z1           	0x23
#define ICM42688_REG_BANK0_ACCEL_DATA_Z0           	0x24
#define ICM42688_REG_BANK0_GYRO_DATA_X1            	0x25
#define ICM42688_REG_BANK0_GYRO_DATA_X0            	0x26
#define ICM42688_REG_BANK0_GYRO_DATA_Y1            	0x27
#define ICM42688_REG_BANK0_GYRO_DATA_Y0            	0x28
#define ICM42688_REG_BANK0_GYRO_DATA_Z1            	0x29
#define ICM42688_REG_BANK0_GYRO_DATA_Z0            	0x2A
#define ICM42688_REG_BANK0_TMST_FSYNCH             	0x2B
#define ICM42688_REG_BANK0_TMST_FSYNCL             	0x2C
#define ICM42688_REG_BANK0_INT_STATUS              	0x2D
#define ICM42688_REG_BANK0_FIFO_COUNTH             	0x2E
#define ICM42688_REG_BANK0_FIFO_COUNTL             	0x2F
#define ICM42688_REG_BANK0_FIFO_DATA               	0x30
#define ICM42688_REG_BANK0_APEX_DATA0              	0x31
#define ICM42688_REG_BANK0_APEX_DATA1              	0x32
#define ICM42688_REG_BANK0_APEX_DATA2              	0x33
#define ICM42688_REG_BANK0_APEX_DATA3              	0x34
#define ICM42688_REG_BANK0_APEX_DATA4              	0x35
#define ICM42688_REG_BANK0_APEX_DATA5              	0x36
#define ICM42688_REG_BANK0_INT_STATUS2             	0x37
#define ICM42688_REG_BANK0_INT_STATUS3             	0x38
#define ICM42688_REG_BANK0_SIGNAL_PATH_RESET   		0x4B
#define ICM42688_REG_BANK0_INTF_CONFIG0        		0x4C
#define ICM42688_REG_BANK0_INTF_CONFIG1        		0x4D
#define ICM42688_REG_BANK0_PWR_MGMT0           		0x4E
#define ICM42688_REG_BANK0_GYRO_CONFIG0        		0x4F
#define ICM42688_REG_BANK0_ACCEL_CONFIG0       		0x50
#define ICM42688_REG_BANK0_GYRO_CONFIG1        		0x51
#define ICM42688_REG_BANK0_GYRO_ACCEL_CONFIG0  		0x52
#define ICM42688_REG_BANK0_ACCEFL_CONFIG1      		0x53
#define ICM42688_REG_BANK0_TMST_CONFIG         		0x54
#define ICM42688_REG_BANK0_APEX_CONFIG0           	0x56
#define ICM42688_REG_BANK0_SMD_CONFIG             	0x57
#define ICM42688_REG_BANK0_FIFO_CONFIG1           	0x5F
#define ICM42688_REG_BANK0_FIFO_CONFIG2           	0x60
#define ICM42688_REG_BANK0_FIFO_CONFIG3           	0x61
#define ICM42688_REG_BANK0_FSYNC_CONFIG           	0x62
#define ICM42688_REG_BANK0_INT_CONFIG0            	0x63
#define ICM42688_REG_BANK0_INT_CONFIG1            	0x64
#define ICM42688_REG_BANK0_INT_SOURCE0            	0x65
#define ICM42688_REG_BANK0_INT_SOURCE1            	0x66
#define ICM42688_REG_BANK0_INT_SOURCE3           	0x68
#define ICM42688_REG_BANK0_INT_SOURCE4           	0x69
#define ICM42688_REG_BANK0_FIFO_LOST_PKT0           0x6C
#define ICM42688_REG_BANK0_FIFO_LOST_PKT1  			0x6D
#define ICM42688_REG_BANK0_SELF_TEST_CONFIG  		0x70
#define ICM42688_REG_BANK0_WHO_AM_I           		0x75

/**
 * @brief Registers bank 1
 */
#define ICM42688_REG_BANK1_SENSOR_CONFIG0  			0x03
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC2   	0x0B
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC3   	0x0C
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC4   	0x0D
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC5   	0x0E
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC6   	0x0F
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC7  	0x10
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC8   	0x11
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC9   	0x12
#define ICM42688_REG_BANK1_GYRO_CONFIG_STATIC10  	0x13
#define ICM42688_REG_BANK1_XG_ST_DATA  				0x5F
#define ICM42688_REG_BANK1_YG_ST_DATA           	0x60
#define ICM42688_REG_BANK1_ZG_ST_DATA           	0x61
#define ICM42688_REG_BANK1_TMSTVAL0             	0x62
#define ICM42688_REG_BANK1_TMSTVAL1             	0x63
#define ICM42688_REG_BANK1_TMSTVAL2             	0x64
#define ICM42688_REG_BANK1_INTF_CONFIG4           	0x7A
#define ICM42688_REG_BANK1_INTF_CONFIG5           	0x7B
#define ICM42688_REG_BANK1_INTF_CONFIG6           	0x7C

/**
 * @brief Registers bank 2
 */
#define ICM42688_REG_BANK2_ACCEL_CONFIG_STATIC2  	0x03
#define ICM42688_REG_BANK2_ACCEL_CONFIG_STATIC3  	0x04
#define ICM42688_REG_BANK2_ACCEL_CONFIG_STATIC4  	0x05
#define ICM42688_REG_BANK2_XA_ST_DATA           	0x3B
#define ICM42688_REG_BANK2_YA_ST_DATA           	0x3C
#define ICM42688_REG_BANK2_ZA_ST_DATA           	0x3D

/**
 * @brief Registers bank 4
 */
#define ICM42688_REG_BANK4_APEX_CONFIG1           	0x40
#define ICM42688_REG_BANK4_APEX_CONFIG2           	0x41
#define ICM42688_REG_BANK4_APEX_CONFIG3           	0x42
#define ICM42688_REG_BANK4_APEX_CONFIG4           	0x43
#define ICM42688_REG_BANK4_APEX_CONFIG5           	0x44
#define ICM42688_REG_BANK4_APEX_CONFIG6           	0x45
#define ICM42688_REG_BANK4_APEX_CONFIG7           	0x46
#define ICM42688_REG_BANK4_APEX_CONFIG8           	0x47
#define ICM42688_REG_BANK4_APEX_CONFIG9           	0x48
#define ICM42688_REG_BANK4_ACCEL_WOM_X_THR  		0x4A
#define ICM42688_REG_BANK4_ACCEL_WOM_Y_THR  		0x4B
#define ICM42688_REG_BANK4_ACCEL_WOM_Z_THR  		0x4C
#define ICM42688_REG_BANK4_INT_SOURCE6      		0x4D
#define ICM42688_REG_BANK4_INT_SOURCE7      		0x4E
#define ICM42688_REG_BANK4_INT_SOURCE8      		0x4F
#define ICM42688_REG_BANK4_INT_SOURCE9      		0x50
#define ICM42688_REG_BANK4_INT_SOURCE10     		0x51
#define ICM42688_REG_BANK4_OFFSET_USER0           	0x77
#define ICM42688_REG_BANK4_OFFSET_USER1           	0x78
#define ICM42688_REG_BANK4_OFFSET_USER2           	0x79
#define ICM42688_REG_BANK4_OFFSET_USER3           	0x7A
#define ICM42688_REG_BANK4_OFFSET_USER4           	0x7B
#define ICM42688_REG_BANK4_OFFSET_USER5           	0x7C
#define ICM42688_REG_BANK4_OFFSET_USER6           	0x7D
#define ICM42688_REG_BANK4_OFFSET_USER7           	0x7E
#define ICM42688_REG_BANK4_OFFSET_USER8           	0x7F

/**
 * @brief Accesible from all user banks
 */
#define ICM42688_REG_BANK_SEL 						0x76



#define ICM42688_CS_ACTIVE 							0
#define ICM42688_CS_UNACTIVE 						1

typedef struct icm42688 {
	icm42688_gyro_mode_t  			gyro_mode;					/*!< Gyro mode */
	icm42688_gyro_fs_sel_t  		gyro_fs_sel;				/*!< Gyro full scale */
	icm42688_gyro_odr_t  			gyro_odr;					/*!< Gyro oversample data rate */
	icm42688_accel_mode_t   		accel_mode;					/*!< Accel mode */
	icm42688_accel_odr_t   			accel_odr;					/*!< Accel oversample data rate */
	icm42688_accel_fs_sel_t   		accel_fs_sel;				/*!< Accel full scale */
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

	handle->gyro_mode 					= config.gyro_mode;
	handle->gyro_fs_sel 				= config.gyro_fs_sel;
	handle->gyro_odr 					= config.gyro_odr;
	handle->accel_mode 					= config.accel_mode;
	handle->accel_fs_sel 				= config.accel_fs_sel;
	handle->accel_odr 					= config.accel_odr;
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

	icm42688_set_bank(handle, 0);

	uint8_t who_am_i;
	uint8_t config_data = 0;
	icm42688_recv(handle, ICM42688_REG_BANK0_WHO_AM_I, &who_am_i, 1);

	if (who_am_i != 0x47)
	{
		return ERR_CODE_FAIL;
	}

	/* Configure accel mode and gyro mode */
	config_data = (handle->gyro_mode << 2) | handle->accel_mode;
	icm42688_send(handle, ICM42688_REG_BANK0_PWR_MGMT0, &config_data, 1);
	handle->delay(1);

	/* Configure accel ODR and FS */
	config_data = (handle->accel_fs_sel << 5) | handle->accel_odr;
	icm42688_send(handle, ICM42688_REG_BANK0_ACCEL_CONFIG0, &config_data, 1);

	/* Configure gyro ODR and FS */
	config_data = (handle->gyro_fs_sel << 5) | handle->gyro_odr;
	icm42688_send(handle, ICM42688_REG_BANK0_GYRO_CONFIG0, &config_data, 1);
  
  	/* set gyro and accel bandwidth to ODR/10 */
  	config_data = 0x44;
  	icm42688_send(handle, ICM42688_REG_BANK0_GYRO_ACCEL_CONFIG0, &config_data, 1);

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
	icm42688_send(handle, ICM42688_REG_BANK0_DEVICE_CONFIG, &buf_send, 1);

	if (handle->delay != NULL)
	{
		handle->delay(100);
	}

	return ERR_CODE_SUCCESS;
}
