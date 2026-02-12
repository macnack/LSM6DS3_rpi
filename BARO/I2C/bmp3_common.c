/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <string.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>

#include "bmp3.h"
#include "bmp3_common.h"

/*! BMP3 shuttle board ID */
#define BMP3_SHUTTLE_ID  0xD3

#define channel 0  
#define IIC_Dev  "/dev/i2c-1"
#define USEIIC 1

/* Variable to store the device address */
static uint8_t dev_addr;

int fd;

BMP3_INTF_RET_TYPE bmp3_user_i2c_init(void)
{

    /* Implement I2C bus initialization according to the target machine. */
    if ((fd = open(IIC_Dev, O_RDWR)) < 0) 
    {
        printf("Failed to open the i2c bus");
        return(1);
    }
    if (ioctl(fd, I2C_SLAVE, BMP3_ADDR_I2C_SEC) < 0) 
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        return(1);
    }
    return 0;
}

BMP3_INTF_RET_TYPE bmp3_user_spi_init(void)
{

    /* Implement SPI bus initialization according to the target machine. */
    if(wiringPiSetup() < 0)
    {
        return 1;
    }
    pinMode (27,OUTPUT);
    spi_bmp3_cs_low();//once pull down means use SPI Interface
    wiringPiSPISetup(channel,2000000);
    return 0;
}
void spi_bmp3_cs_high(void)
{
	digitalWrite(27,1);
}
void spi_bmp3_cs_low(void)
{
	digitalWrite(27,0);
}

/*!
 * I2C read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    
    /* Read from registers using I2C. Return 0 for a successful execution. */
    write(fd, &reg_addr,1);
    read(fd, reg_data, len);
    return 0;
}

/*!
 * I2C write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* Write to registers using I2C. Return 0 for a successful execution. */
    int8_t *buf;
    buf = malloc(len +1);
    buf[0] = reg_addr;
    memcpy(buf +1, reg_data, len);
    write(fd, buf, len +1);
    free(buf);
    return 0;
}

/*!
 * SPI read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* Read from registers using SPI. Return 0 for a successful execution. */
    int8_t rslt = 0;
	
	spi_bmp3_cs_high();
	spi_bmp3_cs_low();
	wiringPiSPIDataRW(channel,&reg_addr,1);
	wiringPiSPIDataRW(channel,reg_data,len);
	spi_bmp3_cs_high();
	
	return rslt;
}

/*!
 * SPI write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_user_spi_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* Write to registers using SPI. Return 0 for a successful execution. */
    int8_t rslt = 0;

	spi_bmp3_cs_high();
	spi_bmp3_cs_low();

	wiringPiSPIDataRW(channel,&reg_addr,1);
    wiringPiSPIDataRW(channel,reg_data,len);
    spi_bmp3_cs_high();
    return rslt;
}

/*!
 * Delay function map to COINES platform
 */
void bmp3_user_delay_us(uint32_t period, void *intf_ptr)
{
    /* Wait for a period amount of microseconds. */
    usleep(period);
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3)
{
    int8_t rslt = BMP3_OK;

    if(bmp3!=NULL)
    {
        if(USEIIC)
        {
            bmp3->intf = BMP3_I2C_INTF;
        }
        else
        {
            bmp3->intf = BMP3_SPI_INTF;
        }
        /* Bus configuration : I2C */
        if (bmp3->intf == BMP3_I2C_INTF)
        {
            printf("I2C Interface\n");
            bmp3_user_i2c_init();
            dev_addr = BMP3_ADDR_I2C_SEC;
            bmp3->read = bmp3_user_i2c_read;
            bmp3->write = bmp3_user_i2c_write;
        }
        /* Bus configuration : SPI */
        else if (bmp3->intf == BMP3_SPI_INTF)
        {
            printf("SPI Interface\n");
            bmp3_user_spi_init();
            dev_addr = 0;
            bmp3->read = bmp3_user_spi_read;
            bmp3->write = bmp3_user_spi_write;
        }

        bmp3->delay_us = bmp3_user_delay_us;
        bmp3->intf_ptr = &dev_addr;
    }
    else
    {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}

