/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BMP3_COMMON_H
#define _BMP3_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdio.h>
#include "bmp3.h"

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bmp3      : Structure instance of bmp3_dev
 *  @param[in] intf     : Interface selection parameter
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */

BMP3_INTF_RET_TYPE bmp3_user_i2c_init(void);
BMP3_INTF_RET_TYPE bmp3_user_spi_init(void);
void spi_bmp3_cs_high(void);
void spi_bmp3_cs_low(void);

BMP3_INTF_RET_TYPE bmp3_user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] len          : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMP3_INTF_RET_TYPE bmp3_user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMP3_INTF_RET_TYPE bmp3_user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] len          : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMP3_INTF_RET_TYPE bmp3_user_spi_write(uint8_t reg_addr,uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period       : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void bmp3_user_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmp3_check_rslt(const char api_name[], int8_t rslt);

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */

#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif