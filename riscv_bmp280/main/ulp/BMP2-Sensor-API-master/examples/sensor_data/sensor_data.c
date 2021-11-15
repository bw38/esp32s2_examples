/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmp2.h"
#include "common.h"

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to get compensated pressure and temperature data.
 *
 *  @param[in] period   : Contains the delay in microseconds
 *  @param[in] conf     : Structure instance of bmp2_config.
 *  @param[in] dev      : Structure instance of bmp2_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    int8_t rslt;
    uint32_t meas_time;
    struct bmp2_dev dev;
    struct bmp2_config conf;

    /* Interface selection is to be updated as parameter
     * For I2C :  BMP2_I2C_INTF
     * For SPI :  BMP2_SPI_INTF
     */
    rslt = bmp2_interface_selection(&dev, BMP2_I2C_INTF);
    bmp2_error_codes_print_result("bmp2_interface_selection", rslt);

    rslt = bmp2_init(&dev);
    bmp2_error_codes_print_result("bmp2_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bmp2_get_config(&conf, &dev);
    bmp2_error_codes_print_result("bmp2_get_config", rslt);

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP2_FILTER_OFF;

    /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
    conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

    /* Setting the output data rate */
    conf.odr = BMP2_ODR_250_MS;

    rslt = bmp2_set_config(&conf, &dev);
    bmp2_error_codes_print_result("bmp2_set_config", rslt);

    /* Set normal power mode */
    rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &conf, &dev);
    bmp2_error_codes_print_result("bmp2_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bmp2_compute_meas_time(&meas_time, &conf, &dev);
    bmp2_error_codes_print_result("bmp2_compute_meas_time", rslt);

    /* Read pressure and temperature data */
    rslt = get_data(meas_time, &conf, &dev);
    bmp2_error_codes_print_result("get_data", rslt);

    bmp2_coines_deinit();

    return 0;
}

/*!
 *  @brief This internal API is used to get compensated pressure and temperature data.
 */
static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_E_NULL_PTR;
    int8_t idx = 1;
    struct bmp2_status status;
    struct bmp2_data comp_data;

    printf("Measurement delay : %lu us\n", (long unsigned int)period);

    while (idx <= 50)
    {
        rslt = bmp2_get_status(&status, dev);
        bmp2_error_codes_print_result("bmp2_get_status", rslt);

        if (status.measuring == BMP2_MEAS_DONE)
        {
            /* Delay between measurements */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bmp2_get_sensor_data(&comp_data, dev);
            bmp2_error_codes_print_result("bmp2_get_sensor_data", rslt);

            #ifdef BMP2_64BIT_COMPENSATION
            comp_data.pressure = comp_data.pressure / 256;
            #endif

            #ifdef BMP2_DOUBLE_COMPENSATION
            printf("Data[%d]:    Temperature: %.4lf deg C	Pressure: %.4lf Pa\n",
                   idx,
                   comp_data.temperature,
                   comp_data.pressure);
            #else
            printf("Data[%d]:    Temperature: %ld deg C	Pressure: %lu Pa\n", idx, (long int)comp_data.temperature,
                   (long unsigned int)comp_data.pressure);
            #endif

            idx++;
        }
    }

    return rslt;
}
