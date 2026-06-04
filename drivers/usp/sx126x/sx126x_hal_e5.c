/**
 * @file      sx126x_hal_e5.c
 *
 * @brief     sx126x_hal_e5 HAL implementation
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <stdio.h>

#include <sx126x_hal.h>
#include "sx126x_hal_context.h"

LOG_MODULE_DECLARE( lora_sx126x, CONFIG_LORA_BASICS_MODEM_DRIVERS_LOG_LEVEL );

/**
 * @brief Wait until radio busy pin returns to inactive state or
 * until CONFIG_LORA_BASICS_MODEM_DRIVERS_HAL_WAIT_ON_BUSY_TIMEOUT_MSEC passes.
 *
 * @param context
 */
static void sx126x_hal_wait_on_busy( const void* context )
{
    uint32_t end       = k_uptime_get_32( ) + CONFIG_LORA_BASICS_MODEM_DRIVERS_HAL_WAIT_ON_BUSY_TIMEOUT_MSEC;
    bool     timed_out = false;

    while( k_uptime_get_32( ) <= end )
    {
        timed_out = ( LL_PWR_IsActiveFlag_RFBUSYS() == 0 ) ? true : false;
        if( timed_out == true )
        {
            break;
        }
        else
        {
            k_usleep( 100 );
        }
    }

    if( !timed_out )
    {
        LOG_ERR( "Timeout of %dms hit when waiting for sx126x busy!",
                 CONFIG_LORA_BASICS_MODEM_DRIVERS_HAL_WAIT_ON_BUSY_TIMEOUT_MSEC );
        // k_oops( );
    }
}

/**
 * @brief Wake up the radio and ensure it's ready
 *
 * @param context
 */
static void sx126x_hal_check_device_ready( const void* context )
{
    const struct device*                   dev    = ( const struct device* ) context;
    struct sx126x_hal_context_data_t*      data   = dev->data;

    if( data->radio_status != RADIO_SLEEP )
    {
        sx126x_hal_wait_on_busy( context );
    }
    else
    {
        /* Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS */

        // LOG_INF("Radio was sleeping\r\n");
        
        LL_PWR_SelectSUBGHZSPI_NSS();   /* NSS = 0; */
        k_usleep( 100 );                /* Wait Radio wakeup */  
        LL_PWR_UnselectSUBGHZSPI_NSS(); /* NSS = 1 */

        // Switch is turned off when device is in sleep mode and turned on is all other modes
        // SX126xAntSwOn( );

        sx126x_hal_wait_on_busy( context );
        data->radio_status = RADIO_AWAKE;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    const struct device*                   dev      = ( const struct device* ) context;
    const struct sx126x_hal_context_cfg_t* config   = dev->config;
    struct sx126x_hal_context_data_t*      dev_data = dev->data;
    int                                    ret;

    const struct spi_buf tx_bufs[] = {
        { .buf = ( void* ) command, .len = command_length },
        { .buf = ( void* ) data, .len = data_length },
    };

    const struct spi_buf_set tx_buf_set = { tx_bufs, .count = ARRAY_SIZE( tx_bufs ) };

    sx126x_hal_check_device_ready( context );
    ret = spi_write_dt( &config->spi, &tx_buf_set );
    if( ret )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    /* 0x84 - SX126x_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1
     * => do not test it
     */
    if( command[0] == 0x84 )
    {
        dev_data->radio_status = RADIO_SLEEP;
        k_usleep( 500 );
    }
    else
    {
        sx126x_hal_check_device_ready( context );
    }

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    const struct device*                   dev    = ( const struct device* ) context;
    const struct sx126x_hal_context_cfg_t* config = dev->config;
    int                                    ret;

    const struct spi_buf tx_bufs[] = { { .buf = ( uint8_t* ) command, .len = command_length },
                                       { .buf = NULL, .len = data_length } };

    const struct spi_buf rx_bufs[] = { { .buf = NULL, .len = command_length }, { .buf = data, .len = data_length } };

    const struct spi_buf_set tx_buf_set = { .buffers = tx_bufs, .count = ARRAY_SIZE( tx_bufs ) };
    const struct spi_buf_set rx_buf_set = { .buffers = rx_bufs, .count = ARRAY_SIZE( rx_bufs ) };

    sx126x_hal_check_device_ready( context );
    ret = spi_transceive_dt( &config->spi, &tx_buf_set, &rx_buf_set );
    if( ret )
    {
        return SX126X_HAL_STATUS_ERROR;
    }
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
    const struct device*                   dev    = ( const struct device* ) context;
    struct sx126x_hal_context_data_t*      data   = dev->data;

    LOG_INF("sx126x_hal_reset entry");

    LL_RCC_RF_EnableReset();
	k_sleep(K_MSEC(20));
	LL_RCC_RF_DisableReset();
	k_sleep(K_MSEC(10));

    data->radio_status = RADIO_AWAKE;
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
    sx126x_hal_check_device_ready( context );
    return SX126X_HAL_STATUS_OK;
}
