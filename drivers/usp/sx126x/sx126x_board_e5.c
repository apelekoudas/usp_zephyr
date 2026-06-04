/**
 * @file      sx126x_board_e5.c
 *
 * @brief     sx126x_board_e5 implementation
 *
 **/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include <zephyr/usp/lora_lbm_transceiver.h>
#include "sx126x_hal_context.h"

#define DT_DRV_COMPAT st_stm32wl_subghz_radio

LOG_MODULE_REGISTER( lora_sx126x, CONFIG_LORA_BASICS_MODEM_DRIVERS_LOG_LEVEL );

#define SX126X_SPI_OPERATION ( SPI_WORD_SET( 8 ) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB )

/*
*/
static void radio_isr(const struct device *dev)
{
	struct sx126x_hal_context_data_t *dev_data = dev->data;
	
    LOG_INF("radio_isr entry");
    
    irq_disable(DT_INST_IRQN(0));

    #if defined( CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_OWN_THREAD )
        k_sem_give( &data->gpio_sem );
    el#if defined( CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_GLOBAL_THREAD )
        k_work_submit( &dev_data->work );
    #elif defined( CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_NO_THREAD )
        if( dev_data->event_interrupt_cb )
        {
            dev_data->event_interrupt_cb( dev_data->sx126x_dev );
        }
    #endif
}

#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_OWN_THREAD
static void sx126x_thread( struct sx126x_hal_context_data_t* data )
{
    while( 1 )
    {
        k_sem_take( &data->gpio_sem, K_FOREVER );
        if( data->event_interrupt_cb )
        {
            data->event_interrupt_cb( data->sx126x_dev );
        }
    }
}
#endif /* CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_OWN_THREAD */

#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_GLOBAL_THREAD
static void sx126x_work_cb( struct k_work* work )
{
    struct sx126x_hal_context_data_t* data = CONTAINER_OF( work, struct sx126x_hal_context_data_t, work );
    if( data->event_interrupt_cb )
    {
        data->event_interrupt_cb( data->sx126x_dev );
    }
}
#endif /* CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_GLOBAL_THREAD */

void lora_transceiver_board_attach_interrupt( const struct device* dev, event_cb_t cb )
{
#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER
    struct sx126x_hal_context_data_t* data = dev->data;

    data->event_interrupt_cb = cb;
#else
    LOG_ERR( "Event trigger not supported!" );
#endif /* CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER */
}

void lora_transceiver_board_enable_interrupt( const struct device* dev )
{
#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER
	NVIC_ClearPendingIRQ(DT_INST_IRQN(0));
	irq_enable(DT_INST_IRQN(0));    
#else
    LOG_ERR( "Event trigger not supported!" );
#endif /* CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER */
}

void lora_transceiver_board_disable_interrupt( const struct device* dev )
{
#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER
    irq_disable(DT_INST_IRQN(0));
#else
    LOG_ERR( "Event trigger not supported!" );
#endif /* CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER */
}

uint32_t lora_transceiver_get_tcxo_startup_delay_ms( const struct device* dev )
{
    const struct sx126x_hal_context_cfg_t* config = dev->config;

    return config->tcxo_cfg.wakeup_time_ms;
}

static int sx126x_init( const struct device* dev )
{
    const struct sx126x_hal_context_cfg_t* config = dev->config;
    struct sx126x_hal_context_data_t*      data   = dev->data;
    int                                    ret = 0;

    LOG_DBG("sx126x_init");

    data->radio_status               = RADIO_AWAKE;
    data->tx_power_offset_db_current = config->tx_power_offset_db;

    /* Event pin trigger config */
#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER
    data->sx126x_dev = dev;
#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_GLOBAL_THREAD
    data->work.handler = sx126x_work_cb;
#elif CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_OWN_THREAD
    k_sem_init( &data->trig_sem, 0, K_SEM_MAX_LIMIT );
    k_thread_create( &data->thread, data->thread_stack,
                     CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_THREAD_STACK_SIZE,
                     ( k_thread_entry_t ) sx126x_thread, data, NULL, NULL,
                     K_PRIO_COOP( CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_THREAD_PRIORITY ), 0, K_NO_WAIT );
#endif /* CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_OWN_THREAD */

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
	 	    radio_isr, DEVICE_DT_GET( DT_CHOSEN( zephyr_lorawan_transceiver ) ), 0);
    LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_44);
    irq_enable(DT_INST_IRQN(0));
#endif /* CONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER */

    return ret;
}

#if IS_ENABLED( CONFIG_PM_DEVICE )
/**
 * @brief Power management action define.
 * Not implemented as LoRa Basics Modem handles this on its side.
 *
 * @param dev
 * @param action
 * @return int
 */
static int sx126x_pm_action( const struct device* dev, enum pm_device_action action )
{
    int ret = 0;

    switch( action )
    {
    case PM_DEVICE_ACTION_RESUME:
        /* Put the lr11xx into normal operation mode */
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        /* Put the lr11xx into sleep mode */
        break;
    default:
        return -ENOTSUP;
    }
    return ret;
}
#endif /* IS_ENABLED(CONFIG_PM_DEVICE) */

/*
 * Device creation macro.
 */

#define SX126X_XOSC_CFG( node_id )                                                                          \
    COND_CODE_1( DT_PROP( node_id, tcxo_power_startup_delay_ms ) == 0, ( RAL_XOSC_CFG_XTAL ),                          \
                 ( COND_CODE_1( DT_PROP( node_id, dio3_tcxo_voltage ), ( RAL_XOSC_CFG_TCXO_RADIO_CTRL ), \
                                ( RAL_XOSC_CFG_TCXO_EXT_CTRL ) ) ) )

/* Derive dio3-tcxo-voltage to know xosc_cfg */
#define SX126X_CFG_TCXO( node_id )                              \
    .tcxo_cfg = {                                               \
        .xosc_cfg       = SX126X_XOSC_CFG( node_id ),           \
        .voltage        = DT_PROP( node_id, dio3_tcxo_voltage ),\
        .wakeup_time_ms = DT_PROP( node_id, tcxo_power_startup_delay_ms ), \
    }

#define SX126X_CONFIG( node_id )                                            \
    {                                                                       \
        .spi = SPI_DT_SPEC_GET( node_id, SX126X_SPI_OPERATION, 0 ),         \
        SX126X_CFG_TCXO( node_id ),                                         \
        .capa_xta = DT_PROP_OR( node_id, xtal_capacitor_value_xta, 0xFF ),  \
        .capa_xtb = DT_PROP_OR( node_id, xtal_capacitor_value_xtb, 0xFF ),  \
        .reg_mode = SX126X_REG_MODE_DCDC,                                   \
        .tx_power_offset_db = DT_PROP_OR( node_id, tx_power_offset, 0 ),    \
        .rx_boosted         = DT_PROP_OR( node_id, rx_boosted, false ),     \
        .pa_ramp_time       = DT_PROP_OR( node_id, pa_ramp_time, 0x02 ),    \
        .dio2_as_rf_switch  = false,                                        \
        .pa_hp_sel = DT_INST_STRING_UPPER_TOKEN(0, power_amplifier_output), \
    }

#define SX126X_DEVICE_INIT( node_id )                                                            \
    DEVICE_DT_DEFINE( node_id, sx126x_init, PM_DEVICE_DT_GET( node_id ), &sx126x_data_##node_id, \
                      &sx126x_config_##node_id, POST_KERNEL, CONFIG_LORA_BASICS_MODEM_DRIVERS_INIT_PRIORITY, NULL );

#define SX126X_DEFINE( node_id )                                                                     \
    static struct sx126x_hal_context_data_t      sx126x_data_##node_id;                              \
    static const struct sx126x_hal_context_cfg_t sx126x_config_##node_id = SX126X_CONFIG( node_id ); \
    PM_DEVICE_DT_DEFINE( node_id, sx126x_pm_action );                                                \
    SX126X_DEVICE_INIT( node_id )

DT_FOREACH_STATUS_OKAY( st_stm32wl_subghz_radio, SX126X_DEFINE )