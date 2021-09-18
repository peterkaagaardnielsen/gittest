
#include <stdint.h>
#include <string.h>
#include "app_scheduler.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"d
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_pwm.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_cus.h"
#include "gpio_def.h"
#include "nrf_delay.h"
#include "main.h"

//#include "app_pwm.h"

#define DEVICE_NAME                     "AED Sensor"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Flexsolution"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(1000)     

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       12
#define SCHED_QUEUE_SIZE                10  


#define LIGHT_SENS_I2C_ADDR             0x52
#define LIGHT_SENS_I2C_ADDR_LEN         0x1

//MAIN CTRL BITS
#define CTRL_CS_MODE_RGB_INFRARED       0x04
#define CTRL_LS_EN                      0x02                                    //lo=disable (standbymode), hi=enable light sensor (goes from standby to active mode)
#define CTRL_RESET                      0x10

#define MAIN_CTRL                       0                                       //LS operation mode control, SW reset
#define LS_MEAS_RATE LS                 4                                       //measurement rate and resolution in active moe
#define LS_GAIN LS                      5                                       //analog gain range   
#define PART_ID                         6                                       //PARTnumber ID and revision ID B5HEX
#define MAIN_STATUS                     7                                       //Power-on status, interrupt status, data status
#define LS_DATA_IR_0                    0xa                                     //IR ADC measurement data - LSB
#define LS_DATA_GREEN_0                 0xd                                     //Green ADC measurement data - LSB 00HEX
#define LS_DATA_BLUE_0                  0x10                                    //Blue ADC measurement data - LSB 00HEX
#define LS_DATA_RED_0                   0x13                                    //Red ADC measurement data - LSB 00HEX
#define INT_CFG                         0x19                                    //Interrupt configuration
#define INT_PERSISTENCE                 0x1a                                    //Interrupt persist setting 00HEX
#define LS_THRES_UP_0                   0x21                                    //LS interrupt upper threshold
#define LS_THRES_UP_1                   0x22                                    //LS interrupt upper threshold
#define LS_THRES_UP_2                   0x23                                    //LS interrupt upper threshold
#define LS_THRES_LOW_0                  0x24                                    //LS interrupt lower threshold
#define LS_THRES_LOW_1                  0x25                                    //LS interrupt lower threshold
#define LS_THRES_LOW_2                  0x26                                    //LS interrupt lower threshold
#define LS_THRES_VAR                    0x27                                    //LS interrupt variance threshold




NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< GATT module instance. */
BLE_CUS_DEF(m_cus);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */



APP_TIMER_DEF(job_rd_light_timer1_id);
APP_TIMER_DEF(job_rd_light_timer2_id);
APP_TIMER_DEF(job_buzz_timer1_id);
APP_TIMER_DEF(job_buzz_timer2_id);
APP_TIMER_DEF(job_set_ui_led_timer1_id);
APP_TIMER_DEF(job_set_ui_led_timer2_id);

#define JOB_RD_LIGHT_EVT1                       1
#define JOB_RD_LIGHT_EVT2                       2
#define JOB_BUZZ_EVT1                           3
#define JOB_BUZZ_EVT2                           4
#define JOB_SET_UI_LED_EVT1                     5
#define JOB_SET_UI_LED_EVT2                     6

#define JOB_RD_LIGHT_INTERVAL_MS                10000
#define JOB_RD_LIGHT_TICK_MS                    10
#define JOB_BUZZ_INTERVAL_MS                    33000
#define JOB_BUZZ_PULSE_MS                       250
#define JOB_SET_UI_LED_INTERVAL_MS              4000
#define JOB_SET_UI_LED_PULSE_MS                 50

//static uint8_t m_custom_value = 0;

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN }
};

#define TWI_INSTANCE_ID 0
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static void advertising_start(bool erase_bonds);

static ret_code_t i2c_write_ls(uint8_t reg, uint8_t *pdata, uint8_t size);
static ret_code_t i2c_read_ls(uint8_t reg, uint8_t *pdata, uint8_t size);
static ret_code_t i2c_write_ls_ctrl_pwroff();
static ret_code_t i2c_write_ls_ctrl_pwron();
static ret_code_t i2c_read_ls_id(uint8_t *id);
static ret_code_t i2c_read_ls_rgbi(uint32_t *red, uint32_t *green, uint32_t *blue, uint32_t *infrared);




static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    
}



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
/*
static void notification_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    // Increment the value of m_custom_value before nortifing it.
    m_custom_value++;
    
    err_code = ble_cus_custom_value_update(&m_cus, m_custom_value);
    APP_ERROR_CHECK(err_code);
}
*/


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/
/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_cus_evt(ble_cus_t     * p_cus_service,
                       ble_cus_evt_t * p_evt)
{
    //ret_code_t err_code;
    
    switch(p_evt->evt_type)
    {
        case BLE_CUS_EVT_NOTIFICATION_ENABLED:
            
             //err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
             //APP_ERROR_CHECK(err_code);
             break;

        case BLE_CUS_EVT_NOTIFICATION_DISABLED:

            //err_code = app_timer_stop(m_notification_timer_id);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_CUS_EVT_CONNECTED:
             NRF_LOG_INFO("MY_app got conn evt");
             
            break;

        case BLE_CUS_EVT_DISCONNECTED:
          NRF_LOG_INFO("MY_app got disconn evt");
              break;

        default:
              // No implementation needed.
              break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        ret_code_t          err_code;
        nrf_ble_qwr_init_t  qwr_init = {0};
        ble_cus_init_t      cus_init = {0};

        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(err_code);

         // Initialize CUS Service init structure to zero.
        cus_init.evt_handler                = on_cus_evt;
    
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.cccd_write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm);
    
        err_code = ble_cus_init(&m_cus, &cus_init);
        APP_ERROR_CHECK(err_code);
                
    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
/*
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}
*/

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
/*
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
*/

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

static ret_code_t i2c_write_ls(uint8_t reg, uint8_t *pdata, uint8_t size)
{
    ret_code_t ret;
    uint8_t buffer[9]; /* Addr + data */
 
    buffer[0] = reg;
    if(pdata != NULL) memcpy(buffer + sizeof(reg), pdata, size);
    ret = nrf_drv_twi_tx(&m_twi_master, LIGHT_SENS_I2C_ADDR, buffer, size + sizeof(reg), false);
    return ret;
}

static ret_code_t i2c_read_ls(uint8_t reg, uint8_t *pdata, uint8_t size)
{
    ret_code_t ret; 
    ret = nrf_drv_twi_tx(&m_twi_master, LIGHT_SENS_I2C_ADDR, &reg, sizeof(reg) , false);
    if (NRF_SUCCESS != ret)
    {
       
    }
    ret = nrf_drv_twi_rx(&m_twi_master, LIGHT_SENS_I2C_ADDR, pdata, size);
    return ret;
}

static ret_code_t i2c_read_ls_rgbi(uint32_t *red, uint32_t *green, uint32_t *blue, uint32_t *infrared)
{
    ret_code_t ret; 
    uint8_t buf[16];

    ret = i2c_read_ls(LS_DATA_IR_0, buf, 12);
    *infrared = (uint32_t) buf[0] + (((uint32_t) buf[1]) << 8) + (((uint32_t) buf[2]) << 16);
    *green    = (uint32_t) buf[3] + (((uint32_t) buf[4]) << 8) + (((uint32_t) buf[5]) << 16);
    *blue     = (uint32_t) buf[6] + (((uint32_t) buf[7]) << 8) + (((uint32_t) buf[8]) << 16);
    *red      = (uint32_t) buf[9] + (((uint32_t) buf[10])<< 8) + (((uint32_t) buf[11])<< 16);    
    return ret;
}

static ret_code_t i2c_read_ls_id(uint8_t *id)
{
    ret_code_t ret = i2c_read_ls(PART_ID, id, 1);
    return ret;
}


static ret_code_t i2c_write_ls_ctrl_pwron()
{
  uint8_t ctrl_data = CTRL_LS_EN | CTRL_CS_MODE_RGB_INFRARED;
  ret_code_t ret = i2c_write_ls(MAIN_CTRL, &ctrl_data, 1);
  return ret;
}

static ret_code_t i2c_write_ls_ctrl_pwroff()
{
  uint8_t ctrl_data = 0;
  ret_code_t ret = i2c_write_ls(MAIN_CTRL, &ctrl_data, 1);
  return ret;
}

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = GPIO_SCL,
       .sda                = GPIO_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);


    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}

/**@brief Function for application main entry.
 */


static void initGpio()
{    
    //nrf_gpio_cfg_input(GPIO_SEN_INT2,NRF_GPIO_PIN_PULLDOWN);    
    //GPIO_SEN_INT1 controlled from GPIOTE
    // but we init because secure dfu dont use GPIOTE
    
    //nrf_gpio_cfg_sense_input(GPIO_BOOT_RX, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    
    nrf_gpio_pin_clear(GPIO_BUZZ);
    nrf_gpio_cfg_output(GPIO_BUZZ); 
    nrf_gpio_pin_clear(GPIO_LED_WHITE);
    nrf_gpio_cfg_output(GPIO_LED_WHITE); 
    nrf_gpio_pin_set(GPIO_LED_GREEN);
    nrf_gpio_cfg_output(GPIO_LED_GREEN); 
    nrf_gpio_pin_set(GPIO_LED_RED);
    nrf_gpio_cfg_output(GPIO_LED_RED); 
    
    nrf_gpio_cfg_input(GPIO_INT,NRF_GPIO_PIN_PULLDOWN); //NRF_GPIO_PIN_PULLDOWN or NRF_GPIO_PIN_PULLUP or NRF_GPIO_PIN_NOPULL
}



static void buzzEnable(bool enable)
{
    
    /*
     * This demo, similarly to demo1, plays back a sequence with different
     * values for individual channels. Unlike demo 1, however, it does not use
     * an event handler. Therefore, the PWM peripheral does not use interrupts
     * and the CPU can stay in sleep mode.
     * The LEDs (1-4) blink separately. They are turned on for 125 ms each,
     * in counterclockwise order (looking at the board).
     */

    if(enable)
    {  
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            GPIO_BUZZ|NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED,
        },
        .irq_priority = 3,
        .base_clock   = NRF_PWM_CLK_125kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 16,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));


    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static nrf_pwm_values_individual_t /*const*/ seq_values[] =
    {
         
        { 0x8000},
        { 0x0000},
        
        
   
    };
    nrf_pwm_sequence_t const seq =
    {
        .values.p_individual = seq_values,
        .length              = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats             = 0,
        .end_delay           = 0
    };

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    }
    else
    {
      nrfx_pwm_stop(&m_pwm0, false);
      nrfx_pwm_uninit(&m_pwm0);
    }
}





static void job_rd_light_handler1(void * p_context)
{
  static uint8_t evt;
  UNUSED_PARAMETER(p_context);
  evt = JOB_RD_LIGHT_EVT1;
  app_sched_event_put(&evt,1,my_state_machine_handler);
}
static void job_rd_light_handler2(void * p_context)
{
  static uint8_t evt;
  UNUSED_PARAMETER(p_context);
  evt = JOB_RD_LIGHT_EVT2;
  app_sched_event_put(&evt,1,my_state_machine_handler);
}
static void job_buzz_handler1(void * p_context)
{
  static uint8_t evt;
  UNUSED_PARAMETER(p_context);
  evt = JOB_BUZZ_EVT1;
  app_sched_event_put(&evt,1,my_state_machine_handler);
}
static void job_buzz_handler2(void * p_context)
{
  static uint8_t evt;
  UNUSED_PARAMETER(p_context);
  evt = JOB_BUZZ_EVT2;
  app_sched_event_put(&evt,1,my_state_machine_handler);
}
static void job_set_ui_led_handler1(void * p_context)
{
  static uint8_t evt;
  UNUSED_PARAMETER(p_context);
  evt = JOB_SET_UI_LED_EVT1;
  app_sched_event_put(&evt,1,my_state_machine_handler);
}
static void job_set_ui_led_handler2(void * p_context)
{
  static uint8_t evt;
  UNUSED_PARAMETER(p_context);
  evt = JOB_SET_UI_LED_EVT2;
  app_sched_event_put(&evt,1,my_state_machine_handler);
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    app_timer_init();
    err_code = app_timer_create(&job_rd_light_timer1_id,APP_TIMER_MODE_REPEATED,job_rd_light_handler1);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&job_rd_light_timer2_id,APP_TIMER_MODE_SINGLE_SHOT,job_rd_light_handler2);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&job_buzz_timer1_id,APP_TIMER_MODE_REPEATED,job_buzz_handler1);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&job_buzz_timer2_id,APP_TIMER_MODE_SINGLE_SHOT,job_buzz_handler2);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&job_set_ui_led_timer1_id,APP_TIMER_MODE_REPEATED,job_set_ui_led_handler1);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&job_set_ui_led_timer2_id,APP_TIMER_MODE_SINGLE_SHOT,job_set_ui_led_handler2);
    APP_ERROR_CHECK(err_code);
}


static void start_app_timer_rd_light1(void)
{
    uint32_t err_code;
    uint32_t timer_ticks;
 
    timer_ticks = APP_TIMER_TICKS(JOB_RD_LIGHT_INTERVAL_MS);
    err_code = app_timer_start(job_rd_light_timer1_id, timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}
static void start_app_timer_rd_light2(void)
{
    uint32_t err_code;
    uint32_t timer_ticks;
 
    timer_ticks = APP_TIMER_TICKS(JOB_RD_LIGHT_TICK_MS);
    err_code = app_timer_start(job_rd_light_timer2_id, timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}
static void start_app_timer_buzz1(void)
{
    uint32_t err_code;
    uint32_t timer_ticks;
 
    timer_ticks = APP_TIMER_TICKS(JOB_BUZZ_INTERVAL_MS);
    err_code = app_timer_start(job_buzz_timer1_id, timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}
static void start_app_timer_buzz2(void)
{
    uint32_t err_code;
    uint32_t timer_ticks;
 
    timer_ticks = APP_TIMER_TICKS(JOB_BUZZ_PULSE_MS);
    err_code = app_timer_start(job_buzz_timer2_id, timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

static void start_app_timer_ui_led1(void)
{
    uint32_t err_code;
    uint32_t timer_ticks;
 
    timer_ticks = APP_TIMER_TICKS(JOB_SET_UI_LED_INTERVAL_MS);
    err_code = app_timer_start(job_set_ui_led_timer1_id, timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}
static void start_app_timer_ui_led2(void)
{
    uint32_t err_code;
    uint32_t timer_ticks;
 
    timer_ticks = APP_TIMER_TICKS(JOB_SET_UI_LED_PULSE_MS);
    err_code = app_timer_start(job_set_ui_led_timer2_id, timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}




static void jobSetUiLed(uint8_t event)
{
  switch(event)
  {
    case JOB_SET_UI_LED_EVT1:
       nrf_gpio_pin_clear(GPIO_LED_GREEN);
       start_app_timer_ui_led2();
       break;
       
    case JOB_SET_UI_LED_EVT2:
       nrf_gpio_pin_set(GPIO_LED_GREEN);
       break;
  }
}
  
static void jobBuzz(uint8_t event)
{  
  switch(event)
  {
    case JOB_BUZZ_EVT1:
      //nrf_gpio_pin_set(GPIO_LED_WHITE);
      buzzEnable(true);
      start_app_timer_buzz2();
      break;
      
    case JOB_BUZZ_EVT2:
      buzzEnable(false);
      break;
  }
}



#define STATE_LS_I2C_PWR_ON             1
#define STATE_LS_I2C_RD_ID              2
#define STATE_LS_READ_DELAY             3
#define STATE_LS_READ                   4
#define STATE_LS_FINISH                 5

#define NO_OF_LS_SAMPLES                50
#define LS_READ_TIME_DELAY              13   //delaytime_in_ms = LS_READ_TIME_DELAY * 10
static void jobRdLight(uint8_t event)
{ 
  static uint32_t tick_10ms = 0, lsSamples = 0, state = STATE_LS_I2C_PWR_ON,ls_red, ls_green, ls_blue, ls_infrared;
  ret_code_t err_code=0;
  uint8_t ls_hw_id = 0;
  
  switch(event)
  {
    case JOB_RD_LIGHT_EVT1:
      err_code = i2c_write_ls_ctrl_pwron();    
      tick_10ms = 0;
      start_app_timer_rd_light2();
      break;
      
    case JOB_RD_LIGHT_EVT2:
      tick_10ms++;
      switch(state)
      {
        case STATE_LS_I2C_PWR_ON:
          //nrf_gpio_pin_set(GPIO_LED_WHITE);
          err_code = i2c_write_ls_ctrl_pwron();                                 //pwr on light sensor - ready´within 10msec
          APP_ERROR_CHECK(err_code);
          state = STATE_LS_I2C_RD_ID;
          start_app_timer_rd_light2();
          break;

        case STATE_LS_I2C_RD_ID:
          err_code = i2c_read_ls_id(&ls_hw_id);
          APP_ERROR_CHECK(err_code);
          NRF_LOG_DEBUG("id:0x%X",ls_hw_id);
          NRF_LOG_FLUSH();
          state = STATE_LS_READ_DELAY;
          tick_10ms = 0;
          lsSamples = 0;
          start_app_timer_rd_light2();
           //nrf_gpio_pin_clear(GPIO_LED_WHITE);
          break;
          
        case STATE_LS_READ_DELAY:
          if(tick_10ms > LS_READ_TIME_DELAY) 
          {
            state = STATE_LS_READ;
          }
          start_app_timer_rd_light2();
    
          break;
          
        case STATE_LS_READ:
          err_code = i2c_read_ls_rgbi(&ls_red, &ls_green, &ls_blue, &ls_infrared);
          APP_ERROR_CHECK(err_code);
          NRF_LOG_DEBUG("r:%d, g:%d, b:%d, ir:%d ",ls_red, ls_green , ls_blue, ls_infrared);
          NRF_LOG_FLUSH();
          if(++lsSamples >= NO_OF_LS_SAMPLES) 
          {
            state = STATE_LS_FINISH;
            start_app_timer_rd_light2();
          }
          else
          {
            state = STATE_LS_READ_DELAY;
            tick_10ms = 0;
            start_app_timer_rd_light2();
          }
          break;
         
        case STATE_LS_FINISH:
          err_code = i2c_write_ls_ctrl_pwroff();        
          APP_ERROR_CHECK(err_code);
          state = STATE_LS_I2C_PWR_ON;
          // DO NOT START single shot timer:   start_app_timer_rd_light2();
          break;
      }
      break;
    }
}
    
     
 

#define MY_EVT_TYPE  0
#define MY_CHAR_TYPE  1
#define TEMP_SAMPLES  4
void my_state_machine_handler(void * p_event_data, uint16_t event_size)
{
    static uint8_t* u8Ptr;
    static uint8_t arr[24],event;
    
          
    if (event_size > 24) event_size = 24;
    u8Ptr = (uint8_t *) p_event_data;
    for(uint16_t a=0; a< event_size;a++)
    {
      arr[a] = *(u8Ptr+a);
    }
    event = arr[MY_EVT_TYPE];    
    jobRdLight(event);
    jobBuzz(event);
    jobSetUiLed(event);
}
     
     

int main(void)
{
    bool erase_bonds = true;
    ret_code_t err_code=0;
    
    // Initialize.
    initGpio();
    log_init();
    timers_init();
    //buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    err_code = twi_master_init();
    if(err_code != NRF_SUCCESS)
    {
      NRF_LOG_DEBUG("ERROR: could not init I2C");
    }

    //app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(256L, GPIO_BUZZ);

    /* Switch the polarity of the second channel. */
    //pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    //err_code = app_pwm_init(&PWM0,&pwm1_cfg,pwm_ready_callback);
    //APP_ERROR_CHECK(err_code);
    //app_pwm_enable(&PWM0);

    
    
    /* Set the duty cycle - keep trying until PWM is ready... */
    //while (app_pwm_channel_duty_set(&PWM0, 0, 50) == NRF_ERROR_BUSY);//while (!ready_flag);
    //nrf_delay_ms(1000);
    //while (app_pwm_channel_duty_set(&PWM0, 0, 0) == NRF_ERROR_BUSY);//while (!ready_flag);
    //nrf_delay_ms(1000);
    
 
    
    
    //while(1)
    {
/*
      while(1){
      nrf_gpio_pin_clear(GPIO_BUZZ);
      nrf_delay_us(50050);
      nrf_gpio_pin_set(GPIO_BUZZ);
      nrf_delay_us(50050);
      }*/
  
    //nrf_gpio_pin_set(GPIO_LED_GREEN);
    //nrf_gpio_pin_clear(GPIO_LED_RED);
    //nrf_gpio_pin_clear(GPIO_LED_WHITE);
     //nrf_delay_ms(300);
    }

    // Start execution.
    NRF_LOG_INFO("AED Light Sensor app. started.");
    start_app_timer_rd_light1();
    start_app_timer_buzz1();
    start_app_timer_ui_led1();
    advertising_start(erase_bonds);
    
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
        app_sched_execute();	        
    }
}


/**
 * @}
 */


