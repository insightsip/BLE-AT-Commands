 /******************************************************************************
 * @file    ble_manager.c
 * @author  Insight SiP
 * @brief   BLE manager
 *
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "ble_manager.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "ser_pkt_fw.h"
#include "flash_manager.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**@brief Macro for converting ticks to milliseconds.
 *
 * @param[in] UNITS         Number of ticks.
 * @param[in] RESOLUTION    Unit to be converted to in [us/ticks].
 */
#define UNITS_TO_MSEC(UNITS, RESOLUTION) (((UNITS) * RESOLUTION) / (1000))


#define DEVICE_NAME                     "ISP_BLE_UART"                              /**< Name of device. Will be included in the advertising data. */
#define MIN_CONN_INTERVAL               20                                          /**< Minimum acceptable connection interval (20 ms). */
#define MAX_CONN_INTERVAL               75                                          /**< Maximum acceptable connection interval (75 ms). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                4000                                        /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL                500                                         /**< The advertising interval (in ms). */
#define APP_ADV_DURATION                0                                           /**< The advertising duration in units of 10 milliseconds. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
static uint16_t m_conn_handle          = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static uint8_t m_dcdc_mode = 0;                                                     /**< Current DCDC mode. */
static int8_t m_txp = 0;                                                            /**< Current txp. */
static ble_gap_phys_t m_phys = {0,0};                                               /**< Current phys. */ 
static uint8_t m_device_name[26] = DEVICE_NAME;                                     /**< Current device name. */
static uint16_t m_adv_interval = APP_ADV_INTERVAL;                                  /**< Current advertising interval (in ms). */
static ble_gap_conn_params_t m_gap_conn_params;                                     /**< Current connection parameters */
static ble_manager_state_t m_state = BLE_MANAGER_STATE_INIT;                        /**< Current state */


/**@brief A function for processing the HAL Transport layer events.
 *
 * @param[in] event    serial packet forwarder layer event.
 */
static void ser_pkt_fw_event_handler (ser_pkt_fw_evt_t event)
{   
    switch (event.evt_type)
    {
        case SER_PKT_FW_EVT_TX_PKT_SENT:
        {
            break;
        }

        case SER_PKT_FW_EVT_RX_PKT_RECEIVED:
        {
            ble_nus_data_send(&m_nus, event.evt_params.rx_pkt_received.p_buffer, &event.evt_params.rx_pkt_received.num_of_bytes, m_conn_handle);
            break;
        }

        case SER_PKT_FW_EVT_PHY_ERROR:
        {
            NRF_LOG_ERROR("SER_PKT_FW_EVT_PHY_ERROR");
            break;
        }

        default:
        {
            /* do nothing */
            break;
        }
    }
}

/**@brief Function for updating new configuration in the flash.
 *
 */
static void update_flash(void)
{
    flash_manager_ble_cfg_t flash;

    flash.dcdc_mode = m_dcdc_mode;
    flash.advparam = m_adv_interval;
    flash.gap_conn_params = m_gap_conn_params;
    memcpy(flash.name, m_device_name, sizeof(m_device_name));
    flash.phys = m_phys;
    flash.txp = m_txp;
    flash_manager_ble_cfg_store(&flash);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static uint32_t gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) m_device_name, sizeof(m_device_name));
    VERIFY_SUCCESS(err_code);

    err_code = sd_ble_gap_ppcp_set(&m_gap_conn_params);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the Connection Parameters module.
 */
static uint32_t conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = &m_gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            sd_ble_gap_rssi_start(m_conn_handle, 1, 5);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
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

        case BLE_GAP_EVT_PHY_UPDATE:
        {
             NRF_LOG_DEBUG("PHY update.");
             m_phys.rx_phys = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;
             m_phys.tx_phys = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            NRF_LOG_DEBUG("CONN PARAM update.");
            m_gap_conn_params = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params;
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static uint32_t ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    VERIFY_SUCCESS(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    VERIFY_SUCCESS(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    VERIFY_SUCCESS(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    return NRF_SUCCESS;
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
static uint32_t gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_DEBUG("BLE_ADV_EVT_FAST.");
            m_state = BLE_MANAGER_STATE_ADVERTISING;
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_DEBUG("BLE_ADV_EVT_IDLE.");
            m_state = BLE_ADV_EVT_IDLE;
        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static uint32_t advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = MSEC_TO_UNITS(m_adv_interval, UNIT_0_625_MS);
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    VERIFY_SUCCESS(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    return NRF_SUCCESS;
}


/**@brief Function for starting advertising.
 */
static uint32_t advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
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


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        err_code = ser_pkt_fw_tx_send(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length, SER_PKT_FW_PORT_BLE);

        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (ser_pkt_fw_tx_send("\n", 1, SER_PKT_FW_PORT_BLE) == NRF_ERROR_BUSY);
        }
    }

}


/**@brief Function for initializing services that will be used by the application.
 */
static uint32_t services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    VERIFY_SUCCESS(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t ble_manager_init()
{
    uint32_t err_code;
    flash_manager_ble_cfg_t *flash;

    // Load configuration from flash
    err_code = flash_manager_ble_cfg_load(&flash);
    VERIFY_SUCCESS(err_code);

    // Apply configuration
    m_dcdc_mode = flash->dcdc_mode;
    m_txp = flash->txp;
    m_phys = flash->phys;
    memcpy(m_device_name, flash->name, sizeof(m_device_name));
    m_adv_interval = flash->advparam;
    m_gap_conn_params = flash->gap_conn_params;

    // Inititiale BLE
    err_code = ble_stack_init();
    VERIFY_SUCCESS(err_code);

    err_code = gap_params_init();
    VERIFY_SUCCESS(err_code);

    err_code = gatt_init();
    VERIFY_SUCCESS(err_code);

    err_code = services_init();
    VERIFY_SUCCESS(err_code);

    err_code = advertising_init();
    VERIFY_SUCCESS(err_code);

    err_code = conn_params_init();
    VERIFY_SUCCESS(err_code);

    // Register to serial packet forwarder
    ser_pkt_fw_path_add(ser_pkt_fw_event_handler, SER_PKT_FW_PORT_BLE);
    VERIFY_SUCCESS(err_code);

    m_state = BLE_MANAGER_STATE_IDLE;

    // Start advertising
    advertising_start();

    return err_code;
}

uint32_t ble_manager_connstate_read(uint8_t *state)
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        *state = 0;
    }
    else
    {
        *state = 1;
    }

    return NRF_SUCCESS;
}


uint32_t ble_manager_dcdc_set(uint8_t dcdc_mode)
{
    uint32_t err_code;
    
    // Enable DCDC
    err_code =  sd_power_dcdc_mode_set(dcdc_mode);
    VERIFY_SUCCESS(err_code);

    // Update flash if dcdc_mode changed
    if (m_dcdc_mode != dcdc_mode)
    {
        m_dcdc_mode = dcdc_mode;
        update_flash();
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_dcdc_read(uint8_t *mode)
{
    *mode = m_dcdc_mode;

    return NRF_SUCCESS;
}

uint32_t ble_manager_txp_set(int8_t txp)
{
    uint32_t err_code;

    // Change TXP
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, txp);
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, txp);
    }
    VERIFY_SUCCESS(err_code);

    // Update flash if value changed
    if (m_txp != txp)
    {
        m_txp = txp;
        update_flash();
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_txp_read(int8_t *txp)
{
    *txp = m_txp;

    return NRF_SUCCESS;
}

uint32_t ble_manager_phy_set(uint8_t phy_tx, uint8_t phy_rx)
{
    uint32_t err_code;
    ble_gap_phys_t temp_phys;

    //NOTE: current implementation change PHY only if connected
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Request PHY change
        temp_phys.tx_phys = phy_tx;
        temp_phys.rx_phys = phy_rx;

        err_code = sd_ble_gap_phy_update(m_conn_handle, &temp_phys);
        VERIFY_SUCCESS(err_code);

        // Update flash if value changed
        if ((m_phys.tx_phys != phy_tx) || (m_phys.rx_phys != phy_rx))
        {
            m_phys.tx_phys = phy_tx;
            m_phys.rx_phys = phy_rx;

            update_flash();
        }
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_phy_read(uint8_t *phy_tx, uint8_t *phy_rx)
{
    *phy_tx = m_phys.tx_phys;
    *phy_rx = m_phys.rx_phys;

    return NRF_SUCCESS;
}

uint32_t ble_manager_advparam_set(uint16_t interval)
{
    uint32_t err_code;
    ble_advertising_init_t init;
    ble_adv_mode_t current_mode = m_advertising.adv_mode_current;

    if (current_mode != BLE_ADV_MODE_IDLE)
    {
        // Device is advertising, Stop it before reconfiguration
        sd_ble_gap_adv_stop(m_advertising.adv_handle);
    }

    // Set new advertising interval
    memset(&init, 0, sizeof(init));
    init.advdata.name_type              = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance     = false;
    init.advdata.flags                  = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_fast_enabled    = true;
    init.config.ble_adv_fast_interval   = MSEC_TO_UNITS(interval, UNIT_0_625_MS);;
    init.config.ble_adv_fast_timeout    = 0;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    VERIFY_SUCCESS(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    // Update flash if value changed
    if (m_adv_interval != interval)
    {
        m_adv_interval = interval;

        update_flash();
    }

    if (current_mode != BLE_ADV_MODE_IDLE)
    {
        // Device was advertising, resume
        err_code = ble_advertising_start(&m_advertising, current_mode);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_advparam_read(uint16_t *interval)
{
    *interval = m_adv_interval;

    return NRF_SUCCESS;
}

uint32_t ble_manager_connparam_set(float conn_interval_min, float conn_interval_max, uint16_t conn_latency, uint16_t conn_timeout)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t temp_gap_conn_params;

    temp_gap_conn_params.min_conn_interval = MSEC_TO_UNITS(conn_interval_min, UNIT_1_25_MS);
    temp_gap_conn_params.max_conn_interval = MSEC_TO_UNITS(conn_interval_max, UNIT_1_25_MS);
    temp_gap_conn_params.slave_latency     = conn_latency;
    temp_gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(conn_timeout, UNIT_10_MS);
    
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        ble_conn_params_stop();

        memset(&cp_init, 0, sizeof(cp_init));
        cp_init.p_conn_params                  = &temp_gap_conn_params;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = false;
        cp_init.evt_handler                    = on_conn_params_evt;
        cp_init.error_handler                  = conn_params_error_handler;
        err_code = ble_conn_params_init(&cp_init);
        VERIFY_SUCCESS(err_code);
    }
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Device is already connected, request connection parameters update to the peer
        err_code = ble_conn_params_change_conn_params(m_conn_handle, &temp_gap_conn_params);
        VERIFY_SUCCESS(err_code);
    }

    // Update flash if value changed
    if ((m_gap_conn_params.conn_sup_timeout != temp_gap_conn_params.conn_sup_timeout)       || 
        (m_gap_conn_params.max_conn_interval != temp_gap_conn_params.max_conn_interval)     ||
        (m_gap_conn_params.min_conn_interval != temp_gap_conn_params.min_conn_interval)     ||
        (m_gap_conn_params.slave_latency != temp_gap_conn_params.slave_latency))
    {
        m_gap_conn_params.conn_sup_timeout = temp_gap_conn_params.conn_sup_timeout;
        m_gap_conn_params.max_conn_interval = temp_gap_conn_params.max_conn_interval;
        m_gap_conn_params.min_conn_interval = temp_gap_conn_params.min_conn_interval;
        m_gap_conn_params.slave_latency = temp_gap_conn_params.slave_latency;

        update_flash();
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_connparam_read(float *conn_interval_min, float *conn_interval_max, uint16_t *conn_latency, uint16_t *conn_timeout)
{  
    *conn_interval_min = UNITS_TO_MSEC((float)m_gap_conn_params.min_conn_interval, UNIT_1_25_MS);
    *conn_interval_max = UNITS_TO_MSEC((float)m_gap_conn_params.max_conn_interval, UNIT_1_25_MS);
    *conn_latency = m_gap_conn_params.slave_latency;
    *conn_timeout = UNITS_TO_MSEC(m_gap_conn_params.conn_sup_timeout, UNIT_10_MS);

    return NRF_SUCCESS;
}

uint32_t ble_manager_addr_read(uint8_t *addr)
{
    uint32_t err_code;
    ble_gap_addr_t gap_addr;

    err_code = sd_ble_gap_addr_get(&gap_addr);
    VERIFY_SUCCESS(err_code); 

    memcpy(addr, &gap_addr.addr, BLE_GAP_ADDR_LEN);

    return NRF_SUCCESS;
}

uint32_t ble_manager_name_set(uint8_t *name)
{
    uint32_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_adv_mode_t current_mode = m_advertising.adv_mode_current;

    // Change name
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) name, sizeof(m_device_name));
    VERIFY_SUCCESS(err_code); 

    // Update flash if value changed
    if (strncmp(m_device_name, name, sizeof(m_device_name)))
    {
        strncpy(m_device_name, name, sizeof(m_device_name));

        update_flash();
    }

    if (current_mode != BLE_ADV_MODE_IDLE)
    {
        // Device is advertising, restart it
        sd_ble_gap_adv_stop(m_advertising.adv_handle);
        advertising_init();
        ble_advertising_start(&m_advertising, current_mode);
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_name_read(uint8_t *name)
{
    uint32_t err_code;
    uint8_t temp_name[31];
    uint16_t temp_length = 31;
    
    err_code = sd_ble_gap_device_name_get(temp_name, &temp_length);
    VERIFY_SUCCESS(err_code); 

    strncpy(name, temp_name, temp_length);

    return NRF_SUCCESS;
}

uint32_t ble_manager_advertise(uint8_t start)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (start)
    {
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        VERIFY_SUCCESS(err_code); 
    }
    else
    {
        m_advertising.adv_mode_current = BLE_ADV_MODE_IDLE;
        err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
        VERIFY_SUCCESS(err_code); 
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_disconnect()
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        VERIFY_SUCCESS(err_code);
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_rssi_read(int8_t *rssi)
{
    uint32_t err_code;
    uint8_t ch_index;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {   
        err_code = sd_ble_gap_rssi_get(m_conn_handle, rssi, &ch_index);
        VERIFY_SUCCESS(err_code); 
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t ble_manager_restore(void)
{
    uint32_t err_code;
    flash_manager_ble_cfg_t *flash;

    err_code = flash_manager_ble_cfg_restore();
    VERIFY_SUCCESS(err_code); 

    // wait for flash to be ready
    while (is_flash_manager_ready() == false)
    {
        nrf_pwr_mgmt_run();
    }

    return NRF_SUCCESS;
}
