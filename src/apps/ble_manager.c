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

#include "ble_manager.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_nus.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "boards.h"
#include "flash_manager.h"
#include "ser_phy.h" // Only for ser_phy_buffer_length_set(...)
#include "ser_pkt_fw.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**@brief Macro for converting ticks to milliseconds.
 *
 * @param[in] UNITS         Number of ticks.
 * @param[in] RESOLUTION    Unit to be converted to in [us/ticks].
 */
#define UNITS_TO_MSEC(UNITS, RESOLUTION) (((UNITS)*RESOLUTION) / (1000))

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */
#define APP_BLE_CONN_CFG_TAG 1                               /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO 3                              /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_DURATION 0                                   /**< The advertising duration in units of 10 milliseconds. */
#define APP_SCAN_SCAN_INTERVAL 160
#define APP_SCAN_SCAN_WINDOW 80
#define APP_SCAN_SCAN_DURATION 0
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */
#define NRF_BLE_GQ_QUEUE_SIZE 8                          /**< Queue size for BLE GATT Queue module. */

NRF_BLE_GATT_DEF(m_gatt);                             /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                               /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                   /**< Advertising module instance. */
BLE_NUS_DEF(m_ble_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
#if defined(BLE_CAP_CENTRAL)
BLE_DB_DISCOVERY_DEF(m_db_disc); /**< Database discovery module instance. */
BLE_NUS_C_DEF(m_ble_nus_c);      /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_SCAN_DEF(m_scan);        /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, /**< BLE GATT Queue instance. */
    NRF_SDH_BLE_CENTRAL_LINK_COUNT,
    NRF_BLE_GQ_QUEUE_SIZE);
#endif

static ble_manager_evt_handler_t m_evt_handler;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;               /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
    {
        .uuid = BLE_UUID_NUS_SERVICE,
        .type = NUS_SERVICE_UUID_TYPE};

static ble_uuid_t m_adv_uuids[] = {m_nus_uuid}; /**< Universally unique service identifier. */
static uint8_t devices_list_index = 0;
static device_info_t found_devices[MAX_SCAN_DEVICE_LIST];
static uint16_t m_adv_interval; /**< Current advertising interval (in ms). */

static ble_gap_scan_params_t m_gap_scan_params; /**< Current scan parameters */

/**@brief A function for processing the HAL Transport layer events.
 *
 * @param[in] event    serial packet forwarder layer event.
 */
static void ser_pkt_fw_event_handler(ser_pkt_fw_evt_t event) {
    switch (event.evt_type) {
    case SER_PKT_FW_EVT_TX_PKT_SENT: {
        break;
    }

    case SER_PKT_FW_EVT_RX_PKT_RECEIVED: {
        uint16_t role = ble_conn_state_role(m_conn_handle);
        if (role == BLE_GAP_ROLE_PERIPH) {
            ble_nus_data_send(&m_ble_nus, event.evt_params.rx_pkt_received.p_buffer, &event.evt_params.rx_pkt_received.num_of_bytes, m_conn_handle);
        }
#if defined(BLE_CAP_CENTRAL)
        else if (role == BLE_GAP_ROLE_CENTRAL) {
            ble_nus_c_string_send(&m_ble_nus_c, event.evt_params.rx_pkt_received.p_buffer, event.evt_params.rx_pkt_received.num_of_bytes);
        }
#endif
        break;
    }

    case SER_PKT_FW_EVT_PHY_ERROR: {
        NRF_LOG_ERROR("SER_PKT_FW_EVT_PHY_ERROR");
        break;
    }

    default: {
        /* do nothing */
        break;
    }
    }
}

uint8_t get_devices_list_id(ble_gap_addr_t gap_addr) {
    uint8_t device_index = 0;

    /* check address type - TODO */
    // if(gap_addr.addr_type == BLE_GAP_ADDR_TYPE_PUBLIC)

    /* seek address in found device list */
    while ((0 != strncmp((const char *)(gap_addr.addr), (const char *)(found_devices[device_index].gap_addr.addr), (size_t)6)) && (device_index < devices_list_index)) {
        device_index++;
    }

    /* if the address has not been found in the list */
    if (device_index >= devices_list_index) {
        /* return "not found" value */
        device_index = 0xFF;
    }

    return device_index;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static uint32_t gap_params_init(uint8_t *device_name, uint16_t device_name_length, ble_gap_conn_params_t conn_params) {
    uint32_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)device_name, device_name_length);
    VERIFY_SUCCESS(err_code);

    err_code = sd_ble_gap_ppcp_set(&conn_params);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
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
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the Connection Parameters module.
 */
static uint32_t conn_params_init(ble_gap_conn_params_t conn_params) {
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params = &conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief   Function for handling BLE events from peripheral applications.
 * @details Updates the status LEDs used to report the activity of the peripheral applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const *p_ble_evt) {
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        sd_ble_gap_rssi_start(m_conn_handle, 1, 5);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
        NRF_LOG_DEBUG("CONN PARAM update.");
        ble_manager_evt_t evt;
        evt.evt_type = BLE_MANAGER_EVT_CONN_PARAMS_CHANGED;
        evt.evt_params.conn_params = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params;
        m_evt_handler(evt);
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

#if defined(BLE_CAP_CENTRAL)
/**@brief Function for handling BLE Stack events that are related to central application.
 *
 * @details This function keeps the connection handles of central application up-to-date. It
 * parses scanning reports, initiates a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              must be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const *p_ble_evt) {
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED: {
        NRF_LOG_INFO("CENTRAL: Connected, handle: %d.", p_gap_evt->conn_handle);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        sd_ble_gap_rssi_start(m_conn_handle, 1, 5);

        // start discovery of services. The NUS Client waits for a discovery result
        err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
        APP_ERROR_CHECK(err_code);
    } break; // BLE_GAP_EVT_CONNECTED

    case BLE_GAP_EVT_DISCONNECTED: {
        NRF_LOG_INFO("CENTRAL: Disconnected, reason: 0x%x", p_gap_evt->params.disconnected.reason);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
    } break; // BLE_GAP_EVT_DISCONNECTED

    case BLE_GAP_EVT_TIMEOUT: {
        // Timeout for scanning is not specified, so only connection attemps can time out.
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
            NRF_LOG_DEBUG("CENTRAL: Connection Request timed out.");
        }
    } break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("Connection parameters update request.");
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &p_gap_evt->params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(err_code);

        ble_manager_evt_t evt;
        evt.evt_type = BLE_MANAGER_EVT_CONN_PARAMS_CHANGED;
        evt.evt_params.conn_params = p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params;
        m_evt_handler(evt);
    } break;

    default:
        // No implementation needed.
        break;
    }
}
#endif

/**@brief Function for handling BLE Stack events that are common to both the central and peripheral roles.
 * @param[in] conn_handle Connection Handle.
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(uint16_t conn_handle, ble_evt_t const *p_ble_evt) {
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("CENTRAL: GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("CENTRAL: GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_PHY_UPDATE: {
        NRF_LOG_DEBUG("PHY update.");
        if (p_ble_evt->evt.gap_evt.params.phy_update.status == BLE_HCI_STATUS_CODE_SUCCESS) {
            ble_manager_evt_t evt;
            evt.evt_type = BLE_MANAGER_EVT_PHY_CHANGED;
            evt.evt_params.phy.rx_phys = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;
            evt.evt_params.phy.tx_phys = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
            m_evt_handler(evt);
        }
    } break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    } break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for checking whether a bluetooth stack event is an advertising timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const *p_ble_evt) {
    return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role = ble_conn_state_role(conn_handle);

    on_ble_evt(conn_handle, p_ble_evt);

    // Based on the role this device plays in the connection, dispatch to the right handler.
    if (role == BLE_GAP_ROLE_PERIPH || ble_evt_is_advertising_timeout(p_ble_evt)) {
        on_ble_peripheral_evt(p_ble_evt);
    }
#if defined(BLE_CAP_CENTRAL)
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT)) {
        on_ble_central_evt(p_ble_evt);
    }
#endif
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static uint32_t ble_stack_init(void) {
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
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        ser_phy_buffer_length_set(m_ble_nus_max_data_len);
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
static uint32_t gatt_init(void) {
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
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    uint32_t err_code;

    switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_DEBUG("BLE_ADV_EVT_FAST.");
        break;
    case BLE_ADV_EVT_IDLE:
        NRF_LOG_DEBUG("BLE_ADV_EVT_IDLE.");
    default:
        break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static uint32_t advertising_init(uint16_t adv_interval) {
    uint32_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = MSEC_TO_UNITS(adv_interval, UNIT_0_625_MS);
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    VERIFY_SUCCESS(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    return NRF_SUCCESS;
}
#if defined(BLE_CAP_CENTRAL)
/**@brief Function for handling database discovery events.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt) {
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}

/** @brief Function for initializing the database discovery module. */
static uint32_t db_discovery_init(void) {
    ble_db_discovery_init_t db_init;
    uint32_t err_code;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    err_code = ble_db_discovery_init(&db_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function to start scanning. */
static uint32_t scan_start(void) {
    ret_code_t ret;

    // reset device list
    memset(found_devices, 0, sizeof(found_devices));
    devices_list_index = 0;

    ret = nrf_ble_scan_start(&m_scan);
    VERIFY_SUCCESS(ret);

    return NRF_SUCCESS;
}

/**@brief Function to stop scanning. */
static uint32_t scan_stop(void) {
    nrf_ble_scan_stop();

    return NRF_SUCCESS;
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const *p_scan_evt) {
    ret_code_t err_code;

    switch (p_scan_evt->scan_evt_id) {
    case NRF_BLE_SCAN_EVT_NOT_FOUND: {
        uint8_t index = 0;
        uint16_t offset = 0;
        uint16_t length = 0;
        ble_gap_evt_adv_report_t const *p_adv_report = p_scan_evt->params.p_not_found;

        // Check if device already be found with NRF_BLE_SCAN_EVT_FILTER_MATCH
        // Here we just want to update the device name as it is not found using NRF_BLE_SCAN_EVT_FILTER_MATCH
        index = get_devices_list_id(p_adv_report->peer_addr);
        if (index != 0xFF) {
            length = ble_advdata_search(p_adv_report->data.p_data, p_adv_report->data.len, &offset, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
            if (length == 0) {
                // Look for the short local name if it was not found as complete.
                length = ble_advdata_search(p_adv_report->data.p_data, p_adv_report->data.len, &offset, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME);
            }
            // update device name
            if (length != 0) {
                memcpy(found_devices[index].name, &p_adv_report->data.p_data[offset], length);
                found_devices[index].name_length = strlen(found_devices[index].name);
            }
        }
    } break;

    case NRF_BLE_SCAN_EVT_FILTER_MATCH: {
        uint8_t index = 0;
        ble_gap_evt_adv_report_t const *p_adv_report = p_scan_evt->params.filter_match.p_adv_report;

        index = get_devices_list_id(p_adv_report->peer_addr);

        // Device not found in the list
        if (index == 0xFF) {
            index = devices_list_index;
            devices_list_index++;
            // insert new device in the list
            strncpy((char *)(found_devices[index].gap_addr.addr), (char *)(p_adv_report->peer_addr.addr), (size_t)6);
            found_devices[index].gap_addr.addr_type = p_adv_report->peer_addr.addr_type;
            found_devices[index].rssi = p_adv_report->rssi;
        }
    } break;

    case NRF_BLE_SCAN_EVT_CONNECTING_ERROR: {
        err_code = p_scan_evt->params.connecting_err.err_code;
        APP_ERROR_CHECK(err_code);
    } break;

    case NRF_BLE_SCAN_EVT_CONNECTED: {
        ble_gap_evt_connected_t const *p_connected = p_scan_evt->params.connected.p_connected;
        // Scan is automatically stopped by the connection.
        NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
            p_connected->peer_addr.addr[0],
            p_connected->peer_addr.addr[1],
            p_connected->peer_addr.addr[2],
            p_connected->peer_addr.addr[3],
            p_connected->peer_addr.addr[4],
            p_connected->peer_addr.addr[5]);
    } break;

    case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: {
        NRF_LOG_INFO("Scan timed out.");
        scan_start();
    } break;

    default:
        break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static uint32_t scan_init(void) {
    ret_code_t err_code;
    nrf_ble_scan_init_t init_scan;

    // Set the default scan parameters.
    m_gap_scan_params.active = 1;
    m_gap_scan_params.interval = APP_SCAN_SCAN_INTERVAL;
    m_gap_scan_params.window = APP_SCAN_SCAN_WINDOW;
    m_gap_scan_params.timeout = APP_SCAN_SCAN_DURATION;
    m_gap_scan_params.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
    m_gap_scan_params.scan_phys = BLE_GAP_PHY_1MBPS;

    memset(&init_scan, 0, sizeof(init_scan));
    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param = &m_gap_scan_params;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
#endif

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
static void ble_nus_data_evt_handler(ble_nus_evt_t *p_evt) {
    if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
        uint32_t err_code;

        err_code = ser_pkt_fw_tx_send(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length, SER_PKT_FW_PORT_BLE);

        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r') {
            while (ser_pkt_fw_tx_send("\n", 1, SER_PKT_FW_PORT_BLE) == NRF_ERROR_BUSY)
                ;
        }
    }
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

#if defined(BLE_CAP_CENTRAL)
static void ble_nus_c_evt_handler(ble_nus_c_t *p_ble_nus_c, ble_nus_c_evt_t const *p_ble_nus_evt) {
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type) {
    case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        NRF_LOG_INFO("Discovery complete.");
        err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
        APP_ERROR_CHECK(err_code);

        err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Connected to device with Nordic UART Service.");
        break;

    case BLE_NUS_C_EVT_NUS_TX_EVT:
        err_code = ser_pkt_fw_tx_send(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len, SER_PKT_FW_PORT_BLE);

        if (p_ble_nus_evt->p_data[p_ble_nus_evt->data_len - 1] == '\r') {
            while (ser_pkt_fw_tx_send("\n", 1, SER_PKT_FW_PORT_BLE) == NRF_ERROR_BUSY)
                ;
        }
        break;

    case BLE_NUS_C_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
        scan_start();
        break;
    }
}
#endif // defined(BLE_CAP_CENTRAL)

/**@brief Function for initializing services that will be used by the application.
 */
static uint32_t services_init(void) {
    uint32_t err_code;
    ble_nus_init_t nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    VERIFY_SUCCESS(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = ble_nus_data_evt_handler;

    err_code = ble_nus_init(&m_ble_nus, &nus_init);
    VERIFY_SUCCESS(err_code);

#if defined(BLE_CAP_CENTRAL)
    // Initialize NUS client.
    ble_nus_c_init_t nus_c_init;
    nus_c_init.evt_handler = ble_nus_c_evt_handler;
    nus_c_init.error_handler = nus_error_handler;
    nus_c_init.p_gatt_queue = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &nus_c_init);
    VERIFY_SUCCESS(err_code);
#endif // defined(BLE_CAP_CENTRAL)

    return NRF_SUCCESS;
}

uint32_t ble_manager_init(ble_init_cfg_t *init_cfg, ble_manager_evt_handler_t evt_handler) {
    uint32_t err_code;
    flash_manager_ble_cfg_t *flash;

    if (init_cfg == NULL) {
        return NRF_ERROR_NULL;
    }

    // Check and Register event
    if (evt_handler == NULL) {
        return NRF_ERROR_NULL;
    }
    m_evt_handler = evt_handler;

    // Inititiale BLE
    err_code = ble_stack_init();
    VERIFY_SUCCESS(err_code);

    m_adv_interval = init_cfg->advparam; // Needed
    err_code = gap_params_init(init_cfg->name, init_cfg->name_length, init_cfg->gap_conn_params);
    VERIFY_SUCCESS(err_code);

    err_code = gatt_init();
    VERIFY_SUCCESS(err_code);

    err_code = conn_params_init(init_cfg->gap_conn_params);
    VERIFY_SUCCESS(err_code);

#if defined(BLE_CAP_CENTRAL)
    err_code = db_discovery_init();
    VERIFY_SUCCESS(err_code);

    err_code = scan_init();
    VERIFY_SUCCESS(err_code);
#endif // defined(BLE_CAP_CENTRAL)

    err_code = services_init();
    VERIFY_SUCCESS(err_code);

    err_code = advertising_init(init_cfg->advparam);
    VERIFY_SUCCESS(err_code);

    err_code = ble_dcdc_set(init_cfg->dcdc_mode);
    VERIFY_SUCCESS(err_code);

    err_code = ble_txp_set(init_cfg->txp);
    VERIFY_SUCCESS(err_code);

    // Register to serial packet forwarder
    err_code = ser_pkt_fw_path_add(ser_pkt_fw_event_handler, SER_PKT_FW_PORT_BLE);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t ble_connection_state_get(uint8_t *state) {
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        *state = 0;
    } else {
        *state = ble_conn_state_role(m_conn_handle);
    }

    return NRF_SUCCESS;
}

uint32_t ble_dcdc_set(uint8_t dcdc_mode) {
    uint32_t err_code;

    // Enable DCDC
    err_code = sd_power_dcdc_mode_set(dcdc_mode);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_txp_set(int8_t txp) {
    uint32_t err_code;

    // Change TXP
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, txp);
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, txp);
    }
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_phy_set(uint8_t phy_tx, uint8_t phy_rx) {
    uint32_t err_code;
    ble_gap_phys_t temp_phys;

    // NOTE: current implementation change PHY only if connected
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        // Request PHY change
        temp_phys.tx_phys = phy_tx;
        temp_phys.rx_phys = phy_rx;

        err_code = sd_ble_gap_phy_update(m_conn_handle, &temp_phys);
        VERIFY_SUCCESS(err_code);
    } else {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t ble_advparam_set(uint16_t interval) {
    uint32_t err_code;
    ble_adv_mode_t current_mode = m_advertising.adv_mode_current;

    // If device is advertising, Stop it before reconfiguration
    if (current_mode != BLE_ADV_MODE_IDLE) {
        sd_ble_gap_adv_stop(m_advertising.adv_handle);
    }

    advertising_init(interval);
    m_adv_interval = interval;

    // If device was advertising, resume
    if (current_mode != BLE_ADV_MODE_IDLE) {
        err_code = ble_advertising_start(&m_advertising, current_mode);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t ble_connparam_set(float conn_interval_min, float conn_interval_max, uint16_t conn_latency, uint16_t conn_timeout) {
    uint32_t err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t temp_gap_conn_params;

    temp_gap_conn_params.min_conn_interval = MSEC_TO_UNITS(conn_interval_min, UNIT_1_25_MS);
    temp_gap_conn_params.max_conn_interval = MSEC_TO_UNITS(conn_interval_max, UNIT_1_25_MS);
    temp_gap_conn_params.slave_latency = conn_latency;
    temp_gap_conn_params.conn_sup_timeout = MSEC_TO_UNITS(conn_timeout, UNIT_10_MS);

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        ble_conn_params_stop();

        memset(&cp_init, 0, sizeof(cp_init));
        cp_init.p_conn_params = &temp_gap_conn_params;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail = false;
        cp_init.evt_handler = on_conn_params_evt;
        cp_init.error_handler = conn_params_error_handler;
        err_code = ble_conn_params_init(&cp_init);
        VERIFY_SUCCESS(err_code);
    }
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        // Device is already connected, request connection parameters update to the peer
        err_code = ble_conn_params_change_conn_params(m_conn_handle, &temp_gap_conn_params);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t ble_addr_get(uint8_t *addr) {
    uint32_t err_code;
    ble_gap_addr_t gap_addr;

    err_code = sd_ble_gap_addr_get(&gap_addr);
    VERIFY_SUCCESS(err_code);

    memcpy(addr, &gap_addr.addr, BLE_GAP_ADDR_LEN);

    return NRF_SUCCESS;
}

uint32_t ble_name_set(uint8_t *name, uint16_t length) {
    uint32_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_advertising_init_t init;
    ble_adv_mode_t current_mode = m_advertising.adv_mode_current;

    // If device is advertising, Stop it before reconfiguration
    if (current_mode != BLE_ADV_MODE_IDLE) {
        sd_ble_gap_adv_stop(m_advertising.adv_handle);
    }

    // Change name
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)name, length);
    VERIFY_SUCCESS(err_code);

    // Re encode ad/sr packet
    advertising_init(m_adv_interval);

    // If device was advertising, resume
    if (current_mode != BLE_ADV_MODE_IDLE) {
        ble_advertising_start(&m_advertising, current_mode);
    }

    return NRF_SUCCESS;
}

uint32_t ble_name_get(uint8_t *name, uint16_t *length) {
    uint32_t err_code;
    uint8_t temp_name[31];
    uint16_t temp_length = 31;

    err_code = sd_ble_gap_device_name_get(temp_name, &temp_length);
    VERIFY_SUCCESS(err_code);

    strncpy(name, temp_name, temp_length);
    *length = temp_length;

    return NRF_SUCCESS;
}

uint32_t ble_advertise(uint8_t start) {
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        return NRF_ERROR_INVALID_STATE;
    }

    if (start == BLE_ADV_START) {
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        VERIFY_SUCCESS(err_code);
    } else {
        m_advertising.adv_mode_current = BLE_ADV_MODE_IDLE;
        err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t ble_disconnect() {
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        VERIFY_SUCCESS(err_code);
    } else {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t ble_rssi_get(int8_t *rssi) {
    uint32_t err_code;
    uint8_t ch_index;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        err_code = sd_ble_gap_rssi_get(m_conn_handle, rssi, &ch_index);
        VERIFY_SUCCESS(err_code);
    } else {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t ble_restore(void) {
    uint32_t err_code;
    flash_manager_ble_cfg_t *flash;

    err_code = flash_manager_ble_cfg_restore();
    VERIFY_SUCCESS(err_code);

    // wait for flash to be ready
    while (is_flash_manager_ready() == false) {
        nrf_pwr_mgmt_run();
    }

    return NRF_SUCCESS;
}

uint32_t ble_scan(uint8_t start) {
#if defined(BLE_CAP_CENTRAL)
    if (start == BLE_SCAN_START) {
        scan_start();
    } else {
        scan_stop();
    }

    return NRF_SUCCESS;
#else
    return NRF_ERROR_FORBIDDEN;
#endif
}

uint32_t ble_scan_list(device_info_t *list, uint8_t *nb_devices_found) {
#if defined(BLE_CAP_CENTRAL)
    *nb_devices_found = devices_list_index;

    for (int i = 0; i < devices_list_index; i++) {
        list[i] = found_devices[i];
    }

    return NRF_SUCCESS;
#else
    return NRF_ERROR_FORBIDDEN;
#endif
}

uint32_t ble_connect(uint8_t *addr, ble_gap_conn_params_t gap_conn_params) {
#if defined(BLE_CAP_CENTRAL)
    uint32_t err_code;
    ble_gap_addr_t gap_addr;
    memcpy(gap_addr.addr, addr, BLE_GAP_ADDR_LEN);
    gap_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC; // only support BLE_GAP_ADDR_TYPE_RANDOM_STATIC

    // Stop scanning.
    nrf_ble_scan_stop();

    sd_ble_gap_connect_cancel();

    // Establish connection.
    err_code = sd_ble_gap_connect(&gap_addr, &m_gap_scan_params, &gap_conn_params, APP_BLE_CONN_CFG_TAG);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
#else
    return NRF_ERROR_FORBIDDEN;
#endif
}