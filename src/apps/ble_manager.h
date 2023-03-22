/******************************************************************************
 * @file    ble_manager.h
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

#ifndef BLE_MANAGER_H__
#define BLE_MANAGER_H__

#include "ble_conn_params.h"
#include <stdint.h>

#define SER_PKT_FW_PORT_BLE 1

#define MAX_SCAN_DEVICE_LIST 10
#define BLE_PERIPHERAL 0
#define BLE_CENTRAL 1
#define BLE_ADV_START 1
#define BLE_ADV_STOP 0
#define BLE_SCAN_START 1
#define BLE_SCAN_STOP 0
#define BLE_TXP_M40DBM -40
#define BLE_TXP_M20DBM -20
#define BLE_TXP_M16DBM -16
#define BLE_TXP_M12DBM -12
#define BLE_TXP_M8DBM -8
#define BLE_TXP_M4DBM -4
#define BLE_TXP_0DBM 0
#define BLE_TXP_2DBM 2
#define BLE_TXP_3DBM 3
#define BLE_TXP_4DBM 4
#define BLE_TXP_5DBM 5
#define BLE_TXP_6DBM 6
#define BLE_TXP_7DBM 7
#define BLE_TXP_8DBM 8
#define DCDC_ON 1
#define DCDC_OFF 0

/**@brief Structure containing scan result
 */
typedef struct
{
    ble_gap_addr_t gap_addr;
    uint8_t name[26];
    uint8_t name_length;
    int8_t rssi;
} device_info_t;


/**@brief Structure defining the init BLE configuration data.
 */
typedef struct
{
    uint8_t role;
    uint8_t dcdc_mode;
    int8_t txp;
    ble_gap_phys_t phys;
    uint16_t name_length;
    uint8_t name[31];
    uint16_t advparam;
    ble_gap_conn_params_t gap_conn_params;
}ble_init_cfg_t;


/**@brief BLE manage event types. */
typedef enum {
    BLE_MANAGER_EVT_PHY_CHANGED = 0,     /**< PHY changed event */
    BLE_MANAGER_EVT_CONN_PARAMS_CHANGED, /**< Connection parameters changed event */
    BLE_MANAGER_EVT_TYPE_MAX             /**< Enumeration upper bound. */
} ble_manager_evt_type_t;

/**@brief A struct containing events from a BLE manager.
 *
 * @note  Some events do not have parameters, then whole information is contained in the evt_type.
 */
typedef struct
{
    ble_manager_evt_type_t evt_type; /**< Type of event. */

      union  /**< Union alternative identified by evt_type in enclosing struct. */
      {
          ble_gap_phys_t phy;
          ble_gap_conn_params_t conn_params;
    //      ser_phy_evt_hw_error_params_t        hw_error;
      } evt_params;
} ble_manager_evt_t;

/**@brief Callback function handler for the BLE manager.
 *
 * @param[in] event    BLE manager event.
 */
typedef void (*ble_manager_evt_handler_t)(ble_manager_evt_t evt);

/**@brief Function for opening and initializing the BLE manager.
 */
uint32_t ble_manager_init(ble_init_cfg_t *init_cfg, ble_manager_evt_handler_t evt_handler);

/**@brief Function for getting the connection state.
 *
 * @param[out] state                connection state.
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_connection_state_get(uint8_t *state);

/**@brief Function for setting the DCDC mode.
 *
 * @param[in] mode                  DCDC enabled or disabled
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Wrong parameter.
 */
uint32_t ble_dcdc_set(uint8_t mode);

/**@brief Function for setting the transmit power.
 *
 * @param[in] power                  power in dBm
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Wrong parameter.
 */
uint32_t ble_txp_set(int8_t txp);

/**@brief Function for setting the phy.
 *
 * @param[in] phy_tx                 TX PHY slection, 0:Auto, 1:1Mbps, 2:2Mbps, 4:coded
 * @param[in] phy_rx                 RX PHY slection, 0:Auto, 1:1Mbps, 2:2Mbps, 4:coded
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Wrong parameter.
 */
uint32_t ble_phy_set(uint8_t phy_tx, uint8_t phy_rx);

/**@brief Function for setting the advertising parameters.
 *
 * @param[in] interval              Interval in ms
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Wrong parameter.
 */
uint32_t ble_advparam_set(uint16_t interval);

/**@brief Function for setting the connection parameters.
 *
 * @param[in] conn_interval_min     Interval min in ms
 * @param[in] conn_interval_max     Interval max in ms
 * @param[in] conn_latency          Slave latency
 * @param[in] conn_timeout          timeout in ms
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Wrong parameter.
 */
uint32_t ble_connparam_set(float conn_interval_min, float conn_interval_max, uint16_t conn_latency, uint16_t conn_timeout);

/**@brief Function for setting the device name.
 *
 * @param[in] name                  pointer to device name
 * @param[in] length                device name length
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Wrong parameter.
 */
uint32_t ble_name_set(uint8_t *name, uint16_t length);

/**@brief Function for getting the advertising parameters.
 *
 * @param[out] name                 pointer to device name
 * @param[out] length               pointer to device length
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_name_get(uint8_t *name, uint16_t *length);

/**@brief Function for starting/stopping advertising.
 *
 * @param[in] start                 start/stop advertising
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Wrong parameter.
 */
uint32_t ble_advertise(uint8_t start);

/**@brief Function for disconnecting from the peer.
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. Not connected
 */
uint32_t ble_disconnect();

/**@brief Function for getting the last rssi.
 *
 * @param[out] name                 pointer to device name
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_rssi_get(int8_t *rssi);

/**@brief Function for getting the device BLE address.
 *
 * @param[out] addr                 pointer to address
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_addr_get(uint8_t *addr);

/**@brief Function for restauring default parameters.
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_restore(void);

/**@brief Function for starting or stopping scan.
 *
 * @param[in] start                 start/stop scannning
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_scan(uint8_t start);

/**@brief Function for getting result of the scan.
 *
 * @param[out] list                 pointer to the lidt of devices
 * @param[out] nb_devices           number of devices found
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_scan_list(device_info_t *list, uint8_t *nb_devices);

/**@brief Function for connecting to a peripheral.
 *
 * @param[in] addr                  Address of the device to connect to
 * @param[in] gap_conn_params       Connection parameters
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ble_connect(uint8_t *addr, ble_gap_conn_params_t gap_conn_params);

#endif