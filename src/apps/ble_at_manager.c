/******************************************************************************
 * @file    ble_at_manager.c
 * @author  Insight SiP
 * @brief  at commands for BLE
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

#include "ble_at_manager.h"
#include "app_error.h"
#include "ble_manager.h"
#include "boards.h"
#include "flash_manager.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "ser_pkt_fw.h"
#include <stdbool.h>
#include <string.h>

/**
 * @brief Macro for creating a new AT Command
 */
#define AT_COMMAND_DEF(_name, _set, _read, _test) \
    {                                             \
        .name = _name,                            \
        .name_size = sizeof(_name) - 1,           \
        .set = _set,                              \
        .read = _read,                            \
        .test = _test,                            \
    }

#define CONVERT_NRF_TO_AT_ERROR(nrf_error, at_error)       \
    do {                                                   \
        if (nrf_error == NRF_SUCCESS) {                    \
            at_error = AT_OK;                              \
        } else if (nrf_error == NRF_ERROR_INVALID_PARAM) { \
            at_error = AT_ERROR_INVALID_PARAM;             \
        } else if (nrf_error == NRF_ERROR_BUSY) {          \
            at_error = AT_ERROR_BUSY;                      \
        } else if (nrf_error == NRF_ERROR_INVALID_STATE) { \
            at_error = AT_ERROR_INVALID_STATE;             \
        } else {                                           \
            at_error = AT_ERROR_OTHER;                     \
        }                                                  \
    } while (0)

#define AT_VERIFY_SUCCESS(err_code) \
    do {                            \
        if (!(err_code == AT_OK)) { \
            return err_code;        \
        }                           \
    } while (0)

/**
 * @brief  Structure defining an AT Command
 */
typedef struct
{
    const uint8_t *name;                           /*< command name, after the "AT" */
    const uint8_t name_size;                       /*< size of the command name, not including the final \r or \n */
    at_error_code_t (*set)(const uint8_t *param);  /*< = after the string to set a new value, or \0 if not parameters*/
    at_error_code_t (*read)(const uint8_t *param); /*< ? after the name to get the current value*/
    at_error_code_t (*test)(const uint8_t *param); /*< =? test command */
} at_command_t;

static bool m_at_command_ready = false;
static uint8_t m_echo = 0;
static uint8_t m_rx_at_command[128] = {0};
static uint8_t m_tx_buffer[128] = {0};
static uint8_t m_current_role = BLE_PERIPHERAL;

/**
 * @brief  Array corresponding to the description of each possible AT Error
 */
static const uint8_t *at_error_description[] =
    {
        "OK\r\n",                   /* AT_OK */
        "UNKNOWN_CMD\r\n",          /* AT_UNKNOWN_CMD */
        "ERROR_INTERNAL\r\n",       /* AT_ERROR_INTERNA */
        "ERROR_NOT_FOUND\r\n",      /* AT_ERROR_NOT_FOUND */
        "ERROR_NOT_SUPPORTED\r\n",  /* AT_ERROR_NOT_SUPPORTED */
        "ERROR_INVALID_PARAM\r\n",  /* AT_ERROR_INVALID_PARAM */
        "ERROR_INVALID_STATE\r\n",  /* AT_ERROR_INVALID_STATE */
        "ERROR_INVALID_LENGTH\r\n", /* AT_ERROR_INVALID_LENGTH */
        "ERROR_INVALID_FLAGS\r\n",  /* AT_ERROR_INVALID_FLAGS */
        "ERROR_INVALID_DATA\r\n",   /* AT_ERROR_INVALID_DATA */
        "ERROR_DATA_SIZE\r\n",      /* AT_ERROR_DATA_SIZE */
        "ERROR_TIMEOUT\r\n",        /* AT_ERROR_TIMEOUT */
        "ERROR_NULL\r\n",           /* AT_ERROR_NULL */
        "ERROR_FORBIDDEN \r\n",     /* AT_ERROR_FORBIDDEN */
        "ERROR_BUSY\r\n",           /* AT_ERROR_BUSY */
        "ERROR\r\n",                /* AT_MAX */
};

static at_error_code_t at_error_not_supported(const uint8_t *param) {
    return AT_ERROR_NOT_SUPPORTED;
}

static at_error_code_t at_error_supported(const uint8_t *param) {
    return AT_OK;
}

static at_error_code_t at_reset(const uint8_t *param) {
    NVIC_SystemReset();

    return AT_OK;
}

static at_error_code_t at_echo_set(const uint8_t *param) {
    int echo;

    // Check parameters
    if (sscanf(param, "%u", &echo) != 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    echo = (uint8_t)echo;

    if (echo > 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    m_echo = echo;

    return AT_OK;
}

at_error_code_t at_echo_read(const uint8_t *param) {
    // Send response
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_ECHO, m_echo);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_info_read(const uint8_t *param) {
    // Send module name
    sprintf(m_tx_buffer, "%s\r\n", MODULE_NAME);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    // Send device id
    sprintf(m_tx_buffer, "%X%X\r\n", NRF_FICR->DEVICEID[0], NRF_FICR->DEVICEID[1]);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    // Send firmware version
    sprintf(m_tx_buffer, "%s\r\n", FW_REVISION);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

static at_error_code_t at_deepsleep_set(const uint8_t *param) {
    ser_pkt_fw_release();
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);

    return AT_OK;
}

static at_error_code_t at_factory_reset(const uint8_t *param) {
    ble_manager_restore();
    ser_pkt_fw_restore();

    NVIC_SystemReset();

    return AT_OK;
}

static at_error_code_t at_connstate_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t state;

    // Read value
    err_code = ble_manager_connstate_read(&state);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    sprintf(m_tx_buffer, "%s: %u\r\n", AT_BLE_CONNSTATE, state);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_dcdc_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int dcdc_mode;

    // Check parameters
    if (sscanf(param, "%u", &dcdc_mode) != 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    dcdc_mode = (uint8_t)dcdc_mode;

    if (dcdc_mode > 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    // Run command
    err_code = ble_manager_dcdc_set(dcdc_mode);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_dcdc_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t dcdc_mode;

    // Read value
    err_code = ble_manager_dcdc_read(&dcdc_mode);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // Send response
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_DCDC, dcdc_mode);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_dcdc_test(const uint8_t *param) {
    // Send response
    sprintf(m_tx_buffer, "%s: (0,1)\r\n", AT_DCDC);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_version_read(const uint8_t *param) {
    // Send response
    sprintf(m_tx_buffer, "%s: %s\r\n", AT_VERSION, FW_REVISION);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_txp_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int txp;

    // Check parameters
    if (sscanf(param, "%d", &txp) != 1) {
        return AT_ERROR_INVALID_PARAM;
    }

#if (BLE_MAX_TXP_8DBM == 1)
    if ((txp != -40) && (txp != -20) && (txp != -16) && (txp != -12) && (txp != -8) && (txp != -4) && (txp != 0) && (txp != 3) && (txp != 4) &&
        (txp != 5) && (txp != 6) && (txp != 7) && (txp != 8))
#else
    if ((txp != -40) && (txp != -20) && (txp != -16) && (txp != -12) && (txp != -8) && (txp != -4) && (txp != 0) && (txp != 3) && (txp != 4))
#endif
    {
        return AT_ERROR_INVALID_PARAM;
    }

    // Run command
    err_code = ble_manager_txp_set(txp);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_txp_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int8_t txp;

    // Read value
    err_code = ble_manager_txp_read(&txp);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // send response
    sprintf(m_tx_buffer, "%s: %d\r\n", AT_BLE_TXP, txp);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_txp_test(const uint8_t *param) {
    // send response
#if (BLE_MAX_TXP_8DBM == 1)
    sprintf(m_tx_buffer, "%s: (-40,-20,-16,-12,-8,-4,0,3,4,5,6,7,8)\r\n", AT_BLE_TXP);
#else
    sprintf(m_tx_buffer, "%s: (-40,-20,-16,-12,-8,-4,0,3,4)\r\n", AT_BLE_TXP);
#endif
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_phy_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int phy_tx, phy_rx;

    // Check parameters
    if (sscanf(param, "%u,%u", &phy_tx, &phy_rx) != 2) {
        return AT_ERROR_INVALID_PARAM;
    }
#if (BLE_CAP_PHY_CODED == 1)
    if ((phy_tx != 0) && (phy_tx != 1) && (phy_tx != 2) && (phy_tx != 4))
#else
    if ((phy_tx != 0) && (phy_tx != 1) && (phy_tx != 2))
#endif
    {
        return AT_ERROR_INVALID_PARAM;
    }

#if (BLE_CAP_PHY_CODED == 1)
    if ((phy_rx != 0) && (phy_rx != 1) && (phy_rx != 2) && (phy_rx != 4))
#else
    if ((phy_rx != 0) && (phy_rx != 1) && (phy_rx != 2))
#endif
    {
        return AT_ERROR_INVALID_PARAM;
    }

    // Run command
    err_code = ble_manager_phy_set(phy_tx, phy_rx);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_phy_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t phy_tx, phy_rx;

    // Read value
    err_code = ble_manager_phy_read(&phy_tx, &phy_rx);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // send response
    sprintf(m_tx_buffer, "%s: %u,%u\r\n", AT_BLE_PHY, phy_tx, phy_rx);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_phy_test(const uint8_t *param) {
#if (BLE_CAP_PHY_CODED == 1)
    sprintf(m_tx_buffer, "%s: (0,1,2,4),(0,1,2,4)\r\n", AT_BLE_PHY);
#else
    sprintf(m_tx_buffer, "%s: (0,1,2),(0,1,2)\r\n", AT_BLE_PHY);
#endif
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_connparam_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    float interval_min;
    float interval_max;
    uint32_t latency;
    uint32_t timeout;

    // Check parameters
    if (sscanf(param, "%f,%f,%u,%u", &interval_min, &interval_max, &latency, &timeout) != 4) {
        return AT_ERROR_INVALID_PARAM;
    }

    if ((interval_min < 7.5) || (interval_min > 4000) ||
        (interval_max < 7.5) || (interval_max > 4000) ||
        (timeout < 10) || (timeout > 32000) ||
        (latency > 4000)) {
        return AT_ERROR_INVALID_PARAM;
    }

    // Run command
    err_code = ble_manager_connparam_set(interval_min, interval_max, latency, timeout);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_connparam_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    float interval_min;
    float interval_max;
    uint16_t latency;
    uint16_t timeout;

    // Read value
    err_code = ble_manager_connparam_read(&interval_min, &interval_max, &latency, &timeout);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // send response
    sprintf(m_tx_buffer, "%s: %.2f,%.2f,%u,%u\r\n", AT_BLE_CONNPARAM, interval_min, interval_max, latency, timeout);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_connparam_test(const uint8_t *param) {
    sprintf(m_tx_buffer, "%s: (7.5-4000),(7.5-4000),(0-500),(10-32000)\r\n", AT_BLE_CONNPARAM);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_addr_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t devaddr[6];

    // Check parameters
    //    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
    //             &devaddr[0], &devaddr[1], &devaddr[2], &devaddr[3], &devaddr[4], &devaddr[5]) != 6)
    //    {
    //        return AT_ERROR_INVALID_PARAM;
    //    }

    // Run command
    //    err_code = ble_manager_addr_set(devaddr);
    //    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    //    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_addr_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t devaddr[6];

    err_code = ble_manager_addr_read(devaddr);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    sprintf(m_tx_buffer, "%s: %02X-%02X-%02X-%02X-%02X-%02X\r\n",
        AT_BLE_ADDR, devaddr[0], devaddr[1], devaddr[2], devaddr[3], devaddr[4], devaddr[5]);

    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_addr_test(const uint8_t *param) {
    sprintf(m_tx_buffer, "%s: hh-hh-hh-hh-hh-hh\r\n", AT_BLE_ADDR);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_uart_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int flowcontrol;
    int baudrate;

    if (sscanf(param, "%u,%u", &flowcontrol, &baudrate) != 2) {
        return AT_ERROR_INVALID_PARAM;
    }

    if ((flowcontrol > 1) ||
        (baudrate != 1200) && (baudrate != 2400) && (baudrate != 4800) && (baudrate != 9600) &&
            (baudrate != 19200) && (baudrate != 38400) && (baudrate != 57600) && (baudrate != 115200) &&
            (baudrate != 230400) && (baudrate != 460800) && (baudrate != 921600) && (baudrate != 1000000)) {
        return AT_ERROR_INVALID_PARAM;
    }

    // Run command
    err_code = ser_pkt_fw_path_configure(SER_PKT_FW_PORT_BLE, flowcontrol, baudrate);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_uart_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t flowcontrol;
    uint32_t baudrate;

    // Read value
    err_code = ser_pkt_fw_path_check(SER_PKT_FW_PORT_BLE, &flowcontrol, &baudrate);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // send response
    sprintf(m_tx_buffer, "%s: %u,%u\r\n", AT_UART, flowcontrol, baudrate);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_uart_test(const uint8_t *param) {
    sprintf(m_tx_buffer, "%s: (0,1),(1200,2400,4800,9600,19200,38400,57600,115200,230400,460800,921600,1000000)\r\n", AT_UART);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_disconnect_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;

    // Run command
    err_code = ble_manager_disconnect();
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_rssi_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int8_t rssi;

    // Read value
    err_code = ble_manager_rssi_read(&rssi);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // send response
    sprintf(m_tx_buffer, "%s: %d\r\n", AT_BLE_RSSI, rssi);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_role_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t new_role, conn_state;

    // Check parameters
    if (sscanf(param, "%u", &new_role) != 1) {
        return AT_ERROR_INVALID_PARAM;
    }
    if (new_role > 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    // disconnect if a connection is active in the wrong role
    ble_manager_connstate_read(&conn_state);
    if ((conn_state == BLE_GAP_ROLE_PERIPH && new_role == BLE_CENTRAL) ||
        (conn_state == BLE_GAP_ROLE_CENTRAL && new_role == BLE_PERIPHERAL)) {
        ble_manager_disconnect();
    }
    // Stop advertising or scan
    if (conn_state == 0 && m_current_role == BLE_PERIPHERAL) {
        ble_manager_advertise(BLE_ADV_STOP);
    }
    if (conn_state == 0 && m_current_role == BLE_CENTRAL) {
        // TODO stop scan
    }

    m_current_role = new_role;

    // TODO do we start adv or scan here ?
    // TODO store role in flash

    return AT_OK;
}

at_error_code_t at_role_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t flowcontrol;
    uint32_t baudrate;

    // send response
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_BLE_ROLE, m_current_role);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_role_test(const uint8_t *param) {
    sprintf(m_tx_buffer, "%s: (0=Peripheral,1=Central)\r\n", AT_BLE_ROLE);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_name_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t *name = param;

    // Check that role is Peripheral
    if (m_current_role != BLE_PERIPHERAL) {
        return AT_ERROR_FORBIDDEN;
    }

    if (strlen(name) > 27) {
        return AT_ERROR_INVALID_LENGTH;
    }

    // Run command
    err_code = ble_manager_name_set(name);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_name_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t name[31] = "";

    // Check that role is Peripheral
    if (m_current_role != BLE_PERIPHERAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Read value
    err_code = ble_manager_name_read(name);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // send response
    sprintf(m_tx_buffer, "%s: %s\r\n", AT_BLE_NAME, name);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_advparam_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int interval;

    // Check that role is Peripheral
    if (m_current_role != BLE_PERIPHERAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Check parameters
    if (sscanf(param, "%u ,%u", &interval) != 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    if ((interval < 20) || (interval > 10240)) {
        return AT_ERROR_INVALID_PARAM;
    }

    // Run command
    err_code = ble_manager_advparam_set(interval);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_advparam_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint16_t interval;

    // Check that role is Peripheral
    if (m_current_role != BLE_PERIPHERAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Read value
    err_code = ble_manager_advparam_read(&interval);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // send response
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_BLE_ADVPARAM, interval);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_advparam_test(const uint8_t *param) {
    // Check that role is Peripheral
    if (m_current_role != BLE_PERIPHERAL) {
        return AT_ERROR_FORBIDDEN;
    }

    sprintf(m_tx_buffer, "%s: (20-10240)\r\n", AT_BLE_ADVPARAM);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_advertise_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    int start = 0;

    // Check that role is Peripheral
    if (m_current_role != BLE_PERIPHERAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Check parameters
    if (sscanf(param, "%u", &start) != 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    start = (uint8_t)start;

    if (start > 1) {
        return AT_ERROR_INVALID_PARAM;
    }

    // Run command
    err_code = ble_manager_advertise(start);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_advertise_test(const uint8_t *param) {
    // Check that role is Peripheral
    if (m_current_role != BLE_PERIPHERAL) {
        return AT_ERROR_FORBIDDEN;
    }

    sprintf(m_tx_buffer, "%s: (0,1)\r\n", AT_BLE_ADVERTISE);
    ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);

    return AT_OK;
}

at_error_code_t at_scan_start_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;

    // Check that role is Central
    if (m_current_role != BLE_CENTRAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Run command
    err_code = ble_manager_scan(BLE_SCAN_START);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_scan_stop_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;

    // Check that role is Central
    if (m_current_role != BLE_CENTRAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Run command
    err_code = ble_manager_scan(BLE_SCAN_STOP);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_scan_list_read(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    device_info_t scan_list[MAX_SCAN_DEVICE_LIST];
    uint8_t nb_device_found;
    uint8_t addr[BLE_GAP_ADDR_LEN];
    uint8_t name[26];

    // Check that role is Central
    if (m_current_role != BLE_CENTRAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Read value
    err_code = ble_manager_scan_list(scan_list, &nb_device_found);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    // Send response
    for (int i=0; i<nb_device_found; i++) {
        memset(name, 0, 26);
        strncpy(name, scan_list[i].name, scan_list[i].name_length);
        memcpy(addr, scan_list[i].gap_addr.addr, BLE_GAP_ADDR_LEN);
        
        sprintf(m_tx_buffer, "%s: %d, %s, 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\r\n", AT_BLE_SCANLIST, scan_list[i].rssi, name, addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
        ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);
    }

    return AT_OK;
}

at_error_code_t at_connect_set(const uint8_t *param) {
    uint32_t err_code;
    at_error_code_t at_err_code;
    uint8_t addr[BLE_GAP_ADDR_LEN];

    // Check that role is Central
    if (m_current_role != BLE_CENTRAL) {
        return AT_ERROR_FORBIDDEN;
    }

    // Check parameters
    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx", &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]) != 6) {
        return AT_ERROR_INVALID_PARAM;
    }


    // Run command
    err_code = ble_manager_connect(addr);
    CONVERT_NRF_TO_AT_ERROR(err_code, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

/**
 * @brief  List of all supported AT Commands
 */
static at_command_t at_commands[] =
    {
        AT_COMMAND_DEF(AT_RESET, at_reset, at_error_not_supported, at_error_supported),
        AT_COMMAND_DEF(AT_ECHO, at_echo_set, at_echo_read, at_error_supported),
        AT_COMMAND_DEF(AT_INFO, at_error_not_supported, at_info_read, at_error_supported),
        AT_COMMAND_DEF(AT_DEEPSLEEP, at_deepsleep_set, at_error_not_supported, at_error_supported),
        AT_COMMAND_DEF(AT_DCDC, at_dcdc_set, at_dcdc_read, at_dcdc_test),
        AT_COMMAND_DEF(AT_VERSION, at_error_not_supported, at_version_read, at_error_supported),
        AT_COMMAND_DEF(AT_UART, at_uart_set, at_uart_read, at_uart_test),
        AT_COMMAND_DEF(AT_FACTORYRESET, at_factory_reset, at_error_not_supported, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_CONNSTATE, at_error_not_supported, at_connstate_read, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_TXP, at_txp_set, at_txp_read, at_txp_test),
        AT_COMMAND_DEF(AT_BLE_PHY, at_phy_set, at_phy_read, at_phy_test),
        AT_COMMAND_DEF(AT_BLE_CONNPARAM, at_connparam_set, at_connparam_read, at_connparam_test),
        AT_COMMAND_DEF(AT_BLE_ADDR, at_error_not_supported, at_addr_read, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_RSSI, at_error_not_supported, at_rssi_read, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_ROLE, at_role_set, at_role_read, at_role_test),
        AT_COMMAND_DEF(AT_BLE_DISCONNECT, at_disconnect_set, at_error_not_supported, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_ADVERTISE, at_advertise_set, at_error_not_supported, at_advertise_test),
        AT_COMMAND_DEF(AT_BLE_NAME, at_name_set, at_name_read, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_ADVPARAM, at_advparam_set, at_advparam_read, at_advparam_test),
#if defined(BLE_CAP_CENTRAL)
        AT_COMMAND_DEF(AT_BLE_SCANSTART, at_scan_start_set, at_error_not_supported, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_SCANSTOP, at_scan_stop_set, at_error_not_supported, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_SCANLIST, at_error_not_supported, at_scan_list_read, at_error_supported),
        AT_COMMAND_DEF(AT_BLE_CONNECT, at_connect_set, at_error_not_supported, at_error_supported),
#endif // defined(BLE_CAP_CENTRAL)
};

/**
 * @brief  Send final response
 * @param[in]
 */
static void final_response_send(at_error_code_t err_code) {
    if (err_code > AT_ERROR_OTHER) {
        err_code = AT_ERROR_OTHER;
    }
    ser_pkt_fw_tx_send(at_error_description[err_code], strlen(at_error_description[err_code]), SER_PKT_FW_PORT_AT);
}

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
        memcpy(m_rx_at_command, event.evt_params.rx_pkt_received.p_buffer, event.evt_params.rx_pkt_received.num_of_bytes);
        m_at_command_ready = true;
        ble_at_manager_execute();
        break;
    }

    case SER_PKT_FW_EVT_PHY_ERROR: {
        // APP_ERROR_CHECK(NRF_ERROR_FORBIDDEN);
        NRF_LOG_ERROR("SER_PKT_FW_EVT_PHY_ERROR");
        break;
    }

    default: {
        /* do nothing */
        break;
    }
    }
}

at_error_code_t ble_at_manager_execute() {
    at_error_code_t err_code;
    at_command_t *current_at_command;
    uint8_t *p_data;

    if (m_at_command_ready) {
        // If enabled echo command
        if (m_echo) {
            sprintf(m_tx_buffer, "%s", m_rx_at_command);
            m_tx_buffer[strcspn(m_tx_buffer, "\r\n")] = 0;
            strcat(m_tx_buffer, "\r\n");
            ser_pkt_fw_tx_send(m_tx_buffer, strlen(m_tx_buffer), SER_PKT_FW_PORT_AT);
        }
        // Verify that command starts with AT
        if ((m_rx_at_command[0] != 'A') || (m_rx_at_command[1] != 'T')) {
            err_code = AT_ERROR_UNKNOWN_CMD;
        }
        // Search AT commands in the list and execute correponding function
        else {
            err_code = AT_ERROR_UNKNOWN_CMD;
            p_data = m_rx_at_command + 2;

            if (p_data[0] == '\r' || p_data[0] == '\n') {
                err_code = AT_OK;
            } else {
                for (int i = 0; i < (sizeof(at_commands) / sizeof(at_command_t)); i++) {
                    if (strncmp(p_data, at_commands[i].name, at_commands[i].name_size) == 0) {
                        // Command found
                        current_at_command = &(at_commands[i]);
                        p_data += current_at_command->name_size;

                        //  Parse the type (set, read or test), and jump to the corresponding function
                        if (p_data[0] == '\r' || p_data[0] == '\n') {
                            err_code = current_at_command->set(p_data + 1);
                        } else if (p_data[0] == '?') {
                            err_code = current_at_command->read(p_data + 1);
                        } else if (p_data[0] == '=') {
                            if (p_data[1] == '?') {
                                err_code = current_at_command->test(p_data + 1);
                            } else if (p_data[1] != '\r' && p_data[1] != '\n') {
                                err_code = current_at_command->set(p_data + 1);
                            }
                        }
                        // We end the loop as the command was found
                        break;
                    }
                }
            }
        }

        // Send final response
        final_response_send(err_code);

        memset(m_rx_at_command, 0, 128);
        m_at_command_ready = false;
    }
}

at_error_code_t ble_at_manager_init() {
    uint32_t err_code;

    // Initialize flash manager
    err_code = flash_manager_init();
    VERIFY_SUCCESS(err_code);

    // Initialize BLE manager
    err_code = ble_manager_init();
    VERIFY_SUCCESS(err_code);

    // Initialize serial packet fowarder
    err_code = ser_pkt_fw_init(PIN_SER_PKT_FW_SELECT);
    VERIFY_SUCCESS(err_code);

    err_code = ser_pkt_fw_path_add(ser_pkt_fw_event_handler, SER_PKT_FW_PORT_AT);
    VERIFY_SUCCESS(err_code);

    return err_code;
}