 /******************************************************************************
 * @file    ser_pkt_fw.c
 * @author  Insight SiP
 * @version V0.2.0
 * @date    13-03-2020
 * @brief  serial packet forwader
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

#include <stdbool.h>
#include <string.h>
#include "app_error.h"
#include "ser_phy.h"
#include "ser_pkt_fw.h"
#include "nrf_drv_gpiote.h"
#include "nrf_pwr_mgmt.h"
#include "sdk_macros.h"
#include "flash_manager.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



#define SER_PKT_FW_MAX_PORTS    2
#define SER_PKT_FW_DEFAULT_PORT 0
#define SER_PKT_FW_PORT_1       1

/**
 * @brief States of the RX state machine.
 */
typedef enum
{
    SER_PKT_FW_RX_STATE_CLOSED = 0,
    SER_PKT_FW_RX_STATE_IDLE,
    SER_PKT_FW_RX_STATE_RECEIVED,
    SER_PKT_FW_RX_STATE_MAX
}ser_pkt_fw_rx_states_t;

/**
 * @brief States of the TX state machine.
 */
typedef enum
{
    SER_PKT_FW_TX_STATE_CLOSED = 0,
    SER_PKT_FW_TX_STATE_IDLE,
    SER_PKT_FW_TX_STATE_TRANSMITTED,
    SER_PKT_FW_TX_STATE_MAX
}ser_pkt_fw_tx_states_t;

/** @brief RX state. */
static ser_pkt_fw_rx_states_t m_rx_state = SER_PKT_FW_RX_STATE_CLOSED;

/** @brief TX state. */
static ser_pkt_fw_tx_states_t m_tx_state = SER_PKT_FW_TX_STATE_CLOSED;

/** @brief Callback functions handler for Serial packets forwarder events. */
static ser_pkt_fw_events_handler_t m_events_handler[SER_PKT_FW_MAX_PORTS];

/** @brief Forwarder port number */
uint8_t m_ser_pkt_fw_port = 0;

/** @brief Select pin number */
uint32_t m_select_pin = 0xFF;

/** @brief Table containing serial configuration for each port */
ser_pkt_fw_config_t m_ser_config[SER_PKT_FW_MAX_PORTS];



/**
 * @brief A callback function to be used to handle a PHY module events. This function is called in an interrupt context.
 */
static void ser_phy_events_handler(ser_phy_evt_t phy_event)
{
    uint32_t            err_code = 0;
    ser_pkt_fw_evt_t    ser_pkt_fw_event;

    memset(&ser_pkt_fw_event, 0, sizeof (ser_pkt_fw_evt_t));
    ser_pkt_fw_event.evt_type = SER_PKT_FW_EVT_TYPE_MAX;

    switch (phy_event.evt_type)
    {
        case SER_PHY_EVT_TX_PKT_SENT:
        {
            m_tx_state = SER_PKT_FW_TX_STATE_TRANSMITTED;

            /* An event to an upper layer that a packet has been transmitted. */
            ser_pkt_fw_event.evt_type = SER_PKT_FW_EVT_TX_PKT_SENT;
            m_events_handler[m_ser_pkt_fw_port](ser_pkt_fw_event);

            break;
        }

        case SER_PHY_EVT_RX_PKT_RECEIVED:
        {
            m_rx_state = SER_PKT_FW_RX_STATE_RECEIVED;

            /* Generate the event to an upper layer. */
            ser_pkt_fw_event.evt_type = SER_PKT_FW_EVT_RX_PKT_RECEIVED;
            ser_pkt_fw_event.evt_params.rx_pkt_received.p_buffer        = phy_event.evt_params.rx_pkt_received.p_buffer;
            ser_pkt_fw_event.evt_params.rx_pkt_received.num_of_bytes    = phy_event.evt_params.rx_pkt_received.num_of_bytes;
            m_events_handler[m_ser_pkt_fw_port](ser_pkt_fw_event);

            break;
        }

         case SER_PHY_EVT_RX_OVERFLOW_ERROR:
        {
            /* Generate the event to an upper layer. */
            ser_pkt_fw_event.evt_type                        = SER_PKT_FW_EVT_PHY_ERROR;
            ser_pkt_fw_event.evt_params.phy_error.error_type = SER_PKT_FW_PHY_ERROR_RX_OVERFLOW;
            m_events_handler[m_ser_pkt_fw_port](ser_pkt_fw_event);
            break;
        }

        case SER_PHY_EVT_TX_OVERREAD_ERROR:
        {
            /* Generate the event to an upper layer. */
            ser_pkt_fw_event.evt_type                        = SER_PKT_FW_EVT_PHY_ERROR;
            ser_pkt_fw_event.evt_params.phy_error.error_type = SER_PKT_FW_PHY_ERROR_TX_OVERREAD;
            m_events_handler[m_ser_pkt_fw_port](ser_pkt_fw_event);
            break;
        }

        case SER_PHY_EVT_HW_ERROR:
        {
            /* Generate the event to an upper layer. */
            ser_pkt_fw_event.evt_type                           = SER_PKT_FW_EVT_PHY_ERROR;
            ser_pkt_fw_event.evt_params.phy_error.error_type    = SER_PKT_FW_PHY_ERROR_HW_ERROR;
            ser_pkt_fw_event.evt_params.phy_error.hw_error_code = phy_event.evt_params.hw_error.error_code;
            m_events_handler[m_ser_pkt_fw_port](ser_pkt_fw_event);

            break;
        }

        default:
        {
            APP_ERROR_CHECK_BOOL(false);
            break;
        }
    }
}

/**
 * @brief A callback function to be used to handle a GPIOTE events. This function is called in an interrupt context.
 */
static void nrf_drv_gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin != m_select_pin) return;

    CRITICAL_REGION_ENTER();

    if (nrf_gpio_pin_read(pin))
    {
        m_ser_pkt_fw_port = 1;
    }
    else
    {
        m_ser_pkt_fw_port = 0;
    }

    // Reconfigure serial PHY
    ser_phy_close();
    ser_phy_open(m_ser_config[m_ser_pkt_fw_port].flow_control, m_ser_config[m_ser_pkt_fw_port].baudrate, ser_phy_events_handler);

    NRF_LOG_INFO("Serial packet forwarder port: %d", m_ser_pkt_fw_port);

    CRITICAL_REGION_EXIT();
}

uint32_t ser_pkt_fw_init(uint32_t select_pin)
{
    uint32_t err_code = NRF_SUCCESS;
    flash_manager_ser_cfg_t *flash;

    if ((SER_PKT_FW_RX_STATE_CLOSED != m_rx_state) || (SER_PKT_FW_TX_STATE_CLOSED != m_tx_state))
    {
       return NRF_ERROR_INVALID_STATE;
    }
    if ((select_pin) > 32 || (select_pin < 2))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    /* We have to change states before calling lower layer because ser_phy_open() function is going to enable interrupts.
    * On success an event from PHY layer can be emitted immediately after return from ser_phy_open(). */
    m_rx_state = SER_PKT_FW_RX_STATE_IDLE;
    m_tx_state = SER_PKT_FW_TX_STATE_IDLE;

    /* Load configuration from flash */
    err_code = flash_manager_ser_cfg_load(&flash);
    VERIFY_SUCCESS(err_code);

    /* Apply configuration from flash */
    m_ser_config[SER_PKT_FW_PORT_1].flow_control = flash->flow_control;
    m_ser_config[SER_PKT_FW_PORT_1].baudrate = flash->baudrate;

    m_select_pin = select_pin;
    m_ser_config[SER_PKT_FW_DEFAULT_PORT].flow_control = 0;
    m_ser_config[SER_PKT_FW_DEFAULT_PORT].baudrate = 38400;

    /* Initialize the serial PHY module. */
    err_code = ser_phy_open(m_ser_config[SER_PKT_FW_DEFAULT_PORT].flow_control, m_ser_config[SER_PKT_FW_DEFAULT_PORT].baudrate, ser_phy_events_handler);
    if (NRF_SUCCESS != err_code)
    {
        m_rx_state = SER_PKT_FW_RX_STATE_CLOSED;
        m_tx_state = SER_PKT_FW_TX_STATE_CLOSED;

        for (int i=0; i<SER_PKT_FW_MAX_PORTS;i++)
        {
            m_events_handler[i] = NULL;
        }

        if (NRF_ERROR_INVALID_PARAM != err_code)
        {
            err_code = NRF_ERROR_INTERNAL;
        }

        return err_code;
    }

    /* Initialize gpiote */
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    err_code = nrf_drv_gpiote_in_init(select_pin, &in_config,  nrf_drv_gpiote_evt_handler);
    if (NRF_SUCCESS != err_code)
    {    
        m_select_pin = 0xFF;
        return err_code;
    }

    if (nrf_gpio_pin_read(m_select_pin))
    {
        m_ser_pkt_fw_port = 1;
    }
    else
    {
        m_ser_pkt_fw_port = SER_PKT_FW_DEFAULT_PORT;
    }

    nrf_drv_gpiote_in_event_enable(select_pin, true);

    return err_code;
}

uint32_t ser_pkt_fw_path_configure(uint8_t port, uint8_t flow_control, uint32_t baudrate)
{
    uint32_t err_code = NRF_SUCCESS;

    if ((SER_PKT_FW_RX_STATE_CLOSED == m_rx_state) || (SER_PKT_FW_TX_STATE_CLOSED == m_tx_state))
    {
       return NRF_ERROR_INVALID_STATE;
    }

    if (port == SER_PKT_FW_DEFAULT_PORT)
    {
        m_ser_config[port].baudrate = baudrate;
        m_ser_config[port].flow_control = flow_control;

        /* Unitilizalize the serial PHY module */
        ser_phy_close();

        /* Initialize the serial PHY module. */
        err_code = ser_phy_open(m_ser_config[port].flow_control, m_ser_config[port].baudrate, ser_phy_events_handler);
        if (NRF_SUCCESS != err_code)
        {
            m_rx_state = SER_PKT_FW_RX_STATE_CLOSED;
            m_tx_state = SER_PKT_FW_TX_STATE_CLOSED; 

            if (NRF_ERROR_INVALID_PARAM != err_code)
            {
                err_code = NRF_ERROR_INTERNAL;
            }

            return err_code;
        }
    }
    else if (port == SER_PKT_FW_PORT_1)
    {
        /* Update flash if config changed */
        if (m_ser_config[port].baudrate != baudrate || m_ser_config[port].flow_control != flow_control)
        {
            m_ser_config[port].baudrate = baudrate;
            m_ser_config[port].flow_control = flow_control;

            flash_manager_ser_cfg_t flash;
            flash.flow_control = m_ser_config[port].flow_control;
            flash.baudrate = m_ser_config[port].baudrate;
            flash_manager_ser_cfg_store(&flash);
        }
    }

    return err_code;
}

uint32_t ser_pkt_fw_path_check(uint8_t port, uint8_t *flow_control, uint32_t *baudrate)
{
    VERIFY_PARAM_NOT_NULL(flow_control);
    VERIFY_PARAM_NOT_NULL(baudrate);

    *baudrate = m_ser_config[port].baudrate;
    *flow_control = m_ser_config[port].flow_control;

    return NRF_SUCCESS;
}

void ser_pkt_fw_release(void)
{
    /* Reset generic handler for all events, reset internal states and close PHY module. */
    m_rx_state = SER_PKT_FW_RX_STATE_CLOSED;
    m_tx_state = SER_PKT_FW_TX_STATE_CLOSED;

    for (int i=0 ; i<SER_PKT_FW_MAX_PORTS; i++)
    {
        m_events_handler[i] = NULL;
    }
    
    nrf_drv_gpiote_in_event_disable(m_select_pin);
    nrf_drv_gpiote_in_uninit(m_select_pin);
    m_select_pin = 0xFF;

    ser_phy_close();
}

uint32_t ser_pkt_fw_path_add(ser_pkt_fw_events_handler_t events_handler, uint8_t port)
{
    if (events_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (port > SER_PKT_FW_MAX_PORTS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (m_events_handler[port] != NULL)
    {
        return NRF_ERROR_RESOURCES;
    }

    m_events_handler[port] = events_handler;

    return NRF_SUCCESS;
}

uint32_t ser_pkt_fw_path_remove(uint8_t port)
{
    if (port > SER_PKT_FW_MAX_PORTS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (m_events_handler[port] != NULL)
    {
        return NRF_ERROR_RESOURCES;
    }

    m_events_handler[port] = NULL;

    return NRF_SUCCESS;
}

uint32_t ser_pkt_fw_tx_send(const uint8_t * p_buffer, uint16_t num_of_bytes, uint8_t port)
{
    uint32_t err_code = NRF_SUCCESS;

    if (NULL == p_buffer)
    {
        return NRF_ERROR_NULL;
    }
    if (0 == num_of_bytes)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (port != m_ser_pkt_fw_port)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    err_code = ser_phy_tx_pkt_send((uint8_t *)p_buffer, num_of_bytes);

    return err_code;
}

uint32_t ser_pkt_fw_restore(void)
{
    uint32_t err_code;
    flash_manager_ser_cfg_t *flash;

    err_code = flash_manager_ser_cfg_restore();
    VERIFY_SUCCESS(err_code); 

    // wait for flash to be ready
    while (is_flash_manager_ready() == false)
    {
        nrf_pwr_mgmt_run();
    }
    
    return NRF_SUCCESS;
}
