 /******************************************************************************
 * @file    ser_phy.c
 * @author  Insight SiP
 * @brief  uart phy level
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
#include "app_uart.h"
#include "ble_gatt.h"
#include "nrf_delay.h"

#include "ser_phy.h"
#include "boards.h"


#define UART_TX_BUF_SIZE 256                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                                        /**< UART RX buffer size. */
#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

static uint8_t m_rx_buffer[UART_RX_BUF_SIZE];
static ser_phy_events_handler_t m_ser_phy_event_handler;
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
void uart_event_handler (app_uart_evt_t * p_event)
{
    ser_phy_evt_t evt;
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            // fectch data
            UNUSED_VARIABLE(app_uart_get(&m_rx_buffer[index]));
            index++;

            if ((m_rx_buffer[index - 1] == '\r') || (m_rx_buffer[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                if (index > 1)
                {
                   // Generate SER_PHY_EVT_RX_PKT_RECEIVED event
                    evt.evt_type = SER_PHY_EVT_RX_PKT_RECEIVED;
                    evt.evt_params.rx_pkt_received.num_of_bytes = index;
                    evt.evt_params.rx_pkt_received.p_buffer = m_rx_buffer;
                    m_ser_phy_event_handler(evt);
                }

                index = 0;
            }
            break;

        case APP_UART_TX_EMPTY:
            // Generate SER_PHY_EVT_TX_PKT_SENT event
            evt.evt_type = SER_PHY_EVT_TX_PKT_SENT;
            m_ser_phy_event_handler(evt);
            break;

        case APP_UART_COMMUNICATION_ERROR:
            // Generate SER_PHY_EVT_HW_ERROR event
            evt.evt_type = SER_PHY_EVT_HW_ERROR;
            m_ser_phy_event_handler(evt);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

uint32_t ser_phy_open(uint8_t flow_control, uint32_t baudrate, ser_phy_events_handler_t events_handler)
{
    uint32_t err_code;
    app_uart_comm_params_t comm_params;

    if (events_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Check if function was not called before.
    if (m_ser_phy_event_handler != NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    // Initialize app_uart
    comm_params.rx_pin_no = PIN_UART_RX;
    comm_params.tx_pin_no = PIN_UART_TX;
    comm_params.rts_pin_no = PIN_UART_RTS;
    comm_params.cts_pin_no = PIN_UART_CTS;
    comm_params.flow_control = flow_control? APP_UART_FLOW_CONTROL_ENABLED : APP_UART_FLOW_CONTROL_DISABLED;
    comm_params.use_parity   = false;
    switch (baudrate)
    {
        case 1200:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud1200;
            break;
        case 2400:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud2400;
            break;
        case 4800:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud4800;
            break;
        case 9600:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud9600;
            break;
        case 14400:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud14400;
            break;
        case 19200:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud19200;
            break;
        case 28800:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud28800;
            break;
        case 38400:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud38400;
            break;
        case 57600:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud57600;
            break;
        case 76800:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud76800;
            break;
        case 115200:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200;
            break;
        case 230400:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud230400;
            break;
        case 250000:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud250000;
            break;
        case 460800:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud460800;
            break;
        case 921600:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud921600;
            break;
        case 1000000:
            comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud1M;
            break;
    }

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handler,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);

    // Register event
    m_ser_phy_event_handler = events_handler;

    return err_code;
}

void ser_phy_close(void)
{
    // Unitialize app_uart
    app_uart_close();

    // Unregister event
    m_ser_phy_event_handler = NULL;
}

uint32_t ser_phy_tx_pkt_send (uint8_t *p_buffer, uint8_t num_of_bytes)
{
    for (uint32_t i=0; i<num_of_bytes; i++) 
    {
        while(app_uart_put(p_buffer[i]) != NRF_SUCCESS);
       // nrf_delay_us(500);
    }
}

uint32_t ser_phy_buffer_length_set(uint8_t length)
{
    m_ble_nus_max_data_len = length;

    return NRF_SUCCESS;
}
