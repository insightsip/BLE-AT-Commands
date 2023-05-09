 /******************************************************************************
 * @file    ser_pkt_fw.h
 * @author  Insight SiP
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

#ifndef SER_PKT_FW_H__
#define SER_PKT_FW_H__


#include <stdint.h>


/**@brief Serial Packet fowarder event types. */
typedef enum
{
    SER_PKT_FW_EVT_TX_PKT_SENT = 0,     /**< An event indicating that TX packet has been transmitted. */
    SER_PKT_FW_EVT_RX_PKT_RECEIVING,    /**< An event indicating that RX packet is being scheduled to receive or to drop. */
    SER_PKT_FW_EVT_RX_PKT_RECEIVED,     /**< An event indicating that RX packet is ready for read. */
    SER_PKT_FW_EVT_RX_PKT_DROPPED,      /**< An event indicating that RX packet was dropped because it was longer than available buffer. */
    SER_PKT_FW_EVT_PHY_ERROR,           /**< An event indicating error on PHY layer. */
    SER_PKT_FW_EVT_TYPE_MAX             /**< Enumeration upper bound. */
} ser_pkt_fw_evt_type_t;

/**@brief PHY layer error types. */
typedef enum
{
    SER_PKT_FW_PHY_ERROR_RX_OVERFLOW = 0, /**< An error indicating that more information has been transmitted than the PHY module could handle. */
    SER_PKT_FW_PHY_ERROR_TX_OVERREAD,     /**< An error indicating that the PHY module was forced to transmit more information than possessed. */
    SER_PKT_FW_PHY_ERROR_HW_ERROR,        /**< An error indicating a hardware error in the PHY module. */
    SER_PKT_FW_PHY_ERROR_TYPE_MAX         /**< Enumeration upper bound. */
} ser_pkt_fw_phy_error_type_t;

/**@brief Serial Packet fowarder containing configuration for a port. */
typedef struct 
{
    uint32_t baudrate;
    uint8_t flow_control;
} ser_pkt_fw_config_t;

/**@brief Struct containing parameters of event of type @ref SER_PKT_FW_EVT_RX_PKT_RECEIVED.
 */
typedef struct
{
    uint8_t * p_buffer;         /**< Pointer to a buffer containing a packet to read. */
    uint16_t  num_of_bytes;     /**< Length of a received packet in octets. */
} ser_pkt_fw_evt_rx_pkt_received_params_t;


/**@brief Struct containing parameters of event of type @ref SER_PKT_FW_EVT_PHY_ERROR. */
typedef struct
{
    ser_pkt_fw_phy_error_type_t error_type;     /**< Type of the PHY error. */
    uint32_t                    hw_error_code;  /**< Hardware error code - specific for a microcontroller. Parameter
                                                        is valid only for the PHY error of type @ref SER_PKT_FW_PHY_ERROR_HW_ERROR. */
} ser_pkt_fw_evt_phy_error_params_t;

/**@brief Struct containing events from the  HAL Transport layer.
 *
 * @note  Some events do not have parameters, then the whole information is contained in the evt_type.
 */
typedef struct
{
    ser_pkt_fw_evt_type_t evt_type;   /**< Type of event. */
    union           /**< Union alternative identified by evt_type in the enclosing struct. */
    {
        ser_pkt_fw_evt_rx_pkt_received_params_t   rx_pkt_received; /**< Parameters of event of type @ref SER_PKT_FW_EVT_RX_PKT_RECEIVED. */
        ser_pkt_fw_evt_phy_error_params_t         phy_error;       /**< Parameters of event of type @ref SER_PKT_FW_EVT_PHY_ERROR. */
    } evt_params;
} ser_pkt_fw_evt_t;


/**@brief Generic callback function type to be used by all Serial packet fowarder events.
 *
 * @param[in] event     Serial packet fowarder event.
 */
typedef void (*ser_pkt_fw_events_handler_t)(ser_pkt_fw_evt_t event);


/**@brief Function for opening and initializing the Serial packet fowarder.
 *
 * @warning If the function has been already called, the function @ref ser_pkt_fw_release has
 *          to be called before ser_pkt_fw_init can be called again.
 *
 * @param[in] select_pin            Number of the pin which select the path.
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Hardware initialization parameters taken from
 *                                  the configuration file are wrong.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. The function has been already called. To call
 *                                  it again the function @ref at_hal_transport_close has to be called first.
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred.
 */
uint32_t ser_pkt_fw_init(uint32_t select_pin);

/**@brief Function for closing a transport channel.
 *
 * @note The function disables the hardware, resets internal module states, and unregisters the events
 *       callback function. Can be called multiple times, also for a channel that is not opened.
 */
void ser_pkt_fw_release(void);

/**@brief Function for adding a new path to the Serial packet fowarder.
 *
 * @param[in] events_handler        Generic callback function to be used by all Serial packet fowarder events
 * @param[in] port                  The port number associated with the event_handler
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. port out of bound.
 * @retval NRF_ERROR_RESOURCES      Operation failure. Path already exists.
 */
uint32_t ser_pkt_fw_path_add(ser_pkt_fw_events_handler_t events_handler, uint8_t port);

/**@brief Function for removing an existing path from the Serial packet fowarder.
 *
 * @param[in] port                  The port number associated with the event_handler
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. port out of bound.
 * @retval NRF_ERROR_RESOURCES      Operation failure. Path already exists.
 */
uint32_t ser_pkt_fw_path_remove(uint8_t port);

/**@brief Function for configuring a path of Serial packet fowarder.
 *
 * @warning The function @ref ser_pkt_fw_init has ser_pkt_fw_configure can be called.
 *
 * @param[in] flow_control          Flow control enable
 * @param[in] baudrate              serial baudrate
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Hardware initialization parameters taken from
 *                                  the configuration file are wrong.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. The function has been already called. To call
 *                                  it again the function @ref at_hal_transport_close has to be called first.
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred.
 */
uint32_t ser_pkt_fw_configure(uint8_t flow_control, uint32_t baudrate);

/**@brief Function for configuring a path of Serial packet fowarder.
 *
 * @warning The function @ref ser_pkt_fw_init has ser_pkt_fw_configure can be called.
 *
 * @param[out] flow_control         Flow control enable
 * @param[out] baudrate             Serial baudrate
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.
 *
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred.
 */
uint32_t ser_pkt_fw_conf_check(uint8_t *flow_control, uint32_t *baudrate);

/**@brief Function for transmitting a packet.
 *
 * @note The function adds a packet pointed by the p_buffer parameter to a transmission queue. A buffer
 *       provided to this function must be allocated by the @ref at_hal_transport_tx_pkt_alloc function.
 *
 * @warning Completion of this method does not guarantee that actual peripheral transmission will be completed.
 *
 * @param[in] p_buffer        Pointer to the buffer to transmit.
 * @param[in] num_of_bytes    Number of octets to transmit. Must be more than 0.
 * @param[in] port            Which port to forward
 *
 * @retval NRF_SUCCESS              Operation success. Packet was added to the transmission queue.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. num_of_bytes is equal to 0.
 * @retval NRF_ERROR_INVALID_ADDR   Operation failure. Not a valid pointer (provided address is not
 *                                  the starting address of a buffer managed by HAL Transport layer).
 * @retval NRF_ERROR_DATA_SIZE      Operation failure. Packet size exceeds limit.
 * @retval NRF_ERROR_BUSY           Operation failure. Transmission queue is full so packet was not
 *                                  added to the transmission queue.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. Transmittion channel was not opened by
 *                                  @ref ser_hal_transport_open function or provided buffer was not
 *                                  allocated by @ref ser_hal_transport_tx_pkt_alloc function.
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred.
 */
uint32_t ser_pkt_fw_tx_send(const uint8_t * p_buffer, uint16_t num_of_bytes, uint8_t port);

/**@brief Function for restauring default parameters.
 *
 * @retval NRF_SUCCESS              Operation success.
 */
uint32_t ser_pkt_fw_restore(void);

#endif