/******************************************************************************
 * @file    at_manager.h
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

#ifndef AT_MANAGER_H__
#define AT_MANAGER_H__

#include <stdint.h>

#define SER_PKT_FW_PORT_AT 0

typedef enum {
    AT_OK = 0,               // 0
    AT_ERROR_UNKNOWN_CMD,    // 1
    AT_ERROR_INTERNAL,       // 2
    AT_ERROR_NOT_FOUND,      // 3
    AT_ERROR_NOT_SUPPORTED,  // 4
    AT_ERROR_INVALID_PARAM,  // 5
    AT_ERROR_INVALID_STATE,  // 6
    AT_ERROR_INVALID_LENGTH, // 7
    AT_ERROR_INVALID_FLAGS,  // 8
    AT_ERROR_INVALID_DATA,   // 9
    AT_ERROR_DATA_SIZE,      // 10
    AT_ERROR_TIMEOUT,        // 11
    AT_ERROR_NULL,           // 12
    AT_ERROR_FORBIDDEN,      // 13
    AT_ERROR_BUSY,           // 14
    AT_ERROR_OTHER,          // 15
} at_ret_code_t;

/* AT Command list */
/* Common */
#define AT_RESET "Z"
#define AT_ECHO "E"
#define AT_INFO "I"
#define AT_DEEPSLEEP "+DEEPSLEEP"
#define AT_UART "+UART"
#define AT_DCDC "+DCDC"
#define AT_FACTORYRESET "+FACTORYRESET"
#define AT_BLE_CONNSTATE "+BLECONNSTATE"
#define AT_BLE_TXP "+BLETXP"
#define AT_BLE_PHY "+BLEPHY"
#define AT_BLE_DISCONNECT "+BLEDISCONNECT"
#define AT_BLE_RSSI "+BLERSSI"
#define AT_BLE_CONNPARAM "+BLECONNPARAM"
#define AT_BLE_ADDR "+BLEADDR"
#define AT_BLE_ROLE "+BLEROLE"
/* Peripheral */
#define AT_BLE_ADVPARAM "+BLEADVPARAM"
#define AT_BLE_NAME "+BLENAME"
#define AT_BLE_ADVSTART "+BLEADVSTART"
#define AT_BLE_ADVSTOP "+BLEADVSTOP"
/* Central */
#define AT_BLE_SCANSTART "+BLESCANSTART"
#define AT_BLE_SCANSTOP "+BLESCANSTOP"
#define AT_BLE_SCANLIST "+BLESCANLIST"
#define AT_BLE_CONNECT "+BLECONNECT"


#define ECHO_ON 1
#define ECHO_OFF 0

uint32_t at_cmd_manager_init();
void at_cmd_manager_execute();

#endif