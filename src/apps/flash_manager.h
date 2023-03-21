 /******************************************************************************
 * @file    flash_manager.h
 * @author  Insight SiP
 * @brief   Flash manager
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

#ifndef FLASH_MANAGER_H_
#define FLASH_MANAGER_H_

#include "ble_conn_params.h"

/**@brief Structure defining the BLE configuration data.
 */
typedef struct
{
    uint8_t role;
    uint8_t dcdc_mode;
    int8_t txp;
    ble_gap_phys_t phys;
    uint8_t name[31];
    uint16_t name_length;
    uint16_t advparam;
    ble_gap_conn_params_t gap_conn_params;
}flash_manager_ble_cfg_t;

/**@brief Structure defining the SER configuration data.
 */
typedef struct
{
    uint8_t flow_control;
    uint32_t baudrate;
}flash_manager_ser_cfg_t;



/**@brief Function for initializing BLE manager flash handling.
 *
 * @retval NRF_SUCCESS      If initialization was successful.
 * @retval Other codes from the underlying drivers.
 */
uint32_t flash_manager_init();

/**@brief Function for storing the BLE configuration.
 *
 * @param[in] p_config  Pointer to configuration to be stored.
 *
 * @retval NRF_SUCCESS      If initialization was successful.
 * @retval NRF_ERROR_NULL   If a NULL pointer was supplied.
 * @retval Other codes from the underlying drivers.
 */
uint32_t flash_manager_ble_cfg_store(flash_manager_ble_cfg_t *p_data);

/**@brief Function for loading the BLE configuration.
 *
 * @param[out] p_config  Pointer to loaded configuration.
 *
 * @retval NRF_SUCCESS      If initialization was successful.
 * @retval Other codes from the underlying drivers.
 */
uint32_t flash_manager_ble_cfg_load(flash_manager_ble_cfg_t **p_data);

/**@brief Function for resstoring default BLE configuration.
 *
 * @retval NRF_SUCCESS      If initialization was successful.
 * @retval NRF_ERROR_NULL   If a NULL pointer was supplied.
 * @retval Other codes from the underlying drivers.
 */
uint32_t flash_manager_ble_cfg_restore(void);

/**@brief Function for storing the SER configuration.
 *
 * @param[in] p_config  Pointer to configuration to be stored.
 *
 * @retval NRF_SUCCESS      If initialization was successful.
 * @retval NRF_ERROR_NULL   If a NULL pointer was supplied.
 * @retval Other codes from the underlying drivers.
 */
uint32_t flash_manager_ser_cfg_store(flash_manager_ser_cfg_t *p_data);

/**@brief Function for loading the SER configuration.
 *
 * @param[out] p_config  Pointer to loaded configuration.
 *
 * @retval NRF_SUCCESS      If initialization was successful.
 * @retval Other codes from the underlying drivers.
 */
uint32_t flash_manager_ser_cfg_load(flash_manager_ser_cfg_t **p_data);

/**@brief Function for resstoring default SER configuration.
 *
 * @retval NRF_SUCCESS      If initialization was successful.
 * @retval NRF_ERROR_NULL   If a NULL pointer was supplied.
 * @retval Other codes from the underlying drivers.
 */
uint32_t flash_manager_ser_cfg_restore(void);

bool is_flash_manager_ready(void);

#endif