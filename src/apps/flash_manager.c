 /******************************************************************************
 * @file    flash_manager.c
 * @author  Insight SiP
 * @version V0.2.0
 * @date    13-03-2020
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

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "fds.h"
#include "flash_manager.h"


/* File ID and Key used for the configuration records. */
#define FLASH_MANAGER_FILE_ID       (0x8010)
#define FLASH_MANAGER_BLE_REC_KEY   (0x7010)
#define FLASH_MANAGER_SER_REC_KEY   (0x7011)


#define DEFAULT_DCDC_MODE           0
#define DEFAULT_TXP                 0
#define DEFAULT_PHYS_TX             0
#define DEFAULT_PHYS_RX             0
#define MIN_CONN_INTERVAL           20     
#define MAX_CONN_INTERVAL           75            
#define SLAVE_LATENCY               0           
#define CONN_SUP_TIMEOUT            4000 
#define DEFAULT_NAME                "ISP_BLE_UART"
#define DEFAULT_ADVINT              300


/**@brief Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

static bool m_fds_initialized;                                      /**< Flag to check fds initialization. */
static bool m_fds_updated;                                          /**< Flag to check fds updating. */
static fds_record_desc_t m_ble_cfg_records_desc;                    /**< BLE configuration Record descriptor */
static fds_record_desc_t m_ser_cfg_records_desc;                    /**< BLE configuration Record descriptor */

/**@brief BLE configuration data. */
static flash_manager_ble_cfg_t m_ble_cfg =
{
    .dcdc_mode = DEFAULT_DCDC_MODE,
    .txp = DEFAULT_TXP,
    .phys.rx_phys = DEFAULT_PHYS_RX,
    .phys.tx_phys = DEFAULT_PHYS_TX,
    .name = DEFAULT_NAME,
    .advparam = DEFAULT_ADVINT,
    .gap_conn_params.min_conn_interval = MSEC_TO_UNITS(MIN_CONN_INTERVAL, UNIT_1_25_MS),
    .gap_conn_params.max_conn_interval = MSEC_TO_UNITS(MAX_CONN_INTERVAL, UNIT_1_25_MS),
    .gap_conn_params.slave_latency = SLAVE_LATENCY,
    .gap_conn_params.conn_sup_timeout = MSEC_TO_UNITS(CONN_SUP_TIMEOUT, UNIT_10_MS),
};

/**@brief SER configuration data. */
static flash_manager_ser_cfg_t m_ser_cfg =
{
    .flow_control = 0,
    .baudrate = 38400
};

/**@brief Record containing BLE configuration data. */
static fds_record_t const m_ble_cfg_record =
{
    .file_id           = FLASH_MANAGER_FILE_ID,
    .key               = FLASH_MANAGER_BLE_REC_KEY,
    .data.p_data       = &m_ble_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_ble_cfg) + 3) / sizeof(uint32_t),
};

/**@brief Record containing SER configuration data. */
static fds_record_t const m_ser_cfg_record =
{
    .file_id           = FLASH_MANAGER_FILE_ID,
    .key               = FLASH_MANAGER_SER_REC_KEY,
    .data.p_data       = &m_ser_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_ser_cfg) + 3) / sizeof(uint32_t),
};

/**@brief Flash data storage handler. */
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Event: %s received (NRF_SUCCESS)", fds_evt_str[p_evt->id]);
    }
    else
    {
        NRF_LOG_INFO("Event: %s received (%s)", fds_evt_str[p_evt->id], fds_err_str(p_evt->result));
    }

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_UPDATE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
            m_fds_updated = true;
        } break;


        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}

uint32_t flash_manager_init(void)
{
    uint32_t err_code;
    fds_find_token_t  tok  = {0};

    err_code = fds_register(fds_evt_handler);
    VERIFY_SUCCESS(err_code);

     /* Initialize fds. */
    err_code = fds_init();
    VERIFY_SUCCESS(err_code);

    /* Wait for fds to initialize. */
    while (m_fds_initialized == false)
    {
        nrf_pwr_mgmt_run();
    }
    m_fds_updated = true;


    /* Find record containing BLE configuration */
    err_code = fds_record_find(FLASH_MANAGER_FILE_ID, FLASH_MANAGER_BLE_REC_KEY, &m_ble_cfg_records_desc, &tok);
    if (err_code == FDS_ERR_NOT_FOUND)
    {
        // Record not foud, create it and fill with default values
        err_code = fds_record_write(&m_ble_cfg_records_desc, &m_ble_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_ERROR("No space in flash, delete some records to update the config file.");
        }
            else
            {
                VERIFY_SUCCESS(err_code);
            }
    }

    /* Find record containing SER configuration */
    err_code = fds_record_find(FLASH_MANAGER_FILE_ID, FLASH_MANAGER_SER_REC_KEY, &m_ser_cfg_records_desc, &tok);
    if (err_code == FDS_ERR_NOT_FOUND)
    {
        // Record not foud, create it and fill with default values
        err_code = fds_record_write(&m_ser_cfg_records_desc, &m_ser_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_ERROR("No space in flash, delete some records to update the config file.");
        }
            else
            {
                VERIFY_SUCCESS(err_code);
            }
    }

    return NRF_SUCCESS;
}

uint32_t flash_manager_ble_cfg_load(flash_manager_ble_cfg_t **p_data)
{
    uint32_t err_code;
    fds_flash_record_t  flash_record;
    fds_find_token_t    ftok;

    err_code = fds_record_open(&m_ble_cfg_records_desc, &flash_record);
    VERIFY_SUCCESS(err_code);

    memcpy(&m_ble_cfg, flash_record.p_data, sizeof(flash_manager_ble_cfg_t));

    err_code = fds_record_close(&m_ble_cfg_records_desc);
    VERIFY_SUCCESS(err_code);

    *p_data = &m_ble_cfg;

    return NRF_SUCCESS;
}

uint32_t flash_manager_ble_cfg_store(flash_manager_ble_cfg_t * p_config)
{
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_config);

    m_fds_updated = false;
    memcpy(&m_ble_cfg, p_config, sizeof(flash_manager_ble_cfg_t));
    
    err_code = fds_record_update(&m_ble_cfg_records_desc, &m_ble_cfg_record);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t flash_manager_ble_cfg_restore(void)
{
    uint32_t err_code;

    m_fds_updated = false;

    m_ble_cfg.dcdc_mode = DEFAULT_DCDC_MODE;
    m_ble_cfg.txp = DEFAULT_TXP;
    m_ble_cfg.phys.rx_phys = DEFAULT_PHYS_RX,
    m_ble_cfg.phys.tx_phys = DEFAULT_PHYS_TX,
    strncpy(m_ble_cfg.name, DEFAULT_NAME, sizeof(m_ble_cfg.name));
    m_ble_cfg.advparam = DEFAULT_ADVINT;
    m_ble_cfg.gap_conn_params.min_conn_interval = MSEC_TO_UNITS(MIN_CONN_INTERVAL, UNIT_1_25_MS);
    m_ble_cfg.gap_conn_params.max_conn_interval = MSEC_TO_UNITS(MAX_CONN_INTERVAL, UNIT_1_25_MS);
    m_ble_cfg.gap_conn_params.slave_latency = SLAVE_LATENCY,
    m_ble_cfg.gap_conn_params.conn_sup_timeout = MSEC_TO_UNITS(CONN_SUP_TIMEOUT, UNIT_10_MS);

    err_code = fds_record_update(&m_ble_cfg_records_desc, &m_ble_cfg_record);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t flash_manager_ser_cfg_load(flash_manager_ser_cfg_t **p_data)
{
    uint32_t err_code;
    fds_flash_record_t  flash_record;
    fds_find_token_t    ftok;

    err_code = fds_record_open(&m_ser_cfg_records_desc, &flash_record);
    VERIFY_SUCCESS(err_code);

    memcpy(&m_ser_cfg, flash_record.p_data, sizeof(flash_manager_ser_cfg_t));

    err_code = fds_record_close(&m_ser_cfg_records_desc);
    VERIFY_SUCCESS(err_code);

    *p_data = &m_ser_cfg;

    return NRF_SUCCESS;
}

uint32_t flash_manager_ser_cfg_store(flash_manager_ser_cfg_t * p_config)
{
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_config);

    m_fds_updated = false;

    memcpy(&m_ser_cfg, p_config, sizeof(flash_manager_ser_cfg_t));

    err_code = fds_record_update(&m_ser_cfg_records_desc, &m_ser_cfg_record);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t flash_manager_ser_cfg_restore(void)
{
    uint32_t err_code;

    m_fds_updated = false;

    m_ser_cfg.baudrate = 38400;
    m_ser_cfg.flow_control = 0;

    err_code = fds_record_update(&m_ser_cfg_records_desc, &m_ser_cfg_record);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

bool is_flash_manager_ready(void)
{
    if (m_fds_initialized && m_fds_updated)
    {
        return true;
    }
    return false;
}