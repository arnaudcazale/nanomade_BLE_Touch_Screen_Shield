/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_hids_mouse_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_mouse
 * @brief HID Mouse Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Service for implementing a simple mouse functionality. This application uses the
 * @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device. This implementation of the
 * application will not know whether a connected central is a known device or not.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "ble_advertising.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager_handler.h"
#include "ble_radio_notification.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"


#define DEVICE_NAME                     "Nanomade"                                /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
#define SWIPE_INTERVAL                  APP_TIMER_TICKS(20)                                                /**< Sampling timer. */
#define SAMPLING_INTERVAL               APP_TIMER_TICKS(20)                         /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                        /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                      /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                      /**< Product Version. */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
#define SLAVE_LATENCY                   20                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define SWIFT_PAIR_SUPPORTED            1                                           /**< Swift Pair feature is supported. */
#if SWIFT_PAIR_SUPPORTED == 1
#define MICROSOFT_VENDOR_ID             0x0006                                      /**< Microsoft Vendor ID.*/
#define MICROSOFT_BEACON_ID             0x03                                        /**< Microsoft Beacon ID, used to indicate that Swift Pair feature is supported. */
#define MICROSOFT_BEACON_SUB_SCENARIO   0x00                                        /**< Microsoft Beacon Sub Scenario, used to indicate how the peripheral will pair using Swift Pair feature. */
#define RESERVED_RSSI_BYTE              0x80                                        /**< Reserved RSSI byte, used to maintain forwards and backwards compatibility. */
#endif

#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              1                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_DIGITIZER_LEN         10
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_DIGITIZER_INDEX       0
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_DIGITIZER_ID      1                                           /**< Id of reference to Mouse Input Report containing media player data. */

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVENT_DATA_SIZE             /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_ADV_FAST_INTERVAL           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           0x00A0                                      /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 100 ms.). */

#define APP_ADV_FAST_DURATION           3000                                        /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           18000                                       /**< The advertising duration of slow advertising in units of 10 milliseconds. */


APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
APP_TIMER_DEF(m_swipe_timer_id);                                                    /**< Battery timer. */
APP_TIMER_DEF(m_sampling_timer_id);                                                  /**< Sampling timer. */
BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */
BLE_HIDS_DEF(m_hids,                                                                /**< HID service instance. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REP_BUTTONS_LEN,
             INPUT_REP_MOVEMENT_LEN             );
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); 

enum SWIPE_TYPE{
  LEFT,
  RIGHT,
  UP,
  DOWN,
  UNKNOW,
};

static uint8_t swipe_type = UNKNOW;
static volatile bool flag_busy = false;
static volatile bool flag_sampling = false;

static uint8_t cpt = 0;     
static uint16_t x = 0;
static uint16_t y = 0;

/* SPI instance ID. */
#define SPI_INSTANCE      1     
#define SPI_SCK_PIN       28
//#define SPI_MISO_PIN      30
#define SPI_MOSI_PIN      29
#define SPI_SS_PIN_POT       31
#define SPI_SS_PIN_AMP       30

bool current_radio_active_state = false;
bool flag_connected = false;

/* Indicates if operation on SPI has ended. */
static volatile bool spi_xfer_done;  
static uint8_t       m_tx_buf[] = {((uint8_t)0x00),((uint8_t)0x00)}; 
static uint8_t       m_rx_buf[sizeof(m_tx_buf)];  
/* SPI instance. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static int wiper_data[6];
static uint8_t channel;
static bool calib[6] = {false,false,false,false,false,false};
//Origine coordonées -> UP/LEFT (size screen env 30 000)
static uint16_t m_x[] = {5000, 5000, 5000, 5000, 25000, 25000, 25000, 25000};
static uint16_t m_y[] = {25000, 20000, 15000, 10000, 10000, 15000, 20000, 25000};
                                           
/* TWI instance ID. */
#define TWI_INSTANCE_ID     0 
#define TWI_SCL             22 //3 //27,
#define TWI_SDA             23 //4 //26,
#define CAP1208_ADDR        0x28 
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;
//static volatile bool m_cal_running = false;
/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* Buffer for CAP1208 driver. */
uint8_t reg[2] = {0xFD, 0x00};
static uint8_t cmd_read;
static double data_read[8];
static uint8_t sampling_line;

/* SAADC */
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)
static nrf_saadc_value_t adc_buf[2];
static nrf_saadc_value_t ADC_RESULT;
static volatile bool m_sampling_done = false;
static volatile uint16_t adc_result_in_milli_volts;

/* GPIO */
#define A 2
#define B 25
#define C 24

static bool              m_in_boot_mode = false;                                    /**< Current protocol mode. */
static uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                                 /**< Device reference handle to the current bonded central. */
static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */
static ble_uuid_t        m_adv_uuids[] =                                            /**< Universally unique service identifiers. */
{
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
};

#if SWIFT_PAIR_SUPPORTED == 1
static uint8_t m_sp_payload[] =                                                     /**< Payload of advertising data structure for Microsoft Swift Pair feature. */
{
    MICROSOFT_BEACON_ID,
    MICROSOFT_BEACON_SUB_SCENARIO,
    RESERVED_RSSI_BYTE
};
static ble_advdata_manuf_data_t m_sp_manuf_advdata =                                /**< Advertising data structure for Microsoft Swift Pair feature. */
{
    .company_identifier = MICROSOFT_VENDOR_ID,
    .data               =
    {
        .size   = sizeof(m_sp_payload),
        .p_data = &m_sp_payload[0]
    }
};
static uint8_t            m_sp_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];          /**< Advertising data buffer. */
static ble_gap_adv_data_t m_sp_advdata_buf =                                        /**< Advertising data buffer descriptor. */
{
    .adv_data =
    {
        .p_data = m_sp_enc_advdata,
        .len    = sizeof(m_sp_enc_advdata)
    }
};
#endif

typedef PACKED_STRUCT
{
    uint8_t  tip_switch : 1;
    uint8_t  tip_pressure : 7;
    uint8_t  contact_id;
    uint16_t x;
    uint16_t y;
    uint16_t scan_time;
    uint8_t  contact_count;
    uint8_t  contact_count_max;
}digitizer_report_t;

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for setting filtered whitelist.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void whitelist_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                   peer_id_count + 1,
                   BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

    err_code = pm_whitelist_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting filtered device identities.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void identities_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);

        ret_code_t ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                // Note: You should check on what kind of white list policy your application should use.

                whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void timer_swipe_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_swipe_timer_id, SWIPE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void timer_sampling_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_sampling_timer_id, SAMPLING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void timer_swipe_stop(void)
{
    ret_code_t err_code;
    err_code = app_timer_stop(m_swipe_timer_id);
    APP_ERROR_CHECK(err_code);
}

static void timer_sampling_stop(void)
{
    ret_code_t err_code;

    err_code = app_timer_stop(m_sampling_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    NRF_LOG_INFO("battery_level: %d", battery_level);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

void read_sensorCAP_data(uint8_t line)
{
    ret_code_t err_code;
    m_xfer_done = false;

    ////Read SENSOR INPUT 1 DELTA COUNT (Register 0x10)
    reg[0] = line + 0x10;
    err_code = nrf_drv_twi_tx(&m_twi, CAP1208_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_done);
    m_xfer_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, CAP1208_ADDR, &cmd_read, sizeof(cmd_read));
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_done);
    m_xfer_done = false;

    //nrf_delay_us(500);
}

static void sampling_timeout_handler(void * p_context)
{
    flag_sampling = true;

    // Log execution mode.
//    if (current_int_priority_get() == APP_IRQ_PRIORITY_THREAD)
//    {
//        NRF_LOG_INFO("Timeout handler is executing in thread/main mode.");
//    }
//    else
//    {
//        NRF_LOG_INFO("Timeout handler is executing in interrupt handler mode.");
//    }
}

void digitizer_send(uint8_t id, uint8_t c_count, uint16_t x, uint16_t y, bool tip_down)
{
    //flag_busy = false;

    digitizer_report_t report = {0};
        
    report.tip_switch = (tip_down > 0) ? 1 : 0;
    report.tip_pressure = (tip_down > 0) ? 127 : 0;
    report.contact_id = id;
    report.x = x;
    report.y = y;
    report.scan_time = 0x64;
    report.contact_count = c_count;
    report.contact_count_max = 6;

    uint32_t err_code;

    do
    {
      err_code = ble_hids_inp_rep_send(&m_hids,
                                           INPUT_REP_DIGITIZER_INDEX,
                                           INPUT_REP_DIGITIZER_LEN,
                                           (uint8_t *)&report,
                                           m_conn_handle); 

      //APP_ERROR_CHECK(err_code);
    }while(err_code == NRF_ERROR_RESOURCES);
}

static void swipe_timeout_handler(void * p_context)
{

  switch (swipe_type)
    {
        case LEFT:
            y = 15000;
            x = (30000) - cpt*2500;
            cpt++;
            digitizer_send(0, 1, x, y, true);
            NRF_LOG_INFO("Send... cpt = %d, x = %d, y = %d, tip = %d", cpt, x, y, true);
            if(cpt == 5) {
                digitizer_send(0, 1, x, y, false);
                timer_swipe_stop();
                cpt = 0;
            }
            break;

        case RIGHT:
            y = 15000;
            x = cpt*2500;
            cpt++;
            digitizer_send(0, 1, x, y, true);
            NRF_LOG_INFO("Send... cpt = %d, x = %d, y = %d, tip = %d", cpt, x, y, true);
            if(cpt == 5) {
                digitizer_send(0, 1, x, y, false);
                timer_swipe_stop();
                cpt = 0;
            }
            break;

        case UP:
            y = (30000) - cpt*2500;
            x = 0;
            cpt++;
            digitizer_send(0, 1, x, y, true);
            NRF_LOG_INFO("Send... cpt = %d, x = %d, y = %d, tip = %d", cpt, x, y, true);
            if(cpt == 5) {
                digitizer_send(0, 1, x, y, false);
                NRF_LOG_INFO("Send... cpt = %d, x = %d, y = %d", cpt, x, y, false);
                timer_swipe_stop();
                cpt = 0;
            }
            break;

        case DOWN:
            y = cpt*2500;
            x = 0;
            cpt++;
            digitizer_send(0, 1, x, y, true);
            NRF_LOG_INFO("Send... cpt = %d, x = %d, y = %d, tip = %d", cpt, x, y, true);
            if(cpt == 10) {
                digitizer_send(0, 1, x, y, false);
                NRF_LOG_INFO("Send... cpt = %d, x = %d, y = %d", cpt, x, y, false);
                timer_swipe_stop();
                cpt = 0;
            }
            break;

        default:
            // No implementation needed.
            break;
    }
   
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);


    APP_ERROR_CHECK(err_code);

    // Create sapling timer.
    err_code = app_timer_create(&m_sampling_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sampling_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Createswipe timer.
    err_code = app_timer_create(&m_swipe_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                swipe_timeout_handler);


    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
    
    ble_gap_addr_t ble_address = {.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
                                                      .addr_id_peer = 0,
                                                      .addr = {0xC3,0x11,0x34,0x34,0x44,0xFF}};
    err_code = sd_ble_gap_addr_set(&ble_address);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
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


/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    dis_init_obj.dis_char_rd_sec = SEC_JUST_WORKS;

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    bas_init_obj.bl_rd_sec        = SEC_JUST_WORKS;
    bas_init_obj.bl_cccd_wr_sec   = SEC_JUST_WORKS;
    bas_init_obj.bl_report_rd_sec = SEC_JUST_WORKS;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                err_code;
    ble_hids_init_t           hids_init_obj;
    ble_hids_inp_rep_init_t * p_input_report;
    uint8_t                   hid_info_flags;

    static ble_hids_inp_rep_init_t inp_rep_array[INPUT_REPORT_COUNT];
    static uint8_t rep_map_data[] =
    {
        0x05, 0x0D,        // Usage Page (Digitizer)
        0x09, 0x04,        // Usage (Touch Screen)
        0xA1, 0x01,        // Collection (Application)
        0x85, 0x01,        //   Report Id 1
        0x09, 0x22,        //   Usage (Finger)
        0xA1, 0x02,        //   Collection (Logical)
        0x09, 0x42,        //     Usage (Tip Switch)
        0x15, 0x00,        //     Logical Minimum (0)
        0x25, 0x01,        //     Logical Maximum (1)
        0x75, 0x01,        //     Report Size (1)
        0x95, 0x01,        //     Report Count (1)
        0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x09, 0x30,        //     Usage (Tip Pressure)
        0x25, 0x7F,        //     Logical Maximum (127)
        0x75, 0x07,        //     Report Size (7)
        0x95, 0x01,        //     Report Count (1)
        0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x09, 0x51,        //     Usage (0x51 - Contact identifier)
        0x26, 0xFF, 0x00,  //     Logical Maximum (255)
        0x75, 0x08,        //     Report Size (8)
        0x95, 0x01,        //     Report Count (1)
        0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
        0x09, 0x30,        //     Usage (X)
        0x09, 0x31,        //     Usage (Y)
        0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
        0x65, 0x00,        //     Unit (None)
        0x75, 0x10,        //     Report Size (16)
        0x95, 0x02,        //     Report Count (2)
        0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0xC0,              //   End Collection
        0x05, 0x0D,        //   Usage Page (Digitizer)
        0x27, 0xFF, 0xFF, 0x00, 0x00,  //   Logical Maximum (65534)
        0x09, 0x56,        //   Usage (0x56 - Scan Time)
        0x75, 0x10,        //   Report Size (16)
        0x95, 0x01,        //   Report Count (1)
        0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x09, 0x54,        //   Usage (0x54 - Contact count)
        0x25, 0x05,        //   Logical Maximum (5)
        0x75, 0x08,        //   Report Size (8)
        0x95, 0x01,        //   Report Count (1)
        0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x05, 0x0D,        //   Usage Page (Digitizer)
        0x09, 0x55,        //   Usage (0x55 - Contact point maximum)
        0x25, 0x0A,        //   Logical Maximum (10)
        0x75, 0x08,        //   Report Size (8)
        0x95, 0x01,        //   Report Count (1)
        0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0xC0               // End Collection  
    };

    memset(inp_rep_array, 0, sizeof(inp_rep_array));
    // Initialize HID Service.
    p_input_report                      = &inp_rep_array[INPUT_REP_DIGITIZER_INDEX];
    p_input_report->max_len             = INPUT_REP_DIGITIZER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_DIGITIZER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;
    hids_init_obj.is_mouse                       = true;
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array                = inp_rep_array;
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
    hids_init_obj.rep_map.p_data                 = rep_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    hids_init_obj.rep_map.rd_sec         = SEC_JUST_WORKS;
    hids_init_obj.hid_information.rd_sec = SEC_JUST_WORKS;

    hids_init_obj.boot_mouse_inp_rep_sec.cccd_wr = SEC_JUST_WORKS;
    hids_init_obj.boot_mouse_inp_rep_sec.wr      = SEC_JUST_WORKS;
    hids_init_obj.boot_mouse_inp_rep_sec.rd      = SEC_JUST_WORKS;

    hids_init_obj.protocol_mode_rd_sec = SEC_JUST_WORKS;
    hids_init_obj.protocol_mode_wr_sec = SEC_JUST_WORKS;
    hids_init_obj.ctrl_point_wr_sec    = SEC_JUST_WORKS;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    qwr_init();
    dis_init();
    bas_init();
    hids_init();
}


/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void timer_battery_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}

void on_tx_complete()
{
    if (flag_busy)
    {
        flag_busy = false;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            NRF_LOG_INFO("Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
#if SWIFT_PAIR_SUPPORTED == 1
            err_code = ble_advertising_advdata_update(&m_advertising, &m_sp_advdata_buf, false);
            APP_ERROR_CHECK(err_code);
#endif
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                           addr_cnt,
                           irk_cnt);

            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {

                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    // Manipulate identities to exclude peers with no Central Address Resolution.
                    identities_set(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            flag_connected = true;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.

            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            flag_connected = false;
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

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_tx_complete();
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


#if SWIFT_PAIR_SUPPORTED == 1
/**@brief Function for encoding Swift Pair advertising data set, which should be used in Swift Pair
 *        mode.
 *
 * @param[in]   p_new_advdata      Pointer to the structure which specifies content of encoded data.
 * @param[out]  p_new_advdata_buf  Pointer to the buffer where encoded data will be stored.
 */
static void sp_advdata_prepare(ble_advdata_t const * const p_new_advdata,
                               ble_gap_adv_data_t  * const p_new_advdata_buf)
{
    ret_code_t ret = ble_advdata_encode(p_new_advdata,
                                        p_new_advdata_buf->adv_data.p_data,
                                        &p_new_advdata_buf->adv_data.len);
    APP_ERROR_CHECK(ret);

    if (p_new_advdata_buf->scan_rsp_data.p_data != NULL)
    {
        ret = ble_advdata_encode(p_new_advdata,
                                 p_new_advdata_buf->scan_rsp_data.p_data,
                                 &p_new_advdata_buf->scan_rsp_data.len);
        APP_ERROR_CHECK(ret);
    }
}
#endif


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    uint8_t                adv_flags;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = true;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled               = true;
    init.config.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

#if SWIFT_PAIR_SUPPORTED == 1
    init.advdata.p_manuf_specific_data = &m_sp_manuf_advdata;
    sp_advdata_prepare(&init.advdata, &m_sp_advdata_buf);
#endif
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                swipe_type = UP;
                //timer_swipe_start();
                timer_sampling_start();
            }
            break;

        case BSP_EVENT_KEY_1:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                swipe_type = DOWN;
                timer_swipe_start();
            }
            break;

        case BSP_EVENT_KEY_2:

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                swipe_type = LEFT;
                timer_swipe_start();
            }
           
            break;

        case BSP_EVENT_KEY_3:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                swipe_type = RIGHT;
                timer_swipe_start();
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**CUSTOM FUNCTION**/

void ble_on_radio_active_evt(bool radio_active)
{
    current_radio_active_state = radio_active;
    if(radio_active)
    {
      bsp_board_led_on(1);
    }else bsp_board_led_off(1);
}

void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        adc_result_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;

        //NRF_LOG_INFO("ADC results in millivolts: %d", adc_result_in_milli_volts);

        m_sampling_done = true;
    }
}

__STATIC_INLINE void data_handler(uint8_t data)
{
    if(data & 0x80){  //Si nombre négatif, reset à 0
        data = 0;
    }

    //Convert results
    double data_norm = ((double)data/127)*100 ;
    if(data_norm > 100){  //Si > 100, reset à 0
        data_norm = 100;
    }
    data_read[sampling_line] = data_norm;
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received:");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
}

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
//            NRF_LOG_INFO("TWI NRF_DRV_TWI_EVT_DONE");
//            NRF_LOG_FLUSH();
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(cmd_read);
            }
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
//            NRF_LOG_INFO("TWI NRF_DRV_TWI_EVT_ADDRESS_NACK");
//            NRF_LOG_FLUSH();
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
//            NRF_LOG_INFO("TWI NRF_DRV_TWI_EVT_DATA_NACK");
//            NRF_LOG_FLUSH();
            break;
        default:
            break;
    }
}

static void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1); //NRF_SAADC_INPUT_VDD
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}

static void saadc_sample()
{
    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

static void spi_init()
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED; //SPI_SS_PIN;
    spi_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    //spi_config.bit_order = NRF_SPI_BIT_ORDER_LSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));    
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = TWI_SCL, //3, //27,
       .sda                = TWI_SDA, //4, //26,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void radio_notification_init(void)
{
    uint32_t err_code;

    err_code = ble_radio_notification_init(APP_IRQ_PRIORITY_LOW,
                                           NRF_RADIO_NOTIFICATION_DISTANCE_800US,
                                           ble_on_radio_active_evt);
    APP_ERROR_CHECK(err_code);
}


static void CAP1208_init(void)
{
    NRF_LOG_INFO("Start CAP1208_init.");

    ret_code_t err_code;

    /* Read CHIP_ID (Register 0xFD) -> Chip test */
    err_code = nrf_drv_twi_tx(&m_twi, CAP1208_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_done);
    m_xfer_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, CAP1208_ADDR, &cmd_read, sizeof(cmd_read));
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_done);
    m_xfer_done = false;

    NRF_LOG_INFO("CHIP ID = %X", cmd_read);
    NRF_LOG_INFO("Stop CAP1208_init.");
}

static void MAX_9939_init()
{

    uint8_t command[1]; 
    command[0] = 0xc0; //Gain a 20
    
    nrf_gpio_pin_clear(SPI_SS_PIN_AMP);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, command, sizeof(command), NULL, NULL));
    //NRF_LOG_HEXDUMP_INFO(command, sizeof(command));

    while (!spi_xfer_done)
    {
        __WFE();
    }

    spi_xfer_done = false;
    nrf_gpio_pin_set(SPI_SS_PIN_AMP);

    //NRF_LOG_FLUSH();
}

static void gpio_init()
{
    nrf_gpio_cfg_output(A);
    nrf_gpio_cfg_output(B);
    nrf_gpio_cfg_output(C);
    nrf_gpio_cfg_output(SPI_SS_PIN_POT);
    nrf_gpio_cfg_output(SPI_SS_PIN_AMP);

    nrf_gpio_pin_clear(A);
    nrf_gpio_pin_clear(B);
    nrf_gpio_pin_clear(C);
    nrf_gpio_pin_set(SPI_SS_PIN_POT);
    nrf_gpio_pin_set(SPI_SS_PIN_AMP);
}

static void mux_switch(uint8_t channel)
{
    switch (channel)
    {
        case 1:
              nrf_gpio_pin_set(A);
              nrf_gpio_pin_clear(B);
              nrf_gpio_pin_clear(C);
            break;
        case 2:
              nrf_gpio_pin_clear(A);
              nrf_gpio_pin_set(B);
              nrf_gpio_pin_clear(C);
            break;
        case 3:
              nrf_gpio_pin_set(A);
              nrf_gpio_pin_set(B);
              nrf_gpio_pin_clear(C);
            break;
        case 4:
              nrf_gpio_pin_clear(A);
              nrf_gpio_pin_clear(B);
              nrf_gpio_pin_set(C);
            break;
        case 5:
              nrf_gpio_pin_set(A);
              nrf_gpio_pin_clear(B);
              nrf_gpio_pin_set(C);
            break;
        case 6:
              nrf_gpio_pin_clear(A);
              nrf_gpio_pin_set(B);
              nrf_gpio_pin_set(C);
            break;
        default:
              nrf_gpio_pin_clear(A);
              nrf_gpio_pin_clear(B);
              nrf_gpio_pin_clear(C);
            break;
    }
}

static void set_potentiometer(uint8_t channel, uint8_t wiper_data)
{
    channel = channel-1;
    m_tx_buf[0] = channel; 
    m_tx_buf[1] = wiper_data;

    nrf_gpio_pin_clear(SPI_SS_PIN_POT);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sizeof(m_tx_buf), NULL, NULL));
    //NRF_LOG_HEXDUMP_INFO(m_tx_buf, sizeof(m_tx_buf));

    while (!spi_xfer_done)
    {
        __WFE();
    }

    spi_xfer_done = false;
    nrf_gpio_pin_set(SPI_SS_PIN_POT);

    //NRF_LOG_FLUSH();
}

static void calibration()
{
  NRF_LOG_INFO("Start Calibration");

  for(int i = 0; i<6; i++)
  {
    channel = i;
    wiper_data[channel] = 255;
    mux_switch(channel+1);

    // Set potentiometer, block until set
    set_potentiometer(channel+1, wiper_data[channel]);

    // Sampling, block until get data
    saadc_sample();
    while(!m_sampling_done);
    m_sampling_done = false;

    // Decrement wiper data to be in the bridge middle point
    if(adc_result_in_milli_volts > 1655)
    {
      while ( (adc_result_in_milli_volts > 1655) && (wiper_data[channel] > 0) )
      {
        wiper_data[channel]--;

        // Set potentiometer, block until set
        set_potentiometer(channel+1, wiper_data[channel]);

        // Sampling, block until get data
        saadc_sample();
        while(!m_sampling_done);
        m_sampling_done = false;
      }
    }

    //Check if wiper data is correct
    if( (wiper_data[channel] > 0) && (wiper_data[channel] < 255) )
    {
      calib[channel] = true;
    }

    NRF_LOG_INFO("Flags_calib[%d] = %d ", channel,  calib[channel]); 
    NRF_LOG_INFO("Wiper_data[%d] = %d", channel, wiper_data[channel]);
  }

  NRF_LOG_INFO("End Calibration.");    
}

void activity()
{   
  for(uint8_t i=0; i<8; i++)
  {
     sampling_line = i;
     // Read data of capacitive driver
     read_sensorCAP_data(i);
     if(data_read[sampling_line] > 50)
     {
        digitizer_send(0, 1, m_x[sampling_line], m_y[sampling_line], true);
     }
  }
}

/**@brief Function for application main entry.
 */
int main(void)
     {
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    //radio_notification_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();

    // Peripheral initializtion
    gpio_init();
    adc_configure();
    spi_init();
    twi_init();
    MAX_9939_init();
    calibration();
    CAP1208_init();

    // Start execution.
    NRF_LOG_INFO("HID Mouse example started.");
    NRF_LOG_FLUSH();

    //timer_battery_start();
    timer_sampling_start();
    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();

        if( flag_sampling && flag_connected)
        {
            // Run code previously in timeout handler   
           activity();
           flag_sampling = false;
        }
    }
}







