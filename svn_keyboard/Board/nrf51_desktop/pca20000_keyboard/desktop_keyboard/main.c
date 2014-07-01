/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $$
 */

#include <stdint.h>
#include <string.h>
#include "app_scheduler.h"
#include "app_timer.h"
#include "ble_bondmngr.h"
#include "common_params.h"
#include "io_cfg.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nrf_gzp_config.h"
#include "nrf_nvmc.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#include "m_batt_meas.h"
#include "m_coms.h"
#include "m_coms_ble.h"
#include "m_nfc.h"
#include "m_keyboard.h"
#include "m_pwr_and_clk_mgmt.h"
#include "m_led.h"
#include "project_params.h"

#include "hal_gpiote.h"
#include "hal_rng.h"
// my added 
#include "dbg_biao.h"
#include "app_kbrpt.h"
#include "m_coms_ble_hid_cfg.h"
#define SCHED_QUEUE_SIZE           16
#define SCHED_MAX_EVENT_DATA_SIZE  MAX(sizeof(m_coms_ble_evt_t), MAX(sizeof(m_keyboard_data_t), sizeof(app_timer_event_t)))


#define APP_TIMER_MAX_TIMERS    10 // One for each module + one for ble_conn_params + a few extra
#define APP_TIMER_OP_QUEUE_SIZE 10 // Maximum number of timeout handlers pending execution

#define PACKET_BUFFER_SIZE SCHED_QUEUE_SIZE + 1 // Include empty element

static enum
{
    state_disconnected,
    state_connected
} s_connection_state = state_disconnected;

typedef struct

{
    m_comm_report_data_t buffer[PACKET_BUFFER_SIZE]; 
    uint32_t start_idx;
    uint32_t end_idx;
} s_packet_buffer_t;
static s_packet_buffer_t s_packet_buffer;
static bool            s_buffer_timer_running = false;
static app_timer_id_t  s_packet_buffer_id = 0;
static app_timer_id_t  s_gzll_keep_alive_id = 0;
static protocol_mode_t s_protocol_mode;
static uint8_t         s_oob_key[16];

// Static function declarations
static void m_keyboard_handler(void * p_event_data, uint16_t event_size);
static void m_coms_handler(void * p_event_data, uint16_t event_size);
static void m_batt_meas_handler(void * p_event_data, uint16_t event_size);
static void hid_boot_mode_handler(m_coms_hid_boot_pkt_t *      p_boot_pkt, 
                                  m_coms_ble_hid_boot_type_t * p_pkt_type, 
                                  uint8_t *                    p_data, 
                                  uint8_t                      p_len, 
                                  uint8_t                      p_hid_interface, 
                                  uint8_t                      p_report_idx);

static uint8_t            s_passkey_idx = 0;
static uint8_t            s_passkey_buf[6];
static bool               s_waiting_for_passkey = false;
static m_comm_report_data_t  s_gzll_keepalive_keyboard_pkt;
static void buffer_timer_handler(void* p_context);
static uint32_t gzll_keep_alive_init(void);
static void gzll_keep_alive_handler(void* p_context);
static uint8_t s_hid_interface_idx;
static struct
{
    uint8_t keyboard_rep_idx;
	uint8_t input_rep_idx[NUM_INPUT_REPORT];
    uint8_t output_rep_idx;
    uint8_t feature_rep_idx;
} s_hid_reports;


/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{

    // Copying parameters to static variables because parameters are not accessible in debugger.
    static volatile uint8_t  s_file_name[128];
    static volatile uint16_t s_line_num;
    static volatile uint32_t s_error_code;

    strcpy((char *)s_file_name, (const char *)p_file_name);
    s_line_num   = line_num;
    s_error_code = error_code;
    UNUSED_VARIABLE(s_file_name);
    UNUSED_VARIABLE(s_line_num);
    UNUSED_VARIABLE(s_error_code);  
	dbgmode_printf(DBG_MAIN,"app_error_handler code:%d:%s",error_code,dbg_erro_string(error_code));
#ifndef DEBUG_NRF_USER    
    NVIC_SystemReset();
#endif /* DEBUG_NRF_USER */

    for (;;)
    {
        // Loop forever. On assert, the system can only recover on reset.
    }
}

/**@brief Assert macro callback function.
 *
 * @details This function will be called if the ASSERT macro fails.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    app_error_handler(0, line_num, file_name);
}

/**@brief Module initialization.
 */

static void modules_init(void)
{
    m_prw_and_clk_mgmt_init_t pwr_and_clk_params;
    m_coms_init_t             m_coms_params;
    uint32_t                  err_code;
    ble_gap_addr_t            ble_addr;
    // Led module init
    err_code = m_led_init();
    // Battery measurement module
    err_code =  m_batt_meas_init(m_batt_meas_handler, 3);
    APP_ERROR_CHECK(err_code);     
    
    // Communication module init
    m_coms_params.protocol = protocol_mode_auto;
    M_COMS_BLE_PARAMS_FILL(&m_coms_params.ble_params);
    M_COMS_GZLL_PARAMS_FILL(&m_coms_params.gzll_params);
    m_coms_params.event_callback = m_coms_handler;
    m_coms_params.ble_params.boot_mode_callback = hid_boot_mode_handler;

    err_code = m_coms_init(&m_coms_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = m_coms_ble_report_descriptor_add(s_keyboard_hid_descriptor, sizeof(s_keyboard_hid_descriptor), ble_boot_pkt_keyboard, &s_hid_interface_idx);
    APP_ERROR_CHECK(err_code);


    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_input, false, INPUT_REP_REF_BUTTONS_ID, ATTR_LEN_HID_BUTTON_REPORT, &s_hid_reports.keyboard_rep_idx);
    APP_ERROR_CHECK(err_code);
	s_hid_reports.input_rep_idx[filter_ID_to_IDX(INPUT_REP_REF_BUTTONS_ID)]=s_hid_reports.keyboard_rep_idx;

	dbgmode_printf(DBG_COMS_HID_DESCRIPTOR,"keyboard_rep_idx= %d",s_hid_reports.keyboard_rep_idx);

		
      err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_input, false, INPUT_REP_REF_CONSUMER_ID, ATTR_LEN_HID_CONSUMER_REPORT, &s_hid_reports.keyboard_rep_idx);
    APP_ERROR_CHECK(err_code);
	dbgmode_printf(DBG_COMS_HID_DESCRIPTOR,"keyboard_rep_idx= %d",s_hid_reports.keyboard_rep_idx);  
	s_hid_reports.input_rep_idx[filter_ID_to_IDX(INPUT_REP_REF_CONSUMER_ID)]=s_hid_reports.keyboard_rep_idx;
	


	
    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_output, false, 0, 1, &s_hid_reports.output_rep_idx);
    APP_ERROR_CHECK(err_code);
    
    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_feature, false, 0, 2, &s_hid_reports.feature_rep_idx);
    APP_ERROR_CHECK(err_code);
	dbgmode_printf(DBG_COMS_HID_DESCRIPTOR,"keyboard_rep_idx= %d",s_hid_reports.keyboard_rep_idx);
    err_code = m_coms_enable();
    APP_ERROR_CHECK(err_code);    
	#ifdef NFC_EN
    dbgmode_printf(DBG_MAIN,"m_coms_ble_addr_get");
    // NFC module init
    err_code = m_coms_ble_addr_get(&ble_addr);
    if (err_code != NRF_ERROR_SOFTDEVICE_NOT_ENABLED)
    {
        APP_ERROR_CHECK(err_code);
    }
    dbgmode_printf(DBG_MAIN,"m_nfc_init");
    err_code = m_nfc_init(ble_addr.addr,
                          s_oob_key,
                          (uint8_t *)m_coms_params.ble_params.device_info.device_name,
                          strlen(m_coms_params.ble_params.device_info.device_name), 
                          m_coms_params.ble_params.appearance);
    APP_ERROR_CHECK(err_code);
	#endif
    dbgmode_printf(DBG_MAIN,"m_keyboard_init");
    // Keyboard module. 
    err_code = m_keyboard_init(keyboard_format_usbhid, m_keyboard_handler);    
    APP_ERROR_CHECK(err_code); 

    dbgmode_printf(DBG_MAIN,"m_keyboard_matrix_enable");
    m_keyboard_matrix_enable();
	
    // Power management module
    pwr_and_clk_params.sysoff.callbacks[0] = m_keyboard_wakeup_prepare;
    pwr_and_clk_params.sysoff.callbacks[1] = m_coms_wakeup_prepare;
    pwr_and_clk_params.sysoff.callbacks[2] = m_batt_meas_wakeup_prepare;
	pwr_and_clk_params.sysoff.callbacks[3] = m_led_wakeup_prepare;
    pwr_and_clk_params.sysoff.num          = 4;
    pwr_and_clk_params.idle.num            = 0;
    pwr_and_clk_params.sysoff_timeout      = DEFAULT_BTLE_INACTIVITY_DISCONNECT_PERIOD;
    pwr_and_clk_params.idle_timeout        = DEFAULT_IDLE_TIMEOT; 
    pwr_and_clk_params.lfclk_cal_interval  = (8000/250); // 8000ms LFCLK calibration interval. 250ms pr tick.    
    
    err_code = m_pwr_and_clk_mgmt_init(&pwr_and_clk_params);
    APP_ERROR_CHECK(err_code);
    m_led_poweron();


}

/**@brief Initialization of unused I/Os
 */
static void misc_io_init(void)
{
    int i;
    uint32_t unused_inputs[] = 
        //{IO_MISC_RXD_PIN, IO_MISC_TXD_PIN/*, 26, 27*/};
    	{IO_MISC_RXD_PIN, IO_MISC_TXD_PIN,IO_STRFNFCA_EN_NFC_PIN,IO_PAIRING_BTN_PIN/*, 26, 27*/};
    uint32_t unused_outputs[] = {20};

    for (i = 0; i < (sizeof(unused_inputs) / sizeof(unused_inputs[0])); ++i)
    {
        uint32_t pin = unused_inputs[i];
        
        NRF_GPIO->PIN_CNF[pin] =
            (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)    |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)  |
            (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)   |
            (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)  |
            (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);           
    }
    
    for (i = 0; i < (sizeof(unused_outputs) / sizeof(unused_outputs[0])); ++i)
    {
        uint32_t pin = unused_outputs[i];
        
        NRF_GPIO->PIN_CNF[pin] =
            (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)    |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)  |
            (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)   |
            (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)  |
            (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
        NRF_GPIO->OUTCLR = (1 << pin);
    }    
}

/**@brief Packet buffering initialization
 */
static void buffer_init(void)
{
    uint32_t err_code;

    memset(&s_packet_buffer, 0, sizeof(s_packet_buffer));
    s_packet_buffer.start_idx = 0;
    s_packet_buffer.end_idx = 0;

    err_code =  app_timer_create(&s_packet_buffer_id,
                               APP_TIMER_MODE_REPEATED,
                               buffer_timer_handler);   
    APP_ERROR_CHECK(err_code);

}

/**@brief Utility function to buffer packets that fails to be sent
 */
static void buffer_keys(const m_comm_report_data_t * packet)
{
	dbgmode_printf(DBG_FREE,"#b");

    if ((s_packet_buffer.end_idx + 1) % PACKET_BUFFER_SIZE == s_packet_buffer.start_idx) 
    {
        // Buffer is full. Won't overwrite.
        return;
    }
    else
    {
        uint32_t err_code;

        memcpy(&s_packet_buffer.buffer[s_packet_buffer.end_idx], packet, sizeof(s_packet_buffer_t));
        s_packet_buffer.end_idx = (s_packet_buffer.end_idx + 1) % PACKET_BUFFER_SIZE;
        
        if (s_buffer_timer_running == false)
        {
            // Start the timeout handler
            err_code = app_timer_start(s_packet_buffer_id, APP_TIMER_TICKS(15 /* [ms] */, APP_TIMER_PRESCALER), 0);
            APP_ERROR_CHECK(err_code);
            s_buffer_timer_running = true;
        }
    }
}
static uint32_t m_coms_types_hid_report_send(const m_comm_report_data_t * report_data )
{
	uint32_t err_code=NRF_SUCCESS;
	uint8_t i;
	uint8_t report[8]={0};
	uint8_t report_id = report_data->report_id;
	dbgmode_printf(DBG_REPORT_SEND,"report_id=%d",report_id);
	switch(report_id)
	{
		case INPUT_REP_REF_BUTTONS_ID:
			{		
				standard_report_t *standard_report = (standard_report_t*)(report_data->report_struct_buff);	
            	report[0] = standard_report->modifierKeys;
					
	            for ( i = 0; i < report_data->num_keys; i++)
	            {
	                report[i + 2] = standard_report->key[i];
	            }
				#if DBG_EN
				dbgmode_printf(DBG_REPORT_SEND,"report_data:%d,%d,%d,%d,%d,%d,%d,%d"
								,report[0],report[1],report[2],report[3],
								report[4],report[5],report[6],report[7]);
				#endif
			}				
			break;
		case 	INPUT_REP_REF_CONSUMER_ID:
	            for ( i = 0; i < 2; i++)
	            {
	            	//the same to consmer_report_t no need to change types
	                report[i] = report_data->report_struct_buff[i];
	            }
				#if DBG_EN
				dbgmode_printf(DBG_REPORT_SEND,"report_data:%X %X"
								,report[0],report[1]);
				#endif				
			break;
		case INPUT_REP_REF_SYSTEM_ID:
				for ( i = 0; i < 1; i++)
	            {
	            	//the same to consmer_report_t no need to change types
	                report[i] = report_data->report_struct_buff[i];
	            }
			//not added yeat;
			break;
		default:

			return err_code;
	}
    err_code = m_coms_hid_report_send(report, 
                                      filter_ID_to_reportLen(report_id), 
                                      0,
                                      s_hid_reports.input_rep_idx[filter_ID_to_IDX(report_id)]);
    if (err_code != NRF_SUCCESS)
    { 
        buffer_keys(report_data);
    }
	return err_code;
}


/**@brief Timeout function used when packet buffer is not empty
 */
static void buffer_timer_handler(void* p_context)
 {    
     uint32_t err_code;
     //uint8_t report[8] = {0};
	dbgmode_printf(DBG_FREE,"#h");
     if (s_connection_state == state_disconnected)
     {
         // No point in trying to send if we are disconnected.
         return;
     }
     
     if (s_packet_buffer.end_idx == s_packet_buffer.start_idx)
     {
         // Buffer is empty
         if (s_buffer_timer_running)
         {
             err_code = app_timer_stop(s_packet_buffer_id);
             APP_ERROR_CHECK(err_code);
             s_buffer_timer_running = false;
         }
         
         return;
     }

	#if 0 
    report[0] = s_packet_buffer.buffer[s_packet_buffer.start_idx].modifier_keys;
	uint8_t reportID = s_packet_buffer.buffer[s_packet_buffer.start_idx].report_ID;  
    
    for (uint32_t i = 0; i < s_packet_buffer.buffer[s_packet_buffer.start_idx].num_keys; i++)
    {
        report[i + 2] = s_packet_buffer.buffer[s_packet_buffer.start_idx].keys[i];
    }
    
    err_code = m_coms_hid_report_send(report, 
                                      filter_ID_to_reportLen(reportID), 
                                      s_hid_interface_idx,
	s_hid_reports.input_rep_idx[filter_ID_to_IDX(reportID)]);//s_hid_reports.keyboard_rep_idx);   
	#endif
			
			m_comm_report_data_t *keyboard_pkt = (m_comm_report_data_t *)&s_packet_buffer.buffer[s_packet_buffer.start_idx].report_id;
			err_code =m_coms_types_hid_report_send(keyboard_pkt);

     if (err_code == NRF_SUCCESS)
     {
         s_packet_buffer.start_idx = (s_packet_buffer.start_idx + 1) % PACKET_BUFFER_SIZE;
         
         if (s_packet_buffer.end_idx == s_packet_buffer.start_idx)
         {
             if (s_buffer_timer_running)
             {
                 err_code = app_timer_stop(s_packet_buffer_id);
                 APP_ERROR_CHECK(err_code);
                 s_buffer_timer_running = false;
             }
         }
     }
 }



/**@brief  Initialization of keep alive timer for gazell.
 */
static uint32_t gzll_keep_alive_init(void)
{
    return app_timer_create(&s_gzll_keep_alive_id, APP_TIMER_MODE_REPEATED, gzll_keep_alive_handler);
}

/**@brief  keep alive timer handler for gazell. Sends empty motion packets.
 */
static void gzll_keep_alive_handler(void* p_context)
{
    // Put the previous keyboard packet in buffer.
   // buffer_keys(&s_gzll_keepalive_keyboard_pkt);   
}

/**@brief Callback function used to re-assemble packets in HID boot mode.
 *
 * @details When in HID boot mode, a non-configurable packet format is used for mouse and keyboard reports.
 *          Applicable reports (in this case keyboard reports) needs to be re-formatted to fit the Boot format.
 * @note Only Boot keyboard and Boot mouse HID reports can be sent when in Boot mode.
 *
 * @param[in\out] p_packet      Packet which needs re-assembly
 * @param[out]    p_packet_type Type of packet. If type is @ref ble_boot_pkt_none the packet is discarded.
 */
static void hid_boot_mode_handler(m_coms_hid_boot_pkt_t *      p_boot_pkt, 
                                  m_coms_ble_hid_boot_type_t * p_pkt_type, 
                                  uint8_t *                    p_data, 
                                  uint8_t                      p_len, 
                                  uint8_t                      p_hid_interface, 
                                  uint8_t                      p_report_idx)
{
	dbgmode_printf(DBG_MAIN,"hid_boot_mode_handler");
    if (p_hid_interface == s_hid_interface_idx && p_report_idx == s_hid_reports.keyboard_rep_idx)
    {
        // Keyboard report: boot packet and standard keyboard packet has the same format
        memcpy(p_boot_pkt->keyboard_data.keys, p_data, p_len);
        *p_pkt_type = ble_boot_pkt_keyboard;
    }
    else
    {
        // Unknown report
    }
}


/**@brief Handler function for keyboard module events
 */
static void m_keyboard_handler(void * p_event_data, uint16_t event_size)
{

    uint32_t            err_code;
    m_comm_report_data_t * keyboard_pkt;
    
    ASSERT(event_size == sizeof(m_comm_report_data_t));
	// dbgmode_printf(DBG_MAIN,"m_keyboard_handler");
    keyboard_pkt = (m_comm_report_data_t *) p_event_data;

    if (keyboard_pkt->pairing_button)
    {
    	dbgmode_printf(DBG_MAIN,"pairing_button");
        uint8_t release_report[8] = {0};
        
        app_timer_stop(s_gzll_keep_alive_id);
        
        // Stop buffer timer and flush
        if (s_buffer_timer_running)
        {
            err_code = app_timer_stop(s_packet_buffer_id);
            APP_ERROR_CHECK(err_code);
            s_buffer_timer_running = false;
        }
        
        s_packet_buffer.start_idx = 0;
        s_packet_buffer.end_idx = 0;
        
        // Send release packet
		err_code = m_coms_hid_report_send(release_report, 
		                                  BUTTON_REPORT_LEN, 
		                                  0,
		                                  s_hid_reports.input_rep_idx[filter_ID_to_IDX(INPUT_REP_REF_BUTTONS_ID)]);
		 err_code = m_coms_hid_report_send(release_report, 
		                                  CONSUMER_REPORT_LEN, 
		                                  0,
		                                  s_hid_reports.input_rep_idx[filter_ID_to_IDX(INPUT_REP_REF_CONSUMER_ID)]);        
        // Start bonding
        m_coms_bonding_start();
    }
    else
    {
        if (s_waiting_for_passkey)
        {
            if (keyboard_pkt->num_keys > 0)
            {
                memcpy(&s_passkey_buf[s_passkey_idx], keyboard_pkt->report_struct_buff, keyboard_pkt->num_keys);
                s_passkey_idx += keyboard_pkt->num_keys;
                
                if (s_passkey_idx >= 6)
                {
                    err_code = m_coms_ble_passkey_set(s_passkey_buf);
                    APP_ERROR_CHECK(err_code);
                    s_passkey_idx = 0;
                }
            }
        }
        else
        {
			m_coms_types_hid_report_send(keyboard_pkt);
            
            memcpy(&s_gzll_keepalive_keyboard_pkt, keyboard_pkt, sizeof(m_comm_report_data_t));
        
            if (s_protocol_mode == protocol_mode_gzll)
            {
                if (m_keyboard_packet_is_empty(keyboard_pkt))
                {
                    app_timer_stop(s_gzll_keep_alive_id);
                }
                else
                {       
                    app_timer_start(s_gzll_keep_alive_id, APP_TIMER_TICKS(500, APP_TIMER_PRESCALER), 0);
                }
            }
        }
    }
	
	   //dbgmode_printf(DBG_COMS_HID_DESCRIPTOR,"keyboard_rep_idx= %d",s_hid_reports.keyboard_rep_idx);

    // Notifying the power manager of activity
    m_pwr_mgmt_feed();
}

/**@brief Handler function for communication module events
 */
static void m_coms_handler(void * p_event_data, uint16_t event_size)
{
    m_coms_evt_t * evt;
    ble_gap_addr_t ble_addr;
    uint32_t       err_code;
    uint8_t        rng_tries;
    
    ASSERT(event_size == sizeof(m_coms_evt_t));
    
    evt = (m_coms_evt_t *) p_event_data;
	 dbgmode_printf(DBG_MAIN,"m_coms_handler type:%s",dbg_com_event_string(evt->type));
    // Events coming from the communications module.
    switch (evt->type)
    {
        case com_event_connected:
            s_connection_state = state_connected;
            m_keyboard_matrix_enable();

            s_protocol_mode = m_coms_protocol_mode_get();  
            if (s_protocol_mode == protocol_mode_ble)
            {
                app_timer_stop(s_gzll_keep_alive_id);
                m_prw_mgmt_set_sysoff_timeout(DEFAULT_BTLE_INACTIVITY_DISCONNECT_PERIOD);
            }
            else
            {
                m_prw_mgmt_set_sysoff_timeout(DEFAULT_GZLL_INACTIVITY_DISCONNECT_PERIOD);
            }
			m_led_setsubstate(DISCONNECTED);

            break;

        case com_event_data_received:
            break;

        case com_event_timing_update:
            break;

        case com_event_advertising_bondable:
            m_keyboard_matrix_disable();
        	dbgmode_printf(DBG_COMS_BLE_ADV,"com_event_advertising_bondable");
            rng_tries = 3;
            err_code  = NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES;
            while (err_code == NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES && rng_tries != 0)
            {
                err_code   = hal_rng_vector_get(s_oob_key, 16);
                rng_tries -= 1;
            }
			#ifdef NFC_EN
            m_nfc_set_oob_key(s_oob_key);        
            m_nfc_enable();
			#endif
            break;
        
        case com_event_address_changed:
            m_coms_ble_addr_get(&ble_addr);
			#ifdef NFC_EN
            m_nfc_set_ble_addr(ble_addr.addr);
			#endif
			dbgmode_printf(DBG_MAIN,"m_coms_handler type:%s Type:%X,%X%X%X%X%X%X",dbg_com_event_string(evt->type),ble_addr.addr_type,
			ble_addr.addr[0],ble_addr.addr[1],ble_addr.addr[2],ble_addr.addr[3],ble_addr.addr[4],ble_addr.addr[5]
			);

            break;
        
        case com_event_passkey_req:
			#ifdef NFC_EN
            m_nfc_disable();
			#endif
            m_keyboard_matrix_enable();
            m_keyboard_format_set(keyboard_format_ascii);
            s_waiting_for_passkey = true;
            break;
        
        case com_event_oobkey_req:
            m_coms_ble_oobkey_set(s_oob_key);
			#ifdef NFC_EN
            m_nfc_disable();
			#endif
            m_keyboard_matrix_enable();
            break;        

        case com_event_key_sent:
            m_keyboard_format_set(keyboard_format_usbhid);
            s_waiting_for_passkey = false;            
            break;
        case com_event_disconnected:
            s_connection_state = state_disconnected;
         /* Fall through */
        case com_event_advertising_timeout:
			dbgmode_printf(DBG_COMS_BLE_ADV,"com_event_advertising_timeout OR com_event_disconnected");
			#ifdef IO_PAIR_KEY
			if (!m_keyboard_pairing_btn_pressed())
			#endif
			{
                m_pwr_mgmt_goto_sysoff();
            }
            break;

        default:
         break;
    }
 }

/**@brief Handler function for battery module events
 */
static void m_batt_meas_handler(void * p_event_data, uint16_t event_size)
{
	 dbgmode_printf(DBG_MAIN,"m_batt_meas_handler~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
     uint32_t err_code;
     
     ASSERT(event_size == 1);
     
     err_code = m_coms_ble_battery_level_update(*(uint8_t*) p_event_data);
     if (err_code != NRF_SUCCESS)
     {
         // Don't care
     }else
     {
		if(*(uint8_t*) p_event_data<M_BATT_MEAS_WARRING_LEVEL_P)
		m_led_lowbatery();
		else
		m_led_exitlowbatery();
	}
}


/**@brief Application main function.
 */
int main(void)
{   
    NRF_POWER->DCDCEN = (POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos); 
    
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    buffer_init();
    gzll_keep_alive_init();
	debug_print_init(0, BIAO_UART_TXD_PIN_NUMBER, 0, BIAO_UART_RXD_PIN_NUMBER, false,NULL,false);
	dbgmode_printf(DBG_MAIN,"%s","power up");    
	modules_init();
    misc_io_init();
	//added by biao

	        // Start bonding
    //    m_coms_bonding_start();
	dbgmode_printf(DBG_MAIN,"%s","Enter main loop");    
		dbgmode_printf(DBG_COMS_HID_DESCRIPTOR,"keyboard_rep_idx= %d",s_hid_reports.keyboard_rep_idx);

    // Enter main loop
    for (;;)
    {
   // dbgmode_printf(DBG_FREE, "_");
        app_sched_execute();
        m_pwr_mgmt_run();
    }
}

