//#include <stdint.h>
//#include <string.h>
//#include "nordic_common.h"
//#include "nrf.h"
//#include "app_error.h"
//#include "ble.h"
//#include "ble_err.h"
//#include "ble_hci.h"
//#include "ble_srv_common.h"
//#include "ble_advdata.h"
//#include "nrf_sdh.h"
//#include "nrf_sdh_ble.h"
//#include "nrf_pwr_mgmt.h"
//#include "ble_gap.h"
//#include "ble_advertising.h"
//#include "nrf52.h"
//#include "nrf_sdh_ble.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_log.h"
#include "nrf_sdh.h"

#include "radio.h"
#include "hub.h"
#include "enocean.h"
#include "system.h"

/* Module Instances */
BLE_LBS_DEF(m_lbs);																															//LED Button Service instance
NRF_BLE_GATT_DEF(m_gatt);																												//GATT module instance. 
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);													//Context for the Queued Write module.

/* Connection Handle */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;												//Handle of the current connection. 

/* Universal Advertising Data */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;										//Advertising handle used to identify an advertising set

/* Connectable Advertising Data */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];              			//Buffer for storing an encoded advertising set. 
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];					//Buffer for storing an encoded scan data. 
static ble_gap_adv_data_t m_adv_data =
{
	.adv_data =
	{
		.p_data = m_enc_advdata,
		.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
	},
	.scan_rsp_data =
	{
		.p_data = m_enc_scan_response_data,
		.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
	}
};

/* Broadcaster Advertising Data */
volatile	packet_buffer_t 		packet_inbox;																			//structure for inbox ring buffer
volatile	packet_buffer_t 		packet_outbox;																		//structure for outbox ring buffer
static		uint8_t 						m_enc_bcast_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; 	//array for storing an encoded advertising set
static		ble_gap_adv_data_t 	m_bcast_data =																		//structure that contains a pointer to encoded advertising data and the length of that data
{
	.adv_data =
	{
			.p_data = m_enc_bcast_data,
			.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
	}
};

/* Observer Data */	
static uint8_t ad_buffer[BLE_GAP_SCAN_BUFFER_MIN];
static ble_data_t observerAdBuf = 
{
	ad_buffer,
	BLE_GAP_SCAN_BUFFER_MIN
};
static ble_gap_scan_params_t scanParameters =
{
	.extended = 0,																//Accept extended advertising packets?
	.report_incomplete_evts = 0,									//Report incomplete events? (only relevant to extended scanning)
	.active = 0,																	//Perform active scanning by sending scan requests?
	.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,	//Accept all advertising packets except directed advertising packets not addressed to this device.
	.scan_phys = BLE_GAP_PHY_1MBPS,								//Bitfield of PHYs to scan on.
	.interval = 0x0020,														//Scan interval, in 625us increments
	.window = 0x0050,															//Scan window, in 625us increments
	.timeout = 0x0000,														//Scan timeout, in 10ms increments (0 disables timeout)
	.channel_mask = 0															//Channel mask for primary and secondary advertising channels. At least one of the primary channels, that is channel index 37-39, must be set to 0.
};

























/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            NRF_LOG_INFO("Send button state change.");
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


























/**@brief Function for pushing a packet into a packet buffer
 *
 * @param[in] pbuf	        The buffer to push the packet into.
 * @param[in] packet				The packet to push into the buffer.
 * @param[in] length				Number of bytes in packet.
 */
static void packet_buffer_push( volatile packet_buffer_t* pbuf, uint8_t* packet, uint8_t length )
{
	uint8_t new_head = pbuf->head + 1;
	
	//handle wrap around
	if( new_head >= PACKET_BUFFER_SIZE )
		new_head = 0;
	
	//handle full buffer
	if( new_head == pbuf->tail )
	{
		//TODO: return false?
		return;	//buffer is full, cannot record new packet
	}
	
	//check for duplicate by first comparing last bit
	for( int i = pbuf->tail; i != pbuf->head; i++ )
	{
		if( packet[length - 1] == pbuf->packet[i][length - 1] )
		{
			//final bits matched, now confirm duplicate
			int j;
			for( j = 0; j < PACKET_BUFFER_SIZE && packet[j] == pbuf->packet[i][j]; j++ );
			
			if( j == PACKET_BUFFER_SIZE )
				return; //confirmed duplicate, do not record new packet
		}
		
		if( i == PACKET_BUFFER_SIZE - 1 )
			i = -1;//will be incremented to zero
	}
	
	//record new packet
	for( int k = 0; k < BLE_ADV_BYTES_MAX; k++ )
	{
		if( k < length )
			pbuf->packet[pbuf->head][k] = packet[k];
		else
			pbuf->packet[pbuf->head][k] = 0;
	}
	pbuf->head = new_head;
}


/**@brief Function for popping the next packet out of a packet buffer
 *
 * @param[in] pbuf	        The buffer to push the packet into.
 * @param[in] buff					Array that the packet will be returned to.
 * @param[in] length				Number of bytes in returned packet.
 *
 * @param[return] Returns true if a packet was available to be popped, otherwise false.
 */
static bool packet_buffer_pop( volatile packet_buffer_t* pbuf, uint8_t buff[BLE_ADV_BYTES_MAX], uint8_t* length )
{
	uint8_t new_tail = pbuf->tail + 1;
	
	//handle no data
	if( pbuf->tail == pbuf->head )
		return false;
	
	//handle wrap around
	if( new_tail >= PACKET_BUFFER_SIZE )
		new_tail = 0;
	
	//read data
	for( int i = 0; i < BLE_ADV_BYTES_MAX; i++ )
	{
		buff[i] = pbuf->packet[pbuf->tail][i];
		if( buff[i] != 0 )
			*length = i + 1;
	}
	
	pbuf->tail = new_tail;
	return true;
}


/**@brief Function for undoing the most recent pop. For when a popped packet fails to transmit.
 *
 * @param[in] pbuf	        The buffer to unpop.
 *
 */
static void packet_restore_previous( volatile packet_buffer_t* pbuf )
{
	uint8_t new_tail = pbuf->tail - 1;
	
	//handle wrap around
	if( new_tail >= PACKET_BUFFER_SIZE )
		new_tail = PACKET_BUFFER_SIZE - 1;
	
	pbuf->tail = new_tail;
}

/*
void packet_push_outbox( uint8_t* packet, uint8_t length )
{
	packet_buffer_push( &packet_outbox, packet, length );
}
*/

/**@brief Function for regularly handling incoming and outgoing packets
 *					Register as a system task, called at regular intervals
 * 					If there is a pending outgoing packet, send it
 *					If there is a pending incoming packet, handle it
 */
void packet_handler( void )
{
	uint8_t packet[BLE_ADV_BYTES_MAX];
	uint8_t packetLength;
	
	//Send next pending packet in outbox
	if( packet_buffer_pop( &packet_outbox, packet, &packetLength ) )
	{
		//try to transmit packet. if not successful, restore packet's place in buffer
		if( broadcast_packet( packet, packetLength ) == false )
		{
			packet_restore_previous( &packet_outbox );
		}
	}
	
	//Process next pending packet in inbox
	if( packet_buffer_pop( &packet_inbox, packet, &packetLength ) )
	{
		//TODO: fix RSSI value -- should not be hardcoded to 0
		inspired_packet_handler( 0, 0, packet, packetLength );
	}
}

void packet_handler_init( void )
{
	sys_task( packet_handler, 333 );
}








/**@brief Function for starting or restarting passive scanning for advertising packets.
 * 
 *
 */
void observe_start( void )
{	
	sd_ble_gap_scan_stop();
	
	//start observing, don't stop trying until you succeed
	while( sd_ble_gap_scan_start( &scanParameters, &observerAdBuf ) != NRF_SUCCESS );
}


/**@brief Function for handling (non-connectable) advertising packets.
 *
 * @details Called when ble_evt_handler receives a BLE_GAP_EVT_ADV_REPORT event ID.
 *
 */
static void on_adv_report( const ble_gap_evt_adv_report_t* adv_report )
{
	uint8_t*				packet				= adv_report->data.p_data;																									//Pointer to advertising packet data
	uint16_t				packetLength	= adv_report->data.len;																											//Advertisement packet length
	int8_t					rssi					= adv_report->rssi;																													//Received signal strength
	const uint8_t*	bleID					= adv_report->peer_addr.addr;																								//array of bytes forming bluetooth id
	uint16_t				bleID_MSB			= (bleID[5] << 8) | (bleID[4] << 0);																				//two most significant bytes from bluetooth ID
	uint32_t				bleID_LSB			= (bleID[3] << 24) | (bleID[2] << 16) | (bleID[1] << 8) | (bleID[0] << 0);	//four lest significant bytes from bluetooth ID
	uint16_t				enoceanID			= (packet[3] << 8) | (packet[2] << 0);																			//enOcean manufacturer id 
	uint16_t				inspiredID		= (packet[4] << 8) | (packet[5] << 0);																			//inspired identification
	
	
	//Is advertising signal from an EnOcean switch? 
	//If so, upper two bytes of bluetooth ID should be 0xE215
	//And the manufacturer ID in the Advertising packet should be 0x03DA.
	if( bleID_MSB == ENOCEAN_BLUETOOTH_ID_MSB && enoceanID == ENOCEAN_MANUFACTURE_ID ) 
	{			
		uint8_t switch_status = packet[8];
		
		enocean_switch_handler( bleID_LSB, switch_status );
	}
	//If it's not an EnOcean switch, is it an Inspired LED device?
	//If so, it should have 0x13D5 in the inspiredID field
	//And it should be validated with the parity bit
	else if( inspiredID == INSPIRED_MANUFACTURE_ID )
	{
		uint8_t checksum = 0;
		for( int i = 4; i < packetLength; i++ )
			checksum ^= packet[i];
		if( checksum == 0 )
		{
			//valid inspired packet, checksum confirmed
			//inspired_packet_handler( bleID_LSB, rssi, packet, packetLength );
			
			//TODO: push this to be handled at regular intervals: packet_buffer_push( packet, packetLength );
			//			-must implement system tasks
			packet_buffer_push( &packet_inbox, packet, packetLength );
			
		}
	}
}










/**@brief Function for broadcasting advertising packets.
 *
 * @details Provide a pointer to the packet data, provide the length of the data
 *						Function returns true when packet is sent successfully, false if tranmission fails.
 *
 * */
bool broadcast_packet( uint8_t* data, uint8_t length )
{
	bool								      success = true;
	ble_advdata_t             adv_data;
	ble_gap_adv_params_t			adv_params;
	ble_advdata_manuf_data_t	mfg_data;

	//clear data structures
	memset( &adv_data, 0, sizeof(adv_data) );
	memset( &adv_params, 0, sizeof(adv_params) );
	memset( &mfg_data, 0, sizeof(mfg_data) );
	
	//global m_bcast_data.adv_data.len needs to be reset between packets
	//when a packet is sent, len is updated to the packet length (truncating any trailing zeros)
	//if a longer packet is sent after a shorter packet without updating len, ble_adv_data_enc will return 0x0C -- invalid data size
	//apparently, len is updated AFTER the size check in ble_adv_data_enc -->> manuf_specific_data_enc
	m_bcast_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;	
	
	//check packet length, truncate if too long
	if( length > BROADCAST_LENGTH_MAX )
		length = BROADCAST_LENGTH_MAX;

	//configure custom data
	mfg_data.company_identifier = BROADCAST_COMPANY_ID;
	mfg_data.data.p_data = data;
	mfg_data.data.size = length;

	//point advertising data structure to custom data
	adv_data.p_manuf_specific_data = &mfg_data;

	//encode advertising data in m_bcast_data structure
	if( ble_advdata_encode( &adv_data, m_bcast_data.adv_data.p_data, &m_bcast_data.adv_data.len ) != NRF_SUCCESS )
		return !success;

	//set advertising parameters
	adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;																				
	adv_params.duration        = 12;																											//Duration, in 10ms units
	adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;	//Broadcast packets are non-connectable, non-scannable, and undirected
	adv_params.interval        = 0x30;                                                    //Interval, in 625us units

	//stop any advertisement activity
	sd_ble_gap_adv_stop( m_adv_handle );

	//configure the packet with specified parameters
	if( sd_ble_gap_adv_set_configure(&m_adv_handle, &m_bcast_data, &adv_params) != NRF_SUCCESS )
		return !success;

	//start broadcast
	if( sd_ble_gap_adv_start( m_adv_handle, APP_BLE_CONN_CFG_TAG ) != NRF_SUCCESS )
		return !success;

	return success;
}









/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
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




/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF!");
    }
}




/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

		for( uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++ )
		{
			err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
			APP_ERROR_CHECK(err_code);
		}

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}




/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_on(ADVERTISING_LED);
}




/**@brief Function for assigning new connection handle to available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
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
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            multi_qwr_conn_handle_assign(m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
						break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
						advertising_init();
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
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

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

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

				case BLE_GAP_EVT_ADV_REPORT:
						//handle advertising packet
						on_adv_report( &p_ble_evt->evt.gap_evt.params.adv_report );
						observe_start();						//continue observing role
						break;
				
				case BLE_GAP_EVT_ADV_SET_TERMINATED:
						if( m_conn_handle == BLE_CONN_HANDLE_INVALID )
						{
							advertising_init();
							advertising_start();
						}
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
void ble_stack_init(void)
{
	uint32_t ram_start = 0;
	
	//enable soft device, keep trying until successful
	while( nrf_sdh_enable_request() != NRF_SUCCESS );

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	// Keep trying until successful
	while( nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start) != NRF_SUCCESS );

	// Enable BLE stack, keep at it until successful
	while( nrf_sdh_ble_enable(&ram_start) != NRF_SUCCESS );

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}




