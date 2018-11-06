/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
//#include "ble_conn_params.h"
//#include "nrf_sdh.h"
//#include "nrf_sdh_ble.h"
//#include "app_button.h"
//#include "ble_lbs.h"
//#include "nrf_ble_gatt.h"
//#include "nrf_ble_qwr.h"
//#include "nrf_pwr_mgmt.h"
//#include "ble_gap.h"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"
//#include "ble_advertising.h"

#include <string.h>
#include "boards.h"
#include "app_timer.h"

#include "system.h"
#include "radio.h"



/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
*/
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	// Initialize timer module, making it use the scheduler
	while( app_timer_init() != NRF_SUCCESS );
}






//test switch teacher
/*void switch_teaching_test( void )
{
	static uint8_t sent = 0;
	if( hub.switches[0].ID != 0 && hub.nodes[0].ID != 0 && sent == 0 )
	{
		inspired_packet_send_assign_switch( hub.switches[0].ID, hub.switches[0].mask, hub.nodes[0].ID );
		sent = 1;
	}
}*/


//Test advertising
void broadcast_test( void )
{
	static uint16_t 	count = 1;
	uint8_t 					data[BLE_ADV_BYTES_MAX];
	
	//construct packet
	for( int i = 0; i < 8; i++ )
		data[i] = 0xFF >> i;
	
	//increment packet count (final two bytes)
	count++;
	data[8] = (count & 0xFF00) >> 8;
	data[9] = (count & 0x00FF) >> 0;
	
	//send packet to outbox
	packet_push_outbox( data, 10 );
	
	count++;
	data[8] = (count & 0xFF00) >> 8;
	data[9] = (count & 0x00FF) >> 0;
	packet_push_outbox( data, 10 );
}


uint32_t timer = 0;
uint32_t timer_shadow = 0;
void timer_increment_task( void )
{
	timer++;
}

/**@brief Function for application main entry.
 */
int main(void)
{
	// Initialize.
	timers_init();								//NRF library uses this for button polling, replace eventually
	leds_init();
	buttons_init();
	
	//memset( (void*) &hub, 0, sizeof(hub) );
	
	sys_init();
	//packet_handler_init();
	
	ble_stack_init();
	
	gap_params_init();
	gatt_init();
	conn_params_init();
	services_init();
	advertising_init();
	advertising_start();
	
	observe_start();
	
	//sys_task( broadcast_test, 3000 );
	sys_task( timer_increment_task, 1 );	//packet handling moved to sys task via packet_handling_init
	
	// Enter main loop.
	while( true )
	{
		if( timer != timer_shadow )
		{
			if( timer % 3000 == 0 )
				broadcast_test();
			
			if( timer % 333 == 0 && timer != timer_shadow )
				packet_handler();
		}
		
		timer_shadow = timer;
	}
}


/**
 * @}
 */
