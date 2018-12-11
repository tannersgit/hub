
#include "nrf_gpio.h"
#include "radio.h"
#include "hub.h"

volatile hub_t hub;

void hub_pins_init( void )
{
	//configure pins as outputs
	nrf_gpio_cfg_output( HUB_PIN_LED );
	
	//set outputs low
	nrf_gpio_pin_set( HUB_PIN_LED );
}

void hub_output( bool on )
{
	if( on )
		nrf_gpio_pin_set( HUB_PIN_LED );
	else
		nrf_gpio_pin_clear( HUB_PIN_LED );
}

/* breaks 32 bit number into four 8 bit numbers */
void discretize32( uint32_t thirtyTwo, uint8_t* eight )
{
	eight[0] = (thirtyTwo >> 24) & 0xFF;
	eight[1] = (thirtyTwo >> 16) & 0xFF;
	eight[2] = (thirtyTwo >>  8) & 0xFF;
	eight[3] = (thirtyTwo >>  0) & 0xFF;
}

/* breaks 16 bit number into two 8 bit numbers */
void discretize16( uint16_t sixteen, uint8_t* eight )
{
	eight[0] = (sixteen >> 8) & 0xFF;
	eight[1] = (sixteen >> 0) & 0xFF;
}

/* sends properly formatted inspired led packet */
void inspired_packet_send( uint32_t destinationID, uint8_t* messageData, uint8_t messageLength )
{
	uint8_t length = messageLength + INSPIRED_PACKET_BYTES_MIN;
	uint8_t checksum = 0;
	uint8_t data[BLE_ADV_BYTES_MAX] = { 0 };
	
	data[0] = (INSPIRED_MANUFACTURE_ID >> 8) & 0xFF;
	data[1] = (INSPIRED_MANUFACTURE_ID >> 0) & 0xFF;
	
	//TODO: use MAC address instead?
	discretize32( hub.ID, data + 2 );
	
	discretize32( destinationID, data + 6 );
	
	for( int i = 0; i < messageLength; i++ )
		data[10 + i] = messageData[i];
	
	for( int i = 0; i < length; i++ )
		checksum ^= data[i];
	
	data[length - 1] = checksum;
	
	//push the packet to the outbox to be transmitted when possible
	packet_push_outbox( data, length );
}

/* "hello, node" message */
void inspired_packet_send_hello_node( uint32_t nodeID )
{
	uint8_t data[4] = {0};
	
	discretize32( INSPIRED_MESSAGE_HELLO_NODE, data );
	
	inspired_packet_send( nodeID, data, 4 );
}

/* "node, my node" message */
void inspired_packet_send_node_my_node( uint32_t nodeID )
{
	uint8_t data[4] = {0};
	
	discretize32( INSPIRED_MESSAGE_NODE_MY_NODE, data );
	
	inspired_packet_send( nodeID, data, 4 );
}

/* "assign switch" message */
void inspired_packet_send_assign_switch( uint32_t switchID, uint8_t switchMask, uint32_t nodeID )
{
	uint8_t data[9] = {0};
	
	discretize32( INSPIRED_MESSAGE_ASSIGN_SWITCH, data );
	
	discretize32( switchID, data + 4 );
	
	data[8] = switchMask;
	
	inspired_packet_send( nodeID, data, 9 );
}

/* "remove switch" message */
void inspired_packet_send_remove_switch( uint32_t switchID, uint8_t switchMask, uint32_t nodeID )
{
	uint8_t data[9] = {0};
	
	discretize32( INSPIRED_MESSAGE_REMOVE_SWITCH, data );
	
	discretize32( switchID, data + 4 );
	
	data[8] = switchMask;
	
	inspired_packet_send( nodeID, data, 9 );
}

/* "change state" message */
void inspired_packet_send_change_state( uint16_t brightness, uint16_t color, uint32_t nodeID )
{
	uint8_t data[8] = {0};
	
	discretize32( INSPIRED_MESSAGE_CHANGE_STATE, data );
	
	discretize16( brightness, data + 4 );
	
	discretize16( color, data + 6 );
	
	inspired_packet_send( nodeID, data, 8 );
}

/* "assign group" message */
void inspired_packet_send_assign_group( uint32_t groupID, uint32_t nodeID )
{
	uint8_t data[8] = {0};
	
	discretize32( INSPIRED_MESSAGE_ASSIGN_GROUP, data );
	
	discretize32( groupID, data + 4 );
	
	inspired_packet_send( nodeID, data, 8 );
}



/* handles valid inspired led device packets 
 *		packet validated in radio.c
 *		function declared in radio.h
 */
void inspired_packet_handler( uint32_t id, int8_t rssi, uint8_t* packet, uint8_t packet_length )
{
	uint32_t sourceID = (packet[6] << 24) | (packet[7] << 16) | (packet[8] << 8) | (packet[9] << 0);
	uint32_t recipientID = (packet[10] << 24) | (packet[11] << 16) | (packet[12] << 8) | (packet[13] << 0);
	uint32_t messageID = (packet[14] << 24) | (packet[15] << 16) | (packet[16] << 8) | (packet[17] << 0);
	
	//TODO: confirm packet addressed to node before going deeper
	
	
	switch( messageID ){
		
		case INSPIRED_MESSAGE_HELLO_HUB:
			if( hub.mode != NODE_LEARN )
			{
				//a node is trying to register
				hub.mode = NODE_LEARN;
				hub.counter = 0;																//reset LEARN_NODE counter
				inspired_packet_send_hello_node( sourceID );		//respond to node
			}
			break;
			
		case INSPIRED_MESSAGE_HUB_MY_HUB:
			if( hub.mode == NODE_LEARN )
			{
				//register node
				hub.mode = IDLE;
				for( int i = 0; i < NODES_MAX && hub.nodes[i].ID != sourceID; i++ )
				{
					if( hub.nodes[i].ID == 0 )
					{
						hub.nodes[i].ID = sourceID;
						inspired_packet_send_node_my_node( sourceID );
						break;
					}
				}
			}
			break;
		
		case INSPIRED_MESSAGE_GOT_IT:
			break;
		
		case INSPIRED_MESSAGE_UPDATE:
			break;
		
		default: 
			break;
	}
}


















