
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

/* sends properly formatted inspired led packet */
void inspired_packet_send( uint32_t destinationID, uint8_t* messageData, uint8_t messageLength )
{
	uint8_t length = messageLength + INSPIRED_PACKET_BYTES_MIN;
	uint8_t checksum = 0;
	uint8_t data[BLE_ADV_BYTES_MAX] = { 0 };
	
	data[0] = (INSPIRED_MANUFACTURE_ID >> 8) & 0xFF;
	data[1] = (INSPIRED_MANUFACTURE_ID >> 0) & 0xFF;
	
	//TODO: use MAC address instead?
	data[2] = (hub.ID >> 24) & 0xFF;
	data[3] = (hub.ID >> 16) & 0xFF;
	data[4] = (hub.ID >>  8) & 0xFF;
	data[5] = (hub.ID >>  0) & 0xFF;
	
	data[6] = (destinationID >> 24) & 0xFF;
	data[7] = (destinationID >> 16) & 0xFF;
	data[8] = (destinationID >>  8) & 0xFF;
	data[9] = (destinationID >>  0) & 0xFF;
	
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
	
	data[0] = (INSPIRED_MESSAGE_HELLO_NODE >> 24) & 0xFF;
	data[1] = (INSPIRED_MESSAGE_HELLO_NODE >> 16) & 0xFF;
	data[2] = (INSPIRED_MESSAGE_HELLO_NODE >>  8) & 0xFF;
	data[3] = (INSPIRED_MESSAGE_HELLO_NODE >>  0) & 0xFF;
	
	inspired_packet_send( nodeID, data, 4 );
}



/* "node, my node" message */
void inspired_packet_send_node_my_node( uint32_t nodeID )
{
	uint8_t data[4] = {0};
	
	data[0] = (INSPIRED_MESSAGE_NODE_MY_NODE >> 24) & 0xFF;
	data[1] = (INSPIRED_MESSAGE_NODE_MY_NODE >> 16) & 0xFF;
	data[2] = (INSPIRED_MESSAGE_NODE_MY_NODE >>  8) & 0xFF;
	data[3] = (INSPIRED_MESSAGE_NODE_MY_NODE >>  0) & 0xFF;
	
	inspired_packet_send( nodeID, data, 4 );
}

/* "assign switch" message */
void inspired_packet_send_assign_switch( uint32_t switchID, uint32_t switchMask, uint32_t nodeID )
{
	uint8_t data[9] = {0};
	
	data[0] = (INSPIRED_MESSAGE_ASSIGN_SWITCH >> 24) & 0xFF;
	data[1] = (INSPIRED_MESSAGE_ASSIGN_SWITCH >> 16) & 0xFF;
	data[2] = (INSPIRED_MESSAGE_ASSIGN_SWITCH >>  8) & 0xFF;
	data[3] = (INSPIRED_MESSAGE_ASSIGN_SWITCH >>  0) & 0xFF;
	
	data[4] = (switchID >> 24) & 0xFF;
	data[5] = (switchID >> 16) & 0xFF;
	data[6] = (switchID >>  8) & 0xFF;
	data[7] = (switchID >>  0) & 0xFF;
	
	data[8] = switchMask;
	
	inspired_packet_send( nodeID, data, 9 );
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
	
	if( messageID == INSPIRED_MESSAGE_HELLO_HUB && hub.mode != NODE_LEARN )
	{
		//a node is trying to register
		hub.mode = NODE_LEARN;
		hub.counter = 0;																//reset LEARN_NODE counter
		inspired_packet_send_hello_node( sourceID );		//respond to node
	}
	else if( messageID == INSPIRED_MESSAGE_HUB_MY_HUB && hub.mode == NODE_LEARN )
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
}


