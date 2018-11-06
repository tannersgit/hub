/* Node functions for handling ble stuff */

/* Header guard */
#ifndef HUB_H__
#define HUB_H__

#include <stdbool.h>
#include "radio.h"
#include "nrf52.h"

#define NODE_TYPE												0x00
#define NODES_MAX												100
#define SWITCHES_MAX										100
#define HUB_NEW_RSSI_MIN								0
#define LEARN_HUB_DURATION							500	//ms to wait for hub
#define LEARN_HUB_RSSI_MIN							(-60)
#define INSPIRED_PACKET_BYTES_MIN				11	//2 inspiredID, 4 nodeID, 4 destinationID, 1 parity

#define DESTINATION_ID_BROADCAST				0xFFFFFFFF

#define INSPIRED_MESSAGE_HELLO_HUB			0x05021988
#define INSPIRED_MESSAGE_HELLO_NODE			0x01192004
#define INSPIRED_MESSAGE_HUB_MY_HUB			0x01192017
#define INSPIRED_MESSAGE_NODE_MY_NODE		0x01192018

#define INSPIRED_MESSAGE_ASSIGN_SWITCH	0xADD1E215
#define INSPIRED_MESSAGE_REMOVE_SWITCH	0xDEADE215

#define INSPIRED_MESSAGE_GOT_IT					0x60717B55

#define HUB_PIN_LED											19

typedef enum{
	CONNECTED,		//connected to phone
	IDLE,					//waiting for something interesting to happen
	NODE_LEARN,		//in the process of learning new node
	NODE_WAIT,		//waiting for node response
	STOPPING,			//stopping advertising
	BROADCASTING,
} hub_mode_t;

typedef struct{
	char			name[25];
	uint32_t	ID;
	uint32_t	mask;
} switches_t;

typedef struct{
	uint32_t	ID;
	uint32_t	type;
	uint32_t	state;
	char			name[25];
} nodes_t;

typedef struct{
	uint32_t 		ID;
	char 				password[25];
	nodes_t 		nodes[NODES_MAX];
	switches_t 	switches[SWITCHES_MAX];
	uint32_t 		checkSum;
	uint32_t 		counter;
	hub_mode_t	mode;
	uint8_t			packet_last[BLE_ADV_BYTES_MAX];
} hub_t;

volatile extern hub_t hub;

void hub_pins_init( void );
void hub_output( bool on );

void inspired_packet_send_assign_switch( uint32_t switchID, uint32_t switchMask, uint32_t nodeID );
void inspired_packet_send_node_my_node( uint32_t nodeID );
void inspired_packet_send_node_my_node( uint32_t nodeID );


#endif // RADIO_H__

