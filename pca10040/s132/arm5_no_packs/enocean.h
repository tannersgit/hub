#ifndef ENOCEAN_H__
#define ENOCEAN_H__


#define NODE_TYPE												0x00
#define HUB_NEW_RSSI_MIN								0
#define LEARN_HUB_DURATION							500	//ms to wait for hub
#define LEARN_HUB_RSSI_MIN							(-60)
#define INSPIRED_PACKET_BYTES_MIN				11	//2 inspiredID, 4 nodeID, 4 destinationID, 1 parity

#define SWITCH_MASK_B										0x18
#define SWITCH_MASK_A										0x06

#define DESTINATION_ID_BROADCAST				0xFFFFFFFF

#define INSPIRED_MESSAGE_HELLO_HUB			0x05021988
#define INSPIRED_MESSAGE_HELLO_NODE			0x01192004
#define INSPIRED_MESSAGE_HUB_MY_HUB			0x01192017
#define INSPIRED_MESSAGE_NODE_MY_NODE		0x01192018

#define INSPIRED_MESSAGE_ASSIGN_SWITCH	0xADD1E215
#define INSPIRED_MESSAGE_REMOVE_SWITCH	0xDEADE215

#define INSPIRED_MESSAGE_GOT_IT					0x60717B55

#define NODE_PIN_LED										17
#define NODE_PIN_RELAY									13



#endif	//ENOCEAN_H__




