#include "radio.h"
#include "hub.h"
#include "enocean.h"

void enocean_switch_handler( uint32_t id, uint8_t status )
{
	//add switch to list of known switches
	for( int i = 0; i < SWITCHES_MAX && hub.switches[i].ID != id; i++ )
	{
		if( hub.switches[i].ID == 0 )
		{
			if( status & SWITCH_MASK_A )
				hub.switches[i].mask |= SWITCH_MASK_A;
			if( status & SWITCH_MASK_B )
				hub.switches[i].mask |= SWITCH_MASK_B;
			
			hub.switches[i].ID = id;
			break;
		}
	}
	
	
	//Respond to advertising data
	if( status & 0x14 && status & 0x01 )
	{
		//output on
		hub_output( true );
		
	}
	else if( status & 0x0A && status & 0x01 )
	{
		//output off
		hub_output( false );
	}
}
