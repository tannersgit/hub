/* radio functions for handling ble stuff */

/* Header guard */
#ifndef RADIO_H__
#define RADIO_H__

#include "nrf52.h"

#define APP_BLE_CONN_CFG_TAG      1   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO     3		/**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define BLE_ADV_BYTES_MAX					27	
#define ENOCEAN_MANUFACTURE_ID		((uint16_t)0x03DA)
#define ENOCEAN_BLUETOOTH_ID_MSB	((uint16_t)0xE215)
#define INSPIRED_MANUFACTURE_ID		((uint16_t)0x13D5)

#define PACKET_BUFFER_SIZE							10																			//Maximum number of advertising packets to store in circular buffer


#define ADVERTISING_LED                 BSP_BOARD_LED_0                         //< Is on when device is advertising. 
#define CONNECTED_LED                   BSP_BOARD_LED_1                         //< Is on when device has connected.
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         //< LED to be toggled with the help of the LED Button Service. 
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            //< Button that will trigger the notification event with the LED Button Service 

#define DEVICE_NAME                     "Dev"                      							//< Name of device. Will be included in the advertising data. 

#define APP_BLE_OBSERVER_PRIO           3                                       //< Application's BLE observer priority. You shouldn't need to modify this value. 
#define APP_BLE_CONN_CFG_TAG            1                                       //< A tag identifying the SoftDevice BLE configuration. 

#define APP_ADV_INTERVAL                0x1000                                   //< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). 
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   //< The advertising time-out (in units of seconds). When set to 0, we will never time out. 


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        //< Minimum acceptable connection interval (0.5 seconds). 
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250, UNIT_1_25_MS)        //< Maximum acceptable connection interval (1 second). 
#define SLAVE_LATENCY                   0                                       //< Slave latency. 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         //< Connection supervisory time-out (4 seconds). 

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(/*20000*/ 5000)         //< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). 
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(/*5000*/ 30000)         //< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). 
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       //< Number of attempts before giving up the connection parameter negotiation. 

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     //< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). 

#define DEAD_BEEF                       0xDEADBEEF                              //< Value used as error code on stack dump, can be used to identify stack location on stack unwind. 


#define BROADCAST_COMPANY_ID        		0xFFFF
#define BROADCAST_LENGTH_MAX        		27



typedef struct {
	uint8_t packet[PACKET_BUFFER_SIZE][BLE_ADV_BYTES_MAX];
	int8_t head;
	int8_t tail;
}packet_buffer_t;





volatile extern char radioBrodcasting;

void packet_handler_init( void );

void ble_stack_init( void );
void observe_start( void );

_Bool broadcast_packet( uint8_t* data, uint8_t length );
void packet_push_outbox( uint8_t* packet, uint8_t length );
void packet_handler( void );

void gap_params_init( void );
void gatt_init( void );
void conn_params_init( void );
void services_init( void );
void advertising_init( void );
void advertising_start( void );

void buttons_init( void );


//define externally
void enocean_switch_handler( uint32_t id, uint8_t status );
void inspired_packet_handler( uint32_t id, int8_t rssi, uint8_t* packet, uint8_t packet_length );

#endif // RADIO_H__

