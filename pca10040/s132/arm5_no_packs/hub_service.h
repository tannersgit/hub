/** @file
 *
 * @defgroup Hub Service Server
 *
 * @brief HUB Service Server module.
 *
 * @details This module implements a custom Hub Service with interactive characteristics.
 *          During initialization, the module adds the Hub Service and Characteristics
 *          to the BLE stack database.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_hids_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_HIDS_BLE_OBSERVER_PRIO,
 *                                   ble_hids_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef HUB_SERVICE_H__
#define HUB_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"


#define HUB_SERVICE_OBSERVER_PRIORITY		2
	
/**@brief   Macro for defining a hub service instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define HUB_SERVICE_DEF(_name)                                                		\
static hub_service_t _name;                                                   		\
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                           		\
                     HUB_SERVICE_OBSERVER_PRIORITY,                           		\
                     hub_service_on_ble_evt, &_name)


#define HUB_UUID_BASE					{	0xB9,	0x46,	0x6F,	0x43,	0xFA,	0xE2,	0x27,	0xD6, 	\
																0x65,	0x0D,	0xBE,	0xA3,	0x00,	0x00,	0x00,	0x00	}

#define HUB_UUID_SERVICE     		0x1523
#define HUB_UUID_ASSIGN_SWITCH	0x1524
#define HUB_UUID_REMOVE_SWITCH	0x1525
#define HUB_UUID_CHANGE_STATE		0x1526
#define HUB_UUID_ASSIGN_GROUP		0x1527
#define HUB_UUID_SET_TIME				0x1528
#define HUB_UUID_STATUS_UPDATE	0x1529


// Forward declaration of the hub_service_t type.
typedef struct hub_service_s hub_service_t;
																
// Typedef for the service write handlers -- provides data from the APP
typedef void (*hub_service_write_handler_t) (uint16_t conn_handle, hub_service_t* p_hub_service, uint8_t* data);
//typedef void (*ble_lbs_led_write_handler_t) (uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t new_state);

/** @brief HUB Service init structure. This structure contains all call back functions to be used by the service */
typedef struct
{
    hub_service_write_handler_t	service_handler_assign_switch; 	/**< Event handler to be called when the Assign Switch Characteristic is written. */
    hub_service_write_handler_t	service_handler_remove_switch; 	/**< Event handler to be called when the Remove Switch Characteristic is written. */
    hub_service_write_handler_t service_handler_change_state; 	/**< Event handler to be called when the Change State Characteristic is written. */
    hub_service_write_handler_t service_handler_assign_group; 	/**< Event handler to be called when the Assign Group Characteristic is written. */
    hub_service_write_handler_t service_handler_set_time; 			/**< Event handler to be called when the Set Time Characteristic is written. */
} hub_service_init_t;

/**@brief HUB Service structure. This structure contains various status information for the service. */
struct hub_service_s
{
    uint16_t                    service_handle;      						/**< Handle of LED Button Service (as provided by the BLE stack). */
    uint8_t                     uuid_type;           						/**< UUID type for the LED Button Service. */
    ble_gatts_char_handles_t    char_handles_assign_switch;    	/**< Handles related to the Assign Switch Characteristic. */
    ble_gatts_char_handles_t    char_handles_remove_switch;    	/**< Handles related to the Remove Switch Characteristic. */
    ble_gatts_char_handles_t    char_handles_change_state;    	/**< Handles related to the Change State Characteristic. */
    ble_gatts_char_handles_t    char_handles_assign_group;    	// Handles related to the Assign Group Characteristic. */
    ble_gatts_char_handles_t    char_handles_set_time;    			// Handles related to the Set Time Characteristic.
		ble_gatts_char_handles_t		char_handles_status_update;			// Handles related to the Status Update Characteristic
    hub_service_write_handler_t	service_handler_assign_switch; 	/**< Event handler to be called when the Assign Switch Characteristic is written. */
    hub_service_write_handler_t	service_handler_remove_switch; 	/**< Event handler to be called when the Remove Switch Characteristic is written. */
    hub_service_write_handler_t service_handler_change_state; 	/**< Event handler to be called when the Change State Characteristic is written. */
    hub_service_write_handler_t service_handler_assign_group; 	/**< Event handler to be called when the Assign Group Characteristic is written. */
    hub_service_write_handler_t service_handler_set_time; 			/**< Event handler to be called when the Set Time Characteristic is written. */
};


/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_hub_service  			HUB Service structure pointer. This structure must be supplied by
 *                        					the application. It is initialized by this function and will later
 *                        					be used to identify this particular service instance.
 * @param[in] p_hub_service_init  	Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t hub_service_init(hub_service_t* p_hub_service, const hub_service_init_t* p_hub_service_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the HUB Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  HUB Service structure.
 */
void hub_service_on_ble_evt(ble_evt_t const * p_ble_evt, void* p_context);


/**@brief Function for sending a Status Update notification to the app.
 *
 ' @param[in] conn_handle   	Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_hub_service		LED Button Service structure.
 * @param[in] data  					New state.
 * @param[in] len							Length of update data
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t hub_service_status_update( uint16_t conn_handle, hub_service_t * p_hub_service, uint8_t* data, uint16_t len );


#endif // HUB_SERVICE_H__

/** @} */
