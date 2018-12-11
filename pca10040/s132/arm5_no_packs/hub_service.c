#include "sdk_common.h"
#include "hub_service.h"
#include "ble_srv_common.h"


/**@brief Function for handling a Write event.
 *
 * @param[in] p_hub_service 	HUB Service structure.
 * @param[in] p_ble_evt  			Event received from the BLE stack.
 */
static void on_write(hub_service_t * p_hub_service, ble_evt_t const * p_ble_evt)
{
	ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;


	//TODO: change from switch statement to series of IFs -- switch comparison values cannot be variable
	
	//Assign Switch write event
	if( (p_evt_write->handle == p_hub_service->char_handles_assign_switch.value_handle) &&
			(p_evt_write->len == 9) &&
			(p_hub_service->service_handler_assign_switch != NULL) )
	{
		p_hub_service->service_handler_assign_switch(p_ble_evt->evt.gap_evt.conn_handle, p_hub_service, (uint8_t*)p_evt_write->data);
	}

	//Remove Switch write event
	else if(	(p_evt_write->handle == p_hub_service->char_handles_remove_switch.value_handle) &&
						(p_evt_write->len == 9) &&
						(p_hub_service->service_handler_remove_switch != NULL) )
	{
		p_hub_service->service_handler_remove_switch(p_ble_evt->evt.gap_evt.conn_handle, p_hub_service, (uint8_t*)p_evt_write->data);
	}

	//Change State write event
	else if(	(p_evt_write->handle == p_hub_service->char_handles_change_state.value_handle) &&
						(p_evt_write->len == 14) &&
						(p_hub_service->service_handler_change_state != NULL) )
	{
		p_hub_service->service_handler_change_state(p_ble_evt->evt.gap_evt.conn_handle, p_hub_service, (uint8_t*)p_evt_write->data);
	}

	//Assign Group write event
	else if(	(p_evt_write->handle == p_hub_service->char_handles_assign_group.value_handle) &&
						(p_evt_write->len == 8) &&
						(p_hub_service->service_handler_assign_group != NULL) )
	{
		p_hub_service->service_handler_assign_group(p_ble_evt->evt.gap_evt.conn_handle, p_hub_service, (uint8_t*)p_evt_write->data);
	}

	//Set Time write event
	else if(	(p_evt_write->handle == p_hub_service->char_handles_set_time.value_handle) &&
						(p_evt_write->len == 4) &&
						(p_hub_service->service_handler_set_time != NULL) )
	{
		p_hub_service->service_handler_set_time(p_ble_evt->evt.gap_evt.conn_handle, p_hub_service, (uint8_t*)p_evt_write->data);
	}
}


void hub_service_on_ble_evt(ble_evt_t const * p_ble_evt, void* p_context)
{
    hub_service_t * p_hub_service = (hub_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_hub_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the a writable Characteristic.
 *
 * @param[in] p_hub_service				HUB Service structure.
 * @param[in] p_hub_service_init	HUB Service initialization structure.
 * @param[in] char_uuid						Characteristic UUID
 * @param[in] len									Number of bytes of data communicated via Characteristic
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t add_writable_characteristic( hub_service_t * p_hub_service, ble_gatts_char_handles_t * p_char_handle, uint16_t char_uuid, uint16_t len )
{
    ble_gatts_char_md_t char_md;													// GATT Characteristic metadata structure
    ble_gatts_attr_t    attr_char_value;									// GATT Attribute structure
    ble_uuid_t          ble_uuid;													// Bluetooth Low Energy UUID type, encapsulates both 16-bit and 128-bit UUIDs.
    ble_gatts_attr_md_t attr_md;													// Attribute metadata structure
	
    memset(&char_md, 0, sizeof(char_md));
		memset(&attr_md, 0, sizeof(attr_md));
		memset(&attr_char_value, 0, sizeof(attr_char_value));

    char_md.char_props.read  = 1;													// Writing the value with Write Request permitted
    char_md.char_props.write = 1;													// Reading the value permitted	
    char_md.p_char_user_desc = NULL;											// Pointer to a UTF-8 encoded string (non-NULL terminated), NULL if the descriptor is not required.
    char_md.p_char_pf        = NULL;											// Pointer to a presentation format structure or NULL if the CPF descriptor is not required.
    char_md.p_user_desc_md   = NULL;											// Attribute metadata for the User Description descriptor, or NULL for default values.
    char_md.p_cccd_md        = NULL;											// Attribute metadata for the Client Characteristic Configuration Descriptor, or NULL for default values.
    char_md.p_sccd_md        = NULL;											// Attribute metadata for the Server Characteristic Configuration Descriptor, or NULL for default values.

		ble_uuid.type = p_hub_service->uuid_type;							// UUID type -- BLE_UUID_TYPE_UNKNOWN: invalid UUID; BLE_UUID_TYPE_BLE: Bluetooth SIG UUID (16-bit); BLE_UUID_TYPE_VENDOR_BEGIN: Vendor UUID types start at this index (128-bit).
    ble_uuid.uuid = char_uuid;														// 16-bit UUID value or octets 12-13 of 128-bit UUID.
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);		// Set sec_mode, pointed to by ptr, to require no protection -- open link.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);	// Set sec_mode, pointed to by ptr, to require no protection -- open link.
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;								// Value location, see @ref BLE_GATTS_VLOCS.
    attr_md.rd_auth = 0;																	// Read authorization and value will be requested from the application on every read operation.
    attr_md.wr_auth = 0;																	// Write authorization will be requested from the application on every Write Request operation (but not Write Command).
    attr_md.vlen    = 0;																	// Variable length attribute.
		
    attr_char_value.p_uuid    = &ble_uuid;								// Pointer to the attribute UUID.
    attr_char_value.p_attr_md = &attr_md;									// Pointer to the attribute metadata structure.
    attr_char_value.init_len  = len * sizeof(uint8_t);		// Initial attribute value length in bytes.
    attr_char_value.init_offs = 0;												// Initial attribute value offset in bytes. If different from zero, the first init_offs bytes of the attribute value will be left uninitialized.
    attr_char_value.max_len   = len * sizeof(uint8_t);		// Maximum attribute value length in bytes, see @ref BLE_GATTS_ATTR_LENS_MAX for maximum values (510 or 512).
    attr_char_value.p_value   = NULL;											// Pointer to the attribute data. Please note that if the @ref BLE_GATTS_VLOC_USER value location is selected in the attribute metadata, this will have to point to a buffer that remains valid through the lifetime of the attribute. This excludes usage of automatic variables that may go out of scope or any other temporary location. The stack may access that memory directly without the application's knowledge. For writable characteristics, this value must not be a location in flash memory.

    return sd_ble_gatts_characteristic_add(p_hub_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           p_char_handle);
}


/**@brief Function for adding a notification Characteristic.
 *
 * @param[in] p_hub_service      	Hub Service structure.
 * @param[in] p_hub_service_init 	Hub Service initialization structure.
 * @param[in] char_uuid						Characteristic UUID
 * @param[in] len									Number of bytes of data communicated via Characteristic
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t add_notification_characteristic( hub_service_t * p_hub_service, ble_gatts_char_handles_t * p_char_handle, uint16_t char_uuid, uint16_t len )
{
    ble_gatts_char_md_t char_md;															// GATT Characteristic metadata structure
    ble_gatts_attr_md_t cccd_md;															// Attribute metadata structure for client characteristic configuration descriptor
    ble_gatts_attr_t    attr_char_value;											// GATT Attribute structure
    ble_uuid_t          ble_uuid;															// Bluetooth Low Energy UUID type, encapsulates both 16-bit and 128-bit UUIDs.
    ble_gatts_attr_md_t attr_md;															// Attribute metadata structure

    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&char_md, 0, sizeof(char_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));	

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);				// Set sec_mode, pointed to by ptr, to require no protection -- open link.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);			// Set sec_mode, pointed to by ptr, to require no protection -- open link.
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;											// Value location, see @ref BLE_GATTS_VLOCS.

    char_md.char_props.read   = 1;														// Reading the value permitted
    char_md.char_props.notify = 1;														// Notification of the value permitted
    char_md.p_char_user_desc  = NULL;													// Pointer to a UTF-8 encoded string (non-NULL terminated), NULL if the descriptor is not required.
    char_md.p_char_pf         = NULL;													// Pointer to a presentation format structure or NULL if the CPF descriptor is not required.
    char_md.p_user_desc_md    = NULL;													// Attribute metadata for the User Description descriptor, or NULL for default values.
    char_md.p_cccd_md         = &cccd_md;											// Attribute metadata for the Client Characteristic Configuration Descriptor, or NULL for default values.
    char_md.p_sccd_md         = NULL;													// Attribute metadata for the Server Characteristic Configuration Descriptor, or NULL for default values.

    ble_uuid.type = p_hub_service->uuid_type;									// UUID type -- BLE_UUID_TYPE_UNKNOWN: invalid UUID; BLE_UUID_TYPE_BLE: Bluetooth SIG UUID (16-bit); BLE_UUID_TYPE_VENDOR_BEGIN: Vendor UUID types start at this index (128-bit).
    ble_uuid.uuid = char_uuid;																// 16-bit UUID value or octets 12-13 of 128-bit UUID.

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);				// Set sec_mode, pointed to by ptr, to require no protection -- open link.
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);	// Set sec_mode pointed to by ptr to have no access rights.
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;										// Value location, see @ref BLE_GATTS_VLOCS.
    attr_md.rd_auth = 0;																			// Read authorization and value will be requested from the application on every read operation.
    attr_md.wr_auth = 0;																			// Write authorization will be requested from the application on every Write Request operation (but not Write Command).
    attr_md.vlen    = 0;																			// Variable length attribute.

    attr_char_value.p_uuid    = &ble_uuid;										// Pointer to the attribute UUID.
    attr_char_value.p_attr_md = &attr_md;											// Pointer to the attribute metadata structure.
    attr_char_value.init_len  = len * sizeof(uint8_t);				// Initial attribute value length in bytes.
    attr_char_value.init_offs = 0;														// Initial attribute value offset in bytes. If different from zero, the first init_offs bytes of the attribute value will be left uninitialized.
    attr_char_value.max_len   = len * sizeof(uint8_t);				// Maximum attribute value length in bytes, see @ref BLE_GATTS_ATTR_LENS_MAX for maximum values (510 or 512).
    attr_char_value.p_value   = NULL;													// Pointer to the attribute data. Please note that if the @ref BLE_GATTS_VLOC_USER value location is selected in the attribute metadata, this will have to point to a buffer that remains valid through the lifetime of the attribute. This excludes usage of automatic variables that may go out of scope or any other temporary location. The stack may access that memory directly without the application's knowledge. For writable characteristics, this value must not be a location in flash memory.

    return sd_ble_gatts_characteristic_add(p_hub_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           p_char_handle);
}


uint32_t hub_service_init( hub_service_t * p_hub_service, const hub_service_init_t * p_hub_service_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_hub_service->service_handler_assign_switch 	= p_hub_service_init->service_handler_assign_switch;
    p_hub_service->service_handler_remove_switch 	= p_hub_service_init->service_handler_remove_switch;
    p_hub_service->service_handler_change_state		= p_hub_service_init->service_handler_change_state;
    p_hub_service->service_handler_assign_group 	= p_hub_service_init->service_handler_assign_group;
    p_hub_service->service_handler_set_time 			= p_hub_service_init->service_handler_set_time;

    // Add service.
    ble_uuid128_t base_uuid = {HUB_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_hub_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_hub_service->uuid_type;
    ble_uuid.uuid = HUB_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_hub_service->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = add_writable_characteristic( p_hub_service, &p_hub_service->char_handles_assign_switch, HUB_UUID_ASSIGN_SWITCH, 9 );
    VERIFY_SUCCESS(err_code);
		
    err_code = add_writable_characteristic( p_hub_service, &p_hub_service->char_handles_remove_switch, HUB_UUID_REMOVE_SWITCH, 9 );
    VERIFY_SUCCESS(err_code);
		
    err_code = add_writable_characteristic( p_hub_service, &p_hub_service->char_handles_change_state, HUB_UUID_CHANGE_STATE, 14 );
    VERIFY_SUCCESS(err_code);
		
    err_code = add_writable_characteristic( p_hub_service, &p_hub_service->char_handles_assign_group, HUB_UUID_ASSIGN_GROUP, 8 );
    VERIFY_SUCCESS(err_code);
		
    err_code = add_writable_characteristic( p_hub_service, &p_hub_service->char_handles_set_time, HUB_UUID_SET_TIME, 4 );
    VERIFY_SUCCESS(err_code);

    err_code = add_notification_characteristic( p_hub_service, &p_hub_service->char_handles_status_update, HUB_UUID_STATUS_UPDATE, 8 );
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t hub_service_status_update( uint16_t conn_handle, hub_service_t * p_hub_service, uint8_t* data, uint16_t len )
{
    ble_gatts_hvx_params_t 	params;

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_hub_service->char_handles_status_update.value_handle;
    params.p_data = data;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}

