/* includes that will always be required */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

/* includes for debugging/temporary */
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ttpms);


// How we keep track of state.
// Use Zephyr atomic set, clear, test functions.
ATOMIC_DEFINE(flags, 18);
#define IFL_CONNECTED_FLAG		0
#define IFR_CONNECTED_FLAG		1
#define IRL_CONNECTED_FLAG		2
#define IRR_CONNECTED_FLAG		3
#define EFL_CONNECTED_FLAG		4
#define EFR_CONNECTED_FLAG		5
#define ERL_CONNECTED_FLAG		6
#define ERR_CONNECTED_FLAG		7
#define TEMP_ENABLED_FLAG		8
#define PRESSURE_ENABLED_FLAG	9
#define IFL_SUBSCRIBED_FLAG		10
#define IFR_SUBSCRIBED_FLAG		11
#define IRL_SUBSCRIBED_FLAG		12
#define IRR_SUBSCRIBED_FLAG		13
#define EFL_SUBSCRIBED_FLAG		14
#define EFR_SUBSCRIBED_FLAG		15
#define ERL_SUBSCRIBED_FLAG		16
#define ERR_SUBSCRIBED_FLAG		17


#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)

const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});
static struct gpio_callback button_cb_data;


// Standard 11-bit CAN IDs can be up to 2047 (0x7FF). Lower = higher priority
#define TTPMS_CAN_BASE_ID 0x400

/* START CAN FRAMES*/
// Ensure this is always up to date with CAN .dbc file to be kept in version control by the team

// This frame is sent by dash or other controller to enable & configure TTPMS
#define TTPMS_SETTINGS_FRAME_ID TTPMS_CAN_BASE_ID	// not creating a can_frame struct because we will be receiving this

// This frame is sent out by TTPMS RX to indicate general data
struct can_frame TTPMS_status = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 1, .dlc = 8};

// All temp values are uint8_t with 0.5 scale and 0 offset
// Each CAN frame can only hold 8 data bytes. Thus multiple frames are needed for each full sensor reading

// Internal front left (16 temp pixels + 24-bit pressure)
struct can_frame IFL_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 2, .dlc = 8};
struct can_frame IFL_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 3, .dlc = 8};
struct can_frame FL_pressure = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 4, .dlc = 3};
 
// Internal front right (16 temp pixels + 24-bit pressure)
struct can_frame IFR_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 5, .dlc = 8};
struct can_frame IFR_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 6, .dlc = 8};
struct can_frame FR_pressure = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 7, .dlc = 3};

// Internal rear left (16 temp pixels + 24-bit pressure)
struct can_frame IRL_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 8, .dlc = 8};
struct can_frame IRL_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 9, .dlc = 8};
struct can_frame RL_pressure = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 10, .dlc = 3};

// Internal rear right (16 temp pixels + 24-bit pressure)
struct can_frame IRR_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 11, .dlc = 8};
struct can_frame IRR_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 12, .dlc = 8};
struct can_frame RR_pressure = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 13, .dlc = 3};

// External front left (32 temp pixels)
struct can_frame EFL_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 14, .dlc = 8};
struct can_frame EFL_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 15, .dlc = 8};
struct can_frame EFL_temp_3 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 16, .dlc = 8};
struct can_frame EFL_temp_4 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 17, .dlc = 8};

// External front right (32 temp pixels)
struct can_frame EFR_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 18, .dlc = 8};
struct can_frame EFR_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 19, .dlc = 8};
struct can_frame EFR_temp_3 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 20, .dlc = 8};
struct can_frame EFR_temp_4 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 21, .dlc = 8};

// External rear left (16 temp pixels)
struct can_frame ERL_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 22, .dlc = 8};
struct can_frame ERL_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 23, .dlc = 8};

// External rear right (16 temp pixels)
struct can_frame ERR_temp_1 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 24, .dlc = 8};
struct can_frame ERR_temp_2 = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 25, .dlc = 8};

/* END CAN FRAMES*/


// First hex char must be C for self-assigning address/identity
#define TTPMS_RX_BT_ID "CA:69:F1:F1:69:69"		// for this device
#define TTPMS_IFL_BT_ID "CA:69:F1:F1:33:42"		// Internal Front Left BT ID
#define TTPMS_IFR_BT_ID "CA:69:F1:F1:33:43"		// Internal Front Right BT ID
#define TTPMS_IRL_BT_ID "CA:69:F1:F1:33:44"		// Internal Rear Left BT ID
#define TTPMS_IRR_BT_ID "CA:69:F1:F1:33:45"		// Internal Rear Right BT ID
#define TTPMS_EFL_BT_ID "CA:69:F1:F1:11:22"		// External Front Left BT ID
#define TTPMS_EFR_BT_ID "CA:69:F1:F1:11:23"		// External Front Right BT ID
#define TTPMS_ERL_BT_ID "CA:69:F1:F1:11:24"		// External Rear Left BT ID
#define TTPMS_ERR_BT_ID "CA:69:F1:F1:11:25"		// External Rear Right BT ID

#define TTPMS_TEST_BT_ID "F6:B3:F2:9C:20:20"	// from nrf dongle (for testing)

#define INIT_INTERVAL	16	/* 10 ms */
#define INIT_WINDOW		16	/* 10 ms */
#define CONN_INTERVAL	8	/* 10 ms */
#define CONN_LATENCY	0
#define CONN_TIMEOUT	MIN(MAX((CONN_INTERVAL * 125 * \
			       		MAX(CONFIG_BT_MAX_CONN, 6) / 1000), 10), 3200)

static bt_addr_le_t IFL_bt_addr;
static bt_addr_le_t IFR_bt_addr;
static bt_addr_le_t IRL_bt_addr;
static bt_addr_le_t IRR_bt_addr;
static bt_addr_le_t EFL_bt_addr;
static bt_addr_le_t EFR_bt_addr;
static bt_addr_le_t ERL_bt_addr;
static bt_addr_le_t ERR_bt_addr;

static bt_addr_le_t TEST_bt_addr;

struct bt_le_conn_param conn_param = {
		.interval_min = CONN_INTERVAL,
		.interval_max = CONN_INTERVAL,
		.latency = CONN_LATENCY,
		.timeout = CONN_TIMEOUT,
	};


/* --- CAN WORK FUNCTIONS START --- */

void IFL_CAN_tx_work_handler(struct k_work *work)
{
	IFL_temp_1.data[0] = 0x69;
	IFL_temp_1.data[1] = 0x42;
	IFL_temp_1.data[2] = 0x11;
	IFL_temp_1.data[3] = 0x22;
	IFL_temp_1.data[4] = 0x33;
	IFL_temp_1.data[5] = 0x44;
	IFL_temp_1.data[6] = 0x55;
	IFL_temp_1.data[7] = 0xAF;
	can_send(can_dev, &IFL_temp_1, K_FOREVER, NULL, NULL);

	IFL_temp_2.data[0] = 0x69;
	IFL_temp_2.data[1] = 0x42;
	IFL_temp_2.data[2] = 0x11;
	IFL_temp_2.data[3] = 0x22;
	IFL_temp_2.data[4] = 0x33;
	IFL_temp_2.data[5] = 0x44;
	IFL_temp_2.data[6] = 0x55;
	IFL_temp_2.data[7] = 0xAF;
	can_send(can_dev, &IFL_temp_2, K_FOREVER, NULL, NULL);
}

K_WORK_DEFINE(IFL_CAN_tx_work, IFL_CAN_tx_work_handler);

/* --- CAN WORK FUNCTIONS END --- */


/* --- BLE CALLBACK FUNCTIONS START --- */

static void connected(struct bt_conn *conn, uint8_t err)	// here we set the connected bit for the sensor that connected (self-explanatory)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	const bt_addr_le_t *addr = bt_conn_get_dst(conn);

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if (err) {
		LOG_WRN("Failed to connect to %s (%u)", addr_str, err);
	} else {
		if (bt_addr_le_eq(addr, &IFL_bt_addr)) {

			atomic_set_bit(flags, IFL_CONNECTED_FLAG);
			LOG_INF("Internal FL connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &IFR_bt_addr)) {

			atomic_set_bit(flags, IFR_CONNECTED_FLAG);
			LOG_INF("Internal FR connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &IRL_bt_addr)) {

			atomic_set_bit(flags, IRL_CONNECTED_FLAG);
			LOG_INF("Internal RL connected, addr: %s", addr_str);
			
		} else if (bt_addr_le_eq(addr, &IRR_bt_addr)) {

			atomic_set_bit(flags, IRR_CONNECTED_FLAG);
			LOG_INF("Internal RR connected, addr: %s", addr_str);
			
		} else if (bt_addr_le_eq(addr, &EFL_bt_addr)) {

			atomic_set_bit(flags, EFL_CONNECTED_FLAG);
			LOG_INF("External FL connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &EFR_bt_addr)) {

			atomic_set_bit(flags, EFR_CONNECTED_FLAG);
			LOG_INF("External FR connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &ERL_bt_addr)) {

			atomic_set_bit(flags, ERL_CONNECTED_FLAG);
			LOG_INF("External RL connected, addr: %s", addr_str);
			
		} else if (bt_addr_le_eq(addr, &ERR_bt_addr)) {

			atomic_set_bit(flags, ERR_CONNECTED_FLAG);
			LOG_INF("External RR connected, addr: %s", addr_str);
			
		} else {
			LOG_WRN("UNRECOGNIZED SENSOR CONNECTED, addr: %s", addr_str);
		}
	}

	err = bt_conn_le_create_auto(BT_CONN_LE_CREATE_CONN, &conn_param);
	if (err) {
		LOG_ERR("Failed to start automatically connecting (err %d)", err);
	}

}

static void disconnected(struct bt_conn *conn, uint8_t reason)	// here we clear the connected bit for the sensor that disconnected (self-explanatory), and we also clear the subscribe bit so that the logic in main will know to subscribe again if re-connected
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	const bt_addr_le_t *addr = bt_conn_get_dst(conn);

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if (bt_addr_le_eq(addr, &IFL_bt_addr)) {

		atomic_clear_bit(flags, IFL_CONNECTED_FLAG);
		atomic_clear_bit(flags, IFL_SUBSCRIBED_FLAG);
		LOG_INF("Internal FL disconnected, addr: %s (reason 0x%02x)", addr_str, reason);

	} else if (bt_addr_le_eq(addr, &IFR_bt_addr)) {

		atomic_clear_bit(flags, IFR_CONNECTED_FLAG);
		atomic_clear_bit(flags, IFR_SUBSCRIBED_FLAG);
		LOG_INF("Internal FR disconnected, addr: %s (reason 0x%02x)", addr_str, reason);

	} else if (bt_addr_le_eq(addr, &IRL_bt_addr)) {

		atomic_clear_bit(flags, IRL_CONNECTED_FLAG);
		atomic_clear_bit(flags, IRL_SUBSCRIBED_FLAG);
		LOG_INF("Internal RL disconnected, addr: %s (reason 0x%02x)", addr_str, reason);
		
	} else if (bt_addr_le_eq(addr, &IRR_bt_addr)) {

		atomic_clear_bit(flags, IRR_CONNECTED_FLAG);
		atomic_clear_bit(flags, IRR_SUBSCRIBED_FLAG);
		LOG_INF("Internal RR disconnected, addr: %s (reason 0x%02x)", addr_str, reason);
		
	} else if (bt_addr_le_eq(addr, &EFL_bt_addr)) {

		atomic_clear_bit(flags, EFL_CONNECTED_FLAG);
		atomic_clear_bit(flags, EFL_SUBSCRIBED_FLAG);
		LOG_INF("External FL disconnected, addr: %s (reason 0x%02x)", addr_str, reason);

	} else if (bt_addr_le_eq(addr, &EFR_bt_addr)) {

		atomic_clear_bit(flags, EFR_CONNECTED_FLAG);
		atomic_clear_bit(flags, EFR_SUBSCRIBED_FLAG);
		LOG_INF("External FR disconnected, addr: %s (reason 0x%02x)", addr_str, reason);

	} else if (bt_addr_le_eq(addr, &ERL_bt_addr)) {

		atomic_clear_bit(flags, ERL_CONNECTED_FLAG);
		atomic_clear_bit(flags, ERL_SUBSCRIBED_FLAG);
		LOG_INF("External RL disconnected, addr: %s (reason 0x%02x)", addr_str, reason);
		
	} else if (bt_addr_le_eq(addr, &ERR_bt_addr)) {

		atomic_clear_bit(flags, ERR_CONNECTED_FLAG);
		atomic_clear_bit(flags, ERR_SUBSCRIBED_FLAG);
		LOG_INF("External RR disconnected, addr: %s (reason 0x%02x)", addr_str, reason);
		
	} else {
		LOG_WRN("UNRECOGNIZED SENSOR DISCONNECTED, addr: %s (reason 0x%02x)", addr_str, reason);
	}

}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

uint8_t IFL_temp_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
	// should we update a global temp value variable with the data, then do the CAN_tx work?
	// or should we pass the data to the CAN_tx work?
	// do we want global temp value variables?

	if (data == NULL){	// When successfully unsubscribed, notify callback is called one last time with data set to NULL (from Zephyr docs)
		LOG_INF("Successfully unsubscribed from IFL temp");
		return BT_GATT_ITER_STOP; // might as well return this (it's zero), shouldn't matter
	}

	if (!atomic_test_bit(flags, TEMP_ENABLED_FLAG)) {	// if temp is not enabled, we need to unsubscribe
		LOG_INF("Attempting to unsubscribe from IFL temp");
		return BT_GATT_ITER_STOP;	// returning this tells the BT Host to unsubscribe us
	}

	// do the actual normal notification stuff here

	return BT_GATT_ITER_CONTINUE;

}

static struct bt_gatt_subscribe_params IFL_subscribe_params = {
		.notify = IFL_temp_notify_cb,
};

// NOTE: each sensor needs to have its own subscribe_params variable since it remains tied to each subscription (from Zephyr docs)

/* --- BLE CALLBACK FUNCTIONS END --- */


void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if(!atomic_test_bit(flags, TEMP_ENABLED_FLAG)) {
		atomic_set_bit(flags, TEMP_ENABLED_FLAG);
		LOG_INF("Button pressed, ENABLED temperature");
	} else {
		atomic_clear_bit(flags, TEMP_ENABLED_FLAG);
		LOG_INF("Button pressed, DISABLED temperature");
	}
}


void main(void)
{
	LOG_INF("Running ttpms_v2_receiver");

	int err;
	/*
	if (!device_is_ready(can_dev)) {
		LOG_WRN("CAN device not ready");
		return;
	}

	err = can_start(can_dev);
	if (err != 0) {
		LOG_WRN("Error starting CAN controller (err %d)", err);
		return;
	}

	*/

	// button for testing purposes
	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Error: button device %s is not ready", button.port->name);
		return;
	}
	err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d", err, button.port->name, button.pin);
		return;
	}
	err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", err, button.port->name, button.pin);
		return;
	}
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	LOG_ERR("Set up button at %s pin %d", button.port->name, button.pin);


	bt_addr_le_t addr;

	// create self BT address
	err = bt_addr_le_from_str(TTPMS_RX_BT_ID, "random", &addr);
	if (err) {
		LOG_WRN("Invalid BT address (err %d)", err);
	}

	// assign self BT address
	int bt_identity;
	bt_identity = bt_id_create(&addr, NULL);
	if (bt_identity < 0) {
		LOG_WRN("Creating new BT ID failed (err %d)", err);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_WRN("Bluetooth init failed (err %d)", err);
	} else {
		LOG_INF("Bluetooth initialized");
	}

	// fill address variables for the devices we want to filter for
	err = bt_addr_le_from_str(TTPMS_IFL_BT_ID, "random", &IFL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_IFR_BT_ID, "random", &IFR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_IRL_BT_ID, "random", &IRL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_IRR_BT_ID, "random", &IRR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_EFL_BT_ID, "random", &EFL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_EFR_BT_ID, "random", &EFR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_ERL_BT_ID, "random", &ERL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_ERR_BT_ID, "random", &ERR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	err = bt_addr_le_from_str(TTPMS_TEST_BT_ID, "random", &TEST_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)", err); }

	// Add address of the devices we want to filter accept list
	err = bt_le_filter_accept_list_add(&IFL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_le_filter_accept_list_add(&IFR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_le_filter_accept_list_add(&IRL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_le_filter_accept_list_add(&IRR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_le_filter_accept_list_add(&EFL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_le_filter_accept_list_add(&EFR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_le_filter_accept_list_add(&ERL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	//err = bt_le_filter_accept_list_add(&ERR_bt_addr);
	//if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_le_filter_accept_list_add(&TEST_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	err = bt_conn_le_create_auto(BT_CONN_LE_CREATE_CONN, &conn_param);
	if (err) {
		LOG_ERR("Failed to start automatically connecting (err %d)", err);
	}

	while(1)
	{
		
		if (atomic_test_bit(flags, TEMP_ENABLED_FLAG))	{ // if temp is enabled, make sure we are subscribed to all connected sensors

			if (atomic_test_bit(flags, IFL_CONNECTED_FLAG) && !atomic_test_bit(flags, IFL_SUBSCRIBED_FLAG)) // if connected and not subscribed, we need to subscribe
			{
				/*
				atomic_set_bit(&flags, IFL_SUBSCRIBED_FLAG); // we want to be sure this is set before the first notify callback, which will unsubscribe itself if this is not set
				// set da params? (IFL_subscribe_params) prob need to to proper discover in order to get att handle :(
				err = bt_gatt_subscribe(bt_conn_lookup_addr_le(bt_identity, &IFL_bt_addr), &IFL_subscribe_params);
				if (err) {
					LOG_WRN("Failed to subscribe to IFL temp (err %d)", err);
					atomic_clear_bit(&flags, IFL_SUBSCRIBED_FLAG); // clear the flag we just set (see above) if we did not actually successfully subscribe
				}*/

				LOG_INF("Attempting to discover");
				// do discover here
				atomic_set_bit(flags, IFL_SUBSCRIBED_FLAG); // temporary so we don't keep discovering (which is also temporarily here)
			}

		} 
		// NOTE: the notify callbacks will unsubscribe themselves if they see that temp is not enabled
	}
}
