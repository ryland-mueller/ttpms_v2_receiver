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

#include "ttpms_common.h"

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



/* --- CAN BUS START --- */

// Standard 11-bit CAN IDs can be up to 2047 (0x7FF). Lower = higher priority
#define TTPMS_CAN_BASE_ID 0x300

// This frame is sent by dash or other controller to enable & configure TTPMS
// 0th byte:	0th bit = temp enable	1th bit = pressure enable
#define TTPMS_SETTINGS_FRAME_ID TTPMS_CAN_BASE_ID	// not creating a can_frame struct because we will only be receiving this

// This frame is sent out by TTPMS RX to indicate general data
// 0th byte:	0th bit = temp enabled	1th bit = pressure enabled
// 1th byte:	connected sensors (0th bit = IFL .... 7th bit = ERR)
struct can_frame TTPMS_status = {.flags = 0, .id = TTPMS_CAN_BASE_ID + 1, .dlc = 2};

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

const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

const struct can_filter settings_frame_filter = {
        .flags = CAN_FILTER_DATA,
        .id = TTPMS_SETTINGS_FRAME_ID,
        .mask = CAN_STD_ID_MASK
};

void settings_frame_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{

    if (frame->data[0] & 0x01) {
		if(!atomic_test_and_set_bit(flags, TEMP_ENABLED_FLAG)) {
			LOG_INF("Temperature ENABLED via CAN");
		}
	} else {
		if(atomic_test_and_clear_bit(flags, TEMP_ENABLED_FLAG)) {
			LOG_INF("Temperature DISABLED via CAN");
		}
	}

	if (frame->data[0] & 0x02) {
		if(!atomic_test_and_set_bit(flags, PRESSURE_ENABLED_FLAG)) {
			LOG_INF("Pressure ENABLED via CAN");
		}
	} else {
		if(atomic_test_and_clear_bit(flags, PRESSURE_ENABLED_FLAG)) {
			LOG_INF("Pressure DISABLED via CAN");
		}
	}
}

// The CAN frame structs are filled with data from BLE notification callback.
// The filled frames are then sent here in system workqueue since can_send is blocking.
// NOTE: This system should be changed, as the CAN frame data is not linked to each CAN send work event,
// so if CAN messages become delayed more than ~120ms, the next BLE notification will overwrite the CAN frame data before the frame was sent with that data,
// resulting in data loss and duplicate frames being sent.

void status_CAN_tx_work_handler(struct k_work *work)
{
	//LOG_INF("status_CAN_tx_work_handler: Sending TTPMS status frame");
	can_send(can_dev, &TTPMS_status, K_FOREVER, NULL, NULL);
}
K_WORK_DEFINE(status_CAN_tx_work, status_CAN_tx_work_handler);

void IFL_temp_CAN_tx_work_handler(struct k_work *work)
{
	//LOG_INF("IFL_temp_CAN_tx_work_handler: Sending IFL temp frames");
	can_send(can_dev, &IFL_temp_1, K_FOREVER, NULL, NULL);
	can_send(can_dev, &IFL_temp_2, K_FOREVER, NULL, NULL);
}
K_WORK_DEFINE(IFL_temp_CAN_tx_work, IFL_temp_CAN_tx_work_handler);

void EFL_temp_CAN_tx_work_handler(struct k_work *work)
{
	//LOG_INF("EFL_temp_CAN_tx_work_handler: Sending EFL temp frames");
	can_send(can_dev, &EFL_temp_1, K_FOREVER, NULL, NULL);
	can_send(can_dev, &EFL_temp_2, K_FOREVER, NULL, NULL);
	can_send(can_dev, &EFL_temp_3, K_FOREVER, NULL, NULL);
	can_send(can_dev, &EFL_temp_4, K_FOREVER, NULL, NULL);
}
K_WORK_DEFINE(EFL_temp_CAN_tx_work, EFL_temp_CAN_tx_work_handler);

void EFR_temp_CAN_tx_work_handler(struct k_work *work)
{
	//LOG_INF("EFR_temp_CAN_tx_work_handler: Sending EFR temp frames");
	can_send(can_dev, &EFR_temp_1, K_FOREVER, NULL, NULL);
	can_send(can_dev, &EFR_temp_2, K_FOREVER, NULL, NULL);
	can_send(can_dev, &EFR_temp_3, K_FOREVER, NULL, NULL);
	can_send(can_dev, &EFR_temp_4, K_FOREVER, NULL, NULL);
}
K_WORK_DEFINE(EFR_temp_CAN_tx_work, EFR_temp_CAN_tx_work_handler);

void TTPMS_CAN_init(void)
{
	int err;

	if (!device_is_ready(can_dev)) {
		LOG_WRN("CAN device not ready");
		return;
	}

	err = can_start(can_dev);
	if (err != 0) {
		LOG_WRN("Error starting CAN controller (err %d)", err);
		return;
	}

	err = can_add_rx_filter(can_dev, settings_frame_cb, NULL, &settings_frame_filter);
	if (err < 0) {
		LOG_ERR("Unable to add CAN RX filter (err %d)", err);
	}
}

/* --- CAN BUS END --- */



/* --- BLE STUFF START --- */

#define CONN_INTERVAL	24	// * 1.25 = 30 ms	try changing back to 10ms for the lols?
#define CONN_LATENCY	0
#define CONN_TIMEOUT	MIN(MAX((CONN_INTERVAL * 125 * \
			       		MAX(CONFIG_BT_MAX_CONN, 6) / 1000), 10), 3200)

//#define SCAN_INTERVAL	400	// * 0.625 = 250 ms
//#define SCAN_WINDOW		64	// * 0.625 = 40 ms

struct bt_le_conn_param conn_param = {
		.interval_min = CONN_INTERVAL,
		.interval_max = CONN_INTERVAL,
		.latency = CONN_LATENCY,
		.timeout = CONN_TIMEOUT,
	};

struct bt_conn_le_create_param scan_param = {
		.options = BT_CONN_LE_OPT_NONE,
		//.interval = SCAN_INTERVAL,
		//.window = SCAN_WINDOW,
	};

static bt_addr_le_t IFL_bt_addr;
static bt_addr_le_t IFR_bt_addr;
static bt_addr_le_t IRL_bt_addr;
static bt_addr_le_t IRR_bt_addr;
static bt_addr_le_t EFL_bt_addr;
static bt_addr_le_t EFR_bt_addr;
static bt_addr_le_t ERL_bt_addr;
static bt_addr_le_t ERR_bt_addr;

int bt_identity;	// self BT address/identity

// here we set the connected bit for the sensor that connected (self-explanatory)
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	const bt_addr_le_t *addr = bt_conn_get_dst(conn);

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if (err) {
		LOG_WRN("Failed to connect to %s (%u)", addr_str, err);
	} else {

		if (bt_addr_le_eq(addr, &IFL_bt_addr)) {

			if(atomic_test_and_set_bit(flags, IFL_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("Internal FL connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &IFR_bt_addr)) {

			if(atomic_test_and_set_bit(flags, IFR_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("Internal FR connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &IRL_bt_addr)) {

			if(atomic_test_and_set_bit(flags, IRL_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("Internal RL connected, addr: %s", addr_str);
			
		} else if (bt_addr_le_eq(addr, &IRR_bt_addr)) {

			if(atomic_test_and_set_bit(flags, IRR_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("Internal RR connected, addr: %s", addr_str);
			
		} else if (bt_addr_le_eq(addr, &EFL_bt_addr)) {

			if(atomic_test_and_set_bit(flags, EFL_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("External FL connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &EFR_bt_addr)) {

			if(atomic_test_and_set_bit(flags, EFR_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("External FR connected, addr: %s", addr_str);

		} else if (bt_addr_le_eq(addr, &ERL_bt_addr)) {

			if(atomic_test_and_set_bit(flags, ERL_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("External RL connected, addr: %s", addr_str);
			
		} else if (bt_addr_le_eq(addr, &ERR_bt_addr)) {

			if(atomic_test_and_set_bit(flags, ERR_CONNECTED_FLAG)) {
				LOG_WRN("WARNING, DUPLICATE CONNECTION:");
			}
			LOG_INF("External RR connected, addr: %s", addr_str);
			
		} else {
			LOG_INF("Unrecognized device connected, addr: %s", addr_str);
		}

	}


	// NOTE: there's gotta be a way to have the BT thread prioritize notifications over scanning. below is kinda dumb

	// if all the sensors we care about are connected, use slow scanning so that BT thread is used mainly for TTPMS throughput
	// if we still want to find more sensors, use fast scanning to get them connected quick
	if (atomic_test_bit(flags, EFL_CONNECTED_FLAG) && atomic_test_bit(flags, EFR_CONNECTED_FLAG)) {

		scan_param.interval = BT_GAP_SCAN_SLOW_INTERVAL_1;
		scan_param.window = BT_GAP_SCAN_SLOW_WINDOW_1;
		err = bt_conn_le_create_auto(&scan_param, &conn_param);
		if (err) {
			LOG_ERR("Failed to start automatically connecting (err %d)", err);
		}

	} else {

		scan_param.interval = BT_GAP_SCAN_FAST_INTERVAL;
		scan_param.window = BT_GAP_SCAN_FAST_WINDOW;
		err = bt_conn_le_create_auto(&scan_param, &conn_param);
		if (err) {
			LOG_ERR("Failed to start automatically connecting (err %d)", err);
		}

	}

	

}

// Here we clear the connected bit for the sensor that disconnected (self-explanatory),
// and we also clear the subscribe bit so that the logic in main will know we need to
// subscribe again if re-connected (although there is also be a callback indicating the unsubscribe event upon disconnect).
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;

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
		LOG_INF("Unknown device disconnected, addr: %s (reason 0x%02x)", addr_str, reason);
	}


	// see note in connected(), it would be nice to remove the below if the BT thread can be made to prioritize notifications over scanning
	// if we lost a sensor we want to have, stop slow scanning and start fast scanning to get it connected again quickly
	if (!(atomic_test_bit(flags, EFL_CONNECTED_FLAG) && atomic_test_bit(flags, EFR_CONNECTED_FLAG))) {

		err = bt_conn_create_auto_stop();
		if (err) {
			LOG_ERR("Failed to stop automatically connecting (err %d)", err);
		}

		scan_param.interval = BT_GAP_SCAN_FAST_INTERVAL;
		scan_param.window = BT_GAP_SCAN_FAST_WINDOW;
		err = bt_conn_le_create_auto(&scan_param, &conn_param);
		if (err) {
			LOG_ERR("Failed to start automatically connecting (err %d)", err);
		}

	}

}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void IFL_temp_subscribed_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params)
{
	if(params->value == BT_GATT_CCC_NOTIFY) {

		atomic_set_bit(flags, IFL_SUBSCRIBED_FLAG);
		LOG_INF("IFL_temp_subscribed_cb: subscribed");

	} else if (params->value == 0) {

		atomic_clear_bit(flags, IFL_SUBSCRIBED_FLAG);
		LOG_INF("IFL_temp_subscribed_cb: unsubscribed");

	} else {
		LOG_WRN("IFL_temp_subscribed_cb: unknown CCC value");
	}
}

uint8_t IFL_temp_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
	
	if (data == NULL){	// When successfully unsubscribed, (or if unpurposefully unsubscribed?), notify callback is called one last time with data set to NULL (from Zephyr docs)
		LOG_INF("IFL_temp_notify_cb: unsubscribed");
		atomic_clear_bit(flags, IFL_SUBSCRIBED_FLAG);
		return BT_GATT_ITER_STOP;
	}

	if (!atomic_test_bit(flags, TEMP_ENABLED_FLAG)) {	// if temp is not enabled, we need to unsubscribe
		LOG_INF("IFL_temp_notify_cb: attempting to unsubscribe");
		return BT_GATT_ITER_STOP;	// returning this tells the BT Host to unsubscribe us
	}

	if (length != 16) {
		LOG_ERR("IFL_temp_notify_cb: Invalid data received from notification");
		return BT_GATT_ITER_CONTINUE;
	}
	
	//LOG_INF("IFL_temp_notify_cb: Notification received");

	// fill CAN frames with data
	for (int i = 0; i < 8; i++)
	{
		IFL_temp_1.data[i] = ((uint8_t *)data)[i];
	}
	for (int i = 0; i < 8; i++)
	{
		IFL_temp_2.data[i] = ((uint8_t *)data)[8 + i];
	}

	// let the system workqueue actually send the frames (can_send is blocking)
	k_work_submit(&IFL_temp_CAN_tx_work);

	return BT_GATT_ITER_CONTINUE;	// stay subscribed
}

static struct bt_gatt_subscribe_params IFL_temp_subscribe_params = {
		.value = BT_GATT_CCC_NOTIFY,
		.notify = IFL_temp_notify_cb,
		.subscribe = IFL_temp_subscribed_cb,
		.value_handle = TTPMS_GATT_TEMP_HANDLE,
		.ccc_handle = TTPMS_GATT_TEMP_HANDLE + 1,	// see note in ttpms_common.h
};


void EFL_temp_subscribed_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params)
{
	if(params->value == BT_GATT_CCC_NOTIFY) {

		atomic_set_bit(flags, EFL_SUBSCRIBED_FLAG);
		LOG_INF("EFL_temp_subscribed_cb: subscribed");

	} else if (params->value == 0) {

		atomic_clear_bit(flags, EFL_SUBSCRIBED_FLAG);
		LOG_INF("EFL_temp_subscribed_cb: unsubscribed");

	} else {
		LOG_WRN("EFL_temp_subscribed_cb: unknown CCC value");
	}
}

uint8_t EFL_temp_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
	
	if (data == NULL){	// When successfully unsubscribed, (or if unpurposefully unsubscribed?), notify callback is called one last time with data set to NULL (from Zephyr docs)
		LOG_INF("EFL_temp_notify_cb: unsubscribed");
		atomic_clear_bit(flags, EFL_SUBSCRIBED_FLAG);
		return BT_GATT_ITER_STOP;
	}

	if (!atomic_test_bit(flags, TEMP_ENABLED_FLAG)) {	// if temp is not enabled, we need to unsubscribe
		LOG_INF("EFL_temp_notify_cb: attempting to unsubscribe");
		return BT_GATT_ITER_STOP;	// returning this tells the BT Host to unsubscribe us
	}

	if (length != 32) {
		LOG_ERR("EFL_temp_notify_cb: Invalid data received from notification");
		return BT_GATT_ITER_CONTINUE;
	}
	
	//LOG_INF("EFL_temp_notify_cb: Notification received");

	// fill CAN frames with data
	for (int i = 0; i < 8; i++)
	{
		EFL_temp_1.data[i] = ((uint8_t *)data)[i];
	}
	for (int i = 0; i < 8; i++)
	{
		EFL_temp_2.data[i] = ((uint8_t *)data)[8 + i];
	}
	for (int i = 0; i < 8; i++)
	{
		EFL_temp_3.data[i] = ((uint8_t *)data)[16 + i];
	}
	for (int i = 0; i < 8; i++)
	{
		EFL_temp_4.data[i] = ((uint8_t *)data)[24 + i];
	}

	// let the system workqueue actually send the frames (can_send is blocking)
	k_work_submit(&EFL_temp_CAN_tx_work);

	return BT_GATT_ITER_CONTINUE;	// stay subscribed
}

static struct bt_gatt_subscribe_params EFL_temp_subscribe_params = {
		.value = BT_GATT_CCC_NOTIFY,
		.notify = EFL_temp_notify_cb,
		.subscribe = EFL_temp_subscribed_cb,
		.value_handle = TTPMS_GATT_TEMP_HANDLE,
		.ccc_handle = TTPMS_GATT_TEMP_HANDLE + 1,	// see note in ttpms_common.h
};


void EFR_temp_subscribed_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params)
{
	if(params->value == BT_GATT_CCC_NOTIFY) {

		atomic_set_bit(flags, EFR_SUBSCRIBED_FLAG);
		LOG_INF("EFR_temp_subscribed_cb: subscribed");

	} else if (params->value == 0) {

		atomic_clear_bit(flags, EFR_SUBSCRIBED_FLAG);
		LOG_INF("EFR_temp_subscribed_cb: unsubscribed");

	} else {
		LOG_WRN("EFR_temp_subscribed_cb: unknown CCC value");
	}
}

uint8_t EFR_temp_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
	
	if (data == NULL){	// When successfully unsubscribed, (or if unpurposefully unsubscribed?), notify callback is called one last time with data set to NULL (from Zephyr docs)
		LOG_INF("EFR_temp_notify_cb: unsubscribed");
		atomic_clear_bit(flags, EFR_SUBSCRIBED_FLAG);
		return BT_GATT_ITER_STOP;
	}

	if (!atomic_test_bit(flags, TEMP_ENABLED_FLAG)) {	// if temp is not enabled, we need to unsubscribe
		LOG_INF("EFR_temp_notify_cb: attempting to unsubscribe");
		return BT_GATT_ITER_STOP;	// returning this tells the BT Host to unsubscribe us
	}

	if (length != 32) {
		LOG_ERR("EFR_temp_notify_cb: Invalid data received from notification");
		return BT_GATT_ITER_CONTINUE;
	}
	
	//LOG_INF("EFR_temp_notify_cb: Notification received");

	// fill CAN frames with data
	for (int i = 0; i < 8; i++)
	{
		EFR_temp_1.data[i] = ((uint8_t *)data)[i];
	}
	for (int i = 0; i < 8; i++)
	{
		EFR_temp_2.data[i] = ((uint8_t *)data)[8 + i];
	}
	for (int i = 0; i < 8; i++)
	{
		EFR_temp_3.data[i] = ((uint8_t *)data)[16 + i];
	}
	for (int i = 0; i < 8; i++)
	{
		EFR_temp_4.data[i] = ((uint8_t *)data)[24 + i];
	}

	// let the system workqueue actually send the frames (can_send is blocking)
	k_work_submit(&EFR_temp_CAN_tx_work);

	return BT_GATT_ITER_CONTINUE;	// stay subscribed
}

static struct bt_gatt_subscribe_params EFR_temp_subscribe_params = {
		.value = BT_GATT_CCC_NOTIFY,
		.notify = EFR_temp_notify_cb,
		.subscribe = EFR_temp_subscribed_cb,
		.value_handle = TTPMS_GATT_TEMP_HANDLE,
		.ccc_handle = TTPMS_GATT_TEMP_HANDLE + 1,	// see note in ttpms_common.h
};

// NOTE: each sensor needs to have its own subscribe_params variable since it remains tied to each subscription (from Zephyr docs)

void TTPMS_BLE_init(void)
{
	int err;

	bt_addr_le_t addr;

	// create self BT address
	err = bt_addr_le_from_str(TTPMS_RX_BT_ID, "random", &addr);
	if (err) {
		LOG_WRN("Invalid BT address (err %d)", err);
	}

	// assign self BT address
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

	err = bt_le_filter_accept_list_add(&ERR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)", err); }

	scan_param.interval = BT_GAP_SCAN_FAST_INTERVAL;
	scan_param.window = BT_GAP_SCAN_FAST_WINDOW;
	err = bt_conn_le_create_auto(&scan_param, &conn_param);
	if (err) {
		LOG_ERR("Failed to start automatically connecting (err %d)", err);
	}
}

/* --- BLE STUFF END --- */



void main(void)
{
	LOG_INF("Running ttpms_v2_receiver in CAN bus mode");

	k_sleep(K_MSEC(1));

	TTPMS_CAN_init();

	TTPMS_BLE_init();

	
	int err;

	struct bt_conn *conn;

	int counter = 0;

	while(1)
	{
		
		if (atomic_test_bit(flags, TEMP_ENABLED_FLAG))	{ // if temp is enabled, make sure we are subscribed to all connected sensors

			if (atomic_test_bit(flags, IFL_CONNECTED_FLAG) && !atomic_test_bit(flags, IFL_SUBSCRIBED_FLAG)) // if connected and not subscribed, we need to subscribe
			{
				LOG_INF("main: Attempting to subscribe to IFL temp");
				IFL_temp_subscribe_params.value = BT_GATT_CCC_NOTIFY;	// this gets changed to 0 by the BT stack after an unsubscription event, need to set it back
				atomic_set_bit(flags, IFL_SUBSCRIBED_FLAG);	// bt_gatt_subscribe is not blocking, so if we don't set this here, we may try to subscribe twice!
				conn = bt_conn_lookup_addr_le(bt_identity, &IFL_bt_addr);
				err = bt_gatt_subscribe(conn, &IFL_temp_subscribe_params);
				if (err) {
					LOG_WRN("main: Failed to subscribe to IFL temp (err %d)", err);
					atomic_clear_bit(flags, IFL_SUBSCRIBED_FLAG);	// see note above. must clear if we actually didn't subscribe
				}
				bt_conn_unref(conn);
			}

			if (atomic_test_bit(flags, EFL_CONNECTED_FLAG) && !atomic_test_bit(flags, EFL_SUBSCRIBED_FLAG)) // if connected and not subscribed, we need to subscribe
			{
				LOG_INF("main: Attempting to subscribe to EFL temp");
				EFL_temp_subscribe_params.value = BT_GATT_CCC_NOTIFY;	// this gets changed to 0 by the BT stack after an unsubscription event, need to set it back
				atomic_set_bit(flags, EFL_SUBSCRIBED_FLAG);	// bt_gatt_subscribe is not blocking, so if we don't set this here, we may try to subscribe twice!
				conn = bt_conn_lookup_addr_le(bt_identity, &EFL_bt_addr);
				err = bt_gatt_subscribe(conn, &EFL_temp_subscribe_params);
				if (err) {
					LOG_WRN("main: Failed to subscribe to EFL temp (err %d)", err);
					atomic_clear_bit(flags, EFL_SUBSCRIBED_FLAG);	// see note above. must clear if we actually didn't subscribe
				}
				bt_conn_unref(conn);
			}

			if (atomic_test_bit(flags, EFR_CONNECTED_FLAG) && !atomic_test_bit(flags, EFR_SUBSCRIBED_FLAG)) // if connected and not subscribed, we need to subscribe
			{
				LOG_INF("main: Attempting to subscribe to EFR temp");
				EFR_temp_subscribe_params.value = BT_GATT_CCC_NOTIFY;	// this gets changed to 0 by the BT stack after an unsubscription event, need to set it back
				atomic_set_bit(flags, EFR_SUBSCRIBED_FLAG);	// bt_gatt_subscribe is not blocking, so if we don't set this here, we may try to subscribe twice!
				conn = bt_conn_lookup_addr_le(bt_identity, &EFR_bt_addr);
				err = bt_gatt_subscribe(conn, &EFR_temp_subscribe_params);
				if (err) {
					LOG_WRN("main: Failed to subscribe to EFR temp (err %d)", err);
					atomic_clear_bit(flags, EFR_SUBSCRIBED_FLAG);	// see note above. must clear if we actually didn't subscribe
				}
				bt_conn_unref(conn);
			}

		} 
		// NOTE: the notify callbacks will unsubscribe themselves if they see that temp is not enabled
	
		k_sleep(K_MSEC(100));

		counter++;
		if (counter >= 5) {		// send out TTPMS status message to dash every 500ms
			counter = 0;
			TTPMS_status.data[0] = (atomic_test_bit(flags, TEMP_ENABLED_FLAG) | (atomic_test_bit(flags, PRESSURE_ENABLED_FLAG) << 1));
			TTPMS_status.data[1] = (atomic_get(flags) & 0xFF);
			k_work_submit(&status_CAN_tx_work);
		}
		
	}
	
}
