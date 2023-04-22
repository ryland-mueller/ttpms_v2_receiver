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

#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)

const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);

struct can_frame test_frame = {
		.flags = 0,
		.id = 0x100,
		.dlc = 8
};

// Standard 11-bit CAN IDs can be up to 2047 (0x7FF). Lower = higher priority
#define TTPMS_CAN_BASE_ID 0x400

/* START CAN FRAMES*/
// Ensure this is always up to date with CAN .dbc file to be kept in version control by the team

// This frame is sent by dash or other controller to enable & configure TTPMS
struct can_frame TTPMS_settings = {.flags = 0, .id = TTPMS_CAN_BASE_ID, .dlc = 1};

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

// First hex char must be C for random static address
#define TTPMS_RX_BT_ID "CA:69:F1:F1:69:69"		// for this device
#define TTPMS_IFL_BT_ID "CA:69:F1:F1:33:42"		// Internal Front Left BT ID
#define TTPMS_IFR_BT_ID "CA:69:F1:F1:33:43"		// Internal Front Right BT ID
#define TTPMS_IRL_BT_ID "CA:69:F1:F1:33:44"		// Internal Rear Left BT ID
#define TTPMS_IRR_BT_ID "CA:69:F1:F1:33:45"		// Internal Rear Right BT ID
#define TTPMS_EFL_BT_ID "CA:69:F1:F1:11:22"		// External Front Left BT ID
#define TTPMS_EFR_BT_ID "CA:69:F1:F1:11:23"		// External Front Right BT ID
#define TTPMS_ERL_BT_ID "CA:69:F1:F1:11:24"		// External Rear Left BT ID
#define TTPMS_ERR_BT_ID "CA:69:F1:F1:11:25"		// External Rear Right BT ID

#define TTPMS_TEST_BT_ID "F6:B3:F2:9C:20:20"	// from nrf dongle (for testing, may change randomly?)

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

static struct bt_conn *IFL_conn;
static struct bt_conn *IFR_conn;
static struct bt_conn *IRL_conn;
static struct bt_conn *IRR_conn;
static struct bt_conn *EFL_conn;
static struct bt_conn *EFR_conn;
static struct bt_conn *ERL_conn;
static struct bt_conn *ERR_conn;

static struct bt_conn *TEST_conn;

/* --- BLE FUNCTIONS START --- */

// start_scan and device_found reference each other, so one must be declared first
static void start_scan(void);

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	int err;

	struct bt_conn *conn_ptr;

	char addr_str[BT_ADDR_LE_STR_LEN];

	// this is just for printing to log
	//bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if (bt_addr_le_eq(addr, &IFL_bt_addr)) {
		conn_ptr = &IFL_conn;
	} else if (bt_addr_le_eq(addr, &IFR_bt_addr)) {
		conn_ptr = &IFR_conn;
	} else if (bt_addr_le_eq(addr, &IRL_bt_addr)) {
		conn_ptr = &IRL_conn;
	} else if (bt_addr_le_eq(addr, &IRR_bt_addr)) {
		conn_ptr = &IRR_conn;
	} else if (bt_addr_le_eq(addr, &EFL_bt_addr)) {
		conn_ptr = &EFL_conn;
	} else if (bt_addr_le_eq(addr, &EFR_bt_addr)) {
		conn_ptr = &EFR_conn;
	} else if (bt_addr_le_eq(addr, &ERL_bt_addr)) {
		conn_ptr = &ERL_conn;
	} else if (bt_addr_le_eq(addr, &ERR_bt_addr)) {
		conn_ptr = &ERR_conn;
	} else if (bt_addr_le_eq(addr, &TEST_bt_addr)) {
		conn_ptr = &TEST_conn;
	} else {
		return;
	}

	if (bt_le_scan_stop()) {	// must stop scanning before connecting
		return;
	}


	struct bt_le_conn_param conn_param = {
		.interval_min = CONN_INTERVAL,
		.interval_max = CONN_INTERVAL,
		.latency = CONN_LATENCY,
		.timeout = CONN_TIMEOUT,
	};

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, &conn_param, conn_ptr);
	if (err) {
		LOG_WRN("Create conn failed (%u)\n", err);
		start_scan();
	}

}

static void start_scan(void)
{
	int err;

	// Add address of the devices we want to filter accept list (these address variables were filled in main)
	err = bt_le_filter_accept_list_add(&IFL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }
	err = bt_le_filter_accept_list_add(&IFR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }
	err = bt_le_filter_accept_list_add(&IRL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }
	err = bt_le_filter_accept_list_add(&IRR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }
	err = bt_le_filter_accept_list_add(&EFL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }
	err = bt_le_filter_accept_list_add(&EFR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }
	err = bt_le_filter_accept_list_add(&ERL_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }
	err = bt_le_filter_accept_list_add(&ERR_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }

	err = bt_le_filter_accept_list_add(&TEST_bt_addr);
	if (err) { LOG_WRN("Failed to add address to filter accept list (err %d)\n", err); }

	err = bt_le_scan_start(BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE, \
					    BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST, \
					    BT_GAP_SCAN_FAST_INTERVAL, \
					    BT_GAP_SCAN_FAST_WINDOW), device_found);
	if (err) {
		LOG_WRN("Scanning failed to start (err %d)\n", err);
		return;
	}

	LOG_INF("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_WRN("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	LOG_INF("Connected: %s\n", addr);

	//bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* --- BLE FUNCTIONS END --- */


/* --- CAN FUNCTIONS START --- */

// CAN TX callback function (necessary for non-blocking TX, why??? also, is blocking fine since it's in workqueue?)
void tx_irq_callback(const struct device *dev, int error, void *arg)
{
	char *sender = (char *)arg;

	ARG_UNUSED(dev);

	if (error != 0) {
		LOG_WRN("Callback! error-code: %d\nSender: %s\n",
		       error, sender);
	}
}

/* --- CAN FUNCTIONS END --- */


/* --- WORK FUNCTIONS START --- */

// submitting to system work queue is necessary so that this isn't blocking
// (SPI was crashing without this)
void CAN_transmit_work_handler(struct k_work *work)
{
	// this will be triggered by BLE notifications
	test_frame.data[0] = 0x69;
	test_frame.data[1] = 0x42;
	test_frame.data[2] = 0x11;
	test_frame.data[3] = 0x22;
	test_frame.data[4] = 0x33;
	test_frame.data[5] = 0x44;
	test_frame.data[6] = 0x55;
	test_frame.data[7] = 0xAF;
	can_send(can_dev, &test_frame, K_FOREVER, tx_irq_callback, "Test message");
}

K_WORK_DEFINE(CAN_transmit_work, CAN_transmit_work_handler);


/* --- WORK FUNCTIONS END --- */


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

	bt_addr_le_t addr;

	// create self BT address
	err = bt_addr_le_from_str(TTPMS_RX_BT_ID, "random", &addr);
	if (err) {
		LOG_WRN("Invalid BT address (err %d)\n", err);
	}

	// assign self BT address
	err = bt_id_create(&addr, NULL);
	if (err < 0) {
		LOG_WRN("Creating new BT ID failed (err %d)\n", err);
	}

	// fill address variables for the devices we want to filter for
	err = bt_addr_le_from_str(TTPMS_IFL_BT_ID, "random", &IFL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }
	err = bt_addr_le_from_str(TTPMS_IFR_BT_ID, "random", &IFR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }
	err = bt_addr_le_from_str(TTPMS_IRL_BT_ID, "random", &IRL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }
	err = bt_addr_le_from_str(TTPMS_IRR_BT_ID, "random", &IRR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }
	err = bt_addr_le_from_str(TTPMS_EFL_BT_ID, "random", &EFL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }
	err = bt_addr_le_from_str(TTPMS_EFR_BT_ID, "random", &EFR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }
	err = bt_addr_le_from_str(TTPMS_ERL_BT_ID, "random", &ERL_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }
	err = bt_addr_le_from_str(TTPMS_ERR_BT_ID, "random", &ERR_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }

	err = bt_addr_le_from_str(TTPMS_TEST_BT_ID, "random", &TEST_bt_addr);
	if (err) { LOG_WRN("Invalid BT address (err %d)\n", err); }

	err = bt_enable(NULL);
	if (err) {
		LOG_WRN("Bluetooth init failed (err %d)\n", err);
	} else {
		LOG_INF("Bluetooth initialized\n");
	}

	start_scan();
}