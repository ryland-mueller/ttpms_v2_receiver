/* includes that will always be required */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>

/* includes for debugging/temporary */
#include <zephyr/logging/log.h>

/* boilerplate defines, etc */
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)
LOG_MODULE_REGISTER(ttpms);


const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);

struct can_frame test_frame = {
		.flags = 0,
		.id = 0x100,
		.dlc = 8,
};


// this callback function is necessary so that can_send is non-blocking
void tx_irq_callback(const struct device *dev, int error, void *arg)
{
	char *sender = (char *)arg;

	ARG_UNUSED(dev);

	if (error != 0) {
		LOG_WRN("Callback! error-code: %d\nSender: %s\n",
		       error, sender);
	}
}


// submitting to system work queue is necessary so that this isn't blocking
// (ie SPI timing gets screwed up if you execute this directly code in the timer handler)
void ins_temp_request_work_handler(struct k_work *work)
{
	// the actual code to go here will be requesting an update from the TTPMS internal sensor temp characteristics
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

K_WORK_DEFINE(ins_temp_request_work, ins_temp_request_work_handler);

void ins_temp_request_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&ins_temp_request_work);
}

K_TIMER_DEFINE(ins_temp_request_timer, ins_temp_request_timer_handler, NULL);


void main(void)
{
	LOG_INF("Running ttpms_v2_receiver");

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

	k_timer_start(&ins_temp_request_timer, K_MSEC(100), K_MSEC(10));	// wait 100ms then execute every 10ms
	
}