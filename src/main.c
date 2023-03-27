// includes that will always be required
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>

// includes for debugging/temporary
#include <zephyr/logging/log.h>

#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)

LOG_MODULE_REGISTER(ttpms);

void tx_irq_callback(const struct device *dev, int error, void *arg)
{
	char *sender = (char *)arg;

	ARG_UNUSED(dev);

	if (error != 0) {
		LOG_WRN("Callback! error-code: %d\nSender: %s\n",
		       error, sender);
	}
}

void main(void)
{
	LOG_INF("Running ttpms_v2_receiver");

	const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
	struct k_sem tx_queue_sem;

	struct can_frame test_frame = {
		.flags = 0,
		.id = 0x100,
		.dlc = 2,
		.data = {0x69, 0x42}
	};

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

	while (1) {
		can_send(can_dev, &test_frame, K_FOREVER, tx_irq_callback, "Test message");
		k_sleep(K_MSEC(1000));
	}

}
