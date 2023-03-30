#include <zephyr/device.h>
#include <zephyr/toolchain.h>

/* 1 : /soc/clock@40000000:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_64[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 2 : /soc/gpio@50000300:
 * Supported:
 *    - /soc/spi@40003000/can1@1
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_116[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 7, DEVICE_HANDLE_ENDS };

/* 3 : /soc/gpio@50000000:
 * Supported:
 *    - /soc/spi@40003000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_10[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 6, DEVICE_HANDLE_ENDS };

/* 4 : /soc/random@4000d000:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_88[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 5 : /entropy_bt_hci:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_4[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 6 : /soc/spi@40003000:
 * Direct Dependencies:
 *    - /soc/gpio@50000000
 * Supported:
 *    - /soc/spi@40003000/can1@1
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_117[] = { 3, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 7, DEVICE_HANDLE_ENDS };

/* 7 : /soc/spi@40003000/can1@1:
 * Direct Dependencies:
 *    - /soc/gpio@50000300
 *    - /soc/spi@40003000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_118[] = { 2, 6, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };
