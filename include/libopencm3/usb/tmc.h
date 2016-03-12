/** @defgroup usb_audio_defines USB Audio Type Definitions
 *
 * @ brief <b>Definitions for the USB Test and Measurement Class</b>
 *
 * @ingroup USB_defines
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2016
 * Alexandru Gagniuc <mr.nuke.me@gmail.com>
 *
 * @date 01 April 2016
 *
 * LGPL License Terms @ref lgpl_license
 */

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2016 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**@{*/

#ifndef __LIBOPENCM3_USB_TMC_H
#define __LIBOPENCM3_USB_TMC_H

#include <stdint.h>

#define USB_SUBCLASS_USBTMC		0x03

/* Table 2: MsgID values */
enum usbtmc_msg_id {
	USBTMC_DEV_DEP_MSG_OUT		= 1,
	USBTMC_DEV_DEP_MSG_IN		= 2,
	USBTMC_MSG_VENDOR_SPECIFIC_OUT	= 126,
	USBTMC_MSG_VENDOR_SPECIFIC_IN	= 127,
};

/* Table 15: USBTMC bRequest values */
enum usbtmc_request_type {
	USBTMC_REQ_INITIATE_ABORT_BULK_OUT	= 1,
	USBTMC_REQ_CHECK_ABORT_BULK_OUT_STATUS	= 2,
	USBTMC_REQ_INITIATE_ABORT_BULK_IN	= 3,
	USBTMC_REQ_CHECK_ABORT_BULK_IN_STATUS	= 4,
	USBTMC_REQ_INITIATE_CLEAR		= 5,
	USBTMC_REQ_CHECK_CLEAR_STATUS		= 6,
	USBTMC_REQ_GET_CAPABILITIES		= 7,
	USBTMC_REQ_INDICATOR_PULSE		= 64,
};

/* Table 16: USBTMC status values */
enum usbtmc_status {
	USBTMC_STATUS_SUCCESS			= 0x01,
	USBTMC_STATUS_PENDING			= 0x02,
	USBTMC_STATUS_FAILED			= 0x80,
	USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS	= 0x81,
	USBTMC_STATUS_SPLIT_NOT_IN_PROGRESS	= 0x82,
	USBTMC_STATUS_SPLIT_IN_PROGRESS		= 0x83,
};

/* Splattered across the USBTMC spec: bmTransferAttributes bitmask values */
enum usbtmc_xfer_attributes {
	USBTMC_XFER_ATTR_EOM		= 1 << 0,
	USBTMC_XFER_ATTR_TERM_CHAR	= 1 << 1,
};

/* Table 37: GET_CAPABILITIES response format */
enum usbtmc_interface_capabilities {
	USBTMC_CAP_INDICATOR_PULSE	= 1 << 2,
	USBTMC_CAP_TALK_ONLY		= 1 << 1,
	USBTMC_CAP_LISTEN_ONLY		= 1 << 0,
};

/* Table 37: GET_CAPABILITIES response format */
enum usbtmc_dev_capabilities {
	USBTMC_CAP_TERM_CHAR		= 1 << 0,
};

/*
 * Table 37: GET_CAPABILITIES response format
 * WARNING: These fields must be sent in little-endian order over USB.
 */
struct usbtmc_capabilities {
	uint8_t status;
	uint8_t reserved;
	uint16_t bcd_usbtmc;
	uint8_t interface_capabilities;
	uint8_t device_capabilities;
	uint8_t reserved_6[6];
} __attribute__((packed));

#endif /* __LIBOPENCM3_USB_TMC_H */

/**@}*/
