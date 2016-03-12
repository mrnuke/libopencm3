/** @defgroup usb_audio_defines USB Audio Type Definitions
 *
 * @ brief <b>Definitions for the USB TMC 488 subclass</b>
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

#ifndef __LIBOPENCM3_USB_TMC_488_H
#define __LIBOPENCM3_USB_TMC_488_H

#include <libopencm3/usb/tmc.h>

#define USB_TMC_PROTOCOL_USB488		0x01

/*
 * Table 9 -- USB488 defined bRequest values
 * These are in addition to USB TMC bRequest values.
 */
enum usb488_request_type {
	USB488_REQ_READ_STATUS_BYTE	= 128,
	USB488_REQ_REN_CONTROL		= 160,
	USB488_REQ_GO_TO_LOCAL		= 161,
	USB488_REQ_LOCAL_LOCKOUT	= 162,
};

/* Table 8: GET_CAPABILITIES response packet */
enum usb488_interface_capabilities {
	USB488_CAP_488_2		= 1 << 2,
	USB488_CAP_REN_CONTROL		= 1 << 1,
	USB488_CAP_TRIGGER		= 1 << 0,
};

/* Table 8: GET_CAPABILITIES response packet */
enum usb488_dev_capabilities {
	USB488_CAP_SCPI_COMPLIANT	= 1 << 3,
	USB488_CAP_SR1			= 1 << 2,
	USB488_CAP_RL1			= 1 << 1,
	USB488_CAP_DT1			= 1 << 0,
};

/*
 * Table 8: GET_CAPABILITIES response packet
 * WARNING: These fields must be sent in little-endian order over USB.
 */
struct usb488_capabilities {
	struct usbtmc_capabilities tmc_caps;
	uint16_t bcd_usb488;
	uint8_t interface_488_capabilities;
	uint8_t device_488_capabilities;
	uint8_t reserved[8];
} __attribute__((packed));

#endif /* __LIBOPENCM3_USB_TMC_488_H */

/**@}*/
