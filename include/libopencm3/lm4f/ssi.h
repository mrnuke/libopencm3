/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2012 - 2013 Mauro Scomparin <scompo@gmail.com>
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

#ifndef LM4F_SSI_H
#define LM4F_SSI_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/lm4f/memorymap.h>

/* 
 * ============================================================================
 * Convenience macros
 * ----------------------------------------------------------------------------
 */
/** @defgroup ssi_reg_base SSI register base addresses
 * @{*/
#define SSI0				SSI0_BASE
#define SSI1				SSI1_BASE
#define SSI2				SSI2_BASE
#define SSI3				SSI3_BASE
/** @} */

/* 
 * ============================================================================
 * SSI registers
 * ----------------------------------------------------------------------------
 */

/* SSI control registers */
#define SSI_CR0(ssi_base)		MMIO32(ssi_base + 0x00)
#define SSI_CR1(ssi_base)		MMIO32(ssi_base + 0x04)

/* SSI data register */
#define SSI_DR(ssi_base)		MMIO32(ssi_base + 0x08)

/* SSI status register */
#define SSI_SR(ssi_base)		MMIO32(ssi_base + 0x0c)

/* SSI clock prescaler register */
#define SSI_CPSR(ssi_base)		MMIO32(ssi_base + 0x10)

/* 
* SSI interrupt registers 
* (mask, raw status, masked status, clear)
*/
#define SSI_IM(ssi_base)		MMIO32(ssi_base + 0x14)
#define SSI_RIS(ssi_base)		MMIO32(ssi_base + 0x18)
#define SSI_MIS(ssi_base)		MMIO32(ssi_base + 0x1c)
#define SSI_ICR(ssi_base)		MMIO32(ssi_base + 0x20)

/* SSI DMA control register */
#define SSI_DMACTL(ssi_base)		MMIO32(ssi_base + 0x24)

/* SSI Clock configuration register */
#define SSI_CC(ssi_base)		MMIO32(ssi_base + 0xFc8)

/* SSI peripherial indentification registers (0-7) */
#define SSI_PERIPH_ID4(ssi_base)	MMIO32(ssi_base + 0xfd0)
#define SSI_PERIPH_ID5(ssi_base)	MMIO32(ssi_base + 0xfd4)
#define SSI_PERIPH_ID6(ssi_base)	MMIO32(ssi_base + 0xfd8)
#define SSI_PERIPH_ID7(ssi_base)	MMIO32(ssi_base + 0xfdc)
#define SSI_PERIPH_ID0(ssi_base)	MMIO32(ssi_base + 0xfe0)
#define SSI_PERIPH_ID1(ssi_base)	MMIO32(ssi_base + 0xfe4)
#define SSI_PERIPH_ID2(ssi_base)	MMIO32(ssi_base + 0xfe8)
#define SSI_PERIPH_ID3(ssi_base)	MMIO32(ssi_base + 0xfec)

/* SSI PrimeCell identification registers (0-3) */
#define SSI_PCELL_ID0(ssi_base)		MMIO32(ssi_base + 0xff0)
#define SSI_PCELL_ID1(ssi_base)		MMIO32(ssi_base + 0xff4)
#define SSI_PCELL_ID2(ssi_base)		MMIO32(ssi_base + 0xff8)
#define SSI_PCELL_ID3(ssi_base)		MMIO32(ssi_base + 0xffc)

/* 
* =============================================================================
* SSI_CR0 register
* -----------------------------------------------------------------------------
*/
/** DSS (data size) values */
#define SSI_CR0_DSS_MASK		(0xf << 0)
/* 0-2 reserved */
#define SSI_CR0_DSS_3			(0x3 << 0)
#define SSI_CR0_DSS_4			(0x4 << 0)
#define SSI_CR0_DSS_5			(0x5 << 0)
#define SSI_CR0_DSS_6			(0x6 << 0)
#define SSI_CR0_DSS_7			(0x7 << 0)
#define SSI_CR0_DSS_8			(0x8 << 0)
#define SSI_CR0_DSS_9			(0x9 << 0)
#define SSI_CR0_DSS_10			(0xa << 0)
#define SSI_CR0_DSS_11			(0xb << 0)
#define SSI_CR0_DSS_12			(0xc << 0)
#define SSI_CR0_DSS_13			(0xd << 0)
#define SSI_CR0_DSS_14			(0xe << 0)
#define SSI_CR0_DSS_15			(0xf << 0)
/** FRF (frame format) values */
#define SSI_CR0_FRF_MASK		(0x3 << 4)
#define SSI_CR0_FRF_0			(0 << 4)
#define SSI_CR0_FRF_1			(1 << 4)
#define SSI_CR0_FRF_2			(2 << 4)
/* 3 reserved */
/** SPO (serial clock polarity) */
#define SSI_CR0_SPO			(1 << 6)
/** SPH (serial clock phase) */
#define SSI_CR0_SPH			(1 << 7)
/** SCR (serial clock rate) values */
#define SSI_CR0_SCR_MASK		(0xff << 8)

/* 
* =============================================================================
* SSI_CR1 register
* -----------------------------------------------------------------------------
*/
/** SLBY6 (slave bypass mode) */
#define SSI_CR1_SLBY6			(1 << 5)
/** EOT (end of transmission) */
#define SSI_CR1_EOT			(1 << 4)
/** SOD (slave mode output disable) */
#define SSI_CR1_SOD			(1 << 3)
/** MS (master slave mode select) */
#define SSI_CR1_MS			(1 << 2)
/** SEE (syncronous serial port enable) */
#define SSI_CR1_SEE			(1 << 1)
/** LBM (loopback mode) */
#define SSI_CR1_LBM			(1 << 0)

/* 
* =============================================================================
* SSI_DR register
* -----------------------------------------------------------------------------
*/
/** Data mask */
#define SSI_DR_DATA_MASK		(0xff << 0)

/* 
* =============================================================================
* SSI_SR register
* -----------------------------------------------------------------------------
*/
/** ssi busy */
#define SSI_SR_BSY			(1 << 4)
/** ssi recv fifo full */
#define SSI_SR_RFF			(1 << 3)
/** ssi recv fifo not empty */
#define SSI_SR_RNE			(1 << 2)
/** ssi tx fifo not full */
#define SSI_SR_TNF			(1 << 1)
/** ssi tx fifo empty */
#define SSI_SR_TFE			(1 << 0)

/* 
* =============================================================================
* SSI_CPSR register
* -----------------------------------------------------------------------------
*/
/** prescaler divisor mask */
#define SSI_CPSR_CPSDVSR_MASK		(0xff << 0)

/* 
* =============================================================================
* SSI_IM register
* -----------------------------------------------------------------------------
*/
/** tx fifo mask interrupt */
#define SSI_IM_TXIM			(1 << 3)
/** rx fifo mask interrupt */
#define SSI_IM_RXIM			(1 << 2)
/** recv timeout mask interrupt */
#define SSI_IM_RTIM			(1 << 1)
/** recv overrun mask interrupt */
#define SSI_IM_RORIM			(1 << 0)

/* 
* =============================================================================
* SSI_RIS register
* -----------------------------------------------------------------------------
*/
/** tx fifo status interrupt */
#define SSI_RIS_TXRIS			(1 << 3)
/** rx fifo status interrupt */
#define SSI_RIS_RXRIS			(1 << 2)
/** recv timeout status interrupt */
#define SSI_RIS_RTRIS			(1 << 1)
/** recv overrun status interrupt */
#define SSI_RIS_RORRIS			(1 << 0)

/* 
* =============================================================================
* SSI_MIS register
* -----------------------------------------------------------------------------
*/
/** tx fifo masked status interrupt */
#define SSI_MIS_TXMIS			(1 << 3)
/** rx fifo masked status interrupt */
#define SSI_MIS_RXMIS			(1 << 2)
/** recv timeout masked status interrupt */
#define SSI_MIS_RTMIS			(1 << 1)
/** recv overrun masked status interrupt */
#define SSI_MIS_RORMIS			(1 << 0)

/* 
* =============================================================================
* SSI_ICR register
* -----------------------------------------------------------------------------
*/
/** recive timeout int clear */
#define SSI_ICR_RTIC			(1 << 1)
/** recive overrun int clear */
#define SSI_ICR_RORIC			(1 << 0)

/* 
* =============================================================================
* SSI_DMACTL register
* -----------------------------------------------------------------------------
*/
/** tx dma enable */
#define SSI_DMACTL_TXDMAE 		(1 << 1)
/** rx dma enable */
#define SSI_DMACTL_RXDMAE 		(1 << 0)

/* 
* =============================================================================
* SSI_CC register
* -----------------------------------------------------------------------------
*/
/** baud clock source values */
#define SSI_CC_CS_MASK			(0xf << 0)
/* values 1-4, 6-0xf reserved */
#define SSI_CC_CS_0			(0 << 0)
#define SSI_CC_CS_5			(5 << 0)

/* 
* Peripherial id + primecell registers TODO!!!!!!!
*/

/* 
* =============================================================================
* Convenience enums.
* -----------------------------------------------------------------------------
*/

typedef enum{
	SSI_CLOCK_SOURCE_SYSTEM,
	SSI_CLOCK_SOURCE_PIOSC,
}ssi_clock_source;

typedef enum{
	SSI_MODE_MASTER,
	SSI_MODE_SLAVE_TX_ENABLED,
	SSI_MODE_SLAVE_TX_DISABLED,
}ssi_mode;

typedef enum{
	SSI_PROTOCOL_FREESCALE,
	SSI_PROTOCOL_TI,
	SSI_PROTOCOL_MICROWIRE,
}ssi_protocol;

/* 
* =============================================================================
* Function forward declarations.
* -----------------------------------------------------------------------------
*/
BEGIN_DECLS

void ssi_enable(u32 ssi);
void ssi_disable(u32 ssi);
void ssi_set_mode(u32 ssi, ssi_mode mode);
void ssi_set_frame_format(u32 ssi, 
			u8 bits, u8 phase, 
			u8 polarity, ssi_protocol protocol);
void ssi_set_clock_source(u32 ssi, ssi_clock_source source);
//void ssi_set_clock_rate(u32 ssi, u8 rate);
//void ssi_set_clock_prescaler(u32 ssi, u8 val);
void ssi_set_bit_rate(u32 ssi, u8 divider, u8 rate);
void ssi_set_DMA_enable(u32 ssi, u8 val);

void ssi_send(u32 ssi,u32 data);
u32 ssi_recv(u32 ssi);
END_DECLS
/* TODO add others? */

#endif /* LIBOPENCM3_LM4F_SSI_H */
