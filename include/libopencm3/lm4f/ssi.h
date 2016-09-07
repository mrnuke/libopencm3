/** @defgroup ssi_defines Synchronous Serial Interface
 *
 * @brief <b>Defined Constants and Types for the LM4F Synchronous Serial Interface (SSI)</b>
 *
 * @ingroup LM4Fxx_defines
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2014
 * Tiago Costa <nippius+github@gmail.com>
 *
 * @date 11 June 2014
 *
 * LGPL License Terms @ref lgpl_license
 */

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Tiago Costa <nippius+github@gmail.com>
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

/**@{*/

#include <libopencm3/cm3/common.h>
#include <libopencm3/lm4f/memorymap.h>
#include <stdlib.h>

/* =============================================================================
 * Convenience macros
 * ---------------------------------------------------------------------------*/
/** @defgroup ssi_base SSI register base addresses
 * @{*/
#define SSI0				SSI0_BASE
#define SSI1				SSI1_BASE
#define SSI2				SSI2_BASE
#define SSI3				SSI3_BASE
/** @} */

/* =============================================================================
 * SSI registers
 * ---------------------------------------------------------------------------*/

/* SSI Control 0 */
#define SSI_CR0(port)		MMIO32((port) + 0x000)

/* SSI Control 1 */
#define SSI_CR1(port)		MMIO32((port) + 0x004)

/* SSI Data */
#define SSI_DR(port)		MMIO32((port) + 0x008)

/* SSI Satus */
#define SSI_SR(port)		MMIO32((port) + 0x00C)

/* SSI Clock Prescale */
#define SSI_CPSR(port)		MMIO32((port) + 0x010)

/* SSI Interrupt Mask */
#define SSI_IM(port)		MMIO32((port) + 0x014)

/* SSI Raw Interrupt Status */
#define SSI_RIS(port)		MMIO32((port) + 0x018)

/* SSI Masked Interrupt Status */
#define SSI_MIS(port)		MMIO32((port) + 0x01C)

/* SSI Interrupt Clear */
#define SSI_ICR(port)		MMIO32((port) + 0x020)

/* SSI DMA Control */
#define SSI_DMACTL(port)	MMIO32((port) + 0x024)

/* SSI Clock Configuration */
#define SSI_CC(port)		MMIO32((port) + 0xFC8)

/* SSI Peripheral Identification */
#define SSI_PERIPH_ID4(port)	MMIO32((port) + 0xFD0)
#define SSI_PERIPH_ID5(port)	MMIO32((port) + 0xFD4)
#define SSI_PERIPH_ID6(port)	MMIO32((port) + 0xFD8)
#define SSI_PERIPH_ID7(port)	MMIO32((port) + 0xFDC)
#define SSI_PERIPH_ID0(port)	MMIO32((port) + 0xFE0)
#define SSI_PERIPH_ID1(port)	MMIO32((port) + 0xFE4)
#define SSI_PERIPH_ID2(port)	MMIO32((port) + 0xFE8)
#define SSI_PERIPH_ID3(port)	MMIO32((port) + 0xFEC)

/* SSI PrimeCell Identification */
#define SSI_PCELL_ID0(port)	MMIO32((port) + 0xFF0)
#define SSI_PCELL_ID1(port)	MMIO32((port) + 0xFF4)
#define SSI_PCELL_ID2(port)	MMIO32((port) + 0xFF8)
#define SSI_PCELL_ID3(port)	MMIO32((port) + 0xFFC)

/* SSI_CR0 values */
#define SSI_CR0_SCR_MASK	(0xff << 8)
#define SSI_CR0_SCR(scr)	(((scr) << 8) & SSI_CR0_SCR_MASK)
#define SSI_CR0_SPH		(1 << 7)
#define SSI_CR0_SPO		(1 << 6)
#define SSI_CR0_FRF_MASK	(0x3 << 4)
#define SSI_CR0_FRF_FREESCALE	(0x0 << 4)
#define SSI_CR0_FRF_TEXAS	(0x1 << 4)
#define SSI_CR0_FRF_MICROWIRE	(0x2 << 4)
#define SSI_CR0_DSS(x)		((x) & 0xf)

#define SSI_CR1_EOT		(1 << 4)
#define SSI_CR1_MS		(1 << 2)
#define SSI_CR1_SSE		(1 << 1)
#define SSI_CR1_LBM		(1 << 0)

#define SSI_SR_BSY		(1 << 4)
#define SSI_SR_RFF		(1 << 3)
#define SSI_SR_RNE		(1 << 2)
#define SSI_SR_TNF		(1 << 1)
#define SSI_SR_TFE		(1 << 0)

/* Interrupt mask values (applicable to SSI_IM, SSI_RIS, SSI_MIS registers) */
#define SSI_IM_TXIM		(1 << 3)
#define SSI_IM_RXIM		(1 << 2)
#define SSI_IM_RTIM		(1 << 1)
#define SSI_IM_RORIM		(1 << 0)

#define SSI_DMACTL_TXDMAEN	(1 << 1)
#define SSI_DMACTL_RXDMAEN	(1 << 0)

#define SSI_CC_MASK		(0xf << 0)
#define SSI_CC_SYSCLK		(0x0 << 0)
#define SSI_CC_PIOSC		(0x5 << 0)

/* =============================================================================
 * Function prototypes
 * ---------------------------------------------------------------------------*/
BEGIN_DECLS

#define SPI_MODE(cpha, cpol)	(((cpha) << 1) | (cpol))

enum spi_format {
	SSI_FORMAT_FREESCALE,
	SSI_FORMAT_TEXAS,
	SSI_FORMAT_MICROWIRE,
};

enum spi_interrupt_flag {
	SPI_INT_TX = SSI_IM_TXIM,
	SPI_INT_RX = SSI_IM_RXIM,
	SPI_INT_TIMEOUT = SSI_IM_RTIM,
	SPI_INT_RX_OVERRUN = SSI_IM_RORIM,
};

struct lm4f_spi_divs {
	uint8_t cpsdvsr;
	uint8_t csr;
};

void spi_enable(uint32_t spi);
void spi_disable(uint32_t spi);
void spi_clock_from_piosc(uint32_t spi);
void spi_clock_from_sysclk(uint32_t spi);
void spi_init_master(uint32_t spi, uint8_t spi_mode, uint8_t data_bits,
		    uint32_t rate_hz);
void spi_configure(uint32_t spi, uint8_t spi_mode, uint8_t data_bits);
uint32_t spi_configure_clock_rate(uint32_t spi, uint32_t rate_hz);
uint32_t spi_find_divisors(struct lm4f_spi_divs *divs,
				  uint32_t target_clock, uint32_t source_clock);
void spi_configure_divisors(uint32_t spi, const struct lm4f_spi_divs *divs);
void spi_configure_frame_format(uint32_t spi, enum spi_format format);

void spi_write_fifo(uint32_t spi, uint16_t data);
uint16_t spi_read_fifo(uint32_t spi);
size_t spi_fill_fifo8(uint32_t spi, const uint8_t *buf, size_t len);
size_t spi_fill_fifo16(uint32_t spi, const uint16_t *buf, size_t len);
size_t spi_drain_fifo8(uint32_t spi, uint8_t *buf, size_t len);
size_t spi_drain_fifo16(uint32_t spi, uint16_t *buf, size_t len);

void spi_enable_interrupts(uint32_t spi, enum spi_interrupt_flag ints);
void spi_disable_interrupts(uint32_t spi, enum spi_interrupt_flag ints);
void spi_clear_interrupt_flag(uint32_t spi, enum spi_interrupt_flag ints);

void spi_enable_tx_dma(uint32_t spi);
void spi_disable_tx_dma(uint32_t spi);
void spi_enable_rx_dma(uint32_t spi);
void spi_disable_rx_dma(uint32_t spi);

static inline
bool spi_is_tx_fifo_full(uint32_t uart)
{
	return !(SSI_SR(uart) & SSI_SR_TNF);
}

static inline
bool spi_is_tx_fifo_empty(uint32_t uart)
{
	return !!(SSI_SR(uart) & SSI_SR_TFE);
}

static inline
bool spi_is_rx_fifo_full(uint32_t uart)
{
	return !!(SSI_SR(uart) & SSI_SR_RFF);
}

static inline
bool spi_is_rx_fifo_empty(uint32_t uart)
{
	return !(SSI_SR(uart) & SSI_SR_RNE);
}

END_DECLS

/**@}*/

#endif	/* LM4F_SSI_H */

