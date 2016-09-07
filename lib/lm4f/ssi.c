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

/**
 * @defgroup ssi_file SSI
 *
 * @ingroup LM4Fxx
 *
 * @author @htmlonly &copy; @endhtmlonly 2016 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 *
 * \brief <b>libopencm3 LM4F Synchronous Serial Interface (SPI)</b>
 *
 * Texas Instruments datasheets refer to the peripheral as SSI, even though it
 * is, for all practical purposes, SPI. The terms SSI is used to refer to the
 * peripheral, while SPI is used to refer to the communication channel.
 *
 * Please see the individual UART modules for more details. To use the UART, the
 * uart.h header needs to be included:
 * @code{.c}
 *	#include <libopencm3/lm4f/ssi.h>
 * @endcode
 *
 * @{
 */

#include <libopencm3/lm4f/rcc.h>
#include <libopencm3/lm4f/ssi.h>
#include <stdlib.h>

#define ROUND_UP_DIV(x, y)	(((x) + (y) - 1) / (y))

/** @defgroup spi_config SPI configuration
 *
 * \brief <b>Enabling and configuring the SSI</b>
 *
 * The configuration API is modeled under the assumption that generic
 * configuration parameters, such as clock, polarity, data rate, and data bits
 * can be adjusted on-the-fly. On the other hand, TI-specific features such as
 * the frame format are assumed to not change, and thus are specified by
 * separate functions.
 *
 * This is designed to model a system where several SPI devices which need
 * different settings may be connected to the same SPI master.
 *
 * There are two ways to initialize the SSI peripheral. For one-time
 * configuration, the @ref spi_init_master() takes care of all the details,
 * including divisor calculations.
 *  -# The SSI clock must be enabled with @ref periph_clock_enable().
 *  -# The SSI clock should be selected from eigther PIOSC or system clock.
 *  -# @ref spi_init_master() takes care of all other initialization details.
 *
 * For example, to enable SSI2 in master mode, at 8 data bits and 1 MHz:
 * @code{.c}
 *	// Enable the SSI clock
 *	periph_clock_enable(RCC_SSI2);
 *	// Configure the SSI clock source as precision internal oscillator
 *	uart_clock_from_piosc();
 *	// Initialize SSI in SPI master mode 0, 8 data bits, at 1MHz bitrate
 *	spi_init_master(SSI2, SPI_MODE(0, 0), 8, 1000000);
 * @endcode
 *
 * # Fine-grained configuration
 *
 * On-the-fly configuration of the SSI can be achieved using more fine-grained
 * configuration functions. Note that unlike using @ref spi_init_master(), the
 * SSI must first be disabled with @ref spi_disable(). Once the configuration
 * changes are done, the peripheral may be restarted with @ref spi_enable().
 *
 * SPI communication parameters can be changed using @ref spi_configure(). This
 * is most useful for changing the SPI parameters between SPI slaves which need
 * different configurations.
 *
 * For example, to change SSI2 to SPI mode 3 and 7 data bits:
 * @code{.c}
 *	// Disable for configuration
 *	spi_disable(SSI2);
 *	// Change SSI2 configuration
 *	spi_configure(SSI2, SPI_MODE(1, 1), 7);
 *	// Re-enable the peripheral
 *	spi_enable(SSI2);
 * @endcode
 *
 * The bitrate of the SPI clock can be adjusted with
 * @ref spi_configure_clock_rate(). This calculates the optimal set of divisors
 * to achieve a bitrate closest to the target bitrate. In order to prevent
 * re-calculating the divisors, this function can be replaced by
 * @ref spi_find_divisors() and @ref spi_configure_divisors(). Thus divisors
 * only need to be calculated once for a given rate.
 *
 * In the following examples, and MPU9250 and a microchip fartmach are connected
 * to the same SPI bus, but each need different SPI settings.
 * @code{.c}
 *	struct lm4f_spi_divs divs_slave_mpu9250, divs_slave_microchip_fartmach;
 *	spi_find_divisors(&divs_slave_mpu9250, 1000000, 16000000);
 *	spi_find_divisors(&divs_slave_microchip_fartmach, 400000, 16000000);
 *	while (1) {
 *		// Get ready to talk to MPU9250
 *		spi_disable(SSI2);
 *		spi_configure_divisors(SSI2, &divs_slave_mpu9250);
 *		spi_configure(SSI2, SPI_MODE(0, 0), 8);
 *		spi_enable(SSI2);
 *		// Talk to MPU9250
 *		...
 *		// Get ready to talk to microchip fartmach
 *		spi_disable(SSI2);
 *		spi_configure_divisors(SSI2, &divs_slave_microchip_fartmach);
 *		spi_configure(SSI2, SPI_MODE(1, 1), 7);
 *		spi_enable(SSI2);
 *		// Talk to microchip fartmach
 *		...
 *	}
 * @endcode
 *
 * Most configuration formats assume standard SPI configuration. This is known
 * as the FREESCALE mode. The LM4F SSI supports other modes. In order to select
 * a different mode, use @ref spi_configure_frame_format() after configuring
 * the SSI. Note that @ref spi_init_master() and @ref spi_configure()
 * automatically select Freescale mode. To change from the default mode,
 * @ref spi_configure_frame_format() to be called after the SPI parameters are
 * changed.
 */
/**@{*/

void spi_enable(uint32_t spi)
{
	SSI_CR1(spi) |= SSI_CR1_SSE;
}

void spi_disable(uint32_t spi)
{
	SSI_CR1(spi) &= ~SSI_CR1_SSE;
}

void spi_clock_from_piosc(uint32_t spi)
{
	SSI_CC(spi) = SSI_CC_PIOSC;
}

void spi_clock_from_sysclk(uint32_t spi)
{
	SSI_CC(spi) = SSI_CC_SYSCLK;
}

void spi_init_master(uint32_t spi, uint8_t spi_mode, uint8_t data_bits,
		    uint32_t rate_hz)
{
	spi_disable(spi);

	spi_configure(spi, spi_mode, data_bits);
	spi_configure_clock_rate(spi, rate_hz);

	/* Enable master, and disable loopback mode. */
	SSI_CR1(SSI2) &= ~(SSI_CR1_MS | SSI_CR1_LBM);

	spi_enable(spi);
}

void spi_configure(uint32_t spi, uint8_t spi_mode, uint8_t data_bits)
{
	/* Keep clock divisor, but change everything else */
	uint32_t cr0 = SSI_CR0(spi) & (SSI_CR0_SCR_MASK);
	cr0 |= (spi_mode << 6) & (SSI_CR0_SPH | SSI_CR0_SPO);
	cr0 |= SSI_CR0_FRF_FREESCALE;
	cr0 |= SSI_CR0_DSS(data_bits - 1);
	SSI_CR0(spi) = cr0;
}

uint32_t spi_configure_clock_rate(uint32_t spi, uint32_t rate_hz)
{
	struct lm4f_spi_divs divs;
	uint32_t clock, actual_rate;

	/* Are we running off the internal clock or system clock? */
	if (SSI_CC(spi) == SSI_CC_PIOSC) {
		clock = 16000000;
	} else {
		clock = rcc_get_system_clock_frequency();
	}

	actual_rate = spi_find_divisors(&divs, rate_hz, clock);
	spi_configure_divisors(spi, &divs);

	return actual_rate;
}

uint32_t spi_find_divisors(struct lm4f_spi_divs *divs,
				  uint32_t target_clock, uint32_t source_clock)
{
	uint32_t actual_clock, last_error = ~0, current_error, prediv, postdiv;
	uint16_t best_post = 256;
	uint8_t best_pre = 254;

	/* Minimum pre-divisor, based on a maximum post-divider of 256. */
	prediv = ROUND_UP_DIV(source_clock, (target_clock * 256));
	prediv &= ~1;
	if (prediv > 254)
		prediv = 254;


	for ( ; prediv <= 254; prediv += 2) {
		postdiv = ROUND_UP_DIV(source_clock, prediv * target_clock);
		if (postdiv > 256)
			postdiv = 256;
		actual_clock = source_clock / (prediv * postdiv);
		current_error = abs(target_clock - actual_clock);

		if (current_error < last_error) {
			last_error = current_error;
			best_pre = prediv;
			best_post = postdiv;
		}

		/* Exact match ? */
		if (current_error == 0)
			break;

	}

	divs->cpsdvsr = best_pre;
	divs->csr = best_post - 1;

	return actual_clock;
}

void spi_configure_divisors(uint32_t spi, const struct lm4f_spi_divs *divs)
{
	uint32_t cr0;

	cr0 = SSI_CR0(spi) & ~SSI_CR0_SCR_MASK;
	cr0 |= SSI_CR0_SCR(divs->csr);
	SSI_CR0(spi) = cr0;

	SSI_CPSR(spi) = divs->cpsdvsr;

}

void spi_configure_frame_format(uint32_t spi, enum spi_format format)
{
	uint32_t cr0;

	cr0 = SSI_CR0(spi) & ~SSI_CR0_FRF_MASK;

	if (format == SSI_FORMAT_MICROWIRE)
		cr0 |= SSI_CR0_FRF_MICROWIRE;
	else if (format == SSI_FORMAT_TEXAS)
		cr0 |= SSI_CR0_FRF_TEXAS;
	else
		cr0 |= SSI_CR0_FRF_FREESCALE;

	SSI_CR0(spi) = cr0;
}

/**@}*/
/** @defgroup spi_xfer SPI transmission and reception
 *
 * \brief <b>Pocket guide on using the SSI to send SPI data</b>
 *
 * With the SSI enabled and configured, any write to the FIFO willl start a SPI
 * transfer. Once the transfer is complete, the data is available in the receive
 * FIFO. The FIFOs cannot be disabled or circumvented.
 *
 * The basic primitives for transceiving SPI data are @ref spi_write_fifo() and
 * @ref spi_read_fifo(). these primitives do not take into account the state of
 * the FIFO, so it possible to overwrite data.
 *
 * To model the more common scenario where scenario where a block of data is to
 * be transcieved @ref spi_fill_fifo8() and @ref spi_fill_fifo16() are provided.
 * They will write data until either the buffer is exhausted, or the FIFO is
 * full and cannot accept any more data. They return the number of bytes
 * currently transferred to the FIFO. Two versions are provided, depending on
 * whether the source buffer contains 8-bit or 16-bit values. The 16-bit
 * versions are useful for transfers where the SSI is conficured for word sizes
 * greater than 8 bits.
 * For recovering data from the FIFO @ref spi_fill_fifo8() and
 * @ref spi_fill_fifo16(), which are symmetrical to the transmit functions.
 */
/**@{*/

void spi_write_fifo(uint32_t spi, uint16_t data)
{
	SSI_DR(spi) = data;
}

uint16_t spi_read_fifo(uint32_t spi)
{
	return SSI_DR(spi);
}

size_t spi_fill_fifo8(uint32_t spi, const uint8_t *buf, size_t len)
{
	size_t num_bytes = 0;

	while(!spi_is_tx_fifo_full(spi) && len--) {
		spi_write_fifo(spi, buf[num_bytes++]);
	}
	return num_bytes;
}

size_t spi_fill_fifo16(uint32_t spi, const uint16_t *buf, size_t len)
{
	size_t num_bytes = 0;

	while(!spi_is_tx_fifo_full(spi) && len--) {
		spi_write_fifo(spi, buf[num_bytes++]);
	}
	return num_bytes;
}

size_t spi_drain_fifo8(uint32_t spi, uint8_t *buf, size_t len)
{
	size_t num_bytes = 0;

	while(!spi_is_rx_fifo_empty(spi) && len--) {
		buf[num_bytes++] = spi_read_fifo(spi);
	}
	return num_bytes;
}

size_t spi_drain_fifo16(uint32_t spi, uint16_t *buf, size_t len)
{
	size_t num_bytes = 0;

	while(!spi_is_rx_fifo_empty(spi) && len--) {
		buf[num_bytes++] = spi_read_fifo(spi);
	}
	return num_bytes;
}

/**@}*/

/** @defgroup ssi_irq SSI Interrupt control
 *
 * \brief <b>Configuring interrupts from the UART</b>
 *
 * To have an event generate an interrupt, its interrupt source must be
 * unmasked. This can be achieved with @ref spi_enable_interrupts(). Interrupts
 * which are no longer needed can be disabled through
 * @ref spi_disable_interrupts().
 *
 * In order for the interrupt to generate an IRQ and a call to the interrupt
 * service routine, the interrupt for the target SSI must be routed through the
 * NVIC with @ref nvic_enable_irq(). For this last step, the nvic.h header is
 * needed:
 * @code{.c}
 *	#include <libopencm3/lm4f/nvic.h>
 * @endcode
 *
 * Enabling an interrupt is as simple as unmasking the desired interrupt, and
 * routing the desired UART's interrupt through the NVIC.
 * @code{.c}
 *	// Unmask receive and overrun interrupts
 *	uart_enable_interrupt(SSI2, SPI_INT_RX | SPI_INT_RX_OVERRUN);
 *	// Make sure the interrupt is routed through the NVIC
 *	nvic_enable_irq(NVIC_SSI2_IRQ);
 * @endcode
 *
 * After interrupts are properly enabled and routed through the NVIC, when an
 * event occurs, the appropriate IRQ flag is set by hardware, and execution
 * jumps to the SSI ISR. The ISR should query the IRQ flags to determine which
 * event caused the interrupt. For this, use @ref ssi_is_interrupt_source(),
 * with the desired SSI_INT flag. After one or more interrupt sources are
 * serviced, the IRQ flags must be cleared by the ISR. This can be done with
 * @ref ssi_clear_interrupt_flag(). Note that the Rx timeout and the overrun
 * interrupts can be cleared thius way. The TX and TR interrupts are only
 * cleared when the cause is rectified, i.e. the TX FIFO is seeded to more
 * than half full, or the RX FIFO is drained to more than half empty. The TX and
 * RX interrupts should be masked if there is no more data to transcieve.
 *
 * A typical SSI ISR may look like the following:
 * @code{.c}
 * void ssi2_isr(void)
 * {
 *	size_t transferred;
 *	uint32_t serviced_irqs = 0;
 *
 *	// Process individual IRQs
 *	if (uart_is_interrupt_source(UART0, SSI_INT_TX)) {
 *		process_tx_event();
 *		if (mored_data_to_send()) {
 *			transferred = spi_fill_fifo8(SSI2, data, len);
 *			data += transferred;
 *			len -= transferred;
 *		} else {
 *			// Done. Disable TX IRQ to prevent interrupt storm.
 *			spi_disable_interrupts(SSI_INT_TX);
 *		}
 *
 *	}
 *	if (uart_is_interrupt_source(UART0, UART_TIMEOUT)) {
 *		process_timeout_event();
 *		serviced_irq |= SSI_TIMEOUT;
 *	}
 *
 *	// Clear the interrupt flag for the processed IRQs
 *	ssi_clear_interrupt_flag(SSI2, serviced_irqs);
 * }
 * @endcode
 */
/**@{*/

void spi_enable_interrupts(uint32_t spi, enum spi_interrupt_flag ints)
{
	SSI_IM(spi) |= ints;
}

void spi_disable_interrupts(uint32_t spi, enum spi_interrupt_flag ints)
{
	SSI_IM(spi) &= ~ints;
}

void spi_clear_interrupt_flag(uint32_t spi, enum spi_interrupt_flag ints)
{
	SSI_ICR(spi) |= ints;
}

/**@}*/
/**
 * @}
 */

