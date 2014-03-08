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

#include <libopencm3/lm4f/ssi.h>
#include <libopencm3/lm4f/systemcontrol.h>

/**
 * \brief Enable the SSI
 *
 * @param[in] ssi SSI block register address base @ref ssi_reg_base
 */
void ssi_enable(uint32_t ssi)
{
	SSI_CR1(ssi) |= SSI_CR1_SEE;
}

/**
 * \brief Enable the SSI
 *
 * @param[in] ssi SSI block register address base @ref ssi_reg_base
 */
void ssi_disable(uint32_t ssi)
{
	SSI_CR1(ssi) &= ~SSI_CR1_SEE;
}

/**
 * \brief Set Mode for SSI
 *
 * @param[in] ssi SSI block register address base @ref ssi_reg_base
 * @param[in] mode SSI mode to set
 */
void ssi_set_mode(uint32_t ssi, enum ssi_mode mode)
{
	switch(mode){
		case SSI_MODE_MASTER:
			return;
		case SSI_MODE_SLAVE_TX_ENABLED:
			SSI_CR1(ssi) |= SSI_CR1_MS;
			return;
		case SSI_MODE_SLAVE_TX_DISABLED:
			SSI_CR1(ssi) |= (SSI_CR1_MS | SSI_CR1_SOD);
			return;
		default:
			/* I should not end here! */
			return;
	}
}

/**
 * \brief Set frame format for SSI
 *
 * @param[in] ssi SSI block register address base @ref ssi_reg_base
 * @param[in] bits SSI databits
 * @param[in] phase SSI phase
 * @param[in] polarity SSI polarity
 * @param[in] mode SSI mode
 */
void ssi_set_frame_format(uint32_t ssi,
			uint8_t bits, uint8_t phase,
			uint8_t polarity, enum ssi_protocol protocol)
{
	uint32_t regTemp = 0;
	switch(protocol){
		case SSI_PROTOCOL_FREESCALE:
			// Here I need to set polarity and phase
			// not otherwise.
			regTemp |= (SSI_CR0_FRF_MASK & SSI_CR0_FRF_0);
			if(phase==1)
				regTemp|= SSI_CR0_SPH;
			if(polarity==1)
				regTemp|= SSI_CR0_SPO;
			break;
		case SSI_PROTOCOL_TI:
			regTemp |= (SSI_CR0_FRF_MASK & SSI_CR0_FRF_1);
			break;
		case SSI_PROTOCOL_MICROWIRE:
			regTemp |= (SSI_CR0_FRF_MASK & SSI_CR0_FRF_2);
			break;
		default:
			// I should not end here!
			return;
	}
	regTemp |= (SSI_CR0_DSS_MASK & (bits-1));
	uint32_t reg32 = SSI_CR0(ssi);
	reg32 |= regTemp;
	SSI_CR0(ssi)= reg32;
}

/**
 * \brief Set clock source for SSI
 *
 * @param[in] ssi SSI block register address base @ref ssi_reg_base
 * @param[in] source SSI clock source
 */
void ssi_set_clock_source(uint32_t ssi, enum ssi_clock_source source)
{
	uint32_t regTemp=0;
	switch(source){
		case SSI_CLOCK_SOURCE_SYSTEM:
			regTemp |= (SSI_CC_CS_MASK & SSI_CC_CS_0);
			break;
		case SSI_CLOCK_SOURCE_PIOSC:
			regTemp |= (SSI_CC_CS_MASK & SSI_CC_CS_5);
			break;
	}
	uint32_t reg32 = SSI_CC(ssi);
	reg32|= regTemp;
	SSI_CC(ssi) = reg32;
}

/**
 * \brief Set clock rate for SSI
 *
 * @param[in] ssi SSI block register address base @ref ssi_reg_base
 * @param[in] rate SSI clock rate
 */
void ssi_set_clock_rate(uint32_t ssi, uint8_t rate)
{
	uint32_t regTemp=0;

	uint32_t reg32 = SSI_CR0(ssi);
	reg32 |= regTemp;
	SSI_CR0(ssi)= reg32;
}
/**
 * \brief Set bit rate for SSI
 *
 * @param[in] ssi SSI block register address base @ref ssi_reg_base
 * @param[in] rate SSI clock rate
 * @param[in] divider SSI clock divider
 */
void ssi_set_bit_rate(uint32_t ssi, uint8_t divider, uint8_t rate)
{
}

/* TODO: implement the other stuff */
