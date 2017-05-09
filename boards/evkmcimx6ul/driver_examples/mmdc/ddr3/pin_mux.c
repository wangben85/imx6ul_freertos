/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Function Name : BOARD_InitPins */
void BOARD_InitPins(void)
{
    /* Initialize UART1 pins below */
    IOMUXC_SetPinMux(IOMUXC_UART1_TX_DATA_UART1_TX, 0);
    IOMUXC_SetPinMux(IOMUXC_UART1_RX_DATA_UART1_RX, 0);
    IOMUXC_SetPinConfig(IOMUXC_UART1_TX_DATA_UART1_TX, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_UART1_RX_DATA_UART1_RX, IOMUXC_SW_PAD_CTL_PAD_SPEED(2));

    /* Initialize MMDC pins below */
    IOMUXC_SetPinConfig(IOMUXC_GRP_DDR_TYPE, IOMUXC_SW_PAD_CTL_GRP_DDR_SEL(3));
    IOMUXC_SetPinConfig(IOMUXC_GRP_DDRPKE, 0);
    IOMUXC_SetPinConfig(IOMUXC_DRAM_SDCLK0_P, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_CAS_B, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_RAS_B, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_GRP_ADDDS, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_RESET, IOMUXC_SW_PAD_CTL_GRP_DSE(6) | IOMUXC_SW_PAD_CTL_GRP_DDR_SEL(3));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_SDBA2, 0);
    IOMUXC_SetPinConfig(IOMUXC_DRAM_ODT0, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_ODT1, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_GRP_CTLDS, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_GRP_DDRMODE_CTL, IOMUXC_SW_PAD_CTL_GRP_DDR_INPUT(1));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_SDQS0_P, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_SDQS1_P, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_GRP_DDRMODE, IOMUXC_SW_PAD_CTL_GRP_DDR_INPUT(1));
    IOMUXC_SetPinConfig(IOMUXC_GRP_B0DS, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_GRP_B1DS, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_DQM0, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
    IOMUXC_SetPinConfig(IOMUXC_DRAM_DQM1, IOMUXC_SW_PAD_CTL_GRP_DSE(6));
}
