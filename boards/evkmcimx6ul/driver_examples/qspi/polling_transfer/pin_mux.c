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

    /* Initialize QSPI pins below */
    IOMUXC_SetPinMux(IOMUXC_NAND_ALE_QSPI_A_DQS, 0);
    IOMUXC_SetPinMux(IOMUXC_NAND_WP_B_QSPI_A_SCLK, 0);
    IOMUXC_SetPinMux(IOMUXC_NAND_READY_B_QSPI_A_DATA00, 0);
    IOMUXC_SetPinMux(IOMUXC_NAND_CE0_B_QSPI_A_DATA01, 0);
    IOMUXC_SetPinMux(IOMUXC_NAND_CE1_B_QSPI_A_DATA02, 0);
    IOMUXC_SetPinMux(IOMUXC_NAND_CLE_QSPI_A_DATA03, 0);
    IOMUXC_SetPinMux(IOMUXC_NAND_DQS_QSPI_A_SS0_B, 0);
    IOMUXC_SetPinMux(IOMUXC_NAND_DATA07_QSPI_A_SS1_B, 0);

    IOMUXC_SetPinConfig(IOMUXC_NAND_ALE_QSPI_A_DQS, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_NAND_WP_B_QSPI_A_SCLK, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_NAND_READY_B_QSPI_A_DATA00,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_NAND_CE0_B_QSPI_A_DATA01, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_NAND_CE1_B_QSPI_A_DATA02, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_NAND_CLE_QSPI_A_DATA03, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_NAND_DQS_QSPI_A_SS0_B, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
    IOMUXC_SetPinConfig(IOMUXC_NAND_DATA07_QSPI_A_SS1_B, IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(2));
}
