/*
 * Copyright 2022-2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
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
 *
 */

#include <linux\irqdesc.h>
#include <linux\clk-provider.h>

#define NR_IRQS 49

extern struct irq_desc irq_desc[NR_IRQS];

struct platform_device;

void board_init(struct platform_device* pdev);
void board_deinit(struct platform_device* pdev);
int _platform_irq_count(struct platform_device* pdev);

void mq_board_init(struct platform_device* pdev);
void mq_board_deinit(struct platform_device* pdev);
void qxp_board_init(struct platform_device* pdev);
void qxp_board_deinit(struct platform_device* pdev);
int qxp_irq_count();
void prg_res_init(struct platform_device* pdev, int cnt);
void dprc_res_init(struct platform_device* pdev, int cnt);
void ldb_prop_init(unsigned int ldb_index, unsigned int data_width, const char *bus_mapping);

#define IRQ_DESC_VBLANK  0
#define IRQ_DESC_HDMI_TX 1
#define IRQ_DESC_VBLANK_LCDIF1 2
void mp_board_init(struct platform_device* pdev);
void mp_board_deinit(struct platform_device* pdev);

void mn_board_init(struct platform_device* pdev);
void mn_board_deinit(struct platform_device* pdev);
