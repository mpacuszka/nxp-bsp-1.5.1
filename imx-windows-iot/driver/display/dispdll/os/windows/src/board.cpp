/*
 * Copyright 2022 NXP
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

extern "C" {

#include "boot\dts\freescale\board.h"
#include "linux\platform_device.h"
#include <string.h>

struct irq_desc irq_desc[NR_IRQS];

void board_init(struct platform_device* pdev)
{
    if (!pdev)
    {
        return;
    }

    if (!strcmp(pdev->plat_name, "mq")) {
        return mq_board_init(pdev);
    }
    if (!strcmp(pdev->plat_name, "mp")) {
        return mp_board_init(pdev);
    }
    if (!strcmp(pdev->plat_name, "mn")) {
        return mn_board_init(pdev);
    }
    if (!strcmp(pdev->plat_name, "qxp")) {
        return qxp_board_init(pdev);
    }
}

void board_deinit(struct platform_device* pdev)
{
    if (!pdev)
    {
        return;
    }

    if (!strcmp(pdev->plat_name, "mq")) {
        return mq_board_deinit(pdev);
    }
    if (!strcmp(pdev->plat_name, "mp")) {
        return mp_board_deinit(pdev);
    }
    if (!strcmp(pdev->plat_name, "mn")) {
        return mn_board_deinit(pdev);
    }
    if (!strcmp(pdev->plat_name, "qxp")) {
        return qxp_board_deinit(pdev);
    }
}

int _platform_irq_count(struct platform_device* pdev)
{
    if (!pdev)
    {
        return -1;
    }

    if (!strcmp(pdev->plat_name, "qxp")) {
        return qxp_irq_count();
    }

    return -1;
}

} // extern "C"
