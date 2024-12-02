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

#ifndef IMX_CLK_IMX8Q_H
#define IMX_CLK_IMX8Q_H

#include <svc/ipc.h>
#include "clk_imx8.h"
#include "linux/io.h"

struct imx_clk_lpcg_scu {
    char *reg;
    u8 bit_idx;
    bool hw_gate;
};

struct imx_clk_scu {
    u16 rsrc_id;
    u8 clk_type;

    struct clk **parents;
    u8 num_parents;
};

struct clk;

struct clk *imx8q_clk_fixed(const char *name, struct clk *parent, uint32_t rate);
struct clk *imx_clk_lpcg_scu(const char *name, struct clk *parent,
    char *reg, u8 bit_idx, bool hw_gate);
struct clk *__imx_clk_scu(const char *name, struct clk **parents,
    int num_parents, u16 rsrc_id, u8 clk_type);

static inline struct clk *imx_clk_scu(const char *name, u16 rsrc_id,
    u8 clk_type)
{
    return __imx_clk_scu(name, NULL, 0, rsrc_id, clk_type);
}

static inline struct clk *imx_clk_scu2(const char *name, struct clk **parents,
    int num_parents, u16 rsrc_id, u8 clk_type)
{
    return __imx_clk_scu(name, parents, num_parents, rsrc_id, clk_type);
}

void imx8q_clk_deinit(struct clk *clk);

int imx8q_clk_dump(const struct clk *clk);

#endif
