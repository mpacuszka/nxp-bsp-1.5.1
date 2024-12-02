/*
 * Copyright 2022-2024 NXP
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

#include <svc/scfw.h>
#include <svc/ipc.h>
#include <svc/pm/pm_api.h>
#include "linux/clk.h"
#include "linux/dev_printk.h"
#include "linux/firmware/imx/ipc.h"
#include "clk_imx8q.h"
#include "linux/slab.h"
#include "linux/io.h"

#define CLK_INFO(...)   \
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, __VA_ARGS__)

#define CLK_GATE_SCU_LPCG_MASK		0x3
#define CLK_GATE_SCU_LPCG_HW_SEL	BIT(0)
#define CLK_GATE_SCU_LPCG_SW_SEL	BIT(1)

static int set_gate_lpcg_scu(const struct clk *clk, bool enable)
{
    volatile uint32_t reg, val;

    NT_ASSERT(clk);

    reg = readl(clk->clk_lpcg_scu.reg);
    reg &= ~(CLK_GATE_SCU_LPCG_MASK << clk->clk_lpcg_scu.bit_idx);

    if (enable) {
        val = CLK_GATE_SCU_LPCG_SW_SEL;
        if (clk->clk_lpcg_scu.hw_gate)
            val |= CLK_GATE_SCU_LPCG_HW_SEL;

        reg |= val << clk->clk_lpcg_scu.bit_idx;
    }

    writel(reg, clk->clk_lpcg_scu.reg);

    return 0;
}

static int set_gate_scu(const struct clk *clk, bool enable)
{
    NT_ASSERT(clk);

    sc_ipc_struct_t *ipc = ipc = imx_scu_get_handle();
    if (!ipc) {
        return -EINVAL;
    }

    sc_err_t err = sc_pm_clock_enable(ipc, clk->clk_scu.rsrc_id,
        clk->clk_scu.clk_type, enable, false);
    if (err != SC_ERR_NONE) {
        dev_err(NULL, "[clk: %s] set_gate_scu failed! (error = %d, %s)\n",
            clk->name, err, sc_err_to_string(err));
        return -EIO;
    }

    return 0;
}

static int set_rate_scu(const struct clk *clk, unsigned long rate)
{
    NT_ASSERT(clk);

    sc_ipc_struct_t *ipc = ipc = imx_scu_get_handle();
    if (!ipc) {
        return -EINVAL;
    }

    sc_err_t err = sc_pm_set_clock_rate(ipc, clk->clk_scu.rsrc_id,
        clk->clk_scu.clk_type, &(sc_pm_clock_rate_t)rate);
    if (err != SC_ERR_NONE) {
        dev_err(NULL, "[clk: %s] set_rate_scu() failed! (err = %d, %s)\n",
            clk->name, err, sc_err_to_string(err));
        return err;
    }

    return 0;
}

static unsigned long get_rate_scu(const struct clk *clk)
{
    NT_ASSERT(clk);

    sc_ipc_struct_t *ipc = ipc = imx_scu_get_handle();
    if (!ipc) {
        return 0;
    }

    sc_pm_clock_rate_t rate;
    sc_err_t err = sc_pm_get_clock_rate(ipc, clk->clk_scu.rsrc_id,
        clk->clk_scu.clk_type, &rate);
    if (err != SC_ERR_NONE) {
        dev_err(NULL, "[clk: %s] get_rate_scu() failed! (err = %d, %s)\n",
            clk->name, err, sc_err_to_string(err));
        return 0;
    }

    return rate;
}

static struct clk *get_parent_scu(const struct clk *clk)
{
    NT_ASSERT(clk);

    sc_ipc_struct_t *ipc = ipc = imx_scu_get_handle();
    if (!ipc) {
        return NULL;
    }

    sc_pm_clk_parent_t par_id;
    sc_err_t err = sc_pm_get_clock_parent(ipc, clk->clk_scu.rsrc_id,
        clk->clk_scu.clk_type, &par_id);
    if (err != SC_ERR_NONE) {
        dev_err(NULL, "[clk: %s] get_parent_scu() failed! (err = %d, %s)\n",
            clk->name, err, sc_err_to_string(err));
        return NULL;
    }

    if (par_id >= clk->clk_scu.num_parents) {
        return NULL;
    }

    return clk->clk_scu.parents[par_id];
}

static int set_parent_scu(const struct clk *clk, const struct clk *parent)
{
    NT_ASSERT(clk);

    uint8_t i;
    for (i = 0; i < clk->clk_scu.num_parents; i++) {
        if (!_stricmp(parent->name, clk->clk_scu.parents[i]->name)) {
            break;
        }
    }

    if (i == clk->clk_scu.num_parents) {
        return -EINVAL;
    }

    sc_ipc_struct_t *ipc = ipc = imx_scu_get_handle();
    if (!ipc) {
        return -EINVAL;
    }

    sc_err_t err = sc_pm_set_clock_parent(ipc, clk->clk_scu.rsrc_id,
            clk->clk_scu.clk_type, i);
    if (err != SC_ERR_NONE) {
        dev_err(NULL, "[clk: %s] set_parent_scu() failed! (err = %d, %s)\n",
            clk->name, err, sc_err_to_string(err));
        return -EINVAL;
    }

    return 0;
}

static int imx8q_clk_enable(struct clk *clk, bool enable, bool recursive)
{
    int err = -1;

    if (!clk) {
        return 0;
    }
    dev_info(NULL, "[clk: %s] enable(%2d)\n", clk->name, (int)enable);

    switch (clk->type) {
    case CLK_LPCG_SCU:
        err = set_gate_lpcg_scu(clk, enable);
        break;
    case CLK_SCU:
        err = set_gate_scu(clk, enable);
        break;
    default:
        dev_err(NULL, "[clk: %s] clk_enable() unknown clock type\n",
            clk->name);
        break;
    }

    return err;
}

static unsigned long imx8q_clk_get_rate(struct clk *clk)
{
    unsigned long rate = 0;

    if (!clk) {
        return 0;
    }

    switch (clk->type) {
    case CLK_LPCG_SCU:
        /* Clock gate cannot have rate - try to get parent's rate instead */
        rate = imx_clk_get_rate(clk->parent);
        break;
    case CLK_SCU:
        rate = get_rate_scu(clk);
        break;
    case CLK_FIXED:
        rate = (unsigned long)clk->clk_fixed.rate_hz;
        break;
    default:
        dev_err(NULL, "[clk: %s] clk_get_rate() unknown clock type\n",
            clk->name);
        break;
    }

    dev_info(NULL, "[clk: %s] get_rate() %7d\n", clk->name, rate);

    return rate;
}

static unsigned long imx8q_clk_set_rate(struct clk *clk, unsigned long rate)
{
    if (!clk) {
        return 0;
    }

    dev_info(NULL, "[clk: %s] set_rate(%7d)\n", clk->name, rate);

    unsigned long curr_rate = imx8q_clk_get_rate(clk);
    if (curr_rate == rate) {
        return rate;
    }

    int err = -1;
    switch (clk->type) {
        case CLK_SCU:
            err = set_rate_scu(clk, rate);
            break;
        case CLK_LPCG_SCU:
        case CLK_FIXED:
            break;
        default:
            dev_err(NULL, "[clk: %s] clk_set_rate() unknown clock type\n",
                clk->name);
            break;
    }

    curr_rate = imx8q_clk_get_rate(clk);
    if (curr_rate != rate) {
        dev_err(NULL,
            "[clk: %s] current rate differs from the requested one (%d vs %d)",
            clk->name, curr_rate, rate);
    }

    return err ? 0 : curr_rate;
}

static struct clk *imx8q_clk_get_parent(struct clk *clk)
{
    if (!clk) {
        return NULL;
    }

    struct clk *parent = NULL;

    switch (clk->type) {
    case CLK_SCU:
        parent = get_parent_scu(clk);
        break;
    case CLK_LPCG_SCU:
        parent = clk->parent;
    case CLK_FIXED:
        break;
    default:
        dev_err(NULL, "[clk: %s] clk_get_parent() unknown clock type\n",
            clk->name);
        break;
    }

    dev_info(NULL, "[clk: %s] get_parent(%12s)\n",
        clk->name, parent ? parent->name : "");

    return parent;
}

static int imx8q_clk_set_parent(struct clk *clk, struct clk *parent)
{
    int err = -1;

    if (!clk) {
        return 0;
    }

    dev_info(NULL, "[clk: %s] set_parent(%12s)\n", clk->name, parent->name);

    switch (clk->type) {
    case CLK_SCU:
        err = set_parent_scu(clk, parent);
        break;
    case CLK_LPCG_SCU:
    case CLK_FIXED:
        break;
    default:
        dev_err(NULL, "[clk: %s] clk_set_parent() unknown clock type\n",
            clk->name);
        break;
    }

    return err;
}

static unsigned long imx8q_clk_get_reference_rate(struct clk *clk)
{
    return 0;
}

int imx8q_clk_dump(const struct clk *clk)
{
    uint32_t val, reg, rate;

    if (!clk) {
        return 0;
    }

    switch (clk->type) {
        case CLK_LPCG_SCU:
            val = CLK_GATE_SCU_LPCG_SW_SEL << clk->clk_lpcg_scu.bit_idx;
            reg = readl(clk->clk_lpcg_scu.reg);
            CLK_INFO("[clk: %s] CLK_LPCG_SCU, enabled-%d\n",
                clk->name, (val & reg ? 1 : 0));
            break;
        case CLK_SCU:
            rate = get_rate_scu(clk);
            CLK_INFO("[clk: %s] CLK_SCU, rate-%d\n", clk->name, rate);
            break;
        case CLK_FIXED:
            CLK_INFO("[clk: %s] CLK_FIXED, rate-%d\n",
                clk->name, clk->clk_fixed.rate_hz);
            break;
        default:
            dev_err(NULL, "[clk: %s] clk_dump() unknown clock type\n",
                clk->name);
            break;
    }

    return 0;
}

/* --------------------------- Clock tree initialization functions --------------------------------*/

static void imx_clk_func_init(struct clk *clk)
{
    NT_ASSERT(clk);
    clk->clk_enable = &imx8q_clk_enable;
    clk->clk_get_rate = &imx8q_clk_get_rate;
    clk->clk_set_rate = &imx8q_clk_set_rate;
    clk->clk_get_parent = &imx8q_clk_get_parent;
    clk->clk_set_parent = &imx8q_clk_set_parent;
    clk->clk_get_reference_rate = &imx8q_clk_get_reference_rate;
    clk->flags |= CLK_SCFW_PRESENT;
}

struct clk *imx8q_clk_fixed(const char *name,
    struct clk *parent, uint32_t rate)
{
    struct clk *clk = kmalloc(sizeof(struct clk), __GFP_ZERO);

    if (!clk) {
        dev_err(NULL, "[clk: %s] could not allocate clock structure\n", name);
        return NULL;
    }

    clk->type = CLK_FIXED;
    clk->name = name;
    clk->parent = parent;
    clk->clk_fixed.rate_hz = rate;

    imx_clk_func_init(clk);

    return clk;
}

struct clk *imx_clk_lpcg_scu(const char *name, struct clk *parent,
    char *reg, u8 bit_idx, bool hw_gate)
{
    struct clk *clk = kmalloc(sizeof(struct clk), __GFP_ZERO);

    if (!clk) {
        dev_err(NULL, "[clk: %s] could not allocate clock structure\n", name);
        return NULL;
    }

    clk->type = CLK_LPCG_SCU;
    clk->name = name;
    clk->parent = parent;
    clk->clk_lpcg_scu.reg = reg;
    clk->clk_lpcg_scu.bit_idx = bit_idx;
    clk->clk_lpcg_scu.hw_gate = hw_gate;

    imx_clk_func_init(clk);

    return clk;
}

struct clk *__imx_clk_scu(const char *name, struct clk **parents,
    int num_parents, u16 rsrc_id, u8 clk_type)
{
    struct clk *clk = kmalloc(sizeof(struct clk), __GFP_ZERO);

    if (!clk) {
        dev_err(NULL, "[clk: %s] could not allocate clock structure\n", name);
        return NULL;
    }

    clk->name = name;
    clk->type = CLK_SCU;
    clk->clk_scu.parents = parents;
    clk->clk_scu.num_parents = (u8)num_parents;
    clk->clk_scu.rsrc_id = rsrc_id;
    clk->clk_scu.clk_type = clk_type;

    imx_clk_func_init(clk);

    return clk;
}

void imx8q_clk_deinit(struct clk *clk)
{
    if (clk) {
        kfree(clk);
    }
}
