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


#include <ntddk.h>
#include <ntstrsafe.h>
#include <stddef.h>
#include "linux/clk.h"
#include "clk_imx8m.h"
#include "clk_imx8m_registers.h"
#include "linux/delay.h"
#include "linux/slab.h"
#include "linux/io.h"

/* #define IMX_CLK_TRACE */

#define CLK_PRINTK(...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, __VA_ARGS__)

#ifdef IMX_CLK_DEBUG
#define CLK_DEBUG(...) CLK_PRINTK(__VA_ARGS__)
#else
#define CLK_DEBUG(...)
#endif
#ifdef IMX_CLK_TRACE
#define CLK_TRACE(...) CLK_PRINTK(__VA_ARGS__)
#else
#define CLK_TRACE(...)
#endif


#define CHECK_SUCCESS(e) do { if ((e) != 0) { \
                               CLK_PRINTK("Display CLK: CHECK_SUCCESS failed.\n"); \
                               return -1; \
                           } \
                      } while (0);

#define IMX_PLL_LOCK_TIMEOUT_US  10000U

#define __raw_writel(val, addr)  writel(val,addr)
#define __raw_readl(addr)        readl(addr)


/* --------------------------- HW dependent functions --------------------------------*/


/**
 * Sets the PLL to given rate
 *
 * @param      clk    Pointer to the clock
 * @param      rate   rate to be set
 *
 * @return Rate that was set.
 */
static uint32_t set_rate_pll(imx_clk_t *clk, uint32_t rate)
{
    uint32_t reg_val;
    volatile uint8_t __iomem *reg_pll;
    int i = 0;
    const struct imx_pll_rate_table *rate_tbl = NULL;
    const struct imx_pll_rate_table *rate_tbl_tmp = NULL;

    NT_ASSERT(clk);
    NT_ASSERT(clk->clk_pll.pll_rates);

    /* Find PLL params in the table */
    rate_tbl_tmp = &clk->clk_pll.pll_rates[i];
    while (rate_tbl_tmp->pll_rate != 0) {
        if (rate == rate_tbl_tmp->pll_rate) {
            rate_tbl = &clk->clk_pll.pll_rates[i];
            break;
        }
        i++;
        rate_tbl_tmp = &clk->clk_pll.pll_rates[i];
    }
    if ((rate_tbl == NULL) || (rate_tbl->pll_rate == 0)) {
        return 0;
    }
    if (clk->clk_pll.rate_index == i) {
        /* Pll already set */
        return rate_tbl->pll_rate;
    }
    /* Add PLL block offset to CCM base */
    reg_pll = (volatile uint8_t __iomem *)(clk->clk_pll.reg + clk->clk_pll.phy_offset);
    /* Bypass the VIDEO PLL clock */
    reg_val = __raw_readl(reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
    reg_val |= IMX_CCM_ANALOG_GEN_CTRL_PLL_BYPASS_MASK;
    __raw_writel(reg_val, reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
    /* Enable reset */
    __raw_writel((reg_val & ~(IMX_CCM_ANALOG_GEN_CTRL_PLL_RST_MASK)), reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
    /* Configure VIDEO PLL */
    reg_val = IMX_CCM_ANALOG_FDIV_CTL0_PLL_MAIN_DIV(rate_tbl->mdiv) |
              IMX_CCM_ANALOG_FDIV_CTL0_PLL_PRE_DIV(rate_tbl->pdiv) |
              IMX_CCM_ANALOG_FDIV_CTL0_PLL_POST_DIV(rate_tbl->sdiv);
    __raw_writel(reg_val, reg_pll + IMX_CCM_ANALOG_FDIV_CTL0_OFFSET);
    if (clk->type == CLK_FRAC_PLL) {
        reg_val = IMX_CCM_ANALOG_FDIV_CTL1_PLL_DSM(rate_tbl->kdiv);
        __raw_writel(reg_val, reg_pll + IMX_CCM_ANALOG_FDIV_CTL1_OFFSET);
    }
    usleep_range(3, 4);
    /* Disable reset */
    reg_val = __raw_readl(reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
    reg_val |= IMX_CCM_ANALOG_GEN_CTRL_PLL_RST_MASK;
    __raw_writel(reg_val, reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
    /* Wait for PLL lock */
    uint32_t deadline = IMX_PLL_LOCK_TIMEOUT_US;
    while (((__raw_readl(reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET)) & IMX_CCM_ANALOG_GEN_CTRL_PLL_LOCK_MASK) == 0) {
        if (deadline == 0) {
            CLK_PRINTK("Display CLK: waiting for PLL_LOCK in register IMX_CCM_ANALOG_GEN_CTRL failed\n");
            return 0;
        }
        usleep_range(3, 4);
        deadline--;
    }
    /* Disable bypass */
    reg_val = __raw_readl(reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
    reg_val &= ~(IMX_CCM_ANALOG_GEN_CTRL_PLL_BYPASS_MASK);
    __raw_writel(reg_val, reg_pll + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
    clk->clk_pll.rate_index = i;
    clk->clk_pll.ref_rate_hz = rate_tbl->dsi_phy_ref_rate;

    return rate_tbl->pll_rate;
}

/**
 * Enable/disable clock gate.
 *
 * @param      clk    Pointer to the clock
 * @param      enable true = enable the clock
 *
 * @return 0 on success.
 */
static int set_gate(imx_clk_t *clk, bool enable)
{
    volatile uint32_t cg;

    NT_ASSERT(clk);

    if (enable) {
        if (clk->clk_gate.share_count != NULL) {
            (*clk->clk_gate.share_count)++;
            if (*clk->clk_gate.share_count > 1) {
                /* Shared clock gate is already enabled */
                return 0;
            }
        }
        cg = __raw_readl(clk->clk_gate.reg_base + clk->clk_gate.phy_offset);
        cg &= ~(clk->clk_gate.mask << clk->clk_gate.shift);
        cg |= (clk->clk_gate.val << clk->clk_gate.shift);
    } else {
        if (clk->clk_gate.share_count != NULL) {
            if  (*clk->clk_gate.share_count > 0) {
                (*clk->clk_gate.share_count)--;
                if (*clk->clk_gate.share_count > 0) {
                    /* Shared clock gate is still owned by other clock object. */
                    return 0;
                }
            }
        }
        cg = __raw_readl(clk->clk_gate.reg_base + clk->clk_gate.phy_offset);
        cg &= ~(clk->clk_gate.mask << clk->clk_gate.shift);
    }
    __raw_writel(cg, clk->clk_gate.reg_base + clk->clk_gate.phy_offset);
    return 0;
}

/**
 * Enable/disable composite clock slice.
 *
 * @param      clk    Pointer to the clock
 * @param      enable true = enable the clock
 *
 * @return 0 on success.
 */
static int composite_enable(imx_clk_t *clk, bool enable)
{
    volatile uint32_t cg;

    NT_ASSERT(clk);

    if (enable) {
        if (clk->clk_slice.share_count != NULL) {
            (*clk->clk_slice.share_count)++;
            if (*clk->clk_slice.share_count > 1) {
                /* Shared clock slice is already enabled */
                return 0;
            }
        }
        cg = __raw_readl(clk->clk_slice.base_reg + clk->clk_slice.phy_offset);
        cg |= IMX_CCM_TARGET_ROOT_ENABLE_MASK;
    } else {
        if (clk->clk_slice.share_count != NULL) {
            if  (*clk->clk_slice.share_count > 0) {
                (*clk->clk_slice.share_count)--;
                if (*clk->clk_slice.share_count > 0) {
                    /* Shared clock slice is still owned by other clock object. */
                    return 0;
                }
            }
        }
        cg = __raw_readl(clk->clk_slice.base_reg + clk->clk_slice.phy_offset);
        cg &= ~(IMX_CCM_TARGET_ROOT_ENABLE_MASK);
    }
    __raw_writel(cg, clk->clk_slice.base_reg + clk->clk_slice.phy_offset);

    return 0;
}

/**
 * Set parent clock of the composite clock slice (clock multiplexer).
 *
 * @param      clk    Pointer to the clock
 * @param      parent  Pointer to the parent clock
 *
 * @return 0 on success. -1 on fail.
 */
static int composite_set_parent(imx_clk_t *clk, imx_clk_t *parent)
{
    NT_ASSERT(clk);

    for (int i = 0; i < IMX_CLK_ROOT_MUX_MAX_PARENTS; i++) {
        struct clk_root_mux *parent_mux = &clk->clk_slice.parents[i];
        if (_stricmp(parent_mux->parent_clock->name, parent->name) == 0) {
            clk->parent = parent_mux->parent_clock;
            clk->clk_slice.parent_index = i;
            volatile uint32_t val = __raw_readl(clk->clk_slice.base_reg + clk->clk_slice.phy_offset);
            val &= ~(IMX_CCM_TARGET_ROOT_MUX_MASK);
            val |= IMX_CCM_TARGET_ROOT_MUX_VALUE(i);
            __raw_writel(val, clk->clk_slice.base_reg + clk->clk_slice.phy_offset);
            return 0;
        }
    }

    return -1;
}

/**
 * Sets the composite clock slice to given rate
 *
 * @param      clk    Pointer to the clock
 * @param      rate   rate to be set
 *
 * @return Rate that was set.
 */
static uint32_t composite_set_rate(imx_clk_t *clk, uint32_t rate)
{
    uint32_t parent_rate, div1, div2, rate_tmp, diff, diff_tmp, prediv, postdiv, rate_real;
    volatile uint32_t val;

    NT_ASSERT(clk);

    parent_rate = imx_clk_get_rate(clk->parent);
    diff = parent_rate;
    prediv = 1;
    postdiv = 1;
    rate_real = 0;
    /* slice_clk_out = parent_clk / prediv / postdiv */
    for (div1 = 1; div1 <= IMX_CCM_TARGET_ROOT_PRE_MAX_DIV; div1++) {
        for (div2 = 1; div2 <= IMX_CCM_TARGET_ROOT_POST_MAX_DIV; div2++) {
            rate_tmp = (parent_rate / div1) / div2;
            if (rate > rate_tmp) {
                diff_tmp = rate - rate_tmp;
            } else {
                diff_tmp = rate_tmp - rate;
            }
            if (diff_tmp < diff) {
                prediv = div1;
                postdiv = div2;
                diff = diff_tmp;
                rate_real = rate_tmp;
            }
        }
    }
    if (rate_real != 0) {
        clk->clk_slice.prediv = prediv;
        clk->clk_slice.postdiv = postdiv;
        val = __raw_readl(clk->clk_slice.base_reg + clk->clk_slice.phy_offset);
        val &= ~(IMX_CCM_TARGET_ROOT_PRE_PODF_MASK | IMX_CCM_TARGET_ROOT_POST_PODF_MASK);
        val |= IMX_CCM_TARGET_ROOT_PRE_PODF(prediv);
        val |= IMX_CCM_TARGET_ROOT_POST_PODF(postdiv);
        __raw_writel(val, clk->clk_slice.base_reg + clk->clk_slice.phy_offset);
    }
    CLK_DEBUG("composite_set_rate:%12s prediv=%d postdiv=%d clk=%d\n", clk->name, clk->clk_slice.prediv,
              clk->clk_slice.postdiv, rate_real);
    return rate_real;
}

/* --------------------------- HW independent functions --------------------------------*/

/**
 * Sets the clock rate of the clock
 *
 * If current clock node is not able to set rate,
 * this function sets the rate of its parent instead.
 *
 * @param      clk   Pointer to the clock
 * @param      rate  The rate to be set
 *
 * @return     Actual rate of the clock, 0 on error
 */
unsigned long imx_clk_set_rate(imx_clk_t *clk, unsigned long rate)
{
    unsigned long real_rate = 0;

    if (clk == NULL) {
        return 0;
    }
    /* TODO:add check for clock rate reconfiguration request for shared PLLs */
    if (imx_clk_get_rate(clk) == rate) {
        return rate;
    }

    CLK_TRACE("set_rate(%12s, %7d)\n", clk->name, rate);

    switch (clk->type) {
        case CLK_SLICE:
            real_rate = (unsigned long)composite_set_rate(clk, rate);
            if ((real_rate == 0) && (clk->parent->type == CLK_FRAC_PLL)) {
                /* Clock slice setting was not successfull. It may be because parent PLL
                   was not set, or was set to incompatible value.
                   Try to force set parent PLL first and then clock slice again */
                int i = 0;
                uint32_t pll_rate = 0, diff, best_pclk = 0;
                uint32_t best_diff = clk->parent->clk_pll.pll_rates[0].pclk_rate;
                while (clk->parent->clk_pll.pll_rates[i].pclk_rate != 0) {
                    /* Look for closest suported pixel clock (to rate) */
                    diff = abs(clk->parent->clk_pll.pll_rates[i].pclk_rate - rate);
                    if (diff < best_diff) {
                        best_diff = diff;
                        pll_rate = clk->parent->clk_pll.pll_rates[i].pll_rate;
                        best_pclk = clk->parent->clk_pll.pll_rates[i].pclk_rate;
                    }
                    i++;
                }
                if (pll_rate == 0) {
                    CLK_PRINTK("Display CLK: ERROR - Suitable PLL rate not found to support pixel clock %d.\n", rate);
                    return real_rate;
                } else {
                    if (best_pclk == rate) {
                        CLK_DEBUG("Display CLK: PLL rate found: pclk=%d pll rate=%d i=%d\n", rate, pll_rate, i);
                    } else {
                        CLK_PRINTK("Display CLK: WARNING - Exact PLL rate not found to support pixel clock %d. The closest pixel clock %d selected\n",
                            rate, best_pclk);
                    }
                }
                imx_clk_set_rate(clk->parent, pll_rate);
                real_rate = (unsigned long)composite_set_rate(clk, rate);
            }
            if (real_rate == 0) {
                CLK_DEBUG("Display CLK: composite_set_rate(%s) requested rate %d couldn't be set\n", clk->name, rate);
            }
            break;
        case CLK_GATE:
            /* Clock gate cannot set rate - try to set parent's rate instead */
            real_rate = imx_clk_set_rate(clk->parent, rate);
            break;
        case CLK_FRAC_PLL:
        case CLK_SYS_PLL:
            real_rate = (unsigned long)set_rate_pll(clk, rate);
            break;
        case CLK_FIXED:
            real_rate = (unsigned long)clk->clk_fixed.rate_hz;
            break;
        default:
            CLK_PRINTK("Display CLK: clk_set_rate(%s) unknown clock type\n", clk->name);
            break;
    }

    if (real_rate != rate) {
        CLK_DEBUG("Display CLK: clk_set_rate(%s) requested rate %d differs from the set rate %d\n", clk->name, rate, real_rate);
    }
    return real_rate;
}

/**
 * Gets the clock rate of clock
 *
 * If current clock node is not able to get rate,
 * this function gets the rate of its parent instead.
 *
 * @param      clk   Pointer to the clock
 *
 * @return     Clock rate of the clock, 0 on error
 */
unsigned long imx_clk_get_rate(imx_clk_t *clk)
{
    unsigned long rate = 0;

    if (clk == NULL) {
        return 0;
    }

    switch (clk->type) {
        case CLK_SLICE:
            if ((clk->clk_slice.prediv == 0) || (clk->clk_slice.postdiv == 0)) {
                rate = 0;
            } else {
                rate = (imx_clk_get_rate(clk->parent) / clk->clk_slice.prediv / clk->clk_slice.postdiv);
            }
            break;
        case CLK_GATE:
            /* Clock gate cannot have rate - try to get parent's rate instead */
            if (clk->parent->clk_get_rate) {
                rate = clk->parent->clk_get_rate(clk->parent);
            }
            break;
        case CLK_FRAC_PLL:
        case CLK_SYS_PLL:
            if (clk->clk_pll.rate_index >= 0) {
                rate = (unsigned long) clk->clk_pll.pll_rates[clk->clk_pll.rate_index].pll_rate;
            }
            break;
        case CLK_FIXED:
            rate = (unsigned long)clk->clk_fixed.rate_hz;
            break;
        default:
            CLK_PRINTK("Display CLK: clk_get_rate(%s) unknown clock type\n", clk->name);
            break;
    }

    CLK_TRACE("get_rate(%10s) = %7d\n", clk->name, rate);
    return rate;
}

/**
 * Gets the reference rate of the clock.
 * Returns either CLK_FIXED rate at the top of the clock tree, or PLL reference rate for PLL parents.
 *
 * If current clock node is not able to get rate,
 * this function gets the rate of its parent instead.
 *
 * @param      clk   Pointer to the clock
 *
 * @return     Clock rate of the clock, 0 on error
 */
unsigned long imx_clk_get_reference_rate(imx_clk_t *clk)
{
    unsigned long rate = 0;

    if (clk == NULL) {
        return 0;
    }

    switch (clk->type) {
    case CLK_SLICE:
    case CLK_GATE:
        rate = imx_clk_get_reference_rate(clk->parent);
        break;
    case CLK_FRAC_PLL:
    case CLK_SYS_PLL:
        rate = (unsigned long)clk->clk_pll.ref_rate_hz;
        break;
    case CLK_FIXED:
        rate = (unsigned long)clk->clk_fixed.rate_hz;
        break;
    default:
        CLK_PRINTK("Display CLK: clk_get_reference_rate(%s) unknown clock type\n", clk->name);
        break;
    }

    CLK_TRACE("get_reference_rate(%10s) = %7d\n", clk->name, rate);
    return rate;
}

/**
 * Get parent of the clock
 *
 * @param      clk   Pointer to the clock
 *
 * @return     parent of the clock, NULL on error
 */
imx_clk_t * imx_clk_get_parent(imx_clk_t *clk)
{
    imx_clk_t * parent = NULL;
    struct clk_root_mux *parent_mux;

    if (clk == NULL) {
        return 0;
    }

    switch (clk->type) {
    case CLK_SLICE:
        parent_mux = &clk->clk_slice.parents[clk->clk_slice.parent_index];
        parent = parent_mux->parent_clock;
        break;
    case CLK_GATE:
        parent = imx_clk_get_parent(clk->parent);
        break;
    case CLK_FRAC_PLL:
    case CLK_SYS_PLL:
    case CLK_FIXED:
        break;
    default:
        CLK_PRINTK("Display CLK: clk_get_parent(%s) unknown clock type\n", clk->name);
        break;
    }

    CLK_TRACE("get_parent(%12s, %12s)\n", clk->name, parent->name);
    return parent;
}

/**
 * Set parent of the clock
 *
 * @param      clk   Pointer to the clock
 * @param      parent pointer to the parent clock
 *
 * @return 0 on success. Returns -1 on failure.
 */
int imx_clk_set_parent(imx_clk_t *clk, imx_clk_t *parent)
{
    int err = -1;

    if (clk == NULL) {
        return 0;
    }
    CLK_TRACE("set_parent(%12s, %12s)\n", clk->name, parent->name);

    switch (clk->type) {
    case CLK_SLICE:
        err = composite_set_parent(clk, parent);
        break;
    case CLK_GATE:
        err = imx_clk_set_parent(clk->parent, parent);
        break;
    case CLK_FRAC_PLL:
    case CLK_SYS_PLL:
    case CLK_FIXED:
        break;
    default:
        CLK_PRINTK("Display CLK: clk_set_parent(%s) unknown clock type\n", clk->name);
        break;
    }
    return err;
}

/**
 * Enable the clock
 *
 * This function enables a clock gate. First, it recursively goes
 * through the clocking tree and enables all clock parents along
 * the way.
 *
 * @param      clk     Pointer to the clock
 * @param      enable  0 - disable clock, 1 - enable clock
 * @param      recursive 0 - call only for selected clock, 1 - call recursively also for all parent clocks
 *
 * @return     0 on success, otherwise error
 */
int imx_clk_enable(imx_clk_t *clk, bool enable, bool recursive)
{
    int err = -1;

    if (clk == NULL) {
        return 0;
    }
    CLK_TRACE("enable(%16s, %2d)\n", clk->name, (int)enable);

    if (recursive) {
        CHECK_SUCCESS(imx_clk_enable(clk->parent, enable, recursive));
    }

    switch (clk->type) {
        case CLK_SLICE:
            err = composite_enable(clk, enable);
            break;
        case CLK_GATE:
            err = set_gate(clk, enable);
            break;
        case CLK_FRAC_PLL:
        case CLK_SYS_PLL:
        case CLK_FIXED:
            err = 0;
            break;
        default:
            CLK_PRINTK("Display CLK: clk_enable(%s) unknown clock type\n", clk->name);
            break;
    }

    return err;
}

/**
 * Log registers settings and clock frequency for particular clock type.
 * Clock id is computed as the address difference of the clock item from the start of the clock tree
 *
 * @param  clk  The clock to be printed
 *
 * @return     0 on success
 */
int imx_clk_dump(imx_clk_t *clk)
{
    volatile uint32_t val;
#ifdef IMX_CLK_DEBUG
    uint32_t addr;
    char str[5];
#endif
    uint32_t freq = 0, out_freq;
    int err = 0;

    if (clk == NULL) {
        return 0;
    }

    switch (clk->type) {
        case CLK_SLICE:
#ifdef IMX_CLK_DEBUG
            val = __raw_readl(clk->clk_slice.base_reg + clk->clk_slice.phy_offset);
            if (clk->clk_slice.share_count != NULL) {
                RtlStringCbPrintfA(str, sizeof(str), "%d", *clk->clk_slice.share_count);
            } else {
                RtlStringCbPrintfA(str, sizeof(str), "NULL");
            }
            CLK_DEBUG("Display CLK: CLK_SLICE=%s base=0x%x off=0x%x enbl=%d mux=%d pred=%d postd=%d shared=%s\n",
                     clk->name, IMX_CCM_BASE, clk->clk_slice.phy_offset,
                     ((val & IMX_CCM_TARGET_ROOT_ENABLE_MASK) >> IMX_CCM_TARGET_ROOT_ENABLE_SHIFT),
                     ((val & IMX_CCM_TARGET_ROOT_MUX_MASK) >> IMX_CCM_TARGET_ROOT_MUX_SHIFT),
                     ((val & IMX_CCM_TARGET_ROOT_PRE_PODF_MASK) >> IMX_CCM_TARGET_ROOT_PRE_PODF_SHIFT),
                     ((val & IMX_CCM_TARGET_ROOT_POST_PODF_MASK) >> IMX_CCM_TARGET_ROOT_POST_PODF_SHIFT), str);
#endif
            freq = imx_clk_get_rate(clk->parent);
            if ((clk->clk_slice.prediv == 0) || (clk->clk_slice.postdiv == 0)) {
                out_freq = 0;
            } else {
                out_freq = freq / clk->clk_slice.prediv / clk->clk_slice.postdiv;
            }
            CLK_PRINTK("Display CLK: CLK_SLICE=%s parent_frequency=%d parent_name=%s div=%d out_frequency=%d\n",
                     clk->name, freq, clk->parent->name, (clk->clk_slice.prediv * clk->clk_slice.postdiv), out_freq);
            break;
        case CLK_GATE:
            val = __raw_readl(clk->clk_gate.reg_base + clk->clk_gate.phy_offset);
#ifdef IMX_CLK_DEBUG
            if (clk->clk_gate.share_count != NULL) {
                RtlStringCbPrintfA(str, sizeof(str), "%d", *clk->clk_gate.share_count);
            }
            else {
                RtlStringCbPrintfA(str, sizeof(str), "NULL");
            }
            CLK_DEBUG("Display CLK: CLK_GATE=%s base=0x%x off=0x%x reg value=0x%x shared=%s\n", clk->name, clk->clk_gate.phy_base,
                      clk->clk_gate.phy_offset, val, str);
#endif
            freq = imx_clk_get_rate(clk);
            CLK_PRINTK("Display CLK: CLK_GATE=%s frequency=%d gate=%d\n", clk->name, freq, (((val) >> clk->clk_gate.shift) & clk->clk_gate.mask));
            break;
        case CLK_FRAC_PLL:
#ifdef IMX_CLK_DEBUG
            addr = clk->clk_pll.phy_offset;
            val = __raw_readl(clk->clk_pll.reg + addr + IMX_CCM_ANALOG_GEN_CTRL_OFFSET);
            CLK_DEBUG("Display CLK: CLK_PLL=%s base=0x%x off=0x%x GEN_CTRL=0x%x\n", clk->name, IMX_CCM_ANALOG_BASE,
                     (addr + IMX_CCM_ANALOG_GEN_CTRL_OFFSET), val);
            val = __raw_readl(clk->clk_pll.reg + addr + IMX_CCM_ANALOG_FDIV_CTL0_OFFSET);
            CLK_DEBUG("Display CLK: CLK_PLL=%s base=0x%x off=0x%x FDIV_CTL0=0x%x main=%d pred=%d postd=%d\n",
                     clk->name, IMX_CCM_ANALOG_BASE, (addr + IMX_CCM_ANALOG_FDIV_CTL0_OFFSET), val,
                     ((val & IMX_CCM_ANALOG_FDIV_CTL0_PLL_MAIN_DIV_MASK) >> IMX_CCM_ANALOG_FDIV_CTL0_PLL_MAIN_DIV_SHIFT),
                     ((val & IMX_CCM_ANALOG_FDIV_CTL0_PLL_PRE_DIV_MASK) >> IMX_CCM_ANALOG_FDIV_CTL0_PLL_PRE_DIV_SHIFT),
                     ((val & IMX_CCM_ANALOG_FDIV_CTL0_PLL_POST_DIV_MASK) >> IMX_CCM_ANALOG_FDIV_CTL0_PLL_POST_DIV_SHIFT));
            val = __raw_readl(clk->clk_pll.reg + addr + IMX_CCM_ANALOG_FDIV_CTL1_OFFSET);
            CLK_DEBUG("Display CLK: CLK_PLL=%s base=0x%x off=0x%x FDIV_CTL1=0x%x\n", clk->name, IMX_CCM_ANALOG_BASE,
                     (addr + IMX_CCM_ANALOG_FDIV_CTL1_OFFSET), val);
#endif
            freq = imx_clk_get_rate(clk);
            CLK_PRINTK("Display CLK: CLK_PLL=%s frequency=%d\n", clk->name, freq);
            break;
        case CLK_FIXED:
            CLK_DEBUG("Display CLK: CLK_FIXED=%s frequency=%d\n", clk->name, clk->clk_fixed.rate_hz);
            break;
        case CLK_CUSTOM:
            if (clk->clk_get_rate) {
                freq = clk->clk_get_rate(clk);
                CLK_PRINTK("Display CLK: CLK_PLL=%s frequency=%d\n", clk->name, freq);
            }
            break;
        default:
            CLK_PRINTK("Display CLK: clk_dump(%s) unknown clock type\n", clk->name);
            err = -1;
            break;
    }

    return err;
}

/* --------------------------- Clock tree initialization functions --------------------------------*/

/**
 * Initalize pointers to functions.
 *
 * @param  clk         clock object to initialize pointers to functions
 */
static void imx_clk_func_init(imx_clk_t *clk)
{
    NT_ASSERT(clk);
    clk->clk_enable = &imx_clk_enable;
    clk->clk_get_rate = &imx_clk_get_rate;
    clk->clk_set_rate = &imx_clk_set_rate;
    clk->clk_set_parent = &imx_clk_set_parent;
    clk->clk_get_parent = &imx_clk_get_parent;
    clk->clk_get_reference_rate = &imx_clk_get_reference_rate;
    clk->flags = 0;
}

/**
 * Initalize clock structure for composite clock slice.
 *
 * @param  name        clock name
 * @param  parent      parent of the clock in the clock tree
 * @param  reg         register base address of the CCM module
 * @param  offset      offset of the control register for the clock slice from the CCM base
 * @param  parents     pointer to the table of possible parent clocks (mux options)
 * @param  share_count count variable for shared clock object
 *
 * @return clk structure for given name and parameters on success. NULL on failure.
 */
imx_clk_t * imx_clk_slice(const char *name, imx_clk_t *parent, void __iomem *reg, unsigned offset,
                                            struct clk_root_mux *parents, unsigned *share_count)
{
    imx_clk_t *clk = kmalloc(sizeof(imx_clk_t), __GFP_ZERO);

    if (!clk) {
        CLK_PRINTK("Display CLK: Could not allocate clock structure\n");
        return NULL;
    }

    clk->type = CLK_SLICE;
    clk->name = name;
    clk->clk_slice.base_reg = (uint8_t *)reg;
    clk->clk_slice.parents = parents;
    clk->clk_slice.parent_index = 0;
    clk->clk_slice.prediv = 0;
    clk->clk_slice.postdiv = 0;
    clk->parent = parent;
    clk->clk_slice.share_count = share_count;
    clk->clk_slice.phy_offset = offset;
    imx_clk_func_init(clk);

    return clk;
}

/**
 * Initalize clock structure for clock gated.
 *
 * @param  name        clock name
 * @param  parent      parent of the clock in the clock tree
 * @param  reg_base    register base address of whole clk gate block
 * @param  base_addr   physical base address of whole block (debug purpose)
 * @param  offset      offset of the control register for the clock gate
 * @param  shift       shift of the clock gate value befor witten to the register
 * @param  val         the clock gate value being shifted by "shift"
 * @param  mask        mask for clearing the clock gate bitfield
 * @param  share_count count variable for shared clock object
 *
 * @return clk structure for given name and parameters on success. NULL on failure.
 */
imx_clk_t * imx_clk_gate(const char *name, imx_clk_t *parent, void __iomem *reg_base,
                                           unsigned offset, unsigned base_addr,
                                           unsigned shift, unsigned val, unsigned mask, unsigned *share_count)
{
    imx_clk_t *clk = kmalloc(sizeof(imx_clk_t), __GFP_ZERO);

    if (!clk) {
        CLK_PRINTK("Display CLK: Could not allocate clock structure\n");
        return NULL;
    }

    clk->type = CLK_GATE;
    clk->name = name;
    clk->clk_gate.shift = shift;
    clk->clk_gate.val = val;
    clk->clk_gate.mask = mask;
    clk->clk_gate.share_count = share_count;
    clk->clk_gate.reg_base = (uint8_t *)reg_base;
    clk->clk_gate.phy_base = base_addr;
    clk->clk_gate.phy_offset = offset;
    clk->parent = parent;
    imx_clk_func_init(clk);

    return clk;
}

/**
 * Initalize clock structure for fractional PLL clock (audio, video PLL).
 *
 * @param  name       clock name
 * @param  parent     parent of the clock in the clock tree
 * @param  reg        register base address of the CCM
 * @param  offset      offset of the selected PLL block
 * @param  rate_table pointer to the rate table for the PLL
 *
 * @return clk structure for given name and parameters on success. NULL on failure.
 */
imx_clk_t * imx_clk_pll(const char *name, imx_clk_t *parent, void __iomem *reg, unsigned offset,
                                          const struct imx_pll_rate_table *rate_table)
{
    imx_clk_t *clk = kmalloc(sizeof(imx_clk_t), __GFP_ZERO);

    if (!clk) {
        CLK_PRINTK("Display CLK: Could not allocate clock structure\n");
        return NULL;
    }

    clk->type = CLK_FRAC_PLL;
    clk->name = name;
    clk->clk_pll.reg = (uint8_t *)reg;
    clk->clk_pll.pll_rates = rate_table;
    clk->clk_pll.rate_index = -1;
    clk->clk_pll.phy_offset = offset;
    clk->parent = parent;
    imx_clk_func_init(clk);

    return clk;
}

/**
 * Initalize clock structure for fixed clock.
 *
 * @param  name      clock name
 * @param  parent    parent of the clock in the clock tree
 * @param  rate      rate of the clock
 *
 * @return clk structure for given name on success. NULL on failure.
 */
imx_clk_t * imx_clk_fixed(const char *name, imx_clk_t *parent, uint32_t rate)
{
    imx_clk_t *clk = kmalloc(sizeof(imx_clk_t), __GFP_ZERO);

    if (!clk) {
        CLK_PRINTK("Display CLK: Could not allocate clock structure\n");
        return NULL;
    }

    clk->type = CLK_FIXED;
    clk->name = name;
    clk->parent = parent;
    clk->clk_fixed.rate_hz = rate;
    imx_clk_func_init(clk);
    return clk;
}

/**
 * De-initalize clock item.
 *
 * @param  clk        clock tree item to de-initialize
  */
void imx_clk_uninit(imx_clk_t *clk)
{
    if (clk) {
        kfree(clk);
    }
}
