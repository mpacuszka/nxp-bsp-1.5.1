/*
 * Copyright 2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef    _GPIO_IO_MAP_H_
#define    _GPIO_IO_MAP_H_

#define IO_MAP_ASSERT_CONCAT_(a, b) a##b
#define IO_MAP_ASSERT_CONCAT(a, b) IO_MAP_ASSERT_CONCAT_(a, b)
#define IO_MAP_STATIC_ASSERT(e) enum { IO_MAP_ASSERT_CONCAT(static_assert_, __LINE__) = 1/(int)(!!(e)) }


/*
 * GPIO registers definition
 */

#define GPIO_DR                                         0x00U
#define GPIO_GDIR                                       0x04U
#define GPIO_PSR                                        0x08U
#define GPIO_ICR1                                       0x0CU
#define GPIO_ICR2                                       0x10U
#define GPIO_IMR                                        0x14U
#define GPIO_IMS                                        0x18U
#define GPIO_EDGE_SEL                                   0x1CU
#define GPIO_DR_SET                                     0x84U
#define GPIO_DR_CLEAR                                   0x88U
#define GPIO_DR_TOGGLE                                  0x8CU


/*
 * GPIO_ICR1 register bits *
 */
#define GPIO_ICR1_ICR15_MASK                            0xC0000000U
#define GPIO_ICR1_ICR15_SHIFT                           30U
#define GPIO_ICR1_ICR14_MASK                            0x30000000U
#define GPIO_ICR1_ICR14_SHIFT                           28U
#define GPIO_ICR1_ICR13_MASK                            0x0C000000U
#define GPIO_ICR1_ICR13_SHIFT                           26U
#define GPIO_ICR1_ICR12_MASK                            0x03000000U
#define GPIO_ICR1_ICR12_SHIFT                           24U
#define GPIO_ICR1_ICR11_MASK                            0x00C00000U
#define GPIO_ICR1_ICR11_SHIFT                           22U
#define GPIO_ICR1_ICR10_MASK                            0x00300000U
#define GPIO_ICR1_ICR10_SHIFT                           20U
#define GPIO_ICR1_ICR9_MASK                             0x000C0000U
#define GPIO_ICR1_ICR9_SHIFT                            18U
#define GPIO_ICR1_ICR8_MASK                             0x00030000U
#define GPIO_ICR1_ICR8_SHIFT                            16U
#define GPIO_ICR1_ICR7_MASK                             0x0000C000U
#define GPIO_ICR1_ICR7_SHIFT                            14U
#define GPIO_ICR1_ICR6_MASK                             0x00003000U
#define GPIO_ICR1_ICR6_SHIFT                            12U
#define GPIO_ICR1_ICR5_MASK                             0x00000C00U
#define GPIO_ICR1_ICR5_SHIFT                            10U
#define GPIO_ICR1_ICR4_MASK                             0x00000300U
#define GPIO_ICR1_ICR4_SHIFT                            8U
#define GPIO_ICR1_ICR3_MASK                             0x000000C0U
#define GPIO_ICR1_ICR3_SHIFT                            6U
#define GPIO_ICR1_ICR2_MASK                             0x00000030U
#define GPIO_ICR1_ICR2_SHIFT                            4U
#define GPIO_ICR1_ICR1_MASK                             0x0000000CU
#define GPIO_ICR1_ICR1_SHIFT                            2U
#define GPIO_ICR1_ICR0_MASK                             0x00000003U
#define GPIO_ICR1_ICR0_SHIFT                            0U

/*
 * GPIO_ICR2 register bits *
 */
#define GPIO_ICR2_ICR31_MASK                            0xC0000000U
#define GPIO_ICR2_ICR31_SHIFT                           30U
#define GPIO_ICR2_ICR30_MASK                            0x30000000U
#define GPIO_ICR2_ICR30_SHIFT                           28U
#define GPIO_ICR2_ICR29_MASK                            0x0C000000U
#define GPIO_ICR2_ICR29_SHIFT                           26U
#define GPIO_ICR2_ICR28_MASK                            0x03000000U
#define GPIO_ICR2_ICR28_SHIFT                           24U
#define GPIO_ICR2_ICR27_MASK                            0x00C00000U
#define GPIO_ICR2_ICR27_SHIFT                           22U
#define GPIO_ICR2_ICR26_MASK                            0x00300000U
#define GPIO_ICR2_ICR26_SHIFT                           20U
#define GPIO_ICR2_ICR25_MASK                            0x000C0000U
#define GPIO_ICR2_ICR25_SHIFT                           18U
#define GPIO_ICR2_ICR24_MASK                            0x00030000U
#define GPIO_ICR2_ICR24_SHIFT                           16U
#define GPIO_ICR2_ICR23_MASK                            0x0000C000U
#define GPIO_ICR2_ICR23_SHIFT                           14U
#define GPIO_ICR2_ICR22_MASK                            0x00003000U
#define GPIO_ICR2_ICR22_SHIFT                           12U
#define GPIO_ICR2_ICR21_MASK                            0x00000C00U
#define GPIO_ICR2_ICR21_SHIFT                           10U
#define GPIO_ICR2_ICR20_MASK                            0x00000300U
#define GPIO_ICR2_ICR20_SHIFT                           8U
#define GPIO_ICR2_ICR19_MASK                            0x000000C0U
#define GPIO_ICR2_ICR19_SHIFT                           6U
#define GPIO_ICR2_ICR18_MASK                            0x00000030U
#define GPIO_ICR2_ICR18_SHIFT                           4U
#define GPIO_ICR2_ICR17_MASK                            0x0000000CU
#define GPIO_ICR2_ICR17_SHIFT                           2U
#define GPIO_ICR2_ICR16_MASK                            0x00000003U
#define GPIO_ICR2_ICR16_SHIFT                           0U


/*
 * GPIO_DR register bits definition *
 */
typedef union GPIO_DR_union_t {
    UINT32 R;
    struct {
        UINT32 DV0   :  1;  /* Data value 0. */
        UINT32 DV1   :  1;  /* Data value 1. */
        UINT32 DV2   :  1;  /* Data value 2. */
        UINT32 DV3   :  1;  /* Data value 3. */
        UINT32 DV4   :  1;  /* Data value 4. */
        UINT32 DV5   :  1;  /* Data value 5. */
        UINT32 DV6   :  1;  /* Data value 6. */
        UINT32 DV7   :  1;  /* Data value 7. */
        UINT32 DV8   :  1;  /* Data value 8. */
        UINT32 DV9   :  1;  /* Data value 9. */
        UINT32 DV10  :  1;  /* Data value 10. */
        UINT32 DV11  :  1;  /* Data value 11. */
        UINT32 DV12  :  1;  /* Data value 12. */
        UINT32 DV13  :  1;  /* Data value 13. */
        UINT32 DV14  :  1;  /* Data value 14. */
        UINT32 DV15  :  1;  /* Data value 15. */
        UINT32 DV16  :  1;  /* Data value 16. */
        UINT32 DV17  :  1;  /* Data value 17. */
        UINT32 DV18  :  1;  /* Data value 18. */
        UINT32 DV19  :  1;  /* Data value 19. */
        UINT32 DV20  :  1;  /* Data value 20. */
        UINT32 DV21  :  1;  /* Data value 21. */
        UINT32 DV22  :  1;  /* Data value 22. */
        UINT32 DV23  :  1;  /* Data value 23. */
        UINT32 DV24  :  1;  /* Data value 24. */
        UINT32 DV25  :  1;  /* Data value 25. */
        UINT32 DV26  :  1;  /* Data value 26. */
        UINT32 DV27  :  1;  /* Data value 27. */
        UINT32 DV28  :  1;  /* Data value 28. */
        UINT32 DV29  :  1;  /* Data value 29. */
        UINT32 DV30  :  1;  /* Data value 30. */
        UINT32 DV31  :  1;  /* Data value 31. */
    } B;
} GPIO_DR_t;

/*
 * GPIO_GDIR register bits definition *
 */
typedef union GPIO_GDIR_union_t {
    UINT32 R;
    struct {
        UINT32 DIR0   :  1;  /* Direction 0. */
        UINT32 DIR1   :  1;  /* Direction 1. */
        UINT32 DIR2   :  1;  /* Direction 2. */
        UINT32 DIR3   :  1;  /* Direction 3. */
        UINT32 DIR4   :  1;  /* Direction 4. */
        UINT32 DIR5   :  1;  /* Direction 5. */
        UINT32 DIR6   :  1;  /* Direction 6. */
        UINT32 DIR7   :  1;  /* Direction 7. */
        UINT32 DIR8   :  1;  /* Direction 8. */
        UINT32 DIR9   :  1;  /* Direction 9. */
        UINT32 DIR10  :  1;  /* Direction 10. */
        UINT32 DIR11  :  1;  /* Direction 11. */
        UINT32 DIR12  :  1;  /* Direction 12. */
        UINT32 DIR13  :  1;  /* Direction 13. */
        UINT32 DIR14  :  1;  /* Direction 14. */
        UINT32 DIR15  :  1;  /* Direction 15. */
        UINT32 DIR16  :  1;  /* Direction 16. */
        UINT32 DIR17  :  1;  /* Direction 17. */
        UINT32 DIR18  :  1;  /* Direction 18. */
        UINT32 DIR19  :  1;  /* Direction 19. */
        UINT32 DIR20  :  1;  /* Direction 20. */
        UINT32 DIR21  :  1;  /* Direction 21. */
        UINT32 DIR22  :  1;  /* Direction 22. */
        UINT32 DIR23  :  1;  /* Direction 23. */
        UINT32 DIR24  :  1;  /* Direction 24. */
        UINT32 DIR25  :  1;  /* Direction 25. */
        UINT32 DIR26  :  1;  /* Direction 26. */
        UINT32 DIR27  :  1;  /* Direction 27. */
        UINT32 DIR28  :  1;  /* Direction 28. */
        UINT32 DIR29  :  1;  /* Direction 29. */
        UINT32 DIR30  :  1;  /* Direction 30. */
        UINT32 DIR31  :  1;  /* Direction 31. */
    } B;
} GPIO_GDIR_t;

/*
 * GPIO_PSR register bits definition *
 */
typedef union GPIO_PSR_union_t {
    UINT32 R;
    struct {
        UINT32 PS0   :  1;  /* GPIO pad status 0. */
        UINT32 PS1   :  1;  /* GPIO pad status 1. */
        UINT32 PS2   :  1;  /* GPIO pad status 2. */
        UINT32 PS3   :  1;  /* GPIO pad status 3. */
        UINT32 PS4   :  1;  /* GPIO pad status 4. */
        UINT32 PS5   :  1;  /* GPIO pad status 5. */
        UINT32 PS6   :  1;  /* GPIO pad status 6. */
        UINT32 PS7   :  1;  /* GPIO pad status 7. */
        UINT32 PS8   :  1;  /* GPIO pad status 8. */
        UINT32 PS9   :  1;  /* GPIO pad status 9. */
        UINT32 PS10  :  1;  /* GPIO pad status 10. */
        UINT32 PS11  :  1;  /* GPIO pad status 11. */
        UINT32 PS12  :  1;  /* GPIO pad status 12. */
        UINT32 PS13  :  1;  /* GPIO pad status 13. */
        UINT32 PS14  :  1;  /* GPIO pad status 14. */
        UINT32 PS15  :  1;  /* GPIO pad status 15. */
        UINT32 PS16  :  1;  /* GPIO pad status 16. */
        UINT32 PS17  :  1;  /* GPIO pad status 17. */
        UINT32 PS18  :  1;  /* GPIO pad status 18. */
        UINT32 PS19  :  1;  /* GPIO pad status 19. */
        UINT32 PS20  :  1;  /* GPIO pad status 20. */
        UINT32 PS21  :  1;  /* GPIO pad status 21. */
        UINT32 PS22  :  1;  /* GPIO pad status 22. */
        UINT32 PS23  :  1;  /* GPIO pad status 23. */
        UINT32 PS24  :  1;  /* GPIO pad status 24. */
        UINT32 PS25  :  1;  /* GPIO pad status 25. */
        UINT32 PS26  :  1;  /* GPIO pad status 26. */
        UINT32 PS27  :  1;  /* GPIO pad status 27. */
        UINT32 PS28  :  1;  /* GPIO pad status 28. */
        UINT32 PS29  :  1;  /* GPIO pad status 29. */
        UINT32 PS30  :  1;  /* GPIO pad status 30. */
        UINT32 PS31  :  1;  /* GPIO pad status 31. */
    } B;
} GPIO_PSR_t;

/*
 * GPIO_ICR1 register bits definition *
 */
typedef union GPIO_ICR1_union_t {
    UINT32 R;
    struct {
        UINT32 ICR0   :  2;  /* Interrupt configuration field. */
        UINT32 ICR1   :  2;  /* Interrupt configuration field. */
        UINT32 ICR2   :  2;  /* Interrupt configuration field. */
        UINT32 ICR3   :  2;  /* Interrupt configuration field. */
        UINT32 ICR4   :  2;  /* Interrupt configuration field. */
        UINT32 ICR5   :  2;  /* Interrupt configuration field. */
        UINT32 ICR6   :  2;  /* Interrupt configuration field. */
        UINT32 ICR7   :  2;  /* Interrupt configuration field. */
        UINT32 ICR8   :  2;  /* Interrupt configuration field. */
        UINT32 ICR9   :  2;  /* Interrupt configuration field. */
        UINT32 ICR10  :  2;  /* Interrupt configuration field. */
        UINT32 ICR11  :  2;  /* Interrupt configuration field. */
        UINT32 ICR12  :  2;  /* Interrupt configuration field. */
        UINT32 ICR13  :  2;  /* Interrupt configuration field. */
        UINT32 ICR14  :  2;  /* Interrupt configuration field. */
        UINT32 ICR15  :  2;  /* Interrupt configuration field. */
    } B;
} GPIO_ICR1_t;

/*
 * GPIO_ICR2 register bits definition *
 */
typedef union GPIO_ICR2_union_t {
    UINT32 R;
    struct {
        UINT32 ICR06  :  2;  /* Interrupt configuration field. */
        UINT32 ICR17  :  2;  /* Interrupt configuration field. */
        UINT32 ICR18  :  2;  /* Interrupt configuration field. */
        UINT32 ICR19  :  2;  /* Interrupt configuration field. */
        UINT32 ICR20  :  2;  /* Interrupt configuration field. */
        UINT32 ICR21  :  2;  /* Interrupt configuration field. */
        UINT32 ICR22  :  2;  /* Interrupt configuration field. */
        UINT32 ICR23  :  2;  /* Interrupt configuration field. */
        UINT32 ICR24  :  2;  /* Interrupt configuration field. */
        UINT32 ICR25  :  2;  /* Interrupt configuration field. */
        UINT32 ICR26  :  2;  /* Interrupt configuration field. */
        UINT32 ICR27  :  2;  /* Interrupt configuration field. */
        UINT32 ICR28  :  2;  /* Interrupt configuration field. */
        UINT32 ICR29  :  2;  /* Interrupt configuration field. */
        UINT32 ICR30  :  2;  /* Interrupt configuration field. */
        UINT32 ICR31  :  2;  /* Interrupt configuration field. */
    } B;
} GPIO_ICR2_t;

/*
 * GPIO_IMR register bits definition *
 */
typedef union GPIO_IMR_union_t {
    UINT32 R;
    struct {
        UINT32 IM0   :  1;  /* Interrupt mask 0. */
        UINT32 IM1   :  1;  /* Interrupt mask 1. */
        UINT32 IM2   :  1;  /* Interrupt mask 2. */
        UINT32 IM3   :  1;  /* Interrupt mask 3. */
        UINT32 IM4   :  1;  /* Interrupt mask 4. */
        UINT32 IM5   :  1;  /* Interrupt mask 5. */
        UINT32 IM6   :  1;  /* Interrupt mask 6. */
        UINT32 IM7   :  1;  /* Interrupt mask 7. */
        UINT32 IM8   :  1;  /* Interrupt mask 8. */
        UINT32 IM9   :  1;  /* Interrupt mask 9. */
        UINT32 IM10  :  1;  /* Interrupt mask 10. */
        UINT32 IM11  :  1;  /* Interrupt mask 11. */
        UINT32 IM12  :  1;  /* Interrupt mask 12. */
        UINT32 IM13  :  1;  /* Interrupt mask 13. */
        UINT32 IM14  :  1;  /* Interrupt mask 14. */
        UINT32 IM15  :  1;  /* Interrupt mask 15. */
        UINT32 IM16  :  1;  /* Interrupt mask 16. */
        UINT32 IM17  :  1;  /* Interrupt mask 17. */
        UINT32 IM18  :  1;  /* Interrupt mask 18. */
        UINT32 IM19  :  1;  /* Interrupt mask 19. */
        UINT32 IM20  :  1;  /* Interrupt mask 20. */
        UINT32 IM21  :  1;  /* Interrupt mask 21. */
        UINT32 IM22  :  1;  /* Interrupt mask 22. */
        UINT32 IM23  :  1;  /* Interrupt mask 23. */
        UINT32 IM24  :  1;  /* Interrupt mask 24. */
        UINT32 IM25  :  1;  /* Interrupt mask 25. */
        UINT32 IM26  :  1;  /* Interrupt mask 26. */
        UINT32 IM27  :  1;  /* Interrupt mask 27. */
        UINT32 IM28  :  1;  /* Interrupt mask 28. */
        UINT32 IM29  :  1;  /* Interrupt mask 29. */
        UINT32 IM30  :  1;  /* Interrupt mask 30. */
        UINT32 IM31  :  1;  /* Interrupt mask 31. */
    } B;
} GPIO_IMR_t;

/*
 * GPIO_ISR register bits definition *
 */
typedef union GPIO_ISR_union_t {
    UINT32 R;
    struct {
        UINT32 IS0   :  1;  /* Interrupt status 0. */
        UINT32 IS1   :  1;  /* Interrupt status 1. */
        UINT32 IS2   :  1;  /* Interrupt status 2. */
        UINT32 IS3   :  1;  /* Interrupt status 3. */
        UINT32 IS4   :  1;  /* Interrupt status 4. */
        UINT32 IS5   :  1;  /* Interrupt status 5. */
        UINT32 IS6   :  1;  /* Interrupt status 6. */
        UINT32 IS7   :  1;  /* Interrupt status 7. */
        UINT32 IS8   :  1;  /* Interrupt status 8. */
        UINT32 IS9   :  1;  /* Interrupt status 9. */
        UINT32 IS10  :  1;  /* Interrupt status 10. */
        UINT32 IS11  :  1;  /* Interrupt status 11. */
        UINT32 IS12  :  1;  /* Interrupt status 12. */
        UINT32 IS13  :  1;  /* Interrupt status 13. */
        UINT32 IS14  :  1;  /* Interrupt status 14. */
        UINT32 IS15  :  1;  /* Interrupt status 15. */
        UINT32 IS16  :  1;  /* Interrupt status 16. */
        UINT32 IS17  :  1;  /* Interrupt status 17. */
        UINT32 IS18  :  1;  /* Interrupt status 18. */
        UINT32 IS19  :  1;  /* Interrupt status 19. */
        UINT32 IS20  :  1;  /* Interrupt status 20. */
        UINT32 IS21  :  1;  /* Interrupt status 21. */
        UINT32 IS22  :  1;  /* Interrupt status 22. */
        UINT32 IS23  :  1;  /* Interrupt status 23. */
        UINT32 IS24  :  1;  /* Interrupt status 24. */
        UINT32 IS25  :  1;  /* Interrupt status 25. */
        UINT32 IS26  :  1;  /* Interrupt status 26. */
        UINT32 IS27  :  1;  /* Interrupt status 27. */
        UINT32 IS28  :  1;  /* Interrupt status 28. */
        UINT32 IS29  :  1;  /* Interrupt status 29. */
        UINT32 IS30  :  1;  /* Interrupt status 30. */
        UINT32 IS31  :  1;  /* Interrupt status 31. */
    } B;
} GPIO_ISR_t;

/*
 * GPIO_EDGE_SEL register bits definition *
 */
typedef union GPIO_EDGE_SEL_union_t {
    UINT32 R;
    struct {
        UINT32 ES0   :  1;  /* Edge select 0. */
        UINT32 ES1   :  1;  /* Edge select 1. */
        UINT32 ES2   :  1;  /* Edge select 2. */
        UINT32 ES3   :  1;  /* Edge select 3. */
        UINT32 ES4   :  1;  /* Edge select 4. */
        UINT32 ES5   :  1;  /* Edge select 5. */
        UINT32 ES6   :  1;  /* Edge select 6. */
        UINT32 ES7   :  1;  /* Edge select 7. */
        UINT32 ES8   :  1;  /* Edge select 8. */
        UINT32 ES9   :  1;  /* Edge select 9. */
        UINT32 ES10  :  1;  /* Edge select 10. */
        UINT32 ES11  :  1;  /* Edge select 11. */
        UINT32 ES12  :  1;  /* Edge select 12. */
        UINT32 ES13  :  1;  /* Edge select 13. */
        UINT32 ES14  :  1;  /* Edge select 14. */
        UINT32 ES15  :  1;  /* Edge select 15. */
        UINT32 ES16  :  1;  /* Edge select 16. */
        UINT32 ES17  :  1;  /* Edge select 17. */
        UINT32 ES18  :  1;  /* Edge select 18. */
        UINT32 ES19  :  1;  /* Edge select 19. */
        UINT32 ES20  :  1;  /* Edge select 20. */
        UINT32 ES21  :  1;  /* Edge select 21. */
        UINT32 ES22  :  1;  /* Edge select 22. */
        UINT32 ES23  :  1;  /* Edge select 23. */
        UINT32 ES24  :  1;  /* Edge select 24. */
        UINT32 ES25  :  1;  /* Edge select 25. */
        UINT32 ES26  :  1;  /* Edge select 26. */
        UINT32 ES27  :  1;  /* Edge select 27. */
        UINT32 ES28  :  1;  /* Edge select 28. */
        UINT32 ES29  :  1;  /* Edge select 29. */
        UINT32 ES30  :  1;  /* Edge select 30. */
        UINT32 ES31  :  1;  /* Edge select 31. */
    } B;
} GPIO_EDGE_SEL_t;

/*
 * GPIO_DR_SET register bits definition *
 */
typedef union GPIO_DR_SET_union_t {
    UINT32 R;
    struct {
        UINT32 DV0   :  1;  /* Bits to set 0. */
        UINT32 DV1   :  1;  /* Bits to set 1. */
        UINT32 DV2   :  1;  /* Bits to set 2. */
        UINT32 DV3   :  1;  /* Bits to set 3. */
        UINT32 DV4   :  1;  /* Bits to set 4. */
        UINT32 DV5   :  1;  /* Bits to set 5. */
        UINT32 DV6   :  1;  /* Bits to set 6. */
        UINT32 DV7   :  1;  /* Bits to set 7. */
        UINT32 DV8   :  1;  /* Bits to set 8. */
        UINT32 DV9   :  1;  /* Bits to set 9. */
        UINT32 DV10  :  1;  /* Bits to set 10. */
        UINT32 DV11  :  1;  /* Bits to set 11. */
        UINT32 DV12  :  1;  /* Bits to set 12. */
        UINT32 DV13  :  1;  /* Bits to set 13. */
        UINT32 DV14  :  1;  /* Bits to set 14. */
        UINT32 DV15  :  1;  /* Bits to set 15. */
        UINT32 DV16  :  1;  /* Bits to set 16. */
        UINT32 DV17  :  1;  /* Bits to set 17. */
        UINT32 DV18  :  1;  /* Bits to set 18. */
        UINT32 DV19  :  1;  /* Bits to set 19. */
        UINT32 DV20  :  1;  /* Bits to set 20. */
        UINT32 DV21  :  1;  /* Bits to set 21. */
        UINT32 DV22  :  1;  /* Bits to set 22. */
        UINT32 DV23  :  1;  /* Bits to set 23. */
        UINT32 DV24  :  1;  /* Bits to set 24. */
        UINT32 DV25  :  1;  /* Bits to set 25. */
        UINT32 DV26  :  1;  /* Bits to set 26. */
        UINT32 DV27  :  1;  /* Bits to set 27. */
        UINT32 DV28  :  1;  /* Bits to set 28. */
        UINT32 DV29  :  1;  /* Bits to set 29. */
        UINT32 DV30  :  1;  /* Bits to set 30. */
        UINT32 DV31  :  1;  /* Bits to set 31. */
    } B;
} GPIO_DR_SET_t;

/*
 * GPIO_DR_CLEAR register bits definition *
 */
typedef union GPIO_DR_CLEAR_union_t {
    UINT32 R;
    struct {
        UINT32 DV0   :  1;  /* Bits to clear 0. */
        UINT32 DV1   :  1;  /* Bits to clear 1. */
        UINT32 DV2   :  1;  /* Bits to clear 2. */
        UINT32 DV3   :  1;  /* Bits to clear 3. */
        UINT32 DV4   :  1;  /* Bits to clear 4. */
        UINT32 DV5   :  1;  /* Bits to clear 5. */
        UINT32 DV6   :  1;  /* Bits to clear 6. */
        UINT32 DV7   :  1;  /* Bits to clear 7. */
        UINT32 DV8   :  1;  /* Bits to clear 8. */
        UINT32 DV9   :  1;  /* Bits to clear 9. */
        UINT32 DV10  :  1;  /* Bits to clear 10. */
        UINT32 DV11  :  1;  /* Bits to clear 11. */
        UINT32 DV12  :  1;  /* Bits to clear 12. */
        UINT32 DV13  :  1;  /* Bits to clear 13. */
        UINT32 DV14  :  1;  /* Bits to clear 14. */
        UINT32 DV15  :  1;  /* Bits to clear 15. */
        UINT32 DV16  :  1;  /* Bits to clear 16. */
        UINT32 DV17  :  1;  /* Bits to clear 17. */
        UINT32 DV18  :  1;  /* Bits to clear 18. */
        UINT32 DV19  :  1;  /* Bits to clear 19. */
        UINT32 DV20  :  1;  /* Bits to clear 20. */
        UINT32 DV21  :  1;  /* Bits to clear 21. */
        UINT32 DV22  :  1;  /* Bits to clear 22. */
        UINT32 DV23  :  1;  /* Bits to clear 23. */
        UINT32 DV24  :  1;  /* Bits to clear 24. */
        UINT32 DV25  :  1;  /* Bits to clear 25. */
        UINT32 DV26  :  1;  /* Bits to clear 26. */
        UINT32 DV27  :  1;  /* Bits to clear 27. */
        UINT32 DV28  :  1;  /* Bits to clear 28. */
        UINT32 DV29  :  1;  /* Bits to clear 29. */
        UINT32 DV30  :  1;  /* Bits to clear 30. */
        UINT32 DV31  :  1;  /* Bits to clear 31. */
    } B;
} GPIO_DR_CLEAR_t;

/*
 * GPIO_DR_TOGGLE register bits definition *
 */
typedef union GPIO_DR_TOGGLE_union_t {
    UINT32 R;
    struct {
        UINT32 DV0   :  1;  /* Bits to toggle 0. */
        UINT32 DV1   :  1;  /* Bits to toggle 1. */
        UINT32 DV2   :  1;  /* Bits to toggle 2. */
        UINT32 DV3   :  1;  /* Bits to toggle 3. */
        UINT32 DV4   :  1;  /* Bits to toggle 4. */
        UINT32 DV5   :  1;  /* Bits to toggle 5. */
        UINT32 DV6   :  1;  /* Bits to toggle 6. */
        UINT32 DV7   :  1;  /* Bits to toggle 7. */
        UINT32 DV8   :  1;  /* Bits to toggle 8. */
        UINT32 DV9   :  1;  /* Bits to toggle 9. */
        UINT32 DV10  :  1;  /* Bits to toggle 10. */
        UINT32 DV11  :  1;  /* Bits to toggle 11. */
        UINT32 DV12  :  1;  /* Bits to toggle 12. */
        UINT32 DV13  :  1;  /* Bits to toggle 13. */
        UINT32 DV14  :  1;  /* Bits to toggle 14. */
        UINT32 DV15  :  1;  /* Bits to toggle 15. */
        UINT32 DV16  :  1;  /* Bits to toggle 16. */
        UINT32 DV17  :  1;  /* Bits to toggle 17. */
        UINT32 DV18  :  1;  /* Bits to toggle 18. */
        UINT32 DV19  :  1;  /* Bits to toggle 19. */
        UINT32 DV20  :  1;  /* Bits to toggle 20. */
        UINT32 DV21  :  1;  /* Bits to toggle 21. */
        UINT32 DV22  :  1;  /* Bits to toggle 22. */
        UINT32 DV23  :  1;  /* Bits to toggle 23. */
        UINT32 DV24  :  1;  /* Bits to toggle 24. */
        UINT32 DV25  :  1;  /* Bits to toggle 25. */
        UINT32 DV26  :  1;  /* Bits to toggle 26. */
        UINT32 DV27  :  1;  /* Bits to toggle 27. */
        UINT32 DV28  :  1;  /* Bits to toggle 28. */
        UINT32 DV29  :  1;  /* Bits to toggle 29. */
        UINT32 DV30  :  1;  /* Bits to toggle 30. */
        UINT32 DV31  :  1;  /* Bits to toggle 31. */
    } B;
} GPIO_DR_TOGGLE_t;

/*
 * GPIO structure definition
 */
typedef struct GPIO_s {
    GPIO_DR_t             DR;                                       /* 0x00000000 GPIO data register (DR) */
    GPIO_GDIR_t           GDIR;                                     /* 0x00000004 GPIO direction register (GDIR) */
    GPIO_PSR_t            PSR;                                      /* 0x00000008 GPIO pad status register (PSR) */
    GPIO_ICR1_t           ICR1;                                     /* 0x0000000C GPIO interrupt configuration register1 (ICR1) */
    GPIO_ICR2_t           ICR2;                                     /* 0x00000010 GPIO interrupt configuration register2 (ICR2) */
    GPIO_IMR_t            IMR;                                      /* 0x00000014 GPIO interrupt mask register (IMR) */
    GPIO_ISR_t            ISR;                                      /* 0x00000018 GPIO interrupt status register (ISR) */
    GPIO_EDGE_SEL_t       EDGE_SEL;                                 /* 0x0000001C GPIO edge select register (EDGE_SEL) */
    UINT8                 Reserved_0x20[100];                       /* 0x00000020 Reserved */
    GPIO_DR_SET_t         DR_SET;                                   /* 0x00000084 GPIO data register SET (DR_SET) */
    GPIO_DR_CLEAR_t       DR_CLEAR;                                 /* 0x00000088 GPIO data register CLEAR (DR_CLEAR) */
    GPIO_DR_TOGGLE_t      DR_TOGGLE;                                /* 0x0000008C GPIO data register TOGGLE (DR_TOGGLE) */
} volatile GPIO_t;

/* coverity[divide_by_zero] - Suppress coverity divide_by_zero error */
IO_MAP_STATIC_ASSERT(sizeof(GPIO_t) == 0x90);

typedef struct GPIO_debug_s {
    UINT32 RegOffset;
    char*  RegName;
    char*  RegDescr;
} GPIO_debug_t;

/*
 * GPIO debug structure definition
 */
GPIO_debug_t GPIO_DebugInfo[] = {
    {0x00000000, "DR",                          "GPIO data register (DR)"},
    {0x00000004, "GDIR",                        "GPIO direction register (GDIR)"},
    {0x00000008, "PSR",                         "GPIO pad status register (PSR)"},
    {0x0000000C, "ICR1",                        "GPIO interrupt configuration register1 (ICR1)"},
    {0x00000010, "ICR2",                        "GPIO interrupt configuration register2 (ICR2)"},
    {0x00000014, "IMR",                         "GPIO interrupt mask register (IMR)"},
    {0x00000018, "IMS",                         "GPIO interrupt status register (ISR)"},
    {0x0000001C, "EDGE_SEL",                    "GPIO edge select register (EDGE_SEL)"},
    {0x00000084, "DR_SET",                      "GPIO data register SET (DR_SET)"},
    {0x00000088, "DR_CLEAR",                    "GPIO data register CLEAR (DR_CLEAR)"},
    {0x0000008C, "DR_TOGGLE",                   "GPIO data register TOGGLE (DR_TOGGLE)"},
};

#endif /* _GPIO_IO_MAP_H_ */