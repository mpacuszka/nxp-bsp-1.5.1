/*
* Copyright 2023 NXP
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

#ifndef _GPIO_IO_MAP_H_
#define _GPIO_IO_MAP_H_

#define IO_MAP_ASSERT_CONCAT_(a, b) a##b
#define IO_MAP_ASSERT_CONCAT(a, b) IO_MAP_ASSERT_CONCAT_(a, b)
#define IO_MAP_STATIC_ASSERT(e) enum { IO_MAP_ASSERT_CONCAT(static_assert_, __LINE__) = 1/(int)(!!(e)) }


/*
 * GPIO registers definition
 */

#define GPIO_VERID                                       0x0000U
#define GPIO_PARAM                                       0x0004U
#define GPIO_LOCK                                        0x000CU
#define GPIO_PCNS                                        0x0010U
#define GPIO_ICNS                                        0x0014U
#define GPIO_PCNP                                        0x0018U
#define GPIO_ICNP                                        0x001CU
#define GPIO_PDOR                                        0x0040U
#define GPIO_PSOR                                        0x0044U
#define GPIO_PCOR                                        0x0048U
#define GPIO_PTOR                                        0x004CU
#define GPIO_PDIR                                        0x0050U
#define GPIO_PDDR                                        0x0054U
#define GPIO_PIDR                                        0x0058U
#define GPIO_PnDR                                        0x0060U
#define GPIO_ICRn                                        0x0080U
#define GPIO_GICLR                                       0x0100U
#define GPIO_GICHR                                       0x0104U
#define GPIO_ISFR0                                       0x0120U
#define GPIO_ISFR1                                       0x0121U


 /*
  * GPIO_VERID register bits *
  */
#define GPIO_VERID_MAJOR_MASK                            0xFF000000U
#define GPIO_VERID_MAJOR_SHIFT                           24U
#define GPIO_VERID_MINOR_MASK                            0x00FF0000U
#define GPIO_VERID_MINOR_SHIFT                           16U
#define GPIO_VERID_FEATURE_MASK                          0x0000FFFFU
#define GPIO_VERID_FEATURE_SHIFT                         0U

  /*
   * GPIO_PARAM register bits *
   */
#define GPIO_PARAM_IRQNUM_MASK                           0x0000000FU
#define GPIO_PARAM_IRQNUM_SHIFT                          0U

   /*
    * GPIO_LOCK register bits *
    */
#define GPIO_LOCK_ICNP_MASK                              0x00000008U
#define GPIO_LOCK_ICNP_SHIFT                             3U
#define GPIO_LOCK_PCNP_MASK                              0x00000004U
#define GPIO_LOCK_PCNP_SHIFT                             2U
#define GPIO_LOCK_ICNS_MASK                              0x00000002U
#define GPIO_LOCK_ICNS_SHIFT                             1U
#define GPIO_LOCK_PCNS_MASK                              0x00000001U
#define GPIO_LOCK_PCNS_SHIFT                             0U

    /*
     * GPIO_PnDR register bits *
     */
#define GPIO_PnDR_PD_MASK                                0x00000001U
#define GPIO_PnDR_PD_SHIFT                               0U

     /*
      * GPIO_ICRn register bits *
      */
#define GPIO_ICRn_ISF_MASK                               0x01000000U
#define GPIO_ICRn_ISF_SHIFT                              24U
#define GPIO_ICRn_LK_MASK                                0x00800000U
#define GPIO_ICRn_LK_SHIFT                               23U
#define GPIO_ICRn_IRQS_MASK                              0x00100000U
#define GPIO_ICRn_IRQS_SHIFT                             20U
#define GPIO_ICRn_IRQC_MASK                              0x000F0000U
#define GPIO_ICRn_IRQC_SHIFT                             16U

      /*
       * GPIO_GICLR register bits *
       */
#define GPIO_GICLR_GIWD_MASK                             0xFFFF0000U
#define GPIO_GICLR_GIWD_SHIFT                            16U
#define GPIO_GICLR_GIWE15_MASK                           0x00008000U
#define GPIO_GICLR_GIWE15_SHIFT                          15U
#define GPIO_GICLR_GIWE14_MASK                           0x00004000U
#define GPIO_GICLR_GIWE14_SHIFT                          14U
#define GPIO_GICLR_GIWE13_MASK                           0x00002000U
#define GPIO_GICLR_GIWE13_SHIFT                          13U
#define GPIO_GICLR_GIWE12_MASK                           0x00001000U
#define GPIO_GICLR_GIWE12_SHIFT                          12U
#define GPIO_GICLR_GIWE11_MASK                           0x00000800U
#define GPIO_GICLR_GIWE11_SHIFT                          11U
#define GPIO_GICLR_GIWE10_MASK                           0x00000400U
#define GPIO_GICLR_GIWE10_SHIFT                          10U
#define GPIO_GICLR_GIWE9_MASK                            0x00000200U
#define GPIO_GICLR_GIWE9_SHIFT                           9U
#define GPIO_GICLR_GIWE8_MASK                            0x00000100U
#define GPIO_GICLR_GIWE8_SHIFT                           8U
#define GPIO_GICLR_GIWE7_MASK                            0x00000080U
#define GPIO_GICLR_GIWE7_SHIFT                           7U
#define GPIO_GICLR_GIWE6_MASK                            0x00000040U
#define GPIO_GICLR_GIWE6_SHIFT                           6U
#define GPIO_GICLR_GIWE5_MASK                            0x00000020U
#define GPIO_GICLR_GIWE5_SHIFT                           5U
#define GPIO_GICLR_GIWE4_MASK                            0x00000010U
#define GPIO_GICLR_GIWE4_SHIFT                           4U
#define GPIO_GICLR_GIWE3_MASK                            0x00000008U
#define GPIO_GICLR_GIWE3_SHIFT                           3U
#define GPIO_GICLR_GIWE2_MASK                            0x00000004U
#define GPIO_GICLR_GIWE2_SHIFT                           2U
#define GPIO_GICLR_GIWE1_MASK                            0x00000002U
#define GPIO_GICLR_GIWE1_SHIFT                           1U
#define GPIO_GICLR_GIWE0_MASK                            0x00000001U
#define GPIO_GICLR_GIWE0_SHIFT                           0U

       /*
        * GPIO_GICLH register bits *
        */
#define GPIO_GICLH_GIWD_MASK                             0xFFFF0000U
#define GPIO_GICLH_GIWD_SHIFT                            16U
#define GPIO_GICLH_GIWE31_MASK                           0x00008000U
#define GPIO_GICLH_GIWE31_SHIFT                          15U
#define GPIO_GICLH_GIWE30_MASK                           0x00004000U
#define GPIO_GICLH_GIWE30_SHIFT                          14U
#define GPIO_GICLH_GIWE29_MASK                           0x00002000U
#define GPIO_GICLH_GIWE29_SHIFT                          13U
#define GPIO_GICLH_GIWE28_MASK                           0x00001000U
#define GPIO_GICLH_GIWE28_SHIFT                          12U
#define GPIO_GICLH_GIWE27_MASK                           0x00000800U
#define GPIO_GICLH_GIWE27_SHIFT                          11U
#define GPIO_GICLH_GIWE26_MASK                           0x00000400U
#define GPIO_GICLH_GIWE26_SHIFT                          10U
#define GPIO_GICLH_GIWE25_MASK                           0x00000200U
#define GPIO_GICLH_GIWE25_SHIFT                          9U
#define GPIO_GICLH_GIWE24_MASK                           0x00000100U
#define GPIO_GICLH_GIWE24_SHIFT                          8U
#define GPIO_GICLH_GIWE23_MASK                           0x00000080U
#define GPIO_GICLH_GIWE23_SHIFT                          7U
#define GPIO_GICLH_GIWE22_MASK                           0x00000040U
#define GPIO_GICLH_GIWE22_SHIFT                          6U
#define GPIO_GICLH_GIWE21_MASK                           0x00000020U
#define GPIO_GICLH_GIWE21_SHIFT                          5U
#define GPIO_GICLH_GIWE20_MASK                           0x00000010U
#define GPIO_GICLH_GIWE20_SHIFT                          4U
#define GPIO_GICLH_GIWE19_MASK                           0x00000008U
#define GPIO_GICLH_GIWE19_SHIFT                          3U
#define GPIO_GICLH_GIWE18_MASK                           0x00000004U
#define GPIO_GICLH_GIWE18_SHIFT                          2U
#define GPIO_GICLH_GIWE17_MASK                           0x00000002U
#define GPIO_GICLH_GIWE17_SHIFT                          1U
#define GPIO_GICLH_GIWE16_MASK                           0x00000001U
#define GPIO_GICLH_GIWE16_SHIFT                          0U

/*
 * GPIO_VERID register bits definition *
 */
typedef union GPIO_VERID_union_t {
    UINT32 R;
    struct {
        UINT32 FEATURE : 16;  /* Feature Specification Number */
        UINT32 MINOR : 8;  /* Minor Version Number */
        UINT32 MAJOR : 8;  /* Major Version Number */
    } B;
} GPIO_VERID_t;

/*
 * GPIO_PARAM register bits definition *
 */
typedef union GPIO_PARAM_union_t {
    UINT32 R;
    struct {
        UINT32 IRQNUM : 4;  /* Interrupt Number */
        UINT32 Reserved_4 : 28;  /* Reserved */
    } B;
} GPIO_PARAM_t;

/*
 * GPIO_LOCK register bits definition *
 */
typedef union GPIO_LOCK_union_t {
    UINT32 R;
    struct {
        UINT32 PCNS : 1;  /* PCNS register write lock */
        UINT32 ICNS : 1;  /* ICNS register write lock */
        UINT32 PCNP : 1;  /* PCNP register write lock */
        UINT32 ICNP : 1;  /* ICNP register write lock */
        UINT32 Reserved_4 : 28;  /* Reserved */
    } B;
} GPIO_LOCK_t;

/*
 * GPIO_PCNS register bits definition *
 */
typedef union GPIO_PCNS_union_t {
    UINT32 R;
    struct {
        UINT32 NSE0 : 1;  /* Pin 0 configuration in Non-Secure access enable */
        UINT32 NSE1 : 1;  /* Pin 1 configuration in Non-Secure access enable */
        UINT32 NSE2 : 1;  /* Pin 2 configuration in Non-Secure access enable */
        UINT32 NSE3 : 1;  /* Pin 3 configuration in Non-Secure access enable */
        UINT32 NSE4 : 1;  /* Pin 4 configuration in Non-Secure access enable */
        UINT32 NSE5 : 1;  /* Pin 5 configuration in Non-Secure access enable */
        UINT32 NSE6 : 1;  /* Pin 6 configuration in Non-Secure access enable */
        UINT32 NSE7 : 1;  /* Pin 7 configuration in Non-Secure access enable */
        UINT32 NSE8 : 1;  /* Pin 8 configuration in Non-Secure access enable */
        UINT32 NSE9 : 1;  /* Pin 9 configuration in Non-Secure access enable */
        UINT32 NSE10 : 1;  /* Pin 10 configuration in Non-Secure access enable */
        UINT32 NSE11 : 1;  /* Pin 11 configuration in Non-Secure access enable */
        UINT32 NSE12 : 1;  /* Pin 12 configuration in Non-Secure access enable */
        UINT32 NSE13 : 1;  /* Pin 13 configuration in Non-Secure access enable */
        UINT32 NSE14 : 1;  /* Pin 14 configuration in Non-Secure access enable */
        UINT32 NSE15 : 1;  /* Pin 15 configuration in Non-Secure access enable */
        UINT32 NSE16 : 1;  /* Pin 16 configuration in Non-Secure access enable */
        UINT32 NSE17 : 1;  /* Pin 17 configuration in Non-Secure access enable */
        UINT32 NSE18 : 1;  /* Pin 18 configuration in Non-Secure access enable */
        UINT32 NSE19 : 1;  /* Pin 19 configuration in Non-Secure access enable */
        UINT32 NSE20 : 1;  /* Pin 20 configuration in Non-Secure access enable */
        UINT32 NSE21 : 1;  /* Pin 21 configuration in Non-Secure access enable */
        UINT32 NSE22 : 1;  /* Pin 22 configuration in Non-Secure access enable */
        UINT32 NSE23 : 1;  /* Pin 23 configuration in Non-Secure access enable */
        UINT32 NSE24 : 1;  /* Pin 24 configuration in Non-Secure access enable */
        UINT32 NSE25 : 1;  /* Pin 25 configuration in Non-Secure access enable */
        UINT32 NSE26 : 1;  /* Pin 26 configuration in Non-Secure access enable */
        UINT32 NSE27 : 1;  /* Pin 27 configuration in Non-Secure access enable */
        UINT32 NSE28 : 1;  /* Pin 28 configuration in Non-Secure access enable */
        UINT32 NSE29 : 1;  /* Pin 29 configuration in Non-Secure access enable */
        UINT32 NSE30 : 1;  /* Pin 30 configuration in Non-Secure access enable */
        UINT32 NSE31 : 1;  /* Pin 31 configuration in Non-Secure access enable */
    } B;
} GPIO_PCNS_t;

/*
 * GPIO_ICNS register bits definition *
 */
typedef union GPIO_ICNS_union_t {
    UINT32 R;
    struct {
        UINT32 NSE0 : 1;  /* Interrupt 0 configuration in Non-Secure access enable */
        UINT32 NSE1 : 1;  /* Interrupt 1 configuration in Non-Secure access enable */
        UINT32 Reserved_2 : 30;  /* Reserved */
    } B;
} GPIO_ICNS_t;

/*
 * GPIO_PCNP register bits definition *
 */
typedef union GPIO_PCNP_union_t {
    UINT32 R;
    struct {
        UINT32 NPE0 : 1;  /* Pin 0 configuration in Non-Secure access enable */
        UINT32 NPE1 : 1;  /* Pin 1 configuration in Non-Secure access enable */
        UINT32 NPE2 : 1;  /* Pin 2 configuration in Non-Secure access enable */
        UINT32 NPE3 : 1;  /* Pin 3 configuration in Non-Secure access enable */
        UINT32 NPE4 : 1;  /* Pin 4 configuration in Non-Secure access enable */
        UINT32 NPE5 : 1;  /* Pin 5 configuration in Non-Secure access enable */
        UINT32 NPE6 : 1;  /* Pin 6 configuration in Non-Secure access enable */
        UINT32 NPE7 : 1;  /* Pin 7 configuration in Non-Secure access enable */
        UINT32 NPE8 : 1;  /* Pin 8 configuration in Non-Secure access enable */
        UINT32 NPE9 : 1;  /* Pin 9 configuration in Non-Secure access enable */
        UINT32 NPE10 : 1;  /* Pin 10 configuration in Non-Secure access enable */
        UINT32 NPE11 : 1;  /* Pin 11 configuration in Non-Secure access enable */
        UINT32 NPE12 : 1;  /* Pin 12 configuration in Non-Secure access enable */
        UINT32 NPE13 : 1;  /* Pin 13 configuration in Non-Secure access enable */
        UINT32 NPE14 : 1;  /* Pin 14 configuration in Non-Secure access enable */
        UINT32 NPE15 : 1;  /* Pin 15 configuration in Non-Secure access enable */
        UINT32 NPE16 : 1;  /* Pin 16 configuration in Non-Secure access enable */
        UINT32 NPE17 : 1;  /* Pin 17 configuration in Non-Secure access enable */
        UINT32 NPE18 : 1;  /* Pin 18 configuration in Non-Secure access enable */
        UINT32 NPE19 : 1;  /* Pin 19 configuration in Non-Secure access enable */
        UINT32 NPE20 : 1;  /* Pin 20 configuration in Non-Secure access enable */
        UINT32 NPE21 : 1;  /* Pin 21 configuration in Non-Secure access enable */
        UINT32 NPE22 : 1;  /* Pin 22 configuration in Non-Secure access enable */
        UINT32 NPE23 : 1;  /* Pin 23 configuration in Non-Secure access enable */
        UINT32 NPE24 : 1;  /* Pin 24 configuration in Non-Secure access enable */
        UINT32 NPE25 : 1;  /* Pin 25 configuration in Non-Secure access enable */
        UINT32 NPE26 : 1;  /* Pin 26 configuration in Non-Secure access enable */
        UINT32 NPE27 : 1;  /* Pin 27 configuration in Non-Secure access enable */
        UINT32 NPE28 : 1;  /* Pin 28 configuration in Non-Secure access enable */
        UINT32 NPE29 : 1;  /* Pin 29 configuration in Non-Secure access enable */
        UINT32 NPE30 : 1;  /* Pin 30 configuration in Non-Secure access enable */
        UINT32 NPE31 : 1;  /* Pin 31 configuration in Non-Secure access enable */
    } B;
} GPIO_PCNP_t;

/*
 * GPIO_ICNP register bits definition *
 */
typedef union GPIO_ICNP_union_t {
    UINT32 R;
    struct {
        UINT32 NPE0 : 1;  /* Interrupt 0 configuration in Non-Privilege access enable */
        UINT32 NPE1 : 1;  /* Interrupt 1 configuration in Non-Privilege access enable */
        UINT32 Reserved_2 : 30;  /* Reserved */
    } B;
} GPIO_ICNP_t;

/*
 * GPIO_PDOR register bits definition *
 */
typedef union GPIO_PDOR_union_t {
    UINT32 R;
    struct {
        UINT32 PDO0 : 1;  /* Data value 0. */
        UINT32 PDO1 : 1;  /* Data value 1. */
        UINT32 PDO2 : 1;  /* Data value 2. */
        UINT32 PDO3 : 1;  /* Data value 3. */
        UINT32 PDO4 : 1;  /* Data value 4. */
        UINT32 PDO5 : 1;  /* Data value 5. */
        UINT32 PDO6 : 1;  /* Data value 6. */
        UINT32 PDO7 : 1;  /* Data value 7. */
        UINT32 PDO8 : 1;  /* Data value 8. */
        UINT32 PDO9 : 1;  /* Data value 9. */
        UINT32 PDO10 : 1;  /* Data value 10. */
        UINT32 PDO11 : 1;  /* Data value 11. */
        UINT32 PDO12 : 1;  /* Data value 12. */
        UINT32 PDO13 : 1;  /* Data value 13. */
        UINT32 PDO14 : 1;  /* Data value 14. */
        UINT32 PDO15 : 1;  /* Data value 15. */
        UINT32 PDO16 : 1;  /* Data value 16. */
        UINT32 PDO17 : 1;  /* Data value 17. */
        UINT32 PDO18 : 1;  /* Data value 18. */
        UINT32 PDO19 : 1;  /* Data value 19. */
        UINT32 PDO20 : 1;  /* Data value 20. */
        UINT32 PDO21 : 1;  /* Data value 21. */
        UINT32 PDO22 : 1;  /* Data value 22. */
        UINT32 PDO23 : 1;  /* Data value 23. */
        UINT32 PDO24 : 1;  /* Data value 24. */
        UINT32 PDO25 : 1;  /* Data value 25. */
        UINT32 PDO26 : 1;  /* Data value 26. */
        UINT32 PDO27 : 1;  /* Data value 27. */
        UINT32 PDO28 : 1;  /* Data value 28. */
        UINT32 PDO29 : 1;  /* Data value 29. */
        UINT32 PDO30 : 1;  /* Data value 30. */
        UINT32 PDO31 : 1;  /* Data value 31. */
    } B;
} GPIO_PDOR_t;

/*
 * GPIO_PSOR register bits definition *
 */
typedef union GPIO_PSOR_union_t {
    UINT32 R;
    struct {
        UINT32 PTSO0 : 1;  /* Set value 0. */
        UINT32 PTSO1 : 1;  /* Set value 1. */
        UINT32 PTSO2 : 1;  /* Set value 2. */
        UINT32 PTSO3 : 1;  /* Set value 3. */
        UINT32 PTSO4 : 1;  /* Set value 4. */
        UINT32 PTSO5 : 1;  /* Set value 5. */
        UINT32 PTSO6 : 1;  /* Set value 6. */
        UINT32 PTSO7 : 1;  /* Set value 7. */
        UINT32 PTSO8 : 1;  /* Set value 8. */
        UINT32 PTSO9 : 1;  /* Set value 9. */
        UINT32 PTSO10 : 1;  /* Set value 10. */
        UINT32 PTSO11 : 1;  /* Set value 11. */
        UINT32 PTSO12 : 1;  /* Set value 12. */
        UINT32 PTSO13 : 1;  /* Set value 13. */
        UINT32 PTSO14 : 1;  /* Set value 14. */
        UINT32 PTSO15 : 1;  /* Set value 15. */
        UINT32 PTSO16 : 1;  /* Set value 16. */
        UINT32 PTSO17 : 1;  /* Set value 17. */
        UINT32 PTSO18 : 1;  /* Set value 18. */
        UINT32 PTSO19 : 1;  /* Set value 19. */
        UINT32 PTSO20 : 1;  /* Set value 20. */
        UINT32 PTSO21 : 1;  /* Set value 21. */
        UINT32 PTSO22 : 1;  /* Set value 22. */
        UINT32 PTSO23 : 1;  /* Set value 23. */
        UINT32 PTSO24 : 1;  /* Set value 24. */
        UINT32 PTSO25 : 1;  /* Set value 25. */
        UINT32 PTSO26 : 1;  /* Set value 26. */
        UINT32 PTSO27 : 1;  /* Set value 27. */
        UINT32 PTSO28 : 1;  /* Set value 28. */
        UINT32 PTSO29 : 1;  /* Set value 29. */
        UINT32 PTSO30 : 1;  /* Set value 30. */
        UINT32 PTSO31 : 1;  /* Set value 31. */
    } B;
} GPIO_PSOR_t;

/*
 * GPIO_PCOR register bits definition *
 */
typedef union GPIO_PCOR_union_t {
    UINT32 R;
    struct {
        UINT32 PTCO0 : 1;  /* Clear value 0. */
        UINT32 PTCO1 : 1;  /* Clear value 1. */
        UINT32 PTCO2 : 1;  /* Clear value 2. */
        UINT32 PTCO3 : 1;  /* Clear value 3. */
        UINT32 PTCO4 : 1;  /* Clear value 4. */
        UINT32 PTCO5 : 1;  /* Clear value 5. */
        UINT32 PTCO6 : 1;  /* Clear value 6. */
        UINT32 PTCO7 : 1;  /* Clear value 7. */
        UINT32 PTCO8 : 1;  /* Clear value 8. */
        UINT32 PTCO9 : 1;  /* Clear value 9. */
        UINT32 PTCO10 : 1;  /* Clear value 10. */
        UINT32 PTCO11 : 1;  /* Clear value 11. */
        UINT32 PTCO12 : 1;  /* Clear value 12. */
        UINT32 PTCO13 : 1;  /* Clear value 13. */
        UINT32 PTCO14 : 1;  /* Clear value 14. */
        UINT32 PTCO15 : 1;  /* Clear value 15. */
        UINT32 PTCO16 : 1;  /* Clear value 16. */
        UINT32 PTCO17 : 1;  /* Clear value 17. */
        UINT32 PTCO18 : 1;  /* Clear value 18. */
        UINT32 PTCO19 : 1;  /* Clear value 19. */
        UINT32 PTCO20 : 1;  /* Clear value 20. */
        UINT32 PTCO21 : 1;  /* Clear value 21. */
        UINT32 PTCO22 : 1;  /* Clear value 22. */
        UINT32 PTCO23 : 1;  /* Clear value 23. */
        UINT32 PTCO24 : 1;  /* Clear value 24. */
        UINT32 PTCO25 : 1;  /* Clear value 25. */
        UINT32 PTCO26 : 1;  /* Clear value 26. */
        UINT32 PTCO27 : 1;  /* Clear value 27. */
        UINT32 PTCO28 : 1;  /* Clear value 28. */
        UINT32 PTCO29 : 1;  /* Clear value 29. */
        UINT32 PTCO30 : 1;  /* Clear value 30. */
        UINT32 PTCO31 : 1;  /* Clear value 31. */
    } B;
} GPIO_PCOR_t;

/*
 * GPIO_PTOR register bits definition *
 */
typedef union GPIO_PTOR_union_t {
    UINT32 R;
    struct {
        UINT32 PTTO0 : 1;  /* Toggle value 0. */
        UINT32 PTTO1 : 1;  /* Toggle value 1. */
        UINT32 PTTO2 : 1;  /* Toggle value 2. */
        UINT32 PTTO3 : 1;  /* Toggle value 3. */
        UINT32 PTTO4 : 1;  /* Toggle value 4. */
        UINT32 PTTO5 : 1;  /* Toggle value 5. */
        UINT32 PTTO6 : 1;  /* Toggle value 6. */
        UINT32 PTTO7 : 1;  /* Toggle value 7. */
        UINT32 PTTO8 : 1;  /* Toggle value 8. */
        UINT32 PTTO9 : 1;  /* Toggle value 9. */
        UINT32 PTTO10 : 1;  /* Toggle value 10. */
        UINT32 PTTO11 : 1;  /* Toggle value 11. */
        UINT32 PTTO12 : 1;  /* Toggle value 12. */
        UINT32 PTTO13 : 1;  /* Toggle value 13. */
        UINT32 PTTO14 : 1;  /* Toggle value 14. */
        UINT32 PTTO15 : 1;  /* Toggle value 15. */
        UINT32 PTTO16 : 1;  /* Toggle value 16. */
        UINT32 PTTO17 : 1;  /* Toggle value 17. */
        UINT32 PTTO18 : 1;  /* Toggle value 18. */
        UINT32 PTTO19 : 1;  /* Toggle value 19. */
        UINT32 PTTO20 : 1;  /* Toggle value 20. */
        UINT32 PTTO21 : 1;  /* Toggle value 21. */
        UINT32 PTTO22 : 1;  /* Toggle value 22. */
        UINT32 PTTO23 : 1;  /* Toggle value 23. */
        UINT32 PTTO24 : 1;  /* Toggle value 24. */
        UINT32 PTTO25 : 1;  /* Toggle value 25. */
        UINT32 PTTO26 : 1;  /* Toggle value 26. */
        UINT32 PTTO27 : 1;  /* Toggle value 27. */
        UINT32 PTTO28 : 1;  /* Toggle value 28. */
        UINT32 PTTO29 : 1;  /* Toggle value 29. */
        UINT32 PTTO30 : 1;  /* Toggle value 30. */
        UINT32 PTTO31 : 1;  /* Toggle value 31. */
    } B;
} GPIO_PTOR_t;

/*
 * GPIO_PDIR register bits definition *
 */
typedef union GPIO_PDIR_union_t {
    UINT32 R;
    struct {
        UINT32 PDI0 : 1;  /* Pin  0 data input value. */
        UINT32 PDI1 : 1;  /* Pin  1 data input value. */
        UINT32 PDI2 : 1;  /* Pin  2 data input value. */
        UINT32 PDI3 : 1;  /* Pin  3 data input value. */
        UINT32 PDI4 : 1;  /* Pin  4 data input value. */
        UINT32 PDI5 : 1;  /* Pin  5 data input value. */
        UINT32 PDI6 : 1;  /* Pin  6 data input value. */
        UINT32 PDI7 : 1;  /* Pin  7 data input value. */
        UINT32 PDI8 : 1;  /* Pin  8 data input value. */
        UINT32 PDI9 : 1;  /* Pin  9 data input value. */
        UINT32 PDI10 : 1;  /* Pin  10 data input value. */
        UINT32 PDI11 : 1;  /* Pin  11 data input value. */
        UINT32 PDI12 : 1;  /* Pin  12 data input value. */
        UINT32 PDI13 : 1;  /* Pin  13 data input value. */
        UINT32 PDI14 : 1;  /* Pin  14 data input value. */
        UINT32 PDI15 : 1;  /* Pin  15 data input value. */
        UINT32 PDI16 : 1;  /* Pin  16 data input value. */
        UINT32 PDI17 : 1;  /* Pin  17 data input value. */
        UINT32 PDI18 : 1;  /* Pin  18 data input value. */
        UINT32 PDI19 : 1;  /* Pin  19 data input value. */
        UINT32 PDI20 : 1;  /* Pin  20 data input value. */
        UINT32 PDI21 : 1;  /* Pin  21 data input value. */
        UINT32 PDI22 : 1;  /* Pin  22 data input value. */
        UINT32 PDI23 : 1;  /* Pin  23 data input value. */
        UINT32 PDI24 : 1;  /* Pin  24 data input value. */
        UINT32 PDI25 : 1;  /* Pin  25 data input value. */
        UINT32 PDI26 : 1;  /* Pin  26 data input value. */
        UINT32 PDI27 : 1;  /* Pin  27 data input value. */
        UINT32 PDI28 : 1;  /* Pin  28 data input value. */
        UINT32 PDI29 : 1;  /* Pin  29 data input value. */
        UINT32 PDI30 : 1;  /* Pin  30 data input value. */
        UINT32 PDI31 : 1;  /* Pin  31 data input value. */
    } B;
} GPIO_PDIR_t;

/*
 * GPIO_PDDR register bits definition *
 */
typedef union GPIO_PDDR_union_t {
    UINT32 R;
    struct {
        UINT32 PDD0 : 1;  /* Pin 0 data direction. */
        UINT32 PDD1 : 1;  /* Pin 1 data direction. */
        UINT32 PDD2 : 1;  /* Pin 2 data direction. */
        UINT32 PDD3 : 1;  /* Pin 3 data direction. */
        UINT32 PDD4 : 1;  /* Pin 4 data direction. */
        UINT32 PDD5 : 1;  /* Pin 5 data direction. */
        UINT32 PDD6 : 1;  /* Pin 6 data direction. */
        UINT32 PDD7 : 1;  /* Pin 7 data direction. */
        UINT32 PDD8 : 1;  /* Pin 8 data direction. */
        UINT32 PDD9 : 1;  /* Pin 9 data direction. */
        UINT32 PDD10 : 1;  /* Pin 10 data direction. */
        UINT32 PDD11 : 1;  /* Pin 11 data direction. */
        UINT32 PDD12 : 1;  /* Pin 12 data direction. */
        UINT32 PDD13 : 1;  /* Pin 13 data direction. */
        UINT32 PDD14 : 1;  /* Pin 14 data direction. */
        UINT32 PDD15 : 1;  /* Pin 15 data direction. */
        UINT32 PDD16 : 1;  /* Pin 16 data direction. */
        UINT32 PDD17 : 1;  /* Pin 17 data direction. */
        UINT32 PDD18 : 1;  /* Pin 18 data direction. */
        UINT32 PDD19 : 1;  /* Pin 19 data direction. */
        UINT32 PDD20 : 1;  /* Pin 20 data direction. */
        UINT32 PDD21 : 1;  /* Pin 21 data direction. */
        UINT32 PDD22 : 1;  /* Pin 22 data direction. */
        UINT32 PDD23 : 1;  /* Pin 23 data direction. */
        UINT32 PDD24 : 1;  /* Pin 24 data direction. */
        UINT32 PDD25 : 1;  /* Pin 25 data direction. */
        UINT32 PDD26 : 1;  /* Pin 26 data direction. */
        UINT32 PDD27 : 1;  /* Pin 27 data direction. */
        UINT32 PDD28 : 1;  /* Pin 28 data direction. */
        UINT32 PDD29 : 1;  /* Pin 29 data direction. */
        UINT32 PDD30 : 1;  /* Pin 30 data direction. */
        UINT32 PDD31 : 1;  /* Pin 31 data direction. */
    } B;
} GPIO_PDDR_t;

/*
 * GPIO_PIDR register bits definition *
 */
typedef union GPIO_PIDR_union_t {
    UINT32 R;
    struct {
        UINT32 PID0 : 1;  /* Pin 0 input disable. */
        UINT32 PID1 : 1;  /* Pin 1 input disable. */
        UINT32 PID2 : 1;  /* Pin 2 input disable. */
        UINT32 PID3 : 1;  /* Pin 3 input disable. */
        UINT32 PID4 : 1;  /* Pin 4 input disable. */
        UINT32 PID5 : 1;  /* Pin 5 input disable. */
        UINT32 PID6 : 1;  /* Pin 6 input disable. */
        UINT32 PID7 : 1;  /* Pin 7 input disable. */
        UINT32 PID8 : 1;  /* Pin 8 input disable. */
        UINT32 PID9 : 1;  /* Pin 9 input disable. */
        UINT32 PID10 : 1;  /* Pin 10 input disable. */
        UINT32 PID11 : 1;  /* Pin 11 input disable. */
        UINT32 PID12 : 1;  /* Pin 12 input disable. */
        UINT32 PID13 : 1;  /* Pin 13 input disable. */
        UINT32 PID14 : 1;  /* Pin 14 input disable. */
        UINT32 PID15 : 1;  /* Pin 15 input disable. */
        UINT32 PID16 : 1;  /* Pin 16 input disable. */
        UINT32 PID17 : 1;  /* Pin 17 input disable. */
        UINT32 PID18 : 1;  /* Pin 18 input disable. */
        UINT32 PID19 : 1;  /* Pin 19 input disable. */
        UINT32 PID20 : 1;  /* Pin 20 input disable. */
        UINT32 PID21 : 1;  /* Pin 21 input disable. */
        UINT32 PID22 : 1;  /* Pin 22 input disable. */
        UINT32 PID23 : 1;  /* Pin 23 input disable. */
        UINT32 PID24 : 1;  /* Pin 24 input disable. */
        UINT32 PID25 : 1;  /* Pin 25 input disable. */
        UINT32 PID26 : 1;  /* Pin 26 input disable. */
        UINT32 PID27 : 1;  /* Pin 27 input disable. */
        UINT32 PID28 : 1;  /* Pin 28 input disable. */
        UINT32 PID29 : 1;  /* Pin 29 input disable. */
        UINT32 PID30 : 1;  /* Pin 30 input disable. */
        UINT32 PID31 : 1;  /* Pin 31 input disable. */
    } B;
} GPIO_PIDR_t;

/*
 * GPIO_PnDR register bits definition *
 */
typedef union GPIO_PnDR_union_t {
    UINT8 R;
    struct {
        UINT8 PD : 1;  /* Pin n data */
        UINT8 Reserved_1 : 7;  /* Reserved */
    } B;
} GPIO_PnDR_t;

/*
 * GPIO_ICRn register bits definition *
 */
typedef union GPIO_ICRn_union_t {
    UINT32 R;
    struct {
        UINT32 Reserved_15 : 16;  /* Reserved */
        UINT32 IRQC : 4;  /* PCNS register write */
        UINT32 IRQS : 1;  /* Interrupt/DMA line select */
        UINT32 Reserved_21 : 2;  /* Reserved */
        UINT32 LK : 1;  /* ICRn[23:0] lock */
        UINT32 ISF : 1;  /* Interrupt flag */
        UINT32 Reserved_25 : 7;  /* Reserved */
    } B;
} GPIO_ICRn_t;

/*
 * GPIO_GICLR register bits definition *
 */
typedef union GPIO_GICLR_union_t {
    UINT32 R;
    struct {
        UINT32 GIWE0 : 1;  /* Interrupt 0 Write Enable */
        UINT32 GIWE1 : 1;  /* Interrupt 1 Write Enable */
        UINT32 GIWE2 : 1;  /* Interrupt 2 Write Enable */
        UINT32 GIWE3 : 1;  /* Interrupt 3 Write Enable */
        UINT32 GIWE4 : 1;  /* Interrupt 4 Write Enable */
        UINT32 GIWE5 : 1;  /* Interrupt 5 Write Enable */
        UINT32 GIWE6 : 1;  /* Interrupt 6 Write Enable */
        UINT32 GIWE7 : 1;  /* Interrupt 7 Write Enable */
        UINT32 GIWE8 : 1;  /* Interrupt 8 Write Enable */
        UINT32 GIWE9 : 1;  /* Interrupt 9 Write Enable */
        UINT32 GIWE10 : 1;  /* Interrupt 10 Write Enable */
        UINT32 GIWE11 : 1;  /* Interrupt 11 Write Enable */
        UINT32 GIWE12 : 1;  /* Interrupt 12 Write Enable */
        UINT32 GIWE13 : 1;  /* Interrupt 13 Write Enable */
        UINT32 GIWE14 : 1;  /* Interrupt 14 Write Enable */
        UINT32 GIWE15 : 1;  /* Interrupt 15 Write Enable */
        UINT32 GIWD : 16;  /* Interrupt Write Data */
    } B;
} GPIO_GICLR_t;

/*
 * GPIO_GICHR register bits definition *
 */
typedef union GPIO_GICHR_union_t {
    UINT32 R;
    struct {
        UINT32 GIWE16 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE17 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE18 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE19 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE20 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE21 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE22 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE23 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE24 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE25 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE26 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE27 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE28 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE29 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE30 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWE31 : 1;  /* Interrupt n Write Enable */
        UINT32 GIWD : 16;  /* Interrupt Write Data */
    } B;
} GPIO_GICHR_t;

/*
 * GPIO_ISFR0 register bits definition *
 */
typedef union GPIO_ISFR0_union_t {
    UINT32 R;
    struct {
        UINT32 ISF0 : 1;  /* Interupt line 0 status 0. */
        UINT32 ISF1 : 1;  /* Interupt line 0 status 1. */
        UINT32 ISF2 : 1;  /* Interupt line 0 status 2. */
        UINT32 ISF3 : 1;  /* Interupt line 0 status 3. */
        UINT32 ISF4 : 1;  /* Interupt line 0 status 4. */
        UINT32 ISF5 : 1;  /* Interupt line 0 status 5. */
        UINT32 ISF6 : 1;  /* Interupt line 0 status 6. */
        UINT32 ISF7 : 1;  /* Interupt line 0 status 7. */
        UINT32 ISF8 : 1;  /* Interupt line 0 status 8. */
        UINT32 ISF9 : 1;  /* Interupt line 0 status 9. */
        UINT32 ISF10 : 1;  /* Interupt line 0 status 10. */
        UINT32 ISF11 : 1;  /* Interupt line 0 status 11. */
        UINT32 ISF12 : 1;  /* Interupt line 0 status 12. */
        UINT32 ISF13 : 1;  /* Interupt line 0 status 13. */
        UINT32 ISF14 : 1;  /* Interupt line 0 status 14. */
        UINT32 ISF15 : 1;  /* Interupt line 0 status 15. */
        UINT32 ISF16 : 1;  /* Interupt line 0 status 16. */
        UINT32 ISF17 : 1;  /* Interupt line 0 status 17. */
        UINT32 ISF18 : 1;  /* Interupt line 0 status 18. */
        UINT32 ISF19 : 1;  /* Interupt line 0 status 19. */
        UINT32 ISF20 : 1;  /* Interupt line 0 status 20. */
        UINT32 ISF21 : 1;  /* Interupt line 0 status 21. */
        UINT32 ISF22 : 1;  /* Interupt line 0 status 22. */
        UINT32 ISF23 : 1;  /* Interupt line 0 status 23. */
        UINT32 ISF24 : 1;  /* Interupt line 0 status 24. */
        UINT32 ISF25 : 1;  /* Interupt line 0 status 25. */
        UINT32 ISF26 : 1;  /* Interupt line 0 status 26. */
        UINT32 ISF27 : 1;  /* Interupt line 0 status 27. */
        UINT32 ISF28 : 1;  /* Interupt line 0 status 28. */
        UINT32 ISF29 : 1;  /* Interupt line 0 status 29. */
        UINT32 ISF30 : 1;  /* Interupt line 0 status 30. */
        UINT32 ISF31 : 1;  /* Interupt line 0 status 31. */
    } B;
} GPIO_ISFR0_t;

/*
 * GPIO_ISFR1 register bits definition *
 */
typedef union GPIO_ISFR1_union_t {
    UINT32 R;
    struct {
        UINT32 ISF0 : 1;  /* Interupt line 1 status 0. */
        UINT32 ISF1 : 1;  /* Interupt line 1 status 1. */
        UINT32 ISF2 : 1;  /* Interupt line 1 status 2. */
        UINT32 ISF3 : 1;  /* Interupt line 1 status 3. */
        UINT32 ISF4 : 1;  /* Interupt line 1 status 4. */
        UINT32 ISF5 : 1;  /* Interupt line 1 status 5. */
        UINT32 ISF6 : 1;  /* Interupt line 1 status 6. */
        UINT32 ISF7 : 1;  /* Interupt line 1 status 7. */
        UINT32 ISF8 : 1;  /* Interupt line 1 status 8. */
        UINT32 ISF9 : 1;  /* Interupt line 1 status 9. */
        UINT32 ISF10 : 1;  /* Interupt line 1 status 10. */
        UINT32 ISF11 : 1;  /* Interupt line 1 status 11. */
        UINT32 ISF12 : 1;  /* Interupt line 1 status 12. */
        UINT32 ISF13 : 1;  /* Interupt line 1 status 13. */
        UINT32 ISF14 : 1;  /* Interupt line 1 status 14. */
        UINT32 ISF15 : 1;  /* Interupt line 1 status 15. */
        UINT32 ISF16 : 1;  /* Interupt line 1 status 16. */
        UINT32 ISF17 : 1;  /* Interupt line 1 status 17. */
        UINT32 ISF18 : 1;  /* Interupt line 1 status 18. */
        UINT32 ISF19 : 1;  /* Interupt line 1 status 19. */
        UINT32 ISF20 : 1;  /* Interupt line 1 status 20. */
        UINT32 ISF21 : 1;  /* Interupt line 1 status 21. */
        UINT32 ISF22 : 1;  /* Interupt line 1 status 22. */
        UINT32 ISF23 : 1;  /* Interupt line 1 status 23. */
        UINT32 ISF24 : 1;  /* Interupt line 1 status 24. */
        UINT32 ISF25 : 1;  /* Interupt line 1 status 25. */
        UINT32 ISF26 : 1;  /* Interupt line 1 status 26. */
        UINT32 ISF27 : 1;  /* Interupt line 1 status 27. */
        UINT32 ISF28 : 1;  /* Interupt line 1 status 28. */
        UINT32 ISF29 : 1;  /* Interupt line 1 status 29. */
        UINT32 ISF30 : 1;  /* Interupt line 1 status 30. */
        UINT32 ISF31 : 1;  /* Interupt line 1 status 31. */
    } B;
} GPIO_ISFR1_t;

/*
 * GPIO structure definition
 */
typedef struct GPIO_REGS_s {
    GPIO_VERID_t           VERID;                                    /* 0x00000000 Version ID register */
    GPIO_PARAM_t           PARAM;                                    /* 0x00000004 Parameters register */
    UINT32                 Reserved_0x08[1];                         /* 0x00000008 Reserved */
    GPIO_LOCK_t            LOCK;                                     /* 0x0000000C Lock register */
    GPIO_PCNS_t            PCNS;                                     /* 0x00000010 Pin Control Non-Secure access enable register */
    GPIO_ICNS_t            ICNS;                                     /* 0x00000014 Interrupt Control Non-Secure access enable register */
    GPIO_PCNP_t            PCNP;                                     /* 0x00000018 Pin Control Non-Privilege access enable register */
    GPIO_ICNP_t            ICNP;                                     /* 0x0000001C Interrupt Control Non-Privilege access enable register */
    UINT32                 Reserved_0x20[8];                         /* 0x00000020 Reserved */
    GPIO_PDOR_t            PDOR;                                     /* 0x00000040 Port Data Output register */
    GPIO_PSOR_t            PSOR;                                     /* 0x00000044 Port Set Output register */
    GPIO_PCOR_t            PCOR;                                     /* 0x00000048 Port Clear Output register */
    GPIO_PTOR_t            PTOR;                                     /* 0x0000004C Port Toggle Output register */
    GPIO_PDIR_t            PDIR;                                     /* 0x00000050 Port Data Input register */
    GPIO_PDDR_t            PDDR;                                     /* 0x00000054 Port Data Direction register */
    GPIO_PIDR_t            PIDR;                                     /* 0x00000058 Port Input Disable register */
    UINT32                 Reserved_0x5C[1];                         /* 0x0000005C Reserved */
    GPIO_PnDR_t            PnDR[32];                                 /* 0x00000060 Pin n Data register */
    GPIO_ICRn_t            ICRn[32];                                 /* 0x00000080 Interrupt Control register */
    GPIO_GICLR_t           GICLR;                                    /* 0x00000100 Global Interrupt Control Low register */
    GPIO_GICHR_t           GICHR;                                    /* 0x00000104 Global Interrupt Control High register */
    UINT32                 Reserved_0x0108[6];                       /* 0x00000108 Reserved */
    GPIO_ISFR0_t           ISFR0;                                    /* 0x00000120 Interrupt Line 0 Status Flag */
    GPIO_ISFR1_t           ISFR1;                                    /* 0x00000121 Interrupt Line 1 Status Flag */
} volatile GPIO_REGS_t;

/* coverity[divide_by_zero] - Suppress coverity divide_by_zero error */
IO_MAP_STATIC_ASSERT(sizeof(GPIO_REGS_t) == 0x0128);

typedef struct GPIO_REGS_debug_s {
    UINT32 RegOffset;
    char* RegName;
    char* RegDescr;
} GPIO_REGS_debug_t;

/*
 * GPIO debug structure definition
 */
GPIO_REGS_debug_t GPIO_REGS_DebugInfo[] = {
    {0x00000000, "VERID",                       "Version ID register"},
    {0x00000004, "PARAM",                       "Parameters register"},
    {0x0000000C, "LOCK",                        "Lock register"},
    {0x00000010, "PCNS",                        "Pin Control Non-Secure access enable register"},
    {0x00000014, "ICNS",                        "Interrupt Control Non-Secure access enable register"},
    {0x00000018, "PCNP",                        "Pin Control Non-Privilege access enable register"},
    {0x0000001C, "ICNP",                        "Interrupt Control Non-Privilege access enable register"},
    {0x00000040, "PDOR",                        "Port Data Output register"},
    {0x00000044, "PSOR",                        "Port Set Output register"},
    {0x00000048, "PCOR",                        "Port Clear Output register"},
    {0x0000004C, "PTOR",                        "Port Toggle Output register"},
    {0x00000050, "PDIR",                        "Port Data Input register"},
    {0x00000054, "PDDR",                        "Port Data Direction register"},
    {0x00000058, "PIDR",                        "Port Input Disable register"},
    {0x00000060, "PnDR",                        "Pin n Data register"},
    {0x00000080, "ICRn",                        "Interrupt Control register"},
    {0x00000100, "GICLR",                       "Global Interrupt Control Low register"},
    {0x00000104, "GICLH",                       "Global Interrupt Control High register"},
    {0x00000120, "ISFR0",                       "Interrupt Line 0 Status Flag"},
    {0x00000121, "ISFR1",                       "Interrupt Line 1 Status Flag"},
};

#endif /* _GPIO_IO_MAP_H_ */