/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "stm32f10x_type.h"
#include "cortexm3_macro.h"
#include "common.h"

/* macro'd register and peripheral definitions */
#define RCC   ((u32)0x40021000)
#define FLASH ((u32)0x40022000)
#define GPIOA ((u32)0x40010800)
#define GPIOB ((u32)0x40010C00)
#define GPIOC ((u32)0x40011000)
#define GPIOD ((u32)0x40011400)
#define GPIOE ((u32)0x40011800)

#define RCC_CR      RCC
#define RCC_CFGR    (RCC + 0x04)
#define RCC_CIR     (RCC + 0x08)
#define RCC_AHBENR  (RCC + 0x14)
#define RCC_APB2ENR (RCC + 0x18)
#define RCC_APB1ENR (RCC + 0x1C)
#ifdef STM32F10X_CL  
  #define RCC_AHBRSTR 	(RCC + 0x28)
  #define RCC_CFGR2		(RCC + 0x2C)
#endif /* STM32F10X_CL */ 

#define FLASH_ACR     (FLASH + 0x00)
#define FLASH_KEYR    (FLASH + 0x04)
#define FLASH_OPTKEYR (FLASH + 0x08)
#define FLASH_SR      (FLASH + 0x0C)
#define FLASH_CR      (FLASH + 0x10)
#define FLASH_AR      (FLASH + 0x14)
#define FLASH_OBR     (FLASH + 0x1C)
#define FLASH_WRPR    (FLASH + 0x20)

#define FLASH_KEY1     0x45670123
#define FLASH_KEY2     0xCDEF89AB
#define FLASH_RDPRT    0x00A5
#define FLASH_SR_BSY   0x01
#define FLASH_CR_PER   0x02
#define FLASH_CR_PG    0x01
#define FLASH_CR_START 0x40

#define GPIO_CRL(port)  port
#define GPIO_CRH(port)  (port+0x04)
#define GPIO_IDR(port)  (port+0x08)
#define GPIO_ODR(port)  (port+0x0c)
#define GPIO_BSRR(port) (port+0x10)

#define SCS_BASE   ((u32)0xE000E000)
#define NVIC_BASE  (SCS_BASE + 0x0100)
#define SCB_BASE   (SCS_BASE + 0x0D00)


#define SCS      0xE000E000
#define NVIC     (SCS+0x100)
#define SCB      (SCS+0xD00)
#define STK      (SCS+0x10)

#define SCB_VTOR (SCB+0x08)
#define STK_CTRL (STK+0x00)

#define TIM1_APB2_ENB ((u32)0x00000800)
#define TIM1          ((u32)0x40012C00)
#define TIM1_PSC      (TIM1+0x28)
#define TIM1_ARR      (TIM1+0x2C)
#define TIM1_RCR      (TIM1+0x30)
#define TIM1_CR1      (TIM1+0x00)
#define TIM1_CR2      (TIM1+0x04)
#define TIM1_DIER     (TIM1+0x0C)
#define TIM1_UP_IRQ_Channel ((u8)0x19)

#define USB_HP_IRQ  ((u8)0x13)
#define USB_LP_IRQ  ((u8)0x14)
#define TIM2_IRQ    ((u8)0x1C)


/* AIRCR  */
#define AIRCR_RESET         0x05FA0000
#define AIRCR_RESET_REQ     (AIRCR_RESET | (u32)0x04);

/* temporary copyage of example from kiel */
#define __VAL(__TIMCLK, __PERIOD) ((__TIMCLK/1000000UL)*__PERIOD)
#define __PSC(__TIMCLK, __PERIOD)  (((__VAL(__TIMCLK, __PERIOD)+49999UL)/50000UL) - 1)
#define __ARR(__TIMCLK, __PERIOD) ((__VAL(__TIMCLK, __PERIOD)/(__PSC(__TIMCLK, __PERIOD)+1)) - 1)

#define SET_REG(addr,val) do { *(vu32*)(addr)=val; } while(0)
#define GET_REG(addr)     (*(vu32*)(addr))

#define  RCC_CR_HSEON       ((u32)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSION       ((u32)0x00000001)        /*!< Internal High Speed clock enable */
#define  RCC_CR_HSIRDY      ((u32)0x00000002)        /*!< Internal High Speed clock ready flag */
#define  RCC_CR_HSITRIM     ((u32)0x000000F8)        /*!< Internal High Speed clock trimming */
#define  RCC_CR_HSICAL      ((u32)0x0000FF00)        /*!< Internal High Speed clock Calibration */
#define  RCC_CR_HSEON       ((u32)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSERDY      ((u32)0x00020000)        /*!< External High Speed clock ready flag */
#define  RCC_CR_HSEBYP      ((u32)0x00040000)        /*!< External High Speed clock Bypass */
#define  RCC_CR_CSSON       ((u32)0x00080000)        /*!< Clock Security System enable */
#define  RCC_CR_PLLON       ((u32)0x01000000)        /*!< PLL enable */
#define  RCC_CR_PLLRDY      ((u32)0x02000000)        /*!< PLL clock ready flag */
#define  RCC_CR_PLL2ON      ((u32)0x04000000)        /*!< PLL2 enable */
#define  RCC_CR_PLL2RDY     ((u32)0x08000000)        /*!< PLL2 clock ready flag */
#define  RCC_CR_PLL3ON      ((u32)0x10000000)        /*!< PLL3 enable */
#define  RCC_CR_PLL3RDY     ((u32)0x20000000)        /*!< PLL3 clock ready flag */
#define  HSEStartUp_TimeOut   ((u16)0x500)
#define  RCC_CFGR_SW                         ((u32)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_PLL                     ((u32)0x00000002)        /*!< PLL selected as system clock */
#define  RCC_CFGR_SWS                        ((u32)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */

#define  FLASH_ACR_LATENCY                   ((u8)0x03)               /*!<LATENCY[2:0] bits (Latency) */
#define  FLASH_ACR_LATENCY_0                 ((u8)0x00)               /*!<Bit 0 */
#define  FLASH_ACR_LATENCY_1                 ((u8)0x01)               /*!<Bit 0 */
#define  FLASH_ACR_LATENCY_2                 ((u8)0x02)               /*!<Bit 1 */

#define  FLASH_ACR_HLFCYA                    ((u8)0x08)               /*!<Flash Half Cycle Access Enable */
#define  FLASH_ACR_PRFTBE                    ((u8)0x10)               /*!<Prefetch Buffer Enable */
#define  FLASH_ACR_PRFTBS                    ((u8)0x20)               /*!<Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define  FLASH_KEYR_FKEYR                    ((u8)0xFFFFFFFF)        /*!<FPEC Key */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define  FLASH_OPTKEYR_OPTKEYR               ((u8)0xFFFFFFFF)        /*!<Option Byte Key */

/******************  Bit definition for FLASH_SR register  *******************/
#define  FLASH_SR_BSY                        ((u8)0x01)               /*!<Busy */
#define  FLASH_SR_PGERR                      ((u8)0x04)               /*!<Programming Error */
#define  FLASH_SR_WRPRTERR                   ((u8)0x10)               /*!<Write Protection Error */
#define  FLASH_SR_EOP                        ((u8)0x20)               /*!<End of operation */

#define  RCC_CFGR_HPRE_DIV1                  ((u32)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((u32)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((u32)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((u32)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((u32)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((u32)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_PPRE2_DIV1                 ((u32)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((u32)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFGR2_PREDIV2					((u32)0x000000F0)		  /*!< PREDIV2[3:0] bits */
#define  RCC_CFGR2_PLL2MUL					((u32)0x00000F00)		  /*!< PLL2MUL[3:0] bits */
#define  RCC_CFGR2_PREDIV1					((u32)0x0000000F)		  /*!< PREDIV1[3:0] bits */
#define  RCC_CFGR2_PREDIV1SRC				((u32)0x00010000)		  /*!< PREDIV1 entry clock source */
#define  RCC_CFGR2_PREDIV2_DIV5 			((u32)0x00000040)		  /*!< PREDIV2 input clock divided by 5 */
#define  RCC_CFGR2_PLL2MUL8 				((u32)0x00000600)		  /*!< PLL2 input clock * 8 */
#define  RCC_CFGR2_PREDIV1SRC_PLL2			((u32)0x00010000)		  /*!< PLL2 selected as PREDIV1 entry clock source */
#define  RCC_CFGR2_PREDIV1_DIV5 			((u32)0x00000004)		  /*!< PREDIV1 input clock divided by 5 */
#define  RCC_CFGR_PLLXTPRE                   ((u32)0x00020000)        /*!< HSE divider for PLL entry */
#define  RCC_CFGR_PLLSRC                     ((u32)0x00010000)        /*!< PLL entry clock source */
#define  RCC_CFGR_PLLMULL                    ((u32)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLXTPRE_PREDIV1			((u32)0x00000000)		  /*!< PREDIV1 clock not divided for PLL entry */
#define  RCC_CFGR_PLLSRC_PREDIV1			((u32)0x00010000)		  /*!< PREDIV1 clock selected as PLL entry clock source */
#define  RCC_CFGR_PLLMULL9					((u32)0x001C0000)		  /*!< PLL input clock*9 */

/* todo: there must be some major misunderstanding in how we access
   regs. The direct access approach (GET_REG) causes the usb init to
   fail upon trying to activate RCC_APB1 |= 0x00800000. However, using
   the struct approach from ST, it works fine...temporarily switching
   to that approach */
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_RegStruct;
#define pRCC ((RCC_RegStruct *) RCC)

typedef struct {
  vu32 ISER[2];
  u32  RESERVED0[30];
  vu32 ICER[2];
  u32  RSERVED1[30];
  vu32 ISPR[2];
  u32  RESERVED2[30];
  vu32 ICPR[2];
  u32  RESERVED3[30];
  vu32 IABR[2];
  u32  RESERVED4[62];
  vu32 IPR[15];
} NVIC_TypeDef;

typedef struct {
  u8 NVIC_IRQChannel;
  u8 NVIC_IRQChannelPreemptionPriority;
  u8 NVIC_IRQChannelSubPriority;
  bool NVIC_IRQChannelCmd; /* TRUE for enable */
} NVIC_InitTypeDef;

typedef struct {
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;


void setPin    (u32 bank, u8 pin);
void resetPin  (u32 bank, u8 pin);
bool readPin   (u32 bank, u8 pin);
void strobePin (u32 bank, u8 pin, u8 count, u32 rate);

void systemHardReset(void);
void systemReset   (void);
void setupCLK      (void);
void setupLED      (void);
void setupFLASH    (void);
void setupBUTTON   (void);
bool checkUserCode (u32 usrAddr);
void jumpToUser    (u32 usrAddr);

bool flashWriteWord  (u32 addr, u32 word);
bool flashErasePage  (u32 addr);
bool flashErasePages (u32 addr, u16 n);
void flashLock       (void);
void flashUnlock     (void);
void nvicInit        (NVIC_InitTypeDef*);
void nvicDisableInterrupts(void);

#endif
