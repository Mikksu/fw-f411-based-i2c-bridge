#ifndef __BITBAND_H
#define __BITBAND_H

#include "stm32f4xx_hal.h"

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

/*!< GPIO Data Register mapping */

#ifdef STM32F103xx
#define IDR_OFFSET			8
#define ODR_OFFSET			12
#elif (STM32F407xx | STM32F405xx | STM32F411xE)
#define IDR_OFFSET			0x10
#define ODR_OFFSET			0x14
#endif

#define GPIOA_ODR_Addr    (GPIOA_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40020014	*/
#define GPIOB_ODR_Addr    (GPIOB_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40020414	*/
#define GPIOC_ODR_Addr    (GPIOC_BASE + ODR_OFFSET)				/*!< GPIO Output Data Register Address: 0x40020814	*/
#define GPIOD_ODR_Addr    (GPIOD_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40020C14	*/
#define GPIOE_ODR_Addr    (GPIOE_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40021014	*/
#define GPIOF_ODR_Addr    (GPIOF_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40021414	*/
#define GPIOG_ODR_Addr    (GPIOG_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40021814	*/
#define GPIOH_ODR_Addr    (GPIOH_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40021C14	*/
#define GPIOI_ODR_Addr    (GPIOI_BASE + ODR_OFFSET) 			/*!< GPIO Output Data Register Address: 0x40022014	*/

#define GPIOA_IDR_Addr    (GPIOA_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40020010	*/
#define GPIOB_IDR_Addr    (GPIOB_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40020410	*/
#define GPIOC_IDR_Addr    (GPIOC_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40020810	*/
#define GPIOD_IDR_Addr    (GPIOD_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40020C10	*/
#define GPIOE_IDR_Addr    (GPIOE_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40021010	*/
#define GPIOF_IDR_Addr    (GPIOF_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40021410	*/
#define GPIOG_IDR_Addr    (GPIOG_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40021810	*/
#define GPIOH_IDR_Addr    (GPIOH_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40021C10	*/
#define GPIOI_IDR_Addr    (GPIOI_BASE + IDR_OFFSET) 			/*!< GPIO Input Data Register Address: 0x40022010	*/

/*<! Attendtion: The argument 'pin' must be less then 16 */
#define PASet(pin)   	BIT_ADDR(GPIOA_ODR_Addr, pin)
#define PAGet(pin)   	BIT_ADDR(GPIOA_IDR_Addr, pin)

#define PBSet(pin)   	BIT_ADDR(GPIOB_ODR_Addr, pin)
#define PBGet(pin)    	BIT_ADDR(GPIOB_IDR_Addr, pin)

#define PCSet(pin)   	BIT_ADDR(GPIOC_ODR_Addr, pin)
#define PCGet(pin)    	BIT_ADDR(GPIOC_IDR_Addr, pin)

#define PDSet(pin)   	BIT_ADDR(GPIOD_ODR_Addr, pin)
#define PDGet(pin)    	BIT_ADDR(GPIOD_IDR_Addr, pin)

#define PESet(pin)   	BIT_ADDR(GPIOE_ODR_Addr, pin)
#define PEGet(pin)    	BIT_ADDR(GPIOE_IDR_Addr, pin)

#define PFSet(pin)   	BIT_ADDR(GPIOF_ODR_Addr, pin)
#define PFGet(pin)    	BIT_ADDR(GPIOF_IDR_Addr, pin)

#define PGSet(pin)   	BIT_ADDR(GPIOG_ODR_Addr, pin)
#define PGGet(pin)    	BIT_ADDR(GPIOG_IDR_Addr, pin)

#define PHSet(pin)   	BIT_ADDR(GPIOH_ODR_Addr, pin)
#define PHGet(pin)    	BIT_ADDR(GPIOH_IDR_Addr, pin)

#define PISet(pin)   	BIT_ADDR(GPIOI_ODR_Addr, pin)
#define PIGet(pin)    	BIT_ADDR(GPIOI_IDR_Addr, pin)


#endif
