/*****************************************************************************
@File name: MyTypeDef.h 
@Description: 
@Author: Harry Wu
@Version: V1.0
@Date: 2016-8-23
@History: 
*****************************************************************************/

#ifndef _MYTYPEDEF_H_
#define _MYTYPEDEF_H_

#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include "system_stm32f4xx.h"
#include <stdint.h>

/** @addtogroup Exported_types
  * @{
  */  
/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

typedef enum
{
        BOOL_FALSE = 0,
        BOOL_TRUE  = 1
}BOOLEAN;

//typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

//typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
//#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#endif
