/**
 *******************************************
 * @file    libs.h
 * @brief   Internal header for adding sys libs and defines
 *******************************************
*/

#ifndef LIBS_H_
#define LIBS_H_

#include "main.h"    ///< Main project file
#include <stdlib.h>  ///< Standard library
#include <stdint.h>  ///< Std types
#include <stdbool.h> ///< _Bool to bool
#include <string.h>  ///< Lib for memcpy, strlen, etc
#include <stdio.h>   ///< Lib for sprintf, printf, etc

typedef uint8_t u8_t; 	///< 8-bit unsigned
typedef int8_t i8_t;	///< 8-bit signed
typedef uint16_t u16_t; ///< 16-bit unsigned
typedef int16_t i16_t;	///< 16-bit signed
typedef uint32_t u32_t; ///< 32-bit unsigned
typedef int32_t i32_t;	///< 32-bit signed
typedef float fl_t;	///< float type

#define ClearBit(reg, bit)       reg &= (~(1<<(bit)))   //пример: ClearBit(PORTB, 1); //сбросить 1-й бит PORTB
#define SetBit(reg, bit)          reg |= (1<<(bit))     //пример: SetBit(PORTB, 3); //установить 3-й бит PORTB
#define BitIsClear(reg, bit)    ((reg & (1<<(bit))) == 0)		//пример: if (BitIsClear(PORTB,1)) {...} //если бит очищен
#define BitIsSet(reg, bit)       ((reg & (1<<(bit))) != 0)		//пример: if(BitIsSet(PORTB,2)) {...} //если бит установлен



#endif /* LIBS_H_ */
