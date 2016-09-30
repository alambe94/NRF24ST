/*
 * digit_extract.h
 *
 *  Created on: Sep 20, 2016
 *      Author: medprime4
 */

#ifndef DIGIT_EXTRACT_H_
#define DIGIT_EXTRACT_H_
#include "stm32f0xx_hal.h"

void Digit_Extract_Hex(uint16_t val);
void Digit_Extract_Binary(uint16_t digit);
void Digit_Extract_Decimal(int32_t num);



#endif /* DIGIT_EXTRACT_H_ */
