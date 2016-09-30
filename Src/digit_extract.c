/*
 * digit_extract.c
 *
 *  Created on: 09-May-2016
 *      Author: medprime4
 */

#ifndef DIGIT_EXTRACT_C_
#define DIGIT_EXTRACT_C_
#include "digit_extract.h"
#include "stm32f0xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "nrf24l01p.h"
#include "nrf24.h"

char hex_digit[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A',
		'B', 'C', 'D', 'E', 'F' };



/*
void Digit_Extract_Decimal(int32_t val) {

	char digit[]= "000000\0";
	int32_t temp = val;
	int32_t sign;


	if (val != 0) {

		sign=(val&((uint32_t)1<<31));
		if(sign){
			temp=(~val)+1;
			Uart_Tx_Char('-');
		}
		digit[0] = (48 +(temp / 10000));
		temp = temp % 10000;
		digit[1] = (48 +(temp / 1000));
		temp = temp % 1000;
		digit[2] = (48 +(temp / 100));
		temp = temp % 100;
		digit[3] = (48 +(temp / 10));
		digit[4] = (48 +(temp % 10));
		digit[5] = '\0';


	}
	Uart_Tx_String((char*)digit);

}


*/


void Digit_Extract_Decimal(int32_t num) {
	char str[10];
	int i = 0;
	if (num < 0) {
		Uart_Tx_Char('-');
		num *= -1;
	}
	do str[i++] = (num % 10) + '0';
	while ((num /= 10) > 0);
	for (i--; i >= 0; i--){
          Uart_Tx_Char(str[i]);}
}


void Digit_Extract_Hex(uint16_t val) {
	uint8_t digit0,digit1,digit2,digit3;
	digit0=digit1=digit2=digit3=0;
	uint16_t temp;
	if (val != 0) {
		temp = (val & 0xF000);
		digit0 = (temp >> 12);
		temp = (val & 0x0F00);
		digit1 = (temp >> 8);
		temp = (val & 0x00F0);
		digit2 = (temp >> 4);
		digit3 = (val & 0x000F);

}
	Uart_Tx_Char(hex_digit[digit0]);
	Uart_Tx_Char(hex_digit[digit1]);
	Uart_Tx_Char(hex_digit[digit2]);
	Uart_Tx_Char(hex_digit[digit3]);

}

	void Digit_Extract_Binary(uint16_t digit) {

		uint32_t temp, temp2;

		if (digit > 255 && digit ) {
			temp2 = digit;
			for (int i = 0; i <= 15; i++) {
				temp = (temp2 & 0x8000);
				if (temp) {
					temp2 = (temp2 << 1);
					//return 1;
				} else {
					//return 0;
				}
			}

		} else if (digit <= 255) {
			temp2 = digit;
			for (int i = 0; i <= 7; i++) {
				temp = (temp2 & 0x8000);
				if (temp) {
					temp2 = (temp2 << 1);
					//return 1;
				} else {
					//return 0;
				}
			}

		}
	}



#endif /* DIGIT_EXTRACT_C_ */
