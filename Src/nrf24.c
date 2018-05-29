/*
 * nrf24.c
 *
 *  Created on: Sep 20, 2016
 *      Author: medprime4
 */

#include "nrf24.h"

static GPIO_TypeDef* NRF24_CSN_Port=GPIOB;
static uint16_t      NRF24_CSN_Pin=0;

static GPIO_TypeDef* NRF24_CE_Port=GPIOB;
static uint16_t      NRF24_CE_Pin=0;

#define NRF24_CSN_LOW()     HAL_GPIO_WritePin(NRF24_CSN_Port, NRF24_CSN_Pin, GPIO_PIN_RESET)
#define NRF24_CSN_HIGH()    HAL_GPIO_WritePin(NRF24_CSN_Port, NRF24_CSN_Pin, GPIO_PIN_SET)

#define NRF24_CE_LOW()      HAL_GPIO_WritePin(NRF24_CE_Port, NRF24_CE_Pin, GPIO_PIN_RESET)
#define NRF24_CE_HIGH()     HAL_GPIO_WritePin(NRF24_CE_Port, NRF24_CE_Pin, GPIO_PIN_SET)


void NRF24_Init(uint16_t _NRF24_CSN_Pin, GPIO_TypeDef* _NRF24_CSN_Port, uint16_t _NRF24_CE_Pin, GPIO_TypeDef* _NRF24_CE_Port)
{
	/*************SPI port configured in cube***********************/

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();


	NRF24_CSN_Port=_NRF24_CSN_Port;
	NRF24_CSN_Pin=_NRF24_CSN_Pin;

	NRF24_CE_Port=_NRF24_CE_Port;
	NRF24_CE_Pin=_NRF24_CE_Pin;

	/*Configure GPIO pin Output Level */
	NRF24_CSN_HIGH();
	NRF24_CE_HIGH();

	/*Configure GPIO pins : PAPin PAPin PAPin */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Pin = NRF24_CSN_Pin;
	HAL_GPIO_Init(NRF24_CSN_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = NRF24_CE_Pin;
	HAL_GPIO_Init(NRF24_CE_Port, &GPIO_InitStruct);

}

uint8_t SPI_TxRx(uint8_t reg)
{
	uint8_t TxData;
	uint8_t RxData;
	TxData = reg;
	RxData = 0;
	HAL_SPI_TransmitReceive(&hspi1, &TxData, &RxData, 1, 5);
	return RxData;
}

/* Read from a register */
uint8_t NRF24_Read_Register(uint8_t reg)
{
	uint8_t value = 0;
	if (reg <= 32)
	{
		NRF24_CSN_LOW();
		SPI_TxRx(reg); /* Transmit register to read */
		value = SPI_TxRx(0x00); /* Then get the register value */
		NRF24_CSN_HIGH();
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
	return value;
}

/* Writes to a register */
void NRF24_Write_Register(uint8_t reg, uint8_t val)
{

	if (reg <= 32)
	{
		NRF24_CSN_LOW();
		SPI_TxRx(WRITE_REG_NRF | reg);
		SPI_TxRx(val);
		NRF24_CSN_HIGH();
	}

	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

void NRF24_Power_Up(void)
{
	uint8_t temp;
	temp = NRF24_Read_Register(CONFIG);
	NRF24_Write_Register(CONFIG, (uint8_t) (CONFIG_PWR_UP | temp));
}

void NRF24_Power_Down(void)
{
	uint8_t temp;
	temp = NRF24_Read_Register(CONFIG);
	NRF24_Write_Register(CONFIG, (uint8_t) ((~CONFIG_PWR_UP ) & temp));
}

void NRF24_Read_Buffer(uint8_t reg, uint8_t *buff_pointer, uint8_t bytes)
{
	uint8_t byte_ctr;
	if (bytes <= 32)
	{
		NRF24_CSN_LOW();                     // Set CSN low, init SPI tranaction
		SPI_TxRx(reg);  // Select register to write to and read status byte
		for (byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
		{
			*buff_pointer++ = SPI_TxRx(0x00);
		} // Perform SPI_RW to read byte from nRF24L01
		NRF24_CSN_HIGH();    // Set CSN high again

	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}

}

void NRF24_Write_Buffer(uint8_t reg, uint8_t *addr_pointer, uint8_t bytes)
{
	uint8_t byte_ctr;
	if (bytes <= 32)
	{
		NRF24_CSN_LOW();                     // Set CSN low, init SPI tranaction
		SPI_TxRx(WRITE_REG_NRF | reg);
		for (byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
		{ // then write all byte in buffer(*pBuf)
			SPI_TxRx(*addr_pointer++);
		}
		NRF24_CSN_HIGH();                                  // Set CSN high again

	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}

}

void NRF24_Set_TX_Address(uint8_t *addr_pointer, uint8_t bytes)
{
	switch (bytes)
	{
	case 3:
		NRF24_Write_Register(WRITE_REG_NRF | SETUP_AW, 0x01);
		NRF24_Write_Buffer(TX_ADDR, addr_pointer, bytes);
		break;
	case 4:
		NRF24_Write_Register(WRITE_REG_NRF | SETUP_AW, 0x02);
		NRF24_Write_Buffer(TX_ADDR, addr_pointer, bytes);
		break;
	case 5:
		NRF24_Write_Register(WRITE_REG_NRF | SETUP_AW, 0x03);
		NRF24_Write_Buffer(TX_ADDR, addr_pointer, bytes);
		break;
	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}
}

uint8_t NRF24_Read_Address_Width(void)
{
	uint8_t bytes = 0; //illegal address width
	bytes = NRF24_Read_Register(SETUP_AW);
	switch (bytes)
	{
	case 1:
		bytes = 3;
		break;
	case 2:
		bytes = 4;
		break;
	case 3:
		bytes = 5;
		break;
	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}
	return bytes;
}

void NRF24_Read_TX_Address(uint8_t *buff_pointer)
{
	uint8_t bytes;
	bytes = NRF24_Read_Address_Width();
	switch (bytes)
	{
	case 1:
		NRF24_Read_Buffer(TX_ADDR, buff_pointer, 3);
		break;
	case 2:
		NRF24_Read_Buffer(TX_ADDR, buff_pointer, 4);
		break;
	case 3:
		NRF24_Read_Buffer(TX_ADDR, buff_pointer, 5);
		break;
	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}
}

void NRF24_Set_Pipe_Address(Pipe_No pipe, uint8_t *addr_pointer, uint8_t bytes)
{

	switch (bytes)
	{
	case 3:
		NRF24_Write_Register(WRITE_REG_NRF | SETUP_AW, 0x01);
		break;
	case 4:
		NRF24_Write_Register(WRITE_REG_NRF | SETUP_AW, 0x02);
		break;
	case 5:
		NRF24_Write_Register(WRITE_REG_NRF | SETUP_AW, 0x03);
		break;
	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}

	switch (pipe)
	{
	case Pipe0:
	case Pipe1:
		NRF24_Write_Buffer((RX_ADDR_P0 + pipe), addr_pointer, bytes);
		break;

	case Pipe2:
	case Pipe3:
	case Pipe4:
	case Pipe5:
		NRF24_Write_Buffer((RX_ADDR_P0 + pipe), addr_pointer, 1);
		break;

	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}
}

void NRF24_Read_Pipe_Address(Pipe_No pipe, uint8_t *buff_pointer)
{
	uint8_t bytes;
	bytes = NRF24_Read_Address_Width();   //

	switch (pipe)
	{
	case Pipe0:
	case Pipe1:
		NRF24_Read_Buffer((RX_ADDR_P0 + pipe), buff_pointer, bytes);
		break;

	case Pipe2:
	case Pipe3:
	case Pipe4:
	case Pipe5:
		NRF24_Read_Buffer((RX_ADDR_P0 + pipe), buff_pointer, 1);
		break;

	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}

}
/*
 The width of TX-payload
 is counted from the number of bytes written into the TX FIFO from the MCU. */
void NRF24_Write_TX_Payload(uint8_t *buff_pointer, uint8_t bytes)
{
	if (bytes <= 32)
	{
		NRF24_Write_Buffer(WR_TX_PLOAD, buff_pointer, bytes);
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

void NRF24_Read_RX_Payload(uint8_t *buff_pointer, uint8_t bytes)
{
	if (bytes <= 32)
	{
		NRF24_Read_Buffer(RD_RX_PLOAD, buff_pointer, bytes);
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

void NRF24_Enable_Pipe(Pipe_No pipe)
{
	uint8_t pipes;
	if (pipe <= Pipe5)
	{
		pipes = NRF24_Read_Register(EN_RXADDR);
		pipes |= (pipe << 1);
		NRF24_Write_Register(EN_RXADDR, pipes);
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

void NRF24_Disble_Pipe(Pipe_No pipe)
{
	uint8_t pipes;
	if (pipe <= Pipe5)
	{
		pipes = NRF24_Read_Register(EN_RXADDR);
		pipes &= ~(pipe << 1);
		NRF24_Write_Register(EN_RXADDR, pipes);
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

void NRF24_Enable_Auto_ACK(Pipe_No pipe)
{
	uint8_t pipes;
	if (pipe <= Pipe5)
	{
		pipes = NRF24_Read_Register(EN_AA);
		pipes |= (pipe << 1);
		NRF24_Write_Register(EN_AA, pipes);

	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

void NRF24_Disble_Auto_ACK(Pipe_No pipe)
{
	uint8_t pipes;
	if (pipe <= Pipe5)
	{
		pipes = NRF24_Read_Register(EN_AA);
		pipes &= ~(pipe << 1);
		NRF24_Write_Register(EN_AA, pipes);

	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

void NRF24_Set_RF_Channel(uint8_t channel)
{
	if (channel < 126)
	{
		NRF24_Write_Register(RF_CH, channel);
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

uint8_t NRF24_Read_RF_Channel(void)
{
	uint8_t channel;
	channel = NRF24_Read_Register(RF_CH);

	return channel;
}

void NRF24_Set_Pipe_Data_Width(Pipe_No pipe, uint8_t width)
{
	if (width <= 32)
	{
		NRF24_Write_Register((RX_PW_P0 + pipe), width);

	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

uint8_t NRF24_Read_Pipe_Data_Width(Pipe_No pipe)
{
	uint8_t width;
	width = NRF24_Read_Register((RX_PW_P0 + pipe));

	return width;
}

void NRF24_Flush_TX(void)
{
	NRF24_CSN_LOW();
	SPI_TxRx(FLUSH_TX);
	NRF24_CSN_HIGH();
}

void NRF24_Flush_RX(void)
{
	NRF24_CSN_LOW();
	SPI_TxRx(FLUSH_RX);
	NRF24_CSN_HIGH();
}

uint8_t NRF24_Read_RPD(void)
{
	return (NRF24_Read_Register(RPD) & (0x01));
}

void NRF24_Set_Power_Level(NRF24_Power_Level level)
{
	uint8_t temp;
	temp = (NRF24_Read_Register(RF_SETUP) & (0xF8));
	switch (level)
	{
	case PWR_18dB:
		temp |= (0x00);
		NRF24_Write_Register( RF_SETUP, temp);	// Write it to the chip
		break;

	case PWR_12dB:
		temp |= (0x02);
		NRF24_Write_Register( RF_SETUP, temp);	// Write it to the chip
		break;

	case PWR_6dB:
		temp |= (0x04);
		NRF24_Write_Register( RF_SETUP, temp);	// Write it to the chip
		break;

	case PWR_0dB:
		temp |= (0x06);
		NRF24_Write_Register( RF_SETUP, temp);	// Write it to the chip
		break;

	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}

}

NRF24_Power_Level NRF24_Read_Power_Level(void)
{
	uint8_t temp;
	temp = NRF24_Read_Register(RF_SETUP);
	temp &= (RF_PWR1 | RF_PWR0 );
	temp = temp >> 1;

	return ((NRF24_Power_Level) temp);
}

void NRF24_Set_Data_Rate(NRF24_Data_Rate speed)
{
	uint8_t setup;

	setup = NRF24_Read_Register(RF_SETUP);
	switch (speed)
	{
	case Data_Rate_256Kbs:
		setup &= (~RF_DR_HIGH );
		setup |= RF_DR_LOW;
		NRF24_Write_Register(RF_SETUP, setup);
		break;

	case Data_Rate_1Mbs:
		setup &= ~(RF_DR_LOW | RF_DR_HIGH );
		NRF24_Write_Register(RF_SETUP, setup);
		break;

	case Data_Rate_2Mbs:
		setup &= (~(RF_DR_LOW ));
		setup |= (RF_DR_HIGH );
		NRF24_Write_Register(RF_SETUP, setup);
		break;

	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}
}

NRF24_Data_Rate NRF24_Read_Data_Rate(void)
{
	uint8_t setup, temp = 0;
	setup = NRF24_Read_Register(RF_SETUP);
	setup = (setup & (RF_DR_LOW | RF_DR_HIGH ));
	if ((setup) == 0x20)
	{
		temp = Data_Rate_256Kbs;
	}
	else if ((setup) == 0x00)
	{
		temp = Data_Rate_1Mbs;
	}
	else if ((setup) == 0x08)
	{
		temp = Data_Rate_2Mbs;
	}

	return ((NRF24_Data_Rate) temp);
}

void NRF24_Set_CRC_Length(NRF24_CRC_Length length)
{
	uint8_t config;
	if (length <= Two_Byte_CRC)
	{
		config = NRF24_Read_Register(CONFIG);

		if (length == Disable_CRC)
		{
			config &= ~( CONFIG_CRCO | CONFIG_EN_CRC );
			NRF24_Write_Register( CONFIG, config);
		}

		else if (length == One_Byte_CRC)
		{
			config |= CONFIG_EN_CRC;
			config &= ~(CONFIG_CRCO );
			NRF24_Write_Register( CONFIG, config);
		}
		else if (length == Two_Byte_CRC)
		{
			config |= CONFIG_EN_CRC;
			config |= CONFIG_CRCO;
			NRF24_Write_Register( CONFIG, config);
		}
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif

	}
}

NRF24_CRC_Length NRF24_Read_CRC_Length(void)
{

	uint8_t config, result;
	config = NRF24_Read_Register(CONFIG) & ( CONFIG_CRCO | CONFIG_EN_CRC );
	uint8_t AA = NRF24_Read_Register(EN_AA);

	if ((config & CONFIG_EN_CRC ) || AA)
	{
		if (config & CONFIG_CRCO)
		{
			result = Two_Byte_CRC;
		}
		else
		{
			result = One_Byte_CRC;
		}
	}
	else
	{
		result = Disable_CRC;
	}

	return (NRF24_CRC_Length) result;
}

void NRF24_Set_Retries(ARD_Delay delay, uint8_t count)
{
	if (delay > 15 || count > 15)
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}

	else
	{
		NRF24_Write_Register(SETUP_RETR,
				((delay & 0xf) << 4) | ((count & 0xf) << 0));
	}
}

void NRF24_Set_Mode(NRF24_Mode mode)
{
	uint8_t config;
	config = NRF24_Read_Register(CONFIG);
	switch (mode)
	{
	case TX_Mode:
		NRF24_Write_Register(CONFIG, (uint8_t) (config & ~CONFIG_PRIM_RX ));
		NRF24_CE_LOW();      // Set CE pin Low for transmitter
		break;

	case RX_Mode:
		NRF24_Write_Register(CONFIG, (uint8_t) (config | CONFIG_PRIM_RX ));
		NRF24_CE_HIGH();      // Set CE pin High for receiver
		break;

	default:
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
		break;
	}
}

void NRF24_Disable_IRQ(IRQ_Mask mask)
{
	int8_t config;
	config = NRF24_Read_Register(CONFIG);
	if (mask == TX_Max_Retry)
	{
		NRF24_Write_Register(CONFIG, (config | CONFIG_MAX_RT ));

	}
	else if (mask == Tx_Data_Sent)
	{
		NRF24_Write_Register(CONFIG, (config | CONFIG_TX_DS ));

	}
	else if (mask == Rx_Data_Received)
	{
		NRF24_Write_Register(CONFIG, (config | CONFIG_RX_DR ));

	}
}

void NRF24_Clear_IRQ(IRQ_Mask mask)
{
	uint8_t status;
	status = NRF24_Read_Register(STATUS);
	if (mask == TX_Max_Retry)
	{
		NRF24_Write_Register(STATUS, (status | STAT_MAX_RT ));

	}
	else if (mask == Tx_Data_Sent)
	{
		NRF24_Write_Register(STATUS, (status | STAT_TX_DS ));

	}
	else if (mask == Rx_Data_Received)
	{
		NRF24_Write_Register(STATUS, (status | STAT_RX_DR ));

	}
}

void NRF24_Transmit_Payoad(void)
{
	NRF24_CE_HIGH();
	Delay_us(50); //50usec
	NRF24_CE_LOW();
}

void NRF24_Enable_Dynamic_Payload(Pipe_No pipe)
{
	if (pipe <= Pipe5)
	{
		uint8_t temp;
		temp = NRF24_Read_Register(FEATURE);
		NRF24_Write_Register(FEATURE, (temp | EN_DPL ));
		temp = NRF24_Read_Register(DYNPD);
		NRF24_Write_Register(DYNPD, (temp) | (pipe << 1));
	}

}

void NRF24_Enable_ACK_Payload(void)
{
	uint8_t temp;
	temp = NRF24_Read_Register(FEATURE);
	NRF24_Write_Register(FEATURE, temp | (EN_ACK_PAY ) | (EN_DPL ));
}

void NRF24_Write_ACK_Payload(uint8_t pipe, uint8_t *buf, uint8_t bytes)
{

	if (bytes <= 32 && pipe <= Pipe5)
	{
		NRF24_Write_Buffer((WR_ACK_PLOAD + pipe), buf, bytes);
	}
	else
	{
#ifdef  NRF24_Debug_Enable
		//_Error_Handler(char *file, int line);
#endif
	}
}

NRF24_Status NRF24_Read_Status(void)
{
	uint8_t NRF24_Status;
	uint8_t temp = 0;
	NRF24_Status = NRF24_Read_Register(STATUS);

	if (NRF24_Status & STAT_TX_DS)
	{
		temp = Tx_Data_Sent; //data sent
	}
	if (NRF24_Status & STAT_MAX_RT)
	{
		temp = TX_Max_Retry; //
	}
	if (NRF24_Status & STAT_RX_DR)
	{
		temp = Rx_Data_Received; //
	}

	return temp;
}
