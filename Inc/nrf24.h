/*
 * nrf24.h
 *
 *  Created on: 09-May-2016
 *      Author: medprime4
 */

#ifndef NRF24_H_
#define NRF24_H_

/* NRF24 definations  */

#define NRF24_CSN_PORT      GPIOB
#define NRF24_CSN_PIN       GPIO_PIN_1

#define NRF24_CE_PORT       GPIOA
#define NRF24_CE_PIN        GPIO_PIN_4

#define NRF24_CSN_LOW()     HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET)
#define NRF24_CSN_HIGH()    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET)

#define NRF24_CE_LOW()      HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET)
#define NRF24_CE_HIGH()     HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET)


/* nRF24L01 Instruction Definitions */
#define WRITE_REG_NRF     	(uint8_t)0x20
#define RD_RX_PLOAD_W   (uint8_t)0x60
#define RD_RX_PLOAD   	(uint8_t)0x61
#define WR_TX_PLOAD   	(uint8_t)0xA0
#define WR_ACK_PLOAD  	(uint8_t)0xA8
#define WR_NAC_TX_PLOAD (uint8_t)0xB0
#define FLUSH_TX      	(uint8_t)0xE1
#define FLUSH_RX      	(uint8_t)0xE2
#define REUSE_TX_PL   	(uint8_t)0xE3
#define NOP           	(uint8_t)0xFF

/* nRF24L01 Register address definitions */
#define CONFIG        (uint8_t)0x00   // Enable/Disable Radio. Set RX/TX Mode
#define EN_AA         (uint8_t)0x01   // Enable/Disable Auto Acknowledgment
#define EN_RXADDR     (uint8_t)0x02   // Enable data Pipes // Default Pipae1 & pipe0 are enable
#define SETUP_AW      (uint8_t)0x03   // RX/TX Address field width '00'-Illegal (common for all data pipes) 3,4,5 Bytes
#define SETUP_RETR    (uint8_t)0x04   // SET Auto Retransmit Delay, Auto Retransmit Count
#define RF_CH         (uint8_t)0x05   // Set the frequency channel nRF24L01+ operates on   0 to 127
#define RF_SETUP      (uint8_t)0x06   // Set Speed 256Kbs, 1,2Mbps   Power 0dB, -6dB, -12dB, -18dB
#define STATUS        (uint8_t)0x07
#define OBSERVE_TX    (uint8_t)0x08
#define RPD           (uint8_t)0x09   // PD   Received Power Detect
#define RX_ADDR_P0    (uint8_t)0x0A   // Pipe0 Address
#define RX_ADDR_P1    (uint8_t)0x0B   // Pipe1 Address
#define RX_ADDR_P2    (uint8_t)0x0C
#define RX_ADDR_P3    (uint8_t)0x0D
#define RX_ADDR_P4    (uint8_t)0x0E
#define RX_ADDR_P5    (uint8_t)0x0F
#define TX_ADDR       (uint8_t)0x10   // PTX Address
#define RX_PW_P0      (uint8_t)0x11   // Pipe0 PayLoad Width
#define RX_PW_P1      (uint8_t)0x12
#define RX_PW_P2      (uint8_t)0x13
#define RX_PW_P3      (uint8_t)0x14
#define RX_PW_P4      (uint8_t)0x15
#define RX_PW_P5      (uint8_t)0x16
#define FIFO_STATUS   (uint8_t)0x17
#define DYNPD         (uint8_t)0x1C   // Dynamic PayLoad SetUp
#define FEATURE       (uint8_t)0x1D   // Enable/Disable PayLoad Dynamic Settings

/********** Register bit definitions **************/
/* STATUS Reg bits */
#define STAT_MAX_RT		(uint8_t)(1 << 4) // Maximum number of TX retransmits interrupt
#define STAT_TX_DS		(uint8_t)(1 << 5) // Data Sent TX FIFO interrupt
#define STAT_RX_DR		(uint8_t)(1 << 6) // Data Ready RX FIFO interrupt
#define STAT_RX_P_NO	(uint8_t)(7 << 1) // Data pipe number for the PayLoad available for reading from RX_FIFO
#define STAT_TX_FULL	(uint8_t)(1 << 0) // TX FIFO full flag 1: TX FIFO full 0: Available locations in TX FIFO.

/* CONFIG register bits */
#define CONFIG_RX_DR    (uint8_t)(1 << 6) // Mask interrupt caused by RX_DR
#define CONFIG_TX_DS    (uint8_t)(1 << 5) // Mask interrupt caused by TX_DS
#define CONFIG_MAX_RT   (uint8_t)(1 << 4) // Mask interrupt caused by MAX_RT
#define CONFIG_EN_CRC   (uint8_t)(1 << 3) // Enable CRC. Forced high if one of the bits in the EN_AA is high
#define CONFIG_CRCO     (uint8_t)(1 << 2) // CRC encoding scheme 1Byte, 2Byte
#define CONFIG_PWR_UP   (uint8_t)(1 << 1) // 1: POWER UP, 0:POWER DOWN
#define CONFIG_PRIM_RX  (uint8_t)(1 << 0) // RX/TX control 1: PRX, 0: PTX

/* RF_SETUP register bit definitions */
#define RF_CONT_WAVE	(uint8_t)(1 << 7) // Enables continuous carrier transmit when high
#define RF_DR_LOW     	(uint8_t)(1 << 5) // Set RF Data Rate to 250kbps
#define RF_PLL_LOCK   	(uint8_t)(1 << 4) // Force PLL lock signal. Only used in test
#define RF_DR_HIGH     	(uint8_t)(1 << 3) // Select between the high speed data rates
#define RF_PWR1       	(uint8_t)(1 << 2) // Set RF output power in TX mode '00' – -18dBm '01' – -12dBm '10' – -6dBm '11' – 0dBm
#define RF_PWR0       	(uint8_t)(1 << 1) //
#define RF_LNA     	 	(uint8_t)(1 << 0) // Obsolete/Don’t care

/* FIFO_STATUS register bits */
#define TX_REUSE 		(uint8_t)(1 << 6) // Used for a PTX device Pulse the CE high for at least 10μs to Reuse last transmitted PayLoad
#define TX_FIFO_FULL  	(uint8_t)(1 << 5) // TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO
#define TX_EMPTY      	(uint8_t)(1 << 4) // TX FIFO empty flag 1: TX FIFO empty 0: Data in TX FIFO.
#define RX_FULL       	(uint8_t)(1 << 1) // RX FIFO full flag 1: RX FIFO full 0: Available locations in RX FIFO.
#define RX_EMPTY      	(uint8_t)(1 << 0) // RX FIFO empty flag 1: RX FIFO empty 0: Data in RX FIFO.


/*FEATURE register bits */
#define EN_DPL      	(uint8_t)(1 << 2) // Enable Dyanamic Payload length.
#define EN_ACK_PAY      (uint8_t)(1 << 1) // Enable Paload with ACK.
#define EN_DYN_ACK      (uint8_t)(1 << 0) //Enable the W_TX_PAYLOAD_NOACK command



/*DYNPD register bits */
#define DPL_P5      	(uint8_t)(1 << 5) // Enable Dyanamic Payload Data pipe5.
#define DPL_P4      	(uint8_t)(1 << 4) // Enable Dyanamic Payload Data pipe4.
#define DPL_P3      	(uint8_t)(1 << 3) // Enable Dyanamic Payload Data pipe3.
#define DPL_P2      	(uint8_t)(1 << 2) // Enable Dyanamic Payload Data pipe2.
#define DPL_P1      	(uint8_t)(1 << 1) // Enable Dyanamic Payload Data pipe1.
#define DPL_P0      	(uint8_t)(1 << 0) // Enable Dyanamic Payload Data pipe0.






/* pipe numbers */
typedef enum {
	Pipe0 = 0,
	Pipe1 ,
	Pipe2,
	Pipe3,
	Pipe4,
	Pipe5

} Pipe_No;

/* Operation mode */
typedef enum {
	TX_Mode = 0,
	RX_Mode
} NRF24_Mode;

/* Output power modes */
typedef enum {
	PWR_18dB = 0,
	PWR_12dB,
	PWR_6dB,
	PWR_0dB
} NRF24_Power_Level;

/* data rate */
typedef enum {
	Data_Rate_256Kbs = 0,
	Data_Rate_1Mbs,
	Data_Rate_2Mbs
} NRF24_Data_Rate;

typedef enum {
	Disable_CRC= 0,
	One_Byte_CRC ,
	Two_Byte_CRC
} NRF24_CRC_Length;

typedef enum {
	D250us = 0,
	D500us ,
	D750us,
	D1000us,
	D1250us,
	D1500us,
	D1750us,
	D2000us,
	D2250us,
	D2500us,
	D2750us,
	D3000us,
	D3250us,
	D3500us,
	D3750us,
	D4000us

} ARD_Delay;


typedef enum {
	    TX_Max_Retry,
		Tx_Data_Sent,
		Rx_Data_Received
} IRQ_Mask,NRF24_Status;

void NRF24_Init(void);

uint8_t SPI_TxRx(uint8_t reg);

uint8_t NRF24_Read_Register(uint8_t reg);

void NRF24_Write_Register(uint8_t reg, uint8_t val);

void NRF24_Power_Up(void);

void NRF24_Power_Down(void);

void NRF24_Read_Buffer(uint8_t reg, uint8_t *buff_pointer, uint8_t bytes);

void NRF24_Write_Buffer(uint8_t reg, uint8_t *addr_pointer, uint8_t bytes);

void NRF24_Set_TX_Addrress(uint8_t *addr_pointer, uint8_t bytes);

uint8_t NRF24_Read_Addrress_Width(void);

void NRF24_Read_TX_Addrress(uint8_t *buff_pointer);


void NRF24_Set_Pipe_Addrress(Pipe_No pipe, uint8_t *addr_pointer, uint8_t bytes);

void NRF24_Read_Pipe_Addrress(Pipe_No pipe, uint8_t *buff_pointer);

void NRF24_Write_TX_Payload(uint8_t *addr_pointer, uint8_t bytes);

void NRF24_Read_RX_Payload(uint8_t *buff_pointer, uint8_t bytes);

void NRF24_Enable_Pipe(Pipe_No pipe);

void NRF24_Disble_Pipe(Pipe_No pipe);

void NRF24_Enable_Auto_ACK(Pipe_No pipe);

void NRF24_Disble_Auto_ACK(Pipe_No pipe);

void NRF24_Set_RF_Channel(uint8_t channel);

uint8_t NRF24_Read_RF_Channel(void);

void NRF24_Set_Pipe_Data_Width(Pipe_No pipe, uint8_t width);

uint8_t NRF24_Read_Pipe_Data_Width(Pipe_No pipe);

void NRF24_Flush_TX(void);

void NRF24_Flush_RX(void);

void NRF24_Set_Power_Level(NRF24_Power_Level level);

NRF24_Power_Level NRF24_Read_Power_Level(void);

void NRF24_Set_Data_Rate(NRF24_Data_Rate speed);

NRF24_Data_Rate NRF24_Read_Data_Rate(void);

void NRF24_Set_CRC_Length(NRF24_CRC_Length length);

NRF24_CRC_Length NRF24_Read_CRC_Length(void);

void NRF24_Set_Retries(ARD_Delay delay, uint8_t count);

void NRF24_Set_Mode(NRF24_Mode mode);

void NRF24_Disable_IRQ(IRQ_Mask mask);

void NRF24_Clear_IRQ(IRQ_Mask mask);

void NRF24_Transmit_Payoad(void);

void NRF24_Enable_Dynamic_Payload(Pipe_No pipe);

void NRF24_Enable_ACK_Payload(void);

void NRF24_Write_ACK_Payload(uint8_t pipe, uint8_t *buf, uint8_t bytes);

uint8_t NRF24_Read_Status(void);





#endif /* NRF24_H_ */
