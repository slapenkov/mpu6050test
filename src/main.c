//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

/*
 * Some useful links
 * https://vasilisks.wordpress.com/2016/08/05/%D0%BD%D0%B0%D1%81%D1%82%D1%80%D0%BE%D0%B9%D0%BA%D0%B0-eclipse-%D0%BF%D0%BE%D0%B4-stm32/
 * https://mcuoneclipse.com/2016/10/17/tutorial-using-single-wire-output-swo-with-arm-cortex-m-and-eclipse/
 * https://community.particle.io/t/tutorial-using-eclipse-st-link-v2-openocd-to-debug/10042
 * https://github.com/Harinadha/STM32_MPU6050lib
 * http://letanphuc.net/2014/06/stm32-mpu6050-dma-i2c/
 * http://forum.easyelectronics.ru/viewtopic.php?f=35&t=15587
 * */

// ----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "MPU6050.h"
#include "HAL_MPU6050.h"

#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via NONE).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the NONE output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (50u)
#define BLINK_OFF_TICKS (500u)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int16_t AccelGyro[6] = { 0 };

/*typedef struct MpuData{
 int16_t accelX;
 int16_t accelY;
 int16_t accelZ;
 int16_t gyroX;
 int16_t gyroY;
 int16_t gyroZ;
 } mpuData;

 typedef union MpuBuffer{
 mpuData mpuData;
 int16_t accelGyro[6];
 } mpuBuffer;*/

union MpuPacket {
	struct mpuStruct {
		uint8_t header[4];
		int16_t accelGyro[6];
		uint8_t tail[1];

	} mpuPack;
	uint8_t buffer[17];
} packet;

//prototypes
void SetupUSART(void);
void SetupDMA(void);
void USART1_Send(char chr);
void USART1_Send_String(char* str);
void USART1_Send_Buffer(uint8_t* buffer, uint8_t len);

int main(int argc, char* argv[]) {
	// Send a greeting to the trace device (skipped on Release).
	//trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	//trace_printf("System clock: %u Hz\n", SystemCoreClock);

	packet.mpuPack.header[0] = '$';
	packet.mpuPack.header[1] = 'M';
	packet.mpuPack.header[2] = 'P';
	packet.mpuPack.header[3] = 'U';
	packet.mpuPack.tail[0] = '#';

	timer_start();

	blink_led_init();

	uint32_t seconds = 0;
	MPU_I2C_ClockToggling();
	MPU6050_I2C_Init();

	SetupUSART();
	SetupDMA();

	MPU6050_Initialize();

	// Infinite loop
	while (1) {
		blink_led_on();
		//timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);

		//blink_led_off();
		//timer_sleep(BLINK_OFF_TICKS);

		++seconds;

		MPU6050_GetRawAccelGyro(packet.mpuPack.accelGyro);

		// Count seconds on the trace device.
		//trace_printf("Second %u\n", seconds);

		//trace through USART
		USART1_Send_String("Packet: ");
		USART1_Send_Buffer(packet.buffer, 17);
		USART1_Send_String("\r\n");

		/*trace_printf("Some data %i:%i:%i-%i:%i:%i\n", AccelGyro[0],
		 AccelGyro[1], AccelGyro[2], AccelGyro[3], AccelGyro[4],
		 AccelGyro[5]);*/
		blink_led_off();
		timer_sleep(50);
	}
	// Infinite loop, never return.
}

#pragma GCC diagnostic pop

//setup usart
void SetupUSART() {
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	//set clock
	RCC_APB2PeriphClockCmd(
			(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO),
			ENABLE);
	//Init pin PA9 - USART1_Tx
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; //select pin PA9
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; //set max speed
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; //Alternate func, set Push-Pull
	GPIO_Init(GPIOA, &GPIO_InitStruct); //Set GPIOА props

	//Set pin PA10 - USART1_Rx
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; //Select PA10
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Input floating
	GPIO_Init(GPIOA, &GPIO_InitStruct); //Store in GPIOА props

	//Инициализация USART1
	USART_InitStruct.USART_BaudRate = 9600; //Set speed 9600 baud
	USART_InitStruct.USART_WordLength = USART_WordLength_8b; //Length 8 bits
	USART_InitStruct.USART_StopBits = USART_StopBits_1; //1 stop-bit
	USART_InitStruct.USART_Parity = USART_Parity_No; //No parity
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No flow control
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //On transmitter and receiver of USART1
	USART_Init(USART1, &USART_InitStruct); //Store USART1 props

	USART_Cmd(USART1, ENABLE); //On USART1
}

//setup dma
void SetupDMA() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitTypeDef DMA_InitStruct;
	DMA_StructInit(&DMA_InitStruct);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) AccelGyro;
	DMA_InitStruct.DMA_BufferSize = 12;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(DMA1_Channel6, &DMA_InitStruct);

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Channel6, ENABLE);

	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

void USART1_Send(char chr) {
	while (!(USART1->SR & USART_SR_TC))
		;
	USART1->DR = chr;
}

void USART1_Send_String(char* str) {
	int i = 0;
	while (str[i])
		USART1_Send(str[i++]);
}

//send buffer
void USART1_Send_Buffer(uint8_t* buffer, uint8_t len) {
	int i = 0;
	for (i = 0; i < len; i++)
		USART1_Send(buffer[i++]);
}

// ----------------------------------------------------------------------------
