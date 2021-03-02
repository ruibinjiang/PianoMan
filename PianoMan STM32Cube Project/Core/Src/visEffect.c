/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

*/

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "ws2812b.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// RGB Framebuffers
uint8_t frameBuffer[3*144];

void setKey(bool mys[60]){
	memset(frameBuffer, 0, sizeof frameBuffer);
	int i;
	for(i = 0;i<60;i++){
		if(mys[i]){
			frameBuffer[i*3 + 0] = 0xFF; //R of i
		}
	}
}

//uint8_t frameBuffer2[3*20];
void setRed(uint8_t i){
	//memset(frameBuffer, 0, sizeof frameBuffer);
	frameBuffer[(i*2+23)*3 + 2] = 0x8F; //B of i
	frameBuffer[(i*2+24)*3 + 2] = 0x8F; //B of i+1
}

void clearLED(){
	memset(frameBuffer, 0, sizeof frameBuffer);
}

void visRainbow(uint8_t *frameBuffer, uint32_t frameBufferSize, uint32_t effectLength)
{
	uint32_t i;
	static uint8_t x = 0;

	x += 1;

	if(x == 256*5)
		x = 0;
	i=0;
	frameBuffer[i*3 + 0] = 0xFF; //R of i
	frameBuffer[(i+143)*3 + 2] = 0xFF;
	/*
	for( i = 0; i < frameBufferSize / 3; i++)
	{
		uint32_t color = Wheel(((i * 256) / effectLength + x) & 0xFF);

		frameBuffer[i*3 + 0] = color & 0xFF;
		frameBuffer[i*3 + 1] = 0x00;
		frameBuffer[i*3 + 2] = 0x00;
	}*/
}


// Animate effects
/*void visHandle2()
{
	static uint32_t timestamp;

	if(HAL_GetTick() - timestamp > 10)
	{
		timestamp = HAL_GetTick();

		// Animate next frame, each effect into each output RGB framebuffer
		visRainbow(frameBuffer, sizeof(frameBuffer), 15);
		//visDots(frameBuffer2, sizeof(frameBuffer2), 50, 40);
	}
}*/

void visInit()
{




	//uint8_t i=0;

	// HELP
	// Fill the 8 structures to simulate overhead of 8 paralel strips
	// The pins are not enabled in the WS2812B init. There are enabled only PC0-3
	// The 16 channels are possible at 168MHz with 60% IRQ overhead during data TX

	// 4 paralel output LED strips needs 18% overhead during TX
	// 8 paralel output LED strips overhead is 8us of 30us period which is 28% - see the debug output PD15/13

	// If you need more parallel LED strips, increase the WS2812_BUFFER_COUNT value
		// Set output channel/pin, GPIO_PIN_0 = 0, for GPIO_PIN_5 = 5 - this has to correspond to WS2812B_PINS
	//frameBuffer[i*3 + 0] = 0xFF; //R of i
	//frameBuffer[(i+143)*3 + 2] = 0xFF;
		ws2812b.item[0].channel = 0;

		// Every even output line has second frameBuffer2 with different effect

			// Your RGB framebuffer
			ws2812b.item[0].frameBufferPointer = frameBuffer;
			// RAW size of framebuffer
			ws2812b.item[0].frameBufferSize = sizeof(frameBuffer);


	ws2812b_init();
}


void visHandle()
{

	if(ws2812b.transferComplete)
	{
		// Update your framebuffer here or swap buffers
		//visHandle2();

		// Signal that buffer is changed and transfer new data
		ws2812b.startTransfer = 1;
		ws2812b_handle();
	}
}


