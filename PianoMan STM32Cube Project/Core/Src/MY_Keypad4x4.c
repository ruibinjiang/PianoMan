/*
Library:						4x4 Keypad drive for STM32 MCUs
Written by:					Mohamed Yaqoob
Date written:				03/04/2018
Description:				The MY_Keypad4x4 library consists of the following public functions
										Function(1)- Keypad4x4_Init
										Function(2)- Keypad4x4_ReadKeypad
										Function(3)- Keypad4x4_GetChar
*/

//***** Header files *****//
#include "MY_Keypad4x4.h"
#include "stm32f4xx_hal.h"
#include "main.h"

//***** Library variables *****//
//1. Keypad pinout variable
//static Keypad_WiresTypeDef KeypadStruct;
//2. OUT pins position, or pin number in decimal for use in colomn change function
//static uint8_t OutPositions[4];
/*
static char *Keypad_keys[16] =
{
	"1",
	"2",
	"3",
	"A",
	"4",
	"5",
	"6",
	"B",
	"7",
	"8",
	"9",
	"C",
	"*",
	"0",
	"#",
	"D"
};*/

//***** Functions definition *****//
//Function(1): Set Keypad pins and ports
void Keypad4x4_Init()
{
	//Step(1): Copy the Keypad wirings to the library
	//Step(2): Find the positions of the 4 OUT pins
	//Step(3): Initialise all pins to set all OUT pins to RESET
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
	                          |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
    //        |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_SET);


}
/*
//Function(2): Get pin positions for colomn change use, only for out pins
static void Keypad4x4_FindPins_positions(void)
{
	uint8_t idx=0;
	for(idx=0; idx<16; idx++)
	{
		if(((KeypadStruct.OUT0pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[0] = idx;
		}
		if(((KeypadStruct.OUT1pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[1] = idx;
		}
		if(((KeypadStruct.OUT2pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[2] = idx;
		}
		if(((KeypadStruct.OUT3pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[3] = idx;
		}
	}
}
//Function(3): Change colomn number
static void Keypad4x4_ChangeColomn(uint8_t colNum_0_to_3)
{
	if(colNum_0_to_3==0)
	{
		//Make other colomns floating
		GPIOD->OTYPER |= 0x3F;

		//Set selected colomn
		GPIOD->OTYPER &= ~(1UL << 0);
	}
	else if(colNum_0_to_3==1)
	{
		//Set selected colomn
		GPIOD->OTYPER &= ~(1UL << 1);
		
		//Make other colomns floating
		GPIOD->OTYPER |= (1UL << 0);
		GPIOD->OTYPER |= (1UL << 2);
		GPIOD->OTYPER |= (1UL << 3);
	}
	else if(colNum_0_to_3==2)
	{
		//Set selected colomn
		GPIOD->OTYPER &= ~(1UL << 2);
		
		//Make other colomns floating
		GPIOD->OTYPER |= (1UL << 0);
		GPIOD->OTYPER |= (1UL << 1);
		GPIOD->OTYPER |= (1UL << 3);
	}
	else if(colNum_0_to_3==3)
	{
		//Set selected colomn
		GPIOD->OTYPER &= ~(1UL << 3);
		
		//Make other colomns floating
		GPIOD->OTYPER |= (1UL << 0);
		GPIOD->OTYPER |= (1UL << 1);
		GPIOD->OTYPER |= (1UL << 2);
	}
}
*/
//Function(4): Read active keypad button
void Keypad4x4_ReadKeypad(bool keys[60])
{
	//Step(1): Make Col0 High and check the rows
	//GPIOD->OTYPER |= 0x3F;
	//GPIOD->OTYPER &= ~(1UL << 0);
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
	                          |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin, GPIO_PIN_SET);
	keys[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	keys[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	keys[12] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	keys[18] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	keys[24] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	keys[30] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	keys[36] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
	keys[42] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	keys[48] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	keys[54] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);
	
	//Step(2): Make Col1 High and check the rows
	//GPIOD->OTYPER |= 0x3F;
	//GPIOD->OTYPER &= ~(1UL << 1);
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
	                          |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col2_Pin3_Pin, GPIO_PIN_SET);
	keys[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	keys[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	keys[13] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	keys[19] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	keys[25] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	keys[31] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	keys[37] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
	keys[43] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	keys[49] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	keys[55] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);
	
	//Step(3): Make Col2 High and check the rows
	//GPIOD->OTYPER |= 0x3F;
	//GPIOD->OTYPER &= ~(1UL << 2);
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
	                          |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col3_Pin5_Pin, GPIO_PIN_SET);
	keys[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	keys[8] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	keys[14] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	keys[20] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	keys[26] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	keys[32] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	keys[38] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
	keys[44] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	keys[50] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	keys[56] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);
	
	//Step(4): Make Col3 High and check the rows
	//GPIOD->OTYPER |= 0x3F;
	//GPIOD->OTYPER &= ~(1UL << 3);
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
		                          |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col4_Pin7_Pin, GPIO_PIN_SET);
	keys[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	keys[9] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	keys[15] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	keys[21] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	keys[27] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	keys[33] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	keys[39] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
	keys[45] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	keys[51] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	keys[57] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);

	//GPIOD->OTYPER |= 0x3F;
	//GPIOD->OTYPER &= ~(1UL << 4);
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
			                          |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col5_Pin9_Pin, GPIO_PIN_SET);
	keys[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	keys[10] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	keys[16] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	keys[22] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	keys[28] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	keys[34] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	keys[40] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
	keys[46] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	keys[52] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	keys[58] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);

	//GPIOD->OTYPER |= 0x3F;
	//GPIOD->OTYPER &= ~(1UL << 5);
	HAL_GPIO_WritePin(GPIOE, Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin|Col4_Pin7_Pin
				                          |Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col6_Pin11_Pin, GPIO_PIN_SET);
	keys[5] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	keys[11] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	keys[17] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	keys[23] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	keys[29] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	keys[35] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	keys[41] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
	keys[47] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	keys[53] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	keys[59] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);

}	
//Function(5): Get character
/*
char* Keypad4x4_GetChar(uint8_t keypadSw)
{
	return Keypad_keys[keypadSw];
}*/

