/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "midi.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "MY_CS43L22.h"
#include "MY_Keypad4x4.h"
#include "visEffect.h"
#include "lcd16x2_i2c.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS myFATFS;
FIL myFILE;
FIL myFILE2;
UINT readByte;
UINT writeByte;
char myRdData[15];
uint8_t byteBuf;
uint8_t readmidi[MAXMIDI];
bool mys[60];
//int ktime = 0;
int16_t I2Sdummy[4];
extern uint8_t midifile[];
MIDI_Player *mp;
bool gMode = true;
bool pMode = false;
bool tMode = false;


#define RATE 20000
#define N 100
short int wavetable[N];
#define VOICES 15

#define STEP1 1.05946309436
#define STEP2 (STEP1*STEP1)
#define STEP3 (STEP2*STEP1)
#define STEP4 (STEP3*STEP1)
#define STEP5 (STEP4*STEP1)
#define STEP6 (STEP5*STEP1)
#define STEP7 (STEP6*STEP1)
#define STEP8 (STEP7*STEP1)
#define STEP9 (STEP8*STEP1)

#define A14    ((13.75   * N/RATE) * (1<<16)) /* A0 */
#define A27    ((27.5    * N/RATE) * (1<<16)) /* A1 */
#define A55    ((55.0    * N/RATE) * (1<<16)) /* A2 */
#define A110   ((110.0   * N/RATE) * (1<<16)) /* A3 */
#define A220   ((220.0   * N/RATE) * (1<<16)) /* A4 */
#define A440   ((440.0   * N/RATE) * (1<<16)) /* A5 */
#define A880   ((880.0   * N/RATE) * (1<<16)) /* A6 */
#define A1760  ((1760.0  * N/RATE) * (1<<16)) /* A7 */
#define A3520  ((3520.0  * N/RATE) * (1<<16)) /* A8 */
#define A7040  ((7040.0  * N/RATE) * (1<<16)) /* A9 */
#define A14080 ((14080.0 * N/RATE) * (1<<16)) /* A10 */

const int step[] = {
        A14 / STEP9,    // C                         C-1
        A14 / STEP8,    // C# / Db 1
        A14 / STEP7,    // D       2
        A14 / STEP6,    // D# / Eb 3
        A14 / STEP5,    // E       4
        A14 / STEP4,    // F
        A14 / STEP3,    // F# / Gb
        A14 / STEP2,    // G
        A14 / STEP1,    // G# / Ab
        A14,            // A27                       A0
        A14 * STEP1,    // A# / Bb
        A14 * STEP2,    // B       11
        A14 * STEP3,    // C       12                C0
        A14 * STEP4,    // C# / Db 13
        A14 * STEP5,    // D
        A27 * STEP6,    // D# / Eb
        A27 / STEP5,    // E
        A27 / STEP4,    // F
        A27 / STEP3,    // F# / Gb
        A27 / STEP2,    // G
        A27 / STEP1,    // G# / Ab 20
        A27,            // A27                       A1
        A27 * STEP1,    // A# / Bb
        A27 * STEP2,    // B
        A27 * STEP3,    // C       24                C1
        A27 * STEP4,    // C# / Db 25 &&&&&&&
        A27 * STEP5,    // D       26
        A27 * STEP6,    // D# / Eb 27
        A55 / STEP5,    // E       28
        A55 / STEP4,    // F       29
        A55 / STEP3,    // F# / Gb 30
        A55 / STEP2,    // G       31
        A55 / STEP1,    // G# / Ab 32
        A55,            // A55     33                A2
        A55 * STEP1,    // A# / Bb 34
        A55 * STEP2,    // B       35
        A55 * STEP3,    // C       36                C2
        A55 * STEP4,    // C# / Db 37
        A55 * STEP5,    // D       38
        A55 * STEP6,    // D# / Eb 39
        A110 / STEP5,   // E       40
        A110 / STEP4,   // F       41
        A110 / STEP3,   // F# / Gb 42
        A110 / STEP2,   // G       43
        A110 / STEP1,   // G# / Ab 44
        A110,           // A110    45               A3
        A110 * STEP1,   // A# / Bb 46
        A110 * STEP2,   // B       47
        A110 * STEP3,   // C       48               C3
        A110 * STEP4,   // C# / Db 49
        A110 * STEP5,   // D       50
        A110 * STEP6,   // D# / Eb 51
        A220 / STEP5,   // E       52
        A220 / STEP4,   // F       53
        A220 / STEP3,   // F# / Gb 54
        A220 / STEP2,   // G       55
        A220 / STEP1,   // G# / Ab 56
        A220,           // A220    57               A4
        A220 * STEP1,   // A# / Bb 58
        A220 * STEP2,   // B       59
        A220 * STEP3,   // C (middle C) 60          C4 (element #60)
        A220 * STEP4,   // C# / Db 61
        A220 * STEP5,   // D       62
        A220 * STEP6,   // D# / Eb 63
        A440 / STEP5,   // E       64
        A440 / STEP4,   // F       65
        A440 / STEP3,   // F# / Gb 66
        A440 / STEP2,   // G       67
        A440 / STEP1,   // G# / Ab 68
        A440,           // A440    69               A5
        A440 * STEP1,   // A# / Bb 70
        A440 * STEP2,   // B       71
        A440 * STEP3,   // C       72               C5
        A440 * STEP4,   // C# / Db 73
        A440 * STEP5,   // D       74
        A440 * STEP6,   // D# / Eb 75
        A880 / STEP5,   // E       76
        A880 / STEP4,   // F       77
        A880 / STEP3,   // F# / Gb 78
        A880 / STEP2,   // G       79
        A880 / STEP1,   // G# / Ab 80
        A880,           // A880    81               A6
        A880 * STEP1,   // A# / Bb 82
        A880 * STEP2,   // B       83
        A880 * STEP3,   // C       84   &&&&        C6
        A880 * STEP4,   // C# / Db
        A880 * STEP5,   // D
        A880 * STEP6,   // D# / Eb
        A1760 / STEP5,  // E
        A1760 / STEP4,  // F
        A1760 / STEP3,  // F# / Gb
        A1760 / STEP2,  // G
        A1760 / STEP1,  // G# / Ab
        A1760,          // A1760                   A7
        A1760 * STEP1,  // A# / Bb
        A1760 * STEP2,  // B
        A1760 * STEP3,  // C       96              C7
        A1760 * STEP4,  // C# / Db
        A1760 * STEP5,  // D
        A1760 * STEP6,  // D# / Eb
        A3520 / STEP5,  // E
        A3520 / STEP4,  // F
        A3520 / STEP3,  // F# / Gb
        A3520 / STEP2,  // G
        A3520 / STEP1,  // G# / Ab
        A3520,          // A3520                   A8
        A3520 * STEP1,  // A# / Bb
        A3520 * STEP2,  // B
        A3520 * STEP3,  // C       108             C8
        A3520 * STEP4,  // C# / Db
        A3520 * STEP5,  // D
        A3520 * STEP6,  // D# / Eb
        A7040 / STEP5,  // E
        A7040 / STEP4,  // F
        A7040 / STEP3,  // F# / Gb
        A7040 / STEP2,  // G
        A7040 / STEP1,  // G# / Ab
        A7040,          // A7040                   A9
        A7040 * STEP1,  // A# / Bb
        A7040 * STEP2,  // B
        A7040 * STEP3,  // C       120             C9
        A7040 * STEP4,  // C# / Db
        A7040 * STEP5,  // D
        A7040 * STEP6,  // D# / Eb
        A14080 / STEP5, // E
        A14080 / STEP4, // F
        A14080 / STEP3, // F# / Gb
        A14080 / STEP2, // G       127
};

struct {
    uint8_t in_use;
    uint8_t note;
    uint8_t chan;
    uint8_t volume;
    int     step;
    int     offset;
} voice[VOICES];

struct {
    //uint8_t in_use;
    //uint8_t note;
    int 	time;
    //int     step;
    int     offset;
} kvoice[60];

void TIM6_DAC_IRQHandler(void) {

	clearLED();
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
    TIM6->SR &= ~TIM_SR_UIF;

    int x;
    int sample = 0;
    if(gMode){
        for(x=0; x < 60; x++) {

        		if (mys[x]) {
        				setRed(x);
                		//kvoice[x].time = ktime;
                		kvoice[x].offset += step[x+25];
                		if (kvoice[x].offset >= N<<16)
                		kvoice[x].offset -= N<<16;

                    sample += wavetable[kvoice[x].offset>>16];
                }
        }
    }
    else if(tMode){
    	for(x=0; x < 60; x++) {
    	        	if (mys[x]) {

    	                	//kvoice[x].time = ktime;
    	                	kvoice[x].offset += step[x+25];
    	                	if (kvoice[x].offset >= N<<16)
    	                	kvoice[x].offset -= N<<16;
    	                    sample += wavetable[kvoice[x].offset>>16];
    	            }

    	}
    	for(x=0; x < 15; x++) {
    				if (voice[x].in_use) {
    					setRed(voice[x].note-25);
    				}
    	}
    }
    else{
		for(x=0; x < sizeof(voice)/sizeof(voice[0]); x++) {
			if (voice[x].in_use) {
				setRed(voice[x].note-25);
					voice[x].offset += voice[x].step;
					if (voice[x].offset >= N<<16)
						voice[x].offset -= N<<16;
					sample += wavetable[voice[x].offset>>16];
			}
		}
    }



    sample =  sample / 128 + 2048;
    if (sample > 4095) sample = 4095;
    else if (sample < 0) sample = 0;
    DAC->DHR12R1 = sample;
}

void init_DAC(void) {
	//RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= 0x300;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR &= ~DAC_CR_EN1; //disables the DAC so it can be messed with
    DAC->CR &= ~DAC_CR_BOFF1;
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_TSEL1;//software
    DAC->CR |= DAC_CR_EN1; //re-enables the DAC

}


void init_TIM6(void) {

    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 419;
    TIM6->ARR = ((84000000 / (TIM6->PSC + 1)) / RATE) - 1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    NVIC_SetPriority(TIM6_DAC_IRQn,0);


}

void TIM2_IRQHandler(void)
{
    TIM2->SR &= ~TIM_SR_UIF;
    if(!gMode)
    	midi_play();
}

void init_TIM2(int n) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 83;
    TIM2->ARR = n-1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);

    NVIC_SetPriority(TIM2_IRQn,3);
}


void init_hybrid(void) {
    int x;
    for(x=0; x<N; x++)
        wavetable[x] = 16383  * sin(2 * M_PI * x / N) +  16383.0 * (x - N/2) / (1.0*N);
}
int16_t *channel_wavetable[20]; // It's zero, by default.



// Find the voice current playing a note, and turn it off.
void note_off(int time, int chan, int key, int velo)
{
    int n;
    for(n=0; n<sizeof voice / sizeof voice[0]; n++) {
        if (voice[n].in_use && voice[n].note == key) {
            voice[n].in_use = 0; // disable it first...
            voice[n].chan = 0;   // ...then clear its values
            voice[n].note = key;
            voice[n].step = step[key];
            return;
        }
    }
}

// Find an unused voice, and use it to play a note.
void note_on(int time, int chan, int key, int velo)
{
    if (velo == 0) {
        note_off(time, chan, key, velo);
        return;
    }
    int n;
    for(n=0; n<sizeof voice / sizeof voice[0]; n++) {
        if (voice[n].in_use == 0) {
            voice[n].note = key;
            voice[n].step = step[key];
            voice[n].offset = 0;
            voice[n].chan = chan;
            voice[n].volume = velo;
            voice[n].in_use = 1;
            return;
        }
    }
}

void set_tempo(int time, int value, const MIDI_Header *hdr)
{
    TIM2->ARR = value/hdr->divisions - 1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  if(lcd16x2_i2c_init(&hi2c3)){
  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  }
  lcd16x2_i2c_1stLine();
  lcd16x2_i2c_printf("   Welcome to");
  lcd16x2_i2c_2ndLine();
  lcd16x2_i2c_printf("    PianoMan!");

  visInit();
  Keypad4x4_Init();
  //audio codec inits
  CS43_Init(hi2c1,MODE_ANALOG);
  CS43_SetVolume(60);
  CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
  CS43_Start();
  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)I2Sdummy,4);

  memset(myRdData,0,15);
  memset(readmidi,0,MAXMIDI);

  if(f_mount(&myFATFS, (TCHAR const*)SDPath, 1) == FR_OK){
  	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  	  //char myPath[] = "TEST.TXT\0";
  	  //char myRdData[15];

  	  /*if(f_open(&myFILE, "SD_TEST.TXT", FA_OPEN_ALWAYS | FA_READ) == FR_OK){
  		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  		  if(f_read(&myFILE, &myRdData, 15, &readByte) == FR_OK){
  			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  			lcd16x2_i2c_1stLine();
  			lcd16x2_i2c_printf("                ");
  			lcd16x2_i2c_1stLine();
  			lcd16x2_i2c_printf(myRdData);
  			lcd16x2_i2c_2ndLine();
  			lcd16x2_i2c_printf("                ");
  		  }
  		  f_close(&myFILE);
  	  }

  	  int fr = (int)f_open(&myFILE, "SONG.MID", FA_READ);
  	lcd16x2_i2c_1stLine();
    	  	  	  			lcd16x2_i2c_printf("%d",fr);*/
  	  if(f_open(&myFILE, "SONG.MID", FA_READ) == FR_OK){
  		  	  readByte = 1;
  		  	  int idx = 0;
  	  		  while(readByte&&idx<MAXMIDI-6){
  	  			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  	  			f_read(&myFILE, &byteBuf, 1, &readByte);
  	  			readmidi[idx] = byteBuf;
  	  			idx++;
  	  			//lcd16x2_i2c_1stLine();
  	  			//lcd16x2_i2c_printf("%d",idx);
  	  		  }
  	  		f_close(&myFILE);
  	  }
  	  HAL_Delay(1000);
  	  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
/*
  	  //f_open(&myFILE2, "SD_TEST_2.TXT", FA_CREATE_ALWAYS | FA_WRITE);
  	  if(f_open(&myFILE2, "SD.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
  		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  		  if(f_write(&myFILE2, &myRdData, readByte, &writeByte) == FR_OK){
  			//lcd16x2_i2c_2ndLine();
  			//lcd16x2_i2c_printf("%d",writeByte);
  		  }
  		  f_close(&myFILE2);
  	  }

  	  //f_read(&myFILE, myRdData, 15, &testByte);
  	  HAL_Delay(1000);*/
    }

  //midi inits
   init_hybrid();
   init_DAC();
   //init_DMA();
   init_TIM6();
   //mp = midi_init(midifile);
   mp = midi_init(readmidi);
   // The default rate for a MIDI file is 2 beats per second
   // with 48 ticks per beat.  That's 500000/48 microseconds.
   init_TIM2(10417);

	lcd16x2_i2c_1stLine();
	lcd16x2_i2c_printf("General Mode    ");
	lcd16x2_i2c_2ndLine();
	lcd16x2_i2c_printf("                ");

	gMode = true;
	pMode = false;
	tMode = false;

   /*voice[0].offset = 0;
   voice[0].step = step[60];
   */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for(;;)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(!pMode){
		  Keypad4x4_ReadKeypad(mys);
	  }
	  visHandle();
	  HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin
                          |Col4_Pin7_Pin|Col5_Pin9_Pin|Col6_Pin11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin Col1_Pin1_Pin Col2_Pin3_Pin Col3_Pin5_Pin
                           Col4_Pin7_Pin Col5_Pin9_Pin Col6_Pin11_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|Col1_Pin1_Pin|Col2_Pin3_Pin|Col3_Pin5_Pin
                          |Col4_Pin7_Pin|Col5_Pin9_Pin|Col6_Pin11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           PD4 */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1){
	   //This block will be triggered after pin activated.
		//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		//HAL_Delay(1);
		//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		//lcd16x2_i2c_clear();
		lcd16x2_i2c_1stLine();
		lcd16x2_i2c_printf("General Mode    ");
		lcd16x2_i2c_2ndLine();
		lcd16x2_i2c_printf("                ");
		gMode = true;
		pMode = false;
		tMode = false;
		memset(mys, 0, sizeof mys);
		clearLED();
	}
	else if(GPIO_Pin == GPIO_PIN_2){
		   //This block will be triggered after pin activated.
			//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			//HAL_Delay(1);
			//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		//lcd16x2_i2c_clear();
		lcd16x2_i2c_1stLine();
		lcd16x2_i2c_printf("Playback Mode   ");
		lcd16x2_i2c_2ndLine();
		lcd16x2_i2c_printf("                ");
		gMode = false;
		pMode = true;
		tMode = false;
		memset(mys, 0, sizeof mys);
		clearLED();
		memset(voice, 0, sizeof voice);
		mp->nexttick = MAXTICKS;
		mp = midi_init(readmidi);
		}
	else if(GPIO_Pin == GPIO_PIN_3){
		   //This block will be triggered after pin activated.
			//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			//HAL_Delay(1);
			//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		//lcd16x2_i2c_clear();
		lcd16x2_i2c_1stLine();
		lcd16x2_i2c_printf("Traning Mode    ");
		lcd16x2_i2c_2ndLine();
		lcd16x2_i2c_printf("                ");
		gMode = false;
		pMode = false;
		tMode = true;
		memset(mys, 0, sizeof mys);
		clearLED();
		memset(voice, 0, sizeof voice);
		mp->nexttick = MAXTICKS;
		mp = midi_init(readmidi);
		}
	else{
		//Do not do anything when else.
		__NOP();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
