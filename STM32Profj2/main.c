/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LED 24
#define RESET_PULSE 16
#define r 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
//neopixel
static uint8_t spi_bits[NUM_LED*9];
static uint8_t reset_bits[RESET_PULSE]={0,};
//bounceball simul
int32_t rad=0;
int32_t acc[2]={0,0};
int32_t velo[2]={0,0};
int32_t pos[2]={64,32};
uint8_t ballMatrix[]={
		0b00111100,
		0b01111110,
		0b11111111,
		0b11111111,
		0b11111111,
		0b11111111,
		0b01111110,
		0b00111100,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//sh1106
void sh1106_init(void){
	//uint8_t buf;
	const uint8_t init_commands[]={
		0xae, 0x00, 0x10, 0x40, 0x81, 0x80, 0xc0, 0xa8,
		0x3f, 0xd3, 0x00, 0xd5, 0x50, 0xd9, 0x22, 0xda,
		0x12, 0xdb, 0x35, 0xa4, 0xa6, 0xaf};
	//DC_DDR		|= (1<<DC_BIT);
	//OLD_CS_DDR	|= (1<<OLD_CS_BIT);
	//OLD_CS_PORT |= (1<<OLD_CS_BIT);
	//OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0);
	//DC_PORT |= (1<<DC_BIT);
	//DC_PORT &= ~(1<<DC_BIT);
	HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 0);

	for(uint8_t i=0; i<sizeof(init_commands);i++){
		//SpiUSITx(init_commands[i]);
		HAL_SPI_Transmit(&hspi3, &init_commands[i],1,10);
	}
	//OLD_CS_PORT |= (1<<OLD_CS_BIT);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 1);
}

void sh1106_set_location(uint8_t page, uint8_t column){
	uint8_t buf;
	HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 0);

	buf=0xB0|page;
	HAL_SPI_Transmit(&hspi3, &buf,1,10);
	buf=0x00|(column&0x0F);
	HAL_SPI_Transmit(&hspi3, &buf,1,10);
	buf=0x10|((column>>4)&0x0F);
	HAL_SPI_Transmit(&hspi3, &buf,1,10);
}


void sh1106_clear(void){
	uint8_t buf;
	//OLD_CS_PORT |= (1<<OLD_CS_BIT);
	//OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0);

	for(uint8_t page=0;page<8;page++){
		sh1106_set_location(page, 0);
		//DC_PORT |= (1<<DC_BIT);
		HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
		for(uint8_t count=0; count<132;count++){
			buf=0;
			HAL_SPI_Transmit(&hspi3, &buf,1,10);
		}
	}
	//OLD_CS_PORT |= (1<<OLD_CS_BIT);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 1);
}


void sh1106_testpattern(void){
	uint8_t buf;
	//OLD_CS_PORT |= (1<<OLD_CS_BIT);
	//OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0);
	for(uint8_t page=0;page<8;page++){
		sh1106_set_location(page, 34);
		//DC_PORT |= (1<<DC_BIT);
		HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
		for(uint8_t count =0; count<64; count++){
			buf=0xf0;
			HAL_SPI_Transmit(&hspi3, &buf,1,10);
		}
	}
	//OLD_CS_PORT |= (1<<OLD_CS_BIT);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 1);
}


void sh1106_border(void){
	uint8_t buf;
	//OLD_CS_PORT |= (1<<OLD_CS_BIT);
	//OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0);

	sh1106_set_location(0, 0);
	//DC_PORT |= (1<<DC_BIT);
	HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
	for(uint8_t column =0; column<132; column++){
		buf=0x01;
		HAL_SPI_Transmit(&hspi3, &buf,1,10);
	}

	sh1106_set_location(7, 0);
	//DC_PORT |= (1<<DC_BIT);
	HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
	for(uint8_t column =0; column<132; column++){
		buf=0x80;
		HAL_SPI_Transmit(&hspi3, &buf,1,10);
	}


	for(uint8_t page=0; page<8; page++){
		sh1106_set_location(page, 2);
		//DC_PORT |= (1<<DC_BIT);
		HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
		buf=0xff;
		HAL_SPI_Transmit(&hspi3, &buf,1,10);

		sh1106_set_location(page, 129);
		//DC_PORT |= (1<<DC_BIT);
		HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
		buf=0xff;
		HAL_SPI_Transmit(&hspi3, &buf,1,10);
	}

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 1);
}


//neopixel
static void set_spi_bits(uint8_t *buf, uint8_t val){
	uint32_t pulse=0;
	for(unsigned i=0; i<8; i++){
		uint8_t bit=(val>>(7-i))&1;
		pulse = (pulse<<3)|(bit==0 ? 0b100 : 0b110);
	}
	buf[0]=(pulse>>16)&0xff;
	buf[1]=(pulse>>8)&0xff;
	buf[2]=(pulse>>0)&0xff;
}

//bounceball simul
void drawBall(int16_t x, int16_t y){//r:4고정
	if(x<4) x=4;
	if(x>124) x=124;
	if(y<4) y=4;
	if(y>60) y=60;
	uint8_t buf[8];
	uint8_t i;
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0);
	sh1106_set_location((y-r)/8, x-r+2);

	HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
	if((y-r)%8==0){
		//for(i=0;i<8;i++){
		HAL_SPI_Transmit(&hspi3, ballMatrix, 8, 10);
		//}
	}
	else{
		for(i=0;i<8;i++){
			buf[i]=ballMatrix[i]<<((y-r)%8);
		}
		HAL_SPI_Transmit(&hspi3, buf, 8, 10);

		if ((y-r)/8+1!=8){
		sh1106_set_location((y-r)/8+1, x-r+2);
		HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 1);
		for(i=0;i<8;i++){
			buf[i]=ballMatrix[i]>>(8-((y-r)%8));
		}
		HAL_SPI_Transmit(&hspi3, buf, 8, 10);
		}
	}

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 1);
}

void setRad(uint8_t encoderPos){
	rad=encoderPos*(3.14159265/12.0);
}
void setAcc(){
	//deg삼각함수로
	acc[0]=cos(rad)*1.5;
	acc[1]=sin(rad)*1.5;
}
void setVelo(){
	velo[0]+=acc[0];
	velo[1]+=acc[1];
}
void setPos(){
	pos[0]+=velo[0];
	pos[1]+=velo[1];
	if(pos[0]<0)  pos[0]=0;
	if(pos[0]>128)  pos[0]=128;
	if(pos[1]<0)  pos[1]=0;
	if(pos[1]>64)  pos[1]=64;
}
void detactWall(){
	if(pos[0]>=(128-r)||pos[0]<=r){
		velo[0]= -velo[0]*0.75;
		//velo[1]=velo[1]*0.999;
	}
	if(pos[1]>=(64-r)||pos[1]<=r){
		velo[1]= -velo[1]*0.75;
		//velo[0]=velo[0]*0.999;
	}
}

void ball_init(){
	drawBall(64,32);//128x64화면..대충 중간.
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_Delay(10);
  sh1106_init();
  sh1106_clear();
  //sh1106_testpattern();
  //sh1106_border();

  //rotary encoder
  uint8_t position=0;
  uint8_t positionNow=0;

  //bounceball simul
  ball_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0);
	HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 0);


	//bounceball simul
	//position=12;
	setRad(position);
	setAcc();
	setVelo();
	setPos();
	detactWall();
	drawBall(pos[0]/1,pos[1]/1);

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 1);
	HAL_Delay(10);

	//encoder
	positionNow=TIM1->CNT;
	if(positionNow!=position){
		position=positionNow;
		for(unsigned i=0; i<NUM_LED; i++){
		  set_spi_bits(spi_bits+i*9+0, 0);//g
		  set_spi_bits(spi_bits+i*9+3, 0);//b
		  set_spi_bits(spi_bits+i*9+6, 0);//r
		}
		//set_spi_bits(spi_bits+i*9+c*3, 0x80);
		set_spi_bits(spi_bits+position*9+0, 0x04);
		HAL_SPI_Transmit(&hspi1, spi_bits, sizeof(spi_bits),10);
		HAL_SPI_Transmit(&hspi1, reset_bits, sizeof(reset_bits),10);
	}
	HAL_Delay(30);

	//display clear
	sh1106_clear();

	/* USER CODE END 3 */
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0);
	HAL_GPIO_WritePin(SPI3_DC_GPIO_Port, SPI3_DC_Pin, 0);


	//bounceball simul
	//position=12;
	setRad(position);
	setAcc();
	setVelo();
	setPos();
	detactWall();
	drawBall(pos[0]/1,pos[1]/1);

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, 1);
	HAL_Delay(10);

	//encoder
	positionNow=TIM1->CNT;
	if(positionNow!=position){
		position=positionNow;
		for(unsigned i=0; i<NUM_LED; i++){
		  set_spi_bits(spi_bits+i*9+0, 0);//g
		  set_spi_bits(spi_bits+i*9+3, 0);//b
		  set_spi_bits(spi_bits+i*9+6, 0);//r
		}
		//set_spi_bits(spi_bits+i*9+c*3, 0x80);
		set_spi_bits(spi_bits+position*9+0, 0x04);
		HAL_SPI_Transmit(&hspi1, spi_bits, sizeof(spi_bits),10);
		HAL_SPI_Transmit(&hspi1, reset_bits, sizeof(reset_bits),10);
	}
	HAL_Delay(50);

	//display clear
	sh1106_clear();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 23;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 12;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 12;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI3_CS_Pin|SPI3_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SPI3_CS_Pin SPI3_DC_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin|SPI3_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
