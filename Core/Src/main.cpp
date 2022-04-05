/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <iostream>
using namespace std;
#include "cube.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Definition des pins de chaque servos
#define LEFT_ARM   TIM_CHANNEL_1	//D9 sur carte fille - D12 sur stm32
#define LEFT_CLAW  TIM_CHANNEL_2 	//D11 sur carte fille - D11 sur stm32
#define RIGHT_ARM  TIM_CHANNEL_3	//D5 sur carte fille - A3 sur stm32
#define RIGHT_CLAW TIM_CHANNEL_4	//D8 sur carte fille - D7 MORPHEO droite

//Definition des rapports cycliques (en us) pour chaque position de chaque pince.
#define LEFT_CLAW_OPEN   1900
#define LEFT_CLAW_CLOSE  1000
#define LEFT_CLAW_FREE   1300
#define LEFT_CLAW_BIGFREE   1450
#define RIGHT_CLAW_OPEN  2000
#define RIGHT_CLAW_CLOSE 1100
#define RIGHT_CLAW_FREE  1350
#define RIGHT_CLAW_BIGFREE 1500

//Definition des rapports cycliques (en us) pour chaque position de chaque bras.
#define LEFT_ARM_0    530
#define LEFT_ARM_90   1420
#define LEFT_ARM_180  2305
#define RIGHT_ARM_0   495
#define RIGHT_ARM_90  1437
#define RIGHT_ARM_180 2435

//Definition du delai entre chaque mouvement
#define DELAY 500

void LeftCWBody();
void LeftAWBody();
void LeftCWSide();
void LeftAWSide();
void RightCWBody();
void RightAWBody();
void RightCWSide();
void RightAWSide();

void ActR2();
void ActRp();
void ActR();
void ActL2();
void ActLp();
void ActL();
void ActU2();
void ActUp();
void ActU();
void ActD2();
void ActDp();
void ActD();
void ActF2();
void ActFp();
void ActF();
void ActB2();
void ActBp();
void ActB();

void TuneLeftArm();
void TuneRightArm();
void TuneLeftClaw();
void TuneRightClaw();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
RubiksCube cube;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buf[128];		//Buffer pour l'envoi de la séquence de coups depuis l'ordinateur via l'UART


/****************************************************************************/
/*								FONCTION SOLVE()							*/
/*	Parametres: 	Aucun													*/
/*																			*/
/*	Return: 		Code d'état de la résolution (0=Résolution terminée; 	*/
/*					1=En attente de la séquence; -1=erreur)					*/
/*																			*/
/*	Effet:			Fonction principale de résolution du cube, permet		*/
/*					d'appeller les bonnes fonctions en fonction de la		*/
/* 					sequence demandée										*/
/****************************************************************************/
unsigned char solve()
{
	  int i=0;
	  HAL_UART_Receive(&huart2,rx_buf,128,100);
	  if(rx_buf[0]=='G' && rx_buf[1]=='O' && rx_buf[2]==':')
	  {
		  printf("===============Debut resolution============\n\r");
		  i=3;
		  while(rx_buf[i]!=0)
		  {
			  if(rx_buf[i]=='R')
			  {
				  //printf("R\n\r");
				  if (rx_buf[i+1] == '2')		//cas R2
				  {
					  printf("R2\r\n");
					  ActR2();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 39)
				  {
					  printf("R'\r\n");
					  ActRp();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 32)
				  {
					  printf("R\r\n");
					  ActR();
					  i+=2;
				  }
				  else
				  {
					  printf("error\r\n");
					  return 1;
					  }
			  }
			  else if(rx_buf[i]=='L')
			  {
				  //printf("L\n\r");
				  if (rx_buf[i+1] == '2')
				  {
					  printf("L2\r\n");
					  ActL2();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 39)
				  {
					  printf("L'\r\n");
					  ActLp();
					  i+=3;
				  }
					  else if (rx_buf[i+1] == 32)
				  {
					  printf("L\r\n");
					  ActL();
					  i+=2;
				  }
				  else
				  {
					  printf("error\r\n");
					  return 1;
				  }
			  }
			  else if(rx_buf[i]=='U')
			  {
				  //printf("U\n\r");
				  if (rx_buf[i+1] == '2')
				  {
					  printf("U2\r\n");
					  ActU2();
					  i+=3;
				  }
					  else if (rx_buf[i+1] == 39)
				  {
					  printf("U'\r\n");
					  ActUp();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 32)
				  {
					  printf("U\r\n");
					  ActU();
					  i+=2;
				  }
				  else
				  {
					  printf("error\r\n");
					  return 1 ;
				  }
			  }
			  else if(rx_buf[i]=='D')
			  {
				  //printf("D\n\r");
				  if (rx_buf[i+1] == '2')
				  {
					  printf("D2\r\n");
					  ActD2();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 39)
				  {
					  printf("D'\r\n");
					  ActDp();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 32)
				  {
					  printf("D\r\n");
					  ActD();
					  i+=2;
				  }
				  else
				  {
					  printf("error\r\n");
					  return 1;
				  }
			  }
			  else if(rx_buf[i]=='F')
			  {
				  //printf("F\n\r");
				  if (rx_buf[i+1] == '2')
				  {
					  printf("F2\r\n");
					  ActF2();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 39)
				  {
					  printf("F'\r\n");
					  ActFp();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 32)
				  {
					  printf("F\r\n");
					  ActF();
					  i+=2;
				  }
				  else
				  {
					  printf("error\r\n");
					  return 1;
				  }
			  }
			  else if(rx_buf[i]=='B')
			  {
				  //printf("B\n\r");
				  if (rx_buf[i+1] == '2')
				  {
					  printf("B2\r\n");
					  ActB2();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 39)
				  {
					  printf("B'\r\n");
					  ActBp();
					  i+=3;
				  }
				  else if (rx_buf[i+1] == 32)
				  {
					  printf("B\r\n");
					  ActB();
					  i+=2;
				  }
				  else
				  {
					  printf("error\r\n");
					  return 1;
				  }
			  }
		  }
	  }
	  else if(rx_buf[0] == 0)
	  {
			//printf("Rentrer une sequence de resolution :\n\r");
			return 1;
	  }
	  else
	  {
		  printf("Incorrect Input\r\n");
	  }
	  return 0;
}


/****************************************************************************/
/*								FONCTION setPWM()							*/
/*	Parametres: 	-timer; indique le timer de la STM32 à utiliser			*/
/*					-channel; indique le channel du timer à utiliser pour	*/
/*							  generer la PWM								*/
/*					-period; indique la periode en ms de la PWM				*/
/*					-pulse; indique la duree de l'etat en ms haut de la PWM	*/
/*																			*/
/*	Return: 		Aucun													*/
/*																			*/
/****************************************************************************/
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
	HAL_TIM_PWM_Stop(&timer, channel);
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);

	HAL_TIM_PWM_Start(&timer, channel);
}


/****************************************************************************/
/*							FONCTION initServosPosition()					*/
/*	Parametres: 	Aucun													*/
/*																			*/
/*	Return: 		Aucun													*/
/*																			*/
/*	Effet:			-Initialise la position des pinces et des bras 			*/
/*					-Permet un reset sans risques pour le robot				*/
/*					-Permet de positionner le cube dans les pinces avant res*/
/****************************************************************************/
void initServosPosition(){
	  setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_OPEN);
	  HAL_Delay(DELAY);
	  setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_OPEN);
	  HAL_Delay(DELAY);
	  setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_90);
	  HAL_Delay(DELAY);
	  setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_90);
	  HAL_Delay(DELAY);
	  setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_FREE);
	  HAL_Delay(DELAY);
	  setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_FREE);
	  HAL_Delay(5000);
	  setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	  HAL_Delay(DELAY);
  	  setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	  HAL_Delay(DELAY);
}


/****************************************************************************/
/*								FONCTION replace()							*/
/*	Parametres: 	Aucun													*/
/*																			*/
/*	Return: 		Aucun													*/
/*																			*/
/*	Effet:			Fonction qui permet au cube de se replacer dans une 	*/
/*					bonne position et donc d'éviter sa chute				*/
/****************************************************************************/
void replace(){
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_BIGFREE);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_BIGFREE);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void LeftCWBody()
{
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_180);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void LeftAWBody()
{
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_0);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void LeftCWSide()
{
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_180);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void LeftAWSide()
{
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_0);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_ARM, 20000, LEFT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void RightCWBody()
{
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_180);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void RightAWBody()
{
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_0);
	HAL_Delay(DELAY);
	setPWM(htim3, LEFT_CLAW, 20000, LEFT_CLAW_CLOSE);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void RightCWSide()
{
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_180);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void RightAWSide()
{
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_0);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_OPEN);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_ARM, 20000, RIGHT_ARM_90);
	HAL_Delay(DELAY);
	setPWM(htim3, RIGHT_CLAW, 20000, RIGHT_CLAW_CLOSE);
	HAL_Delay(DELAY);
}

void ActR2()
{
	RightCWBody();
	replace();
	LeftAWSide();
	replace();
	LeftAWSide();
	replace();
	RightAWBody();
	replace();
	cube.MoveR2();
}

void ActRp()
{
	RightCWBody();
	replace();
	LeftAWSide();
	replace();
	RightAWBody();
	replace();
	cube.MoveRprime();
}

void ActR()
{
	RightCWBody();
	replace();
	LeftCWSide();
	replace();
	RightAWBody();
	replace();
	cube.MoveR();
}

void ActL2()
{
	RightAWBody();
	replace();
	LeftAWSide();
	replace();
	LeftAWSide();
	replace();
	RightCWBody();
	replace();
	cube.MoveL2();
}

void ActLp()
{
	RightAWBody();
	replace();
	LeftAWSide();
	replace();
	RightCWBody();
	replace();
	cube.MoveLprime();
}

void ActL()
{
	RightAWBody();
	replace();
	LeftCWSide();
	replace();
	RightCWBody();
	replace();
	cube.MoveL();
}

void ActU2()
{
	LeftCWBody();
	replace();
	LeftCWBody();
	replace();
	RightCWSide();
	replace();
	RightCWSide();
	replace();
	LeftCWBody();
	replace();
	LeftCWBody();
	replace();
	cube.MoveU2();
}

void ActUp()
{
	LeftCWBody();
	replace();
	LeftCWBody();
	replace();
	RightAWSide();
	replace();
	LeftCWBody();
	replace();
	LeftCWBody();
	replace();
	cube.MoveUprime();
}

void ActU()
{
	LeftCWBody();
	replace();
	LeftCWBody();
	replace();
	RightCWSide();
	replace();
	LeftCWBody();
	replace();
	LeftCWBody();
	replace();
	cube.MoveU();
}

void ActD2()
{
	RightCWSide();
	replace();
	RightCWSide();
	replace();
	cube.MoveD2();
}

void ActDp()
{
	RightAWSide();
	replace();
	cube.MoveDprime();
}

void ActD()
{
	RightCWSide();
	replace();
	cube.MoveD();
}

void ActF2()
{
	RightCWBody();
	replace();
	RightCWBody();
	replace();
	LeftCWSide();
	replace();
	LeftCWSide();
	replace();
	RightCWBody();
	replace();
	RightCWBody();
	replace();
	cube.MoveF2();
}

void ActFp()
{
	RightCWBody();
	replace();
	RightCWBody();
	replace();
	LeftAWSide();
	replace();
	RightCWBody();
	replace();
	RightCWBody();
	replace();
	cube.MoveFprime();
}

void ActF()
{
	RightCWBody();
	replace();
	RightCWBody();
	replace();
	LeftCWSide();
	replace();
	RightCWBody();
	replace();
	RightCWBody();
	replace();
	cube.MoveF();
}

void ActB2()
{
	LeftCWSide();
	replace();
	LeftCWSide();
	replace();
	cube.MoveB2();
}

void ActBp()
{
	LeftAWSide();
	replace();
	cube.MoveBprime();
}

void ActB()
{
	LeftCWSide();
	replace();
	cube.MoveB();
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  cube.affichage();

  HAL_TIM_PWM_Init(&htim3);
  printf("Power ON\r\n");

  for(int i=0; i<128; i++)
  {
  		  rx_buf[i] = 0;
  }
  initServosPosition();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int sol=1;
  while (sol!=0)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int sol = solve();
	  if(sol==0)
	  {
		  printf("Fini \n\r");
		  cube.affichage();
		  break;
	  }

  }
  return 0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
