/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    int column;
    int row;
} move_typedef;

typedef enum
{
	MENU_CHOOSE_MODE,
	MENU_CHOOSE_DIFF,
	PLAYER_AI,
	PLAYER_PLAYER,
	GAME_OVER
} state_typedef;

typedef enum
{
	PLAYER_1,
	PLAYER_2
} player_turn_typedef;

typedef enum
{
	RELEASED,
	PRESSED
} button_state_typedef;

typedef enum
{
	ZERO,
	POS,
	NEG
} adc_state_typedef;

typedef enum
{
	EASY,
	HARD
} ai_mode_typedef;

typedef enum
{
	PLAYER_1_WON,
	PLAYER_2_WON,
	AI_WON,
	DRAW
} winner_typedef;

typedef enum
{
	POS_1,
	POS_2
} lcd_cursor_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDRESS 0x27<<1
#define LCD_RS 0
#define LCD_RW 1
#define LCD_EN 2

#define cursor_symbol 'z'
#define player 'x'
#define computer 'o'
#define computer_turn 1
#define player_turn 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

osThreadId task_1Handle;
osThreadId task_2Handle;
osThreadId task_3Handle;
/* USER CODE BEGIN PV */
volatile char board[3][3];

volatile move_typedef best_move = {
		.row = -1,
		.column = -1
};
volatile move_typedef cursor = {
		.row = 0,
		.column = 0
};

volatile lcd_cursor_typedef lcd_cursor = POS_1;
volatile state_typedef cur_state = MENU_CHOOSE_MODE;
volatile player_turn_typedef turn = PLAYER_1;
volatile ai_mode_typedef ai_mode = EASY;
volatile winner_typedef winner = PLAYER_1_WON;
volatile uint8_t rand = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void task_1_function(void const * argument);
void task_2_function(void const * argument);
void task_3_function(void const * argument);

/* USER CODE BEGIN PFP */
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_init(void);
void lcd_clear(void);
void lcd_string(char str[]);

uint8_t check_moves_left(char board[][3]);
int evaluate(char board[][3]);
int minimax(char board[][3], uint8_t turn);
void find_best_move(char board[][3]);
int max(int a, int b);
int min(int a, int b);
void board_clear(char board[][3]);
void board_copy(char board[][3], char copy[]);
void board_to_pc(char board[][3], move_typedef *cursor);
void board_easy_mode(char board[][3], uint8_t rand);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of task_1 */
  osThreadDef(task_1, task_1_function, osPriorityNormal, 0, 128);
  task_1Handle = osThreadCreate(osThread(task_1), NULL);

  /* definition and creation of task_2 */
  osThreadDef(task_2, task_2_function, osPriorityNormal, 0, 128);
  task_2Handle = osThreadCreate(osThread(task_2), NULL);

  /* definition and creation of task_3 */
  osThreadDef(task_3, task_3_function, osPriorityBelowNormal, 0, 128);
  task_3Handle = osThreadCreate(osThread(task_3), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void lcd_command(uint8_t cmd)
{
	uint8_t buff[4];
	buff[0] = (0 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (cmd & 0xF0) | 0x08;
	buff[1] = (0 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (cmd & 0xF0) | 0x08;
	buff[2] = (0 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (cmd << 4) | 0x08;
	buff[3] = (0 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (cmd << 4) | 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, buff, 4, 100);
	osDelay(1);
}

void lcd_data(uint8_t data)
{
	uint8_t buff[4];
	buff[0] = (1 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (data & 0xF0) | 0x08;
	buff[1] = (1 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (data & 0xF0) | 0x08;
	buff[2] = (1 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (data << 4) | 0x08;
	buff[3] = (1 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (data << 4) | 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, buff, 4, 100);
	osDelay(1);
}

void lcd_init(void)
{
	osDelay(50);
	lcd_command(0x30);
	osDelay(5);
	lcd_command(0x30);
	lcd_command(0x30);
	lcd_command(0x20);

	lcd_command(0x28);
	lcd_command(0x01);
	osDelay(2);
	lcd_command(0x0E);
	lcd_command(0x06);
}

void lcd_clear(void)
{
	lcd_command(0x01);
	osDelay(2);
}

void lcd_string(char str[])
{
	uint8_t i = 0;
	while (str[i] != '\0')
	{
		lcd_data(str[i]);
		i++;
	}
}

uint8_t check_moves_left(char board[][3])
{
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            if (board[i][j] == '_')
            {
                return 1;
            }
        }
    }
    return 0;
}

int evaluate(char board[][3])
{
    // Check row
    for (uint8_t i = 0; i < 3; i++)
    {
        if ((board[i][0] == board[i][1]) && (board[i][0] == board[i][2]))
        {
            if (board[i][0] == computer)
            {
                return 10;
            }
            else if (board[i][0] == player)
            {
                return -10;
            }
        }
    }

    // Check column
    for (uint8_t i = 0; i < 3; i++)
    {
        if ((board[0][i] == board[1][i]) && (board[0][i] == board[2][i]))
        {
            if (board[0][i] == computer)
            {
                return 10;
            }
            else if (board[0][i] == player)
            {
                return -10;
            }
        }
    }

    // Check diagonal
    if ((board[0][0] == board[1][1]) && (board[0][0] == board[2][2]))
    {
        if (board[0][0] == computer)
        {
            return 10;
        }
        else if (board[0][0] == player)
        {
            return -10;
        }
    }

    if ((board[0][2] == board[1][1]) && (board[0][2] == board[2][0]))
    {
        if (board[0][2] == computer)
        {
            return 10;
        }
        else if (board[0][2] == player)
        {
            return -10;
        }
    }

    return 0;
}

int minimax(char board[][3], uint8_t turn)
{
    int score = evaluate(board);

    if ((score == 10) || (score == -10))
    {
        return score;
    }

    if (!check_moves_left(board))
    {
        return 0;
    }

    if (turn == computer_turn)
    {
        int best = -1000;
        for (uint8_t i = 0; i < 3; i++)
        {
            for (uint8_t j = 0; j < 3; j++)
            {
                if (board[i][j] == '_')
                {
                    board[i][j] = computer;
                    best = max(best, minimax(board, player_turn));
                    board[i][j] = '_';
                }
            }
        }
        return best;
    }

    if (turn == player_turn)
    {
        int best = 1000;
        for (uint8_t i = 0; i < 3; i++)
        {
            for (uint8_t j = 0; j < 3; j++)
            {
                if (board[i][j] == '_')
                {
                    board[i][j] = player;
                    best = min(best, minimax(board, computer_turn));
                    board[i][j] = '_';
                }
            }
        }
        return best;
    }

    return 0;
}

void find_best_move(char board[][3])
{
    int best = -1000;
    best_move.column = -1;
    best_move.row = -1;

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            if (board[i][j] == '_')
            {
                board[i][j] = computer;
                int val = minimax(board, player_turn);
                board[i][j] = '_';
                if (val > best)
                {
                    best_move.row = i;
                    best_move.column = j;
                    best = val;
                }
            }
        }
    }
}

int max(int a, int b)
{
    if (a > b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

int min(int a, int b)
{
    if (a < b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

void board_clear(char board[][3])
{
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            board[i][j] = '_';
        }
    }
}

void board_copy(char board[][3], char copy[])
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            copy[n] = board[i][j];
            n++;
        }
    }
}

void board_to_pc(char board[][3], move_typedef *cursor)
{
	char buff[11];
	buff[10] = '\n';
	board_copy(board, buff);
	buff[9] = cursor->row * 3 + cursor->column + '0';
	HAL_UART_Transmit(&huart1, buff, sizeof(buff), 100);
}

void board_easy_mode(char board[][3], uint8_t rand)
{
	if (rand % 2)
	{
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
				if (board[i][j] == '_')
				{
					board[i][j] = computer;
					return;
				}
			}
		}
	}
	else
	{
		for (uint8_t j = 0; j < 3; j++)
		{
			for (uint8_t i = 0; i < 3; i++)
			{
				if (board[i][j] == '_')
				{
					board[i][j] = computer;
					return;
				}
			}
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_1_function */
/**
  * @brief  Function implementing the task_1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_1_function */
void task_1_function(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	rand++;
	board_to_pc(board, &cursor);
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_2_function */
/**
* @brief Function implementing the task_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_2_function */
void task_2_function(void const * argument)
{
  /* USER CODE BEGIN task_2_function */
  lcd_init();
  /* Infinite loop */
  for(;;)
  {
	switch(cur_state)
	{
	case MENU_CHOOSE_MODE:
		lcd_command(0x0E);
		lcd_command(0x80);
		lcd_string("Choose a mode:  ");
		lcd_command(0xC0);
		lcd_string("1:1P 2:2P       ");
		if (lcd_cursor == POS_1)
		{
			lcd_command(0xC0);
		}
		else
		{
			lcd_command(0xC5);
		}
		break;
	case MENU_CHOOSE_DIFF:
		lcd_command(0x0E);
		lcd_command(0x80);
		lcd_string("Difficulty:     ");
		lcd_command(0xC0);
		lcd_string("1:Easy 2:Hard   ");
		if (lcd_cursor == POS_1)
		{
			lcd_command(0xC0);
		}
		else
		{
			lcd_command(0xC7);
		}
		break;
	case PLAYER_AI:
		lcd_command(0x0C);
		lcd_command(0x80);
		lcd_string("     Playing    ");
		lcd_command(0xC0);
		lcd_string("  against AI... ");
		break;
	case PLAYER_PLAYER:
		lcd_command(0x0C);
		lcd_command(0x80);
		lcd_string("Waiting for     ");
		lcd_command(0xC0);
		if (turn == PLAYER_1)
		{
			lcd_string("Player 1...     ");
		}
		else
		{
			lcd_string("Player 2...     ");
		}
		break;
	case GAME_OVER:
		lcd_command(0x0C);
		lcd_command(0x80);
		if (winner == PLAYER_1_WON)
		{
			lcd_string("    Player 1    ");
		}
		else if (winner == PLAYER_2_WON)
		{
			lcd_string("    Player 2    ");
		}
		else if (winner == AI_WON)
		{
			lcd_string("       AI       ");
		}
		else
		{
			lcd_string("     No one     ");
		}
		lcd_command(0xC0);
		lcd_string("    has won     ");
		break;

	}
    osDelay(100);
  }
  /* USER CODE END task_2_function */
}

/* USER CODE BEGIN Header_task_3_function */
/**
* @brief Function implementing the task_3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_3_function */
void task_3_function(void const * argument)
{
  /* USER CODE BEGIN task_3_function */
  /* Infinite loop */
  board_clear(board);
  uint8_t button, button_2, button_3;
  uint16_t adc_1_val = 0, adc_2_val = 0;
  button_state_typedef button_state = RELEASED;
//  button_state_typedef button_2_state = RELEASED;
//  button_state_typedef button_3_state = RELEASED;
  adc_state_typedef adc_1_state = ZERO;
  adc_state_typedef adc_2_state = ZERO;
  for(;;)
  {
	// GPIO5
	button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
//	button_2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
//	button_3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	// ADC1
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	adc_1_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	// ADC2
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 100);
	adc_2_val = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	switch(button_state)
	{
	case RELEASED:
		if (!button)
		{
			button_state = PRESSED;
			switch (cur_state)
			{
			case MENU_CHOOSE_MODE:
				if (lcd_cursor == POS_1)
				{
					cur_state = MENU_CHOOSE_DIFF;
				}
				else
				{
					cur_state = PLAYER_PLAYER;
					lcd_cursor = POS_1;
				}
				break;
			case MENU_CHOOSE_DIFF:
			{
				cur_state = PLAYER_AI;
				if (lcd_cursor == POS_1)
				{
					ai_mode = EASY;
				}
				else
				{
					ai_mode = HARD;
					lcd_cursor = POS_1;
				}
				break;
			case PLAYER_PLAYER:
				if (turn == PLAYER_1)
				{
					board[cursor.row][cursor.column] = 'x';
					turn = PLAYER_2;
				}
				else
				{
					board[cursor.row][cursor.column] = 'o';
					turn = PLAYER_1;
				}
				if ((evaluate(board)) || (!check_moves_left(board)))
				{
					cur_state = GAME_OVER;
					if (evaluate(board) == -10)
					{
						winner = PLAYER_1_WON;
					}
					else if (evaluate(board) == 10)
					{
						winner = PLAYER_2_WON;
					}
					else
					{
						winner = DRAW;
					}
				}
				break;
			case PLAYER_AI:
				board[cursor.row][cursor.column] = player;
				if ((evaluate(board)) || (!check_moves_left(board)))
				{
					cur_state = GAME_OVER;
					if (evaluate(board) == -10)
					{
						winner = PLAYER_1_WON;
					}
					else if (evaluate(board) == 10)
					{
						winner = AI_WON;
					}
					else
					{
						winner = DRAW;
					}
				}
				else
				{
					if (ai_mode == EASY)
					{
						board_easy_mode(board, rand);
					}
					else
					{
						find_best_move(board);
						board[best_move.row][best_move.column] = computer;
					}
					if ((evaluate(board)) || (!check_moves_left(board)))
					{
						cur_state = GAME_OVER;
						if (evaluate(board) == -10)
						{
							winner = PLAYER_1_WON;
						}
						else if (evaluate(board) == 10)
						{
							winner = AI_WON;
						}
						else
						{
							winner = DRAW;
						}
					}
				}
				break;
			case GAME_OVER:
				board_clear(board);
				cursor.row = 0;
				cursor.column = 0;
				best_move.row = -1;
				best_move.column = -1;
				cur_state = MENU_CHOOSE_MODE;
				break;
			}
			}
		}
		break;
	case PRESSED:
		if (button)
		{
			button_state = RELEASED;
		}
		break;
	}
//	switch(button_2_state)
//	{
//	case RELEASED:
//		if (!button_2)
//		{
//			button_2_state = PRESSED;
//			switch (cur_state)
//			{
//			case MENU_CHOOSE_MODE:
//				if (lcd_cursor == POS_1)
//				{
//					lcd_cursor = POS_2;
//				}
//				else
//				{
//					lcd_cursor = POS_1;
//				}
//				break;
//			case MENU_CHOOSE_DIFF:
//				if (lcd_cursor == POS_1)
//				{
//					lcd_cursor = POS_2;
//				}
//				else
//				{
//					lcd_cursor = POS_1;
//				}
//				break;
//			case PLAYER_AI:
//			case PLAYER_PLAYER:
//				cursor.row++;
//				if (cursor.row > 2)
//				{
//					cursor.row = 0;
//				}
//				break;
//			}
//		}
//		break;
//	case PRESSED:
//		if (button_2)
//		{
//			button_2_state = RELEASED;
//		}
//		break;
//	}
//	switch(button_3_state)
//	{
//	case RELEASED:
//		if (!button_3)
//		{
//			button_3_state = PRESSED;
//			switch (cur_state)
//			{
//			case PLAYER_AI:
//			case PLAYER_PLAYER:
//				cursor.column++;
//				if (cursor.column > 2)
//				{
//					cursor.column = 0;
//				}
//			}
//		}
//		break;
//	case PRESSED:
//		if (button_3)
//		{
//			button_3_state = RELEASED;
//		}
//		break;
//	}
	// ADC1 handler
	switch(adc_1_state)
	{
	case ZERO:
		if (adc_1_val > 3500)
		{
			adc_1_state = POS;
			switch(cur_state)
			{
			case MENU_CHOOSE_MODE:
			case MENU_CHOOSE_DIFF:
				if (lcd_cursor == POS_1)
				{
					lcd_cursor = POS_2;
				}
				else
				{
					lcd_cursor = POS_1;
				}
				break;
			case PLAYER_AI:
			case PLAYER_PLAYER:
				cursor.row++;
				if (cursor.row > 2)
				{
					cursor.row = 0;
				}
				break;
			}
		}
		else if (adc_1_val < 500)
		{
			adc_1_state = NEG;
			switch(cur_state)
			{
			case MENU_CHOOSE_MODE:
			case MENU_CHOOSE_DIFF:
				if (lcd_cursor == POS_1)
				{
					lcd_cursor = POS_2;
				}
				else
				{
					lcd_cursor = POS_1;
				}
				break;
			case PLAYER_AI:
			case PLAYER_PLAYER:
				cursor.row--;
				if (cursor.row < 0)
				{
					cursor.row = 2;
				}
				break;
			}
		}
		break;
	case POS:
		if (adc_1_val < 3500)
		{
			adc_1_state = ZERO;
		}
		break;
	case NEG:
		if (adc_1_val > 500)
		{
			adc_1_state = ZERO;
		}
	}
	// ADC2 handler
	switch(adc_2_state)
	{
	case ZERO:
		if (adc_2_val > 3500)
		{
			adc_2_state = POS;
			switch(cur_state)
			{
			case MENU_CHOOSE_MODE:
			case MENU_CHOOSE_DIFF:
				if (lcd_cursor == POS_1)
				{
					lcd_cursor = POS_2;
				}
				else
				{
					lcd_cursor = POS_1;
				}
				break;
			case PLAYER_AI:
			case PLAYER_PLAYER:
				cursor.column--;
				if (cursor.column < 0)
				{
					cursor.column = 2;
				}
				break;
			}
		}
		else if (adc_2_val < 500)
		{
			adc_2_state = NEG;
			switch(cur_state)
			{
			case MENU_CHOOSE_MODE:
			case MENU_CHOOSE_DIFF:
				if (lcd_cursor == POS_1)
				{
					lcd_cursor = POS_2;
				}
				else
				{
					lcd_cursor = POS_1;
				}
				break;
			case PLAYER_AI:
			case PLAYER_PLAYER:
				cursor.column++;
				if (cursor.column > 2)
				{
					cursor.column = 0;
				}
				break;
			}
		}
		break;
	case POS:
		if (adc_2_val < 3500)
		{
			adc_2_state = ZERO;
		}
		break;
	case NEG:
		if (adc_2_val > 500)
		{
			adc_2_state = ZERO;
		}
	}
	osDelay(10);
  }
  /* USER CODE END task_3_function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
