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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "delay.h"
#include "headfiles.h"
#include "task.h"
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

/* USER CODE BEGIN PV */
float ZhongZhi = -5.5; //机械中值 
float pitch,roll,yaw; 		    
short aacx,aacy,aacz;				
short gyrox,gyroy,gyroz;		
int temp;					    
float Angle_Balance,Gyro_Balance,Gyro_Turn;   	
float Acceleration_Z,Acceleration_X;                       
int moto_pwm_l,moto_pwm_r;
extern pids velo;
float setAngleForward = 0; //全局设置前进时的目标角度 
//float Balance_Kp=300,Balance_Kd=1.0,Velocity_Kp=120,Velocity_Ki=0.4;//PID参数
float bal_kp=1000,bal_ki=0,bal_kd=2.05;
// float bal_kp=300,bal_ki=0,bal_kd=1.0;
float velo_kp=130,velo_ki=0.7,velo_kd=0;
// float bal_kp=800,bal_ki=0,bal_kd=1.6;
// float velo_kp=100,velo_ki=0.35,velo_kd=0;
float turn_kp=160,turn_ki=0,turn_kd=0.4;
u8 taskType = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
  // __HAL_RCC_AFIO_CLK_ENABLE();  // 使能 AFIO 外设时钟
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  // __HAL_RCC_AFIO_CLK_DISABLE();  // 禁用AFIO外设时钟
  //     // 恢复JTAG+SWD (完全恢复默认配置)
  // MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, 0x00000000);

	delay_init(72);
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  OLED_Init(); 

  HAL_TIM_PWM_Init(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	MOTO_init();
	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim1);
	encoder_speed=0;  //

  // OLED_ShowString(1,1,"World Hello",OLED_8X16);
  OLED_Update();

	// pid_init(&bal,390,0,2.5);  
	// pid_init(&velo,200,0.66,0);

  pid_init(&bal,bal_kp,bal_ki,bal_kd);
  pid_init(&velo,velo_kp,velo_ki,velo_kd);

	Moto_SetPwm(0,0);

	MPU6050_initialize();           //=====MPU6050
  delay_ms(50);
	DMP_Init();                     //=====DMP
 
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_UART_Receive_IT(&huart2,&rxdat,1);
  HAL_UART_Receive_IT(&huart1,&rxdat_1,1);
  printf("start\r\n");
	memset(temp_rx,0,30);
  memset(temp_rx_1,0,30);
  //自定义初始化
  turnInit(23); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (oledUpdateFlag == 1)
    {
      // 相对方向名，顺序为：前、左、右、后
      const char* rel_dir_names[] = {"F", "L", "R", "B"};
      // 绝对方向到相对方向的映射表
      // [上次行驶方向][绝对方向] = 相对方向
      // const uint8_t abs2rel[4][4] = {
      //     // 上次方向为0（后）
      //     {0, 1, 3, 2}, // 绝对0后->F, 1左->L, 2前->R, 3右->B
      //     // 上次方向为1（左）
      //     {3, 0, 1, 2}, // 绝对0后->B, 1左->F, 2前->L, 3右->R
      //     // 上次方向为2（前）
      //     {2, 3, 0, 1}, // 绝对0后->R, 1左->B, 2前->F, 3右->L
      //     // 上次方向为3（右）
      //     {1, 2, 3, 0}  // 绝对0后->L, 1左->R, 2前->B, 3右->F
      // };
      // const char* relativeDir[4]={"F","L","R","B"};
      OLED_Clear();
      oledUpdateFlag = 0;
      printf("state:%d\r\n",STATE);
      OLED_ShowString(1,1,"ST:",OLED_8X16);
      OLED_ShowNum(17,1,STATE,1,OLED_8X16);
      OLED_ShowString(33,1,"Dir:",OLED_8X16);
      OLED_ShowNum(65,1,direct%4,1,OLED_8X16);

      if(taskType == 0){
      	OLED_ShowString(105,1,"T0",OLED_8X16);
      }else if (taskType == 1) {
      	OLED_ShowString(105,1,"T1",OLED_8X16);
      }
      // 显示当前点编号
      OLED_ShowString(1, 17, "NO:", OLED_8X16);
      OLED_ShowNum(33, 17, now ? now->no : 0, 3, OLED_8X16);

      // 显示当前点X坐标（带正负号）
      OLED_ShowString(1, 33, "X:", OLED_8X16);
      if(now) {
          if(now->x >= 0) {
              OLED_ShowString(17, 33, "+", OLED_8X16);
              OLED_ShowNum(25, 33, now->x, 4, OLED_8X16);
          } else {
              OLED_ShowString(17, 33, "-", OLED_8X16);
              OLED_ShowNum(25, 33, -now->x, 4, OLED_8X16);
          }
      }

      // 显示当前点Y坐标（带正负号）
      OLED_ShowString(65, 33, "Y:", OLED_8X16);
      if(now) {
          if(now->y >= 0) {
              OLED_ShowString(81, 33, "+", OLED_8X16);
              OLED_ShowNum(89, 33, now->y, 4, OLED_8X16);
          } else {
              OLED_ShowString(81, 33, "-", OLED_8X16);
              OLED_ShowNum(89, 33, -now->y, 4, OLED_8X16);
          }
      }

      OLED_ShowString(1, 49, "WAY:", OLED_8X16);
      u8 x = 33;

      // 当小车到达节点时（STATE == 4），显示相对于当前方向的可行方向
      
          // 使用当前的显示方式（显示上个路口的可行方向）
      // uint8_t rel_dir = last_turn_dir; // 上一次实际行驶方向（0后1左2前3右）
      for(int i=0; i<4; i++) {
          if(last_junction_dirs & (1<<i)) {
              // i为绝对方向，映射到相对方向
              // uint8_t rel = abs2rel[rel_dir][i];
              OLED_ShowString(x, 49, (char *)rel_dir_names[i], OLED_8X16);
              x += 10;
          }
      }
      // 显示当前点的绝对方向
      OLED_Update();
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(! taskType){
      task1();
    }else{
      task2();
    }
    rx_proc();
    key_proc();
    // show_gray_value();
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
