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
#include "memorymap.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "osqp.h"         // OSQP主API
#include "workspace.h"    // 全局solver实例
#include "mpc_matrices.h" // 预计算矩阵和计算函数
#include <stdint.h>
#include <string.h>
#include "math.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// DWT初始化函数
void DWT_Init(void)
{
    // 启用DWT
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    
    // 重置计数器
    DWT->CYCCNT = 0;
    
    // 启用计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 获取微秒时间戳 - 适配550MHz CPU时钟
uint32_t DWT_GetMicros(void)
{
    // 对于550MHz: SystemCoreClock = 550000000
    // 每微秒有550个时钟周期
    // 使用通用公式: 
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

// 获取纳秒时间戳（如果需要更高精度）
uint64_t DWT_GetNanos(void)
{
    // 对于550MHz，每纳秒约0.55个时钟周期
    // 使用通用公式: 
    return ((uint64_t)DWT->CYCCNT * 1000000) / SystemCoreClock;
}

// 微秒级延时
void DWT_DelayMicros(uint32_t micros)
{
    uint32_t start = DWT_GetMicros();
    while ((DWT_GetMicros() - start) < micros);
}


uint32_t mpc_update_time_start_us = 0;
uint32_t mpc_update_time_us = 0;

uint32_t mpc_solve_time_start_us = 0;
uint32_t mpc_solve_time_us = 0;


// === 控制器状态变量 ===
static OSQPFloat x_current[MPC_NX] = {0}; // 当前转向角度 [θ1, θ2, θ3, θ4]
static OSQPFloat r_target[MPC_NX] = {0};  // 目标转向角度
static OSQPFloat u_optimal[MPC_NU] = {0}; // 最优角速度指令 [ω1, ω2, ω3, ω4]

// === QP问题在线参数 ===
static OSQPFloat q_new[MPC_N * MPC_NU] = {0};                    // 线性代价向量 (40)
static OSQPFloat l_new[MPC_N * MPC_NX + MPC_N * MPC_NU] = {0};   // 约束下界 (80)
static OSQPFloat u_new[MPC_N * MPC_NX + MPC_N * MPC_NU] = {0};   // 约束上界 (80)

// === 实时控制循环 (定时器中断) ===
void mpc_control_loop(void) {

	// 步骤1: 在线参数计算 - 使用预生成的高效函数
  mpc_update_time_start_us = DWT_GetMicros();
	calculate_q_vector(q_new, x_current, r_target);
	calculate_constraint_bounds(l_new, u_new, x_current);
  mpc_update_time_us = DWT_GetMicros() - mpc_update_time_start_us;

  mpc_solve_time_start_us = DWT_GetMicros();
	// 步骤2: 更新求解器参数
	osqp_update_data_vec(&solver, q_new, l_new, u_new);

	// 步骤3: 求解优化问题
	OSQPInt exit_code = osqp_solve(&solver);
  mpc_solve_time_us = DWT_GetMicros() - mpc_solve_time_start_us;
	// 步骤4: 执行控制决策
	// OSQP返回值规范：0=成功，>0=错误码
	if (exit_code == 0 && solver.info->status_val == OSQP_SOLVED) {
		// 成功求解：提取最优控制输入
		memcpy(u_optimal, solver.solution->x, MPC_NU * sizeof(OSQPFloat));
		
		// 发送速度指令
		// send_can_velocities(u_optimal);
		
		// 热启动优化：加速下次求解
		osqp_warm_start(&solver, solver.solution->x, solver.solution->y);
	} else {
		// 求解失败：安全停止策略
		OSQPFloat zero_velocities[MPC_NU] = {0.0f, 0.0f, 0.0f, 0.0f};
		// send_can_velocities(zero_velocities);
	}
}

// === 外部接口：更新目标角度 ===
// 输入new_targets：度
void mpc_set_target_angles(const OSQPFloat new_targets[MPC_NX]) {
	// 转化为弧度
	for (int i = 0; i < MPC_NX; i++) {
		r_target[i] = new_targets[i] * MATH_PI / 180.0;
	}
}

// === 外部接口：更新当前角度 ===
// 输入new_currents：度	
void mpc_set_current_angles(const OSQPFloat new_currents[MPC_NX]) {
	// 转化为弧度
	for (int i = 0; i < MPC_NX; i++) {
		x_current[i] = new_currents[i] * MATH_PI / 180.0;
	}
}

// 模拟外部角度给定,单位：度
OSQPFloat new_targets_set[MPC_NX] = {24.0f, 24.0f, 12.0f, 12.0f};
OSQPFloat new_currents_set[MPC_NX] = {4.0f, 8.0f, 4.0f, 8.0f};


static void mpc_demo_loop(void)
{
  mpc_set_target_angles(new_targets_set);
  mpc_set_current_angles(new_currents_set);
  mpc_control_loop();
  // HAL_Delay(MPC_TS * 1000);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  /* USER CODE BEGIN 2 */
  DWT_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    mpc_demo_loop();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
