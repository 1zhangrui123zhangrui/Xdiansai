/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 绳驱巡检装置主程序 (STM32F407VET6)
  *
  * 外设分配:
  *   USART1 (PA9/PA10) - 电机多机通信 (ZDT X42S ×4)
  *   USART2 (PA2/PA3)  - 串口屏
  *   USART3 (PB10/PB11)- 预留 (调试)
  *   SPI1   (PA5/6/7)  - NRF24L01 无线通信
  *     PA8=CE, PC9=CSN, PC8=IRQ
  *   PC9  - NRF CSN (注意: 如需独立蜂鸣器, 需在 IOC 中新增 GPIO)
  *   PB8  - 蜂鸣器 (需在 IOC 中添加)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "platform_config.h"
#include "motor.h"
#include "kinematics.h"
#include "task.h"
#include "screen.h"
#include "nrf_app.h"
#include "buzzer.h"
#include "motor_test.h"
#include "screen_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 取消注释以开启对应测试模式，正常使用时全部保持注释 */
#define MOTOR_TEST_ENABLE
//#define SCREEN_TEST_ENABLE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 串口屏回调 */
static void on_home(void)         { Task_EStop(); }
static void on_start(void)        { Task_StartHome(); }
static void on_area(void)         { Task_StartAreaPatrol(); }
static void on_auto(void)         { Task_StartAutoPatrol(); }
static void on_calibrate(void)    { Task_StartCalibrate(); }
static void on_seq(uint8_t *s)    { Task_StartSeqPatrol(s); }
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* --- 串口屏 --- */
  Screen_Init(SCREEN_UART);
  Screen_RegisterCallback(CMD_HOME,         on_home);
  Screen_RegisterCallback(CMD_START,        on_start);
  Screen_RegisterCallback(CMD_AREA_PATROL,  on_area);
  Screen_RegisterCallback(CMD_AUTO_PATROL,  on_auto);
  Screen_RegisterCallback(CMD_CALIBRATE,    on_calibrate);
  Screen_RegisterSequenceCallback(on_seq);

  /* --- 电机 --- */
  Motor_Init(MOTOR_UART);

  /* --- 上电中心清零, 然后松轴 ---
   * 上电前必须手动把激光点放在中心圆。
   * 此处只把当前位置记为 0 度, 随后关闭使能, 方便评委手动拉到任意位置。
   * 不要在模块被拉开后再次清零, 否则 0 度就不再对应中心圆。
   */
  Motor_ZeroAllPositions();
  HAL_Delay(200);
  Motor_DisableAll();
  HAL_Delay(100);

  /* --- 运动学初始化 (零点已设置) --- */
  Kinematics_Init();

#ifdef MOTOR_TEST_ENABLE
  HAL_Delay(500);
  MotorTest_Run();
  while (1) {}
#endif

#ifdef SCREEN_TEST_ENABLE
  /* ======== 串口屏测试模式 ========
   * ScreenTest_Run 内部自行初始化 USART2，不用 Screen_Init */
  ScreenTest_Run();
  while (1) {}
#endif

  /* --- NRF --- */
  // NrfApp_Init();  /* TODO: SPI 卡死, 暂时禁用以测试电机 */

  /* --- 蜂鸣器 --- */
  Buzzer_Init();

  /* --- 任务调度 --- */
  Task_Init();

  uint32_t last_coord_ms = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 任务状态机 */
    Task_Tick();

    /* 每 100ms 上报坐标到串口屏 */
    if (HAL_GetTick() - last_coord_ms >= COORD_UPDATE_INTERVAL_MS) {
        last_coord_ms = HAL_GetTick();
        Screen_SetCoord(Task_GetLaserX(), Task_GetLaserY());
    }

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
