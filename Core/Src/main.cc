#pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wvolatile"
# include "main.h"
#pragma GCC diagnostic pop

#include <bit>
#include <limits>
#include <array>
#include <cstdio>


extern "C" int __io_putchar(int ch) { return ITM_SendChar(ch); }

inline constexpr auto WORD_BITS = std::numeric_limits<uint32_t>::digits;
static_assert(sizeof (uint32_t) == 4);
static_assert(sizeof (void*) == 4);
static_assert(WORD_BITS == 32);
static_assert(sizeof (uint8_t) == 1);
static_assert(std::numeric_limits<uint8_t>::digits == 8);

namespace SFR
{
    constexpr bool is_continual_pop(uint32_t mask) {
        return WORD_BITS == std::countl_zero(mask) + std::popcount(mask) + std::countr_zero(mask);
    }

    inline void set_and_delay(uint32_t volatile& reg, uint32_t bit) noexcept {
        reg = reg | bit;
        auto tmp = reg & bit;
        (void) tmp;
    }

    template <uint32_t... Args>
    class bits {
        static_assert(0 < sizeof... (Args));
        static_assert((std::has_single_bit(Args) && ...));
        static_assert(sizeof... (Args) == std::popcount((Args | ...)));

    public:
        static constexpr void apply(uint32_t volatile& reg) noexcept {
            reg = reg | (Args | ...);
        }
        static constexpr void apply_seq(uint32_t volatile& reg) noexcept {
            (set_and_delay(reg, Args), ...);
        }
    };

    template <uint32_t Mask, uint32_t Val>
    class field {
        static_assert(0 < std::popcount(Mask));
        static_assert(0 == (~Mask & Val));
        static_assert(is_continual_pop(Mask));

    public:
        static constexpr uint32_t mask_value = Mask;

    public:
        static constexpr void apply(uint32_t volatile& reg) noexcept {
            reg = (reg & ~Mask) | Val;
        }
        static constexpr void apply_seq(uint32_t volatile& reg) noexcept {
            apply(reg);
            auto tmp = reg & Mask;
            (void) tmp;
        }
    };

}

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


int main() {
    // Enable the flash instruction cache, data cache, and pre-fetch buffer.
    SFR::bits<FLASH_ACR_ICEN, FLASH_ACR_DCEN, FLASH_ACR_PRFTEN>::apply(FLASH->ACR);

    //  - Set interrupt group priority.
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    //  - Use sys-tick as time base source and configure 1ms tick.
    constexpr uint32_t TICKS_DIV = 1'000ul;
    SysTick_Config(SystemCoreClock / TICKS_DIV);

    constexpr uint32_t TICK_PRIORITY = 0;
    constexpr uint32_t TICK_PRIORITY_SUB = 0;
    static_assert(TICK_PRIORITY < (1ul << __NVIC_PRIO_BITS));
    NVIC_SetPriority(SysTick_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TICK_PRIORITY, TICK_PRIORITY_SUB));

    SFR::bits<RCC_APB1ENR_PWREN>::apply_seq(RCC->APB1ENR);
    SFR::field<PWR_CR_VOS, PWR_CR_VOS_0 | PWR_CR_VOS_1>::apply_seq(PWR->CR);

    SFR::field<RCC_CR_HSITRIM, RCC_CR_HSITRIM_4>::apply(RCC->CR);

    //(*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE)
    __HAL_RCC_PLL_DISABLE();
    (((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U)
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET) continue;
    RCC->PLLCFGR =
            RCC_PLLSOURCE_HSI |
            16 |
            (336 << RCC_PLLCFGR_PLLN_Pos) |
            (((RCC_PLLP_DIV4 >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) |
            (4 << RCC_PLLCFGR_PLLQ_Pos);
    __HAL_RCC_PLL_ENABLE();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET) continue;


  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      printf("Hello\n");
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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

  /** Configure the main internal regulator output voltage
  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 16;
//  RCC_OscInitStruct.PLL.PLLN = 336;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }


  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
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
  huart2.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  SFR::bits<RCC_AHB1ENR_GPIOCEN, RCC_AHB1ENR_GPIOHEN, RCC_AHB1ENR_GPIOAEN, RCC_AHB1ENR_GPIOBEN>::apply_seq(RCC->AHB1ENR);

  /*Configure GPIO pin Output Level */
  LD2_GPIO_Port->BSRR = LD2_Pin; // Reset

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
