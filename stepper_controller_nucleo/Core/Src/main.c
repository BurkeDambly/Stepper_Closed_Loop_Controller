#include "main.h"
#include "stm32g4xx_hal.h"
#include "core_cm4.h"
#include <stdint.h>

// ================= STEP / DIR =================
#define STEP_PORT GPIOA
#define STEP_PIN  GPIO_PIN_8   // PA8
#define DIR_PORT  GPIOB
#define DIR_PIN   GPIO_PIN_7   // PB7

// ================= ENCODER PWM (AS5048A) =================
// Yellow (P) -> PB6
#define ENC_PWM_PORT GPIOB
#define ENC_PWM_PIN  GPIO_PIN_6

// ================= SPEED =================
#define HIGH_US        4
#define LOW_US         400   // bigger = slower

// ================= ENCODER / LOOP =================
#define ENC_COUNTS_PER_REV       16384

// --- YOU TUNE THIS ---
// counts_per_step = COUNTS_PER_STEP_Q16 / 65536
// Start guess: 8 counts/step (adjust later)
#define COUNTS_PER_STEP_Q16      (8 << 16)

// How often we evaluate lag
#define CHECK_MS                 10

// Only evaluate if we stepped enough in the window
#define MIN_STEPS_IN_WINDOW      15

// Consider "lagging" if expected - actual exceeds this
#define LAG_THRESH_COUNTS        15     // lower = more sensitive (try 20..120)

// Require lag for N consecutive windows before flipping (prevents chatter)
#define LAG_CONSEC_WINDOWS       3      // 1 = fastest, 2–3 = more stable

// Cooldown after flip (prevents immediate flip-back)
#define COOLDOWN_MS              250

// Encoder PWM sanity
#define ENCODER_MIN_PERIOD_US    50
#define ENCODER_MAX_PERIOD_US 50000

// ================= UART (TMC2209) =================
UART_HandleTypeDef huart1;

// ---------------- prototypes ----------------
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);

// ================= DWT =================
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t micros(void)
{
  return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
}

static void delay_us(uint32_t us)
{
  uint32_t start  = DWT->CYCCNT;
  uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
  while ((DWT->CYCCNT - start) < cycles) {}
}

// ================= TMC2209 (write-only) =================
static uint8_t tmc_crc8(const uint8_t *data, int len)
{
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    uint8_t d = data[i];
    for (int b = 0; b < 8; b++) {
      uint8_t mix = (crc ^ d) & 1;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      d >>= 1;
    }
  }
  return crc;
}

static void tmc_write(uint8_t reg, uint32_t val)
{
  uint8_t b[8];
  b[0]=0x05; b[1]=0x00; b[2]=reg|0x80;
  b[3]=val>>24; b[4]=val>>16; b[5]=val>>8; b[6]=val;
  b[7]=tmc_crc8(b,7);
  HAL_UART_Transmit(&huart1, b, 8, HAL_MAX_DELAY);
}

// ================= Encoder PWM capture (EXTI) =================
// Stable angle update only on rising edges
static volatile uint32_t g_rise_us   = 0;
static volatile uint32_t g_period_us = 0;
static volatile uint32_t g_high_us   = 0;

static volatile uint16_t g_angle_counts = 0;
static volatile uint8_t  g_angle_valid  = 0;

static inline void encoder_update_angle_from_pwm(uint32_t period_us, uint32_t high_us)
{
  if (period_us < ENCODER_MIN_PERIOD_US || period_us > ENCODER_MAX_PERIOD_US) return;
  if (high_us == 0 || high_us >= period_us) return;

  uint32_t x = (high_us * ENC_COUNTS_PER_REV) / period_us;
  if (x >= ENC_COUNTS_PER_REV) x = ENC_COUNTS_PER_REV - 1;

  g_angle_counts = (uint16_t)x;
  g_angle_valid = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin != ENC_PWM_PIN) return;

  uint32_t now = micros();

  if (HAL_GPIO_ReadPin(ENC_PWM_PORT, ENC_PWM_PIN) == GPIO_PIN_SET)
  {
    // Rising edge
    if (g_rise_us) {
      g_period_us = now - g_rise_us;
      encoder_update_angle_from_pwm(g_period_us, g_high_us);
    }
    g_rise_us = now;
  }
  else
  {
    // Falling edge
    g_high_us = now - g_rise_us;
  }
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENC_PWM_PIN);
}

// Signed smallest delta a-b in counts: [-8192..+8191]
static inline int16_t wrap_delta_signed(uint16_t a, uint16_t b)
{
  int32_t d = (int32_t)a - (int32_t)b;
  d %= ENC_COUNTS_PER_REV;
  if (d < 0) d += ENC_COUNTS_PER_REV;
  if (d > (ENC_COUNTS_PER_REV / 2)) d -= ENC_COUNTS_PER_REV;
  return (int16_t)d;
}

// ================= Stepping =================
static inline void step_once(void)
{
  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
  delay_us(HIGH_US);
  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
  delay_us(LOW_US);
}

// ================= MAIN =================
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  dwt_init();

  // TMC2209 setup (same as your working config)
  tmc_write(0x10, (16U<<0) | (31U<<8) | (6U<<16));
  tmc_write(0x00, (1U<<2));

  HAL_Delay(150);

  // Direction: +1 forward, -1 reverse
  int dir = +1;
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);

  // Window state
  uint32_t win_start_ms = HAL_GetTick();
  uint32_t steps_in_win = 0;

  uint16_t angle_start = 0;
  uint8_t  angle_start_valid = 0;

  uint32_t cooldown_until_ms = 0;
  uint8_t lag_windows = 0;

  while (1)
  {
    step_once();
    steps_in_win++;

    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - win_start_ms) >= CHECK_MS)
    {
      uint16_t angle_now = g_angle_counts;
      uint8_t  angle_now_valid = g_angle_valid;

      // Seed start sample
      if (!angle_start_valid && angle_now_valid) {
        angle_start = angle_now;
        angle_start_valid = 1;
      }

      if (angle_start_valid && angle_now_valid && steps_in_win >= MIN_STEPS_IN_WINDOW)
      {
        // Actual signed motion in counts this window
        int16_t d_counts = wrap_delta_signed(angle_now, angle_start);

        // Project motion onto the commanded direction (so opposite sign doesn't confuse us)
        int32_t actual_in_dir = (dir > 0) ? (int32_t)d_counts : (int32_t)(-d_counts);
        if (actual_in_dir < 0) actual_in_dir = 0; // if it moved backwards, treat as 0 progress

        // Expected motion (counts) this window
        int32_t expected = (int32_t)(((int64_t)steps_in_win * (int64_t)COUNTS_PER_STEP_Q16) >> 16);

        int32_t lag = expected - actual_in_dir; // positive lag = falling behind

        if (now_ms < cooldown_until_ms) {
          lag_windows = 0; // ignore during cooldown
        } else {
          if (lag > LAG_THRESH_COUNTS) {
            if (lag_windows < 255) lag_windows++;
          } else {
            lag_windows = 0;
          }

          if (lag_windows >= LAG_CONSEC_WINDOWS) {
            dir = -dir;
            HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, (dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            cooldown_until_ms = now_ms + COOLDOWN_MS;
            lag_windows = 0;
          }
        }
      }

      // Reset window
      win_start_ms = now_ms;
      steps_in_win = 0;

      if (angle_now_valid) {
        angle_start = angle_now;
        angle_start_valid = 1;
      } else {
        angle_start_valid = 0;
      }
    }
  }
}

// ================= INIT =================
static void MX_USART1_UART_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef g={0};

  // PA9 = USART1_TX (TMC2209)
  g.Pin = GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &g);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  GPIO_InitTypeDef g={0};

  // STEP PA8
  g.Pin = STEP_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEP_PORT, &g);

  // DIR PB7
  g.Pin = DIR_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_PORT, &g);

  // Encoder PWM PB6 EXTI both edges
  g.Pin = ENC_PWM_PIN;
  g.Mode = GPIO_MODE_IT_RISING_FALLING;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENC_PWM_PORT, &g);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o={0};
  RCC_ClkInitTypeDef c={0};

  o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  o.HSIState = RCC_HSI_ON;
  o.PLL.PLLState = RCC_PLL_OFF;
  if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

  c.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
