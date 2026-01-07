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
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "cs43l22.h"
#include "math.h"

#include "stm32f4_discovery.h"
#include "stm32f4_discovery_accelerometer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// LED roles
#define LED_BLINK LED3
#define LED_ACC_X LED4
#define LED_ACC_Y LED5
#define LED_ACC_Z LED6
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINE_BUFFER_SIZE 256

// macros to define the sine signal
#define FAST_SIN_FREQ 1000
#define MEDIUM_SIN_FREQ 750   
#define SLOW_SIN_FREQ 500

#define SAMPLING_RATE 48000
#define AUDIO_BUFFER_LENGTH SAMPLING_RATE / SLOW_SIN_FREQ

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const uint8_t ISR_FLAG_RX    = 0x01;  // Received data
const uint8_t ISR_FLAG_TIM10 = 0x02;  // Timer 10 Period elapsed
const uint8_t ISR_FLAG_TIM11 = 0x04;  // Timer 11 Period elapsed

// This is the only variable that we will modify both in the ISR and in the main loop
// It has to be declared volatile to prevent the compiler from optimizing it out.
volatile uint8_t isr_flags = 0;

//---Commands---//

/*Commands that we will receive from the PC
The Board will be in sleep mode as much as possible, to reduce power consumption.
Additionally, the board can be sent to stop or standby mode by using a command from the PC.*/

const uint8_t COMMAND_STOP[] = "stop"; // Put Board in stop mode_To wake up from stop mode, the user can press the user button on the Board.
const uint8_t COMMAND_STAND_BY[] = "standby"; // Put Board in standby mode_To wake up from standby mode, the user can press the reset button on the Board.

const uint8_t COMMAND_CHANGE_FREQ[] = "changefreq"; // Set the PWM frequency to the next possible value_The freq can have 3 values "fast, medium, slow"
const uint8_t COMMAND_CHANGE_DUT[] = "changedut"; // Set the PWM duty cycle to the next possible value_The duty cycle can have 3 values, "75%, 50%, 25%"
const uint8_t COMMAND_PWM_MAN[] = "pwmman"; // Set PWM frequency with user button_press user button for a given amount of time, that will become the new freq

const uint8_t COMMAND_LED_PWM[] = "ledpwm"; // Set the LED BLINK to PWM mode_The LED BLINK changes according to the PWM signal generated internally
const uint8_t COMMAND_LED_MAN[] = "ledman"; // Set the LED BLINK to manual mode_In manual mode the LED BLINK can be turned ON and OFF with commands from the PC,and toggled with the user button
const uint8_t COMMAND_LED_ON[] = "ledon"; // Turn ON the LED BLINK
const uint8_t COMMAND_LED_OFF[] = "ledoff"; // Turn OFF the LED BLINK

//The remaining 3 LEDs will be used to indicate the readings from the accelerometer
//The accelerometer will be read every 10ms. Another timer configured to generate interrupts should be used
const uint8_t COMMAND_ACC_ON[] = "accon"; // Enable accelerometer readings_If the accelerometer is enabled, after each reading one of the 3 LED ACCELEROMETERS is
                                          //turned ON, and the rest are turned OFF, according to which direction (X, Y, Z) as the highest accelerometer value, in absolute value.
const uint8_t COMMAND_ACC_OFF[] = "accoff"; // Disable accelerometer readings_If the accelerometer is disabled, the timer should be stopped and all LEDs turned OFF.

const uint8_t COMMAND_MUTE[] = "mute"; // Mute audible signal
const uint8_t COMMAND_UNMUTE[] = "unmute"; // Un-mute audible signal


// Buffer for command
static uint8_t line_ready_buffer[LINE_BUFFER_SIZE]; // Stable buffer for main
// Audio buffer
int16_t buffer_audio[2 * AUDIO_BUFFER_LENGTH];

// ---------- LED BLINK PWM state ----------
// 3 possible PWM frequencies (slow/medium/fast) expressed as total TIM10 ticks
// TIM10: PSC = 16800, so timer runs at 10 kHz -> 1 tick = 0.1 ms
// total_ticks * 0.1 ms = period
#define PWM_NUM_FREQ 3
#define PWM_NUM_DUTY 3

typedef enum {
  PWM_FREQ_SLOW = 0,   // 1.0 s period  (10000 * 0.1 ms)
  PWM_FREQ_MEDIUM,     // 0.5 s period  ( 5000 * 0.1 ms)
  PWM_FREQ_FAST        // 0.2 s period  ( 2000 * 0.1 ms)
} pwm_freq_t;

typedef enum {
  PWM_DUTY_25 = 0,     // 25 %
  PWM_DUTY_50,         // 50 %
  PWM_DUTY_75          // 75 %
} pwm_duty_t;

// Total ticks per PWM period for each speed
static const uint32_t pwm_total_ticks[PWM_NUM_FREQ]   = {10000, 5000, 2000};
// Duty cycles in percent
static const uint8_t  pwm_duty_percent[PWM_NUM_DUTY]  = {25, 50, 75};

// Current configuration
static volatile pwm_freq_t current_pwm_freq  = PWM_FREQ_SLOW;
static volatile pwm_duty_t current_pwm_duty  = PWM_DUTY_50;

// Are we in PWM mode or manual mode?
static volatile uint8_t led_pwm_mode = 1;   // 1 = PWM mode, 0 = manual
static volatile uint8_t led_manual_state = 0; // 0 = off, 1 = on

// Internal counters derived from freq + duty
static uint32_t pwm_on_ticks  = 0;
static uint32_t pwm_off_ticks = 0;
static uint8_t  pwm_led_is_on = 0;

// --------- pwmman / button timing state ---------
// after the user sends "pwmman" command
static volatile uint8_t pwmman_armed = 0;
// while we are currently timing a press
static volatile uint8_t pwmman_measuring = 0;
// when a measurement just finished (main loop will handle it)
static volatile uint8_t pwmman_done = 0;
// counts how many 10 ms ticks the button has been held
static volatile uint32_t button_ticks_10ms = 0;
// last measured duration in ms (for printing / mapping in main)
static volatile uint32_t pwmman_last_duration_ms = 0;

// --------- Accelerometer state ---------
static volatile uint8_t accel_enabled = 0;  // 1 when accon, 0 when accoff
// Last read accelerometer values (mg)
static int16_t accel_xyz[3] = {0, 0, 0};
// To avoid spamming USB: print every N samples (e.g. every 100 ms)
static uint8_t accel_print_div = 0;   // counts TIM11 ticks

static uint32_t SineFreq = PWM_FREQ_SLOW;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Interrupt handlers
void handle_new_line();
void handle_timer10(void);
void handle_timer11(void);

// Helper functions
void init_codec_and_play();
static void build_sine_buffer(uint32_t freq_hz);
static void pwm_update_intervals(void);
static void pwm_apply_settings(void);
static void usb_wait_for_tx_idle(void);

// Commands
void go_to_stop();
void go_to_standby();
void change_freq(void);
void change_duty(void);
void cmd_led_pwm(void);
void cmd_led_man(void);
void cmd_led_on(void);
void cmd_led_off(void);
void cmd_acc_on(void);
void cmd_acc_off(void);
void cmd_mute(void);
void cmd_unmute(void);
void cmd_pwmman(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  //Initializing user LEDs via BSP
  BSP_LED_Init(LED_BLINK);
  BSP_LED_Init(LED_ACC_X);
  BSP_LED_Init(LED_ACC_Y);
  BSP_LED_Init(LED_ACC_Z);

  //Initializing user button
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI); 

  //Initializing timers
  MX_TIM10_Init();
  MX_TIM11_Init();

  // Start with PWM mode active
  led_pwm_mode = 1;
  current_pwm_freq = PWM_FREQ_SLOW;
  current_pwm_duty = PWM_DUTY_50;

  // Start TIM10 in interrupt mode → drives handle_timer10()
  HAL_TIM_Base_Start_IT(&htim10);
  pwm_apply_settings();

  // Initialize accelerometer once at startup
  if (BSP_ACCELERO_Init() != ACCELERO_OK)
  {
     CDC_Transmit_FS((uint8_t*)"Accel init error\r\n", strlen("Accel init error\r\n"));
  }

  // Start with accelerometer disabled; user will enable with "accon"
  accel_enabled = 0;

  // Initialize audio codec and start playback of a sine wave
  init_codec_and_play();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // We check if that there are NO interrupts pending before going into sleep
    if (isr_flags == 0)
    {
      // Go to sleep, waiting for interrupt (WFI).
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
    }

    // Handle timer 10 (PWM / blink LED)
    if (isr_flags & ISR_FLAG_TIM10)
    {
      isr_flags &= ~ISR_FLAG_TIM10;
      handle_timer10();
    }

    // Handle timer 11 (10 ms tick – accel/button later)
    if (isr_flags & ISR_FLAG_TIM11)
    {
      isr_flags &= ~ISR_FLAG_TIM11;
      handle_timer11();
    }

    // This is needed for the UART transmission 
    if (isr_flags & ISR_FLAG_RX)
    {
      isr_flags &= ~ISR_FLAG_RX;
      handle_new_line();
    }

    if (pwmman_done)
    {
    pwmman_done  = 0;    //The flag that indicates that the calculation of the pwm manually is done
    pwmman_armed = 0;   // done with this pwmman round

    uint32_t duration_ms = pwmman_last_duration_ms;

    // Map duration -> PWM frequency
    pwm_freq_t new_freq;
    if (duration_ms < 400)
    {
      new_freq = PWM_FREQ_FAST;
      SineFreq = FAST_SIN_FREQ;
      CDC_Transmit_FS((uint8_t*)"pwmman: FAST freq\r\n", 19);
    }
    else if (duration_ms < 1000)
    {
      new_freq = PWM_FREQ_MEDIUM;
      SineFreq = MEDIUM_SIN_FREQ;
      CDC_Transmit_FS((uint8_t*)"pwmman: MEDIUM freq\r\n", 21);
    }
    else
    {
      new_freq = PWM_FREQ_SLOW;
      SineFreq = SLOW_SIN_FREQ;
      CDC_Transmit_FS((uint8_t*)"pwmman: SLOW freq\r\n", 19);
    }

    current_pwm_freq = new_freq;
    pwm_apply_settings();  // recompute ON/OFF ticks, restart PWM
    build_sine_buffer(SineFreq);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
}

/* USER CODE BEGIN 4 */

//------------------------------------------------callback functions------------------------------------------------
// All of this is used to manage commands from serial interface
static uint8_t line_buffer[LINE_BUFFER_SIZE];
static uint32_t line_len = 0;

void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len)
{
  // Prevent overflow, does not handle the command
  if (line_len + len >= LINE_BUFFER_SIZE)
  {
    line_len = 0;
    return;
  }

  // Append received chunk
  memcpy(&line_buffer[line_len], buf, len);
  line_len += len;

  // Process all complete lines
  while (1)
  {
    // Look for '\n' or '\r' inside line_buffer
    uint8_t *line_feed = memchr(line_buffer, '\n', line_len);
    if (line_feed == NULL){
      line_feed = memchr(line_buffer, '\r', line_len);
      if (line_feed == NULL)
      break;
    }

    // Replace \n or \r by terminator
    *line_feed = '\0';

    // Remove optional '\r' in case input was \r\n
    if (line_feed > line_buffer && *(line_feed - 1) == '\r')
    *(line_feed - 1) = '\0';

    uint32_t processed = (line_feed + 1) - line_buffer;

    // Signal the main that there is a new line ready to be checked
    memcpy(line_ready_buffer, line_buffer, processed);
    isr_flags |= ISR_FLAG_RX;

    // Move leftover bytes to start
    line_len -= processed;
    memmove(line_buffer, line_buffer + processed, line_len);
  }
}


//The callback function to handel the interrupt from the timers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10)
  {
    // Timer 10 expired: set flag so main loop can handle it
    isr_flags |= ISR_FLAG_TIM10;
  }
  else if (htim->Instance == TIM11)
  {
    // Timer 11 expired: 10 ms tick
    isr_flags |= ISR_FLAG_TIM11;
  }
}

/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    // Check if button is actually pressed (line high)
    GPIO_PinState state = HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN);

    // We only care about the NEW press, and only if pwmman was armed
    if (state == GPIO_PIN_SET && pwmman_armed && !pwmman_measuring)
    {
      pwmman_measuring  = 1; //a flag to indicate that the timer started measuring the press time
      button_ticks_10ms = 0; //a flag to add 10msec with each expiration of the timer11

      // Start TIM11 as 10 ms tick, but DO NOT print here
      __HAL_TIM_SET_COUNTER(&htim11, 0);
      HAL_TIM_Base_Start_IT(&htim11);
    }
    else if (!led_pwm_mode)
    {
      // Toggle logical state
      led_manual_state = !led_manual_state;

      // Apply to the actual LED
      if (led_manual_state)
      {
        BSP_LED_On(LED_BLINK);
      }
      else
      {
        BSP_LED_Off(LED_BLINK);
      }
    }

    // We don't handle release here, we detect it in handle_timer11()
  }
}
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    GPIO_PinState state = HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN);

    if (state == GPIO_PIN_SET)
    {
      // 1) First: handle pwmman (has priority)
      if (pwmman_armed && !pwmman_measuring)
      {
        pwmman_measuring  = 1;
        button_ticks_10ms = 0;

        __HAL_TIM_SET_COUNTER(&htim11, 0);
        HAL_TIM_Base_Start_IT(&htim11);
      }
      // 2) Else: manual LED toggle when in manual mode
      else if (!led_pwm_mode)
      {
        led_manual_state = !led_manual_state;

        if (led_manual_state)
          BSP_LED_On(LED_BLINK);
        else
          BSP_LED_Off(LED_BLINK);
      }
    }

    // Release (state == RESET) is ignored here; pwmman release is handled in handle_timer11()
  }
}



//------------------------------------------------handler functions------------------------------------------------
/**
* @brief  Handle possible new command
* @retval None
*/
void handle_new_line()
{
  if (memcmp(line_ready_buffer, COMMAND_STOP, sizeof(COMMAND_STOP)) == 0)
  {
    go_to_stop();
  }
  else if (memcmp(line_ready_buffer, COMMAND_STAND_BY, sizeof(COMMAND_STAND_BY)) == 0)
  {
    go_to_standby();
  }
  else if (memcmp(line_ready_buffer, COMMAND_CHANGE_FREQ, sizeof(COMMAND_CHANGE_FREQ)) == 0)
  {
    change_freq();
  }
  else if (memcmp(line_ready_buffer, COMMAND_CHANGE_DUT, sizeof(COMMAND_CHANGE_DUT)) == 0)
  {
    change_duty();
  }
  else if (memcmp(line_ready_buffer, COMMAND_PWM_MAN, sizeof(COMMAND_PWM_MAN)) == 0)
  {
    cmd_pwmman();
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_PWM, sizeof(COMMAND_LED_PWM)) == 0)
  {
    cmd_led_pwm();
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_MAN, sizeof(COMMAND_LED_MAN)) == 0)
  {
    cmd_led_man();
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_ON, sizeof(COMMAND_LED_ON)) == 0)
  {
    cmd_led_on();
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_OFF, sizeof(COMMAND_LED_OFF)) == 0)
  {
    cmd_led_off();
  }
  else if (memcmp(line_ready_buffer, COMMAND_ACC_ON, sizeof(COMMAND_ACC_ON)) == 0)
  {
    cmd_acc_on();
  }
  else if (memcmp(line_ready_buffer, COMMAND_ACC_OFF, sizeof(COMMAND_ACC_OFF)) == 0)
  {
    cmd_acc_off(); 
  }
  else if (memcmp(line_ready_buffer, COMMAND_MUTE, sizeof(COMMAND_MUTE)) == 0)
  {
    cmd_mute();
  }
  else if (memcmp(line_ready_buffer, COMMAND_UNMUTE, sizeof(COMMAND_UNMUTE)) == 0)
  {
    cmd_unmute(); 
  }
  else
  {
    // If we receive an unknown command, we send an error message back to the PC
    CDC_Transmit_FS((uint8_t*)"Unknown command\r\n", 17);
  }
}


void handle_timer10(void)
{
  if (led_pwm_mode)
  {
    // PWM mode: alternate ON interval and OFF interval
    if (pwm_led_is_on)
    {
      // LED currently ON -> turn it OFF and schedule OFF time
      BSP_LED_Off(LED_BLINK);
      pwm_led_is_on = 0;
      __HAL_TIM_SET_AUTORELOAD(&htim10, pwm_off_ticks);
    }
    else
    {
      // LED currently OFF -> turn it ON and schedule ON time
      BSP_LED_On(LED_BLINK);
      pwm_led_is_on = 1;
      __HAL_TIM_SET_AUTORELOAD(&htim10, pwm_on_ticks);
    }

    // Reset counter each time so the new ARR is effective from 0
    __HAL_TIM_SET_COUNTER(&htim10, 0);
  }
  else
  {
    //Do nothing
  }
}


void handle_timer11(void)
{
  // If we are timing a pwmman button press
  if (pwmman_measuring)
  {
    // Check if button is still pressed
    GPIO_PinState state = HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN);

    if (state == GPIO_PIN_SET)
    {
      // Still pressed -> add 10 ms
      button_ticks_10ms++;
    }
    else
    {
      // Released -> stop timing
      pwmman_measuring = 0;
      // Stop TIM11 only if accelerometer is NOT enabled
      if (!accel_enabled)
      {
      HAL_TIM_Base_Stop_IT(&htim11);
      }

      uint32_t duration_ms = button_ticks_10ms * 10;  // 10 ms per tick
      pwmman_last_duration_ms = duration_ms;
      pwmman_done = 1;  // main loop will finish the job
    }
  }
  // ---- Accelerometer reading ----
  if (accel_enabled)
  {
    // Read X, Y, Z into accel_xyz[]
    BSP_ACCELERO_GetXYZ(accel_xyz);

    int16_t x = accel_xyz[0];
    int16_t y = accel_xyz[1];
    int16_t z = accel_xyz[2];

    // Compute absolute values
    int32_t ax = (x >= 0) ? x : -x;
    int32_t ay = (y >= 0) ? y : -y;
    int32_t az = (z >= 0) ? z : -z;

    if (ax >= ay && ax >= az)
    {
      BSP_LED_On(LED_ACC_X);
      BSP_LED_Off(LED_ACC_Y);
      BSP_LED_Off(LED_ACC_Z);
    }
    else if (ay >= ax && ay >= az)
    {
      BSP_LED_Off(LED_ACC_X);
      BSP_LED_On(LED_ACC_Y);
      BSP_LED_Off(LED_ACC_Z);
    }
    else if (az >= ax && az >= ay)
    {
      BSP_LED_Off(LED_ACC_X);
      BSP_LED_Off(LED_ACC_Y);
      BSP_LED_On(LED_ACC_Z);
    }

    // Print every ~100 ms
    accel_print_div++;
    if (accel_print_div >= 100)
    {
      accel_print_div = 0;
      char msg[64];
      int len = snprintf(msg, sizeof(msg),
                         "ACC X=%6d Y=%6d Z=%6d\r\n",
                         (int)x, (int)y, (int)z);
      CDC_Transmit_FS((uint8_t*)msg, len);
    }
  }
}


//------------------------------------------------helper functions------------------------------------------------
static void build_sine_buffer(uint32_t freq_hz)
{
  for (int i = 0; i < AUDIO_BUFFER_LENGTH; i++)
  {
    // x[n] = A * sin(2*pi*f*n/Fs)
    float sample_f =
        10000.0f * sinf(2.0f * 3.14159265f * (float)freq_hz * (float)i / (float)SAMPLING_RATE);

    int16_t sample = (int16_t)sample_f;

    buffer_audio[2 * i]     = sample; // Left
    buffer_audio[2 * i + 1] = sample; // Right
  }
}


void init_codec_and_play()
{
  cs43l22_init();
  // sine signal
  for(int i = 0; i < AUDIO_BUFFER_LENGTH;i++)
  {
    //for sampling the sine wave at discrete instances, we use the equaltion 
    //x(n)=A.sin(2.pi.f.(n/Fs)), where n = 0,1,2,...
    //AUDIO_BUFFER_LENGTH, the number of samples in one full period of sine
    buffer_audio[2 * i] = 10000 * sin(2 * 3.14 * SLOW_SIN_FREQ * i / SAMPLING_RATE);
    buffer_audio[2 * i + 1] = 10000 * sin(2 * 3.14 * SLOW_SIN_FREQ * i / SAMPLING_RATE);
  }
  cs43l22_play(buffer_audio, 2 * AUDIO_BUFFER_LENGTH);
}


// Recompute ON/OFF durations in timer ticks based on current freq & duty
static void pwm_update_intervals(void)
{
  uint32_t total = pwm_total_ticks[current_pwm_freq];  //the frequency defines the total number of ticks
  uint32_t duty  = pwm_duty_percent[current_pwm_duty]; //the duty cycle defines the number of on ticks and the off ticks

  pwm_on_ticks  = (total * duty) / 100;
  pwm_off_ticks = total - pwm_on_ticks;

  // Avoid zero-length phases (just in case)
  if (pwm_on_ticks == 0)  pwm_on_ticks  = 1;
  if (pwm_off_ticks == 0) pwm_off_ticks = 1;
}


// Apply current PWM settings to TIM10 and LED state
static void pwm_apply_settings(void)
{
  // We restart from LED OFF, and let the first interrupt turn it ON
  pwm_led_is_on = 0;
  BSP_LED_Off(LED_BLINK);

  pwm_update_intervals(); //Calculate the current on and off times

  // First interval: OFF time, so next interrupt will switch it ON
  __HAL_TIM_SET_AUTORELOAD(&htim10, pwm_off_ticks); 
  __HAL_TIM_SET_COUNTER(&htim10, 0);

  // Make sure timer is running
  HAL_TIM_Base_Start_IT(&htim10);
}


static void usb_wait_for_tx_idle(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  if (hcdc == NULL)
  {
    // USB not initialized or not ready; nothing to wait for
    return;
  }

  uint32_t start = HAL_GetTick();

  // Wait until TxState becomes 0, but with a small timeout to avoid getting stuck
  while (hcdc->TxState != 0)
  {
    if ((HAL_GetTick() - start) > 10) // ~10 ms timeout
    {
      break;
    }
  }
}


//----------------------------------------------------Commands----------------------------------------------------
//---Command1_stop---
//-------------------
/**
* @brief Go to stop mode
* @retval None
*/
void go_to_stop()
{
   // Turn off all user LEDs to avoid random light/noise in stop mode
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);

  // First: mute the codec (soft ramp)
  cs43l22_mute();
  HAL_Delay(50);   // ~50 ms gives time for ramp-down --> To reduce the knocking sound

  // If you have accel_enabled / pwmman, they need to be disabled:
  accel_enabled = 0;
  pwmman_armed = 0;
  pwmman_measuring = 0;

  // To avoid noise during stop mode
  cs43l22_stop();

  // Required otherwise the audio wont work after wakeup
  HAL_I2S_DeInit(&hi2s3);

  // wait for any ongoing USB TX to finish to avoid the stuck of the terminal
  usb_wait_for_tx_idle();

  // stop timers 10 and 11
  HAL_TIM_Base_Stop_IT(&htim10);
  HAL_TIM_Base_Stop_IT(&htim11);

  // We disable the systick interrupt before going to stop (1ms tick)
  // Otherwise we would be woken up every 1ms
  HAL_SuspendTick();

  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // We need to reconfigure the system clock after waking up.
  // After exiting from stop mode, the system clock is reset to the default
  // which is not the same we configure in cubeMx, so we do it as done
  // during the init, by calling SystemClock_Config().
  SystemClock_Config();
  HAL_ResumeTick();

  // Required otherwise the audio wont work after wakeup
  MX_I2S3_Init();

  init_codec_and_play();
}

//---Command2_standby---
//-------------------
/**
  * @brief Go to standby mode (deepest low power)
  * @note  Waking up from standby is like a fresh reset.
  *        Use the RESET button on the board.
  */
void go_to_standby(void)
{
  // Turn off user LEDs
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);

  // Stop audio & deinit I2S
  cs43l22_stop();
  HAL_I2S_DeInit(&hi2s3);

  // Stop timers 
  HAL_TIM_Base_Stop_IT(&htim10);
  HAL_TIM_Base_Stop_IT(&htim11);

  // Suspend SysTick so it won't generate interrupts
  HAL_SuspendTick();

  // Clear Wakeup flag (good practice before entering standby)
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  // Enter STANDBY mode
  HAL_PWR_EnterSTANDBYMode();

  // Code NEVER returns here. After reset/wakeup, the MCU starts again from main().
}


//---Command3_changefreq---
//-------------------------
void change_freq(void)
{
  // Cycle SLOW -> MEDIUM -> FAST -> SLOW
  if (current_pwm_freq == PWM_FREQ_SLOW)
  {
    current_pwm_freq = PWM_FREQ_MEDIUM;
    SineFreq = MEDIUM_SIN_FREQ;
    CDC_Transmit_FS((uint8_t*)"Freq: medium\r\n", 14);
  }
  else if (current_pwm_freq == PWM_FREQ_MEDIUM)
  {
    current_pwm_freq = PWM_FREQ_FAST;
    SineFreq = FAST_SIN_FREQ;
    CDC_Transmit_FS((uint8_t*)"Freq: fast\r\n", 12);
  }
  else
  {
    current_pwm_freq = PWM_FREQ_SLOW;
    SineFreq = SLOW_SIN_FREQ;

    CDC_Transmit_FS((uint8_t*)"Freq: slow\r\n", 12);
  }
  // Re-apply PWM settings (update ON/OFF ticks for TIM10)
  pwm_apply_settings();
  build_sine_buffer(SineFreq);
}


//---Command4_changedut---
//------------------------
void change_duty(void)
{
  // Cycle 25% -> 50% -> 75% -> 25%
  if (current_pwm_duty == PWM_DUTY_25)
  {
    current_pwm_duty = PWM_DUTY_50;
    CDC_Transmit_FS((uint8_t*)"Duty: 50%\r\n", 11);
  }
  else if (current_pwm_duty == PWM_DUTY_50)
  {
    current_pwm_duty = PWM_DUTY_75;
    CDC_Transmit_FS((uint8_t*)"Duty: 75%\r\n", 11);
  }
  else
  {
    current_pwm_duty = PWM_DUTY_25;
    CDC_Transmit_FS((uint8_t*)"Duty: 25%\r\n", 11);
  }
  pwm_apply_settings();
}


//---Command5_pwmman---
//---------------------
void cmd_pwmman(void)
{
  // Arm manual PWM measurement.
  pwmman_armed           = 1;  //a flag to indicate that we are using the manual pwm now
  pwmman_measuring       = 0;  //a flag to indicate that timer11 started measuring the press time
  pwmman_done            = 0;  //a flag to indicate that the manual pwm measurement is done
  button_ticks_10ms      = 0;  //a flag to add 10msec with each expiration of the timer11
  pwmman_last_duration_ms = 0; //a flag to indicate the duration of the pressed button at the end

  CDC_Transmit_FS((uint8_t*)"pwmman: ready. Press and hold USER button.\r\n",44);
}


//---Command6_ledpwm---
//---------------------
void cmd_led_pwm(void)
{
  led_pwm_mode = 1;          // LED follows PWM
  // Restart PWM engine so it takes control immediately
  pwm_apply_settings();

  CDC_Transmit_FS((uint8_t*)"LED mode: PWM\r\n", 15);
}


//---Command7_ledman---
//---------------------
void cmd_led_man(void)
{
  led_pwm_mode = 0;          // Manual control

  // Apply manual state to LED immediately
  if (led_manual_state)
    BSP_LED_On(LED_BLINK);
  else
    BSP_LED_Off(LED_BLINK);

  CDC_Transmit_FS((uint8_t*)"LED mode: MANUAL\r\n", 18);
}


//---Command8_ledon---
//--------------------
void cmd_led_on(void)
{
  led_manual_state = 1;
  if (!led_pwm_mode)
  {
    BSP_LED_On(LED_BLINK);
  }
  CDC_Transmit_FS((uint8_t*)"LED: ON\r\n", 9);
}


//---Command9_ledoff---
//---------------------
void cmd_led_off(void)
{
  led_manual_state = 0;
  if (!led_pwm_mode)
  {
    BSP_LED_Off(LED_BLINK);
  }
  CDC_Transmit_FS((uint8_t*)"LED: OFF\r\n", 10);
}


//---Command10_accon---
//---------------------
void cmd_acc_on(void)
{
  accel_enabled = 1;  //A flag used to enable the accelerometer
  // Making sure TIM11 is running for 10 ms ticks
  HAL_TIM_Base_Start_IT(&htim11);
  const char *msg = "Accelerometer ON\r\n";
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}


//---Command11_accoff---
//----------------------
void cmd_acc_off(void)
{
  accel_enabled = 0;
  // Turn off LEDs
  BSP_LED_Off(LED_ACC_X);
  BSP_LED_Off(LED_ACC_Y);
  BSP_LED_Off(LED_ACC_Z);

  // If button-based pwmman is NOT currently measuring, we can stop TIM11
  if (!pwmman_measuring)
  {
    HAL_TIM_Base_Stop_IT(&htim11);
  }
  const char *msg = "Accelerometer OFF\r\n";
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}


//---Command12_mute---
//--------------------
void cmd_mute(void)
{
  int status = cs43l22_mute();
  if (!status)
  {
    const char *msg = "Audio muted\r\n";
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  }
  else
  {
    const char *msg = "Audio mute error\r\n";
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  }
}


//---Command13_unmute---
//----------------------
void cmd_unmute(void)
{
  int status = cs43l22_unmute();
  if (!status)
  {
    const char *msg = "Audio unmuted\r\n";
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  }
  else
  {
    const char *msg = "Audio unmute error\r\n";
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  }
}

//----------------------------------------------------------------------------------------------------------------

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
#ifdef USE_FULL_ASSERT
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
