/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 2D Detumbling ACS Main Program Body
  ******************************************************************************
  * @details
  *
  * This program implements a basic 2D detumbling attitude control system on an
  * STM32F0 microcontroller, designed for satellite applications. It utilises a
  * gyroscope (FXAS21002) and magnetometer (FXOS8700) to measure the angular
  * rate and magnetic field, calculates the required control torque (PWM bright-
  * ness level), and sets PWM outputs for the magnetic actuators.
  *
  * Main functionalities:
  *   - Blue switch start: Waits for user to press and release the blue pushbutton
  *     ("blue switch") before starting the control loop. LED animation
  *     indicates waiting status.
  *   - Sensor initialisation and health check: Verifies sensor connectivity,
  *     signaling errors or success via status LEDs.
  *   - Control loop: Triggered by a timer interrupt at fixed intervals (e.g., 10 Hz).
  *     Each cycle reads sensor data, computes torque, and updates PWM outputs.
  *   - Energy-saving sleep mode: If the angular rate remains below a threshold
  *     for a set period, the MCU enters sleep mode. It wakes up automatically if
  *     the angular rate rises above a defined value.
  *
  * Usage:
  *   1. Power on the system.
  *   2. Press and release the blue button to start the controller.
  *   3. The system will automatically run the detumbling control loop, enter sleep
  *      mode as needed, and wake on new motion.
  *
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

#include <math.h>  		// Include math functions (e.g. fabsf())

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ============================= USER PDs =====================================

// Control law
#define CTRL_THRESHOLD			0.2f	// [°/s] Deadband to ignore gyro noise
#define GAIN_X					0.25f	// [-] Gain of momentum in x-direction
#define GAIN_Y					0.25f	// [-] Gain of momentum in y-direction

// Sleep functionality
#define SLEEP_THRESHOLD			1.0f	// [°/s] Threshold to enter sleep mode
#define WAKEUP_THRESHOLD		2.0f	// [°/s] Threshold to exit sleep mode
#define IDLE_CYCLES				50		// [-] at 10 Hz control cycle: 5 sec

// Pulse Width Modulation
#define PWM_NEUTRAL				50		// [%] Neutral brightness (50 %)
#define PWM_SLEEP				1		// [%] Sleep brightness   (1 %)
#define PWM_MAX					100		// [%] Maximum brightness (100 %)
#define	PWM_MIN					0		// [%] Minimum brightness (0 %)
// ============================================================================
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef PWM_TIMER;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

volatile 	uint8_t 	do_control = 0;		// Timer interrupt flag for control cycle

static 		uint8_t 	isSleeping = 0;		// Sleep flag
static 		uint16_t 	quietCounter = 0;	// If quietCounter value reaches IDLE_CYCLES value enter sleep mode

static float e_int = 0.0f; 					// PI integrator state shared between main() and compute_torque()

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// Check function prototypes
uint8_t check_gyro(void);				// Check who_am_i register. If check (un-)successful, return true (false)
uint8_t check_magnetometer(void);		// Check who_am_i register. If check (un-)successful, return true (false)


// Initialisation
void init_gyro(void);				// Initialisation of control registers of the gyroscope (FXAS21002C)
void init_magnetometer(void);		// Initialisation of control registers of the magnetometer (FXOS8700CQ)

// Read sensor functions
void read_gyro(float* gyro_z);		// Outputs Z-axis sensor values of 3-axis digital angular rate gyroscope (FXAS21002C)
void read_magnetometer(float* mag_x, float* mag_z);		// Outputs X- and Z-axis sensor values of magnetometer (FXOS8700CQ)

// Calculation function
void compute_torque(float gyro_z, float mag_x, float mag_z, uint8_t* pwm_x, uint8_t* pwm_y);	// Computes PWM-value ("torque of magnetorquer") from sensor values

// Sleep function
void handle_sleep_logic(float gyro_z);			// Implements sleep logic with IDLE_CYCLES, SLEEP_THRESHOLD and WAKEUP_THRESHOLD

// PWM function
void set_pwm(uint8_t pwm_x, uint8_t pwm_y);		// Modulates brightness of green LED (pwm_x) and blue LED (pwm_y) with given value (0-100%)

// LED blink function
void blink_sos_pwm(TIM_HandleTypeDef* htim, uint32_t channel);		// Modulates brightness of given LED with "S-O-S"

// Timer-rate function
static inline void ctrl_timer_set_rate_hz(uint32_t hz);

//
static inline void fxas_active(uint8_t on);

//
static inline void fxos_active(uint8_t on);

// Note:
// For further information on the functions' behaviour see the function declaration in /* USER CODE BEGIN 4 */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef PWM_TIMER;
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
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM on green and blue LED
  HAL_TIM_PWM_Start(&PWM_TIMER, PWM_BLUE_LED);
  HAL_TIM_PWM_Start(&PWM_TIMER, PWM_GREEN_LED);


  // Check, if who_am_i register of the sensors returns the correct value and blink accordingly
  if (check_gyro())
  {
    blink_sos_pwm(&PWM_TIMER, PWM_BLUE_LED); // Check successful
  }
  else
  {
    blink_sos_pwm(&PWM_TIMER, PWM_GREEN_LED); // Check failed
  }

  HAL_Delay(2000);  // Pause to make checks visually distinguishable

  if (check_magnetometer())
  {
    blink_sos_pwm(&PWM_TIMER, PWM_BLUE_LED); // Check successful
  }
  else
  {
    blink_sos_pwm(&PWM_TIMER, PWM_GREEN_LED); // Check failed
  }

  HAL_Delay(1000);  // Pause to make checks visually distinguishable

  // Sensor initialisations
  init_gyro();				// Initialisation of control registers of the gyroscope (FXAS21002C)
  init_magnetometer();		// Initialisation of control registers of the magnetometer (FXOS8700CQ)


  // Start control cycle

  // Wait for press of the blue switch (LOW)
  while (HAL_GPIO_ReadPin(blue_switch_GPIO_Port, blue_switch_Pin) == GPIO_PIN_RESET) {
      // Blue LED oscillates slowly while waiting on press
      static uint8_t val = 0;
      val = (val + 5) % 100;
      __HAL_TIM_SET_COMPARE(&PWM_TIMER, PWM_BLUE_LED, val);
      HAL_Delay(50);
  }

  // Wait 50 ms to compensate for possible (de-)bouncing
  HAL_Delay(50);

  // Wait for release of the blue switch (HIGH)
  while (HAL_GPIO_ReadPin(blue_switch_GPIO_Port, blue_switch_Pin) == GPIO_PIN_SET) {
	  // Blue LED blinks intensely while waiting on release
      __HAL_TIM_SET_COMPARE(&PWM_TIMER, PWM_BLUE_LED, PWM_MAX);
      HAL_Delay(50);
      __HAL_TIM_SET_COMPARE(&PWM_TIMER, PWM_BLUE_LED, PWM_MIN);
      HAL_Delay(50);
  }


  // Start control cycle timer interrupt
  ctrl_timer_set_rate_hz(10);   // 10 Hz control loop frequency when active

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (do_control)	// If the timer-interrupt-flag is set (true = 1), start the control cycle
		{
			do_control = 0;	// Reset flag

			// Initialise local variables for current sensor readings and PWM outputs
			float gyro_z = 0.0f;
			float mag_x = 0, mag_z = 0;
			uint8_t pwm_x = 0, pwm_y = 0;

			if (isSleeping) {
			    // Activate sensors and read data when sleeping to check viability of sleeping
			    fxas_active(1);
			    fxos_active(1);
			}

			// Read values from gyroscope (FXAS21002C) and magnetometer (FXOS8700CQ)
			read_gyro(&gyro_z);
			read_magnetometer(&mag_x, &mag_z);

			// Evaluate sleep logic (set isSleeping-flag if necessary conditions are met)
			handle_sleep_logic(gyro_z);

			static uint8_t prevSleeping = 0;
			if (isSleeping != prevSleeping) {
				prevSleeping = isSleeping;
				if (isSleeping) {
					fxas_active(0);          // Change sensor state to ready
					fxos_active(0);
					ctrl_timer_set_rate_hz(1);   // 1 Hz while sleeping
				} else {
					fxas_active(1);          // Change sensor state to active
					fxos_active(1);
					ctrl_timer_set_rate_hz(10); // Change timer rate to 10 Hz (active control rate)

					e_int = 0.0f; // Reset PI integrator on wake to avoid bias after long idle
				}
			}

			if (!isSleeping) {
				// If not sleeping, calculate and set new PWM values for detumbling control
				compute_torque(gyro_z, mag_x, mag_z, &pwm_x, &pwm_y);
				set_pwm(pwm_x, pwm_y);
			} else {
				// If sleeping, set PWM to PWM_SLEEP
				set_pwm(PWM_SLEEP, PWM_SLEEP);
			}
		}

		// Enter sleep mode
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  PWM_TIMER.Instance = TIM3;
  PWM_TIMER.Init.Prescaler = 479;
  PWM_TIMER.Init.CounterMode = TIM_COUNTERMODE_UP;
  PWM_TIMER.Init.Period = 100;
  PWM_TIMER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  PWM_TIMER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&PWM_TIMER) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&PWM_TIMER, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&PWM_TIMER, &sConfigOC, PWM_BLUE_LED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&PWM_TIMER, &sConfigOC, PWM_GREEN_LED) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&PWM_TIMER);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 99;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : blue_switch_Pin */
  GPIO_InitStruct.Pin = blue_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(blue_switch_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  	  	  // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Blink "SOS" on a PWM channel in Morse code
void blink_sos_pwm(TIM_HandleTypeDef* htim, uint32_t channel)
{
	// Blink "S"
  for (int i = 0; i < 3; i++) {
    __HAL_TIM_SET_COMPARE(htim, channel, PWM_MAX);
    HAL_Delay(150);
    __HAL_TIM_SET_COMPARE(htim, channel, PWM_MIN);
    HAL_Delay(150);
  }

  HAL_Delay(300);	// Pause between S and O

  // Blink "O"
  for (int i = 0; i < 3; i++) {
    __HAL_TIM_SET_COMPARE(htim, channel, PWM_MAX);
    HAL_Delay(450);
    __HAL_TIM_SET_COMPARE(htim, channel, PWM_MIN);
    HAL_Delay(150);
  }

  HAL_Delay(300);	// Pause between O and final S

  // Blink "S"
  for (int i = 0; i < 3; i++) {
    __HAL_TIM_SET_COMPARE(htim, channel, PWM_MAX);
    HAL_Delay(150);
    __HAL_TIM_SET_COMPARE(htim, channel, PWM_MIN);
    HAL_Delay(150);
  }

  // Set channel back to PWM_NEUTRAL
  __HAL_TIM_SET_COMPARE(htim, channel, PWM_NEUTRAL);
}


// Checks if the FXAS21002 gyroscope sensor is connected and responding correctly over I2C
// Returns 1 if the correct sensor is detected, 0 otherwise
uint8_t check_gyro(void)
{
  uint8_t fxas_addr = 0x21 << 1;	// I2C address for FXAS21002 (SA0 = HIGH), shifted left for HAL
  uint8_t reg_addr = 0x0C;			// WHO_AM_I register address
  uint8_t id = 0;					// Variable to store sensor ID

  // Read 1 byte from the WHO_AM_I register of the sensor
  if (HAL_I2C_Mem_Read(&hi2c1, fxas_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &id, 1, 100) != HAL_OK)
  {
    return 0;  // I2C read failed (no response or bus error)
  }

  // Check if the returned ID matches the expected value for FXAS21002 (0xD7)
  if (id == 0xD7)
  {
    return 1;  // Sensor is present and correct
  }

  return 0;  // Received a different ID; wrong device or communication error
}


// Checks if the FXOS8700 sensor is connected and responding correctly over I2C
// Returns 1 if the correct sensor is detected, 0 otherwise
uint8_t check_magnetometer(void)
{
  uint8_t fxos_addr = 0x1F << 1;  	// I2C address of FXOS8700 (SA0 = 1), shifted left for HAL
  uint8_t reg_addr = 0x0D;      	// Address of the WHO_AM_I register
  uint8_t id = 0;					// Variable to store the sensor ID

  // Read 1 byte from the WHO_AM_I register of the sensor
  if (HAL_I2C_Mem_Read(&hi2c1, fxos_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &id, 1, 100) != HAL_OK)
  {
    return 0;  // I2C read failed (no response or bus error)
  }

  // Check if the received ID matches the expected value for FXOS8700 (0xC7)
  if (id == 0xC7)
  {
    return 1;  // Sensor is present and correct
  }

  return 0;  // Received a different ID; wrong device or communication error
}


// Initialises the FXAS21002 gyroscope sensor via I2C
// Configures sensor in standby mode, sets measurement range and activates the sensor
void init_gyro(void)
{
    const uint8_t fxas_addr = 0x21 << 1;  // I2C address of FXAS21002 (SA0 = HIGH)

    // Set gyroscope to standby mode with a data rate of 50 Hz (DR=100), ACTIVE=0.
    // Value 0x10 = 0b00010000 written to CTRL_REG1 (0x13)
    uint8_t ctrl_reg1_standby = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, fxas_addr, 0x13, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1_standby, 1, 100);
    HAL_Delay(5); // Delay for register update

    // Set measurement range to +-250 dps and enable BW=01 for ~8 Hz LPF at 50 Hz ODR
    // Value 0x43 = 0b00000011 written to CTRL_REG0 (0x0D), FS1 = 1, FS0 = 1
    uint8_t ctrl_reg0 = 0x43;
    HAL_I2C_Mem_Write(&hi2c1, fxas_addr, 0x0D, I2C_MEMADD_SIZE_8BIT, &ctrl_reg0, 1, 100);
    HAL_Delay(5);


    // Activate the sensor by setting ACTIVE=1 with the same data rate (DR=100)
    // Value 0x12 = 0b00010010 written to CTRL_REG1 (0x13)
    uint8_t ctrl_reg1_active = 0x12;
    HAL_I2C_Mem_Write(&hi2c1, fxas_addr, 0x13, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1_active, 1, 100);

    // Allow time for first valid sample after activation
    HAL_Delay(80);
}


// Initialises the FXOS8700 sensor via I2C
// Configures sensor in standby, enables MAG-only mode with high OSR, and activates at 12.5 Hz (low-noise).
void init_magnetometer(void)
{
    uint8_t fxos_addr = 0x1F << 1;  // I2C address of FXOS8700 (SA0 = HIGH)

    // Set standby mode (CTRL_REG1 = 0x00)
    // Value 0x00 = 0b00000000 disables all activity.
    uint8_t standby = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, fxos_addr, 0x2A, I2C_MEMADD_SIZE_8BIT, &standby, 1, 100);
    HAL_Delay(100); // Conservative delay to allow settings to take effect

    // Configure magnetometer: M_CTRL_REG1 = 0x1D
    // Value 0x1D = 0b00011101, HMS=01 (MAG-only mode), OSR=111 (8x oversampling) for lower noise.
    uint8_t mctrl1 = 0x1D;
    HAL_I2C_Mem_Write(&hi2c1, fxos_addr, 0x5B, I2C_MEMADD_SIZE_8BIT, &mctrl1, 1, 100);
    HAL_Delay(100);

    // Set to active mode: CTRL_REG1 = 0x2D
    // Value 0x2D = 0b00101101, DR=101 (12.5 Hz in MAG-only), LNOISE=1 (low noise), ACTIVE=1.
    uint8_t active = 0x2D;
    HAL_I2C_Mem_Write(&hi2c1, fxos_addr, 0x2A, I2C_MEMADD_SIZE_8BIT, &active, 1, 100);
    HAL_Delay(100);
}


// Reads the Z-axis angular rate from the FXAS21002 gyroscope via I2C
// Stores the angular rate in degrees per second in the variable pointed to by gyro_z
// Returns 0.0f if there is a communication error or if no new data is available
void read_gyro(float* gyro_z) // FXAS21002C
{
    const uint8_t fxas_addr = 0x21 << 1;	// I2C address of FXAS21002 (SA0 = HIGH)
    const uint8_t reg_status = 0x00;		// Address of the STATUS register
    const uint8_t reg_gyro_x_msb = 0x01;	// Address of the first gyro data register (X_MSB)

    uint8_t status = 0;			// Variable to store the STATUS register value
    uint8_t data[6] = {0};  	// Buffer for 6 bytes: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB

    // Read the STATUS register to check if new data is available
    if (HAL_I2C_Mem_Read(&hi2c1, fxas_addr, reg_status, I2C_MEMADD_SIZE_8BIT, &status, 1, 100) != HAL_OK)
    {
        *gyro_z = 0.0f;		// I2C read failed
        return;
    }

    // Check if new data for all axes is available (ZYXDR bit, bit 3 must be set)
    if ((status & 0x08) == 0)
    {
        *gyro_z = 0.0f;		// No new data available
        return;
    }

    // Read 6 bytes of gyro data (X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB)
    if (HAL_I2C_Mem_Read(&hi2c1, fxas_addr, reg_gyro_x_msb, I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK)
    {
        *gyro_z = 0.0f;		// I2C read failed
        return;
    }

    // Combine Z_MSB and Z_LSB to get the raw Z-axis value (16-bit signed integer)
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    // Convert the raw value to degrees per second (7.8125 mdps/LSB = 0.0078125 dps/LSB)
    *gyro_z = raw_z * 0.0078125f;
}


// Reads the X and Z components of the magnetic field from the FXOS8700 sensor via I2C
// Stores the values in mag_x and mag_z and converts them into physical units before
// Sets both values to 0 if no new data is available or if a communication error occurs
void read_magnetometer(float* mag_x, float* mag_z) // fxos8700
{
    const uint8_t fxos_addr = 0x1F << 1;	// I2C address of FXOS8700 (SA0 = HIGH)
    const uint8_t reg_status = 0x32;		// Address of the STATUS register
    const uint8_t reg_mag_x_msb = 0x33;		// Address of the first magnetometer data register (X_MSB)

    uint8_t status = 0;			// Variable to store the STATUS register value
    uint8_t data[6] = {0};  	// Buffer for 6 bytes: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB

    // Read the STATUS register to check if new data is available
    if (HAL_I2C_Mem_Read(&hi2c1, fxos_addr, reg_status, I2C_MEMADD_SIZE_8BIT, &status, 1, 100) != HAL_OK)
    {
        *mag_x = 0;
        *mag_z = 0;
        return;		// I2C read failed
    }

    // Check if new magnetometer data for all axes is available (ZYXDR bit, bit 3 must be set)
    if ((status & 0x08) == 0)
    {
        *mag_x = 0;
        *mag_z = 0;
        return;		// No new data available
    }

    // Read 6 bytes of magnetometer data (X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB)
    if (HAL_I2C_Mem_Read(&hi2c1, fxos_addr, reg_mag_x_msb, I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK)
    {
        *mag_x = 0;
        *mag_z = 0;
        return;		// I2C read failed
    }

    // Combine MSB/LSB to raw data
    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    *mag_x = raw_x * 0.1f;	// [muT/LSB]
    *mag_z = raw_z * 0.1f;

}

// Computes PWM commands for 2D detumbling using a PI controller.
// Input:  gyro_z [deg/s], magnetic field values mag_x, mag_z.
// Output: pwm_x, pwm_y in [0,100] with neutral at PWM_NEUTRAL.
void compute_torque(float gyro_z, float mag_x, float mag_z, uint8_t* pwm_x, uint8_t* pwm_y)
{

    // Controller parameters
    const float Ts = 0.1f;               // [s] control loop sample time (10 Hz)
    const float Kp = 0.05f;              // proportional gain
    const float Ti = 8.0f;               // [s] integral time constant (tunable)
    const float Ki = Kp / Ti;    		 // integral gain

    // If below noise threshold, command neutral PWM (no actuation)
    if (fabsf(gyro_z) < CTRL_THRESHOLD) {
        *pwm_x = PWM_NEUTRAL;
        *pwm_y = PWM_NEUTRAL;
        return;
    }

    // PI control on z-rate (target omega_z = 0): e = -gyro_z
    const float e = -gyro_z;

    // Integrator with simple clamping (anti-windup)
    e_int += e * Ts;
    if (e_int > 500.0f) e_int = 500.0f;
    if (e_int < -500.0f) e_int = -500.0f;

    // Control effort
    const float u = Kp * e + Ki * e_int;

    // Magnetic control: m = -u * (w × B)
    // For w=[0, 0, w_z], B=[B_x,0,B_z] -> w x B=[-w_z * B_z, w_z * B_x, 0]^T
    const float m_x = -u * mag_z; // commanded moment along x-axis
    const float m_y =  u * mag_x; // commanded moment along y-axis

    // Map commanded moment to PWM duty cycle (0–100, neutral at 50)
    int p_x = (int)(50.0f + m_x * GAIN_X);
    int p_y = (int)(50.0f + m_y * GAIN_Y);

    // Clamp to valid PWM range
    if (p_x < PWM_MIN) p_x = PWM_MIN; else if (p_x > PWM_MAX) p_x = PWM_MAX;
    if (p_y < PWM_MIN) p_y = PWM_MIN; else if (p_y > PWM_MAX) p_y = PWM_MAX;

    // Outputs
    *pwm_x = (uint8_t)p_x;
    *pwm_y = (uint8_t)p_y;
}


// Sets the PWM output values for the X and Y channels (green LED (PC9) and blue LED (PC8))
void set_pwm(uint8_t pwm_x, uint8_t pwm_y)
{
	// Set the PWM value for the X axis (connected to PC9, TIM3 Channel 4)
    __HAL_TIM_SET_COMPARE(&PWM_TIMER, PWM_GREEN_LED, pwm_x);

    // Set the PWM value for the Y axis (connected to PC8, TIM3 Channel 3)
    __HAL_TIM_SET_COMPARE(&PWM_TIMER, PWM_BLUE_LED, pwm_y);
}


// Handles the sleep and wakeup logic based on the current rotational rate (gyro_z)
// If the absolute gyro_z stays below SLEEP_THRESHOLD for IDLE_CYCLES, the system enters sleep mode (isSleeping = 1)
// If, during sleep, gyro_z exceeds WAKEUP_THRESHOLD, the system wakes up
void handle_sleep_logic(float gyro_z)
{
    if (!isSleeping) {
    	// If not sleeping, check if rotational rate is below the threshold
        if (fabsf(gyro_z) < SLEEP_THRESHOLD) {
            quietCounter++;
            // If below SLEEP_THRESHOLD long enough (IDLE_CYCLES), enter sleep mode
            if (quietCounter >= IDLE_CYCLES) {
                isSleeping = 1;
            }
        } else {
        	// If movement above SLEEP_THRESHOLD detected, reset counter
            quietCounter = 0;
        }
    } else {
    	// If currently sleeping, check for movement above WAKEUP_THRESHOLD to wake up
        if (fabsf(gyro_z) > WAKEUP_THRESHOLD) {
            isSleeping = 0;
            quietCounter = 0;
        }
    }
}


// Timer interrupt callback function for STM32 HAL
// This function is called automatically whenever the timer (here TIM14) elapses
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check if the interrupt comes from TIM14 (the control loop timer)
    if (htim->Instance == TIM14)
        do_control = 1;		// Set flag to indicate that a control cycle should be performed in the main loop
}



// Change timer rate function
static inline void ctrl_timer_set_rate_hz(uint32_t hz) {
    __HAL_TIM_DISABLE(&htim14);
    htim14.Init.Prescaler = 47999;
    htim14.Init.Period    = (1000 / hz) - 1;
    HAL_TIM_Base_Init(&htim14);
    __HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim14);
}

// Change gyro state function: Toggle between active and ready
static inline void fxas_active(uint8_t on) {
    uint8_t v = on ? 0x12 : 0x10;   // DR=100 (50 Hz), ACTIVE=1/0 (as in init_gyro)
    HAL_I2C_Mem_Write(&hi2c1, 0x21<<1, 0x13, I2C_MEMADD_SIZE_8BIT, &v, 1, 50);
    if (on) HAL_Delay(80);          // >= 1/ODR + margin
}

// Change magnetometer state function: Toggle between active and ready
static inline void fxos_active(uint8_t on) {
    uint8_t v = on ? 0x2D : 0x00;   // CTRL_REG1 ACTIVE=1/0, DR=101 (12.5 Hz), LNOISE=1 (as in init_magnetometer)
    HAL_I2C_Mem_Write(&hi2c1, 0x1F<<1, 0x2A, I2C_MEMADD_SIZE_8BIT, &v, 1, 50);
    if (on) HAL_Delay(100);         // >= 1/ODR + margin
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
