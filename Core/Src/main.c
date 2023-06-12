
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "math.h"
#include "mpu6050.h"
#include "string.h"
#include "stdbool.h"
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU6050_ADDR 0xD0
I2C_HandleTypeDef hi2c1;
/* USER CODE BEGIN PV */

typedef struct {
	float angle;
	float magnitude;
	float prev ;
	int count;
}thresholding;

uint16_t flag = 1;			

char buff[16];
thresholding data;
MPU6050_t MPU6050;
uint32_t odr;

I2C_HandleTypeDef hi2c1;

//funtion delay 
void IncTick(void)
{
  uwTick += uwTickFreq;
}
uint32_t GetTick(void)
{
  return uwTick;
}
void SysTick_Handler1(void){
	IncTick();
}
 void Delay_ms(uint32_t ms)
{
  uint32_t tickstart = GetTick();
  uint32_t wait = ms;

  /* Add a freq to guarantee minimum wait */
  if (wait < 0xFFFFFFFFU)
  {
    wait += (uint32_t)(1U);
  }

  while ((GetTick() - tickstart) < wait){}
}
///////////////check data
float checkData(MPU6050_t DataStruct , thresholding *data){

	float data_G[5];
	float derivative[5];
	for(uint16_t i = 0 ; i < 5; i++){

		MPU6050_Read_Gyro(&hi2c1, &MPU6050);
		
		data->magnitude = sqrt(MPU6050.Gx*MPU6050.Gx + MPU6050.Gy*MPU6050.Gy + MPU6050.Gz*MPU6050.Gz);
		data_G[i] = data->magnitude;
		
		Delay_ms(50);
	}
		for (uint16_t i = 1; i < 4; i++) {
        derivative[i] = data_G[i + 1] - data_G[i - 1];
    }

    
    for (uint16_t i = 0; i < 3; i++) {
        if ((derivative[i] > 0 && derivative[i + 1] < 0) || (derivative[i] < 0 && derivative[i + 1] > 0)) {
            // Kiem tra dieu kien do loc các dinh
            if(data_G[i] >= 40){
                data->count ++;
            }
        }
		}

	}

	
	
///////////////////////////GPIO/////////////////////////
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t PinState)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_PIN_ACTION(PinState));

  if (PinState != 0)
  {
    GPIOx->BSRR = GPIO_Pin;   //bit set reset register
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16u;
  }
}

///////////////////////// Read GPIO
uint16_t GPIO_ReadPin(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin)
{
  uint16_t bitstatus;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  if ((GPIO->IDR & GPIO_Pin) != 0)   //input data register
  {
    bitstatus = 1;
  }
  else
  {
    bitstatus = 0;
  }
  return bitstatus;
}
//////////////////////////togle register
uint16_t Togle(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin){
	/* get current Output Data Register value */
				odr = GPIOx->ODR;
				/* Set selected pins that were at low level, and reset ones that were high */
				GPIOx->BSRR = ((odr & GPIO_Pin) << 16u) | (~odr & GPIO_Pin);
}

void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO Ports Clock Enable 
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  //Configure GPIO pin Output Level */
  GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  //Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	 //Configure GPIO pin : PA1 
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  //Configure GPIO pin : PB3 
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}
	////////////////////////////////I2C_INIT/////////////////////////////
void I2C1_Init(void)
{

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
}


// Ðinh nghia các bien và ma tran cho bo loc Kalman
	
float x_estimated;           // Trang thái uoc luong
float p;                     // Ma tran hiep phuong sai uoc luong
const float q = 0.01;        // Ma tran nhieu quá trình
const float r = 0.1;         // Ma tran nhieu do
float k;                     // Ma tran Kalman Gain
float z;                     // Du lieu do
float x_predicted;           // Trang thái du doán
float y_predicted;           // Trang thái du doán
float z_predicted;           // Trang thái du doán
float p_predicted;           // Ma tran hiep phuong sai du doán

// Hàm cap nhat giá tri uoc luong cua bo loc Kalman
void Kalman_Update(MPU6050_t data_measurement)
{
    // Du doan trang thái chuyen tiep
    
	  x_predicted = data_measurement.Gx;
		y_predicted = data_measurement.Gy;
		z_predicted = data_measurement.Gz;
    p_predicted = p + q;

    // Tinh ma tran Kalman Gain
    k = p_predicted / (p_predicted + r);

    // Cap nhat trang thai uoc luong
    data_measurement.Gx = x_predicted + k * (data_measurement.Gx - x_predicted);
		data_measurement.Gy = y_predicted + k * (data_measurement.Gy - x_predicted);
		data_measurement.Gz = z_predicted + k * (data_measurement.Gz - x_predicted);
    // Cap nhat ma tran hiep phuong sai uoc luong
    p = (1 - k) * p_predicted;
}




/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
 
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  //Configure the system clock 
  SystemClock_Config();


  /* Initialize all configured peripherals */
  GPIO_Init();
  I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	MPU6050_Init(&hi2c1);
	SSD1306_Init();

  /* USER CODE BEGIN WHILE */
	data.count = 0;
	x_estimated = 0;
  p = 1;
	
  while (1)
  {
		
    /* USER CODE END WHILE */
	
		/* USER CODE END WHILE */
		Kalman_Update(MPU6050);
		checkData(MPU6050, &data);
	
		if(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0 && flag == 1){
			GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);		// turn off led green
			SSD1306_Clear();
			SSD1306_GotoXY(50,30);
			sprintf(buff, "Stop");
			SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
			while(1){
				if(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 1){
					flag = 0;
				}
				if(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0 && flag == 0){
					flag = 1;
					SSD1306_Clear();
					break;
				}
				//Led red
				Togle(GPIOB,GPIO_PIN_3);
				Delay_ms(500);
			}
		}
		if(GPIO_ReadPin(GPIOA,GPIO_PIN_1) == 0){
			SSD1306_Clear();
			data.count = 0;	
		}
		SSD1306_GotoXY(20,30);
		sprintf(buff, "Steps:= %d", data.count);
		SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		
		Togle(GPIOC,GPIO_PIN_13);
			Delay_ms(200);
	
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


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