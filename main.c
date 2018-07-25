#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "cmps11.h"
#include "tim.h"
#include "i2c.h"
#include "lcd.h"
#include "esc.h"
#include "adc.h"
//#include "dma.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>


uint8_t i;
float sudutRoll, sudutPitch, sudutYaw;
uint8_t fix;
uint8_t received;
uint16_t adcData;
extern uint8_t singleByte;
extern uint8_t singleByte2;
extern uint8_t byteSent;
extern uint16_t roll, pitch, RawYaw, yaww; 
extern char receivedChr;
extern UART_HandleTypeDef huart6;
extern uint16_t yaww;

int iterasi=0;

void SystemClock_Config(void);
void i2c1_send_data(uint16_t addr, uint8_t* data, uint8_t length);
void i2c1_receive_data(uint16_t addr, uint8_t* data, uint8_t length);

#define g 9.81 // 1g ~ 9.81 m/s^2
#define magnetometer_cal 0.06 //magnetometer calibration
#define kpr 20	//4.028
#define kir 0//0.000035
#define kdr 0//1.3

#define kpp 20//0.034	//4.028
#define kip 0//0.0000008
#define kdp 0//0.055

#define kpy 5 //4.028
#define kiy 0
#define kdy 0

#define kpz 5//3.2//4.028	//4.028
#define kiz 0	//0.0000000183
#define kdz 0//0.5

#define massa 1.210
#define panjang 0.54
#define d 1
#define phi 3.14

float ixx = massa * ((panjang*panjang)+(panjang*panjang));
float iyy = massa * ((panjang*panjang)+(panjang*panjang));
float izz = (massa * (panjang*panjang))/12;
float k1,k2,k3,vzpid, vpitch,vroll,vyaw,vx,vy,vz,u1,u2,u3,u4,f1,f2,f3,f4, setYaw, ftotal, esc1er=0, esc2er=0, esc3er=0, esc4er=0;
int lastTime;
float now, OutPIDVR, OutPIDVP, OutPIDVY, OutPIDVZ;
int hitesc1=0, hitesc2=0, hitesc3=0, hitesc4=0;
float   lastErrVZ, errSumZ, errSumR, lastErrR, errSumP, lastErrP, errSumY, lastErrY, errpitch, errroll, erryaw, errz, lastErrPitch=0, lastErrRoll=0, lastErrYaw=0, lastErrZ=0;
float esc1, esc2, esc3, esc4, VRollErr=0.0, VPitchErr=0.0, VYawErr=0.0, RErr = 0.0, PErr = 0.0, YErr = 0.0, lastVP=0, lastVR=0, lastVY=0,lastVZ=0, VZErr;
double timeChange;
float rollHit,pitchHit,yawHit,z,omegaRoll, omegaPitch, omegaYaw, dErrVP, dErrVY,dErrVR, dErrVZ,hitu1=0, setY=0;
int ARM;
char txData[200];
 int main(void){
  HAL_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
	MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
	 MX_ADC1_Init();
 // MX_DMA_Init();

  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	 
	init_esc();
	HAL_Delay(4000);
	
	
 
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
//	__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
	
	HAL_UART_Transmit_IT(&huart6, (uint8_t*)0xA1, 1);
	HAL_Delay(50);
	huart6.Init.BaudRate = 38400;
	HAL_UART_Init(&huart3);
	
  HAL_UART_Receive_IT(&huart6, &singleByte, 1);
	HAL_UART_Receive(&huart3, &received, 1, 1);
  
  byteSent = 0x14;
  HAL_UART_Transmit_IT(&huart6, &byteSent, 1);
 
	
  ixx = massa * ((panjang*panjang)+(panjang*panjang));
	iyy = massa * ((panjang*panjang)+(panjang*panjang));
	izz = (massa * (panjang*panjang))/12;
 
	HAL_TIM_Base_Start(&htim1);
	 HAL_TIM_Base_Start(&htim2);
	 
	
	 setYaw = yaww/57.29577951308;
	// ARM=1;
	vx = 0;
	vy = 0; 
	lastVZ =375;
	while (1){
		
		HAL_UART_Receive(&huart3, &received, 1, 10);
		ARM= received-'0';
		
		HAL_ADC_Start(&hadc1);
		adcData = HAL_ADC_GetValue(&hadc1);
		
		z = (7.5 * adcData/4096);//-0.214233;
		vz = z/timeChange; 
		now = 0.14;//TIM1->CNT;	168000000  0.04
		timeChange = (now - lastTime);

		//(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		if(roll >=0 && roll <=127){
			rollHit = (roll)*(130)/(127);//((roll) * (85.0/255.0));
			sudutRoll = rollHit/57.29577951308;
		}
		else if (roll>=128){
			rollHit = (roll-255)*(-130)/(-127);
			sudutRoll = rollHit/57.29577951308;
		}
		omegaRoll = sudutRoll/timeChange;
		//pitchHit = ((pitch) * (85.0/255.0));		
		if(pitch >=0 && pitch <=127){
			pitchHit = (pitch)*(130)/(127);//((roll) * (85.0/255.0));
			sudutPitch = pitchHit/57.29577951308;
		}
		else if (pitch>=128){
			pitchHit = (pitch-255)*(-130)/(-127);
			sudutPitch = pitchHit/57.29577951308;
		}
		omegaPitch=sudutPitch/timeChange;
		
		sudutYaw = yaww/57.29577951308;//
		omegaYaw = sudutYaw/timeChange;
		if(setY==0){setYaw=sudutYaw;setY=1;}
		//-------------------Nilai Error--------------------------------------------
		VRollErr = 0 - sudutRoll;//5.00;//(phi*roll/180)/now;
		VPitchErr = 0 - sudutPitch;//0.0;//(phi*pitch/180)/now;
		VYawErr = setYaw - sudutYaw;//(phi*RawYaw/180)/now;
		VZErr = lastVZ - z;
		
		errpitch = lastErrPitch + VPitchErr;
		errroll = lastErrRoll + VRollErr;
		erryaw = lastErrYaw + VYawErr;
		errz = lastErrZ + VZErr;
		
		lastErrPitch=errpitch;
		lastErrRoll=errroll;
		lastErrYaw=erryaw;
		lastErrZ=errz;
	 
		/*lastVR = 0;//omegaRoll;
		lastVP = 0;//omegaPitch;
		lastVY = 0;//omegaYaw;*/
		//vz;
	//---------------------PID VRoll-------------------------------------------------- 
		errSumR += ((errroll)*timeChange/2);
		dErrVR = (VRollErr)/timeChange;

		OutPIDVR = (kpr * VRollErr) + (kir * errSumR) + (kdr * dErrVR);
 
	//---------------------PID VPitch-------------------------------------------------

		errSumP += ((errpitch)*timeChange/2);
		dErrVP = (VPitchErr)/timeChange;
	//
		OutPIDVP = (kpp * VPitchErr) + (kip * errSumP) + (kdp * dErrVP);
  
	//----------------------PID VYaw---------------------------------------------------

		errSumY += ((erryaw)*timeChange/2);
		dErrVY = (VYawErr)/timeChange;
	//
		OutPIDVY = (kpy * VYawErr) + (kiy * errSumY) + (kdy * dErrVY);
 
	//----------------------PID VZ---------------------------------------------------
		
		errSumZ += ((errz)*timeChange/2);
		dErrVZ = (VZErr)/timeChange;
		OutPIDVZ = (kpz * VZErr) + (kiz * errSumZ) + (kdz * dErrVZ);
 
		vroll = OutPIDVR; 
		vpitch = OutPIDVP; 
		vyaw =  OutPIDVY;
		vzpid = OutPIDVZ;
		
		
//---------------------------------------------------Decouple-------------------------------------------------------------------------
		k1 = ((ixx+iyy-izz) * (omegaPitch*omegaRoll* sin(sudutRoll)))+((-ixx + iyy - izz) * (omegaPitch * omegaYaw * sin(sudutPitch) * sin(sudutRoll)))+((ixx + iyy - izz)*omegaRoll*omegaYaw*cos(sudutPitch)*cos(sudutRoll))+((iyy-izz)*(omegaYaw*omegaYaw)*sin(sudutPitch)*cos(sudutPitch)*cos(sudutRoll));
		k2 = ((-iyy + (izz - ixx)*cos(2*sudutRoll))*omegaPitch*omegaYaw*cos(sudutPitch)) + ((izz - ixx)*((omegaPitch*omegaPitch)-(omegaYaw*omegaYaw)*cos(sudutPitch)*cos(sudutPitch))*sin(sudutRoll)*cos(sudutRoll));
		k3 = ((-izz+ixx-iyy)*omegaPitch*omegaRoll*cos(sudutRoll))+((izz+ixx-iyy)*omegaPitch*omegaYaw*sin(sudutPitch)*cos(sudutRoll))+((izz-ixx+iyy)*omegaRoll*omegaYaw*cos(sudutPitch)*sin(sudutRoll))-((ixx-iyy)*(omegaYaw*omegaYaw)*sin(sudutPitch)*cos(sudutPitch)*sin(sudutRoll));
/*/
		k1 = ((ixx+iyy-izz) * (vpitch *vroll * sin(sudutRoll)))+((-ixx + iyy - izz) * (vpitch * vyaw * sin(sudutPitch) * sin (sudutRoll)))+((ixx + iyy - izz)*vroll  *vyaw * cos(sudutPitch) * cos(sudutRoll)) + ((iyy-izz)*(vyaw * vyaw ) * sin(sudutPitch)*cos(sudutPitch)*cos(sudutRoll));
		k2 = ((-iyy + (izz - ixx)*cos(20))*vpitch *vyaw *cos(sudutPitch)) + ((izz - ixx)*((vpitch *vpitch )-(vyaw *vyaw)*cos(sudutPitch)*cos(sudutPitch))*sin(sudutRoll)*cos(sudutRoll));
		k3 = ((-izz+ixx-iyy)*vpitch *vroll *cos(sudutRoll))+((izz+ixx-iyy)*vpitch *vyaw *sin(sudutPitch)*cos(sudutRoll))+((izz-ixx+iyy)*vroll *vyaw *cos(sudutPitch)*sin(sudutRoll))-((ixx-iyy)*(vyaw *vyaw )*sin(sudutPitch)*cos(sudutPitch)*sin(sudutRoll));
		*/
	//	ftotal = f1+f2+f3+f4;
	//	vx=((sin(sudutRoll*cos(sudutYaw))+(sin(sudutPitch)*cos(sudutRoll)*sin(sudutYaw))))*ftotal/massa;
	//	vy=((sin(sudutRoll*cos(sudutYaw))-(sin(sudutPitch)*cos(sudutRoll)*sin(sudutYaw))))*ftotal/massa;
		u1 = massa * sqrt((vx*vx)+(vy*vy)+((vzpid+9.8)*(vzpid+9.8)));
		u2 = ((ixx*cos(sudutRoll)*vpitch)-(ixx*cos(sudutPitch)*sin(sudutRoll)*vyaw))- k1;
		u3 = ((iyy*vroll)+(iyy*sin(sudutPitch)*vyaw))- k2;
		u4 = ((izz*sin(sudutRoll)*vpitch)+(izz*cos(sudutPitch)*cos(sudutRoll)*vyaw))- k3; 
		
	/*	f1 = (u1/4) + (u3/2*panjang) - (u4/4*d);
		f2 = (u1/4) - (u2/2*panjang) + (u4/4*d);
		f3 = (u1/4) - (u3/2*panjang) - (u4/4*d);
		f4 = (u1/4) + (u2/2*panjang) + (u4/4*d); 
		/*/
		
	/*  if(sudutPitch>-0.17453292&&sudutPitch<=-0.0872664) hitesc2-=0.01;
		else if(sudutPitch>-0.261799&&sudutPitch<=-0.17453292) hitesc2-=0.06; 
		else if(sudutPitch>-0.261799&&sudutPitch<=-0.17453292) hitesc2-=0.1; 
		else if(sudutPitch>-0.349065&&sudutPitch<=-0.261799) hitesc2-=0.5; 
		
		if(sudutPitch<0.17453292&&sudutPitch>=0.0872664) hitesc4-=0.01; 
		else if(sudutPitch<0.261799&&sudutPitch>=0.17453292) hitesc4-=0.06; 
		else if(sudutPitch<0.261799&&sudutPitch>=0.17453292) hitesc4-=0.1;  
		else if(sudutPitch<=0.349065&&sudutPitch>0.261799) hitesc4-=0.5; 
   */
	 /*
		f1 = (u1) + (u3/2) - (u4/(4*d));
		f2 = (u1) - ((u2*750*0.92) ) + (u4/(4*d));
		f3 = (u1) - (u3/2) - (u4/(4*d));
		f4 = (u1) + ((u2*750*0.958)) + (u4/(4*d)); 
		//
		f1 = (u1) + (u3) - (u4/(4*d));
		f2 = (u1) - ((u2) ) + (u4/(4*d));
		f3 = (u1) - (u3) - (u4/(4*d));
		f4 = (u1) + ((u2*0.7)) + (u4/(4*d));
		*/
		f1 = (u1/4) + (u3/2*panjang) - (u4/(4*d));
		f2 = (u1/4) - (u2/2*panjang) + (u4/(4*d));
		f3 = (u1/4) - (u3/2*panjang) - (u4/(4*d));
		f4 = (u1/4) + (u2/2*panjang) + (u4/(4*d));
		if(sudutPitch>0&&sudutPitch<0.0872664){hitesc2-=25;hitesc4+=25;}
		if(ARM==1||ARM==-48){
		esc1 = ((1285.0488095238+f1)/1.2584285714);//1.2584285714x – 1285.0488095238
		esc2 = ((1285.0488095238+f2)/1.2584285714);
		esc3 = (1285.0488095238+f3)/1.2584285714;
		esc4 = ((1285.0488095238+f4)/1.2584285714);
		
			
		if(esc1<=1250)esc1=1250;if(esc2<=1250)esc2=1250;if(esc3<=1250)esc3=1250;if(esc4<=1250)esc4=1250;
		if(esc1>=2000)esc1=2000;if(esc2>=2000)esc2=2000;if(esc3>=2000)esc3=2000;if(esc4>=2000)esc4=2000;
		} else if(ARM==0){
		esc1 = 0;
		esc2 = 0;
		esc3 = 0;
		esc4 = 0;
		} 

		
		set_esc(TIM_CHANNEL_1, esc1);
		set_esc(TIM_CHANNEL_2, esc2);
		set_esc(TIM_CHANNEL_3, esc3);
		set_esc(TIM_CHANNEL_4, esc4);
		 
		lastTime = now;
		
		sprintf(txData, "%f", sudutPitch );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", sudutRoll );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", sudutYaw );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", z );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", setYaw );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", esc1 );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", esc2 );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", esc3 );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "\t" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		sprintf(txData, "%f", esc4 );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10); 
		sprintf(txData, "\r \n" );
		HAL_UART_Transmit(&huart3, (uint8_t *)&txData, strlen(txData), 10);
		
 
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

