/**********************************
   _____ ______________  ___
  /  _  \\______   \   \/  /
 /  /_\  \|     ___/\     / 
/    |    \    |    /     \ 
\____|__  /____|   /___/\  \
        \/               \_/

Modified by Aproxtime Dev

Developer	: Aproxtime Dev
Language	: C
File		: esc.c
*********************************/

#include "esc.h"

void init_esc(void) {
	for(int e=1;e<5;e++){
    TIM4->CCR1 = 1000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR1 = 2000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR1 = 1000;
    HAL_Delay(10);
  }
	
	for(int e=1;e<5;e++){
    TIM4->CCR2 = 1000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR2 = 2000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR2 = 1000;
    HAL_Delay(10);
  }
	
	for(int e=1;e<5;e++){
    TIM4->CCR3 = 1000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR3 = 2000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR3 = 1000;
    HAL_Delay(10);
  }
	
	for(int e=1;e<5;e++){
    TIM4->CCR4 = 1000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR4 = 2000;
    HAL_Delay(10);
  }
  for(int e=1;e<5;e++){
    TIM4->CCR4 = 1000;
    HAL_Delay(10);
  }
}

void set_esc(uint16_t channel, int speed) {
	switch (channel) {
		case TIM_CHANNEL_1: //ESC1 PD12
			TIM4->CCR1 = speed;
			break;
		
		case TIM_CHANNEL_2: //ESC2 PD13
			TIM4->CCR2 = speed;
			break;
		
		case TIM_CHANNEL_3: //ESC3 PD14 
			TIM4->CCR3 = speed;
			break;
		
		case TIM_CHANNEL_4: //ESC4 PD15 
			TIM4->CCR4 = speed;
			break;
	}
}

