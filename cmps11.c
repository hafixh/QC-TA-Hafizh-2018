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
File		: cmps11.c
***********************************/

#include "cmps11.h"
#include "adc.h"

uint8_t byteSent;
 uint16_t RawX;
uint16_t RawY;
uint16_t RawZ;
uint8_t byteSent1;
uint8_t sixByte[6];

uint8_t singleByte;
uint8_t singleByte2;
uint8_t fourByte[4];


uint16_t pitch;
uint16_t roll;
uint16_t yaww;
 
uint8_t all4[4];
uint8_t all6[6];


char receivedChr;
char receivedArr[30];
char tesChr;
int beginRec = 0;
int cntRcvChr = 0;
int parsedData[10]={0};

uint8_t pjgData;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	char tmpInt[30];
	int cntTmpInt = -1;
	int numInt=0;
	int _tmpInt = 0;
	
	if(huart->Instance == USART6){
		if (byteSent == 0x14) {
			//store 1 byte
			pitch = singleByte;
			byteSent = 0x15;
			HAL_UART_Receive_IT(&huart6, &singleByte, 1);
			HAL_UART_Transmit_IT(&huart6, &byteSent, 1);
		}else if (byteSent == 0x15) {
			//store 4 byte
			roll = singleByte;
			byteSent = 0x12;
			HAL_UART_Receive_IT(&huart6, &singleByte, 1);
			HAL_UART_Transmit_IT(&huart6, &byteSent, 1);
		}else if (byteSent == 0x12) {
			yaww = 14.18 * singleByte/10;
			byteSent = 0x14;
			HAL_UART_Receive_IT(&huart6, &singleByte, 1);
			HAL_UART_Transmit_IT(&huart6, &byteSent, 1);
			//HAL_UART_Receive_IT(&huart6, (uint8_t*)&sixByte, 6);
			//HAL_UART_Transmit_IT(&huart6, &byteSent, 1);
			//int pnjg = sprintf(DataKirim, "XX$%d,%d,%d#",roll,pitch,yaww);
			//HAL_UART_Transmit_IT(&huart3, (uint8_t*)DataKirim, pnjg);
		}
		/*
				HAL_UART_Receive_IT(&huart6, (uint8_t*)&sixByte, 6);
		HAL_UART_Transmit_IT(&huart6, &byteSent1, 1);
		if (byteSent1 == 0x20) {
			
			RawX = sixByte[0] + (sixByte[1]*256);
 
		 	RawY = sixByte[2] + (sixByte[3]*256);
 
		 	RawZ = sixByte[4] + (sixByte[5]*256);
  
		}*/
		
	}
	
	if(huart->Instance == USART3) {
		//HAL_UART_Transmit_IT(&huart3, (uint8_t*)&receivedChr, 1);
		if (receivedChr == '!') {
			beginRec = 1;
			//receivedArr[cntRcvChr] = receivedChr;
			//cntRcvChr++;
		}
		
		if (beginRec == 1) {
			if(receivedChr != '*'){
				receivedArr[cntRcvChr] = receivedChr;
				cntRcvChr++;
			}else{
				receivedArr[cntRcvChr] = receivedChr;
				//cntRcvChr++;
				
				char UART_Tmp[cntRcvChr+1];
				sprintf(UART_Tmp, "%s", receivedArr);
				
				for(int e=0;e<=cntRcvChr;e++){
					if (receivedArr[e] == ',') {
						tmpInt[cntTmpInt+1] = '\0';
						sscanf(tmpInt, "%d", &_tmpInt);
						parsedData[numInt] = _tmpInt;
						_tmpInt = 0;
						//memset(tmpInt, 0x00, cntTmpInt);
						cntTmpInt = -1;
						numInt++;
					}else if(receivedArr[e] == '*') {
						tmpInt[cntTmpInt+1] = '\0';
						sscanf(tmpInt, "%d", &_tmpInt);
						parsedData[numInt] = _tmpInt;
						//_tmpInt = 0;
						//memset(tmpInt, 0x00, cntTmpInt);
						cntTmpInt = -1;
						numInt = 0;
						beginRec = 0;
						cntRcvChr = 0;
					}else if (receivedArr[e] == '0' || receivedArr[e] == '1' || receivedArr[e] == '2' || receivedArr[e] == '3' || receivedArr[e] == '4' || receivedArr[e] == '5' || receivedArr[e] == '6' || receivedArr[e] == '7' || receivedArr[e] == '8' || receivedArr[e] == '9'){
						tmpInt[cntTmpInt+1] = receivedArr[e];
						cntTmpInt++;
					}
				}
			}
		}
		HAL_UART_Receive_IT(&huart3, (uint8_t*) &receivedChr, 1);
	}

}

