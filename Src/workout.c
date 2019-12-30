/*
 * workout.c
 *
 *  Created on: 2019. 12. 11.
 *      Author: issacs
 */

#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "workout.h"
#include "flash_if.h"
#include "bitopr.h"
#include "circular_buffer.h"
#include "tim.h"

#define radius 55

exerciseReport* exReport;
dataExercise *exData;

/*! \brief
 *
 *
 */
void initExercise( void ) {
	//캘리브레이션 필요 : 초기화 시, 현재 원통의 각도값을 받아 이전각도 변수에 넣어야 함
	exReport = (exerciseReport*)malloc(sizeof(exerciseReport) * totalBuffSize);
	assert(exReport);

	for(int i = 0; i < totalBuffSize - 1; i++) {
		exReport[i].timeStamp = 0;
		exReport[i].distExercised = 0;
		exReport[i].timeExercised = 0;
		exReport[i].dayExercised = 0;
	}

	cbuf = circular_buf_init(exReport, totalBuffSize);
	assert(cbuf);
//	printf("Buffer initialized. ");
//	print_buffer_status(cbuf);
//	exReport->reset = resetReport;

	exData = malloc(sizeof(dataExercise));
	assert(exData);

	exData->acumulatedDegree = 0;
	exData->acumulatedDistance = 0;
	exData->currentDegree = 0;
	exData->previousDegree = 0;
	exData->previousTime[__100ms] = 0;
	exData->previousTime[__1000ms] = 0;
	exData->init = initExercise;
	exData->reset = resetExercise;
	exData->get_acumulatedDegree = get_acumulatedDegree;
}

/*! \brief
 *
 *
 */
void resetReport( void ) {
	for(int i = 0; i < totalBuffSize - 1; i++) {
		exReport[i].timeStamp = 0;
		exReport[i].distExercised = 0;
		exReport[i].timeExercised = 0;
	}
}

/*! \brief
 *
 *
 */
void resetExercise( void ) {
	exData->acumulatedDegree = 0;
	exData->acumulatedDistance = 0;
	exData->currentDegree = 0;
	exData->previousDegree = 0;
	exData->previousTime[__100ms] = 0;
	exData->previousTime[__1000ms] = 0;
}

/*! \brief
 *
 *
 */
void loadDataFromFlash( exerciseReport* exReport, uint8_t day_index ) {
	assert(exReport);

	day_index += 1;
	/* Load boot data from flash */
//	exReport->distExercised	= (*(uint32_t*)(ADDR_FLASH_SECTOR_4));
	memcpy(exReport, (uint32_t*)(ADDR_FLASH_SECTOR_6), sizeof(exerciseReport) * maxCnt * day_index);
}

/*! \brief
 *
 *
 */
void writeDataToFlash( exerciseReport* exReport, uint8_t day_index ) {
	uint32_t ramsource;
	uint32_t offset = sizeof(exerciseReport) * maxCnt * day_index;

	assert(exReport);

	//지우기 전에 기존 데이터 로드
	//loadDataFromFlash(day_index);

//	if(FLASH_If_Erase_Range(ADDR_FLASH_SECTOR_6, ADDR_FLASH_SECTOR_7) == FLASHIF_OK)
//	{
		ramsource = (uint32_t)&exReport[0];

		if(FLASH_If_Write(ADDR_FLASH_SECTOR_6 + offset, (uint32_t*) ramsource, sizeof(exerciseReport) * maxCnt / 4)  != FLASHIF_OK);
			//return ;
//	}
//	else
//		return ;

	//return ;
}

/*! \brief
 *
 *
 */
uint16_t arcLength( float degreeMoved ) {
	return 2 * PI * radius * (degreeMoved / 360.0f);
}

/*! \brief
 *
 *
 */
uint16_t acumulateAngle( uint16_t degree ) {
	static uint16_t previousDegree, currentDegree;
	//최대 속도로 회전 시, 1초에 1.2바퀴 434도 회전 가능함
	previousDegree = currentDegree;
	currentDegree = degree;

	return abs(currentDegree - previousDegree);
}

/*! \brief
 *
 *
 */
uint32_t get_acumulatedDegree( void ) {
	assert(!(exData->acumulatedDegree > UINT_MAX));

	return exData->acumulatedDegree;
}

/*! \brief
 *
 *
 */
void amountOfExercise( dataExercise *exData, uint16_t Roll_offset, uint8_t enable ) {
	static uint16_t second_index = 0;
//	static uint16_t day_index = 0;

	assert(exData);

	if(enable) {
		// 최대 속도로 움직일 시, 100mS동안 43.4도 움직일 수 있음 (400mS은 173.6도)
		//100mS 마다 변화량 측정
		if( abs(HAL_GetTick() - exData->previousTime[__100ms]) > _100mS ) {
			//변동된 각도를 누적한다
			exData->acumulatedDegree += acumulateAngle( abs(get_degree() - Roll_offset) );
			exData->previousTime[__100ms] = HAL_GetTick();
			//		printf("\r acul : %lu\n", exData->get_acumulatedDegree());
		}

		//1초가 되면 기록
		if( abs(HAL_GetTick() - exData->previousTime[__1000ms]) > _1s ) {
			//1초가 되면 운동데이터를 기록한다
			exReport[cbuf->head].timeStamp = get_now_time();
			exReport[cbuf->head].distExercised = arcLength( get_acumulatedDegree() );
			exReport[cbuf->head].timeExercised = second_index;

			//push the buffer
			circular_buf_put(cbuf, &exReport[cbuf->head]);

			//check the overflow
			if(second_index++ > USHRT_MAX) second_index = 0;

			//recod the last time
			exData->previousTime[__1000ms] = HAL_GetTick();
		}
	}
}

