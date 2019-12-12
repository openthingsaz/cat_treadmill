/*
 * workout.c
 *
 *  Created on: 2019. 12. 11.
 *      Author: issacs
 */

#include "workout.h"
#include "flash_if.h"

#define radius 55

exerciseReport* exReport;
dataExercise *exData;

/*! \brief
 *
 *
 */
void
initExercise( void ) {
	//캘리브레이션 필요 : 초기화 시, 현재 원통의 각도값을 받아 이전각도 변수에 넣어야 함
	exReport = (exerciseReport*)malloc(sizeof(exerciseReport)*900);
	for(int i = 0; i < 900; i++) {
		exReport[i].timeStamp = 0;
		exReport[i].distExercised = 0;
		exReport[i].timeExercised = 0;
	}
	exReport->reset = resetRerot;

	exData = malloc(sizeof(dataExercise));
	exData->acumulatedDegree = 0;
	exData->acumulatedDistance = 0;
	exData->currentDegree = 0;
	exData->previousDegree = 0;
	exData->previousTime[_100ms] = 0;
	exData->previousTime[_1000ms] = 0;
	exData->init = initExercise;
	exData->reset = resetExercise;
	exData->get_acumulatedDegree = get_acumulatedDegree;
}

/*! \brief
 *
 *
 */
void
resetRerot( void ) {
	for(int i = 0; i < 900; i++) {
		exReport[i].timeStamp = 0;
		exReport[i].distExercised = 0;
		exReport[i].timeExercised = 0;
	}
}

/*! \brief
 *
 *
 */
void
resetExercise( void ) {
	exData->acumulatedDegree = 0;
	exData->acumulatedDistance = 0;
	exData->currentDegree = 0;
	exData->previousDegree = 0;
	exData->previousTime[_100ms] = 0;
	exData->previousTime[_1000ms] = 0;
}

/*! \brief
 *
 *
 */
void
loadDataFromFlash( void )
{
	/* Load boot data from flash */
//	exReport->distExercised	= (*(uint32_t*)(ADDR_FLASH_SECTOR_4));
	memcpy(exReport, (uint32_t*)(ADDR_FLASH_SECTOR_4), sizeof(sizeof(exerciseReport)*900));
}

/*! \brief
 *
 *
 */
void
writeDataToFlash( exerciseReport* exReport )
{
	uint32_t ramsource;

	if(FLASH_If_Erase_Range(ADDR_FLASH_SECTOR_4, ADDR_FLASH_SECTOR_5) == FLASHIF_OK)
	{
		ramsource = (uint32_t)&exReport[0];

		if(FLASH_If_Write(ADDR_FLASH_SECTOR_4, (uint32_t*) ramsource, 3)  != FLASHIF_OK)
			return ;
	}
	else
		return ;

	return ;
}

/*! \brief
 *
 *
 */
uint16_t
arcLength( float degreeMoved ) {
	return 2 * PI * radius * (degreeMoved /360.0f);
}

/*! \brief
 *
 *
 */
uint16_t
acumulateAngle( uint16_t degree ) {
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
	return exData->acumulatedDegree;
}

/*! \brief
 *
 *
 */
void
amountOfExercise( dataExercise *exData ) {
	static uint16_t index;
	// 최대 속도로 움직일 시, 100mS동안 43.4도 움직일 수 있음 (400mS은 173.6도)
	if( abs(HAL_GetTick() - exData->previousTime[_100ms]) > 100 ) {
		exData->acumulatedDegree += acumulateAngle( get_degree() );
		exData->previousTime[_100ms] = HAL_GetTick();
//		printf("\r acul : %lu\n", exData->get_acumulatedDegree());
	}

	//1초가 되면 기록
	if( abs(HAL_GetTick() - exData->previousTime[_1000ms]) > 1000 ) {
//		exReport->timeStamp = ;
		exReport[index].distExercised = arcLength(get_acumulatedDegree());
		exReport[index].timeExercised += 1;
		exData->previousTime[_1000ms] = HAL_GetTick();
		index++;
	}

	//15분이 경과하면 데이터 기록
	if( (exReport[index].timeExercised > maxTime) || (index > maxCnt) ) {

		//플래쉬에 데이터 기록

		//변수 초기화
			exReport->reset();
	}
}

