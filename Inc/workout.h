/*
 * workout.h
 *
 *  Created on: 2019. 12. 11.
 *      Author: issacs
 */

#ifndef WORKOUT_H_
#define WORKOUT_H_

#include "mpu6050_dmp.h"
#include "ble_cmd.h"
#include "time.h"

#define _100ms 	0
#define _1000ms  1
#define maxTime	9
#define maxCnt	maxTime
#define maxDay	15-1

typedef struct {
	uint32_t timeStamp;
	uint32_t distExercised;
	uint16_t timeExercised;
	uint16_t dayExercised;
//	void (*reset)(void);
}exerciseReport;

typedef struct {
	uint32_t acumulatedDistance;
	uint32_t acumulatedDegree;
	uint16_t currentDegree;
	uint16_t previousDegree;
	__IO uint16_t previousTime[2];
	void (*init)(void);
	void (*reset)(void);
	uint32_t (*get_acumulatedDegree)(void);
}dataExercise;

extern exerciseReport* exReport;
extern dataExercise* exData;

void initExercise( void );
void resetReport( void );
void resetExercise ( void );
void loadDataFromFlash( exerciseReport* exReport, uint8_t day_index );
void writeDataToFlash( exerciseReport* exReport, uint8_t day_index );
uint16_t arcLength ( float degreeMoved );
uint16_t acumulateAngle ( uint16_t degree );
uint32_t get_acumulatedDegree( void );
void amountOfExercise( dataExercise *exData, uint16_t Roll_offset, uint8_t enable );

#endif /* WORKOUT_H_ */
