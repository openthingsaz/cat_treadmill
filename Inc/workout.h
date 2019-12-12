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
#define maxTime	900-1
#define maxCnt	maxTime

typedef struct {
	uint32_t timeStamp;
	uint32_t distExercised;
	uint32_t timeExercised;
	void (*reset)(void);
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
void resetRerot( void );
void resetExercise ( void );
void loadDataFromFlash( void );
void writeDataToFlash( exerciseReport* exReport );
uint16_t arcLength ( float degreeMoved );
uint16_t acumulateAngle ( uint16_t degree );
uint32_t get_acumulatedDegree( void );
void amountOfExercise ( dataExercise *exData );

#endif /* WORKOUT_H_ */
