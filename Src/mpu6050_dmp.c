#include "mpu6050_dmp.h"
//#include "IOI2C.h"
#include "HAL_I2C.h"
//#include "usart.h"
//#include "usart.h"
#include "main.h"
#include "ema_filter.h"
#include <stdio.h>

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define RAD_TO_DEG 57.295779513082320876798154814105

#define q30  1073741824.0f // 2^30
short gyro[3], accel[3], sensors;
float Pitch, Roll, Roll_reverse, Yaw, Rangle=0.0f, Pangle=0.0f;

float base_pitch=0.0f, base_roll=0.0f, base_yaw=0.0f, base_roll_reverse=0.0f;
float dqw=1.0f, dqx=0.0f, dqy=0.0f, dqz=0.0f, sign=0.0f;
float ledPos = 0.0f;
float targetLedPos = 0;
float targetAnglel = 120.0f;
uint8_t Cal_done = 0;


static signed char gyro_orientation[9] = {-1, 0, 0,
		0,-1, 0,
		0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else {
		b = 7;      // error
	}
	return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;


	return scalar;
}

static void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	if (result == 0x7) {
		/* Test passed. We can trust the gyro data here, so let's push it down
		 * to the DMP.
		 */
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
	}
}



uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;



/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈뱟占쎈쿈占쎌굲ADC占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솧筌뤿슣�굲占쎈쐻占쎈뱟占쎈쿈占쎌굲 FIFO占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈쓡塋딅슗�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈뼑占쎈Ŋ�굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲
 *******************************************************************************/

void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	unsigned char i ;
	int32_t sum=0;
	for(i=1;i<10;i++){	//FIFO 占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲

		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[0][9]=ax;//占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈뱟占쎈쿈占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솧筌뚭쑴�굲占쎈쐻占쎈뻻占쎈쿈占쎌굲 占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솧占쎈쿈占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻�뜝占�
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;

	sum=0;

	for(i=0;i<10;i++){	//占쎈쐻占쎈짗占쎌굲占쎈�뱄옙�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲筌╉뀿猷�占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈뼍吏몌옙猷욑옙�굲占쎈쐻占쎈솂�뜝占�

		sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
}


/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		void MPU6050_setClockSource(uint8_t source)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲  MPU6050 占쎈쐻占쎈짗占쎌굲占쎈♧占쎈쐻占쎈짗占쎌굲占쎄틭
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}


/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		void MPU6050_setFullScaleAccelRange(uint8_t range)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲  MPU6050 占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솏占쎌굤占쎈�욑옙踰앾옙�굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻�뜝占�
 *******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}


/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		void MPU6050_setSleepEnabled(uint8_t enabled)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲  MPU6050 占쎈쐻占쎈뼄筌뚭쑴�굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈뼑�뜝�뜴�쐻占쎈짗占쎌굲燁살뮃彛�
				enabled =1   占쎈걖占쎈쐻占쎈짗占쎌굲
			    enabled =0   占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲
 *******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		uint8_t MPU6050_getDeviceID(void)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎌궎  MPU6050 WHO_AM_I 占쎈쐻占쎈짗占쎌굲�깗占�	 占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲 0x68

 *******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

	IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
	return buffer[0];
}


/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		uint8_t MPU6050_testConnection(void)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈１PU6050 占쎈쐻占쎈뼄筌뚭쑴�굲占쎈쐻占쎈뼩占쎈섣占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲
 *******************************************************************************/
uint8_t MPU6050_testConnection(void) {
	if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
		return 1;
	else return 0;
}


/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲 MPU6050 占쎈쐻占쎈뼄筌뚭쑴�굲癲낆�쵻X I2C占쎈쐻占쎈솯占쎈쿈占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲

 *******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}


/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲 MPU6050 占쎈쐻占쎈뼄筌뚭쑴�굲癲낆�쵻X I2C占쎈쐻占쎈솯占쎈쿈占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲

 *******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}


/**************************占쎈샑占쎈쐻占쎈솂�뇡�빘�굲占쎈쐻占쎈짗占쎌굲********************************************
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲沃ㅲ뮪�쐻占쎈짗占쎌굲:		void MPU6050_initialize(void)
 *占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲:	    占쎈쐻占쎈짗占쎌굲�벀占쏙옙�쐻占쎈짗占쎌굲 	MPU6050 占쎈쐻占쎈셾占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솊占쎈뮩占쎈씮�굲占쎈쐻�뜝占�
 *******************************************************************************/
void MPU6050_initialize(void) {
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈♧占쎈쐻占쎈짗占쎌굲
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻�뜝占� +-1000占쎈쐻占쎈짗占쎌굲筌ｌ뇯�쐻占쎈짗占쎌굲
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솏占쎌굤占쎌뵛占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻�뜝占� +-2G
	MPU6050_setSleepEnabled(0); //占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈윥�눧琉룸쐻占쎈짗占쎌굲鹽뽯떥梨�
	MPU6050_setI2CMasterModeEnabled(0);	 //占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲MPU6050 占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲AUXI2C
	MPU6050_setI2CBypassEnabled(0);	 //占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲I2C占쎈쐻占쎈짗占쎌굲	MPU6050占쎈쐻占쎈짗占쎌굲AUXI2C	繞섓옙嚥싥깷�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲繞섓옙占쎈쐻占쎈셽筌뚭쑴�굲占쎈쐻占쎈짗占쎌굲HMC5883L
}




/**************************************************************************

占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솦影�瑜곸굲MPU6050占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲DMP占쎈쐻占쎈뻼占쎈꺋占쎌굲�벀占쏙옙�쐻占쎈짗占쎌굲
占쎈쐻占쎈짗占쎌굲閭잙쑚�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻�뜝占�
占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲  繞볝뀿�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲
占쎈쐻占쎈짗占쎌굲    占쎈쐻占쎈솯影�瑜곸굲占쎈뼓占쎈쐻占쎈짗占쎌굲筌껋뇯�쐻占쎈짗占쎌굲�뜏洹쏅쐻占쎈짗占쎌굲

 **************************************************************************/
void DMP_Init(void)
{ 
	uint8_t temp[1]={0};
	i2cRead(0x68,0x75,1,temp);
	//	 Flag_Show=1;

	printf("mpu_set_sensor complete ......\r\n");

	if(temp[0]!=0x68)NVIC_SystemReset();
	if(!mpu_init())
	{
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			printf("mpu_set_sensor complete ......\r\n");
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			printf("mpu_configure_fifo complete ......\r\n");
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
			printf("mpu_set_sample_rate complete ......\r\n");
		if(!dmp_load_motion_driver_firmware())
			printf("dmp_load_motion_driver_firmware complete ......\r\n");
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
			printf("dmp_set_orientation complete ......\r\n");
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |

				DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
				DMP_FEATURE_GYRO_CAL))

			printf("dmp_enable_feature complete ......\r\n");
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
			printf("dmp_set_fifo_rate complete ......\r\n");
		run_self_test();
		if(!mpu_set_dmp_state(1))
			printf("mpu_set_dmp_state complete ......\r\n");
	}
	//	Flag_Show=0;
}

float qToFloat(long number, unsigned char q)
{
	unsigned long mask;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}
/**************************************************************************
占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솦影�瑜곸굲占쎈쐻占쎈짗占쎌굲占쎌궎MPU6050占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲DMP占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲�뙶袁��쐻占쎈짗占쎌굲占쎈－
占쎈쐻占쎈짗占쎌굲閭잙쑚�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻�뜝占�
占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲  繞볝뀿�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲
占쎈쐻占쎈짗占쎌굲    占쎈쐻占쎈솯影�瑜곸굲占쎈뼓占쎈쐻占쎈짗占쎌굲筌껋뇯�쐻占쎈짗占쎌굲�뜏洹쏅쐻占쎈짗占쎌굲

 **************************************************************************/
//int8_t sign=3, sign_hold=0;//sign_back=3;
void Read_DMP(void)
{	
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	uint8_t degrees = 1;
	//	float gravity0, gravity1, gravity2, ypr0, ypr1, ypr2;

	int8_t req;

	req = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{

		dqw = quat[0] / q30; //w
		dqx = quat[1] / q30; //x
		dqy = quat[2] / q30; //y
		dqz = quat[3] / q30; //z

		float ysqr = dqy * dqy;
//		float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
//		float t1 = +2.0f * (dqx * dqy - dqw * dqz);
		float t2 = -2.0f * (dqx * dqz + dqw * dqy);
		float t3 = +2.0f * (dqy * dqz - dqw * dqx);
		float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;

		// Keep t2 within range of asin (-1, 1)
		t2 = t2 > 1.0f ? 1.0f : t2;
		t2 = t2 < -1.0f ? -1.0f : t2;

//		Pitch = asin(t2) * 2;
		Roll = atan2(t3, t4); //radian
//		Roll_reverse = atan2(t4, t3); //radian
//		Yaw = atan2(t1, t0);

		if(degrees)
		{
//			Pitch *= (180.0 / PI);
			Roll *= (180.0 / PI);
//			Roll_reverse *= (180.0 / PI);
//			Yaw *= (180.0 / PI);

			if(Cal_done) {
				Roll  -= base_roll;
//				Roll_reverse -= base_roll_reverse;
//				Pitch -= base_pitch;
//				Yaw   -= base_yaw;


				if (Roll < 0) Roll = 360.0 + Roll;
//				if (Pitch < 0) Pitch = 360.0 + Pitch;
//				if (Yaw < 0) Yaw = 360.0 + Yaw;
//				if (Roll_reverse < 0) Roll_reverse = 360.0 + Roll_reverse;



				ledPos =  (LED_TOTAL / 360.0f) * roundf(Roll);//Roll);
				ledPos = ledPos - targetLedPos;
				if (ledPos < 0) ledPos = LED_TOTAL + ledPos;
			}
		}
	}
}
/**************************************************************************
占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈솦影�瑜곸굲占쎈쐻占쎈짗占쎌굲占쎌궎MPU6050占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈뱟占쎌굤占쎈솇占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲
占쎈쐻占쎈짗占쎌굲閭잙쑚�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻�뜝占�
占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲  繞볝뀿�쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈뱟占쎌뵛占쎌굲
占쎈쐻占쎈짗占쎌굲    占쎈쐻占쎈솯影�瑜곸굲占쎈뼓占쎈쐻占쎈짗占쎌굲筌껋뇯�쐻占쎈짗占쎌굲�뜏洹쏅쐻占쎈짗占쎌굲
 **************************************************************************/
int Read_Temperature(void)
{	   
	float Temp;
	Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
	if(Temp>32768) Temp-=65536;
	Temp=(36.53+Temp/340)*10;
	return (int)Temp;
}
//------------------End of File----------------------------
