#include "mpu6050_dmp.h"
#include "HAL_I2C.h"
#include "main.h"
#include "ema_filter.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

#define RAD_TO_DEG 57.295779513082320876798154814105

#define q30  1073741824.0f // 2^30
short gyro[3], accel[3], sensors;
float Pitch, Roll, Roll_reverse, Yaw, Rangle=0.0f, Pangle=0.0f;

float base_pitch=0.0f, base_roll=0.0f, base_yaw=0.0f, base_roll_reverse=0.0f;
float dqw=1.0f, dqx=0.0f, dqy=0.0f, dqz=0.0f, sign=0.0f;
float ledPos = 0;
float targetLedPos = 0;
float targetAnglel = 120.0f;
uint8_t Cal_done = 0;

static signed char gyro_orientation[9] =
//	  {-1, 0, 0,
//		0,-1, 0,
//		0, 0, 1};
		{  0, 1, 0,
		 -1, 0, 0,
		  0, 0, 1  };
//{  0, -1, 0,
//  1, 0, 0,
//  0, 0, 1  };

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

uint8_t buffer[14];
int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

/************************** 구현 기능 ***********************************************
* 함수 프로토 타입 : void MPU6050_newValues ​​(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
* 기능 : 필터링을 위해 새로운 ADC 데이터를 FIFO 어레이로 업데이트
*********************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	unsigned char i ;
	int32_t sum=0;
	for(i=1;i<10;i++){	//FIFO

		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[0][9]=ax;  // 데이터 끝에 새 데이터를 배치
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;

	sum=0;

	for(i=0;i<10;i++){	// 현재 배열의 합을 찾아 평균을 구함

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


/************************** 구현 기능 ***********************************************
* 함수 프로토 타입 : void MPU6050_setClockSource (uint8_t source)
* 기능 : 설정 MPU6050 클럭 소스
 * CLK_SEL | 클럭 소스
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


/************************** 구현 기능 ***********************************************
* 함수 프로토 타입 : void MPU6050_setFullScaleAccelRange (uint8_t range)
* 기능 : MPU6050 가속도계의 최대 범위 설정
**************************************************** *****************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}


/************************** 구현 기능 ***********************************************
* 프로토 타입 : void MPU6050_setSleepEnabled (uint8_t enabled)
* 기능 : 절전 모드
* 1 슬립모드
* 0 동작모드
*********************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/************************** 구현 기능 ***********************************************
* 프로토 타입 : uint8_t MPU6050_getDeviceID (void)
* 기능 : MPU6050 WHO_AM_I 플래그를 읽으면 0x68이 반환됨.
*********************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

	IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
	return buffer[0];
}


/************************** 구현 기능 ***********************************************
* 프로토 타입 : uint8_t MPU6050_testConnection (void)
* 기능 : MPU6050이 연결되어 있는지 확인
*********************************************************************************/
uint8_t MPU6050_testConnection(void) {
	if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
		return 1;
	else return 0;
}


/************************** 구현 기능 ***********************************************
* 함수 프로토 타입 : void MPU6050_setI2CMasterModeEnabled (uint8_t enabled)
* 기능 : MPU6050이 AUX I2C 호스트 설정
**************************************************** *****************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}


void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}


/************************** 구현 기능 ***********************************************
* 함수 프로토 타입 : void MPU6050_initialize (void)
* 기능 : MPU6050을 초기화.
*********************************************************************************/
void MPU6050_initialize(void) {
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //클럭설정
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//자이로 최대범위 +/- 1000 도 (초당)
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//가속도 최대범위 +/- 2g
	MPU6050_setSleepEnabled(0); //동작모드 설정
	MPU6050_setI2CMasterModeEnabled(0);	 //auxi2c off
	MPU6050_setI2CBypassEnabled(0);	 //bypass auxi2c
}
//#define CAL_MPU
void run_self_test(void)
{

	long gyro[3], accel[3];
#ifdef CAL_MPU
		int result;
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

		printf("\rgyro : %7.4f, %7.4f, %7.4f\n",
				(float)gyro[0]/1.0f,
				(float)gyro[1]/1.0f,
				(float)gyro[2]/1.0f);

		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;

		printf("\raccel:   %7.4f, %7.4f, %7.4f\n",
				(float)accel[0]/1.0f,
				(float)accel[1]/1.0f,
				(float)accel[2]/1.0f);

		dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
	}
#else
	//250
//	gyro[0] = (long)-7150301;
//	gyro[1] = (long)-2034715;
//	gyro[2] = (long)-998038;

//2g
//	gyro[0] = (long)-7523532;
//	gyro[1] = (long)1612185;
//	gyro[2] = (long)-335872;

	//new 2g <=
	gyro[0] = (long)-9135718;
	gyro[1] = (long)-2418278;
	gyro[2] = (long)-1880883;

	//원통 가운데 상이 0도 기준
//	gyro[0] = (long)-7456358;
//	gyro[1] = (long)1813708;
//	gyro[2] = (long)-1410662;

	//8g
//	gyro[0] = (long)-8262451;
//	gyro[1] = (long)-2082406;
//	gyro[2] = (long)-1545011;

	//8g-2
//	gyro[0] = (long)-17263820;
//	gyro[1] = (long)-4433510;
//	gyro[2] = (long)-2552627;

	//16g
//	gyro[0] = (long)-9270067;
//	gyro[1] = (long)-2552627;
//	gyro[2] = (long)-1410662;
	dmp_set_gyro_bias(gyro);

	//250
//	accel[0] = (long)8814592;
//	accel[1] = (long)-11255808;
//	accel[2] = (long)-10797056;

	//2g
//	accel[0] = (long)88342528;
//	accel[1] = (long)-34340864;
//	accel[2] = (long)-72613888;

	//new 2g <=
	accel[0] = (long)53477376;
	accel[1] = (long)-47710208;
	accel[2] = (long)-8781824;

	//원통 가운데 상이 0도 기준
//	accel[0] = (long)-246415360;
//	accel[1] = (long)-21626880;
//	accel[2] = (long)-43646976;

	//8g
//	accel[0] = (long)8945664;
//	accel[1] = (long)-3899392;
//	accel[2] = (long)-1867776;

	//8g-2
//	accel[0] = (long)16252928;
//	accel[1] = (long)-2359296;
//	accel[2] = (long)-5308416;


	//16g
//	accel[0] = (long)10354688;
//	accel[1] = (long)-1703936;
//	accel[2] = (long)-1114112;
	dmp_set_accel_bias(accel);

	//250
//	accel[0] =  0.0156f;
//	accel[1] = -0.0111f;
//	accel[2] = -0.0117f;
//	gyro[0] = -6.7138f;
//	gyro[1] = -1.6818f;
//	gyro[2] = -0.9470f;


	//TEST 1 2g
//    accel[0] =  0.0779f;
//    accel[1] = -0.0310f;
//    accel[2] = -0.0649f;//
//    gyro[0] = -5.9375f;
//    gyro[1] =  1.1875f;
//    gyro[2] = -0.3125f;

	//TEST 2 2g <=
    accel[0] =  0.0491f;
    accel[1] = -0.0376f;
    accel[2] = -0.0042f;
    gyro[0] = -6.8750f;
    gyro[1] = -1.7500f;
    gyro[2] = -1.3750f;

	//원통 가운데 상이 0도 기준
//    accel[0] = -0.2294f;
//    accel[1] = -0.0187f;
//    accel[2] = -0.0431f;
//    gyro[0] = -6.8750f;
//    gyro[1] = -1.7500f;
//    gyro[2] = -0.3125f;

	//TEST 2 8g
//    accel[0] =  0.0443f;
//    accel[1] = -0.0151f;
//    accel[2] = -0.0018f;
//    gyro[0] = -8.6875f;
//    gyro[1] = -1.6875f;
//    gyro[2] = -1.4375f;

	//8g -2
//    accel[0] =  0.0735f;
//    accel[1] = -0.0010f;
//    accel[2] = -0.0154f;
//    gyro[0] = -8.9062f;
//    gyro[1] = -2.4688f;
//    gyro[2] = -1.3438f;


    //16g
//    accel[0] =  0.0762f;
//    accel[1] = -0.0117f;
//    accel[2] = -0.0049f;
//    gyro[0] = -7.9375f;
//    gyro[1] = -2.2500f;
//    gyro[2] = -1.0625f;
#endif

    for(int i = 0; i<3; i++) {
//    	gyro[i] = (long)(gyro[i] * 16.384f); //convert to +-2000dps
//    	accel[i] *= 16384.0f;//2048.f; //convert to +-16G
//    	accel[i] = accel[i] >> 16;
//    	gyro[i] = (long)(gyro[i] >> 16);
    	gyro[i] = (long)(gyro[i] * 16.384f);//32.8f); //convert to +-1000dps
    	accel[i] *= 4096.0f; //convert to +-8G
    	accel[i] = accel[i] >> 16;
    	gyro[i] = (long)(gyro[i] >> 16);
    }

    mpu_set_gyro_bias_reg(gyro);
    mpu_set_accel_bias_reg(accel);
    printf("setting bias succesfully ......\r\n");

}

#define CAL_FRS
//static inline
void run_self_test2(void)
{

    long gyro[3], accel[3];

#ifdef CAL_FRS
    int result;
    result = mpu_run_self_test(gyro, accel);

    if (result == 0x7) {
    	printf("\rPassed!\n");
        printf("\raccel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        printf("\rgyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
#else
//        accel[0] =  0.0779f;
//        accel[1] = -0.0310f;
//        accel[2] = -0.0649f;
//
//        gyro[0] = -5.9375f;
//        gyro[1] =  1.1875f;
//        gyro[2] = -0.3125f;
//
    	//원통 가운데 상이 0도 기준
        accel[0] = -0.2294f;
        accel[1] = -0.0187f;
        accel[2] = -0.0431f;
        gyro[0] = -6.8750f;
        gyro[1] = -1.7500f;
        gyro[2] = -0.3125f;
#endif
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/
#define USE_CAL_HW_REGISTERS
        #ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 16.384f);//32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);
        mpu_set_accel_bias_reg(accel);
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif

#ifdef CAL_FRS
    }
    else {
            if (!(result & 0x1))
                printf("\rGyro failed.\n");
            if (!(result & 0x2))
            	printf("\rAccel failed.\n");
            if (!(result & 0x4))
            	printf("\rCompass failed.\n");
     }
#endif
}

/****************************************************************************
 * 기능 : MPU6050의 내장 DMP 초기화
 * 입력 매개 변수 : 없음
 * 반환 값 : None
****************************************************************************/
void DMP_Init(void)
{ 
	uint8_t temp[1]={0};
	i2cRead(0x68,0x75,1,temp);
	//	 Flag_Show=1;

	printf("mpu_set_sensor complete ......\r\n");

	if(temp[0]!=0x68)NVIC_SystemReset();
	if(!mpu_init())
	{
//		run_self_test2();
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
				DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_CAL_GYRO))//|
				//DMP_FEATURE_GYRO_CAL))
			printf("dmp_enable_feature complete ......\r\n");
		//run_self_test를 먼저 실행하고 run_self_test2 함수를 실행
		run_self_test();
//		run_self_test2();
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
			printf("dmp_set_fifo_rate complete ......\r\n");
		if(!mpu_set_dmp_state(1))
			printf("mpu_set_dmp_state complete ......\r\n");
	}
	Cal_done = 1;
	//	Flag_Show=0;
}

/****************************************************************************
 * 기능 : MPU6050 내장 DMP의 자세 정보를 읽습니다.
 * 입력 매개 변수 : 없음
 * 반환 값 : None
****************************************************************************/
void Read_DMP(void)
{	
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{

		dqw = quat[0] / q30; //w
		dqx = quat[1] / q30; //x
		dqy = quat[2] / q30; //y
		dqz = quat[3] / q30; //z

		Roll = atan2(2 * dqy * dqz + 2 * dqw * dqx, -2 * dqx * dqx - 2 * dqy* dqy + 1); // roll

		Roll *= (180.0 / PI);

		if(Cal_done) {
			if (Roll < 0) Roll = 360.0 + Roll;
			//
			ledPos =  (LED_TOTAL / 360.0f) * roundf(Roll);
			ledPos = ledPos - targetLedPos;
			if (ledPos < 0) ledPos = LED_TOTAL + ledPos;
		}
	}
}

/****************************************************************************
 * 기능 : MPU6050 내장 온도 센서에서 데이터 읽기
 * 입력 매개 변수 : 없음
 * 반환 값 : 섭씨
****************************************************************************/
int Read_Temperature(void)
{	   
	float Temp;
	Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
	if(Temp>32768) Temp-=65536;
	Temp=(36.53+Temp/340)*10;
	return (int)Temp;
}
//------------------End of File----------------------------
