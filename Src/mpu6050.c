/**
  ******************************************************************************
  * @file    MPU6050.c
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   This file includes the MPU6050 driver functions

  ******************************************************************************
  * @attention
  *
  *Waveshare�� �ҽ��� HAL���̺귯������ ���۵ǵ��� ������
  ******************************************************************************
  */

#include "mpu6050.h"
#include "i2c.h"
#include "math.h"

//센서에서 바로 읽어온 값을 저장할 변수
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;


//변경된 각도와 최종 각도를 저장할 변수
uint32_t last_read_time;
float         last_x_angle;
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;
float         last_gyro_y_angle;
float         last_gyro_z_angle;


uint32_t get_last_time(void) {return last_read_time;}
void calibrate_sensors(void);
float get_last_x_angle(void) {return last_x_angle;}
float get_last_y_angle(void) {return last_y_angle;}
float get_last_z_angle(void) {return last_z_angle;}
float get_last_gyro_x_angle(void) {return last_gyro_x_angle;}
float get_last_gyro_y_angle(void) {return last_gyro_y_angle;}
float get_last_gyro_z_angle(void) {return last_gyro_z_angle;}

MPU6050_TypeDef MPU6050_GyroOffset, MPU6050_AccOffset;

//������ ���� 1����Ʈ �б�
// �Ķ����1 : ���� ��巹��, �Ķ���� 2 : ������ �������� ��巹��
// ���ϰ� : �������䰪

uint8_t MPU6050_ReadOneByte(uint8_t RegAddr)
{
	uint8_t Data = 0;
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,1,&Data,1,1000);
	return Data;
}

//������  1����Ʈ ����
// �Ķ����1 : ���� ��巹��, �Ķ���� 2 : ������ �������� ��巹��
// ���ϰ� : �������䰪
void MPU6050_WriteOneByte(uint8_t RegAddr, uint8_t Data)
{
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,&Data,1,1000);
}

bool MPU6050_WriteBits(uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

   uint8_t Dat, Mask;

		Dat = MPU6050_ReadOneByte(RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    MPU6050_WriteOneByte(RegAddr, Dat);

    return true;
}

bool MPU6050_WriteOneBit(uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;

    Dat = MPU6050_ReadOneByte(RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    MPU6050_WriteOneByte(RegAddr, Dat);

    return true;
}

//�����б� (����̽� ��巹��, �������� ��巹��, ������ ũ��, ���� ������)
bool MPU6050_ReadBuff(uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	// �޸� �б�(����̽� ��巹��, 8��Ʈ ��巹�� �޸� ũ��, ���� ������, ���ۼ���, �õ�Ƚ��)
	return HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,pBuff,Num,1000);
}

bool MPU6050_Check(void)
{
   	if(MPU6050_ADDRESS_AD0_LOW == MPU6050_ReadOneByte(MPU6050_RA_WHO_AM_I))
   	{
   		return true;
   	}
   	else
   	{
   		return false;
   	}
}

void MPU6050_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
	uint8_t i;

  	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;

  	*pOutVal = 0;
	for(i = 0; i < 8; i ++)
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}

void MPU6050_SetClockSource(uint8_t source)
{
    MPU6050_WriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_SetLPF(uint8_t	LowPassFilter)
{
   MPU6050_WriteOneByte(MPU6050_RA_CONFIG, LowPassFilter);
}

void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_SetSleepEnabled(uint8_t enabled)
{
    MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void MPU6050_SetI2CMasterModeEnabled(uint8_t enabled)
{
    MPU6050_WriteOneBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_SetI2CBypassEnabled(uint8_t enabled)
{
    MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_GetData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tmpr)
{
	uint8_t Buffer[14] = {0};
	int16_t InBuffer[7] = {0};
	int16_t temp;

	MPU6050_ReadBuff(MPU6050_RA_ACCEL_XOUT_H, 14, Buffer);

  InBuffer[0] = (((int16_t)Buffer[0]) << 8) | Buffer[1];
  InBuffer[1] = (((int16_t)Buffer[2]) << 8) | Buffer[3];
  InBuffer[2] = (((int16_t)Buffer[4]) << 8) | Buffer[5];

  InBuffer[3] = (((int16_t)Buffer[8]) << 8) | Buffer[9];
  InBuffer[4] = (((int16_t)Buffer[10]) << 8) | Buffer[11];
  InBuffer[5] = (((int16_t)Buffer[12]) << 8) | Buffer[13];

	temp = (((int16_t)Buffer[6]) << 8) | Buffer[7];
	InBuffer[6] = (int16_t)(temp* 10L / 34) + 3653;

	*ax = *(InBuffer + 0);
	*ay = *(InBuffer + 1);
	*az = *(InBuffer + 2);
	*gx = *(InBuffer + 3) / 32.8f;
	*gy = *(InBuffer + 4) / 32.8f;
	*gz = *(InBuffer + 5) / 32.8f;
	*tmpr = *(InBuffer + 6);

	vt100SetCursorPos( 6, 0);
	printf("%f, %f, %f, %f, %f, %f, %f\r\n",InBuffer[0], InBuffer[1], InBuffer[2], InBuffer[3], InBuffer[4], InBuffer[5], InBuffer[6]);
}

void MPU6050_InitOffset(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tmpr)
{
	uint8_t i;
	int32_t	 TempAx = 0, TempAy = 0, TempAz = 0, TempGx = 0, TempGy = 0, TempGz = 0;

 	for(i = 0; i < 32; i ++)
 	{
		MPU6050_GetData(ax,ay,az,gx,gy,gz,tmpr);

		TempAx += *ax;
		TempAy += *ay;
		TempAz += *az;
		TempGx += *gx;
		TempGy += *gy;
		TempGz += *gz;

		HAL_Delay(50);
	}

	MPU6050_AccOffset.X = TempAx >> 5;
	MPU6050_AccOffset.Y = TempAy >> 5;
	MPU6050_AccOffset.Z = TempAz >> 5;

	MPU6050_GyroOffset.X = TempGx >> 5;
	MPU6050_GyroOffset.Y = TempGy >> 5;
	MPU6050_GyroOffset.Z = TempGz >> 5;

}

void MPU6050_Init(uint8_t	lpf)
{
  //reset the whole module first
	MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);

	HAL_Delay(100);//wait for 50ms for the gyro to stable

	//SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	//MPU6050_WriteOneByte(MPU6050_RA_SMPLRT_DIV, 0x00);

	/*
	 * 0 : Internal 8MHz oscillator
	 * 1 : PLL with X axis gyroscope reference
	 * 2 : PLL with Y axis gyroscope reference
	 * 3 : PLL with Z axis gyroscope reference
	 * 4 : PLL with external 32.768kHz reference
	 * 5 : PLL with external 19.2MHz reference
	 * 6 : Reserved
	 * 7 : Stops the clock and keeps the timing generator in reset
	 */
  MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);      //0x01

  /*
   * 0 : +/-  250 degree/second
   * 1 : +/-  500 degree/second
   * 2 : +/- 1000 degree/second
   * 3 : +/- 2000 degree/second
   */
  MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);  //0x02

  /*
   * 0 : +/- 2g
   * 1 : +/- 4g
   * 2 : +/- 8g
   * 3 : +/- 16g
   */
  MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);   //0x00

	/*MPU6050_DLPF_BW_256         0x00
		MPU6050_DLPF_BW_188         0x01
		MPU6050_DLPF_BW_98          0x02
		MPU6050_DLPF_BW_42          0x03
		MPU6050_DLPF_BW_20          0x04
		MPU6050_DLPF_BW_10          0x05
		MPU6050_DLPF_BW_5           0x06
	*/
	MPU6050_SetLPF(lpf); //DLPF_CFG = 1: Fs=1khz; bandwidth=42hz

  MPU6050_SetSleepEnabled(0);
	MPU6050_SetI2CMasterModeEnabled(0);
	MPU6050_SetI2CBypassEnabled(1);

	/*
	 * 0 : active HIGH
	 * 1 : active LOW
	 */
	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);

  /*
   * 0 : push-pull
   * 1 : open drain
   */
	MPU6050_WriteOneBit( MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);

	/*
	 * 0 : emits a 50us long pulse
	 * 1 : hold high until the interrupt is cleared
	 */
	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);

	/*
	 * 0 : When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58)
	 */
	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);

	/*
	 * 0 : When set to 1, this bit enables the Data Ready interrupt,
	 *     which occurs each time a write operation to all of the sensor registers has been completed.
	 */
  //MPU6050_WriteOneBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);
  MPU6050_WriteOneBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 0);
  HAL_Delay(100);  // ���̷� ����ȭ ���

  /*
   * 0 : GYRO_RESET
   * 1 : ACCEL_RESET
   * 2 : TEMP_RESET
   */
  MPU6050_WriteOneByte(MPU6050_RA_SIGNAL_PATH_RESET, 0x07);
  HAL_Delay(50);  // ���̷� ����ȭ ���
}

//각도를 바꾸는 함수
void MPU6050_SetLastReadAngleData(uint32_t time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

void MPU6050_GetRawAccelGyro(int16_t * AccelGyro)
{
  MPU6050_GetData(&AccelGyro[0], &AccelGyro[1], &AccelGyro[2], &AccelGyro[3], &AccelGyro[4], &AccelGyro[5], &AccelGyro[6]);
}


//움직임이 없을 때 일차적으로 값을 읽어와 전역변수에 저장
void MPU6050_CalibrateSensors(void)
{
  int i = 0;
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  int16_t  AccelGyro[6]={0};


  //원시 데이터들의 평균 읽기
  for (i = 0; i < num_readings; i++)
  {
    MPU6050_GetRawAccelGyro(AccelGyro);

    x_accel += AccelGyro[0];
    y_accel += AccelGyro[1];
    z_accel += AccelGyro[2];
    x_gyro += AccelGyro[3];
    y_gyro += AccelGyro[4];
    z_gyro += AccelGyro[5];

    HAL_Delay(100);
  }

  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  //전역 변수에 저장
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
}

void MPU6050_SendSerialAccelGryro( int16_t accelgyro[6] )
{
  //  250 degree/s 범위를 가지고, 이 값을 16 bit 분해능으로 표현하고 있으므로,
  // mpu-6050에서 보내는 값은 -32766 ~ +32766 값이다.
  // 그러므로 이 값을 실제 250 degree/s 로 변환하려면 131로 나눠줘야 한다. 범위가
  // 다르면 이 값도 같이 바껴야한다.
  float FS_SEL = 131;

  //  +-2000 degree/s 범위를 가지고, 이 값을 16 bit 분해능으로 표현하고 있으므로,
  // mpu-6050에서 보내는 값은 -32766 ~ +32766 값이다.
  // 그러므로 이 값을 실제 +-2000 degree/s 로 변환하려면 16.4로 나눠줘야 한다. 범위가
  // 다르면 이 값도 같이 바껴야한다.
  //float FS_SEL = 16.4;

  //float FS_SEL2 = 16384;

  //회전을 했을  시간 알기
  unsigned long t_now = time_ms();


  float gyro_x = (accelgyro[3] - base_x_gyro)/FS_SEL;
  float gyro_y = (accelgyro[4] - base_y_gyro)/FS_SEL;
  float gyro_z = (accelgyro[5] - base_z_gyro)/FS_SEL;


// 가속도 값 범위는?
// 16bit 니 -32766 ~ +32766 범위이고,
//   +-2g 범위라면 mpu-6050으로부터 넘어온 값을 실제 g단위로 환산하려면
//    scale factor(16384)로 나눠줘야 한다. +-2g 범위는 16,384값이다. 즉 32766값이면 2가 된다.

  //acceleration 원시 데이터 저장
  float accel_x = accelgyro[0];
  float accel_y = accelgyro[1];
  float accel_z = accelgyro[2];


  //accelerometer로 부터 각도 얻기
  float RADIANS_TO_DEGREES = 180/3.14159;

  // float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;


  //gyro angles 계산1
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();

  //gyro angles 계산2
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();

  //알파를 이용해서 최종 각도 계산3
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer는 z-angle 없음


 //최종 각도 저장
  MPU6050_SetLastReadAngleData(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

#ifdef TEST_PROCESSING
  // 프로세스 테스트
  // 시작 문자 전송
  printf("S");

  printf("%d", (int16_t)dt );

  // 2 바이트 정수로 보내기 위해 100을 곱하고, 받을 때, 100을 나눠준다.
  printf("%d", (int16_t)(accel_angle_x*100) );
  printf("%d", (int16_t)(accel_angle_y*100) );
  printf("%d", (int16_t)(accel_angle_z*100) );

  printf("%d", (int16_t)(unfiltered_gyro_angle_x*100) );
  printf("%d", (int16_t)(unfiltered_gyro_angle_y*100) );
  printf("%d", (int16_t)(unfiltered_gyro_angle_z*100) );

  printf("%d", (int16_t)(angle_x*100) );
  printf("%d", (int16_t)(angle_y*100) );
  printf("%d", (int16_t)(angle_z*100) );
#else
  vt100SetCursorPos( 2, 0);
//  printf("%f, %f, %f, %f, %f, %f, %f\r\n",accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], accelgyro[6]);
  printf("angle_x : %f\r\n", angle_x);
  printf("angle_y : %f\r\n", angle_y);
  printf("angle_z : %f\r\n", angle_z);
#endif
}
