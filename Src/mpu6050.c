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
#include <math.h>

// Variable definitions
float x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr, x_fil, y_fil, z_fil;
volatile float last_x_angle, last_y_angle, last_z_angle;
float ledPos = 0;

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's

int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds

int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;

float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float SelfTest[6];

// Specify sensor full scale
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;

float aRes, gRes; // scale resolutions per LSB for the sensors

// parameters for 6 DoF sensor fusion calculations
//float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
#define GyroMeasError  1.0471975511966f
#define GyroMeasDrift 0.0174532925199433

//float GyroMeasError;// = PI * (60.0f / 180.0f);
//float GyroMeasDrift;// = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f, count = 0.0f;                              // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0;     // used to calculate integration interval                               // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion


//센서에서 바로 읽어온 값을 저장할 변수
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;


//변경된 각도와 최종 각도를 저장할 변수
//uint32_t last_read_time;
//float         last_x_angle;
//float         last_y_angle;
//float         last_z_angle;
//float         last_gyro_x_angle;
//float         last_gyro_y_angle;
//float         last_gyro_z_angle;


//uint32_t get_last_time(void) {return last_read_time;}
//void calibrate_sensors(void);
//float get_last_x_angle(void) {return last_x_angle;}
//float get_last_y_angle(void) {return last_y_angle;}
//float get_last_z_angle(void) {return last_z_angle;}
//float get_last_gyro_x_angle(void) {return last_gyro_x_angle;}
//float get_last_gyro_y_angle(void) {return last_gyro_y_angle;}
//float get_last_gyro_z_angle(void) {return last_gyro_z_angle;}

MPU6050_TypeDef MPU6050_GyroOffset, MPU6050_AccOffset;

//������ ���� 1����Ʈ �б�
// �Ķ����1 : ���� ��巹��, �Ķ���� 2 : ������ �������� ��巹��
// ���ϰ� : �������䰪

uint8_t MPU6050_ReadOneByte(uint8_t RegAddr)
{
	uint8_t Data = 0;
#ifdef USE_I2C_DMA
	if(HAL_I2C_Mem_Read_DMA(&MPU6050_I2C_PORT, MPU6050_DEVICE_ADDR, RegAddr, 1, &Data, 1) != HAL_OK)
		{
			Error_Handler();
		}
#else
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,1,&Data,1,1000);
#endif
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
#ifdef USE_I2C_DMA
	if(HAL_I2C_Mem_Read_DMA(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,pBuff,Num) != HAL_OK)
	{
		Error_Handler();
	}
	else
		return HAL_OK;
#else
	return HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,pBuff,Num,1000);
#endif
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

void MPU6050_GetGres(void)
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void MPU6050_GetAres(void)
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void MPU6050_ReadAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  MPU6050_ReadBuff(MPU6050_RA_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void MPU6050_ReadGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  MPU6050_ReadBuff(MPU6050_RA_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

int16_t MPU6050_ReadTempData(void)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  MPU6050_ReadBuff(MPU6050_RA_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
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

void MPU6050_SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   float factoryTrim[6];

   // Configure the accelerometer for self-test
   MPU6050_WriteOneByte(MPU6050_RA_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   MPU6050_WriteOneByte(MPU6050_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   HAL_Delay(250);  // Delay a while to let the device execute the self-test
   rawData[0] = MPU6050_ReadOneByte(MPU6050_RA_SELF_TEST_X); // X-axis self-test results
   rawData[1] = MPU6050_ReadOneByte(MPU6050_RA_SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = MPU6050_ReadOneByte(MPU6050_RA_SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = MPU6050_ReadOneByte(MPU6050_RA_SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

 //  Output self-test results and factory trim calculation if desired
 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050_Calibration(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);//wait for 50ms for the gyro to stable

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);      //0x01
	HAL_Delay(200);//wait for stable

	// Configure device for bias calculation
	MPU6050_WriteOneBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 0); // Disable all interrupts
	MPU6050_WriteOneByte(MPU6050_RA_FIFO_EN, 0x00); // Disable FIFO
	MPU6050_WriteOneByte(MPU6050_RA_PWR_MGMT_1, 0x00); // Turn on internal clock source

	MPU6050_WriteOneByte(MPU6050_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
	MPU6050_WriteOneByte(MPU6050_RA_USER_CTRL, 0x00); // Disable FIFO and I2C master modes
	MPU6050_WriteOneByte(MPU6050_RA_USER_CTRL, 0x0C); // Reset FIFO and DMP
	HAL_Delay(15);//wait for stable

	// Configure MPU6050 gyro and accelerometer for bias calculation
	MPU6050_WriteOneByte(MPU6050_RA_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
	MPU6050_WriteOneByte(MPU6050_RA_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
	MPU6050_WriteOneByte(MPU6050_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	MPU6050_WriteOneByte(MPU6050_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec, 250dps
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g, 2g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	MPU6050_WriteOneByte(MPU6050_RA_USER_CTRL, 0x40);   // Enable FIFO
	MPU6050_WriteOneByte(MPU6050_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
	HAL_Delay(8);// accumulate 80 samples in 80 milliseconds = 960 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	MPU6050_WriteOneByte(MPU6050_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	MPU6050_ReadBuff(MPU6050_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

//	printf("\r Packet_count : %d\n", packet_count);
	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		MPU6050_ReadBuff(MPU6050_RA_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	MPU6050_WriteOneByte(MPU6050_RA_XG_OFFS_USRH, data[0]);
	MPU6050_WriteOneByte(MPU6050_RA_XG_OFFS_USRL, data[1]);
	MPU6050_WriteOneByte(MPU6050_RA_YG_OFFS_USRH, data[2]);
	MPU6050_WriteOneByte(MPU6050_RA_YG_OFFS_USRL, data[3]);
	MPU6050_WriteOneByte(MPU6050_RA_ZG_OFFS_USRH, data[4]);
	MPU6050_WriteOneByte(MPU6050_RA_ZG_OFFS_USRL, data[5]);

	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	MPU6050_ReadBuff(MPU6050_RA_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	MPU6050_ReadBuff(MPU6050_RA_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	MPU6050_ReadBuff(MPU6050_RA_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Push accelerometer biases to hardware registers
//	MPU6050_WriteOneByte(MPU6050_RA_XA_OFFSET_H, 	data[0]);
//	MPU6050_WriteOneByte(MPU6050_RA_XA_OFFSET_L_TC, data[1]);
//	MPU6050_WriteOneByte(MPU6050_RA_YA_OFFSET_H, 	data[2]);
//	MPU6050_WriteOneByte(MPU6050_RA_YA_OFFSET_L_TC, data[3]);
//	MPU6050_WriteOneByte(MPU6050_RA_ZA_OFFSET_H, 	data[4]);
//	MPU6050_WriteOneByte(MPU6050_RA_ZA_OFFSET_L_TC, data[5]);

	// Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
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
	// wake up device
//	MPU6050_SetSleepEnabled(0);
	MPU6050_WriteOneByte(MPU6050_RA_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	HAL_Delay(100);
  //reset the whole module first
//	MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
	MPU6050_WriteOneByte(MPU6050_RA_PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
//	HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

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
//  MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);      // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	/*MPU6050_DLPF_BW_256         0x00
		MPU6050_DLPF_BW_188         0x01
		MPU6050_DLPF_BW_98          0x02
		MPU6050_DLPF_BW_42          0x03
		MPU6050_DLPF_BW_20          0x04
		MPU6050_DLPF_BW_10          0x05
		MPU6050_DLPF_BW_5           0x06
	*/
  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
//	MPU6050_SetLPF(lpf); //DLPF_CFG = 1: Fs=1khz; bandwidth=42hz
	MPU6050_WriteOneByte(MPU6050_RA_CONFIG, 0x03);

	// Set sample rate
	//SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) => 1kh / 5 = 200hz
	MPU6050_WriteOneByte(MPU6050_RA_SMPLRT_DIV, 0x04); // Use a 200 Hz rate; the same rate set in CONFIG above


  /*
   * 0 : +/-  250 degree/second
   * 1 : +/-  500 degree/second
   * 2 : +/- 1000 degree/second
   * 3 : +/- 2000 degree/second
   */
	// Set gyroscope full scale range
//  MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);  //0x02
	uint8_t c =  MPU6050_ReadOneByte(MPU6050_RA_GYRO_CONFIG);
	MPU6050_WriteOneByte(MPU6050_RA_GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	MPU6050_WriteOneByte(MPU6050_RA_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	MPU6050_WriteOneByte(MPU6050_RA_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
	c =  MPU6050_ReadOneByte(MPU6050_RA_GYRO_CONFIG);
	printf("GYRO : %x\r\n", c);
  /*
   * 0 : +/- 2g
   * 1 : +/- 4g
   * 2 : +/- 8g
   * 3 : +/- 16g
   */
//  MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);   //0x00
	  c =  MPU6050_ReadOneByte(MPU6050_RA_ACCEL_CONFIG);
	  MPU6050_WriteOneByte(MPU6050_RA_ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	  MPU6050_WriteOneByte(MPU6050_RA_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	  MPU6050_WriteOneByte(MPU6050_RA_ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer
	  c =  MPU6050_ReadOneByte(MPU6050_RA_ACCEL_CONFIG);
	  printf("ACCEL : %x\r\n", c);
//  MPU6050_SetSleepEnabled(0);
//	MPU6050_SetI2CMasterModeEnabled(0);

	  MPU6050_WriteOneByte(MPU6050_RA_INT_PIN_CFG, 0x22);
	  MPU6050_WriteOneByte(MPU6050_RA_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

//	/*
//	 * 0 : active HIGH
//	 * 1 : active LOW
//	 */
//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
//
//  /*
//   * 0 : push-pull
//   * 1 : open drain
//   */
//	MPU6050_WriteOneBit( MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
//
//	/*
//	 * 0 : emits a 50us long pulse
//	 * 1 : hold high until the interrupt is cleared
//	 */
//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
//
//	/*
//	 * 0 : When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58)
//	 */
//	//MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 0);
//
//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, 0);
//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, 0);
//	MPU6050_SetI2CBypassEnabled(1);
//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, 0);
//
//	/*
//	 * 0 : When set to 1, this bit enables the Data Ready interrupt,
//	 *     which occurs each time a write operation to all of the sensor registers has been completed.
//	 */
//  //MPU6050_WriteOneBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 0);
//  MPU6050_WriteOneBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);
////  HAL_Delay(100);  // ���̷� ����ȭ ���
////
////  /*
////   * 0 : GYRO_RESET
////   * 1 : ACCEL_RESET
////   * 2 : TEMP_RESET
////   */
////  MPU6050_WriteOneByte(MPU6050_RA_SIGNAL_PATH_RESET, 0x07);
////  HAL_Delay(50);  // ���̷� ����ȭ ���
}

//각도를 바꾸는 함수
//void MPU6050_SetLastReadAngleData(uint32_t time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
//{
//  last_read_time = time;
//  last_x_angle = x;
//  last_y_angle = y;
//  last_z_angle = z;
//  last_gyro_x_angle = x_gyro;
//  last_gyro_y_angle = y_gyro;
//  last_gyro_z_angle = z_gyro;
//}

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

void MPU6050_Reset( void ) {
  // reset device
	MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);//wait for 50ms for the gyro to stable
  }
