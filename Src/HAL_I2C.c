/**************
  I2C - inv_mpu.c의 읽기 - 쓰기 인터페이스
***************/

#include "HAL_I2C.h"

extern I2C_HandleTypeDef hi2c1;

#define MpuI2c hi2c1

int i2cRead(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *tmp)
{
    int req;
    req = HAL_I2C_Mem_Read( &MpuI2c, addr<<1, reg,I2C_MEMADD_SIZE_8BIT,tmp,len,1000);
    if(!req) return 0;
    else return -1;
}

int i2cWrite(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *tmp)
{
    int req;
    req = HAL_I2C_Mem_Write( &MpuI2c, addr<<1, reg,I2C_MEMADD_SIZE_8BIT,tmp,len,1000);
    if(!req) return 0;
    else return -1;
}

/************************** 구현 기능 ***********************************************
* 프로토 타입 : uint8_t IICwriteBit (uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t 데이터)
* 기능 : 지정된 디바이스의 지정된 레지스터의 1 바이트에서 1 비트 쓰기 수정 읽기
* addr 대상 장치 주소를 입력
* 등록 레지스터 주소
* 대상 바이트를 비트수
* 데이터가 0이면 대상 비트가 지워지고, 그렇지 않으면 설정됨
* 반환 값 성공 1, 실패는 0
**************************************************** *****************************/
uint8_t IICwriteBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t Byte;
    
    HAL_I2C_Mem_Read( &MpuI2c,addr<<1, reg,1,&Byte,1,1000);
    Byte = (data != 0) ? (Byte | (1 << bitNum)) : (Byte & ~(1 << bitNum));
    int req;
    req = HAL_I2C_Mem_Write( &MpuI2c, addr<<1, reg,1,&Byte,1,1000);

    if(req) return 0;
    else return -1;
}


/************************** 구현 기능 ***********************************************
프로토 타입 : uint8_t IICreadBytes (uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
* 기능 : 지정된 장치의 지정된 레지스터의 길이 값을 읽음.
* addr 대상 장치 주소
* 등록 레지스터 주소
* 읽을 길이 바이트 수
* 데이터 포인터
* 리턴값 : 읽은 바이트 수를 반환
*********************************************************************************/
uint8_t IICreadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t req;
  req = i2cRead(addr, reg, length, data);
  if(req) return 0;
  else return -1;
}

uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/************************** 구현 기능 ***********************************************
* 함수 프로토 타입 : uint8_t IICwriteBits (uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
* 기능 : 지정된 장치의 지정된 레지스터의 1 바이트에서 여러 비트를 쓰는 수정 수정
* 개발 대상 장치 주소,
* 입력 등록 레지스터 주소,
* bitStart,
* 대상 바이트의 시작 비트 길이,
* 바이트의 값을 저장
* 반환 값 성공 1, 실패는 0
*********************************************************************************/
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t byte;
    if (IICreadByte(dev, reg, &byte) != 0) 
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        byte &= mask;
        byte |= data;

        return 1;
    }
    return 0;
}

uint8_t I2C_ReadOneByte(uint8_t addr,uint8_t reg)
{
  uint8_t tmp;
  i2cRead(addr, reg, 1, &tmp);
  return tmp;
}
