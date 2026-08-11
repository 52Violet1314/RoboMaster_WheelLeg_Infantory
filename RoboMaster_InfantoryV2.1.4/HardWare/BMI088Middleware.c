#include "BMI088Middleware.h"
#include "main.h"

#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

/**
************************************************************************
* @brief:      	BMI088_GPIO_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088������GPIO��ʼ������
************************************************************************
**/
void BMI088_GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = ACC_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ACC_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GYRO_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GYRO_CS_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_com_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088������ͨ�ų�ʼ������
************************************************************************
**/
void BMI088_com_init(void)
{
    HAL_SPI_MspInit(&BMI088_USING_SPI_UNIT);
    HAL_SPI_Init(&BMI088_USING_SPI_UNIT);
}
/**
************************************************************************
* @brief:      	BMI088_delay_ms(uint16_t ms)
* @param:       ms - Ҫ�ӳٵĺ�����
* @retval:     	void
* @details:    	�ӳ�ָ���������ĺ���������΢���ӳ�ʵ��
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_delay_us(uint16_t us)
* @param:       us - Ҫ�ӳٵ�΢����
* @retval:     	void
* @details:    	΢�뼶�ӳٺ�����ʹ��SysTick��ʱ��ʵ��
************************************************************************
**/
void BMI088_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = us;

    while ((HAL_GetTick() - tickstart) < wait)
    {
    }
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088���ٶȼ�Ƭѡ�ź��õͣ�ʹ�䴦��ѡ��״̬
************************************************************************
**/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088���ٶȼ�Ƭѡ�ź��øߣ�ʹ�䴦�ڷ�ѡ��״̬
************************************************************************
**/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088������Ƭѡ�ź��õͣ�ʹ�䴦��ѡ��״̬
************************************************************************
**/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088������Ƭѡ�ź��øߣ�ʹ�䴦�ڷ�ѡ��״̬
************************************************************************
**/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_read_write_byte(uint8_t txdata)
* @param:       txdata - Ҫ���͵�����
* @retval:     	uint8_t - ���յ�������
* @details:    	ͨ��BMI088ʹ�õ�SPI���߽��е��ֽڵĶ�д����
************************************************************************
**/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

