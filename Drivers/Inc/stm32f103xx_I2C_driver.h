

#ifndef STM32F103XX_I2C_DRIVER_H_
#define STM32F103XX_I2C_DRIVER_H_

#include "stm32f103xx.h"

typedef struct
{
	uint8_t  I2C_DeviceAddress; /* Defined by user */
	uint8_t  I2C_ACK_Control;   /* Ref @I2C_ACK_Control */
	uint16_t I2C_FmDutyCycle;   /* Ref @I2C_FmDutyCycle */
	uint32_t I2C_SCL_Speed;     /* Ref @I2C_SCL_Speed */
} I2C_Config_s;

typedef struct{
	I2c_RegDef_s * I2Cx_p;  /* I2C Instance address */
	I2C_Config_s I2C_Config;
} I2C_Handle_s;

typedef enum
{
	I2C_Write_Mode = 0,
	I2C_Read_Mode,
} I2C_RW_Mode_e;

/*  
	@I2C_ACK_Control
*/

typedef enum
{
	I2C_Ack_Disable = 0,
	I2C_Ack_Enable
} I2C_ACK_Control_e;

/*
	@I2C_FmDutyCycle
*/
#define I2C_FM_DUTY_CYCLE_2 				0
#define I2C_FM_DUTY_CYCLE_16_9 				1u

/*
	@I2C_SCL_Speed
*/

#define I2C_SCL_SPEED_SM_100K 				100000u
#define I2C_SCL_SPEED_FM_200K 				200000u
#define I2C_SCL_SPEED_FM_400K 				400000u

typedef enum
{
	I2C_RepeatedStart_Disable = 0,
	I2C_RepeatedStart_Enable
} I2C_RepeatedStart_e;

/*
	APIs
*/
/* I2C peripheral clock enable and disable */
void I2C_PCLK_Control(I2C_RegDef_s * I2Cx_p, Status_e EnOrDi_e);
void I2C_PERIPH_Enable(I2C_RegDef_s * I2Cx_p, Status_e EnOrDi_e);

/* I2C Init, DeInit*/
void I2C_Init(I2C_Handle_s * I2C_Handle_p);
void I2C_DeInit(I2C_RegDef_s * I2Cx_p);

/* Send and receive data */
void I2C_MasterSendData(I2C_Handle_s * I2C_Handle_p, uint8_t * TxBuffer_p,uint32_t Len_u32, uint8_t SlaveAddress, I2C_RepeatedStart_e RepeatedStart_e);
void I2C_MasterReceiveData(I2C_Handle_s * I2C_Handle_p, uint8_t * RxBuffer_p,uint32_t Len_u32, uint8_t SlaveAddress, I2C_RepeatedStart_e RepeatedStart_e);

/* I2C interrupt Configuration and handling */
void I2C_IRQ_Config(uint8_t IRQ_Number_u8, Status_e EnOrDi_e);
void I2C_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8);


#endif