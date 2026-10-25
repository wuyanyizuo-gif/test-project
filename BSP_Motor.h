#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include <stddef.h>
#include "ti_msp_dl_config.h"

 
 typedef enum
 {
    State_Stop = 0,
    State_Forward,
    State_Backward
}Motor_State;


typedef struct
{
   GPIO_Regs* GPIO_Port;
   uint32_t   GPIO_DIR_Pin;
   
}Motor_GPIO;
    
typedef struct
{
   GPTIMER_Regs* htim_pid;
   GPTIMER_Regs* htim_pwm;
   GPTIMER_Regs* htim_encoder;
   uint32_t      TIM_Channel;
}Motor_TIM;

typedef struct
{
    int16_t Encoder_Num;
    int64_t Encoder_CountSum;

}Motor_Base;



typedef struct
{
    uint8_t     Instance;      //电机实例
    Motor_GPIO  Motor_Pin;
    Motor_TIM   Motor_Timer;
    Motor_Base  Motor_Base;
    Motor_State Motor_State;
    uint32_t    Compare;
    
}Motor_Device;


typedef struct
{
   uint8_t            Motor_Name;
   GPIO_Regs        * gpio_control;
   GPTIMER_Regs     * htim_pid;
   GPTIMER_Regs     * htim_pwm;
   GPTIMER_Regs     * htim_encoder;
   uint32_t           GPIO_DIR_Pin;
   uint32_t           TIM_Channel;
   Motor_State        Motor_State;
   
}Motor_InitConfig;

#define MOTOR_A                     0U
#define MOTOR_B                     1U

#define PWM_PERIOD                  1000U
#define PWM_MAX                     999U
#define MOTOR_PID_PERIOD_MS         10U

//电机闭环定时器句柄
#define Motor_TIM_PID               TIMA1
#define Motor_TIM_PID_INT_IRQN      TIMA1_INT_IRQn

//电机PWM定时器输出通道
#define PWM_TIM                     TIMA0
#define Left_Channel                DL_TIMER_CC_0_INDEX
#define Right_Channel               DL_TIMER_CC_1_INDEX

//电机PWM控速引脚定义
#define Left_PWM_Pin                DL_GPIO_PIN_8
#define Left_PWM_GPIO_Port          GPIOA
#define Left_PWM_IOMUX              (IOMUX_PINCM19)
#define Left_PWM_IOMUX_FUNC         IOMUX_PINCM19_PF_TIMA0_CCP0
#define Right_PWM_Pin               DL_GPIO_PIN_9
#define Right_PWM_GPIO_Port         GPIOA
#define Right_PWM_IOMUX             (IOMUX_PINCM20)
#define Right_PWM_IOMUX_FUNC        IOMUX_PINCM20_PF_TIMA0_CCP1

//电机编码器AB相引脚定义
#define Left_A_Pin                  DL_GPIO_PIN_29
#define Left_B_Pin                  DL_GPIO_PIN_30
#define Left_GPIO_Port              GPIOA
#define Left_A_IOMUX                (IOMUX_PINCM4)
#define Left_B_IOMUX                (IOMUX_PINCM5)
#define Left_A_IOMUX_FUNC           IOMUX_PINCM4_PF_TIMG8_CCP0
#define Left_B_IOMUX_FUNC           IOMUX_PINCM5_PF_TIMG8_CCP1
#define Left_Encoder_TIM            TIMG8

#define Right_A_Pin                 DL_GPIO_PIN_24
#define Right_B_Pin                 DL_GPIO_PIN_25
#define Right_GPIO_Port             GPIOA
#define Right_A_IOMUX               (IOMUX_PINCM54)
#define Right_B_IOMUX               (IOMUX_PINCM55)
#define Right_Encoder_TIM           NULL

//电机启停控制引脚定义
#define Left_DIR_Pin                DL_GPIO_PIN_4
#define Right_DIR_Pin               DL_GPIO_PIN_5
#define DIR_GPIO_Port               GPIOB
#define Left_DIR_IOMUX              (IOMUX_PINCM17)
#define Right_DIR_IOMUX             (IOMUX_PINCM18)

#define MOTOR_DIR_FORWARD           0U
#define MOTOR_DIR_BACKWARD          1U

#define Motor_InitConfig_Left   { MOTOR_A, DIR_GPIO_Port, Motor_TIM_PID, PWM_TIM, Left_Encoder_TIM,  Left_DIR_Pin,  Left_Channel,  State_Stop }
#define Motor_InitConfig_Right  { MOTOR_B, DIR_GPIO_Port, Motor_TIM_PID, PWM_TIM, Right_Encoder_TIM, Right_DIR_Pin, Right_Channel, State_Stop }

void BSP_Motor_SetCompare(GPTIMER_Regs* htim_pwm, uint32_t Motor_Channel_x, uint32_t compare);

/**
  * @brief  电机BSP层设置 CCR寄存器值
  * @param  Motor_tim 电机句柄指针
  * @param  Motor_Channel_x ；可取 Motor_Channel 中的一个值
  * @param  compare；待设置的CCR值,范围 0 ~ 65535
  * @note   在本项目中 compare值范围  0 ~ 8000
  * @retval 无
  */
#define BSP_Motor_SetCCR(Motor_tim,Motor_Channel_x,compare)      \
 do                                                              \
 {                                                               \
   BSP_Motor_SetCompare((Motor_tim),(Motor_Channel_x),(compare));\
 }while (0U)
 
        

Motor_Device* BSP_Motor_Init (Motor_InitConfig *Motor);
void BSP_Motor_SetState(Motor_Device *Motor);
void BSP_Motor_GetEncoderNum(Motor_Device*Motor);
void BSP_Motor_Stop(Motor_Device*Motor);
void BSP_Motor_Start(Motor_Device*Motor);
void BSP_Motor_StartPID(Motor_Device*Motor);
void BSP_Motor_StopPID(Motor_Device*Motor);


#endif

