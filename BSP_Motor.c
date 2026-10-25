#include "BSP_Motor.h"

#define Motor_Device_MAXNUM   2

static Motor_Device Motor_Device_Pool[Motor_Device_MAXNUM] = {0};

static uint8_t Device_Num = 0;

static uint8_t SoftwareEncoder_Num = 0;

static volatile int32_t SoftwareEncoder_Should = 0;

static volatile uint8_t SoftwareEncoder_Last = 0;

static const int8_t s_QTable[4][4] =
{
    { 0, +1, -1,  0},
    {-1,  0,  0, +1},
    {+1,  0,  0, -1},
    { 0, -1, +1,  0},
};

#ifdef CPUCLK_FREQ
#define MOTOR_CPUCLK_HZ        (CPUCLK_FREQ)
#else
#define MOTOR_CPUCLK_HZ        80000000U
#endif

#if (MOTOR_CPUCLK_HZ >= 48000000U)
#define MOTOR_PWM_DIV          DL_TIMER_CLOCK_DIVIDE_8
#else
#define MOTOR_PWM_DIV          DL_TIMER_CLOCK_DIVIDE_4
#endif

#define MOTOR_PID_DIV          DL_TIMER_CLOCK_DIVIDE_8
#define MOTOR_PID_PRESCALE     3U
#define MOTOR_PID_CLK_HZ       (MOTOR_CPUCLK_HZ / 8U / (MOTOR_PID_PRESCALE + 1U))
#define MOTOR_PID_LOAD_VALUE   ((MOTOR_PID_CLK_HZ / 1000U) * MOTOR_PID_PERIOD_MS - 1U)

static const DL_TimerA_ClockConfig gPWM_ClockConfig =
{
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = MOTOR_PWM_DIV,
    .prescale    = 0U
};

static const DL_TimerA_PWMConfig gPWM_Config =
{
    .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period            = PWM_PERIOD,
    .isTimerWithFourCC = true,
    .startTimer        = DL_TIMER_STOP
};

static const DL_TimerG_ClockConfig gQEI_ClockConfig =
{
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U
};

static const DL_TimerA_ClockConfig gPID_ClockConfig =
{
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = MOTOR_PID_DIV,
    .prescale    = MOTOR_PID_PRESCALE
};

static const DL_TimerA_TimerConfig gPID_TimerConfig =
{
    .period     = MOTOR_PID_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP
};

static void BSP_Motor_HWInit(void);
static Motor_Device* BSP_Motor_Find(GPTIMER_Regs* htim_pwm, uint32_t channel);
static void BSP_Motor_WriteCompare(Motor_Device*Motor, uint32_t compare);


static void BSP_Motor_HWInit(void)
{
    DL_GPIO_enablePower(DIR_GPIO_Port);
    DL_GPIO_enablePower(Left_GPIO_Port);

    DL_GPIO_initDigitalOutput(Left_DIR_IOMUX);
    DL_GPIO_clearPins(DIR_GPIO_Port, Left_DIR_Pin);
    DL_GPIO_enableOutput(DIR_GPIO_Port, Left_DIR_Pin);

    DL_GPIO_initDigitalOutput(Right_DIR_IOMUX);
    DL_GPIO_clearPins(DIR_GPIO_Port, Right_DIR_Pin);
    DL_GPIO_enableOutput(DIR_GPIO_Port, Right_DIR_Pin);

    DL_GPIO_initPeripheralOutputFunction(Left_PWM_IOMUX, Left_PWM_IOMUX_FUNC);
    DL_GPIO_enableOutput(Left_PWM_GPIO_Port, Left_PWM_Pin);

    DL_GPIO_initPeripheralOutputFunction(Right_PWM_IOMUX, Right_PWM_IOMUX_FUNC);
    DL_GPIO_enableOutput(Right_PWM_GPIO_Port, Right_PWM_Pin);

    DL_TimerA_reset(PWM_TIM);
    DL_TimerA_enablePower(PWM_TIM);
    delay_cycles(16U);

    DL_TimerA_setClockConfig(PWM_TIM, (DL_TimerA_ClockConfig *)&gPWM_ClockConfig);
    DL_TimerA_initPWMMode(PWM_TIM, (DL_TimerA_PWMConfig *)&gPWM_Config);

    DL_TimerA_setCounterControl(PWM_TIM, DL_TIMER_CZC_CCCTL0_ZCOND, DL_TIMER_CAC_CCCTL0_ACOND, DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_TIM, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_ENABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(PWM_TIM, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_TIM, PWM_PERIOD, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_TIM, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_ENABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(PWM_TIM, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_TIM, PWM_PERIOD, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_TIM);
    DL_TimerA_setCCPDirection(PWM_TIM, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);

    DL_GPIO_initPeripheralInputFunction(Left_A_IOMUX, Left_A_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(Left_B_IOMUX, Left_B_IOMUX_FUNC);

    DL_TimerG_reset(Left_Encoder_TIM);
    DL_TimerG_enablePower(Left_Encoder_TIM);
    delay_cycles(16U);

    DL_TimerG_setClockConfig(Left_Encoder_TIM, (DL_TimerG_ClockConfig *)&gQEI_ClockConfig);
    DL_TimerG_configQEI(Left_Encoder_TIM, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(Left_Encoder_TIM, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(Left_Encoder_TIM, 65535U);
    DL_TimerG_enableClock(Left_Encoder_TIM);
    DL_TimerG_startCounter(Left_Encoder_TIM);

    DL_GPIO_initDigitalInputFeatures(Right_A_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(Right_B_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setUpperPinsPolarity(Right_GPIO_Port, DL_GPIO_PIN_24_EDGE_RISE_FALL |
        DL_GPIO_PIN_25_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(Right_GPIO_Port, Right_A_Pin | Right_B_Pin);

    SoftwareEncoder_Last = (uint8_t)(((DL_GPIO_readPins(Right_GPIO_Port, Right_A_Pin) != 0U) ? 0x02U : 0x00U) |
                                     ((DL_GPIO_readPins(Right_GPIO_Port, Right_B_Pin) != 0U) ? 0x01U : 0x00U));

    DL_TimerA_reset(Motor_TIM_PID);
    DL_TimerA_enablePower(Motor_TIM_PID);
    delay_cycles(16U);

    DL_TimerA_setClockConfig(Motor_TIM_PID, (DL_TimerA_ClockConfig *)&gPID_ClockConfig);
    DL_TimerA_initTimerMode(Motor_TIM_PID, (DL_TimerA_TimerConfig *)&gPID_TimerConfig);
    DL_TimerA_enableInterrupt(Motor_TIM_PID, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_SetPriority(Motor_TIM_PID_INT_IRQN, 2);
    DL_TimerA_enableClock(Motor_TIM_PID);
}


static Motor_Device* BSP_Motor_Find(GPTIMER_Regs* htim_pwm, uint32_t channel)
{
    uint8_t i;

    for (i = 0U; i < Device_Num; i++)
    {
        if ((Motor_Device_Pool[i].Motor_Timer.htim_pwm == htim_pwm) &&
            (Motor_Device_Pool[i].Motor_Timer.TIM_Channel == channel))
        {
            return &Motor_Device_Pool[i];
        }
    }

    return NULL;
}


static void BSP_Motor_WriteCompare(Motor_Device*Motor, uint32_t compare)
{
    uint32_t ccr;
    uint8_t dir_level;

    if (compare > PWM_MAX)
    {
        compare = PWM_MAX;
    }

    if ((Motor->Motor_State == State_Stop) || (compare == 0U))
    {
        dir_level = 0U;
        ccr = PWM_PERIOD;
    }
    else if (Motor->Motor_State == State_Backward)
    {
        dir_level = MOTOR_DIR_BACKWARD;
        ccr = PWM_PERIOD - compare;
    }
    else
    {
        dir_level = MOTOR_DIR_FORWARD;
        ccr = compare;
    }

    if (dir_level != 0U)
    {
        DL_GPIO_setPins(Motor->Motor_Pin.GPIO_Port, Motor->Motor_Pin.GPIO_DIR_Pin);
    }
    else
    {
        DL_GPIO_clearPins(Motor->Motor_Pin.GPIO_Port, Motor->Motor_Pin.GPIO_DIR_Pin);
    }

    DL_TimerA_setCaptureCompareValue(Motor->Motor_Timer.htim_pwm, ccr, Motor->Motor_Timer.TIM_Channel);
}


/**
  * @brief  电机BSP层初始化
  * @param  Motor 电机初始化配置指针
  * @retval Motor_Device* 电机设备结构体指针
  * @note   根据 Motor_InitConfig 配置电机设备结构体并返回其指针
  */
Motor_Device* BSP_Motor_Init (Motor_InitConfig *Motor)
{
    
  if(Motor == NULL || Motor->gpio_control == NULL || Motor->htim_pwm == NULL)
  {
    return NULL;
  }
  if(Device_Num >= Motor_Device_MAXNUM)
  {
    return NULL;
  }
    Motor_Device* Dev = NULL;

    if((Motor->htim_encoder == NULL) && (SoftwareEncoder_Num >= 1U))
    {
        return NULL;
    }

    if(Device_Num == 0U)
    {
        BSP_Motor_HWInit();
    }

    Motor_Device_Pool[Device_Num].Instance                    = Motor ->Motor_Name;
    Motor_Device_Pool[Device_Num].Motor_State                 = Motor ->Motor_State;
    Motor_Device_Pool[Device_Num].Motor_Pin.GPIO_Port         = Motor ->gpio_control;
    Motor_Device_Pool[Device_Num].Motor_Pin.GPIO_DIR_Pin      = Motor ->GPIO_DIR_Pin;
    Motor_Device_Pool[Device_Num].Motor_Timer.htim_pid        = Motor ->htim_pid;
    Motor_Device_Pool[Device_Num].Motor_Timer.htim_pwm        = Motor ->htim_pwm;
    Motor_Device_Pool[Device_Num].Motor_Timer.htim_encoder    = Motor ->htim_encoder;
    Motor_Device_Pool[Device_Num].Motor_Timer.TIM_Channel     = Motor ->TIM_Channel;
    Motor_Device_Pool[Device_Num].Motor_Base.Encoder_Num      = 0;
    Motor_Device_Pool[Device_Num].Motor_Base.Encoder_CountSum = 0;
    Motor_Device_Pool[Device_Num].Compare                     = 0U;

    if(Motor->htim_encoder == NULL)
    {
        SoftwareEncoder_Num++;
    }

    Dev = &Motor_Device_Pool[Device_Num];
    Device_Num++;

    DL_GPIO_clearPins(Motor->gpio_control, Motor->GPIO_DIR_Pin);

    DL_TimerA_setCaptureCompareValue(Motor->htim_pwm, PWM_PERIOD, Motor->TIM_Channel);
    
   
    return Dev;
}



/**
  * @brief  电机BSP层设置 运行时电机转动状态
  * @param  Motor 电机句柄指针
  * @note   根据 Motor_Handle->Motor_State 设置电机转动状态
  * @retval 无
  */
	void BSP_Motor_SetState(Motor_Device*Motor)
  {

    switch (Motor->Motor_State)
    {
    case State_Stop:
        BSP_Motor_WriteCompare(Motor, 0U);
        break;
    
    case State_Forward:
        BSP_Motor_WriteCompare(Motor, Motor->Compare);
        break;

    case State_Backward:
        BSP_Motor_WriteCompare(Motor, Motor->Compare);
        break;
    
    default:
        break;
    }

  }

  /**
  * @brief  电机BSP层终止运行
  * @param  Motor 电机句柄指针
  * @note   该函数在系统错误状态下调用以终止电机运行，停止PWM输出和编码器计数
  * @retval 无
  */
  void BSP_Motor_Stop(Motor_Device*Motor)
  {
    
    if (Motor->Motor_Timer.htim_encoder != NULL)
    {
        DL_TimerG_stopCounter(Motor->Motor_Timer.htim_encoder);                          // 终止编码器计数
    }
    else
    {
        DL_GPIO_disableInterrupt(Right_GPIO_Port, Right_A_Pin | Right_B_Pin);
    }

    DL_GPIO_clearPins(Motor->Motor_Pin.GPIO_Port, Motor->Motor_Pin.GPIO_DIR_Pin);        // 终止PWM输出
    DL_TimerA_setCaptureCompareValue(Motor->Motor_Timer.htim_pwm, PWM_PERIOD, Motor->Motor_Timer.TIM_Channel);  //清零PWM占空比

    if (Motor->Motor_Timer.htim_encoder != NULL)
    {
        DL_TimerG_setTimerCount(Motor->Motor_Timer.htim_encoder, 0);                     //清零编码器计数器以便下次获取时为增量值
    }
    else
    {
        SoftwareEncoder_Should = 0;
    }
  }


   /**
  * @brief  电机BSP层启动运行
  * @param  Motor 电机句柄指针
  * @note   该函数在系统初始化或错误状态复位后执行以启动电机运行，启动PWM输出和编码器计数
  * @retval 无
  */
  void BSP_Motor_Start(Motor_Device*Motor)
  {
    DL_TimerA_startCounter(Motor->Motor_Timer.htim_pwm);                                   // 启动PWM输出
    if (Motor->Motor_Timer.htim_encoder != NULL)
    {
        DL_TimerG_startCounter(Motor->Motor_Timer.htim_encoder);                           // 启动编码器计数
        DL_TimerG_setTimerCount(Motor->Motor_Timer.htim_encoder, 0);                       //清零编码器计数器以便下次获取时为增量值
    }
    else
    {
        SoftwareEncoder_Should = 0;
        SoftwareEncoder_Last = (uint8_t)(((DL_GPIO_readPins(Right_GPIO_Port, Right_A_Pin) != 0U) ? 0x02U : 0x00U) |
                                         ((DL_GPIO_readPins(Right_GPIO_Port, Right_B_Pin) != 0U) ? 0x01U : 0x00U));
        DL_GPIO_clearInterruptStatus(Right_GPIO_Port, Right_A_Pin | Right_B_Pin);
        DL_GPIO_enableInterrupt(Right_GPIO_Port, Right_A_Pin | Right_B_Pin);
        NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
        NVIC_SetPriority(GPIOA_INT_IRQn, 0);
        NVIC_EnableIRQ(GPIOA_INT_IRQn);
    }
  } 

  /**
  * @brief  电机BSP层获取编码器计数值
  * @param  Motor 电机句柄指针
  * @note   获取编码器计数值后会清零计数器以便下次获取时为增量值
  * @retval 无
  */
  void BSP_Motor_GetEncoderNum(Motor_Device*Motor)
  {
    if (Motor->Motor_Timer.htim_encoder != NULL)
    {
        Motor->Motor_Base.Encoder_Num = (int16_t)DL_TimerG_getTimerCount(Motor->Motor_Timer.htim_encoder);
       // Motor->Motor_Base.Encoder_Num = Motor->Motor_Timer.htim_encoder->Instance->CNT; 
        DL_TimerG_setTimerCount(Motor->Motor_Timer.htim_encoder, 0);
        //Motor->Motor_Timer->htim_encoder->Instance->CNT = (uint32_t)0;
    }
    else
    {
        Motor->Motor_Base.Encoder_Num = (int16_t)SoftwareEncoder_Should;
        SoftwareEncoder_Should = 0;
    }
  }

  /**
  * @brief  电机BSP层启动定时中断
  * @param  Motor 电机句柄指针
  * @retval 无
  */
  void BSP_Motor_StartPID(Motor_Device*Motor)
  {
    DL_TimerA_enableInterrupt(Motor ->Motor_Timer.htim_pid, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(Motor_TIM_PID_INT_IRQN);
    NVIC_EnableIRQ(Motor_TIM_PID_INT_IRQN);
    DL_TimerA_startCounter(Motor ->Motor_Timer.htim_pid);
  }


  /**
  * @brief  电机BSP层关闭定时中断
  * @param  Motor 电机句柄指针
  * @retval 无
  */
  void BSP_Motor_StopPID(Motor_Device*Motor)
  {
    DL_TimerA_stopCounter(Motor ->Motor_Timer.htim_pid);
    DL_TimerA_disableInterrupt(Motor ->Motor_Timer.htim_pid, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_DisableIRQ(Motor_TIM_PID_INT_IRQN);
  }


void BSP_Motor_SetCompare(GPTIMER_Regs* htim_pwm, uint32_t Motor_Channel_x, uint32_t compare)
{
    Motor_Device *Motor = BSP_Motor_Find(htim_pwm, Motor_Channel_x);

    if (compare > PWM_MAX)
    {
        compare = PWM_MAX;
    }

    if (Motor != NULL)
    {
        Motor->Compare = compare;
        BSP_Motor_WriteCompare(Motor, compare);
    }
    else
    {
        DL_TimerA_setCaptureCompareValue(htim_pwm, compare, Motor_Channel_x);
    }
}


void GROUP1_IRQHandler(void)
{
    if (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) == DL_INTERRUPT_GROUP1_IIDX_GPIOA)
    {
        uint32_t status = DL_GPIO_getEnabledInterruptStatus(Right_GPIO_Port, Right_A_Pin | Right_B_Pin);

        if (status != 0U)
        {
            uint8_t cur = (uint8_t)(((DL_GPIO_readPins(Right_GPIO_Port, Right_A_Pin) != 0U) ? 0x02U : 0x00U) |
                                    ((DL_GPIO_readPins(Right_GPIO_Port, Right_B_Pin) != 0U) ? 0x01U : 0x00U));

            SoftwareEncoder_Should += s_QTable[SoftwareEncoder_Last & 0x03U][cur & 0x03U];
            SoftwareEncoder_Last = cur;

            DL_GPIO_clearInterruptStatus(Right_GPIO_Port, status);
        }
    }
}


  

