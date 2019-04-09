#ifndef _FRICTIONMOTERPWM_H_
#define _FRICTIONMOTERPWM_H_

#define FRICTION_MOTER_L TIM2->CCR1  //
#define FRICTION_MOTER_R TIM2->CCR2

/*舵机控制在20ms为周期，0.5-2.5为高点平时间*/
#define SERVOS1 TIM5->CCR1 //PH10
#define SERVOS2 TIM5->CCR2 //PH11
#define SERVOS3 TIM5->CCR3 //PH12

void PWM_Init();

#endif

