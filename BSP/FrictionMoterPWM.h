#ifndef _FRICTIONMOTERPWM_H_
#define _FRICTIONMOTERPWM_H_

#define FRICTION_MOTER_L TIM2->CCR1  //
#define FRICTION_MOTER_R TIM2->CCR2

/*���������20msΪ���ڣ�0.5-2.5Ϊ�ߵ�ƽʱ��*/
#define SERVOS1 TIM5->CCR1 //PH10
#define SERVOS2 TIM5->CCR2 //PH11
#define SERVOS3 TIM5->CCR3 //PH12

void PWM_Init();

#endif

