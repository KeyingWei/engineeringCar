/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "BasicPeripherals.h"
#include "freertostask.h"
#include "RemotDbus.h"
#include "string.h"
#include "Auto_attackTask.h"
 

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
// 
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


void  EXTI2_IRQHandler()
{	
  if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
  {
    LED_RED = !LED_RED;

  }
     EXTI_ClearITPendingBit(EXTI_Line2);
}
/*****************************************
函数名：EXTI1_IRQHandler
入口参数：无
出口参数：无
功能：外部中断1中断函数，对陀螺仪数据采样，得
出Y轴P轴的角速度，并积分出Y轴角度
******************************************/
void EXTI9_5_IRQHandler(void)
{
	 BaseType_t  pxHigherPriorityTaskWoken;
  	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
		{
		  xSemaphoreGiveFromISR(ImuDataSem,&pxHigherPriorityTaskWoken);
		}
	EXTI_ClearITPendingBit(EXTI_Line8);//清除LINE1上的中断标志位 
}


void DMA2_Stream5_IRQHandler(void)	
{
   BaseType_t  pxHigherPriorityTaskWoken;
  if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET) 
	{
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		xSemaphoreGiveFromISR(DbusParseSem,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
	
	DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5); //clear DMA flag
}



void CAN1_RX0_IRQHandler(void)
{
	 BaseType_t  pxHigherPriorityTaskWoken;
	 CanRxMsg rx_message; 
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{   
       CAN_Receive(CAN1, CAN_FIFO0, &rx_message);		
	   xQueueOverwriteFromISR(Can1ReceiveQueue,&rx_message,&pxHigherPriorityTaskWoken); 
	    		
	}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);	
	    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
}



struct IMU_EXT Imu_ext;
void CAN2_RX0_IRQHandler(void)
{  
	//BaseType_t  pxHigherPriorityTaskWoken;
	CanRxMsg rx_message; 
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
		
		CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
		
		switch(rx_message.StdId)
		{
				case 0x501:
					
					memcpy(&Imu_ext,rx_message.Data,5);
				    Imu_ext.angle  -= Imu_ext.Gyo_Z * 0.001;
				    Imu_ext.onlinecnt = 0;
				default:
					
					break;		
		}
		//xQueueOverwriteFromISR(Can2ReceiveQueue,&rx_message,&pxHigherPriorityTaskWoken); 	    
	}
	
//	    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	

}


void CAN2_TX_IRQHandler()
{
		if(CAN_GetITStatus(CAN2,CAN_IT_TME) !=RESET)
		{
			 CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		}  
}

void CAN1_TX_IRQHandler()
{
  if(CAN_GetITStatus(CAN1,CAN_IT_TME) !=RESET)
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}

void TIM2_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
		{
			  TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
              TIM_ClearFlag(TIM2, TIM_FLAG_Update);   
		}
} 
void TIM5_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
		{
			  TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
              TIM_ClearFlag(TIM5, TIM_FLAG_Update);   
		}
} 
void DMA1_Stream1_IRQHandler()
{
  if(DMA_GetFlagStatus(DMA1_Stream1,DMA_IT_TCIF1)==SET) 
	{
		DMA_ClearFlag(DMA1_Stream1,DMA_IT_TCIF1); 
		DMA_ClearITPendingBit(DMA1_Stream1,DMA_IT_TCIF1);		

        receiveMainFocusData();		
	}
}
char i=0,j=0;
   Key_ide key_ide[8]={{0},{0},{0},{0},{0},{0},{0},{0}} ;
    p_Key_ide P_Key_ide1[8] ={&key_ide[0],&key_ide[1],&key_ide[2],&key_ide[3],&key_ide[4],&key_ide[5],&key_ide[6],&key_ide[7]} ; 
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
	 
  
		 Key_Scan(KEY_PRESSED_OFFSET_F,P_Key_ide1[0]);
	     Key_Scan(KEY_PRESSED_OFFSET_G,P_Key_ide1[1]);
		 Key_Scan(KEY_PRESSED_OFFSET_R,P_Key_ide1[2]);
		 Key_Scan(KEY_PRESSED_OFFSET_Z,P_Key_ide1[3]);
		 Key_Scan(KEY_PRESSED_OFFSET_X,P_Key_ide1[4]);
		 Key_Scan(KEY_PRESSED_OFFSET_B,P_Key_ide1[5]);
		 Key_Scan(KEY_PRESSED_OFFSET_C,P_Key_ide1[6]);
		 Key_Scan(KEY_PRESSED_OFFSET_V,P_Key_ide1[7]);
		
    		  if(P_Key_ide1[5]->Key_Event == KEY_DOUBLE_CLICK ) //
				{
                         
					i++;
                    PUCH1=1;	
                    j++;					
				}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清楚中断标志位
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
