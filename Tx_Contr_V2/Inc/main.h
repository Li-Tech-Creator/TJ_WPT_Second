/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define Frequency_Start 650  //扫频起始值
#define Frequency_Stop  750  //扫频结束值
#define Frequency_Waiting_Delt 2 //在最大电流对应的频率的基础上加1 kHz，使逆变器呈感性
#define Duty_Scan_Coeff 12 //在检测到车辆的情况下的扫频时占空比被除系数
#define Frequency_Scan_Delt 2 //扫频的步长
#define Frequency_Scan_CurLimit 3200 //如果扫频电流超过这个值，则认为扫频电流过大，停放距离过远，拒绝充电
#define Max_Cur_Trans_AC 500 //最大谐振电流

//定义几种故障的优先级，数字越大，优先级越高
#define FaultLevel_EffiencyLow   2//传输效率过低
#define FaultLevel_UnderPower    3//电压加满也无法提供满功率
#define FaultLevel_FreqScanError 4//扫频电流过大，无法完成扫频，实际上就是停放距离过远
#define FaultLevel_CurOver_Short 5//短时过流
#define FaultLevel_InfraryStop   6//红外中断故障

#define FaultLevel_LimitError    10//指令下发限值和充电限值不一致
#define FaultLevel_VotRecError   11//接收端电池电压与设定值不匹配

#define FaultLevle_CurOver_Per1  17//发射端开BUCK、开逆变器后电流持续过大
#define FaultLevle_CurOver_Per2  18//发射端开BUCK、不开逆变器电流持续过大
#define FaultLevle_CurOver_Per3  19//发射端不开BUCK、不开逆变器电流持续过大

//#define FaultLevel_Stop_CurOver_Short 16//停止态发射端过流，由于处于停止态，问题消除后，可直接设为零，覆盖掉与发射端无关的故障
#define FaultLevel_Trans_Buck_Short 21//发射端BUCK烧穿，输出电压持续过大

#define FLASH_RW_StartAddress ((uint32_t) 0x0803F800)  //FLASH第127页起始地址
#define Power_Trans_Refer_Waiting 50
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
