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
#define Frequency_Start 650  //ɨƵ��ʼֵ
#define Frequency_Stop  750  //ɨƵ����ֵ
#define Frequency_Waiting_Delt 2 //����������Ӧ��Ƶ�ʵĻ����ϼ�1 kHz��ʹ������ʸ���
#define Duty_Scan_Coeff 12 //�ڼ�⵽����������µ�ɨƵʱռ�ձȱ���ϵ��
#define Frequency_Scan_Delt 2 //ɨƵ�Ĳ���
#define Frequency_Scan_CurLimit 3200 //���ɨƵ�����������ֵ������ΪɨƵ��������ͣ�ž����Զ���ܾ����
#define Max_Cur_Trans_AC 500 //���г�����

//���弸�ֹ��ϵ����ȼ�������Խ�����ȼ�Խ��
#define FaultLevel_EffiencyLow   2//����Ч�ʹ���
#define FaultLevel_UnderPower    3//��ѹ����Ҳ�޷��ṩ������
#define FaultLevel_FreqScanError 4//ɨƵ���������޷����ɨƵ��ʵ���Ͼ���ͣ�ž����Զ
#define FaultLevel_CurOver_Short 5//��ʱ����
#define FaultLevel_InfraryStop   6//�����жϹ���

#define FaultLevel_LimitError    10//ָ���·���ֵ�ͳ����ֵ��һ��
#define FaultLevel_VotRecError   11//���ն˵�ص�ѹ���趨ֵ��ƥ��

#define FaultLevle_CurOver_Per1  17//����˿�BUCK����������������������
#define FaultLevle_CurOver_Per2  18//����˿�BUCK�����������������������
#define FaultLevle_CurOver_Per3  19//����˲���BUCK�����������������������

//#define FaultLevel_Stop_CurOver_Short 16//ֹ̬ͣ����˹��������ڴ���ֹ̬ͣ�����������󣬿�ֱ����Ϊ�㣬���ǵ��뷢����޹صĹ���
#define FaultLevel_Trans_Buck_Short 21//�����BUCK�մ��������ѹ��������

#define FLASH_RW_StartAddress ((uint32_t) 0x0803F800)  //FLASH��127ҳ��ʼ��ַ
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
