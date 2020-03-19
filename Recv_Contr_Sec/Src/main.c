/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//�����ǳ����Ƶ���ز���
uint32_t ADC_ConvertedValue[3];//AD��������洢
uint16_t Vot_Battery[30],Cur_Battery[30],Temp_Battery[30];
uint16_t Vot_Battery_Average,Cur_Battery_Average,Temp_Battery_Average;
uint8_t ADC_Cnt;

//������״̬ת������ز���
uint8_t SystemState;
uint16_t Cur_Start,CurStop;
uint8_t Flag_WorkMode_Conversion;
uint16_t TimeCnt_WorkMode_Conversion;
/*******************************************************
SystemState���Ծ�������״̬�л����ݶ������£�
0��δ���״̬��1���������״̬��ֻ��������״̬���м�״̬�������⴦��
*******************************************************/

//�����Ǻ��ⷢ�����ز���
uint8_t Flag1,Flag2,BitCnt,Flag_Infray_Done;
uint8_t Flag_Zhen_No,Flag_Infray_WaitTime;
uint16_t Jiaoyan_Infray;

//ֻ��Flag_Infray_Done��־Ϊ1ʱ��������������ź�
uint32_t Infray_TransDat;//Ҫ���͵�����
uint16_t Period[32];//�洢ÿһλ���ݶ�Ӧ��Pulse��Ϣ
uint16_t TimeCnt_Infray;//���Զ�ʱ1���ӣ���ֹ̬ͣ��ÿ��1���ӣ�����һ�κ��⣬һ��2֡

//�����ǳ���������Ϣ
uint32_t Bike_ID;

//�����Ǵ��ڿ�����Ϣ
uint8_t Flag_StartByte;//�����Ƿ��⵽��ͷ
uint8_t Byte_Cnt;//�Խ����ֽڵ��������м���
uint8_t Data_ReceArray[6];//�洢���յ��Ĵ������ݣ�6���ֽ�Ϊһ�飬��ʱδ���������ȫ������
uint8_t Data_ReceSingle;//���������ֽ�
uint8_t Flag_Trans_Done,Flag_Rece_Done,TimeCnt485;
uint8_t SystInfo_Package[12];
uint16_t Jiaoyan485;

//������FLASH�����Ϣ
uint32_t ReadFlashData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//���ⷢ�亯�����β�ΪҪ���͵�32λ��ֵ
void Ir_Tx(uint32_t Dat);
//���������Ϣ�������
void Infray_InfoPackage_First_First(void);//�����һ��֡��һ��֡
void Infray_InfoPackage_First_Second(void);//�����һ��֡�ڶ���֡
void Infray_InfoPackage_Second(void);//����ڶ���֡

//FLASH��д����
uint32_t ReadFlash(uint32_t Address);//��ָ��λ���ϵ���ֵ��һ�ζ�ȡ4���ֽ�
void WriteFlash(uint32_t Address);//��ָ����ַ��д���ݣ����ֲ�������һ��д�������ֽ�

//������ʼ������
void Variable_Init(void);

//ȫ����Ϣ��������ͺ���
void Info_Trans(void);
void Info_Package(void);

//��ADƽ��ֵ����
void ADC_Average_Function(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FLASH_RW_StartAddress ((uint32_t) 0x08007C04)  //FLASH��31ҳ��ʼ��ַ���ƫ��4���ֽڣ���32ҳ��ÿҳ1k
#define Cur_Start 100
#define Cur_Stop  50
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t ss,sum;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
		
	//������ʼ������������ID�Ķ�ȡ���Լ�һЩ״̬�������趨
	Variable_Init();
	//ʹ��485���գ������жϵ��ֽڽ���ģʽ
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
	
	Flag_Infray_Done=1;//�������⹦��
	//�ϵ�1s������PMOS����·��������
	LED1_ON();LED2_ON();
	HAL_Delay(1000);
	MOS_Switch_ON();
	
	__HAL_IWDG_START(&hiwdg);//��ʱ1.68��	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//����·AD��ƽ��ֵ
		ADC_Average_Function();
		HAL_IWDG_Refresh(&hiwdg);
		//485ͨ�ż�ʱ
		if(Flag_StartByte)
		{
			if(TimeCnt485>50)//�Ѽ�⵽��ͷ����50ms��δ�������6���ֽڣ���λ���½���
			{
				Flag_StartByte=0;
				TimeCnt485=0;
				Byte_Cnt=0;
			}
		}
		//485ָ�����
		if(Flag_Rece_Done)//�������6���ֽڵ���Ч���ݣ����д���
		{
			Flag_Rece_Done=0;
			sum=0;//���ֵ������
			for(ss=1;ss<5;ss++)
			{
				sum+=Data_ReceArray[ss];
			}
			if(sum==Data_ReceArray[5])//�ж�У�飬У��ʧ�������κδ���
			{
				switch(Data_ReceArray[0])
				{
					case 0x0F:
					{
						Info_Trans();
						break;
					}
					case 0x1E:
					{
						//�޸ĳ����豸��ţ�д��FLASH��Ȼ�������е�·��Ϣ
						Bike_ID=0;
						Bike_ID+=Data_ReceArray[4];
						Bike_ID=Bike_ID<<8;
						Bike_ID+=Data_ReceArray[3];
						Bike_ID=Bike_ID<<8;
						Bike_ID+=Data_ReceArray[2];
						Bike_ID=Bike_ID<<8;
						Bike_ID+=Data_ReceArray[1];
						WriteFlash(FLASH_RW_StartAddress);
						Info_Trans();
						break;
					}
					default:
					{
						Info_Trans();
						break;
					}
				}
			}
		}
		//״̬����
		switch(SystemState)
		{
			case 0:
			{
				//���ⷢ���ж�--ֹ̬ͣ�����ͺ����һ��֡��������֡�Լ��ڶ���֡
				if(Flag_Infray_WaitTime)//1���Ӵ���һ��
				{
					//Flag_Zhen_No��ת���ڶ�ʱ���ж���ʵ��
					if(Flag_Infray_Done)
					{
						Flag_Infray_Done=0;
						HAL_Delay(100);
						if(Flag_Zhen_No==0)//���͵�һ��֡��һ��֡
						{
							Infray_InfoPackage_First_First();
							Ir_Tx(Infray_TransDat);
						}
						else if(Flag_Zhen_No==1)//���͵�һ��֡�ڶ���֡
						{
							Infray_InfoPackage_First_Second();
							Ir_Tx(Infray_TransDat);
						}
						else//���͵ڶ���֡
						{
							Flag_Infray_WaitTime=0;
							Infray_InfoPackage_Second();
							Ir_Tx(Infray_TransDat);
						}
					}
				}
				//״̬ת���ж�--��������Ϊ�жϱ�׼
				if(Cur_Battery_Average>Cur_Start)
				{
					if(!Flag_WorkMode_Conversion)//�״μ�⵽�����ѹ�����趨����ֵ
					{
						Flag_WorkMode_Conversion=1;//���ñ�־λ
					}
					else//֮ǰ�Ѽ�⵽�����ѹ���ߣ��˴����е�����ʱ����м�ʱ�ж�
					{
						if(Flag_WorkMode_Conversion>2)//ֵ-1������ʱ���������˴��ж��е�����ʱ���Ƿ��ѳ���1s
						{
							SystemState=1;
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
						}//�������ʱ��δ�����򲻶���
					}
				}
				//�е�����������Flag_WorkMode_Conversion��־λ������ʼ����
				//�������ͣ��������־λ�����¼�⣬ֻ�е����������ڣ��Ż�ʹ��Flag_WorkMode_Conversion��������
				if(Cur_Battery_Average<Cur_Stop)
				{
					Flag_WorkMode_Conversion=0;
					TimeCnt_WorkMode_Conversion=0;
				}
				break;
			}
			case 1:
			{
				//���ⷢ���ж�--���̬�����ͺ���ڶ���֡
				if(Flag_Infray_Done)
				{
					HAL_Delay(100);
					Infray_InfoPackage_Second();
					Ir_Tx(Infray_TransDat);
				}
				//״̬ת���ж�--��������Ϊ�жϱ�׼
				if(Cur_Battery_Average<Cur_Stop)
				{
					if(!Flag_WorkMode_Conversion)//�״μ�⵽����С���趨��ֵ
					{
						Flag_WorkMode_Conversion=1;//���ñ�־λ
					}
					else//֮ǰ�Ѽ�⵽������С���˴��Ե�����С��ʱ����м�ʱ�ж�
					{
						if(Flag_WorkMode_Conversion>31)//ֵ-1������ʱ���������˴��жϵ�����С�Ƿ��ѳ���30��
						{
							SystemState=0;
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
							
							//�趨����Ҫ��������
							Flag_Zhen_No=0;
							Flag_Infray_Done=1;
							Flag_Infray_WaitTime=1;
						}//���ʱ��δ��30s��������
					}
				}
				//������С��������Flag_WorkMode_Conversion��־λ������ʼ����
				//����ֵ�ָ����������־λ�����¼�⣬ֻ�е���ֵ������С���Ż�ʹ��Flag_WorkMode_Conversion��������
				if(Cur_Battery_Average>Cur_Start)
				{
					Flag_WorkMode_Conversion=0;
					TimeCnt_WorkMode_Conversion=0;
				}				
				break;
			}
			default:
			{
				SystemState=0;
				break;
			}
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

//SystemTick 1ms�ж�
void HAL_SYSTICK_Callback(void)
{
	//485ͨ����ؼ�ʱ����־
	if(Flag_StartByte)//��⵽485��ͷ��������ʱ����ʱδ�����꣬������ȫ�����յ������ݣ����¼����ͷ
	{
		if(TimeCnt485<255)
		{
			TimeCnt485++;
		}
	}
	
	//����һ��ADC����
	//ADУ׼��
	HAL_ADC_Start_DMA(&hadc,ADC_ConvertedValue,3);
	
	//���ⶨʱ����
	if(SystemState==0)
	{
		TimeCnt_Infray++;
		if(TimeCnt_Infray>500)
		{
			Flag_Infray_WaitTime=1;
			TimeCnt_Infray=0;
		}
	}
	
	//����״̬ת����ؼ�ʱ����־
	if(Flag_WorkMode_Conversion)
	{
		TimeCnt_WorkMode_Conversion++;
		if(TimeCnt_WorkMode_Conversion>1000)//��ʱ�ﵽ1s
		{
			TimeCnt_WorkMode_Conversion=0;
			Flag_WorkMode_Conversion++;
		}
	}
}

//������ʼ������
void Variable_Init()
{
	uint8_t num;
	//��ȡFLASH�д洢�ĳ�������
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Bike_ID=ReadFlashData;//��������
	
	if(Bike_ID==0xffffffff)
		Bike_ID=8;
	
	//��ʼ������״̬Ϊֹ̬ͣ�����ͳ��������Ϣ
	SystemState=0;
	Flag_WorkMode_Conversion=0;
	TimeCnt_WorkMode_Conversion=0;
	
	//������ر�����ʼ��
	Flag1=0;Flag2=0;
	BitCnt=0;Flag_Infray_Done=1;//�������ⷢ��
	Flag_Zhen_No=0;Jiaoyan_Infray=0;
	TimeCnt_Infray=0;
	
	//������ر�����ʼ��
	Flag_StartByte=0;Byte_Cnt=0;TimeCnt485=0;
	Flag_Trans_Done=0;Flag_Rece_Done=0;
	
	//ADC����ֵ��ʼ��
	Vot_Battery_Average=0;
	Cur_Battery_Average=0;
	Temp_Battery_Average=0;
	for(num=0;num<30;num++)
	{
		Vot_Battery[num]=0;
		Cur_Battery[num]=0;
		Temp_Battery[num]=0;		
	}
	
}

//FLASH�����ݺ���
uint32_t ReadFlash(uint32_t Address)
{
	uint32_t Result;
	Result=*(__IO uint32_t*)(Address);
	return Result;
}

//FLASHд���ݺ���-д�복�����
void WriteFlash(uint32_t Address)
{
	//FLASH�洢����������Stm32F103RCT6�����ڴ��СΪ256K����128ҳ��ÿҳ2K	
	FLASH_EraseInitTypeDef Flash;
	uint32_t PageError;
	
	HAL_FLASH_Unlock();//����д����
	//�����������
	Flash.TypeErase=FLASH_TYPEERASE_PAGES;
	Flash.PageAddress=FLASH_RW_StartAddress;
	Flash.NbPages=1;
	//����PageError
	PageError=0;//0xFFFFFFFF means that all the pages have been correctly erased
	//���ò�������
	HAL_FLASHEx_Erase(&Flash,&PageError);
	//��дFLASH
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address,Bike_ID);
	HAL_FLASH_Lock();
}

//���ڷ�������ж�
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Flag_Trans_Done=1;
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//���ڽ�������ж�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(!Flag_StartByte)//δ��⵽��ͷ����Ҫ�ȼ����ͷ
	{
		if(Data_ReceSingle==0x55)
		{
			Flag_StartByte=1;//�Ѽ�⵽��ͷ�������1
		}
	}
	else//�Ѽ�⵽��ͷ����Ҫ�����Ч����
	{
		Data_ReceArray[Byte_Cnt]=Data_ReceSingle;
		if(Byte_Cnt<5)//��Ч����6���ֽڣ��±���ൽ5
		{
			Byte_Cnt++;
		}
		else//�ѽ����������¼����ͷ�����485��ʱ���ֽ��±��0���趨��־λ
		{
			Flag_StartByte=0;
			TimeCnt485=0;
			Byte_Cnt=0;
			Flag_Rece_Done=1;
		}
	}
	
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//���ն�������Ϣ����
void Info_Trans(void)
{
	Trans_485_En();
	Info_Package();
	HAL_UART_Transmit_DMA(&huart1,SystInfo_Package,sizeof(SystInfo_Package));
}

//485��Ϣ���
void Info_Package(void)
{
	uint8_t i;
	
	//�洢��FLASH�е�ֵ�������ٴ�FLASH��ֱ�Ӷ�ȡ������֤ȷʵ����ȷ��д����FLASH
	SystInfo_Package[0]=0xaa;//��ʼ��ͷ
	
	//��ȡFLASH�д洢��Bike_ID
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Bike_ID=ReadFlashData;//��������
	
	SystInfo_Package[1]=Bike_ID;//���������8λ
	SystInfo_Package[2]=Bike_ID>>8;//��������9-16λ
	SystInfo_Package[3]=Bike_ID>>16;//��������17-24λ
	SystInfo_Package[4]=Bike_ID>>24;//��������25-32λ
	
	//д���ص�ѹ���������¶ȵ���Ϣ
	SystInfo_Package[5]=Vot_Battery_Average;//��ص�ѹ��8λ
	SystInfo_Package[6]=Cur_Battery_Average;//��ص�����8λ
	SystInfo_Package[7]=(Vot_Battery_Average>>4)&0xf0;//ȡ��ص�ѹ9-12λ�������4λ
	SystInfo_Package[7]+=(Cur_Battery_Average>>8)&0x0f;//ȡ��ص���9-12λ�������4λ
	//д�����¶���Ϣ
	SystInfo_Package[8]=Temp_Battery_Average;//����¶ȵ�8λ
	SystInfo_Package[9]=(Temp_Battery_Average>>8)&0x0f;//ȡ����¶�9-12λ�������4λ
	
	//����У��ֵ
	Jiaoyan485=0;
	for(i=1;i<10;i++)
	{
		Jiaoyan485+=SystInfo_Package[i];
	}
	SystInfo_Package[10]=Jiaoyan485;//У��ֵ��8λ
	SystInfo_Package[11]=Jiaoyan485>>8;//У��ֵ��8λ
	
}

//��������һ��֡��һ��֡
void Infray_InfoPackage_First_First(void)
{
	uint8_t Temp;//���԰��ֽڲ��Bike_ID
	
	Infray_TransDat=0x33;//������֡ͷ
	//��ȡ�������
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID;
	Infray_TransDat+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>8;
	Infray_TransDat+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>16;
	Infray_TransDat+=Temp;	
}
//��������һ��֡�ڶ���֡
void Infray_InfoPackage_First_Second(void)
{
	uint8_t Temp;
	
	Infray_TransDat=0x39;//������֡ͷ
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>24;
	Infray_TransDat+=Temp;
	//����У��ֵ
	Jiaoyan_Infray=0;
	
	Temp=Bike_ID;
	Jiaoyan_Infray+=Bike_ID;
	Temp=Bike_ID>>8;
	Jiaoyan_Infray+=Temp;
	Temp=Bike_ID>>16;
	Jiaoyan_Infray+=Temp;
	Temp=Bike_ID>>24;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<16;
	Infray_TransDat+=Jiaoyan_Infray;	
}
//�������ڶ���֡
void Infray_InfoPackage_Second(void)
{
	uint8_t Temp1,Temp2;
	
	Temp1=0;
	
	Infray_TransDat=Cur_Battery_Average&0xfff;//д���ص���ֵ��ȡ��12λ
	Infray_TransDat=Infray_TransDat<<12;
	Infray_TransDat+=(Vot_Battery_Average&0xfff);//д���ص�ѹֵ��ȡ��12λ
	
	//����У��ֵ
	Temp2=Infray_TransDat;
	Temp1+=Temp2;	
	Temp2=Infray_TransDat>>8;
	Temp1+=Temp2;	
	Temp2=Infray_TransDat>>16;
	Temp1+=Temp2;
	//ȡУ��ֵ��4λ
	Temp1=Temp1>>4;
	Temp1&=0x0f;
	//д��֡ͷ
	Temp1+=0xC0;
	
	Infray_TransDat+=Temp1*16777216;//2��24�η����൱������24λ
}

//���ⷢ�亯������
void Ir_Tx(uint32_t Dat)
{
	uint8_t i;
	for(i=0;i<32;i++)
	{
		if(Dat&0x80000000)
			Period[i]=1680;
		else
			Period[i]=560;
		Dat=Dat<<1;
	}
	
	__HAL_TIM_SET_COUNTER(&htim16,0);
	__HAL_TIM_CLEAR_FLAG(&htim16,TIM_FLAG_UPDATE);
	__HAL_TIM_SET_AUTORELOAD(&htim16,9000);	
	Flag1=0;Flag2=0;BitCnt=0;
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim16);
}

//���ⷢ����жϴ���
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//��ʱ�������¼��Ļص�����
{
	//���ƺ����TIM3δ���жϣ���˲���Ҫ����
	if(htim->Instance==TIM16)
	{
		if(Flag1==0)//�������������뷢��
		{
			if(Flag2==0)//���������PWM���䣬��Ҫ�ض�һ��ʱ��
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
				Flag2=1;				
				__HAL_TIM_SET_AUTORELOAD(&htim16,4500);	
			}
			else//����������ߵ͵�ƽ�������
			{
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
				Flag1=1;Flag2=0;				
				__HAL_TIM_SET_AUTORELOAD(&htim16,560);//�ز�560us
			}
		}
		else//��������뷢��
		{
			if(Flag2==0)
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
				Flag2=1;				
				__HAL_TIM_SET_AUTORELOAD(&htim16,Period[BitCnt]);
				if(BitCnt>31)
				{
					HAL_TIM_Base_Stop_IT(&htim16);
					HAL_TIM_Base_Stop_IT(&htim3);
					Flag_Infray_Done=1;//���ⷢ����ɱ�־��ֻ����ɷ��䣬���ܽ�����һ֡����
					
					//ֹ̬ͣ���ڵ�һ��֡�͵ڶ���֮֡���л�
					if(!SystemState)
					{
						Flag_Zhen_No++;
						if(Flag_Zhen_No>2)
						{
							Flag_Zhen_No=0;
							LED2_TOGGLE();
						}
					}
				}
				else
					BitCnt++;
			}
			else
			{
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
				Flag2=0;				
				__HAL_TIM_SET_AUTORELOAD(&htim16,560);
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_UPDATE);
}


//ADC�жϴ���
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
	Cur_Battery[ADC_Cnt]=ADC_ConvertedValue[0]&0xfff;
	Temp_Battery[ADC_Cnt]=ADC_ConvertedValue[1]&0xfff;
	Vot_Battery[ADC_Cnt]=ADC_ConvertedValue[2]&0xfff;
	
	ADC_Cnt++;
	if(ADC_Cnt>29)
		ADC_Cnt=0;
}

//ADC�����ƽ��
void ADC_Average_Function(void)
{
	uint8_t kk;
	uint32_t ADC_Sum_Vot,ADC_Sum_Cur,ADC_Sum_Temp;
	
	ADC_Sum_Vot=0;
	ADC_Sum_Cur=0;
	ADC_Sum_Temp=0;
	
	for(kk=0;kk<30;kk++)
	{
		ADC_Sum_Vot+=Vot_Battery[kk];
		ADC_Sum_Cur+=Cur_Battery[kk];
		ADC_Sum_Temp+=Temp_Battery[kk];
	}
	
	Vot_Battery_Average=ADC_Sum_Vot/30;
	Cur_Battery_Average=ADC_Sum_Cur/30;
	Temp_Battery_Average=ADC_Sum_Temp/30;
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
