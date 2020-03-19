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
//以下是充电控制的相关参数
uint32_t ADC_ConvertedValue[3];//AD采样结果存储
uint16_t Vot_Battery[30],Cur_Battery[30],Temp_Battery[30];
uint16_t Vot_Battery_Average,Cur_Battery_Average,Temp_Battery_Average;
uint8_t ADC_Cnt;

//以下是状态转换的相关参数
uint8_t SystemState;
uint16_t Cur_Start,CurStop;
uint8_t Flag_WorkMode_Conversion;
uint16_t TimeCnt_WorkMode_Conversion;
/*******************************************************
SystemState用以决定工作状态切换，暂定义如下：
0：未充电状态；1：正常充电状态。只有这两种状态，中间状态不做特殊处理
*******************************************************/

//以下是红外发射的相关参数
uint8_t Flag1,Flag2,BitCnt,Flag_Infray_Done;
uint8_t Flag_Zhen_No,Flag_Infray_WaitTime;
uint16_t Jiaoyan_Infray;

//只有Flag_Infray_Done标志为1时，才允许发射红外信号
uint32_t Infray_TransDat;//要发送的数据
uint16_t Period[32];//存储每一位数据对应的Pulse信息
uint16_t TimeCnt_Infray;//用以定时1秒钟，在停止态，每隔1秒钟，发送一次红外，一次2帧

//以下是车辆自身信息
uint32_t Bike_ID;

//以下是串口控制信息
uint8_t Flag_StartByte;//表征是否检测到字头
uint8_t Byte_Cnt;//对接收字节的数量进行计数
uint8_t Data_ReceArray[6];//存储接收到的串口数据，6个字节为一组，超时未接收完成则全部舍弃
uint8_t Data_ReceSingle;//单个接收字节
uint8_t Flag_Trans_Done,Flag_Rece_Done,TimeCnt485;
uint8_t SystInfo_Package[12];
uint16_t Jiaoyan485;

//以下是FLASH相关信息
uint32_t ReadFlashData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//红外发射函数，形参为要发送的32位数值
void Ir_Tx(uint32_t Dat);
//红外待发信息打包函数
void Infray_InfoPackage_First_First(void);//打包第一类帧第一分帧
void Infray_InfoPackage_First_Second(void);//打包第一类帧第二分帧
void Infray_InfoPackage_Second(void);//打包第二类帧

//FLASH读写函数
uint32_t ReadFlash(uint32_t Address);//读指定位置上的数值，一次读取4个字节
void WriteFlash(uint32_t Address);//向指定地址上写数据，半字操作，即一次写入两个字节

//变量初始化函数
void Variable_Init(void);

//全部信息打包及发送函数
void Info_Trans(void);
void Info_Package(void);

//求AD平均值函数
void ADC_Average_Function(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FLASH_RW_StartAddress ((uint32_t) 0x08007C04)  //FLASH第31页起始地址向后偏移4个字节，共32页，每页1k
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
		
	//变量初始化，包括车辆ID的读取，以及一些状态变量的设定
	Variable_Init();
	//使能485接收，开启中断单字节接收模式
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
	
	Flag_Infray_Done=1;//启动红外功能
	//上电1s后启动PMOS，旁路热敏电阻
	LED1_ON();LED2_ON();
	HAL_Delay(1000);
	MOS_Switch_ON();
	
	__HAL_IWDG_START(&hiwdg);//定时1.68秒	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//求三路AD的平均值
		ADC_Average_Function();
		HAL_IWDG_Refresh(&hiwdg);
		//485通信计时
		if(Flag_StartByte)
		{
			if(TimeCnt485>50)//已检测到字头，但50ms内未接收完成6个字节，则复位重新接收
			{
				Flag_StartByte=0;
				TimeCnt485=0;
				Byte_Cnt=0;
			}
		}
		//485指令解析
		if(Flag_Rece_Done)//接收完成6个字节的有效数据，进行处理
		{
			Flag_Rece_Done=0;
			sum=0;//求和值先清零
			for(ss=1;ss<5;ss++)
			{
				sum+=Data_ReceArray[ss];
			}
			if(sum==Data_ReceArray[5])//判断校验，校验失败则不做任何处理
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
						//修改车载设备编号，写入FLASH，然后反馈所有电路信息
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
		//状态处理
		switch(SystemState)
		{
			case 0:
			{
				//红外发射判定--停止态，发送红外第一类帧的两个分帧以及第二类帧
				if(Flag_Infray_WaitTime)//1秒钟触发一次
				{
					//Flag_Zhen_No的转换在定时器中断中实现
					if(Flag_Infray_Done)
					{
						Flag_Infray_Done=0;
						HAL_Delay(100);
						if(Flag_Zhen_No==0)//发送第一类帧第一分帧
						{
							Infray_InfoPackage_First_First();
							Ir_Tx(Infray_TransDat);
						}
						else if(Flag_Zhen_No==1)//发送第一类帧第二分帧
						{
							Infray_InfoPackage_First_Second();
							Ir_Tx(Infray_TransDat);
						}
						else//发送第二类帧
						{
							Flag_Infray_WaitTime=0;
							Infray_InfoPackage_Second();
							Ir_Tx(Infray_TransDat);
						}
					}
				}
				//状态转换判定--充电电流作为判断标准
				if(Cur_Battery_Average>Cur_Start)
				{
					if(!Flag_WorkMode_Conversion)//首次检测到输入电压超出设定的阈值
					{
						Flag_WorkMode_Conversion=1;//设置标志位
					}
					else//之前已检测到输入电压过高，此处对有电流的时间进行计时判断
					{
						if(Flag_WorkMode_Conversion>2)//值-1即已延时的秒数，此处判断有电流的时间是否已持续1s
						{
							SystemState=1;
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
						}//如果持续时间未到，则不动作
					}
				}
				//有电流，则设置Flag_WorkMode_Conversion标志位，并开始计数
				//电流过低，则清除标志位，重新检测，只有电流持续存在，才会使得Flag_WorkMode_Conversion持续增加
				if(Cur_Battery_Average<Cur_Stop)
				{
					Flag_WorkMode_Conversion=0;
					TimeCnt_WorkMode_Conversion=0;
				}
				break;
			}
			case 1:
			{
				//红外发射判定--充电态，发送红外第二类帧
				if(Flag_Infray_Done)
				{
					HAL_Delay(100);
					Infray_InfoPackage_Second();
					Ir_Tx(Infray_TransDat);
				}
				//状态转换判定--充电电流作为判断标准
				if(Cur_Battery_Average<Cur_Stop)
				{
					if(!Flag_WorkMode_Conversion)//首次检测到电流小于设定阈值
					{
						Flag_WorkMode_Conversion=1;//设置标志位
					}
					else//之前已检测到电流过小，此处对电流过小的时间进行计时判断
					{
						if(Flag_WorkMode_Conversion>31)//值-1即已延时的秒数，此处判断电流过小是否已持续30秒
						{
							SystemState=0;
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
							
							//设定红外要继续发送
							Flag_Zhen_No=0;
							Flag_Infray_Done=1;
							Flag_Infray_WaitTime=1;
						}//如果时间未到30s，不动作
					}
				}
				//电流过小，则设置Flag_WorkMode_Conversion标志位，并开始计数
				//电流值恢复，则清除标志位，重新检测，只有电流值持续过小，才会使得Flag_WorkMode_Conversion持续增加
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

//SystemTick 1ms中断
void HAL_SYSTICK_Callback(void)
{
	//485通信相关计时及标志
	if(Flag_StartByte)//检测到485字头，开启计时，超时未接收完，则舍弃全部接收到的数据，重新检测字头
	{
		if(TimeCnt485<255)
		{
			TimeCnt485++;
		}
	}
	
	//开启一次ADC采样
	//AD校准？
	HAL_ADC_Start_DMA(&hadc,ADC_ConvertedValue,3);
	
	//红外定时计数
	if(SystemState==0)
	{
		TimeCnt_Infray++;
		if(TimeCnt_Infray>500)
		{
			Flag_Infray_WaitTime=1;
			TimeCnt_Infray=0;
		}
	}
	
	//工作状态转换相关计时及标志
	if(Flag_WorkMode_Conversion)
	{
		TimeCnt_WorkMode_Conversion++;
		if(TimeCnt_WorkMode_Conversion>1000)//计时达到1s
		{
			TimeCnt_WorkMode_Conversion=0;
			Flag_WorkMode_Conversion++;
		}
	}
}

//变量初始化工作
void Variable_Init()
{
	uint8_t num;
	//读取FLASH中存储的车辆编码
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Bike_ID=ReadFlashData;//车辆编码
	
	if(Bike_ID==0xffffffff)
		Bike_ID=8;
	
	//初始化工作状态为停止态，发送车辆编号信息
	SystemState=0;
	Flag_WorkMode_Conversion=0;
	TimeCnt_WorkMode_Conversion=0;
	
	//红外相关变量初始化
	Flag1=0;Flag2=0;
	BitCnt=0;Flag_Infray_Done=1;//开启红外发射
	Flag_Zhen_No=0;Jiaoyan_Infray=0;
	TimeCnt_Infray=0;
	
	//串口相关变量初始化
	Flag_StartByte=0;Byte_Cnt=0;TimeCnt485=0;
	Flag_Trans_Done=0;Flag_Rece_Done=0;
	
	//ADC采样值初始化
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

//FLASH读数据函数
uint32_t ReadFlash(uint32_t Address)
{
	uint32_t Result;
	Result=*(__IO uint32_t*)(Address);
	return Result;
}

//FLASH写数据函数-写入车辆编号
void WriteFlash(uint32_t Address)
{
	//FLASH存储操作，对于Stm32F103RCT6，其内存大小为256K，共128页，每页2K	
	FLASH_EraseInitTypeDef Flash;
	uint32_t PageError;
	
	HAL_FLASH_Unlock();//解锁写保护
	//定义擦除对象
	Flash.TypeErase=FLASH_TYPEERASE_PAGES;
	Flash.PageAddress=FLASH_RW_StartAddress;
	Flash.NbPages=1;
	//设置PageError
	PageError=0;//0xFFFFFFFF means that all the pages have been correctly erased
	//调用擦除函数
	HAL_FLASHEx_Erase(&Flash,&PageError);
	//烧写FLASH
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address,Bike_ID);
	HAL_FLASH_Lock();
}

//串口发送完成中断
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Flag_Trans_Done=1;
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//串口接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(!Flag_StartByte)//未检测到字头，需要先检测字头
	{
		if(Data_ReceSingle==0x55)
		{
			Flag_StartByte=1;//已检测到字头，标记置1
		}
	}
	else//已检测到字头，需要填充有效数据
	{
		Data_ReceArray[Byte_Cnt]=Data_ReceSingle;
		if(Byte_Cnt<5)//有效数据6个字节，下标最多到5
		{
			Byte_Cnt++;
		}
		else//已接收满，重新检测字头，清除485计时，字节下标归0，设定标志位
		{
			Flag_StartByte=0;
			TimeCnt485=0;
			Byte_Cnt=0;
			Flag_Rece_Done=1;
		}
	}
	
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//接收端所有信息发送
void Info_Trans(void)
{
	Trans_485_En();
	Info_Package();
	HAL_UART_Transmit_DMA(&huart1,SystInfo_Package,sizeof(SystInfo_Package));
}

//485信息打包
void Info_Package(void)
{
	uint8_t i;
	
	//存储到FLASH中的值，必须再从FLASH中直接读取，以验证确实已正确的写入了FLASH
	SystInfo_Package[0]=0xaa;//开始字头
	
	//读取FLASH中存储的Bike_ID
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Bike_ID=ReadFlashData;//车辆编码
	
	SystInfo_Package[1]=Bike_ID;//车辆编码低8位
	SystInfo_Package[2]=Bike_ID>>8;//车辆编码9-16位
	SystInfo_Package[3]=Bike_ID>>16;//车辆编码17-24位
	SystInfo_Package[4]=Bike_ID>>24;//车辆编码25-32位
	
	//写入电池电压、电流、温度等信息
	SystInfo_Package[5]=Vot_Battery_Average;//电池电压低8位
	SystInfo_Package[6]=Cur_Battery_Average;//电池电流低8位
	SystInfo_Package[7]=(Vot_Battery_Average>>4)&0xf0;//取电池电压9-12位，放入高4位
	SystInfo_Package[7]+=(Cur_Battery_Average>>8)&0x0f;//取电池电流9-12位，放入低4位
	//写入电池温度信息
	SystInfo_Package[8]=Temp_Battery_Average;//电池温度低8位
	SystInfo_Package[9]=(Temp_Battery_Average>>8)&0x0f;//取电池温度9-12位，放入低4位
	
	//计算校验值
	Jiaoyan485=0;
	for(i=1;i<10;i++)
	{
		Jiaoyan485+=SystInfo_Package[i];
	}
	SystInfo_Package[10]=Jiaoyan485;//校验值低8位
	SystInfo_Package[11]=Jiaoyan485>>8;//校验值高8位
	
}

//打包红外第一类帧第一分帧
void Infray_InfoPackage_First_First(void)
{
	uint8_t Temp;//用以按字节拆分Bike_ID
	
	Infray_TransDat=0x33;//先输入帧头
	//提取编号数据
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
//打包红外第一类帧第二分帧
void Infray_InfoPackage_First_Second(void)
{
	uint8_t Temp;
	
	Infray_TransDat=0x39;//先输入帧头
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>24;
	Infray_TransDat+=Temp;
	//计算校验值
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
//打包红外第二类帧
void Infray_InfoPackage_Second(void)
{
	uint8_t Temp1,Temp2;
	
	Temp1=0;
	
	Infray_TransDat=Cur_Battery_Average&0xfff;//写入电池电流值，取低12位
	Infray_TransDat=Infray_TransDat<<12;
	Infray_TransDat+=(Vot_Battery_Average&0xfff);//写入电池电压值，取低12位
	
	//计算校验值
	Temp2=Infray_TransDat;
	Temp1+=Temp2;	
	Temp2=Infray_TransDat>>8;
	Temp1+=Temp2;	
	Temp2=Infray_TransDat>>16;
	Temp1+=Temp2;
	//取校验值高4位
	Temp1=Temp1>>4;
	Temp1&=0x0f;
	//写入帧头
	Temp1+=0xC0;
	
	Infray_TransDat+=Temp1*16777216;//2的24次方，相当于左移24位
}

//红外发射函数定义
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

//红外发射的中断处理
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器更新事件的回调函数
{
	//控制红外的TIM3未开中断，因此不需要处理
	if(htim->Instance==TIM16)
	{
		if(Flag1==0)//表明处于引导码发射
		{
			if(Flag2==0)//表明刚完成PWM发射，需要关断一段时间
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
				Flag2=1;				
				__HAL_TIM_SET_AUTORELOAD(&htim16,4500);	
			}
			else//表明引导码高低电平均已完成
			{
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
				Flag1=1;Flag2=0;				
				__HAL_TIM_SET_AUTORELOAD(&htim16,560);//载波560us
			}
		}
		else//完成引导码发射
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
					Flag_Infray_Done=1;//红外发送完成标志，只有完成发射，才能进行下一帧发射
					
					//停止态，在第一分帧和第二分帧之间切换
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


//ADC中断处理
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
	Cur_Battery[ADC_Cnt]=ADC_ConvertedValue[0]&0xfff;
	Temp_Battery[ADC_Cnt]=ADC_ConvertedValue[1]&0xfff;
	Vot_Battery[ADC_Cnt]=ADC_ConvertedValue[2]&0xfff;
	
	ADC_Cnt++;
	if(ADC_Cnt>29)
		ADC_Cnt=0;
}

//ADC结果求平均
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
