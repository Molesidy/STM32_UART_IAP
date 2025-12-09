/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef  void (*IAP_FUN)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//STM32F407VET6 的 FLASH 操作完全以扇区为最小擦除 / 管理单位（无 “页” 的概念）
//其扇区大小并非统一值，而是按 “差异化划分” 设计（前 4 个小扇区、1 个中等扇区、7 个大扇区）
//STM32F407VET6 扇区大小 & 地址范围（精准版）
//扇区编号			起始地址	结束地址	大小	十六进制大小	备注
//FLASH_SECTOR_0	0x08000000	0x08003FFF	16 KB	0x4000	适合存放 Bootloader（小容量、频繁更新）
//FLASH_SECTOR_1	0x08004000	0x08007FFF	16 KB	0x4000	
//FLASH_SECTOR_2	0x08008000	0x0800BFFF	16 KB	0x4000	
//FLASH_SECTOR_3	0x0800C000	0x0800FFFF	16 KB	0x4000	
//FLASH_SECTOR_4	0x08010000	0x0801FFFF	64 KB	0x10000	中等扇区
//FLASH_SECTOR_5	0x08020000	0x0803FFFF	128 KB	0x20000	适合存放 APP 固件（大容量）
//FLASH_SECTOR_6	0x08040000	0x0805FFFF	128 KB	0x20000	
//FLASH_SECTOR_7	0x08060000	0x0807FFFF	128 KB	0x20000	
//FLASH_SECTOR_8	0x08080000	0x0809FFFF	128 KB	0x20000	
//FLASH_SECTOR_9	0x080A0000	0x080BFFFF	128 KB	0x20000	
//FLASH_SECTOR_10	0x080C0000	0x080DFFFF	128 KB	0x20000	
//FLASH_SECTOR_11	0x080E0000	0x080FFFFF	128 KB	0x20000	


//把这个bootloader可以看为也是一个App，不过作用有点特殊而已，主要作用是接受固件，写固件到flash，然后将MSP(SP)主栈指针指向新的App
//立创天空星STM32F407VET6: 主频:168MHz flash:512KB SRAM:196KB
#define SRAM_App_Start_Addr			((uint32_t)0x20010000)							//初始运行用户自定义的bootloader 指定SRAM起始地址 
#define SRAM_App_Size				(0xC000)										//如果App较小 App在SRAM运行时可以限定一下其占SRAM的范围
#define SRAM_App_End_Addr			((uint32_t)(SRAM_App_Start_Addr+SRAM_App_Size))	//初始运行用户自定义的bootloader 指定SRAM结束地址 这是运行bootloader需要的SRAM限定空间
#define Flash_Start_Addr			((uint32_t)0x08000000)							//指定Flash起始地址:0x08000000
#define Bootloader_Size				(0xFFFF)											//大小，64KB，1个sector:16 KB,Bootloader直接使用前4个sector
#define Application_Start_Addr		(Flash_Start_Addr + Bootloader_Size+1 + 0xFFFF+1)	//App的起始地址:0x08020000 把app放在bootloader后面的0x08020000  App直接使用2个sector
#define	Update_CMD	                 "update"										//擦除并升级指令 开始升级
#define Application_Size			(1024 * 128 * 2)								//App大小256KB
#define Application_End_Addr		((uint32_t)(Application_Start_Addr+Application_Size))	//App的结束地址:0x0805FFFF

#define AppFirmwareImageBuf_Size	(256) //App固件接收缓冲区，一包是256B

#define IAP_UART_Port huart1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t AppFirmwareImageBuf[AppFirmwareImageBuf_Size] = {0}; //一片/一包固件  建议不要在栈上开辟数组
IAP_FUN Jump_To_App; //函数指针
uint8_t bootloader_flag = 0; //是否升级标志
uint8_t time_out_flag = 0; //超时标记

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//重定向
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&IAP_UART_Port, (const uint8_t *)&ch, 1, 1000);
	return ch;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("bootloader is running\r\n");
	printf("Let's do it\r\n");
	
	//判断是否更新
	for(uint8_t i = 0; i < 10; i++)
	{
		printf("在十秒内输入: update 即更新App 计时: %d 秒\r\n", (i+1));
		//上电后阻塞10秒等待接收升级指令，连续10秒未收到则跳转App
		//如果10秒内收到，则接收App数据并擦写写入
		HAL_UART_Receive(&IAP_UART_Port, AppFirmwareImageBuf, AppFirmwareImageBuf_Size, 1000); //阻塞等待升级命令
		
		if(strstr((const char *)AppFirmwareImageBuf, Update_CMD) != NULL) //在发来的数据里查找update字符串
		{
			FLASH_EraseInitTypeDef EraseInitStruct; //需要擦除的地方
			
			uint32_t SectorError; //Sector操作返回值
			
			HAL_FLASH_Unlock(); //解锁
			
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;	//操作sector 指定擦除类型（扇区擦除 / 全片擦除）
			EraseInitStruct.Banks = FLASH_BANK_1; //全片擦除时指定要擦除的 FLASH 存储体（Bank） F407 只有FLASH_BANK_1（1MB FLASH 仅占 Bank1）；
			EraseInitStruct.Sector = FLASH_SECTOR_5; //扇区擦除时指定起始扇区（从哪个扇区开始擦） F407 取值：FLASH_SECTOR_0 ~ FLASH_SECTOR_11（共 12 个扇区）
			EraseInitStruct.NbSectors = 2; //扇区擦除时指定要擦除的扇区数量（连续擦除多少个） 取值范围：1 ≤ NbSectors ≤ (总扇区数 - 起始扇区编号)（F407 总扇区 12，比如起始扇区 5 → 最大可设 7）
			EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; //指定芯片供电电压范围（决定 FLASH 擦除的并行度 / 速度）

			
			if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) //擦除App空间
			{
				HAL_FLASH_Lock(); //上锁
				printf("------------Erase fail at:0x%x\r\n",SectorError);
				return -1;
			}
			
			HAL_FLASH_Lock(); //上锁
			bootloader_flag = 1; //升级启动
			printf("------------Erase OK\r\n");
			break; //跳出等待
		}
	}
	
	//是否进入升级
	if(bootloader_flag == 1)
	{
		HAL_StatusTypeDef ret = HAL_ERROR; //存储判断返回值
		uint32_t CurrentAppAddr = 0; //当前写的地址
		uint32_t Data_Word = 0; //一个字的缓存值
		uint8_t packet_count = 0; //记录接收多少包
		
		printf("------------ready to receive bin, please send in 30s\r\n");
		
		CurrentAppAddr = Application_Start_Addr; //以App起始地址开始
		
		ret = HAL_UART_Receive(&IAP_UART_Port, AppFirmwareImageBuf, AppFirmwareImageBuf_Size, 30*1000); //用串口工具发bin文件 30S内必须要接收到
		
		if(ret == HAL_TIMEOUT)
		{
			printf("------------time out, end wait to receive bin\r\n");
			return -1;
		} else if (ret == HAL_OK) {
			//收到则循环接收，每秒阻塞接收256字节，并写入Flash
			while(1)
			{
				HAL_FLASH_Unlock();
				
				for(uint8_t i = 0; i < AppFirmwareImageBuf_Size/4; i++)
				{
					Data_Word = *((uint32_t *)(&AppFirmwareImageBuf[i << 2])); //i << 2 等价于 i * 4（左移 n 位 = 乘以 2^n，2^2=4）即步进4个字节
					if(CurrentAppAddr < Application_End_Addr)
					{
						if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CurrentAppAddr, Data_Word) == HAL_OK) //写入一个字的数据
						{
							CurrentAppAddr += 4;
						}else{
							HAL_FLASH_Lock();
							printf("------------write flash fail at: 0x%x\r\n", CurrentAppAddr);
							return -1;
						}
					}
				} //接收完一包
				
				HAL_FLASH_Lock();
				printf("------------write 256 btye OK: 0x%x\t%d\r\n", CurrentAppAddr, ++packet_count);
				
				ret = HAL_UART_Receive(&IAP_UART_Port, AppFirmwareImageBuf, AppFirmwareImageBuf_Size, 2*1000); //2秒内阻塞式等待
				if(ret == HAL_TIMEOUT) //没有数据发来就默认发完bin文件就跳转到启动App
				{
					time_out_flag++;
					if(time_out_flag == 2) //第二次超时就退出
					{
						printf("------------End write OK\r\n");
						goto START_APP; //写入完App后尝试跳转
					}
				}
			}
		}
	}else{ //bootloader_flag =0
START_APP:		
		printf("------------start user app\r\n");	
		HAL_Delay(10);
		
		/*
        校验App有效性：Flash中App起始地址存储的是App的中断向量表（第一项就是栈顶指针MSP(SP)  第二项“复位地址”）
        向量表第1个4字节：App的栈顶地址（MSP），需落在芯片合法SRAM区间内
		*/
		printf("------------Application_Start_Addr:%0x\r\n", *(unsigned int *)Application_Start_Addr); //没有App的情况下去访问的值为0xFFFFFFFF(擦除状态)
		
		//由KEIL做出来的bin文件，在编译出的时候在魔法棒里会调整整个App程序的链接Flash的入口地址以及SRAM起始地址，比如这里面的Application_Start_Addr:0x08020000
		//在这里面的宏定义Application_Start_Addr是一个宏定义的32位数据，是App空间的起始（写入Flash）地址
		//去访问Application_Start_Addr（0x08020000）这个地址上的值，以unsigned int*访问，即访问到32位的数据，这个数据就是以后App在SRAM运行起来的栈顶指针(地址)
		//然后在下面去判断这个App在SRAM运行起来的栈顶指针(地址) 是否是符合：bootloader和做固件的时候是一同规定的App栈顶指针在限定的SRAM里面所处于的范围
		//由于后续App的固件大小不确定，然后编译出来的bin固件文件里MSP里装的栈顶指针(地址)，这个栈顶实在编译和链接Flash/SRAM过程当中决定的，这个栈顶只能在魔法棒的SRAM起始地址和其大小限定的那个范围里
		//如果属于这个限定的范围内就成功完成校验，然后跳转
		if( ((*(unsigned int *)Application_Start_Addr) >= SRAM_App_Start_Addr) &&
			((*(unsigned int *)Application_Start_Addr) <= SRAM_App_End_Addr)
			)
		{
			//关中断 防止被打断 在App里记得需要开启中断
			__disable_irq();
			//Application_Start_Addr+4 放的是中断向量表的第二项“复位地址” (第一项就是栈顶指针MSP(SP))
			__set_MSP(*(unsigned int *)Application_Start_Addr); //手动指定MSP到新App起始地址
			Jump_To_App = (IAP_FUN)*(unsigned int *)(Application_Start_Addr + 4); //将复位函数指针移到(新App起始地址+4)   Jump_To_App就是函数指针
			Jump_To_App(); //调用复用函数
		}else{
			printf("the value of MSP error\r\n");
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  printf("------------defualt bootloader is running and no app\r\n");
	  HAL_Delay(150);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
