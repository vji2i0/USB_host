/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */
#include "SendMessage.h"

#define IDLE 0
#define DOWN 1
#define UP 2
#define OK 3
#define BACK 4
#define MAX_STRING_LENGTH 10
#define MAX_PATH_LENGTH 100
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern ApplicationTypeDef Appli_state;
FATFS USBDISKFatFs;
FIL MyFile;
extern USBH_HandleTypeDef hUsbHostFS;
volatile int fl=1;

volatile int ReadContentFlag=1;

volatile int status = IDLE;

uint8_t rxBuffer = '\0';
uint8_t rxString[MAX_STRING_LENGTH];
int rxindex = 0;
int commentflag=1;
struct node * pNode;
char path[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
struct node
{
  char name[16]; // the name
  FILINFO fno; // file info. FILINFO : see more in /Middlewares/.../ff.h
  struct node * nextNode; // pointer to the next node
  struct node * previousNode; // pointer to the previous node
};

void PinD12On(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}
void PinD12Off(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}
void PinD13On(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
}
void PinD13Off(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
}
void PinD14On(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}
void PinD14Off(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
}
void PinD15On(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
}
void PinD15Off(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}
void ErrorCase(void)
{
  //ClearNodes();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  Appli_state = APPLICATION_IDLE;
  sendStaicMessage((char *)"Flash-drive is disconnected\n");
}
void FileReadWrite(void)
{
  FRESULT res;
  char rtext[100];
  UINT bytesread;
  if (f_mount(&USBDISKFatFs, (TCHAR const*)USBHPath, 0)!=FR_OK)
  {
    sendStaicMessage((char *)"USB-drive mount error\n");
    //Error_Handler();
    ErrorCase();
  }else{
    if(f_open(&MyFile,"copy.txt",FA_READ)!=FR_OK)//this was changed
    {
      sendStaicMessage((char *)"File open error\n");
      //Error_Handler();
      ErrorCase();
    }else{
      //sendStaicMessage("File open is OK.\n");
      res = f_read(&MyFile, rtext, sizeof(rtext), &bytesread);
      if((bytesread==0)||(res!=FR_OK))
      {
        sendStaicMessage((char *)"File read error\n");
        ErrorCase();
      }else{
        if(fl)
        {
          //sendStaicMessage((uint8_t*)USBHPath);
          fl=0;
          rtext[bytesread]=0;
          sendStaicMessage(rtext);
          f_close(&MyFile);
        }
      }
    }
  }
}
void ReadFileByString(void)
{
//sendStaicMessage("Function is called\n");
  char rtext[100];
/*
  res = f_read(&MyFile, rtext, sizeof(rtext),(void*)&bytesread);
  if((bytesread==0)||(res!=FR_OK))
  {
    sendStaicMessage("File read error\n");
    ErrorCase();
  }else{
    //sendStaicMessage((uint8_t*)USBHPath);
    rtext[bytesread]=0;
    sendStaicMessage(rtext);
  }
*/
  while (f_eof(&MyFile) == 0)
  {
    f_gets(rtext, sizeof(rtext), &MyFile);
    sendStaicMessage(rtext);
  }
}

void RootContentShort(void)
{
  FRESULT res;
  DIR dir;
//  UINT i;
  static FILINFO fno;
  char fileName[100];
  if (f_mount(&USBDISKFatFs, (TCHAR const*)USBHPath, 0)!=FR_OK)
  {
    sendStaicMessage((char *)"USB-drive mount error\n");
    //Error_Handler();
    //fl=1;
    ErrorCase();
  }else{
    if(f_open(&MyFile,"1.txt",FA_READ)!=FR_OK)
    {
      sendStaicMessage((char *)"File open error\n");
      ErrorCase();
    }else{
      sendStaicMessage((char *)"File open OK\n");
      ReadFileByString(); //MyFile is global!
      sendStaicMessage((char *)"THE END 2\n");
      f_close(&MyFile);
    }
  }
}

void RootContent(void)
{
  FRESULT res;
  DIR dir;
//  UINT i;
  static FILINFO fno;
  char fileName[100];
  if (f_mount(&USBDISKFatFs, (TCHAR const*)USBHPath, 0)!=FR_OK)
  {
    sendStaicMessage((char *)"USB-drive mount error\n");
    //Error_Handler();
    fl=1;
    ErrorCase();
  }else{
    res = f_opendir(&dir, USBHPath);
    if(fl)
    {
      if (res!=FR_OK)
      {
        //sendStaicMessage("Directory open error\n");
        fl=1;
        ErrorCase();
      }else{
        sendStaicMessage((char *)"Flash-drive content:\n");
        fl=0;
        res = f_readdir(&dir, &fno);
        while((res==FR_OK)&&(fno.fname[0] != 0))
        {
          if (fno.fattrib & AM_DIR)
          {
            sprintf(fileName, "%s%s\n", fno.fname, "/");
          }else{
            sprintf(fileName, "%s\n", fno.fname);
          }
          sendStaicMessage(fileName);
          sendStaicMessage((char *)"**********\n");
          if (fno.fattrib & AM_DIR)
          {
            sendStaicMessage((char *)"This is foulder\n");
          }else{
            sendStaicMessage((char *)"File found\n");
            if(f_open(&MyFile,fileName,FA_READ)!=FR_OK)
            {
              sendStaicMessage((char *)"File open error\n");
              fl=1;
              ErrorCase();
            }else{
              sendStaicMessage((char *)"File open OK\n");
              ReadFileByString(); //MyFile is global!
              sendStaicMessage((char *)"THE END 2\n");
              f_close(&MyFile);
            }
          }
          res = f_readdir(&dir, &fno);
        }
      }
    }
    //sendStaicMessage("Root content finishes\n");
    f_closedir(&dir);
  }
}

void RootContentLinkedList(void)
{
  FRESULT res;
  DIR dir;
//  UINT i;
  static FILINFO fno;
  if (f_mount(&USBDISKFatFs, (TCHAR const*)USBHPath, 0)!=FR_OK)
  {
    sendStaicMessage((char *)"USB-drive mount error\n");
    //Error_Handler();
    ErrorCase();
  }else{
    res = f_opendir(&dir, USBHPath);
    if (res!=FR_OK)
    {
      //sendStaicMessage("Directory open error\n");
      ErrorCase();
    }else{
      //sendStaicMessage((char *)"Flash-drive content:\n");
      //Dynamic mamory
      struct node * pBuf;
      pBuf = malloc(sizeof(struct node));
      pBuf->previousNode = NULL;
      pNode = pBuf;
      //Linked list gen
      do
      {
        res = f_readdir(&dir, &fno);
        pBuf->fno=fno; // Write the data to this node
        if ((res!=FR_OK)||(fno.fname[0] == 0)) // Check if the list ended
        {
          if (pNode->fno.fname[0] == 0) {pNode = NULL;} // Erase Linked List if empty
          pBuf->previousNode->nextNode=NULL;
          free(pBuf);
          pBuf = NULL;
          break;
        }
        pBuf->nextNode = malloc(sizeof(struct node)); // create next node
        pBuf->nextNode->previousNode = pBuf; //make a backward link
        pBuf = pBuf->nextNode; // move pointer to the next node
        /*if (fno.fattrib & AM_DIR)
        {
          sprintf(fileName, "%s%s\n", fno.fname, "/");
          sendStaicMessage((char *)"This is foulder named ");
          sendStaicMessage(fileName);
        }else{
          sprintf(fileName, "%s\n", fno.fname);
          sendStaicMessage((char *)"This is file named ");
          sendStaicMessage(fileName);
        }*/
      }while(1);//while((res==FR_OK)&&(fno.fname[0] != 0));
      // make the backward links
    }
    //sendStaicMessage("Root content finishes\n");
    f_closedir(&dir);
  }
}
void GoToTheFirstNode(void)
{
  if (pNode!=NULL)
  {
    do
    {
      if(pNode->previousNode==NULL)
      {
        break;
      }else{
       pNode=pNode->previousNode;
      }
    }while(1);
  }
}
void GoToTheLastNode(void)
{
  if (pNode!=NULL)
  {
    do
    {
      if(pNode->nextNode==NULL)
      {
        break;
      }else{
       pNode=pNode->nextNode;
      }
    }while(1);
  }
}
void ClearNodes(void)
{
  if (pNode!=NULL)
  {
    GoToTheLastNode();
    do
    {
      if(pNode->previousNode==NULL)
      {
        break;
      }else{
       pNode=pNode->previousNode;
       free(pNode->nextNode);
      }
    }while(1);
    free(pNode);
    pNode=NULL;
  }
}
void ShowRootContentLinkedList(void)
{
  char fileName[100];
  if (pNode==NULL)
  {
    sendStaicMessage("Flash drive is empty\n");
  }else{
    do
    {
      if (pNode->fno.fattrib & AM_DIR)
      {
        sprintf(fileName, "%s%s\n", pNode->fno.fname, "/");
        sendStaicMessage((char *)"This is foulder named ");
        sendStaicMessage(fileName);
      }else{
        sprintf(fileName, "%s\n", pNode->fno.fname);
        sendStaicMessage((char *)"This is file named ");
        sendStaicMessage(fileName);
      }
      if(pNode->nextNode==NULL)
      {
        break;
      }else{
       pNode=pNode->nextNode;
      }
    }while(1);
  }
  GoToTheFirstNode();
}
void FoulderContentLinkedList(void)
{
  FRESULT res;
  DIR dir;
  char folderPath[100];
  static FILINFO fno;
  GoToTheLastNode();
  if (pNode->fno.fattrib & AM_DIR)
  {
    sprintf(folderPath, "%s%s%s\n", (char*)USBHPath, "/", pNode->fno.fname);
    res = f_opendir(&dir, (TCHAR*)folderPath);
    if (res!=FR_OK)
    {
      sendStaicMessage("Directory open error\n");
      ErrorCase();
    }else{
      sendStaicMessage((char *)"Flash-drive content:\n");
      //Dynamic mamory
      struct node * pBuf;
      pBuf = malloc(sizeof(struct node));
      pBuf->previousNode = NULL;
      ClearNodes();
      pNode = pBuf;
      //Linked list gen
      do
      {
        res = f_readdir(&dir, &fno);
        pBuf->fno=fno; // Write the data to this node
        if ((res!=FR_OK)||(fno.fname[0] == 0)) // Check if the list ended
        {
          if (pNode->fno.fname[0] == 0) {pNode = NULL;} // Erase Linked List if empty
          pBuf->previousNode->nextNode=NULL;
          free(pBuf);
          pBuf = NULL;
          break;
        }
        pBuf->nextNode = malloc(sizeof(struct node)); // create next node
        pBuf->nextNode->previousNode = pBuf; //make a backward link
        pBuf = pBuf->nextNode; // move pointer to the next node
      }while(1);
    }
    //sendStaicMessage("Root content finishes\n");
    f_closedir(&dir);
  }
}
int FindSlash(char * string)
{
  int i=0, j=0;
  for (j=0; j<=MAX_PATH_LENGTH; j++)
  {
    if(string[j]=='/') {i=j;}
  }
  return i;
}

void ReadFile(void)
{
//  UINT i;
  char filePath[100];
  sprintf(filePath, "%s%s%s", path, "/", pNode->fno.fname);
  if(f_open(&MyFile,filePath,FA_READ)!=FR_OK)
  {
    sendStaicMessage((char *)"File open error\n");
    ErrorCase();
  }else{
    ReadFileByString(); //MyFile is global!
    f_close(&MyFile);
  }
}
void OpenFolder(void)
{
  FRESULT res;
  DIR dir;
  static FILINFO fno;
  if (pNode->fno.fattrib & AM_DIR)
  {
    if (FindSlash(path)<3)
    {
      sprintf(path, "%s%s", path, pNode->fno.fname);
    }else{
      sprintf(path, "%s%s%s", path, "/", pNode->fno.fname);
    }
    res = f_opendir(&dir, (TCHAR*)path);
    if (res!=FR_OK)
    {
      sprintf(path, "%s", (char*)USBHPath);
      sendStaicMessage("Directory open error\n");
      ErrorCase();
    }else{
      sendStaicMessage(path);
      sendStaicMessage("\n");
      //Dynamic mamory
      struct node * pBuf;
      pBuf = malloc(sizeof(struct node));
      pBuf->previousNode = NULL;
      ClearNodes();
      pNode = pBuf;
      //Linked list gen
      do
      {
        res = f_readdir(&dir, &fno);
        pBuf->fno=fno; // Write the data to this node
        if ((res!=FR_OK)||(fno.fname[0] == 0)) // Check if the list ended
        {
          if (pNode->fno.fname[0] == 0) {pNode = NULL;} // Erase Linked List if empty
          pBuf->previousNode->nextNode=NULL;
          free(pBuf);
          pBuf = NULL;
          break;
        }
        pBuf->nextNode = malloc(sizeof(struct node)); // create next node
        pBuf->nextNode->previousNode = pBuf; //make a backward link
        pBuf = pBuf->nextNode; // move pointer to the next node
      }while(1);
    }
    //sendStaicMessage("Root content finishes\n");
    f_closedir(&dir);
  }
}
void ShowNode(void)
{
  if (pNode!=NULL)
  {
    sendStaicMessage(pNode->fno.fname);
    sendStaicMessage("\n");
  }
}
void OKfunction(void)
{
  if (pNode!=NULL)
  {
    if(pNode->fno.fattrib & AM_DIR)
    {
      OpenFolder();
      ShowNode();
    }else{
      ReadFile();
    }
  }
}
void BACKfunction(void)
{
  FRESULT res;
  DIR dir;
  FILINFO fno;
  int j, jStart = FindSlash(path);
  if (jStart<3)
  {
    for(j=jStart+1; j<MAX_PATH_LENGTH; j++)
    {
      path[j]='\0';
    }
  }else{
    for(j=jStart; j<MAX_PATH_LENGTH; j++)
    {
      path[j]='\0';
    }
  }
  sendStaicMessage(path);
  sendStaicMessage("\n");
  res = f_opendir(&dir, (TCHAR*)path);
  if (res!=FR_OK)
  {
    sprintf(path, "%s", (char*)USBHPath);
    sendStaicMessage("Directory open error\n");
    ErrorCase();
  }else{
    //Dynamic mamory
    struct node * pBuf;
    pBuf = malloc(sizeof(struct node));
    pBuf->previousNode = NULL;
    ClearNodes();
    pNode = pBuf;
    //Linked list gen
    do
    {
      res = f_readdir(&dir, &fno);
      pBuf->fno=fno; // Write the data to this node
      if ((res!=FR_OK)||(fno.fname[0] == 0)) // Check if the list ended
      {
        if (pNode->fno.fname[0] == 0) {pNode = NULL;} // Erase Linked List if empty
        pBuf->previousNode->nextNode=NULL;
        free(pBuf);
        pBuf = NULL;
        break;
      }
      pBuf->nextNode = malloc(sizeof(struct node)); // create next node
      pBuf->nextNode->previousNode = pBuf; //make a backward link
      pBuf = pBuf->nextNode; // move pointer to the next node
    }while(1);
  }
  //sendStaicMessage("Root content finishes\n");
  f_closedir(&dir);
  ShowNode();
}
void UPfunction(void)
{
  if (pNode->previousNode != NULL)
  {
    pNode = pNode->previousNode;
  }
  ShowNode();
}
void DOWNfunction(void)
{
  if (pNode->nextNode != NULL)
  {
    pNode = pNode->nextNode;
  }
  ShowNode();
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  // Pointer usage example
  /*
  struct node * pBuf;
  pBuf = malloc(sizeof(struct node));
  FILINFO fnoTest;
  FILINFO * pfnoTest;
  fnoTest = pBuf->fno;
  pfnoTest = &(pBuf->fno);
  free(pBuf);
  */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  sendStaicMessage((char *)"Start\n");
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)&rxBuffer, 1);
  //Path initialization
  sprintf(path, "%s", (char*)USBHPath);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */
    if (Appli_state == APPLICATION_START)
    {
      PinD12On();
      PinD13Off();
      PinD14Off();
      PinD15Off();
      //FileReadWrite();
      if (ReadContentFlag==1)
      {
        RootContentLinkedList();
        ShowNode();
        //ShowRootContentLinkedList();
        //FoulderContentLinkedList();
        //ShowRootContentLinkedList();
        //RootContentShort();
        ReadContentFlag=0;
      }
      if(ReadContentFlag==0)
      {
        switch (status)
        {
        case OK:
          OKfunction();
          status = IDLE;
          break;
        case BACK:
          BACKfunction();
          status = IDLE;
          break;
        case DOWN:
          DOWNfunction();
          status = IDLE;
          break;
        case UP:
          UPfunction();
          status = IDLE;
          break;
        }
      }
    }else if (Appli_state == APPLICATION_IDLE){
      PinD12Off();
      PinD13On();
      PinD14Off();
      PinD15Off();
    }else if (Appli_state == APPLICATION_READY){
      PinD12Off();
      PinD13Off();
      PinD14On();
      PinD15Off();
    }else if (Appli_state == APPLICATION_DISCONNECT){
      ClearNodes();
      MX_USB_HOST_Init();
      MX_FATFS_Init();
      PinD12Off();
      PinD13Off();
      PinD14Off();
      ReadContentFlag=1;
      sendStaicMessage((char *)"Flash-drive is disconnected\n");
      Appli_state = APPLICATION_IDLE;
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

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  int i=0;


  if (rxBuffer==';')
  {
    commentflag=0;
    rxString[rxindex]='\n';
  }
  if ((commentflag!=0))
  {
    rxString[rxindex]=rxBuffer;
    rxindex++;
  }
  if ((rxBuffer=='\n'))
  {
    //HAL_UART_Transmit(&huart2, (uint8_t *)rxString, rxindex+1, HAL_MAX_DELAY);
    status = IDLE;
    if ((rxString[0]=='O') && (rxString[1]=='K'))
    {
      status = OK;
      //sendStaicMessage((char *)"OK recieved\n");
      //OKfunction();
      //RootContentShort();
    }
    if ((rxString[0]=='D') && (rxString[1]=='O') && (rxString[2]=='W') && (rxString[3]=='N'))
    {
      status = DOWN;
      //sendStaicMessage((char *)"DOWN recieved\n");
      //DOWNfunction();
    }
    if ((rxString[0]=='U') && (rxString[1]=='P'))
    {
      status = UP;
      //sendStaicMessage((char *)"UP recieved\n");
      //UPfunction();
    }
    if ((rxString[0]=='B') && (rxString[1]=='A') && (rxString[2]=='C') && (rxString[3]=='K'))
    {
      status = BACK;
      //sendStaicMessage((char *)"BACK recieved\n");
      //BACKfunction();
    }


    commentflag=1;
    rxindex=0;
    for (i=0; i<MAX_STRING_LENGTH; i++)
    {
      rxString[i]=0;
    }
  }


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
