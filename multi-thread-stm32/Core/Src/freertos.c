/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"

#include "cmsis_os.h"
#include "internal.h"
#include "main.h"
#include "printf.h"
#include "semphr.h"
#include "stm32f0xx_hal_conf.h"
#include "stm32f0xx_hal_spi.h"
#include "string.h"
#include "task.h"

extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

/* USER CODE END Application */
void TaskSerial(void *argument);
void TaskSPI(void *argument);
void TaskI2C(void *argument);
void TaskReadTimer(void *argument);
void TaskReadUART(void *argument);
void SemaphoreSenderTask(void *parameters);
void SemaphoreReceiverTask(void *parameters);

/* Definitions for Handle Task */
osThreadId_t myTaskSerialHandle;
osThreadId_t TaskHandleSPI;
osThreadId_t TaskHandleI2C;
osThreadId_t TaskHandleTimer;
osThreadId_t TaskHandleUART;
TaskHandle_t SenderTaskHandle;
TaskHandle_t ReceiverTaskHandle;

/* Create semaphore */
SemaphoreHandle_t UART_Semaphore_Handle;

const osThreadAttr_t highTask_attributes = {
    .name = "highTaskPriority",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
const osThreadAttr_t normalTask_attributes = {
    .name = "normalTaskPriority",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
osThreadId_t myTaskSerialHandle;
const osThreadAttr_t lowTask_attributes = {
    .name = "lowTaskPriority",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow1,
};

/* USER CODE BEGIN TaskSerial */
/**
 * @brief Function to read Serial Print.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END TaskSerial */
void TaskSerial(void *argument) {
    for (;;) {
        printf("\n---Task Serial Print---\n");
        printf("Example Serial Print in STM32\n\n");
        osDelay(DELAY_ONE_SECOND);
    }
}

/* USER CODE BEGIN TaskSPI */
/**
 * @brief  Function task to read Data SPI.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END TaskSPI */
void TaskSPI(void *argument) {
    for (;;) {
        uint8_t data_received[10];  // Example buffer to store received data
        printf("\n---Task SPI---\n");
        if ((HAL_SPI_Receive(&hspi2, data_received, sizeof(data_received), SENSOR_TIMEOUT) == HAL_OK) && (strcmp(data_received, "") != 0)) {
            printf("SPI Received data: %s\n", data_received);
        } else {
            // Error handling if read operation fails
            printf("SPI Can't read data!\n");
        }
        osDelay(DELAY_ONE_SECOND);
    }
}

/* USER CODE BEGIN TaskI2C */
/**
 * @brief  Function task to read Data I2C.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END TaskI2C */
void TaskI2C(void *argument) {
    for (;;) {
        uint8_t data_received[10];  // Example buffer to store received data
        printf("\n---Task I2C---\n");
        if ((HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS, data_received, sizeof(data_received), SENSOR_TIMEOUT) == HAL_OK) && (strcmp(data_received, "") != 0)) {
            printf("I2C Received data: %s\n", data_received);
        } else {
            printf("I2C Can't read data!\n");
        }
        // Perform SPI read operation
        osDelay(DELAY_ONE_SECOND * 3);
    }
}

void TaskReadUART(void *argument) {
    uint8_t rx_data = 0;
    for (;;) {
        printf("\n---Task Read UART---\n");
        if ((HAL_UART_Receive(&huart2, &rx_data, 1, 100) == HAL_OK) && (rx_data > 0)) {
            // Receive data from UART2 with timeout of 100ms
            printf("UART Received data: %d\n", rx_data);
        } else {
            printf("UART Can't read data!\n");
        }
    }
}

void TaskReadTimer(void *argument) {
    uint32_t tim_value = 0;
    for (;;) {
        printf("\n---Task Timer---\n");
        tim_value = __HAL_TIM_GET_COUNTER(&htim1);      // Read TIM counter value
        printf("TIM Counter Value: %lu\n", tim_value);  // Process timer value here
        osDelay(DELAY_ONE_SECOND * 5);
    }
}

void SemaphoreSenderTask(void *argument) {
    char tx_data[] = "Hello from Sender Task!\r\n";

    for (;;) {
        // Wait for semaphore to be available
        printf("\n---Task Semaphore Send---\n");
        if (xSemaphoreTake(UART_Semaphore_Handle, portMAX_DELAY) == pdTRUE) {
            // Send data via UART2
            HAL_UART_Transmit(&huart2, (uint8_t *)tx_data, strlen(tx_data), HAL_MAX_DELAY);

            // Give back the semaphore
            xSemaphoreGive(UART_Semaphore_Handle);

            // Delay for a while before sending again
            vTaskDelay(pdMS_TO_TICKS(DELAY_ONE_SECOND));
        }
    }
}

void SemaphoreReceiverTask(void *argument) {
    uint8_t rx_data;
    for (;;) {
        printf("\n---Task Semaphore Give---\n");
        if (xSemaphoreTake(UART_Semaphore_Handle, portMAX_DELAY) == pdTRUE) {
            // Receive data via UART2
            if (HAL_UART_Receive(&huart2, &rx_data, 1, HAL_MAX_DELAY) == HAL_OK) {
                // Print received data
                printf("Received data: %c\n", rx_data);
            }
            // Give back the semaphore
            xSemaphoreGive(UART_Semaphore_Handle);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_ONE_SECOND));
    }
}

void InitFreeRTOS(void) {
    /* Create the thread(s) */
    myTaskSerialHandle = osThreadNew(TaskSerial, NULL, &lowTask_attributes);
    TaskHandleSPI = osThreadNew(TaskSPI, NULL, &normalTask_attributes);
    TaskHandleI2C = osThreadNew(TaskI2C, NULL, &normalTask_attributes);
    TaskHandleTimer = osThreadNew(TaskReadTimer, NULL, &lowTask_attributes);
    TaskHandleUART = osThreadNew(TaskReadUART, NULL, &normalTask_attributes);

    UART_Semaphore_Handle = xSemaphoreCreateBinary();
    xTaskCreate(SemaphoreSenderTask, "Sender_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &SenderTaskHandle);
    xTaskCreate(SemaphoreReceiverTask, "Receiver_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &ReceiverTaskHandle);

    osKernelStart();
}
