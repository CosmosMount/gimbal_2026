#include "bsp_usart.hpp"
#include "XRobot.hpp"
#include "config_remoter.hpp"
#include "config_referee.hpp"
#include "tx_api.h"
#include "usart.h"

/*------------全局变量------------*/
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

// uart7 not used
__attribute__((section (".RAM_D1"))) uint8_t UART7RxBuffer[256] = {0};
__attribute__((section (".RAM_D1"))) uint8_t USART1RxBuffer[256] = {0};
extern uint8_t dr16_rx[DR16_DATA_SIZE];
extern TX_SEMAPHORE RemoterGot;
extern RefereeRingBuffer referee_fifo;

/**
 * @brief  Configures the USART.
 * @param  None
 * @retval None
 */

void USART_Init()
{
  // usart1
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1RxBuffer, 256);
  // uart5
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_uart5_rx, DMA_IT_TC);
  __HAL_UART_SEND_REQ(&huart5, UART_RXDATA_FLUSH_REQUEST);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dr16_rx, DR16_DATA_SIZE);
  // uart7
  // __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
  // __HAL_DMA_ENABLE_IT(&hdma_uart7_rx, DMA_IT_TC);
  // __HAL_DMA_DISABLE_IT(&hdma_uart7_tx, DMA_IT_HT);
  // __HAL_DMA_ENABLE_IT(&hdma_uart7_tx, DMA_IT_TC);
  // // __HAL_UART_SEND_REQ(&huart7, UART_RXDATA_FLUSH_REQUEST); // 清空缓存，消除接收错位
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7RxBuffer, 256);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) 
{
  if (huart == &huart5) 
  {
    SCB_InvalidateDCache_by_Addr((uint32_t*)dr16_rx, DR16_DATA_SIZE);
    tx_semaphore_put(&RemoterGot);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dr16_rx, DR16_DATA_SIZE);
  } 
  else if (huart == &huart7) 
  {
    // SCB_InvalidateDCache_by_Addr((uint32_t*)UART7RxBuffer, 256);
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7RxBuffer, 256);
  }
  else if (huart == &huart1) 
  {
    SCB_InvalidateDCache_by_Addr((uint32_t*)USART1RxBuffer, 256);
    referee_fifo.push(USART1RxBuffer, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1RxBuffer, 256);
  }
}
