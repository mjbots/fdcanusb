// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fw/stm32g4_async_uart.h"

#include "mjlib/micro/atomic_event_queue.h"
#include "mjlib/micro/callback_table.h"

#include "fw/error.h"
#include "fw/stm32_serial.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace fw {

class Stm32G4AsyncUart::Impl {
 public:
  Impl(const Options& options) {

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart1_rx_.Instance = DMA1_Channel2;
    hdma_usart1_rx_.Init.Request = DMA_REQUEST_USART2_RX;
    hdma_usart1_rx_.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx_.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx_.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx_.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx_) != HAL_OK)
    {
      mbed_die();
    }

    __HAL_LINKDMA(&uart_,hdmarx,hdma_usart1_rx_);

    /* USART2_TX Init */
    hdma_usart1_tx_.Instance = DMA1_Channel1;
    hdma_usart1_tx_.Init.Request = DMA_REQUEST_USART2_TX;
    hdma_usart1_tx_.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx_.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx_.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx_.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx_) != HAL_OK)
    {
      mbed_die();
    }

     __HAL_LINKDMA(&uart_,hdmatx,hdma_usart1_tx_);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    __HAL_RCC_USART2_CLK_ENABLE();

    uart_.Instance = USART2;
    uart_.Init.BaudRate = options.baud_rate;
    uart_.Init.WordLength = UART_WORDLENGTH_8B;
    uart_.Init.StopBits = UART_STOPBITS_1;
    uart_.Init.Parity = UART_PARITY_NONE;
    uart_.Init.Mode = UART_MODE_TX_RX;
    uart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_.Init.OverSampling = UART_OVERSAMPLING_16;
    uart_.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart_.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart_.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&uart_) != HAL_OK)
    {
      mbed_die();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&uart_, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      mbed_die();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&uart_, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      mbed_die();
    }
    if (HAL_UARTEx_DisableFifoMode(&uart_) != HAL_OK)
    {
      mbed_die();
    }

    tx_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_DMA_IRQHandler(&hdma_usart1_tx_);
      });

    rx_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_DMA_IRQHandler(&hdma_usart1_rx_);
      });

    uart_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_UART_IRQHandler(&uart_);
      });

    NVIC_SetVector(
        DMA1_Channel1_IRQn,
        reinterpret_cast<uint32_t>(tx_callback_.raw_function));
    NVIC_SetVector(
        DMA1_Channel2_IRQn,
        reinterpret_cast<uint32_t>(rx_callback_.raw_function));
    NVIC_SetVector(
        USART2_IRQn,
        reinterpret_cast<uint32_t>(uart_callback_.raw_function));
  }

  void AsyncReadSome(const base::string_span& data,
                     const micro::SizeCallback& callback) {
    // do nothing for now
  }

  void AsyncWriteSome(const string_view& data,
                      const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_write_callback_);
    current_write_callback_ = callback;
    current_write_bytes_ = data.size();

    if (HAL_UART_Transmit_DMA(
            &uart_,
            const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())),
            data.size()) != HAL_OK) {
      mbed_die();
    }
  }

  void Poll() {
    if (current_write_callback_) {
      if (HAL_UART_GetState(&uart_) == HAL_UART_STATE_READY) {
        auto copy = current_write_callback_;
        current_write_callback_ = {};
        copy(micro::error_code(), current_write_bytes_);
      }
    }
  }

  UART_HandleTypeDef uart_;
  DMA_HandleTypeDef hdma_usart1_rx_;
  DMA_HandleTypeDef hdma_usart1_tx_;

  micro::SizeCallback current_write_callback_;
  ssize_t current_write_bytes_ = 0;

  micro::CallbackTable::Callback tx_callback_;
  micro::CallbackTable::Callback rx_callback_;
  micro::CallbackTable::Callback uart_callback_;
};

Stm32G4AsyncUart::Stm32G4AsyncUart(micro::Pool* pool,
                                   MillisecondTimer* timer,
                                   const Options& options)
    : impl_(pool, options) {}

Stm32G4AsyncUart::~Stm32G4AsyncUart() {}

void Stm32G4AsyncUart::AsyncReadSome(const base::string_span& data,
                                     const micro::SizeCallback& callback) {
  impl_->AsyncReadSome(data, callback);
}

void Stm32G4AsyncUart::AsyncWriteSome(const string_view& data,
                                      const micro::SizeCallback& callback) {
  impl_->AsyncWriteSome(data, callback);
}

void Stm32G4AsyncUart::Poll() {
  impl_->Poll();
}

}
