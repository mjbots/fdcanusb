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
namespace {
uint32_t GetUartTxRequest(USART_TypeDef* uart) {
  switch (reinterpret_cast<uint32_t>(uart)) {
    case UART_1: return DMA_REQUEST_USART1_TX;
    case UART_2: return DMA_REQUEST_USART2_TX;
    case UART_3: return DMA_REQUEST_USART3_TX;
    case UART_4: return DMA_REQUEST_UART4_TX;
#if defined (UART5)
    case UART_5: return DMA_REQUEST_UART5_TX;
#endif
  }
  mbed_die();
  return 0;
}

uint32_t GetUartRxRequest(USART_TypeDef* uart) {
  switch (reinterpret_cast<uint32_t>(uart)) {
    case UART_1: return DMA_REQUEST_USART1_RX;
    case UART_2: return DMA_REQUEST_USART2_RX;
    case UART_3: return DMA_REQUEST_USART3_RX;
    case UART_4: return DMA_REQUEST_UART4_RX;
#if defined (UART5)
    case UART_5: return DMA_REQUEST_UART5_RX;
#endif
  }
  mbed_die();
  return 0;
}

IRQn_Type GetDmaIrq(DMA_Channel_TypeDef* dma) {
  if (dma == DMA1_Channel1) { return DMA1_Channel1_IRQn; }
  if (dma == DMA1_Channel2) { return DMA1_Channel2_IRQn; }
  if (dma == DMA1_Channel3) { return DMA1_Channel3_IRQn; }
  if (dma == DMA1_Channel4) { return DMA1_Channel4_IRQn; }
  if (dma == DMA1_Channel5) { return DMA1_Channel5_IRQn; }
  if (dma == DMA1_Channel6) { return DMA1_Channel6_IRQn; }
  if (dma == DMA1_Channel7) { return DMA1_Channel7_IRQn; }
  if (dma == DMA1_Channel8) { return DMA1_Channel8_IRQn; }

  if (dma == DMA2_Channel1) { return DMA2_Channel1_IRQn; }
  if (dma == DMA2_Channel2) { return DMA2_Channel2_IRQn; }
  if (dma == DMA2_Channel3) { return DMA2_Channel3_IRQn; }
  if (dma == DMA2_Channel4) { return DMA2_Channel4_IRQn; }
  if (dma == DMA2_Channel5) { return DMA2_Channel5_IRQn; }
  if (dma == DMA2_Channel6) { return DMA2_Channel6_IRQn; }
  if (dma == DMA2_Channel7) { return DMA2_Channel7_IRQn; }
  if (dma == DMA2_Channel8) { return DMA2_Channel8_IRQn; }

  mbed_die();
}

IRQn_Type GetUsartIrq(USART_TypeDef* uart) {
  switch(reinterpret_cast<uint32_t>(uart)) {
#if defined (USART1_BASE)
    case UART_1: return USART1_IRQn;
#endif
#if defined (USART2_BASE)
    case UART_2: return USART2_IRQn;
#endif
#if defined (USART3_BASE)
    case UART_3: return USART3_IRQn;
#endif
#if defined (UART4_BASE)
    case UART_4: return UART4_IRQn;
#endif
#if defined (USART4_BASE)
    case UART_4: return USART4_IRQn;
#endif
#if defined (UART5_BASE)
    case UART_5: return UART5_IRQn;
#endif
#if defined (USART5_BASE)
    case UART_5: return USART5_IRQn;
#endif
#if defined (USART6_BASE)
    case UART_6: return USART6_IRQn;
#endif
#if defined (UART7_BASE)
    case UART_7: return UART7_IRQn;
#endif
#if defined (USART7_BASE)
    case UART_7: return USART7_IRQn;
#endif
#if defined (UART8_BASE)
    case UART_8: return UART8_IRQn;
#endif
#if defined (USART8_BASE)
    case UART_8: return USART8_IRQn;
#endif
#if defined (UART9_BASE)
    case UART_9: return UART9_IRQn;
#endif
#if defined (UART10_BASE)
    case UART_10: return UART10_IRQn;
#endif
}
  mbed_die();
  return {};
}

void EnableUart(USART_TypeDef* uart) {
#if defined (USART1_BASE)
  if (uart == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART2_BASE)
  if (uart == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART3_BASE)
  if (uart == USART3) {
    __HAL_RCC_USART3_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART4_BASE)
  if (uart == UART4) {
    __HAL_RCC_UART4_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART4_BASE)
  if (uart == USART4) {
    __HAL_RCC_USART4_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART5_BASE)
  if (uart == UART5) {
    __HAL_RCC_UART5_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART5_BASE)
  if (uart == USART5) {
    __HAL_RCC_USART5_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART6_BASE)
  if (uart == USART6) {
    __HAL_RCC_USART6_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART7_BASE)
  if (uart == UART7) {
    __HAL_RCC_UART7_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART7_BASE)
  if (uart == USART7) {
    __HAL_RCC_USART7_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART8_BASE)
  if (uart == UART8) {
    __HAL_RCC_UART8_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART8_BASE)
  if (uart == USART8) {
    __HAL_RCC_USART8_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART9_BASE)
  if (uart == UART9) {
    __HAL_RCC_UART9_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART10_BASE)
  if (uart == UART10) {
    __HAL_RCC_UART10_CLK_ENABLE();
    return;
  }
#endif
  mbed_die();
}

}

class Stm32G4AsyncUart::Impl {
 public:
  Impl(const Options& options) {
      // : stm32_serial_([&]() {
      //     Stm32Serial::Options s_options;
      //     s_options.tx = options.tx;
      //     s_options.rx = options.rx;
      //     s_options.baud_rate = options.baud_rate;
      //     return s_options;
      //   }()) {

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    pinmap_pinout(options.tx, PinMap_UART_TX);
    pinmap_pinout(options.rx, PinMap_UART_RX);
    if (options.tx != NC) {
      pin_mode(options.tx, PullUp);
    }
    if (options.rx != NC) {
      pin_mode(options.rx, PullUp);
    }

    auto* const uart = [&]() {
      const auto uart_tx = static_cast<UARTName>(
          pinmap_peripheral(options.tx, PinMap_UART_TX));
      const auto uart_rx = static_cast<UARTName>(
          pinmap_peripheral(options.rx, PinMap_UART_RX));
      return reinterpret_cast<USART_TypeDef*>(pinmap_merge(uart_tx, uart_rx));
    }();

    hdma_usart_rx_.Instance = options.rx_dma;
    hdma_usart_rx_.Init.Request = GetUartRxRequest(uart);
    hdma_usart_rx_.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart_rx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart_rx_.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart_rx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart_rx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart_rx_.Init.Mode = DMA_NORMAL;
    hdma_usart_rx_.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart_rx_) != HAL_OK)
    {
      mbed_die();
    }

    __HAL_LINKDMA(&uart_, hdmarx,hdma_usart_rx_);

    hdma_usart_tx_.Instance = options.tx_dma;
    hdma_usart_tx_.Init.Request = GetUartTxRequest(uart);
    hdma_usart_tx_.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart_tx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart_tx_.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart_tx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart_tx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart_tx_.Init.Mode = DMA_NORMAL;
    hdma_usart_tx_.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart_tx_) != HAL_OK)
    {
      mbed_die();
    }

    __HAL_LINKDMA(&uart_, hdmatx,hdma_usart_tx_);


    EnableUart(uart);

    uart_.Instance = uart;
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

    if (HAL_UARTEx_SetTxFifoThreshold(
            &uart_, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      mbed_die();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(
            &uart_, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      mbed_die();
    }
    if (HAL_UARTEx_EnableFifoMode(&uart_) != HAL_OK)
    {
      mbed_die();
    }

    tx_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_DMA_IRQHandler(&hdma_usart_tx_);
      });

    rx_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_DMA_IRQHandler(&hdma_usart_rx_);
      });

    uart_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_UART_IRQHandler(&uart_);
      });

    const auto tx_irq = GetDmaIrq(options.tx_dma);
    const auto rx_irq = GetDmaIrq(options.rx_dma);
    const auto usart_irq = GetUsartIrq(uart);

    NVIC_SetVector(
        tx_irq,
        reinterpret_cast<uint32_t>(tx_callback_.raw_function));
    NVIC_SetVector(
        rx_irq,
        reinterpret_cast<uint32_t>(rx_callback_.raw_function));
    NVIC_SetVector(
        usart_irq,
        reinterpret_cast<uint32_t>(uart_callback_.raw_function));

    HAL_NVIC_SetPriority(tx_irq, 0, 0);
    HAL_NVIC_EnableIRQ(tx_irq);

    HAL_NVIC_SetPriority(rx_irq, 0, 0);
    HAL_NVIC_EnableIRQ(rx_irq);

    HAL_NVIC_SetPriority(usart_irq, 1, 0);
    HAL_NVIC_EnableIRQ(usart_irq);
  }

  void AsyncReadSome(const base::string_span& data,
                     const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_read_callback_);
    current_read_callback_ = callback;
    current_read_bytes_ = data.size();

    // TODO(jpieper): By invoking the HAL receive API for each
    // AsyncReadSome, we are basically guaranteed to drop bytes
    // between calls, even for relatively slow baud rates.
    if (HAL_UART_Receive_DMA(
            &uart_,
            reinterpret_cast<uint8_t*>(data.data()),
            data.size()) != HAL_OK) {
      mbed_die();
    }
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
    const bool idle = [&]() {
      if (uart_.Instance->ISR & USART_ISR_IDLE) {
        uart_.Instance->ICR = USART_ISR_IDLE;
        return true;
      }
      return false;
    }();

    if (idle && current_read_callback_ &&
        uart_.RxState != HAL_UART_STATE_READY) {
      // We would use the HAL API, but it only supports aborting both
      // TX and RX at the same time.  We only want to abort RX.
      CLEAR_BIT(uart_.Instance->CR3, USART_CR3_DMAR);
      if (HAL_DMA_Abort(uart_.hdmarx) != HAL_OK) {
        mbed_die();
      }

      // The contents of 'UART_EndRxTransfer'

      /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
      CLEAR_BIT(uart_.Instance->CR1, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));
      CLEAR_BIT(uart_.Instance->CR3, (USART_CR3_EIE | USART_CR3_RXFTIE));

      /* At end of Rx process, restore huart->RxState to Ready */
      uart_.RxState = HAL_UART_STATE_READY;

      /* Reset RxIsr function pointer */
      uart_.RxISR = NULL;
    }

    if (current_write_callback_) {
      if (uart_.gState == HAL_UART_STATE_READY) {
        decltype(current_write_callback_) copy;
        using std::swap;
        swap(copy, current_write_callback_);
        copy(micro::error_code(), current_write_bytes_);
      }
    }
    if (current_read_callback_) {
      if (uart_.RxState == HAL_UART_STATE_READY) {
        decltype(current_read_callback_) copy;
        using std::swap;
        swap(copy, current_read_callback_);
        copy(micro::error_code(),
             current_read_bytes_ - hdma_usart_rx_.Instance->CNDTR);
      }
    }
  }

  // Stm32Serial stm32_serial_;
  UART_HandleTypeDef uart_{};
  DMA_HandleTypeDef hdma_usart_rx_;
  DMA_HandleTypeDef hdma_usart_tx_;

  micro::SizeCallback current_write_callback_;
  ssize_t current_write_bytes_ = 0;

  micro::SizeCallback current_read_callback_;
  ssize_t current_read_bytes_ = 0;

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
