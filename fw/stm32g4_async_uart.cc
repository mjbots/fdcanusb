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

#define DMA_GET_SR(__HANDLE__) ((reinterpret_cast<uint32_t>((__HANDLE__)->Instance) > (reinterpret_cast<uint32_t>(DMA1_Channel8))) ? (DMA2->ISR) : (DMA1->ISR))

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

IRQn_Type FindUartRxIrq(USART_TypeDef* uart) {
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
}

class Stm32G4AsyncUart::Impl {
 public:
  Impl(micro::Pool* pool, MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        options_(options),
        stm32_serial_([&]() {
            Stm32Serial::Options s_options;
            s_options.rx = options.rx;
            s_options.tx = options.tx;
            s_options.baud_rate = options.baud_rate;
            return s_options;
          }()),
        dir_(options.dir, 0) {

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    uart_ = stm32_serial_.uart();

    MJ_ASSERT(uart_ != nullptr);
    uart_rx_irq_ = FindUartRxIrq(uart_);

    if (options.tx != NC) {
      tx_dma_.Instance = options_.tx_dma;
      tx_dma_.Init.Request = GetUartTxRequest(uart_);
      tx_dma_.Init.Direction = DMA_MEMORY_TO_PERIPH;
      tx_dma_.Init.PeriphInc = DMA_PINC_DISABLE;
      tx_dma_.Init.MemInc = DMA_MINC_ENABLE;
      tx_dma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      tx_dma_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      tx_dma_.Init.Mode = DMA_NORMAL;
      tx_dma_.Init.Priority = DMA_PRIORITY_LOW;
      HAL_DMA_Init(&tx_dma_);

      tx_callback_ = micro::CallbackTable::MakeFunction([this]() {
          this->IRQ_HandleTransmit();
        });

      const auto tx_irq = GetDmaIrq(options.tx_dma);

      NVIC_SetVector(
          tx_irq, reinterpret_cast<uint32_t>(tx_callback_.raw_function));
      HAL_NVIC_SetPriority(tx_irq, 5, 0);

      tx_dma_flags_.gif = __HAL_DMA_GET_GI_FLAG_INDEX(&tx_dma_);
      tx_dma_flags_.te = __HAL_DMA_GET_TE_FLAG_INDEX(&tx_dma_);
      tx_dma_flags_.ht = __HAL_DMA_GET_HT_FLAG_INDEX(&tx_dma_);
      tx_dma_flags_.tc = __HAL_DMA_GET_TC_FLAG_INDEX(&tx_dma_);

      NVIC_EnableIRQ(tx_irq);
    }

    if (options.rx != NC) {
      rx_dma_.Instance = options_.rx_dma;
      rx_dma_.Init.Request = GetUartRxRequest(uart_);
      rx_dma_.Init.Direction = DMA_PERIPH_TO_MEMORY;
      rx_dma_.Init.PeriphInc = DMA_PINC_DISABLE;
      rx_dma_.Init.MemInc = DMA_MINC_ENABLE;
      rx_dma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      rx_dma_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      rx_dma_.Init.Mode = DMA_NORMAL;
      rx_dma_.Init.Priority = DMA_PRIORITY_LOW;
      HAL_DMA_Init(&rx_dma_);

      rx_callback_ =  micro::CallbackTable::MakeFunction([this]() {
          this->IRQ_HandleReceive();
        });

      const auto rx_irq = GetDmaIrq(options.rx_dma);
      NVIC_SetVector(
          rx_irq, reinterpret_cast<uint32_t>(rx_callback_.raw_function));
      HAL_NVIC_SetPriority(rx_irq, 5, 0);

      // Notify when there are idle times on the bus.
      uart_->CR1 |= USART_CR1_IDLEIE;

      uart_callback_ = micro::CallbackTable::MakeFunction([this]() {
          this->IRQ_HandleUart();
        });
      NVIC_SetVector(uart_rx_irq_,
                     reinterpret_cast<uint32_t>(uart_callback_.raw_function));
      HAL_NVIC_SetPriority(uart_rx_irq_, 5, 0);

      rx_dma_flags_.gif = __HAL_DMA_GET_GI_FLAG_INDEX(&rx_dma_);
      rx_dma_flags_.te = __HAL_DMA_GET_TE_FLAG_INDEX(&rx_dma_);
      rx_dma_flags_.ht = __HAL_DMA_GET_HT_FLAG_INDEX(&rx_dma_);
      rx_dma_flags_.tc = __HAL_DMA_GET_TC_FLAG_INDEX(&rx_dma_);

      NVIC_EnableIRQ(rx_irq);
      NVIC_EnableIRQ(uart_rx_irq_);
    }
  }

  void AsyncReadSome(const base::string_span& data,
                     const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_read_callback_);
    current_read_callback_ = callback;
    current_read_bytes_ = data.size();
    if (HAL_DMA_Start_IT(&rx_dma_, reinterpret_cast<uint32_t>(&(uart_->RDR)),
                         reinterpret_cast<uint32_t>(data.data()),
                         current_read_bytes_) != HAL_OK) {
      mbed_die();
    }

    uart_->CR3 |= USART_CR3_DMAR;
  }

  void AsyncWriteSome(const string_view& data,
                      const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_write_callback_);
    current_write_callback_ = callback;
    current_write_bytes_ = data.size();
    if (HAL_DMA_Start_IT(&tx_dma_, reinterpret_cast<uint32_t>(data.data()),
                         reinterpret_cast<uint32_t>(&(uart_->TDR)),
                         data.size() != HAL_OK)) {
      mbed_die();
    }
    uart_->ICR |= USART_ICR_TCCF;
    uart_->CR3 |= USART_CR3_DMAT;
  }

  void IRQ_HandleTransmit() {
    const ssize_t amount_sent = current_write_bytes_ - tx_dma_.Instance->CNDTR;

    const auto dma_sr = DMA_GET_SR(&tx_dma_);
    const auto uart_sr = uart_->ISR;
    (void) uart_sr;

    if (dma_sr & tx_dma_flags_.ht) {
      __HAL_DMA_CLEAR_FLAG(&tx_dma_, tx_dma_flags_.ht);
      // Just ignore this.
    }

    if (dma_sr & tx_dma_flags_.gif) {
      __HAL_DMA_CLEAR_FLAG(&tx_dma_, tx_dma_flags_.gif);
      // Ignore this too.
    }

    bool emit = false;
    micro::error_code error;
    if (dma_sr & tx_dma_flags_.te) {
      __HAL_DMA_CLEAR_FLAG(&tx_dma_, tx_dma_flags_.te);
      error = errc::kDmaStreamTransferError;
      emit = true;
    }

    if (dma_sr & tx_dma_flags_.tc) {
      __HAL_DMA_CLEAR_FLAG(&tx_dma_, tx_dma_flags_.tc);
      error = {};
      emit = true;
    }

    if (emit) {
      // Have the UART stop requesting DMA.
      uart_->CR3 &= ~(USART_CR3_DMAT);
      if (HAL_DMA_Abort_IT(&tx_dma_) != HAL_OK) {
        mbed_die();
      }

      event_queue_.Queue([this, error, amount_sent]() {
          auto copy = current_write_callback_;

          current_write_callback_ = {};
          current_write_bytes_ = 0;

          copy(error, amount_sent);
        });
    }
  }

  void IRQ_HandleReceive() {
    // Process any error flags, then call the callback.

    // Read the status register.
    const auto dma_sr = DMA_GET_SR(&rx_dma_);
    const auto uart_sr = uart_->ISR;
    ssize_t bytes_read = 0;

    if (dma_sr & rx_dma_flags_.ht) {
      __HAL_DMA_CLEAR_FLAG(&rx_dma_, rx_dma_flags_.ht);
      // otherwise ignore
    }

    if (dma_sr & rx_dma_flags_.gif) {
      __HAL_DMA_CLEAR_FLAG(&rx_dma_, rx_dma_flags_.gif);
      // otherwise ignore
    }

    bool emit = false;

    if (dma_sr & rx_dma_flags_.te) {
      emit = true;
      // We had a transfer error.
      pending_rx_error_ = [&]() {
        if (uart_sr & USART_ISR_ORE) {
          uart_->ICR |= USART_ICR_ORECF;
          return errc::kUartOverrunError;
        } else if (uart_sr & USART_ISR_FE) {
          uart_->ICR |= USART_ICR_FECF;
          return errc::kUartFramingError;
        } else if (uart_sr & USART_ISR_NE) {
          uart_->ICR |= USART_ICR_NECF;
          return errc::kUartNoiseError;
        } else if (uart_sr & USART_ISR_PE) {
          uart_->ICR |= USART_ICR_PECF;
          return errc::kUartParityError;
        } else {
          return errc::kDmaStreamTransferError;
        }
      }();

      __HAL_DMA_CLEAR_FLAG(&rx_dma_, rx_dma_flags_.te);
    }

    if (dma_sr & rx_dma_flags_.tc) {
      emit = true;

      __HAL_DMA_CLEAR_FLAG(&rx_dma_, rx_dma_flags_.tc);

      // We completely filled our buffer.
      bytes_read = current_read_bytes_;
    }

    if (emit) {
      // Have the UART stop requesting DMA.
      uart_->CR3 &= ~(USART_CR3_DMAR);
      if (HAL_DMA_Abort_IT(&rx_dma_) != HAL_OK) {
        mbed_die();
      }

      event_queue_.Queue([this, bytes_read]() {
          auto copy = current_read_callback_;
          auto rx_error = pending_rx_error_;

          current_read_callback_ = {};
          current_read_bytes_ = 0;
          pending_rx_error_ = {};

          copy(rx_error, bytes_read);
        });
    }
  }

  void IRQ_HandleUart() {
    // We should only get this for idle flags.  We should cancel our
    // DMA transfer, make sure it is truly done, then we can report
    // the bytes we have.
    if (uart_->ISR & USART_ISR_IDLE) {
      uart_->ICR |= USART_ICR_IDLECF;

      if (HAL_DMA_Abort_IT(&rx_dma_) != HAL_OK) {
        mbed_die();
      }

      ssize_t bytes_read = current_read_bytes_ - rx_dma_.Instance->CNDTR;
      event_queue_.Queue([this, bytes_read]() {
          auto copy = current_read_callback_;
          auto rx_error = pending_rx_error_;

          current_read_callback_ = {};
          current_read_bytes_ = 0;
          pending_rx_error_ = {};

          copy(rx_error, bytes_read);
        });
    }
  }

  MillisecondTimer* const timer_;
  const Options options_;
  Stm32Serial stm32_serial_;
  DigitalOut dir_;

  using EventQueue = micro::AtomicEventQueue<24>;
  EventQueue event_queue_;

  USART_TypeDef* uart_ = nullptr;
  IRQn_Type uart_rx_irq_ = {};

  // This buffer serves as a place to store things in between calls to
  // AsyncReadSome so that there is minimal chance of data loss even
  // at high data rates.
  volatile uint16_t* rx_buffer_ = nullptr;
  uint16_t rx_buffer_pos_ = 0;

  DMA_HandleTypeDef rx_dma_ = {};
  DMA_HandleTypeDef tx_dma_ = {};

  struct DmaFlags {
    uint32_t gif = 0;
    uint32_t te = 0;
    uint32_t ht = 0;
    uint32_t tc = 0;
  };

  DmaFlags rx_dma_flags_;
  DmaFlags tx_dma_flags_;

  micro::CallbackTable::Callback tx_callback_;
  micro::CallbackTable::Callback rx_callback_;
  micro::CallbackTable::Callback uart_callback_;

  micro::SizeCallback current_read_callback_;
  ssize_t current_read_bytes_ = 0;
  micro::error_code pending_rx_error_;

  micro::SizeCallback current_write_callback_;
  ssize_t current_write_bytes_ = 0;

  DMA_TypeDef* dma1_ = DMA1;
  DMA_TypeDef* dma2_ = DMA2;
};

Stm32G4AsyncUart::Stm32G4AsyncUart(micro::Pool* pool,
                                   MillisecondTimer* timer,
                                   const Options& options)
    : impl_(pool, pool, timer, options) {}

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
  impl_->event_queue_.Poll();
}

}
