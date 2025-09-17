// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#pragma once

#include <cstdlib>

namespace fw {
template<typename T, size_t size>
class Fifo {
 public:
  bool full() const {
    return static_cast<uint32_t>((put_index_ - get_index_) == size);
  }

  bool empty() const {
    return put_index_ == get_index_;
  }

  void push(T&& value) {
    data_[put_index_ % size] = std::move(value);
    put_index_ = put_index_ + 1;
  }

  const T& top() const {
    return data_[get_index_ % size];
  }

  void push(const T& value) {
    if (full()) { return; }

    data_[put_index_ % size] = value;
    put_index_ = put_index_ + 1;
  }

  T pop() {
    const auto old_get_index = get_index_;
    get_index_ = get_index_ + 1;
    return data_[old_get_index % size];
  }

  T data_[size] = {};
  uint32_t put_index_ = 0;
  uint32_t get_index_ = 0;
};
}
