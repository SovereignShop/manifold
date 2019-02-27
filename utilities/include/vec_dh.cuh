// Copyright 2019 Emmett Lalish
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#define GLM_FORCE_CUDA
#include <thrust/device_vector.h>
#include <glm/glm.hpp>

namespace manifold {

using EdgeVertsD = thrust::pair<int, int>;

inline std::ostream& operator<<(std::ostream& stream, const EdgeVertsD& edge) {
  return stream << edge.first << ", " << edge.second;
}

template <typename T>
class VecDH {
 public:
  VecDH() {}

  VecDH(int size, T val = T()) {
    device_.resize(size, val);
    host_valid_ = false;
  }

  VecDH(const std::vector<T>& vec) {
    host_ = vec;
    device_valid_ = false;
  }

  int size() const { return device_valid_ ? device_.size() : host_.size(); }

  void resize(int size, T val = T()) {
    if (device_valid_) device_.resize(size, val);
    if (host_valid_) host_.resize(size, val);
  }

  void shrink_to_fit() const {
    if (device_valid_) device_.shrink_to_fit();
    if (host_valid_) host_.shrink_to_fit();
  }

  void swap(VecDH<T>& other) {
    host_.swap(other.host_);
    device_.swap(other.device_);
    thrust::swap(host_valid_, other.host_valid_);
    thrust::swap(device_valid_, other.device_valid_);
  }

  using VecH = typename thrust::host_vector<T>;
  using IterD = typename thrust::device_vector<T>::iterator;
  using IterH = typename thrust::host_vector<T>::iterator;
  using IterDc = typename thrust::device_vector<T>::const_iterator;
  using IterHc = typename thrust::host_vector<T>::const_iterator;

  IterH begin() {
    RefreshHost();
    device_valid_ = false;
    return host_.begin();
  }

  IterH end() {
    RefreshHost();
    device_valid_ = false;
    return host_.end();
  }

  IterHc cbegin() const {
    RefreshHost();
    return host_.cbegin();
  }

  IterHc cend() const {
    RefreshHost();
    return host_.cend();
  }

  IterHc begin() const { return cbegin(); }
  IterHc end() const { return cend(); }

  IterD beginD() {
    RefreshDevice();
    host_valid_ = false;
    return device_.begin();
  }

  IterD endD() {
    RefreshDevice();
    host_valid_ = false;
    return device_.end();
  }

  IterDc cbeginD() const {
    RefreshDevice();
    return device_.cbegin();
  }

  IterDc cendD() const {
    RefreshDevice();
    return device_.cend();
  }

  IterDc beginD() const { return cbeginD(); }
  IterDc endD() const { return cendD(); }

  T* ptrD() {
    RefreshDevice();
    host_valid_ = false;
    return device_.data().get();
  }

  const T* cptrD() const {
    RefreshDevice();
    return device_.data().get();
  }

  const T* ptrD() const { return cptrD(); }

  const VecH& H() const {
    RefreshHost();
    return host_;
  }

  VecH& H() {
    RefreshHost();
    device_valid_ = false;
    return host_;
  }

  void Dump() const {
    RefreshHost();
    std::cout << "VecDH = " << std::endl;
    for (int i = 0; i < size(); ++i) {
      std::cout << i << ", " << host_[i] << ", " << std::endl;
    }
    std::cout << std::endl;
  }

 private:
  mutable bool host_valid_ = true;
  mutable bool device_valid_ = true;
  mutable thrust::host_vector<T> host_;
  mutable thrust::device_vector<T> device_;

  void RefreshHost() const {
    if (!host_valid_) {
      host_ = device_;
      host_valid_ = true;
    }
  }

  void RefreshDevice() const {
    if (!device_valid_) {
      device_ = host_;
      device_valid_ = true;
    }
  }
};

template <typename T>
class VecD {
 public:
  VecD(const VecDH<T>& vec) : ptr_(vec.ptrD()), size_(vec.size()) {}

  __device__ const T& operator[](int i) const { return ptr_[i]; }
  __device__ int size() const { return size_; }

 private:
  T const* const ptr_;
  const int size_;
};

}  // namespace manifold