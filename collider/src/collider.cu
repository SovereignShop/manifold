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

#include "collider.cuh"
#include "utils.cuh"

// Adjustable parameters
constexpr int kInitialLength = 128;
constexpr int kLengthMultiple = 4;
// Fundamental constants
constexpr int kRoot = 1;

namespace {
using namespace manifold;

__host__ __device__ bool IsLeaf(int node) { return node % 2 == 0; }
__host__ __device__ bool IsInternal(int node) { return node % 2 == 1; }
__host__ __device__ int Node2Internal(int node) { return (node - 1) / 2; }
__host__ __device__ int Internal2Node(int internal) { return internal * 2 + 1; }
__host__ __device__ int Node2Leaf(int node) { return node / 2; }
__host__ __device__ int Leaf2Node(int leaf) { return leaf * 2; }

__host__ __device__ int AtomicIncrement(int* ptr) {
#ifdef __CUDA_ARCH__
  return atomicAdd(ptr, 1);
#else
#pragma omp atomic
  return (*ptr)++;
#endif
}

struct CreateRadixTree {
  int* nodeParent_;
  thrust::pair<int, int>* internalChildren_;
  const VecD<uint32_t> leafMorton_;

  __host__ __device__ int PrefixLength(uint32_t a, uint32_t b) const {
// count-leading-zeros is used to find the number of identical highest-order
// bits
#ifdef __CUDA_ARCH__
    return __clz(a ^ b);
#else
    return __builtin_clz(a ^ b);
#endif
  }

  __host__ __device__ int PrefixLength(int i, int j) const {
    if (j < 0 || j >= leafMorton_.size()) {
      return -1;
    } else {
      int out;
      if (leafMorton_[i] == leafMorton_[j])
        // use index to disambiguate
        out = 32 +
              PrefixLength(static_cast<uint32_t>(i), static_cast<uint32_t>(j));
      else
        out = PrefixLength(leafMorton_[i], leafMorton_[j]);
      return out;
    }
  }

  __host__ __device__ int RangeEnd(int i) const {
    // Determine direction of range (+1 or -1)
    int dir = PrefixLength(i, i + 1) - PrefixLength(i, i - 1);
    dir = (dir > 0) - (dir < 0);
    // Compute conservative range length with exponential increase
    int commonPrefix = PrefixLength(i, i - dir);
    int max_length = kInitialLength;
    while (PrefixLength(i, i + dir * max_length) > commonPrefix)
      max_length *= kLengthMultiple;
    // Compute precise range length with binary search
    int length = 0;
    for (int step = max_length / 2; step > 0; step /= 2) {
      if (PrefixLength(i, i + dir * (length + step)) > commonPrefix)
        length += step;
    }
    return i + dir * length;
  }

  __host__ __device__ int FindSplit(int first, int last) const {
    int commonPrefix = PrefixLength(first, last);
    // Find the furthest object that shares more than commonPrefix bits with the
    // first one, using binary search.
    int split = first;
    int step = last - first;
    do {
      step = (step + 1) >> 1;  // divide by 2, rounding up
      int newSplit = split + step;
      if (newSplit < last) {
        int splitPrefix = PrefixLength(first, newSplit);
        if (splitPrefix > commonPrefix) split = newSplit;
      }
    } while (step > 1);
    return split;
  }

  __host__ __device__ void operator()(int internal) {
    int first = internal;
    // Find the range of objects with a common prefix
    int last = RangeEnd(first);
    if (first > last) thrust::swap(first, last);
    // Determine where the next-highest difference occurs
    int split = FindSplit(first, last);
    int child1 = split == first ? Leaf2Node(split) : Internal2Node(split);
    ++split;
    int child2 = split == last ? Leaf2Node(split) : Internal2Node(split);
    // Record parent_child relationships.
    internalChildren_[internal].first = child1;
    internalChildren_[internal].second = child2;
    int node = Internal2Node(internal);
    nodeParent_[child1] = node;
    nodeParent_[child2] = node;
  }
};

template <typename T>
struct FindCollisions {
  int* query_overlaps_;
  int* face_overlaps_;
  int* num_overlaps_;
  const Box* nodeBBox_;
  const thrust::pair<int, int>* internalChildren_;

  __host__ __device__ bool RecordCollision(int node,
                                           const thrust::tuple<T, int>& query) {
    bool overlaps = nodeBBox_[node].DoesOverlap(thrust::get<0>(query));
    if (overlaps && IsLeaf(node)) {
      int pos = AtomicIncrement(num_overlaps_);
      query_overlaps_[pos] = thrust::get<1>(query);
      face_overlaps_[pos] = Node2Leaf(node);
    }
    return overlaps && IsInternal(node);  // Should traverse into node
  }

  __host__ __device__ void operator()(thrust::tuple<T, int> query) {
    // stack cannot overflow because radix tree has max depth 30 (Morton code) +
    // 32 (index).
    int stack[64];
    int top = -1;
    // Depth-first search
    int node = kRoot;
    for (;;) {
      int internal = Node2Internal(node);
      int child1 = internalChildren_[internal].first;
      int child2 = internalChildren_[internal].second;

      bool traverse1 = RecordCollision(child1, query);
      bool traverse2 = RecordCollision(child2, query);

      if (!traverse1 && !traverse2) {
        if (top < 0) break;   // done
        node = stack[top--];  // get a saved node
      } else {
        node = traverse1 ? child1 : child2;  // go here next
        if (traverse1 && traverse2) {
          stack[++top] = child2;  // save the other for later
        }
      }
    }
  }
};

struct BuildInternalBoxes {
  Box* nodeBBox_;
  int* counter_;
  const int* nodeParent_;
  const thrust::pair<int, int>* internalChildren_;

  __host__ __device__ void operator()(int leaf) {
    int node = Leaf2Node(leaf);
    do {
      node = nodeParent_[node];
      int internal = Node2Internal(node);
      if (AtomicIncrement(&counter_[internal]) == 0) return;
      nodeBBox_[node] = nodeBBox_[internalChildren_[internal].first].Union(
          nodeBBox_[internalChildren_[internal].second]);
    } while (node != kRoot);
  }
};

struct TranslateBox {
  const glm::vec3 v;
  __host__ __device__ void operator()(Box& box) { box += v; }
};

struct ScaleBox {
  const glm::vec3 v;
  __host__ __device__ void operator()(Box& box) { box *= v; }
};

}  // namespace

namespace manifold {
Collider::Collider(const VecDH<Box>& leafBB,
                   const VecDH<uint32_t>& leafMorton) {
  ALWAYS_ASSERT(leafBB.size() == leafMorton.size(), runtimeErr, "");
  int num_nodes = 2 * leafBB.size() - 1;
  // assign and allocate members
  nodeBBox_.resize(num_nodes);
  nodeParent_.resize(num_nodes, -1);
  internalChildren_.resize(leafBB.size() - 1, thrust::make_pair(-1, -1));
  // organize tree
  thrust::for_each_n(thrust::make_counting_iterator(0), NumInternal(),
                     CreateRadixTree({nodeParent_.ptrD(),
                                      internalChildren_.ptrD(), leafMorton}));
  UpdateBoxes(leafBB);
}

template <typename T>
void Collider::Collisions(VecDH<int>& querriesOut, VecDH<int>& facesOut,
                          const VecDH<T>& querriesIn) const {
  ALWAYS_ASSERT(querriesOut.size() == facesOut.size(), runtimeErr, "");
  ALWAYS_ASSERT(facesOut.size() > 0, runtimeErr,
                "querriesOut and facesOut are empty! Their size is the maximum "
                "overlaps that will be reported.");
  // scalar number of overlaps found
  VecDH<int> n_overlaps_dev(1, 0);
  // calculate Bounding Box overlaps
  thrust::for_each_n(
      zip(querriesIn.cbeginD(), thrust::make_counting_iterator(0)),
      querriesIn.size(),
      FindCollisions<T>({querriesOut.ptrD(), facesOut.ptrD(),
                         n_overlaps_dev.ptrD(), nodeBBox_.ptrD(),
                         internalChildren_.ptrD()}));
  int n_overlaps = n_overlaps_dev.H()[0];
  ALWAYS_ASSERT(n_overlaps <= querriesOut.size(), runtimeErr,
                "max_overlaps exceeded.");
  // remove unused part of array
  querriesOut.resize(n_overlaps);
  querriesOut.shrink_to_fit();
  facesOut.resize(n_overlaps);
  facesOut.shrink_to_fit();
}

void Collider::UpdateBoxes(const VecDH<Box>& leafBB) {
  ALWAYS_ASSERT(leafBB.size() == NumLeaves(), runtimeErr, "");
  // copy in leaf node Boxs
  strided_range<VecDH<Box>::IterD> leaves(nodeBBox_.beginD(), nodeBBox_.endD(),
                                          2);
  thrust::copy(leafBB.cbeginD(), leafBB.cendD(), leaves.begin());
  // create global counters
  VecDH<int> counter_(NumInternal());
  thrust::fill(counter_.beginD(), counter_.endD(), 0);
  // kernel over leaves to save internal Boxs
  thrust::for_each_n(
      thrust::make_counting_iterator(0), NumLeaves(),
      BuildInternalBoxes({nodeBBox_.ptrD(), counter_.ptrD(), nodeParent_.ptrD(),
                          internalChildren_.ptrD()}));
}

void Collider::Translate(glm::vec3 T) {
  thrust::for_each(nodeBBox_.beginD(), nodeBBox_.endD(), TranslateBox({T}));
}

void Collider::Scale(glm::vec3 T) {
  thrust::for_each(nodeBBox_.beginD(), nodeBBox_.endD(), ScaleBox({T}));
}

template void Collider::Collisions<Box>(VecDH<int>& querriesOut,
                                        VecDH<int>& facesOut,
                                        const VecDH<Box>& querriesIn) const;

template void Collider::Collisions<glm::vec3>(
    VecDH<int>& querriesOut, VecDH<int>& facesOu,
    const VecDH<glm::vec3>& querriesIn) const;

}  // namespace manifold