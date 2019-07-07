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
#include <functional>
#include <memory>
#include "structs.h"

namespace manifold {

class Manifold {
 public:
  // Creation
  Manifold();
  Manifold(const Mesh&);
  Manifold DeepCopy() const;
  static Manifold Tetrahedron();
  static Manifold Cube();
  static Manifold Octahedron();
  static Manifold Sphere(int circularSegments);  // rounds up to multiple of 4

  // Topological
  Manifold(const std::vector<Manifold>&);
  std::vector<Manifold> Decompose() const;

  // Extraction
  Mesh Extract() const;

  // Information
  bool IsEmpty() const;
  int NumVert() const;
  int NumEdge() const;
  int NumTri() const;
  Box BoundingBox() const;
  float Volume() const;
  float SurfaceArea() const;

  // Modification
  Manifold& Translate(glm::vec3);
  Manifold& Scale(glm::vec3);
  Manifold& Rotate(float xDegrees, float yDegrees = 0.0f,
                   float zDegrees = 0.0f);
  Manifold& Warp(std::function<void(glm::vec3&)>);

  // Boolean
  enum class OpType { ADD, SUBTRACT, INTERSECT };
  Manifold Boolean(const Manifold& second, OpType op) const;
  // Boolean operation shorthand
  Manifold operator+(const Manifold&) const;  // ADD (Union)
  Manifold operator-(const Manifold&) const;  // SUBTRACT (Difference)
  Manifold operator^(const Manifold&) const;  // INTERSECT
  // First result is the intersection, second is the difference. This is more
  // efficient than doing them separately.
  std::pair<Manifold, Manifold> Split(const Manifold&) const;
  std::pair<Manifold, Manifold> SplitByPlane(glm::vec3 normal,
                                             float originOffset) const;

  // Testing hooks
  bool IsValid() const;
  int NumOverlaps(const Manifold& second) const;

  ~Manifold();
  Manifold(Manifold&&);
  Manifold& operator=(Manifold&&);
  struct Impl;

 private:
  std::unique_ptr<Impl> pImpl_;

  // Implicit copy is private because it is expensive; use DeepCopy() above.
  Manifold(const Manifold& other);
  Manifold& operator=(const Manifold& other);
};
}  // namespace manifold