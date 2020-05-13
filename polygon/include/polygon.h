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
#include "structs.h"

namespace manifold {

int CCW(glm::vec2 p0, glm::vec2 p1, glm::vec2 p2);
Polygons Halfedge2Poly(const Halfedge *halfedge, const int *halfedgeBegin,
                       const int *halfedgeEnd,
                       std::function<glm::vec2(int)> vertProjection);
std::vector<glm::ivec3> Triangulate(const Polygons &polys);

std::vector<Halfedge> Polygons2Edges(const Polygons &polys);
std::vector<Halfedge> Triangles2Edges(const std::vector<glm::ivec3> &triangles);
void CheckTopology(const std::vector<Halfedge> &halfedges);
void CheckTopology(const std::vector<glm::ivec3> &triangles,
                   const Polygons &polys);
void CheckGeometry(const std::vector<glm::ivec3> &triangles,
                   const Polygons &polys);
void Dump(const Polygons &polys);
ExecutionParams &PolygonParams();

}  // namespace manifold