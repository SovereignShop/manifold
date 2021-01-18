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

#include "polygon.h"

#include <algorithm>
#include <random>

#include "gtest/gtest.h"

namespace {

using namespace manifold;

void StandardizePoly(SimplePolygon &p) {
  auto start = std::min_element(
      p.begin(), p.end(),
      [](const PolyVert &v1, const PolyVert &v2) { return v1.idx < v2.idx; });
  std::rotate(p.begin(), start, p.end());
}

void StandardizePolys(Polygons &polys) {
  for (auto &p : polys) StandardizePoly(p);
  std::sort(polys.begin(), polys.end(),
            [](SimplePolygon &p1, SimplePolygon &p2) {
              return p1[0].idx < p2[0].idx;
            });
}

void Identical(Polygons p1, Polygons p2) {
  ASSERT_EQ(p1.size(), p2.size());
  StandardizePolys(p1);
  StandardizePolys(p2);
  for (int i = 0; i < p1.size(); ++i) {
    ASSERT_EQ(p1[i].size(), p2[i].size());
    for (int j = 0; j < p1[i].size(); ++j) {
      ASSERT_EQ(p1[i][j].idx, p2[i][j].idx);
    }
  }
}

Polygons Turn180(Polygons polys) {
  for (SimplePolygon &poly : polys) {
    for (PolyVert &vert : poly) {
      vert.pos *= -1;
    }
  }
  return polys;
}

Polygons Duplicate(Polygons polys) {
  float xMin = 1.0 / 0.0;
  float xMax = -1.0 / 0.0;
  int indexMax = 0;
  for (SimplePolygon &poly : polys) {
    for (PolyVert &vert : poly) {
      xMin = std::min(xMin, vert.pos.x);
      xMax = std::max(xMax, vert.pos.x);
      indexMax = std::max(indexMax, vert.idx);
    }
  }
  ++indexMax;
  const float shift = xMax - xMin;

  const int nPolys = polys.size();
  for (int i = 0; i < nPolys; ++i) {
    SimplePolygon poly = polys[i];
    for (PolyVert &vert : poly) {
      vert.pos.x += shift;
      vert.idx += indexMax;
    }
    polys.push_back(poly);
  }
  return polys;
}

void TestPoly(const Polygons &polys, int expectedNumTri) {
  //   PolygonParams().verbose = true;
  PolygonParams().intermediateChecks = true;

  std::vector<glm::ivec3> triangles;
  EXPECT_NO_THROW(triangles = Triangulate(polys));
  EXPECT_EQ(triangles.size(), expectedNumTri) << "Basic";

  EXPECT_NO_THROW(triangles = Triangulate(Turn180(polys)));
  EXPECT_EQ(triangles.size(), expectedNumTri) << "Turn 180";

  EXPECT_NO_THROW(triangles = Triangulate(Duplicate(polys)));
  EXPECT_EQ(triangles.size(), 2 * expectedNumTri) << "Duplicate";
}
}  // namespace

/**
 * These polygons are all valid geometry. Some are clearly valid, while many are
 * marginal, but all should produce correct topology and geometry, within
 * tolerance.
 */
TEST(Polygon, SimpleHole) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0, -2), 0, Edge::kNoIdx},  //
      {glm::vec2(2, 2), 1, Edge::kNoIdx},   //
      {glm::vec2(0, 4), 2, Edge::kNoIdx},   //
      {glm::vec2(-3, 3), 3, Edge::kNoIdx},  //
  });
  polys.push_back({
      {glm::vec2(0, -1), 4, Edge::kNoIdx},  //
      {glm::vec2(-1, 1), 5, Edge::kNoIdx},  //
      {glm::vec2(1, 1), 6, Edge::kNoIdx},   //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, SimpleHole2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0, 1.63299), 0, Edge::kNoIdx},           //
      {glm::vec2(-1.41421, -0.816496), 1, Edge::kNoIdx},  //
      {glm::vec2(1.41421, -0.816496), 2, Edge::kNoIdx},   //
  });
  polys.push_back({
      {glm::vec2(0, 1.02062), 3, Edge::kNoIdx},           //
      {glm::vec2(0.883883, -0.51031), 4, Edge::kNoIdx},   //
      {glm::vec2(-0.883883, -0.51031), 5, Edge::kNoIdx},  //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, MultiMerge) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-7, 0), 0, Edge::kNoIdx},   //
      {glm::vec2(-6, 3), 1, Edge::kNoIdx},   //
      {glm::vec2(-5, 1), 2, Edge::kNoIdx},   //
      {glm::vec2(-4, 6), 3, Edge::kNoIdx},   //
      {glm::vec2(-3, 2), 4, Edge::kNoIdx},   //
      {glm::vec2(-2, 5), 5, Edge::kNoIdx},   //
      {glm::vec2(-1, 4), 6, Edge::kNoIdx},   //
      {glm::vec2(0, 12), 7, Edge::kNoIdx},   //
      {glm::vec2(-6, 10), 8, Edge::kNoIdx},  //
      {glm::vec2(-8, 11), 9, Edge::kNoIdx},  //
  });
  polys.push_back({
      {glm::vec2(-5, 7), 10, Edge::kNoIdx},  //
      {glm::vec2(-6, 8), 11, Edge::kNoIdx},  //
      {glm::vec2(-5, 9), 12, Edge::kNoIdx},  //
  });
  TestPoly(polys, 13);
}

TEST(Polygon, Colinear) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-5.48368, -3.73905), 0, Edge::kNoIdx},   //
      {glm::vec2(-4.9881, -4.51552), 1, Edge::kNoIdx},    //
      {glm::vec2(-4.78988, -4.13186), 2, Edge::kNoIdx},   //
      {glm::vec2(-4.82012, -4.13999), 3, Edge::kNoIdx},   //
      {glm::vec2(-4.84314, -4.14617), 4, Edge::kNoIdx},   //
      {glm::vec2(-4.85738, -4.13581), 5, Edge::kNoIdx},   //
      {glm::vec2(-4.86772, -4.12831), 6, Edge::kNoIdx},   //
      {glm::vec2(-4.87337, -4.12422), 7, Edge::kNoIdx},   //
      {glm::vec2(-4.88097, -4.1187), 8, Edge::kNoIdx},    //
      {glm::vec2(-4.89799, -4.10634), 9, Edge::kNoIdx},   //
      {glm::vec2(-4.90219, -4.10329), 10, Edge::kNoIdx},  //
      {glm::vec2(-4.90826, -4.09887), 11, Edge::kNoIdx},  //
      {glm::vec2(-4.90846, -4.09873), 12, Edge::kNoIdx},  //
      {glm::vec2(-4.91227, -4.09597), 13, Edge::kNoIdx},  //
      {glm::vec2(-4.92199, -4.0889), 14, Edge::kNoIdx},   //
      {glm::vec2(-5.0245, -4.01443), 15, Edge::kNoIdx},   //
      {glm::vec2(-5.02494, -4.01412), 16, Edge::kNoIdx},  //
      {glm::vec2(-5.02536, -4.01381), 17, Edge::kNoIdx},  //
      {glm::vec2(-5.0316, -4.00927), 18, Edge::kNoIdx},   //
      {glm::vec2(-5.03211, -4.00891), 19, Edge::kNoIdx},  //
      {glm::vec2(-5.05197, -3.99448), 20, Edge::kNoIdx},  //
      {glm::vec2(-5.14757, -3.92504), 21, Edge::kNoIdx},  //
      {glm::vec2(-5.21287, -3.8776), 22, Edge::kNoIdx},   //
      {glm::vec2(-5.29419, -3.81853), 23, Edge::kNoIdx},  //
      {glm::vec2(-5.29907, -3.81499), 24, Edge::kNoIdx},  //
      {glm::vec2(-5.36732, -3.76541), 25, Edge::kNoIdx},  //
  });
  TestPoly(polys, 24);
}

TEST(Polygon, Merges) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-3.22039, 10.2769), 0, Edge::kNoIdx},   //
      {glm::vec2(-3.12437, 10.4147), 1, Edge::kNoIdx},   //
      {glm::vec2(-3.99093, 10.1781), 2, Edge::kNoIdx},   //
      {glm::vec2(-3.8154, 10.0716), 3, Edge::kNoIdx},    //
      {glm::vec2(-3.78982, 10.0893), 4, Edge::kNoIdx},   //
      {glm::vec2(-3.55033, 10.2558), 5, Edge::kNoIdx},   //
      {glm::vec2(-3.50073, 10.2549), 6, Edge::kNoIdx},   //
      {glm::vec2(-3.47018, 10.2572), 7, Edge::kNoIdx},   //
      {glm::vec2(-3.42633, 10.2605), 8, Edge::kNoIdx},   //
      {glm::vec2(-3.34311, 10.2604), 9, Edge::kNoIdx},   //
      {glm::vec2(-3.32096, 10.2633), 10, Edge::kNoIdx},  //
  });
  TestPoly(polys, 9);
}

TEST(Polygon, ColinearY) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0, 0), 0, Edge::kNoIdx},    //
      {glm::vec2(1, 1), 1, Edge::kNoIdx},    //
      {glm::vec2(2, 1), 2, Edge::kNoIdx},    //
      {glm::vec2(3, 1), 3, Edge::kNoIdx},    //
      {glm::vec2(4, 1), 4, Edge::kNoIdx},    //
      {glm::vec2(4, 2), 5, Edge::kNoIdx},    //
      {glm::vec2(3, 2), 6, Edge::kNoIdx},    //
      {glm::vec2(2, 2), 7, Edge::kNoIdx},    //
      {glm::vec2(1, 2), 8, Edge::kNoIdx},    //
      {glm::vec2(0, 3), 9, Edge::kNoIdx},    //
      {glm::vec2(-1, 2), 10, Edge::kNoIdx},  //
      {glm::vec2(-2, 2), 11, Edge::kNoIdx},  //
      {glm::vec2(-3, 2), 12, Edge::kNoIdx},  //
      {glm::vec2(-4, 2), 13, Edge::kNoIdx},  //
      {glm::vec2(-4, 1), 14, Edge::kNoIdx},  //
      {glm::vec2(-3, 1), 15, Edge::kNoIdx},  //
      {glm::vec2(-2, 1), 16, Edge::kNoIdx},  //
      {glm::vec2(-1, 1), 17, Edge::kNoIdx},  //
  });
  TestPoly(polys, 16);
}

TEST(Polygon, Concave) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-0.707107, -0.707107), 0, 10},    //
      {glm::vec2(1, 0), 1, 21},                    //
      {glm::vec2(0.683013, 0), 2, -1},             //
      {glm::vec2(0.37941, -0.232963), 3, -1},      //
      {glm::vec2(0.37941, -0.232963), 4, -1},      //
      {glm::vec2(1.49012e-08, -0.183013), 5, -1},  //
      {glm::vec2(1.49012e-08, -0.183013), 6, -1},  //
      {glm::vec2(-0.140431, 0), 7, 21},            //
      {glm::vec2(-1, 0), 8, 6},                    //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Concave2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(4, 0), 1, Edge::kNoIdx},    //
      {glm::vec2(3, 2), 3, Edge::kNoIdx},    //
      {glm::vec2(3, 3), 4, Edge::kNoIdx},    //
      {glm::vec2(6, 5), 6, Edge::kNoIdx},    //
      {glm::vec2(6, 14), 13, Edge::kNoIdx},  //
      {glm::vec2(0, 13), 12, Edge::kNoIdx},  //
      {glm::vec2(0, 12), 11, Edge::kNoIdx},  //
      {glm::vec2(3, 11), 10, Edge::kNoIdx},  //
      {glm::vec2(4, 10), 9, Edge::kNoIdx},   //
      {glm::vec2(5, 8), 8, Edge::kNoIdx},    //
      {glm::vec2(1, 7), 7, Edge::kNoIdx},    //
      {glm::vec2(2, 1), 2, Edge::kNoIdx},    //
  });
  TestPoly(polys, 10);
}

TEST(Polygon, Sliver) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(2.82003, 0), 0, 181},         //
      {glm::vec2(2.82003, 0), 1, -1},          //
      {glm::vec2(2.06106, 0), 2, -1},          //
      {glm::vec2(2.05793, 0.0680379), 3, -1},  //
      {glm::vec2(2.0641, 0.206908), 4, -1},    //
      {glm::vec2(2.28446, 1.04696), 5, -1},    //
      {glm::vec2(2.35006, 1.2499), 6, 181},    //
      {glm::vec2(-2.82003, 15), 7, 191},       //
      {glm::vec2(-2.82003, 0), 8, 179},        //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Duplicate) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-32.0774, -10.431), 0, 36},     //
      {glm::vec2(-31.7347, -6.10349), 1, -1},    //
      {glm::vec2(-31.8646, -5.61859), 2, -1},    //
      {glm::vec2(-31.8646, -5.61859), 3, -1},    //
      {glm::vec2(-31.8646, -5.61859), 4, -1},    //
      {glm::vec2(-31.8646, -5.61859), 5, -1},    //
      {glm::vec2(-31.8646, -5.61859), 6, -1},    //
      {glm::vec2(-31.8646, -5.61859), 7, -1},    //
      {glm::vec2(-31.8646, -5.61859), 8, -1},    //
      {glm::vec2(-31.8646, -5.61859), 9, -1},    //
      {glm::vec2(-31.8646, -5.61859), 10, -1},   //
      {glm::vec2(-31.8646, -5.61859), 11, -1},   //
      {glm::vec2(-31.8646, -5.61859), 12, -1},   //
      {glm::vec2(-31.8646, -5.61859), 13, -1},   //
      {glm::vec2(-31.8646, -5.61859), 14, -1},   //
      {glm::vec2(-31.8646, -5.61859), 15, -1},   //
      {glm::vec2(-31.8646, -5.61859), 16, -1},   //
      {glm::vec2(-31.8646, -5.61859), 17, -1},   //
      {glm::vec2(-31.8646, -5.61859), 18, -1},   //
      {glm::vec2(-31.8646, -5.61859), 19, -1},   //
      {glm::vec2(-31.8646, -5.61859), 20, -1},   //
      {glm::vec2(-32.0774, -3.18655), 21, 226},  //
  });
  TestPoly(polys, 20);
}

TEST(Polygon, Folded) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(2.82003, 0), 0, 110},          //
      {glm::vec2(1.23707, 4.20995), 1, -1},     //
      {glm::vec2(1.14141, 4.09091), 2, -1},     //
      {glm::vec2(1.05896, 3.94496), 3, -1},     //
      {glm::vec2(0.00757742, 2.72727), 4, -1},  //
      {glm::vec2(-0.468092, 1.94364), 5, -1},   //
      {glm::vec2(-1.06107, 1.36364), 6, -1},    //
      {glm::vec2(-1.79214, 0.34649), 7, -1},    //
      {glm::vec2(-2.27417, 0), 8, -1},          //
      {glm::vec2(-2.82003, 0), 9, 174},         //
      {glm::vec2(-2.82003, 0), 10, 108},        //
  });
  TestPoly(polys, 9);
}

TEST(Polygon, NearlyLinear) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(2.82003, -8.22814e-05), 0, 231},   //
      {glm::vec2(2.82003, -8.22814e-05), 1, -1},    //
      {glm::vec2(2.31802, -8.22814e-05), 2, -1},    //
      {glm::vec2(-0.164567, -8.22813e-05), 3, -1},  //
      {glm::vec2(-0.857388, -8.22814e-05), 4, -1},  //
      {glm::vec2(-1.01091, -8.22814e-05), 5, 257},  //
      {glm::vec2(-1.01091, -8.22814e-05), 6, 233},  //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Sliver2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(27.4996014, 8.6873703), 74, 151},    //
      {glm::vec2(28.27701, 9.52887344), 76, 156},     //
      {glm::vec2(27.6687469, 10.8811588), 104, 152},  //
      {glm::vec2(27.5080414, 8.79682922), 242, -1},   //
      {glm::vec2(27.5594807, 8.75218964), 207, -1},   //
      {glm::vec2(27.4996014, 8.6873703), 268, 152},   //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Sliver3) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0, -2.65168381), 369, 1173},               //
      {glm::vec2(0, -0.792692184), 1889, 5777},             //
      {glm::vec2(0, -0.792692184), 2330, -1},               //
      {glm::vec2(0, -1.04356134), 2430, -1},                //
      {glm::vec2(-0.953957975, -0.768045247), 2331, 5777},  //
      {glm::vec2(-1.36363637, -0.757460594), 1892, 1174},   //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Colinear2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(11.7864399, -7.4572401), 4176, 13521},    //
      {glm::vec2(11.6818037, -7.30982304), 24873, -1},     //
      {glm::vec2(11.6777582, -7.30626202), 28498, -1},     //
      {glm::vec2(11.6789398, -7.30578804), 24872, 13521},  //
      {glm::vec2(11.3459997, -6.83671999), 4889, 16146},   //
      {glm::vec2(11.25597, -6.9267602), 4888, 13520},      //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Split) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-0.707106769, -0.707106769), 1, 10},     //
      {glm::vec2(1, 0), 14, 21},                          //
      {glm::vec2(0.683012664, 0), 25, -1},                //
      {glm::vec2(0.379409522, -0.232962906), 33, -1},     //
      {glm::vec2(0.379409522, -0.232962906), 32, -1},     //
      {glm::vec2(1.49011612e-08, -0.183012664), 31, -1},  //
      {glm::vec2(1.49011612e-08, -0.183012664), 30, -1},  //
      {glm::vec2(-0.14043057, 0), 24, 21},                //
      {glm::vec2(-1, 0), 4, 6},                           //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Duplicates) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-15, -8.10255623), 1648, 151},        //
      {glm::vec2(-15, -9.02439785), 1650, 157},        //
      {glm::vec2(-13.636364, -9.4640789), 1678, 152},  //
      {glm::vec2(-14.996314, -8.10623646), 1916, -1},  //
      {glm::vec2(-15, -8.10639), 1845, -1},            //
      {glm::vec2(-15, -8.10255623), 1922, 152},        //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Sliver4) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-0.375669807, 8.90489388), 7474, 14932},  //
      {glm::vec2(0, 8.39722729), 7459, 14704},             //
      {glm::vec2(0, 8.9723053), 7468, 14940},              //
      {glm::vec2(0, 8.9723053), 7469, 14998},              //
      {glm::vec2(0, 8.96719646), 7467, 15006},             //
      {glm::vec2(0, 8.89326191), 7466, 15000},             //
      {glm::vec2(0, 8.78047276), 7465, 14941},             //
      {glm::vec2(-0.330551624, 8.8897438), 7473, 14945},   //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, Simple1) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(4.04059982, -4.01843977), 2872, 8988},   //
      {glm::vec2(3.95867562, -4.25263977), 24604, -1},    //
      {glm::vec2(4.23459578, -4.30138493), 28274, -1},    //
      {glm::vec2(4.235569, -4.30127287), 28273, -1},      //
      {glm::vec2(4.23782539, -4.30141878), 24602, 8986},  //
  });
  TestPoly(polys, 3);
}

TEST(Polygon, Simple2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-1, -1), 1, 8},       //
      {glm::vec2(-0.5, -0.5), 9, -1},  //
      {glm::vec2(-1, 0), 11, -1},      //
      {glm::vec2(0, 1), 12, -1},       //
      {glm::vec2(0.5, 0.5), 10, 8},    //
      {glm::vec2(1, 1), 7, 12},        //
      {glm::vec2(-1, 1), 3, 6},        //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Simple3) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(19.7193489, 6.15445995), 19798, 28537},  //
      {glm::vec2(20.2308197, 5.64299059), 31187, -1},     //
      {glm::vec2(20.3464642, 5.65459776), 27273, -1},     //
      {glm::vec2(20.3733711, 5.65404081), 27274, -1},     //
      {glm::vec2(20.373394, 5.65404034), 31188, 28538},   //
      {glm::vec2(20.8738098, 6.15445995), 19801, 28541},  //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Simple4) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(15, -12.7135563), 287, 346},          //
      {glm::vec2(15, -10.6843739), 288, 350},          //
      {glm::vec2(15, -10.6843739), 492, -1},           //
      {glm::vec2(15, -11.0041418), 413, -1},           //
      {glm::vec2(15, -11.4550743), 409, -1},           //
      {glm::vec2(15, -11.4550743), 411, -1},           //
      {glm::vec2(14.9958763, -11.4545326), 408, -1},   //
      {glm::vec2(14.4307623, -11.3802214), 412, -1},   //
      {glm::vec2(13.9298496, -11.2768612), 480, 349},  //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Simple5) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-27.3845081, 0.375669748), 364, 601},  //
      {glm::vec2(-27.6389656, 0), 365, 1210},           //
      {glm::vec2(-27.1156006, 0), 355, 597},            //
      {glm::vec2(-27.1156006, 0), 356, 516},            //
      {glm::vec2(-27.1202412, 0), 359, 528},            //
      {glm::vec2(-27.1875362, 0), 360, 525},            //
      {glm::vec2(-27.290184, 0), 362, 599},             //
      {glm::vec2(-27.3733444, 0.330451876), 363, 586},  //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, Simple6) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-7.99813318, 12.8888826), 25009, 40308},  //
      {glm::vec2(-7.85714436, 12.9125195), 25006, 40309},  //
      {glm::vec2(-7.85714436, 12.9807196), 25005, 40305},  //
      {glm::vec2(-7.88929749, 12.9593039), 25007, 40306},  //
      {glm::vec2(-7.99812126, 12.8888912), 25008, 40310},  //
  });
  TestPoly(polys, 3);
}

TEST(Polygon, TouchingHole) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-2, -1), 0, Edge::kNoIdx},  //
      {glm::vec2(2, -1), 1, Edge::kNoIdx},   //
      {glm::vec2(2, 1), 2, Edge::kNoIdx},    //
      {glm::vec2(-2, 1), 3, Edge::kNoIdx},   //
  });
  polys.push_back({
      {glm::vec2(-1, -1), 4, Edge::kNoIdx},  //
      {glm::vec2(-1, 1), 5, Edge::kNoIdx},   //
      {glm::vec2(1, 1), 6, Edge::kNoIdx},    //
      {glm::vec2(1, -1), 7, Edge::kNoIdx},   //
  });
  TestPoly(polys, 8);
}

TEST(Polygon, Degenerate) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(1, -1), 0, 9},    //
      {glm::vec2(1, 1), 1, 11},    //
      {glm::vec2(1, 1), 2, -1},    //
      {glm::vec2(1, -1), 3, -1},   //
      {glm::vec2(1, -1), 4, -1},   //
      {glm::vec2(-1, -1), 5, 11},  //
      {glm::vec2(-1, -1), 6, 10},  //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Degenerate2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0.0740740597, -0.314814836), 4829, 10824},  //
      {glm::vec2(0.0925925896, -0.314814806), 4828, 10829},  //
      {glm::vec2(0.0925925896, -0.314814806), 4826, 10826},  //
      {glm::vec2(0.0740740597, -0.314814836), 4830, 25738},  //
  });
  TestPoly(polys, 2);
}

TEST(Polygon, Degenerate3) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-0.413580239, -0.216049403), 49696, 40423},   //
      {glm::vec2(-0.42592591, -0.216049403), 49690, 40337},    //
      {glm::vec2(-0.413580239, -0.216049403), 49694, 40389},   //
      {glm::vec2(-0.401234567, -0.216049403), 49713, 40423},   //
      {glm::vec2(-0.401234567, -0.216049403), 49715, 200641},  //
      {glm::vec2(-0.401234567, -0.216049403), 49716, 200638},  //
      {glm::vec2(-0.413580239, -0.216049403), 49697, 200639},  //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Degenerate4) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0, 10), 213, 489},                      //
      {glm::vec2(-0.0696326792, 9.99390793), 360, 375},  //
      {glm::vec2(4.37113897e-07, 10), 276, 195},         //
      {glm::vec2(0.636729717, 9.94429302), 340, 436},    //
  });
  TestPoly(polys, 2);
}

TEST(Polygon, Degenerate5) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(1, 0), 3, 5},    //
      {glm::vec2(1, 1), 7, 10},   //
      {glm::vec2(1, 1), 15, 17},  //
      {glm::vec2(1, 1), 23, 16},  //
      {glm::vec2(1, 1), 21, 12},  //
      {glm::vec2(0, 1), 22, 14},  //
      {glm::vec2(0, 1), 14, 10},  //
      {glm::vec2(0, 1), 6, 3},    //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, Degenerate6) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(4.37113897e-07, -4.37113897e-07), 0, 1},  //
      {glm::vec2(4.37113897e-07, 0), 18, 10},              //
      {glm::vec2(0, 0), 23, 23},                           //
      {glm::vec2(-1.19421679e-06, 0), 25, 14},             //
      {glm::vec2(-8.66025352, 0), 24, 24},                 //
      {glm::vec2(-8.66025352, 1.339746), 19, 7},           //
      {glm::vec2(-10, -4.37113897e-07), 3, 2},             //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Tricky) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(1, 0), 0, Edge::kNoIdx},  //
      {glm::vec2(2, 1), 1, Edge::kNoIdx},  //
      {glm::vec2(3, 0), 2, Edge::kNoIdx},  //
      {glm::vec2(3, 5), 3, Edge::kNoIdx},  //
      {glm::vec2(2, 5), 4, Edge::kNoIdx},  //
      {glm::vec2(3, 4), 5, Edge::kNoIdx},  //
      {glm::vec2(3, 2), 6, Edge::kNoIdx},  //
      {glm::vec2(3, 3), 7, Edge::kNoIdx},  //
      {glm::vec2(0, 6), 8, Edge::kNoIdx},  //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Tricky2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(1, 0), 0, Edge::kNoIdx},    //
      {glm::vec2(3, 1), 1, Edge::kNoIdx},    //
      {glm::vec2(3, 3.5), 9, Edge::kNoIdx},  //
      {glm::vec2(3, 0), 2, Edge::kNoIdx},    //
      {glm::vec2(3, 5), 3, Edge::kNoIdx},    //
      {glm::vec2(2, 5), 4, Edge::kNoIdx},    //
      {glm::vec2(3, 4), 5, Edge::kNoIdx},    //
      {glm::vec2(3, 2), 6, Edge::kNoIdx},    //
      {glm::vec2(3, 3), 7, Edge::kNoIdx},    //
      {glm::vec2(0, 6), 8, Edge::kNoIdx},    //
  });
  TestPoly(polys, 8);
}

TEST(Polygon, Slit) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(27.7069321, 13.5144091), 286, 467},   //
      {glm::vec2(28.664566, 10.8102894), 267, 604},    //
      {glm::vec2(28.7565536, 10.8183374), 266, 477},   //
      {glm::vec2(25.9535275, 19.4451675), 870, 1257},  //
      {glm::vec2(26.0820198, 18.9281673), 865, 1256},  //
      {glm::vec2(26.0820198, 18.9281673), 864, 1255},  //
      {glm::vec2(26.0820198, 18.9281673), 866, 1272},  //
      {glm::vec2(25.8192234, 18.8448315), 867, 467},   //
      {glm::vec2(27.7069321, 13.5144091), 285, 466},   //
      {glm::vec2(27.9789181, 13.2116556), 284, 468},   //
  });
  polys.push_back({
      {glm::vec2(25.6960907, 20.2374783), 891, 477},   //
      {glm::vec2(25.6563644, 20.3597412), 892, 1268},  //
      {glm::vec2(25.6467285, 20.3614731), 893, 1267},  //
  });
  TestPoly(polys, 9);
}

TEST(Polygon, SharedEdge) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0.265432119, 0.0061728349), 61185, 11383},     //
      {glm::vec2(0.277777791, -3.7252903e-09), 61180, 235729},  //
      {glm::vec2(0.277777791, 0.0185185187), 61184, 49810},     //
      {glm::vec2(0.240740761, 0.0185185187), 76345, 11383},     //
      {glm::vec2(0.265432119, 0.00617283955), 61186, 235731},   //
      {glm::vec2(0.265432119, 0.00617283955), 61187, 235730},   //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Comb) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0.462962955, -0.427297652), 18, 37},       //
      {glm::vec2(0.5, -0.42592591), 17, 196},               //
      {glm::vec2(-0.5, -0.42592591), 1283, 228},            //
      {glm::vec2(-0.5, -0.462962955), 1269, 37},            //
      {glm::vec2(-0.462962985, -0.461591214), 1268, 676},   //
      {glm::vec2(-0.462962985, -0.42592591), 1282, 679},    //
      {glm::vec2(-0.42592594, -0.42592591), 1280, 272},     //
      {glm::vec2(-0.42592594, -0.460219473), 1266, 37},     //
      {glm::vec2(-0.351851881, -0.45747599), 1247, 672},    //
      {glm::vec2(-0.351851881, -0.42592591), 1257, 673},    //
      {glm::vec2(-0.314814836, -0.42592591), 1256, 671},    //
      {glm::vec2(-0.314814836, -0.456104249), 1245, 37},    //
      {glm::vec2(-0.240740746, -0.453360766), 1132, 639},   //
      {glm::vec2(-0.240740746, -0.42592591), 1143, 642},    //
      {glm::vec2(-0.203703716, -0.42592591), 1142, 637},    //
      {glm::vec2(-0.203703716, -0.451989025), 1130, 37},    //
      {glm::vec2(-0.129629642, -0.449245542), 1128, 636},   //
      {glm::vec2(-0.129629642, -0.42592591), 1141, 635},    //
      {glm::vec2(-0.092592597, -0.42592591), 1119, 629},    //
      {glm::vec2(-0.092592597, -0.447873801), 1111, 37},    //
      {glm::vec2(-0.0185185187, -0.445130289), 1109, 628},  //
      {glm::vec2(-0.0185185187, -0.42592591), 1117, 632},   //
      {glm::vec2(0.0185185187, -0.42592591), 169, 451},     //
      {glm::vec2(0.0185185187, -0.443758547), 162, 37},     //
      {glm::vec2(0.0925925896, -0.441015065), 160, 449},    //
      {glm::vec2(0.0925925896, -0.42592591), 168, 452},     //
      {glm::vec2(0.129629627, -0.42592591), 152, 443},      //
      {glm::vec2(0.129629627, -0.439643323), 141, 37},      //
      {glm::vec2(0.203703731, -0.436899841), 150, 441},     //
      {glm::vec2(0.203703731, -0.42592591), 151, 445},      //
      {glm::vec2(0.240740761, -0.42592591), 149, 440},      //
      {glm::vec2(0.240740761, -0.4355281), 148, 37},        //
      {glm::vec2(0.314814806, -0.432784617), 40, 401},      //
      {glm::vec2(0.314814806, -0.42592591), 41, 402},       //
      {glm::vec2(0.351851851, -0.42592591), 39, 399},       //
      {glm::vec2(0.351851851, -0.431412876), 38, 37},       //
      {glm::vec2(0.42592591, -0.428669393), 21, 390},       //
      {glm::vec2(0.42592591, -0.42592591), 22, 394},        //
      {glm::vec2(0.462962955, -0.42592591), 19, 389},       //
  });
  TestPoly(polys, 37);
}

TEST(Polygon, Comb2) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-0.5, -0.462962955), 2, 193},             //
      {glm::vec2(-0.462962955, -0.461591214), 4, 389},     //
      {glm::vec2(-0.462962955, -0.42592591), 13, 394},     //
      {glm::vec2(-0.42592591, -0.42592591), 15, 390},      //
      {glm::vec2(-0.42592591, -0.460219473), 7, 193},      //
      {glm::vec2(-0.351851851, -0.45747599), 28, 399},     //
      {glm::vec2(-0.351851851, -0.42592591), 35, 402},     //
      {glm::vec2(-0.314814806, -0.42592591), 36, 401},     //
      {glm::vec2(-0.314814806, -0.456104249), 30, 193},    //
      {glm::vec2(-0.240740761, -0.453360766), 132, 440},   //
      {glm::vec2(-0.240740761, -0.42592591), 144, 445},    //
      {glm::vec2(-0.203703731, -0.42592591), 145, 441},    //
      {glm::vec2(-0.203703731, -0.451989025), 134, 193},   //
      {glm::vec2(-0.129629627, -0.449245542), 137, 443},   //
      {glm::vec2(-0.129629627, -0.42592591), 147, 452},    //
      {glm::vec2(-0.0925925896, -0.42592591), 164, 449},   //
      {glm::vec2(-0.0925925896, -0.447873801), 155, 193},  //
      {glm::vec2(-0.0185185187, -0.445130289), 158, 451},  //
      {glm::vec2(-0.0185185187, -0.42592591), 166, 632},   //
      {glm::vec2(0.0185185187, -0.42592591), 1113, 628},   //
      {glm::vec2(0.0185185187, -0.443758547), 1104, 193},  //
      {glm::vec2(0.092592597, -0.441015065), 1107, 629},   //
      {glm::vec2(0.092592597, -0.42592591), 1116, 635},    //
      {glm::vec2(0.129629642, -0.42592591), 1134, 636},    //
      {glm::vec2(0.129629642, -0.439643323), 1122, 193},   //
      {glm::vec2(0.203703716, -0.436899841), 1137, 637},   //
      {glm::vec2(0.203703716, -0.42592591), 1138, 642},    //
      {glm::vec2(0.240740746, -0.42592591), 1140, 639},    //
      {glm::vec2(0.240740746, -0.4355281), 1139, 193},     //
      {glm::vec2(0.314814836, -0.432784617), 1251, 671},   //
      {glm::vec2(0.314814836, -0.42592591), 1252, 673},    //
      {glm::vec2(0.351851881, -0.42592591), 1254, 672},    //
      {glm::vec2(0.351851881, -0.431412876), 1253, 193},   //
      {glm::vec2(0.42592594, -0.428669393), 1272, 272},    //
      {glm::vec2(0.42592594, -0.42592591), 1273, 679},     //
      {glm::vec2(0.462962985, -0.42592591), 1278, 676},    //
      {glm::vec2(0.462962985, -0.427297652), 1277, 193},   //
      {glm::vec2(0.5, -0.42592591), 1279, 36},             //
      {glm::vec2(-0.5, -0.42592591), 12, 1},               //
  });
  TestPoly(polys, 37);
}

TEST(Polygon, PointPoly) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0, -15.9780979), 32, 1405},            //
      {glm::vec2(5.08144999, -14.2678728), 244, 468},   //
      {glm::vec2(4.83870935, -14.0789623), 243, 461},   //
      {glm::vec2(4.38336992, -13.7492008), 237, 458},   //
      {glm::vec2(4.35483837, -13.7284746), 238, 459},   //
      {glm::vec2(4.33122683, -13.712779), 235, 447},    //
      {glm::vec2(3.87096763, -13.3689337), 230, 446},   //
      {glm::vec2(3.52637458, -13.1333551), 81, 137},    //
      {glm::vec2(3.38709664, -13.0251188), 79, 131},    //
      {glm::vec2(3.10755324, -12.8263216), 75, 129},    //
      {glm::vec2(2.90322566, -12.6806841), 73, 128},    //
      {glm::vec2(2.80962205, -12.6208401), 71, 123},    //
      {glm::vec2(2.41935468, -12.3280048), 69, 120},    //
      {glm::vec2(2.16151524, -12.1544657), 68, 119},    //
      {glm::vec2(1.93548381, -11.9734631), 86, 73},     //
      {glm::vec2(1.56781006, -11.7093639), 47, 71},     //
      {glm::vec2(1.45161283, -11.6084995), 46, 68},     //
      {glm::vec2(1.02412188, -11.2756453), 43, 67},     //
      {glm::vec2(0.967741907, -11.2216129), 44, 64},    //
      {glm::vec2(0.628127813, -10.9296618), 40, 1546},  //
      {glm::vec2(0, -10.9296618), 33, 277},             //
  });
  polys.push_back({
      {glm::vec2(15, -10.9296618), 1052, 1405},  //
      {glm::vec2(15, -10.9296618), 1051, 1546},  //
      {glm::vec2(15, -10.9296618), 1053, 1898},  //
  });
  TestPoly(polys, 20);
};

TEST(Polygon, KissingZigzag) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(37.4667358, 0), 0, -1},              //
      {glm::vec2(27.8904877, -3.04520559), 1, -1},    //
      {glm::vec2(27.8904877, 3.04520559), 2, -1},     //
      {glm::vec2(37.4667358, 0), 3, -1},              //
      {glm::vec2(36.4568138, 8.64042377), 4, -1},     //
      {glm::vec2(27.8409691, 3.46886754), 5, -1},     //
      {glm::vec2(26.4364243, 9.39511108), 6, -1},     //
      {glm::vec2(36.4568138, 8.64042377), 7, -1},     //
      {glm::vec2(33.4814987, 16.8150406), 8, -1},     //
      {glm::vec2(26.2905369, 9.79593277), 9, -1},     //
      {glm::vec2(23.5571651, 15.2385235), 10, -1},    //
      {glm::vec2(33.4814987, 16.8150406), 11, -1},    //
      {glm::vec2(28.7011852, 24.0831528), 12, -1},    //
      {glm::vec2(23.322773, 15.5948954), 13, -1},     //
      {glm::vec2(19.4079323, 20.2604218), 14, -1},    //
      {glm::vec2(28.7011852, 24.0831528), 15, -1},    //
      {glm::vec2(22.3735847, 30.0529385), 16, -1},    //
      {glm::vec2(19.0976753, 20.5531368), 17, -1},    //
      {glm::vec2(14.2124147, 24.1900768), 18, -1},    //
      {glm::vec2(22.3735847, 30.0529385), 19, -1},    //
      {glm::vec2(14.8398142, 34.4025612), 20, -1},    //
      {glm::vec2(13.8430138, 24.4033508), 21, -1},    //
      {glm::vec2(8.2507, 26.8156395), 22, -1},        //
      {glm::vec2(14.8398142, 34.4025612), 23, -1},    //
      {glm::vec2(6.50603199, 36.8975296), 24, -1},    //
      {glm::vec2(7.84207535, 26.937973), 25, -1},     //
      {glm::vec2(1.84419155, 27.9955635), 26, -1},    //
      {glm::vec2(6.50603199, 36.8975296), 27, -1},    //
      {glm::vec2(-2.1784966, 37.4033508), 28, -1},    //
      {glm::vec2(1.41836619, 28.0203648), 29, -1},    //
      {glm::vec2(-4.66174126, 27.6662388), 30, -1},   //
      {glm::vec2(-2.1784966, 37.4033508), 31, -1},    //
      {glm::vec2(-10.7455816, 35.8927383), 32, -1},   //
      {glm::vec2(-5.08180761, 27.5921688), 33, -1},   //
      {glm::vec2(-10.916357, 25.8454189), 34, -1},    //
      {glm::vec2(-10.7455816, 35.8927383), 35, -1},   //
      {glm::vec2(-18.7333698, 32.4471436), 36, -1},   //
      {glm::vec2(-11.3080206, 25.6764717), 37, -1},   //
      {glm::vec2(-16.5824718, 22.6312675), 38, -1},   //
      {glm::vec2(-18.7333698, 32.4471436), 39, -1},   //
      {glm::vec2(-25.711235, 27.2523136), 40, -1},    //
      {glm::vec2(-16.924614, 22.3765507), 41, -1},    //
      {glm::vec2(-21.3546181, 18.1970577), 42, -1},   //
      {glm::vec2(-25.711235, 27.2523136), 43, -1},    //
      {glm::vec2(-31.3030052, 20.5883045), 44, -1},   //
      {glm::vec2(-21.6287994, 17.8703041), 45, -1},   //
      {glm::vec2(-24.9755325, 12.7818384), 46, -1},   //
      {glm::vec2(-31.3030052, 20.5883045), 47, -1},   //
      {glm::vec2(-35.2072144, 12.8143806), 48, -1},   //
      {glm::vec2(-25.1669636, 12.4006672), 49, -1},   //
      {glm::vec2(-27.2500057, 6.67755318), 50, -1},   //
      {glm::vec2(-35.2072144, 12.8143806), 51, -1},   //
      {glm::vec2(-37.213398, 4.34962463), 52, -1},    //
      {glm::vec2(-27.3483734, 6.26250458), 53, -1},   //
      {glm::vec2(-28.0554276, 0.213274717), 54, -1},  //
      {glm::vec2(-37.213398, 4.34962463), 55, -1},    //
      {glm::vec2(-37.213398, -4.34962177), 56, -1},   //
      {glm::vec2(-28.0554276, -0.21327281), 57, -1},  //
      {glm::vec2(-27.3483734, -6.26250267), 58, -1},  //
      {glm::vec2(-37.213398, -4.34962177), 59, -1},   //
      {glm::vec2(-35.2072144, -12.8143787), 60, -1},  //
      {glm::vec2(-27.2500057, -6.67755222), 61, -1},  //
      {glm::vec2(-25.1669636, -12.4006662), 62, -1},  //
      {glm::vec2(-35.2072144, -12.8143787), 63, -1},  //
      {glm::vec2(-31.3029995, -20.5883102), 64, -1},  //
      {glm::vec2(-24.9755306, -12.7818432), 65, -1},  //
      {glm::vec2(-21.6287937, -17.8703079), 66, -1},  //
      {glm::vec2(-31.3029995, -20.5883102), 67, -1},  //
      {glm::vec2(-25.7112312, -27.2523193), 68, -1},  //
      {glm::vec2(-21.3546143, -18.1970615), 69, -1},  //
      {glm::vec2(-16.9246101, -22.3765545), 70, -1},  //
      {glm::vec2(-25.7112312, -27.2523193), 71, -1},  //
      {glm::vec2(-18.7333641, -32.4471474), 72, -1},  //
      {glm::vec2(-16.5824661, -22.6312695), 73, -1},  //
      {glm::vec2(-11.3080158, -25.6764736), 74, -1},  //
      {glm::vec2(-18.7333641, -32.4471474), 75, -1},  //
      {glm::vec2(-10.7455835, -35.8927383), 76, -1},  //
      {glm::vec2(-10.9163589, -25.8454189), 77, -1},  //
      {glm::vec2(-5.08180904, -27.5921688), 78, -1},  //
      {glm::vec2(-10.7455835, -35.8927383), 79, -1},  //
      {glm::vec2(-2.17849016, -37.4033508), 80, -1},  //
      {glm::vec2(-4.66173601, -27.6662388), 81, -1},  //
      {glm::vec2(1.41837108, -28.0203648), 82, -1},   //
      {glm::vec2(-2.17849016, -37.4033508), 83, -1},  //
      {glm::vec2(6.50602913, -36.8975296), 84, -1},   //
      {glm::vec2(1.84418964, -27.9955635), 85, -1},   //
      {glm::vec2(7.84207344, -26.937973), 86, -1},    //
      {glm::vec2(6.50602913, -36.8975296), 87, -1},   //
      {glm::vec2(14.8398247, -34.4025574), 88, -1},   //
      {glm::vec2(8.25070763, -26.8156357), 89, -1},   //
      {glm::vec2(13.8430195, -24.403347), 90, -1},    //
      {glm::vec2(14.8398247, -34.4025574), 91, -1},   //
      {glm::vec2(22.3735847, -30.0529385), 92, -1},   //
      {glm::vec2(14.2124147, -24.1900768), 93, -1},   //
      {glm::vec2(19.0976753, -20.5531368), 94, -1},   //
      {glm::vec2(22.3735847, -30.0529385), 95, -1},   //
      {glm::vec2(28.7011795, -24.0831585), 96, -1},   //
      {glm::vec2(19.4079285, -20.2604256), 97, -1},   //
      {glm::vec2(23.3227692, -15.5949011), 98, -1},   //
      {glm::vec2(28.7011795, -24.0831585), 99, -1},   //
      {glm::vec2(33.4815025, -16.8150368), 100, -1},  //
      {glm::vec2(23.5571671, -15.2385206), 101, -1},  //
      {glm::vec2(26.2905388, -9.79592991), 102, -1},  //
      {glm::vec2(33.4815025, -16.8150368), 103, -1},  //
      {glm::vec2(36.4568138, -8.64042759), 104, -1},  //
      {glm::vec2(26.4364243, -9.39511299), 105, -1},  //
      {glm::vec2(27.8409691, -3.46886992), 106, -1},  //
      {glm::vec2(36.4568138, -8.64042759), 107, -1},  //
  });
  TestPoly(polys, 106);
}

TEST(Polygon, Sponge) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(-0.5, -0.5), 22, 50},                  //
      {glm::vec2(-0.388888896, -0.388888896), 23, 1},   //
      {glm::vec2(-0.388888896, -0.388888896), 24, 2},   //
      {glm::vec2(-0.388888896, -0.277777791), 26, 5},   //
      {glm::vec2(-0.388888896, -0.277777791), 27, 7},   //
      {glm::vec2(-0.277777791, -0.277777791), 30, 50},  //
      {glm::vec2(-0.166666657, -0.166666672), 41, 19},  //
      {glm::vec2(-0.166666672, -0.166666672), 42, 18},  //
      {glm::vec2(-0.166666672, -0.166666672), 43, 36},  //
      {glm::vec2(-0.166666672, 0.166666672), 76, 21},   //
      {glm::vec2(-0.166666672, 0.166666672), 77, 65},   //
      {glm::vec2(0.166666657, 0.166666672), 142, 50},   //
      {glm::vec2(0.277777791, 0.277777791), 153, 77},   //
      {glm::vec2(0.277777791, 0.277777791), 154, 76},   //
      {glm::vec2(0.277777791, 0.277777791), 155, 78},   //
      {glm::vec2(0.277777791, 0.388888896), 156, 80},   //
      {glm::vec2(0.277777791, 0.388888896), 157, 81},   //
      {glm::vec2(0.388888896, 0.388888896), 161, 83},   //
      {glm::vec2(0.388888896, 0.388888896), 160, 50},   //
      {glm::vec2(0.5, 0.5), 163, 71},                   //
      {glm::vec2(-0.5, 0.5), 69, 24},                   //
  });
  polys.push_back({
      {glm::vec2(-0.388888896, -0.055555556), 33, 9},   //
      {glm::vec2(-0.388888896, -0.055555556), 34, 23},  //
      {glm::vec2(-0.388888896, 0.055555556), 63, 25},   //
      {glm::vec2(-0.388888896, 0.055555556), 64, 26},   //
      {glm::vec2(-0.277777791, 0.055555556), 66, 27},   //
      {glm::vec2(-0.277777791, 0.055555556), 65, 12},   //
      {glm::vec2(-0.277777791, -0.055555556), 35, 11},  //
      {glm::vec2(-0.277777791, -0.055555556), 36, 10},  //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, 0.277777791), 72, 31},  //
      {glm::vec2(-0.277777791, 0.277777791), 73, 30},  //
      {glm::vec2(-0.388888896, 0.277777791), 67, 28},  //
      {glm::vec2(-0.388888896, 0.277777791), 68, 29},  //
      {glm::vec2(-0.388888896, 0.388888896), 70, 34},  //
      {glm::vec2(-0.388888896, 0.388888896), 71, 35},  //
      {glm::vec2(-0.277777791, 0.388888896), 74, 33},  //
      {glm::vec2(-0.277777791, 0.388888896), 75, 32},  //
  });
  polys.push_back({
      {glm::vec2(-0.055555556, 0.277777791), 78, 38},  //
      {glm::vec2(-0.055555556, 0.277777791), 79, 40},  //
      {glm::vec2(-0.055555556, 0.388888896), 80, 41},  //
      {glm::vec2(-0.055555556, 0.388888896), 81, 70},  //
      {glm::vec2(0.055555556, 0.388888896), 147, 69},  //
      {glm::vec2(0.055555556, 0.388888896), 148, 68},  //
      {glm::vec2(0.055555556, 0.277777791), 146, 67},  //
      {glm::vec2(0.055555556, 0.277777791), 145, 39},  //
  });
  TestPoly(polys, 49);
}

TEST(Polygon, SquareHoles) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0.388888896, -0.277777791), 10, 0},  //
      {glm::vec2(0.388888896, -0.388888896), 8, 1},   //
      {glm::vec2(0.277777791, -0.388888896), 9, 3},   //
      {glm::vec2(0.277777791, -0.277777791), 11, 2},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, -0.055555556), 14, 4},  //
      {glm::vec2(0.277777791, -0.055555556), 12, 9},  //
      {glm::vec2(0.277777791, 0.055555556), 16, 8},   //
      {glm::vec2(0.388888896, 0.055555556), 15, 7},   //
  });
  polys.push_back({
      {glm::vec2(0.055555556, -0.277777791), 25, 5},    //
      {glm::vec2(0.055555556, -0.388888896), 13, 17},   //
      {glm::vec2(-0.055555556, -0.388888896), 23, 18},  //
      {glm::vec2(-0.055555556, -0.277777791), 24, 19},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, 0.277777791), 18, 11},  //
      {glm::vec2(0.277777791, 0.277777791), 17, 12},  //
      {glm::vec2(0.277777791, 0.388888896), 20, 13},  //
      {glm::vec2(0.388888896, 0.388888896), 19, 10},  //
  });
  polys.push_back({
      {glm::vec2(0.166666672, 0.166666672), 32, 14},    //
      {glm::vec2(0.166666672, -0.166666672), 21, 20},   //
      {glm::vec2(-0.166666672, -0.166666672), 26, 28},  //
      {glm::vec2(-0.166666672, 0.166666672), 33, 27},   //
  });
  polys.push_back({
      {glm::vec2(0.055555556, 0.388888896), 35, 15},   //
      {glm::vec2(0.055555556, 0.277777791), 22, 30},   //
      {glm::vec2(-0.055555556, 0.277777791), 34, 31},  //
      {glm::vec2(-0.055555556, 0.388888896), 36, 32},  //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, -0.277777791), 30, 22},  //
      {glm::vec2(-0.277777791, -0.388888896), 27, 21},  //
      {glm::vec2(-0.388888896, -0.388888896), 28, 24},  //
      {glm::vec2(-0.388888896, -0.277777791), 29, 23},  //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, -0.055555556), 37, 25},  //
      {glm::vec2(-0.388888896, -0.055555556), 31, 36},  //
      {glm::vec2(-0.388888896, 0.055555556), 39, 35},   //
      {glm::vec2(-0.277777791, 0.055555556), 38, 34},   //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, 0.388888896), 42, 38},  //
      {glm::vec2(-0.277777791, 0.277777791), 40, 39},  //
      {glm::vec2(-0.388888896, 0.277777791), 41, 41},  //
      {glm::vec2(-0.388888896, 0.388888896), 43, 40},  //
  });
  polys.push_back({
      {glm::vec2(-0.5, -0.5), 1, 16},  //
      {glm::vec2(0.5, -0.5), 0, 6},    //
      {glm::vec2(0.5, 0.5), 3, 33},    //
      {glm::vec2(-0.5, 0.5), 2, 37},   //
  });
  TestPoly(polys, 56);
}

TEST(Polygon, BigSponge) {
  Polygons polys;
  polys.push_back({
      {glm::vec2(0.5, 0.5), 1, 8596},                         //
      {glm::vec2(0.487654328, 0.487654328), 13834, 8314},     //
      {glm::vec2(0.487654328, 0.487654328), 83869, 11104},    //
      {glm::vec2(0.487654328, 0.475308657), 83868, 11106},    //
      {glm::vec2(0.475308657, 0.475308657), 83870, 8310},     //
      {glm::vec2(0.475308657, 0.475308657), 13835, 8596},     //
      {glm::vec2(0.462962955, 0.462962955), 13833, 8303},     //
      {glm::vec2(0.462962955, 0.42592594), 83857, 8274},      //
      {glm::vec2(0.42592594, 0.42592594), 13830, 8596},       //
      {glm::vec2(0.413580239, 0.413580239), 13832, 8275},     //
      {glm::vec2(0.413580239, 0.413580239), 83861, 11066},    //
      {glm::vec2(0.413580239, 0.401234567), 83862, 11065},    //
      {glm::vec2(0.401234567, 0.401234567), 83863, 8276},     //
      {glm::vec2(0.401234567, 0.401234567), 13831, 8596},     //
      {glm::vec2(0.388888896, 0.388888896), 13836, 8315},     //
      {glm::vec2(0.388888896, 0.388888896), 83826, 10930},    //
      {glm::vec2(0.388888896, 0.277777791), 83834, 10955},    //
      {glm::vec2(0.277777791, 0.277777791), 83842, 8179},     //
      {glm::vec2(0.277777791, 0.277777791), 13829, 8596},     //
      {glm::vec2(0.265432119, 0.265432119), 13827, 8177},     //
      {glm::vec2(0.265432119, 0.265432119), 83840, 10961},    //
      {glm::vec2(0.265432119, 0.253086448), 83839, 10960},    //
      {glm::vec2(0.253086448, 0.253086448), 83841, 8178},     //
      {glm::vec2(0.253086448, 0.253086448), 13828, 8596},     //
      {glm::vec2(0.240740746, 0.240740746), 13824, 7658},     //
      {glm::vec2(0.240740761, 0.240740746), 83783, 10340},    //
      {glm::vec2(0.240740761, 0.203703716), 83781, 10339},    //
      {glm::vec2(0.203703731, 0.203703716), 83784, 7659},     //
      {glm::vec2(0.203703731, 0.203703746), 13825, 8596},     //
      {glm::vec2(0.19135803, 0.19135803), 13823, 7661},       //
      {glm::vec2(0.19135803, 0.19135803), 83782, 10326},      //
      {glm::vec2(0.19135803, 0.179012358), 83776, 10330},     //
      {glm::vec2(0.179012358, 0.179012358), 83785, 7653},     //
      {glm::vec2(0.179012358, 0.179012358), 13826, 8596},     //
      {glm::vec2(0.166666672, 0.166666657), 13822, 7614},     //
      {glm::vec2(0.166666672, -0.166666672), 83546, 6140},    //
      {glm::vec2(-0.166666672, -0.166666672), 83871, 11975},  //
      {glm::vec2(-0.166666672, -0.166666657), 13837, 8596},   //
      {glm::vec2(-0.179012358, -0.179012358), 13818, 2746},   //
      {glm::vec2(-0.179012358, -0.19135803), 83565, 2739},    //
      {glm::vec2(-0.19135803, -0.19135803), 13820, 8596},     //
      {glm::vec2(-0.203703731, -0.203703731), 13821, 2741},   //
      {glm::vec2(-0.203703716, -0.203703731), 83562, 6164},   //
      {glm::vec2(-0.203703716, -0.240740761), 83563, 6165},   //
      {glm::vec2(-0.240740746, -0.240740761), 83564, 2740},   //
      {glm::vec2(-0.240740746, -0.240740731), 13819, 8596},   //
      {glm::vec2(-0.253086448, -0.253086448), 13816, 2191},   //
      {glm::vec2(-0.253086448, -0.265432119), 83519, 2192},   //
      {glm::vec2(-0.265432119, -0.265432119), 13817, 8596},   //
      {glm::vec2(-0.277777791, -0.277777791), 13815, 2190},   //
      {glm::vec2(-0.277777791, -0.388888896), 83485, 2057},   //
      {glm::vec2(-0.388888896, -0.388888896), 13808, 8596},   //
      {glm::vec2(-0.401234567, -0.401234567), 13814, 2120},   //
      {glm::vec2(-0.401234567, -0.413580239), 83509, 2121},   //
      {glm::vec2(-0.413580239, -0.413580239), 13813, 8596},   //
      {glm::vec2(-0.42592594, -0.42592594), 13809, 2072},     //
      {glm::vec2(-0.42592594, -0.462962955), 83504, 2089},    //
      {glm::vec2(-0.462962955, -0.462962955), 13810, 8596},   //
      {glm::vec2(-0.475308657, -0.475308657), 13811, 2081},   //
      {glm::vec2(-0.475308657, -0.487654328), 83505, 2078},   //
      {glm::vec2(-0.487654328, -0.487654328), 13812, 8596},   //
      {glm::vec2(-0.5, -0.5), 4, 615},                        //
      {glm::vec2(0.5, -0.5), 0, 822},                         //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.487654328), 82702, 2927},  //
      {glm::vec2(0.475308657, -0.487654328), 82704, 2933},  //
      {glm::vec2(0.475308657, -0.475308657), 82705, 2934},  //
      {glm::vec2(0.487654328, -0.475308657), 82703, 2930},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.475308657), 82706, 2938},  //
      {glm::vec2(0.450617284, -0.487654328), 82707, 2937},  //
      {glm::vec2(0.438271612, -0.487654328), 82709, 2939},  //
      {glm::vec2(0.438271612, -0.475308657), 82708, 2940},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.462962955), 82710, 2952},  //
      {glm::vec2(0.42592591, -0.462962955), 82723, 2962},   //
      {glm::vec2(0.42592591, -0.42592591), 82735, 2988},    //
      {glm::vec2(0.462962955, -0.42592591), 82714, 2953},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.450617284), 82711, 2947},  //
      {glm::vec2(0.475308657, -0.450617284), 82713, 2949},  //
      {glm::vec2(0.475308657, -0.438271612), 82715, 2948},  //
      {glm::vec2(0.487654328, -0.438271612), 82712, 2946},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.475308657), 82716, 2958},  //
      {glm::vec2(0.413580239, -0.487654328), 82717, 2957},  //
      {glm::vec2(0.401234567, -0.487654328), 82718, 2960},  //
      {glm::vec2(0.401234567, -0.475308657), 82719, 2959},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.475308657), 82720, 2961},  //
      {glm::vec2(0.376543224, -0.487654328), 82721, 3044},  //
      {glm::vec2(0.364197552, -0.487654328), 82756, 3045},  //
      {glm::vec2(0.364197552, -0.475308657), 82722, 3046},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.438271612), 82724, 2964},  //
      {glm::vec2(0.413580239, -0.450617284), 82725, 2963},  //
      {glm::vec2(0.401234567, -0.450617284), 82726, 2966},  //
      {glm::vec2(0.401234567, -0.438271612), 82727, 2965},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.450617284), 82728, 3051},  //
      {glm::vec2(0.364197552, -0.450617284), 82762, 3053},  //
      {glm::vec2(0.364197552, -0.438271612), 82730, 3052},  //
      {glm::vec2(0.376543224, -0.438271612), 82729, 2967},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.413580239), 82731, 2985},  //
      {glm::vec2(0.475308657, -0.413580239), 82733, 2986},  //
      {glm::vec2(0.475308657, -0.401234567), 82734, 2998},  //
      {glm::vec2(0.487654328, -0.401234567), 82732, 2984},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.413580239), 82736, 2990},  //
      {glm::vec2(0.438271612, -0.413580239), 82739, 2991},  //
      {glm::vec2(0.438271612, -0.401234567), 82738, 3003},  //
      {glm::vec2(0.450617284, -0.401234567), 82737, 2989},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.376543224), 82740, 3002},  //
      {glm::vec2(0.475308657, -0.376543224), 82742, 3165},  //
      {glm::vec2(0.475308657, -0.364197552), 82799, 3166},  //
      {glm::vec2(0.487654328, -0.364197552), 82741, 3164},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.376543224), 82743, 3004},  //
      {glm::vec2(0.438271612, -0.376543224), 82745, 3168},  //
      {glm::vec2(0.438271612, -0.364197552), 82801, 3169},  //
      {glm::vec2(0.450617284, -0.364197552), 82744, 3167},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.413580239), 82746, 3007},  //
      {glm::vec2(0.401234567, -0.413580239), 82748, 3009},  //
      {glm::vec2(0.401234567, -0.401234567), 82749, 3011},  //
      {glm::vec2(0.413580239, -0.401234567), 82747, 3008},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.413580239), 82750, 3073},  //
      {glm::vec2(0.364197552, -0.413580239), 82781, 3074},  //
      {glm::vec2(0.364197552, -0.401234567), 82752, 3079},  //
      {glm::vec2(0.376543224, -0.401234567), 82751, 3010},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.364197552), 82753, 3177},  //
      {glm::vec2(0.413580239, -0.376543224), 82754, 3012},  //
      {glm::vec2(0.401234567, -0.376543224), 82755, 3179},  //
      {glm::vec2(0.401234567, -0.364197552), 82808, 3178},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.475308657), 82757, 3048},  //
      {glm::vec2(0.339506179, -0.487654328), 82758, 3047},  //
      {glm::vec2(0.327160507, -0.487654328), 82759, 3050},  //
      {glm::vec2(0.327160507, -0.475308657), 82760, 3049},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, -0.462962955), 82761, 3055},  //
      {glm::vec2(0.314814806, -0.462962955), 82764, 3056},  //
      {glm::vec2(0.314814806, -0.42592591), 82783, 3075},   //
      {glm::vec2(0.351851851, -0.42592591), 82763, 3054},   //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.475308657), 82765, 3058},  //
      {glm::vec2(0.302469134, -0.487654328), 82766, 3057},  //
      {glm::vec2(0.290123463, -0.487654328), 82767, 3059},  //
      {glm::vec2(0.290123463, -0.475308657), 82768, 3060},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.487654328), 82769, 3061},  //
      {glm::vec2(0.253086448, -0.487654328), 82772, 3063},  //
      {glm::vec2(0.253086448, -0.475308657), 82771, 3064},  //
      {glm::vec2(0.265432119, -0.475308657), 82770, 3062},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.450617284), 82773, 3066},  //
      {glm::vec2(0.290123463, -0.450617284), 82775, 3067},  //
      {glm::vec2(0.290123463, -0.438271612), 82776, 3068},  //
      {glm::vec2(0.302469134, -0.438271612), 82774, 3065},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.450617284), 82777, 3069},  //
      {glm::vec2(0.253086448, -0.450617284), 82779, 3072},  //
      {glm::vec2(0.253086448, -0.438271612), 82780, 3071},  //
      {glm::vec2(0.265432119, -0.438271612), 82778, 3070},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.401234567), 82782, 3076},  //
      {glm::vec2(0.339506179, -0.413580239), 82784, 3077},  //
      {glm::vec2(0.327160507, -0.413580239), 82785, 3078},  //
      {glm::vec2(0.327160507, -0.401234567), 82786, 3080},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, -0.388888896), 82787, 3081},  //
      {glm::vec2(0.277777791, -0.388888896), 82844, 3259},  //
      {glm::vec2(0.277777791, -0.277777791), 82848, 3267},  //
      {glm::vec2(0.388888896, -0.277777791), 82814, 3185},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.413580239), 82788, 3083},  //
      {glm::vec2(0.290123463, -0.413580239), 82790, 3084},  //
      {glm::vec2(0.290123463, -0.401234567), 82791, 3088},  //
      {glm::vec2(0.302469134, -0.401234567), 82789, 3082},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.413580239), 82792, 3085},  //
      {glm::vec2(0.253086448, -0.413580239), 82795, 3087},  //
      {glm::vec2(0.253086448, -0.401234567), 82794, 3089},  //
      {glm::vec2(0.265432119, -0.401234567), 82793, 3086},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.376543224), 82796, 3090},  //
      {glm::vec2(0.253086448, -0.376543224), 82798, 3257},  //
      {glm::vec2(0.253086448, -0.364197552), 82842, 3258},  //
      {glm::vec2(0.265432119, -0.364197552), 82797, 3256},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.339506179), 82800, 3172},  //
      {glm::vec2(0.475308657, -0.339506179), 82804, 3173},  //
      {glm::vec2(0.475308657, -0.327160507), 82805, 3174},  //
      {glm::vec2(0.487654328, -0.327160507), 82803, 3171},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.351851851), 82802, 3170},  //
      {glm::vec2(0.42592591, -0.351851851), 82810, 3180},   //
      {glm::vec2(0.42592591, -0.314814806), 82807, 3176},   //
      {glm::vec2(0.462962955, -0.314814806), 82806, 3175},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.339506179), 82809, 3181},  //
      {glm::vec2(0.401234567, -0.339506179), 82812, 3184},  //
      {glm::vec2(0.401234567, -0.327160507), 82813, 3183},  //
      {glm::vec2(0.413580239, -0.327160507), 82811, 3182},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.302469134), 82815, 3198},  //
      {glm::vec2(0.475308657, -0.302469134), 82818, 3201},  //
      {glm::vec2(0.475308657, -0.290123463), 82817, 3200},  //
      {glm::vec2(0.487654328, -0.290123463), 82816, 3199},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.302469134), 82819, 3202},  //
      {glm::vec2(0.438271612, -0.302469134), 82822, 3204},  //
      {glm::vec2(0.438271612, -0.290123463), 82821, 3205},  //
      {glm::vec2(0.450617284, -0.290123463), 82820, 3203},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.265432119), 82823, 3211},  //
      {glm::vec2(0.475308657, -0.265432119), 82825, 3212},  //
      {glm::vec2(0.475308657, -0.253086448), 82826, 3213},  //
      {glm::vec2(0.487654328, -0.253086448), 82824, 3209},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.253086448), 82827, 3215},  //
      {glm::vec2(0.450617284, -0.265432119), 82828, 3216},  //
      {glm::vec2(0.438271612, -0.265432119), 82829, 3217},  //
      {glm::vec2(0.438271612, -0.253086448), 82830, 3218},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.290123463), 82831, 3223},  //
      {glm::vec2(0.413580239, -0.302469134), 82832, 3222},  //
      {glm::vec2(0.401234567, -0.302469134), 82834, 3225},  //
      {glm::vec2(0.401234567, -0.290123463), 82833, 3224},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.253086448), 82835, 3227},  //
      {glm::vec2(0.413580239, -0.265432119), 82836, 3226},  //
      {glm::vec2(0.401234567, -0.265432119), 82837, 3229},  //
      {glm::vec2(0.401234567, -0.253086448), 82838, 3228},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.253086448), 82839, 3230},  //
      {glm::vec2(0.376543224, -0.265432119), 82840, 3264},  //
      {glm::vec2(0.364197552, -0.265432119), 82849, 3265},  //
      {glm::vec2(0.364197552, -0.253086448), 82841, 3266},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.339506179), 82843, 3260},  //
      {glm::vec2(0.253086448, -0.339506179), 82847, 3262},  //
      {glm::vec2(0.253086448, -0.327160507), 82846, 3263},  //
      {glm::vec2(0.265432119, -0.327160507), 82845, 3261},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.253086448), 82850, 3268},  //
      {glm::vec2(0.339506179, -0.265432119), 82851, 3269},  //
      {glm::vec2(0.327160507, -0.265432119), 82852, 3271},  //
      {glm::vec2(0.327160507, -0.253086448), 82853, 3270},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.302469134), 82854, 3272},  //
      {glm::vec2(0.253086448, -0.302469134), 82857, 3275},  //
      {glm::vec2(0.253086448, -0.290123463), 82856, 3274},  //
      {glm::vec2(0.265432119, -0.290123463), 82855, 3273},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.253086448), 82858, 3276},  //
      {glm::vec2(0.302469134, -0.265432119), 82859, 3277},  //
      {glm::vec2(0.290123463, -0.265432119), 82860, 3278},  //
      {glm::vec2(0.290123463, -0.253086448), 82861, 3279},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.265432119), 82862, 3280},  //
      {glm::vec2(0.253086448, -0.265432119), 82864, 3282},  //
      {glm::vec2(0.253086448, -0.253086448), 82865, 3283},  //
      {glm::vec2(0.265432119, -0.253086448), 82863, 3281},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.487654328), 82866, 3344},  //
      {glm::vec2(0.216049403, -0.487654328), 82868, 3347},  //
      {glm::vec2(0.216049403, -0.475308657), 82869, 3346},  //
      {glm::vec2(0.228395075, -0.475308657), 82867, 3345},  //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.462962955), 82870, 3349},  //
      {glm::vec2(0.203703731, -0.462962955), 82875, 3351},  //
      {glm::vec2(0.203703731, -0.42592591), 82890, 3377},   //
      {glm::vec2(0.240740761, -0.42592591), 82874, 3350},   //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.475308657), 82871, 3348},   //
      {glm::vec2(0.19135803, -0.487654328), 82872, 3353},   //
      {glm::vec2(0.179012358, -0.487654328), 82879, 3355},  //
      {glm::vec2(0.179012358, -0.475308657), 82873, 3356},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.450617284), 82876, 3364},   //
      {glm::vec2(0.179012358, -0.450617284), 82884, 3366},  //
      {glm::vec2(0.179012358, -0.438271612), 82878, 3365},  //
      {glm::vec2(0.19135803, -0.438271612), 82877, 3352},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.475308657), 82880, 3358},  //
      {glm::vec2(0.154320985, -0.487654328), 82881, 3357},  //
      {glm::vec2(0.141975313, -0.487654328), 82882, 3359},  //
      {glm::vec2(0.141975313, -0.475308657), 82883, 3360},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.438271612), 82885, 3368},  //
      {glm::vec2(0.154320985, -0.450617284), 82886, 3367},  //
      {glm::vec2(0.141975313, -0.450617284), 82887, 3369},  //
      {glm::vec2(0.141975313, -0.438271612), 82889, 3370},  //
  });
  polys.push_back({
      {glm::vec2(0.129629627, -0.42592591), 82888, 3371},    //
      {glm::vec2(0.129629627, -0.462962955), 82916, 3410},   //
      {glm::vec2(0.0925925896, -0.462962955), 82921, 3412},  //
      {glm::vec2(0.0925925896, -0.42592591), 82936, 3443},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.413580239), 82891, 3379},  //
      {glm::vec2(0.216049403, -0.413580239), 82893, 3380},  //
      {glm::vec2(0.216049403, -0.401234567), 82894, 3382},  //
      {glm::vec2(0.228395075, -0.401234567), 82892, 3378},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.413580239), 82895, 3384},   //
      {glm::vec2(0.179012358, -0.413580239), 82903, 3385},  //
      {glm::vec2(0.179012358, -0.401234567), 82897, 3391},  //
      {glm::vec2(0.19135803, -0.401234567), 82896, 3381},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.376543224), 82898, 3383},  //
      {glm::vec2(0.216049403, -0.376543224), 82900, 3511},  //
      {glm::vec2(0.216049403, -0.364197552), 82956, 3509},  //
      {glm::vec2(0.228395075, -0.364197552), 82899, 3508},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.376543224), 82901, 3392},   //
      {glm::vec2(0.179012358, -0.376543224), 82908, 3516},  //
      {glm::vec2(0.179012358, -0.364197552), 82958, 3517},  //
      {glm::vec2(0.19135803, -0.364197552), 82902, 3512},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.401234567), 82904, 3387},  //
      {glm::vec2(0.154320985, -0.413580239), 82905, 3388},  //
      {glm::vec2(0.141975313, -0.413580239), 82906, 3389},  //
      {glm::vec2(0.141975313, -0.401234567), 82907, 3394},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.364197552), 82909, 3518},  //
      {glm::vec2(0.154320985, -0.376543224), 82910, 3395},  //
      {glm::vec2(0.141975313, -0.376543224), 82911, 3519},  //
      {glm::vec2(0.141975313, -0.364197552), 82965, 3520},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.475308657), 82912, 3403},  //
      {glm::vec2(0.117283955, -0.487654328), 82913, 3402},  //
      {glm::vec2(0.104938276, -0.487654328), 82915, 3405},  //
      {glm::vec2(0.104938276, -0.475308657), 82914, 3404},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.475308657), 82917, 3407},  //
      {glm::vec2(0.0802469105, -0.487654328), 82918, 3406},  //
      {glm::vec2(0.0679012313, -0.487654328), 82919, 3408},  //
      {glm::vec2(0.0679012313, -0.475308657), 82920, 3409},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.438271612), 82922, 3413},  //
      {glm::vec2(0.0802469105, -0.450617284), 82923, 3414},  //
      {glm::vec2(0.0679012313, -0.450617284), 82924, 3416},  //
      {glm::vec2(0.0679012313, -0.438271612), 82925, 3415},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.475308657), 82926, 3418},  //
      {glm::vec2(0.0432098769, -0.487654328), 82927, 3417},  //
      {glm::vec2(0.0308641978, -0.487654328), 82928, 3421},  //
      {glm::vec2(0.0308641978, -0.475308657), 82929, 3419},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.475308657), 82930, 3422},   //
      {glm::vec2(0.00617283955, -0.487654328), 83272, 4982},   //
      {glm::vec2(-0.00617283955, -0.487654328), 83273, 4984},  //
      {glm::vec2(-0.00617283955, -0.475308657), 83274, 4986},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.450617284), 82931, 3425},  //
      {glm::vec2(0.0308641978, -0.450617284), 82933, 3428},  //
      {glm::vec2(0.0308641978, -0.438271612), 82934, 3427},  //
      {glm::vec2(0.0432098769, -0.438271612), 82932, 3426},  //
  });
  polys.push_back({
      {glm::vec2(0.0185185187, -0.42592591), 82935, 3429},    //
      {glm::vec2(0.0185185187, -0.462962955), 83275, 4992},   //
      {glm::vec2(-0.0185185187, -0.462962955), 83280, 4993},  //
      {glm::vec2(-0.0185185187, -0.42592591), 83299, 5026},   //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.413580239), 82937, 3445},  //
      {glm::vec2(0.104938276, -0.413580239), 82940, 3446},  //
      {glm::vec2(0.104938276, -0.401234567), 82939, 3450},  //
      {glm::vec2(0.117283955, -0.401234567), 82938, 3444},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.413580239), 82941, 3448},  //
      {glm::vec2(0.0679012313, -0.413580239), 82943, 3449},  //
      {glm::vec2(0.0679012313, -0.401234567), 82944, 3452},  //
      {glm::vec2(0.0802469105, -0.401234567), 82942, 3447},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.376543224), 82945, 3451},  //
      {glm::vec2(0.104938276, -0.376543224), 82947, 3563},  //
      {glm::vec2(0.104938276, -0.364197552), 82996, 3562},  //
      {glm::vec2(0.117283955, -0.364197552), 82946, 3561},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.364197552), 82948, 3565},  //
      {glm::vec2(0.0802469105, -0.376543224), 82949, 3453},  //
      {glm::vec2(0.0679012313, -0.376543224), 82950, 3566},  //
      {glm::vec2(0.0679012313, -0.364197552), 82998, 3567},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.413580239), 82951, 3455},  //
      {glm::vec2(0.0308641978, -0.413580239), 82953, 3456},  //
      {glm::vec2(0.0308641978, -0.401234567), 82954, 3460},  //
      {glm::vec2(0.0432098769, -0.401234567), 82952, 3454},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.401234567), 82955, 3458},   //
      {glm::vec2(0.00617283955, -0.413580239), 83300, 5027},   //
      {glm::vec2(-0.00617283955, -0.413580239), 83301, 5028},  //
      {glm::vec2(-0.00617283955, -0.401234567), 83302, 5034},  //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.351851851), 82957, 3510},  //
      {glm::vec2(0.203703731, -0.351851851), 82962, 741},   //
      {glm::vec2(0.203703731, -0.314814806), 82961, 3514},  //
      {glm::vec2(0.240740761, -0.314814806), 82960, 3513},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.339506179), 82959, 3522},   //
      {glm::vec2(0.179012358, -0.339506179), 82967, 3523},  //
      {glm::vec2(0.179012358, -0.327160507), 82964, 3524},  //
      {glm::vec2(0.19135803, -0.327160507), 82963, 3515},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.339506179), 82966, 3525},  //
      {glm::vec2(0.141975313, -0.339506179), 82969, 3527},  //
      {glm::vec2(0.141975313, -0.327160507), 82970, 3529},  //
      {glm::vec2(0.154320985, -0.327160507), 82968, 3526},  //
  });
  polys.push_back({
      {glm::vec2(0.129629627, -0.314814806), 82971, 3528},   //
      {glm::vec2(0.129629627, -0.351851851), 82997, 3564},   //
      {glm::vec2(0.0925925896, -0.351851851), 83000, 3569},  //
      {glm::vec2(0.0925925896, -0.314814806), 83001, 3568},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.302469134), 82972, 3530},  //
      {glm::vec2(0.216049403, -0.302469134), 82975, 3533},  //
      {glm::vec2(0.216049403, -0.290123463), 82974, 3532},  //
      {glm::vec2(0.228395075, -0.290123463), 82973, 3531},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.302469134), 82976, 3540},   //
      {glm::vec2(0.179012358, -0.302469134), 82986, 3542},  //
      {glm::vec2(0.179012358, -0.290123463), 82978, 3541},  //
      {glm::vec2(0.19135803, -0.290123463), 82977, 3534},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.265432119), 82979, 3536},  //
      {glm::vec2(0.216049403, -0.265432119), 82981, 3538},  //
      {glm::vec2(0.216049403, -0.253086448), 82982, 3537},  //
      {glm::vec2(0.228395075, -0.253086448), 82980, 3535},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.253086448), 82983, 3539},   //
      {glm::vec2(0.19135803, -0.265432119), 82984, 3547},   //
      {glm::vec2(0.179012358, -0.265432119), 82991, 3548},  //
      {glm::vec2(0.179012358, -0.253086448), 82985, 3549},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.290123463), 82987, 3544},  //
      {glm::vec2(0.154320985, -0.302469134), 82988, 3543},  //
      {glm::vec2(0.141975313, -0.302469134), 82990, 3545},  //
      {glm::vec2(0.141975313, -0.290123463), 82989, 3546},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.253086448), 82992, 3551},  //
      {glm::vec2(0.154320985, -0.265432119), 82993, 3552},  //
      {glm::vec2(0.141975313, -0.265432119), 82994, 3553},  //
      {glm::vec2(0.141975313, -0.253086448), 82995, 3554},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.339506179), 82999, 3571},  //
      {glm::vec2(0.0679012313, -0.339506179), 83003, 3572},  //
      {glm::vec2(0.0679012313, -0.327160507), 83004, 3573},  //
      {glm::vec2(0.0802469105, -0.327160507), 83002, 3570},  //
  });
  polys.push_back({
      {glm::vec2(0.055555556, -0.277777791), 83005, 3574},   //
      {glm::vec2(0.055555556, -0.388888896), 83307, 5035},   //
      {glm::vec2(-0.055555556, -0.388888896), 83369, 5153},  //
      {glm::vec2(-0.055555556, -0.277777791), 83379, 5180},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.302469134), 83006, 3588},  //
      {glm::vec2(0.104938276, -0.302469134), 83009, 3591},  //
      {glm::vec2(0.104938276, -0.290123463), 83008, 3590},  //
      {glm::vec2(0.117283955, -0.290123463), 83007, 3589},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.290123463), 83010, 3593},  //
      {glm::vec2(0.0802469105, -0.302469134), 83011, 3592},  //
      {glm::vec2(0.0679012313, -0.302469134), 83013, 3595},  //
      {glm::vec2(0.0679012313, -0.290123463), 83012, 3594},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.253086448), 83014, 3596},  //
      {glm::vec2(0.117283955, -0.265432119), 83015, 3597},  //
      {glm::vec2(0.104938276, -0.265432119), 83016, 3599},  //
      {glm::vec2(0.104938276, -0.253086448), 83017, 3598},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.253086448), 83018, 3600},  //
      {glm::vec2(0.0802469105, -0.265432119), 83019, 3601},  //
      {glm::vec2(0.0679012313, -0.265432119), 83020, 3602},  //
      {glm::vec2(0.0679012313, -0.253086448), 83021, 3603},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.253086448), 83022, 3604},  //
      {glm::vec2(0.0432098769, -0.265432119), 83023, 3605},  //
      {glm::vec2(0.0308641978, -0.265432119), 83024, 3607},  //
      {glm::vec2(0.0308641978, -0.253086448), 83025, 3606},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.253086448), 83026, 3609},   //
      {glm::vec2(0.00617283955, -0.265432119), 83380, 5181},   //
      {glm::vec2(-0.00617283955, -0.265432119), 83381, 5182},  //
      {glm::vec2(-0.00617283955, -0.253086448), 83382, 5184},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.228395075), 83027, 4077},  //
      {glm::vec2(0.475308657, -0.228395075), 83029, 4078},  //
      {glm::vec2(0.475308657, -0.216049403), 83030, 4086},  //
      {glm::vec2(0.487654328, -0.216049403), 83028, 4076},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.203703731), 83031, 4081},  //
      {glm::vec2(0.462962955, -0.240740761), 83032, 4080},  //
      {glm::vec2(0.42592591, -0.240740761), 83040, 4098},   //
      {glm::vec2(0.42592591, -0.203703731), 83036, 4092},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.19135803), 83033, 4088},   //
      {glm::vec2(0.475308657, -0.19135803), 83035, 4122},   //
      {glm::vec2(0.475308657, -0.179012358), 83053, 4123},  //
      {glm::vec2(0.487654328, -0.179012358), 83034, 4119},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.19135803), 83037, 4093},   //
      {glm::vec2(0.438271612, -0.19135803), 83039, 4127},   //
      {glm::vec2(0.438271612, -0.179012358), 83055, 4128},  //
      {glm::vec2(0.450617284, -0.179012358), 83038, 4126},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.216049403), 83041, 4100},  //
      {glm::vec2(0.413580239, -0.228395075), 83042, 4099},  //
      {glm::vec2(0.401234567, -0.228395075), 83043, 4101},  //
      {glm::vec2(0.401234567, -0.216049403), 83044, 4103},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.216049403), 83045, 4102},  //
      {glm::vec2(0.376543224, -0.228395075), 83046, 4153},  //
      {glm::vec2(0.364197552, -0.228395075), 83073, 4154},  //
      {glm::vec2(0.364197552, -0.216049403), 83047, 4158},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.179012358), 83048, 4138},  //
      {glm::vec2(0.413580239, -0.19135803), 83049, 4104},   //
      {glm::vec2(0.401234567, -0.19135803), 83050, 4140},   //
      {glm::vec2(0.401234567, -0.179012358), 83064, 4139},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.19135803), 83051, 4159},   //
      {glm::vec2(0.364197552, -0.19135803), 83077, 4172},   //
      {glm::vec2(0.364197552, -0.179012358), 83066, 4173},  //
      {glm::vec2(0.376543224, -0.179012358), 83052, 4141},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.154320985), 83054, 4132},  //
      {glm::vec2(0.475308657, -0.154320985), 83058, 4134},  //
      {glm::vec2(0.475308657, -0.141975313), 83059, 4133},  //
      {glm::vec2(0.487654328, -0.141975313), 83057, 4131},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.154320985), 83056, 4135},  //
      {glm::vec2(0.438271612, -0.154320985), 83062, 1323},  //
      {glm::vec2(0.438271612, -0.141975313), 83061, 4136},  //
      {glm::vec2(0.450617284, -0.141975313), 83060, 1322},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.129629627), 83063, 4137},   //
      {glm::vec2(0.42592591, -0.129629627), 83127, 4276},    //
      {glm::vec2(0.42592591, -0.0925925896), 83118, 4271},   //
      {glm::vec2(0.462962955, -0.0925925896), 83116, 4266},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.154320985), 83065, 4142},  //
      {glm::vec2(0.401234567, -0.154320985), 83069, 4145},  //
      {glm::vec2(0.401234567, -0.141975313), 83070, 4144},  //
      {glm::vec2(0.413580239, -0.141975313), 83068, 4143},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.154320985), 83067, 4177},  //
      {glm::vec2(0.364197552, -0.154320985), 83098, 4179},  //
      {glm::vec2(0.364197552, -0.141975313), 83072, 4178},  //
      {glm::vec2(0.376543224, -0.141975313), 83071, 4146},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, -0.203703731), 83074, 4155},  //
      {glm::vec2(0.351851851, -0.240740761), 83075, 4156},  //
      {glm::vec2(0.314814806, -0.240740761), 83076, 4157},  //
      {glm::vec2(0.314814806, -0.203703731), 83079, 4160},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.179012358), 83078, 4174},  //
      {glm::vec2(0.339506179, -0.19135803), 83080, 4161},   //
      {glm::vec2(0.327160507, -0.19135803), 83081, 4176},   //
      {glm::vec2(0.327160507, -0.179012358), 83096, 4175},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.216049403), 83082, 4162},  //
      {glm::vec2(0.302469134, -0.228395075), 83083, 4163},  //
      {glm::vec2(0.290123463, -0.228395075), 83084, 4164},  //
      {glm::vec2(0.290123463, -0.216049403), 83085, 4168},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.228395075), 83086, 4165},  //
      {glm::vec2(0.253086448, -0.228395075), 83089, 4167},  //
      {glm::vec2(0.253086448, -0.216049403), 83088, 4170},  //
      {glm::vec2(0.265432119, -0.216049403), 83087, 4166},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.19135803), 83090, 4169},   //
      {glm::vec2(0.290123463, -0.19135803), 83092, 4186},   //
      {glm::vec2(0.290123463, -0.179012358), 83103, 4187},  //
      {glm::vec2(0.302469134, -0.179012358), 83091, 4185},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.19135803), 83093, 4171},   //
      {glm::vec2(0.253086448, -0.19135803), 83095, 4189},   //
      {glm::vec2(0.253086448, -0.179012358), 83105, 4190},  //
      {glm::vec2(0.265432119, -0.179012358), 83094, 4188},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.154320985), 83097, 4180},  //
      {glm::vec2(0.327160507, -0.154320985), 83100, 4183},  //
      {glm::vec2(0.327160507, -0.141975313), 83101, 4182},  //
      {glm::vec2(0.339506179, -0.141975313), 83099, 4181},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, -0.129629627), 83102, 4184},   //
      {glm::vec2(0.314814806, -0.129629627), 83159, 1374},   //
      {glm::vec2(0.314814806, -0.0925925896), 83160, 4308},  //
      {glm::vec2(0.351851851, -0.0925925896), 83158, 1373},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.154320985), 83104, 4192},  //
      {glm::vec2(0.290123463, -0.154320985), 83108, 4193},  //
      {glm::vec2(0.290123463, -0.141975313), 83109, 4194},  //
      {glm::vec2(0.302469134, -0.141975313), 83107, 4191},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.154320985), 83106, 4195},  //
      {glm::vec2(0.253086448, -0.154320985), 83112, 4198},  //
      {glm::vec2(0.253086448, -0.141975313), 83111, 4197},  //
      {glm::vec2(0.265432119, -0.141975313), 83110, 4196},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.117283955), 83113, 4263},  //
      {glm::vec2(0.475308657, -0.117283955), 83115, 4264},  //
      {glm::vec2(0.475308657, -0.104938276), 83117, 4265},  //
      {glm::vec2(0.487654328, -0.104938276), 83114, 4262},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.0802469105), 83119, 4267},  //
      {glm::vec2(0.475308657, -0.0802469105), 83121, 4269},  //
      {glm::vec2(0.475308657, -0.0679012313), 83122, 4270},  //
      {glm::vec2(0.487654328, -0.0679012313), 83120, 4268},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.0802469105), 83123, 4272},  //
      {glm::vec2(0.438271612, -0.0802469105), 83126, 4274},  //
      {glm::vec2(0.438271612, -0.0679012313), 83125, 4275},  //
      {glm::vec2(0.450617284, -0.0679012313), 83124, 4273},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.104938276), 83128, 4278},  //
      {glm::vec2(0.413580239, -0.117283955), 83129, 4277},  //
      {glm::vec2(0.401234567, -0.117283955), 83130, 4280},  //
      {glm::vec2(0.401234567, -0.104938276), 83131, 4279},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.117283955), 83132, 4302},  //
      {glm::vec2(0.364197552, -0.117283955), 83157, 4303},  //
      {glm::vec2(0.364197552, -0.104938276), 83134, 4304},  //
      {glm::vec2(0.376543224, -0.104938276), 83133, 4281},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.0802469105), 83135, 4282},  //
      {glm::vec2(0.401234567, -0.0802469105), 83137, 4285},  //
      {glm::vec2(0.401234567, -0.0679012313), 83138, 4284},  //
      {glm::vec2(0.413580239, -0.0679012313), 83136, 4283},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.0802469105), 83139, 4305},  //
      {glm::vec2(0.364197552, -0.0802469105), 83161, 4306},  //
      {glm::vec2(0.364197552, -0.0679012313), 83141, 4307},  //
      {glm::vec2(0.376543224, -0.0679012313), 83140, 4286},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.0432098769), 83142, 4288},  //
      {glm::vec2(0.475308657, -0.0432098769), 83145, 4289},  //
      {glm::vec2(0.475308657, -0.0308641978), 83144, 4293},  //
      {glm::vec2(0.487654328, -0.0308641978), 83143, 4287},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.0432098769), 83146, 4290},  //
      {glm::vec2(0.438271612, -0.0432098769), 83149, 4292},  //
      {glm::vec2(0.438271612, -0.0308641978), 83148, 4295},  //
      {glm::vec2(0.450617284, -0.0308641978), 83147, 4291},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.00617283955), 83150, 4294},  //
      {glm::vec2(0.475308657, -0.00617283955), 83569, 9948},  //
      {glm::vec2(0.475308657, 0.00617283955), 83568, 9947},   //
      {glm::vec2(0.487654328, 0.00617283955), 83567, 9946},   //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.0185185187), 83151, 4296},  //
      {glm::vec2(0.42592591, -0.0185185187), 83580, 9959},   //
      {glm::vec2(0.42592591, 0.0185185187), 83571, 9951},    //
      {glm::vec2(0.462962955, 0.0185185187), 83570, 9950},   //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.0308641978), 83152, 4298},  //
      {glm::vec2(0.413580239, -0.0432098769), 83153, 4297},  //
      {glm::vec2(0.401234567, -0.0432098769), 83155, 4299},  //
      {glm::vec2(0.401234567, -0.0308641978), 83154, 4300},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.00617283955), 83156, 4301},  //
      {glm::vec2(0.401234567, -0.00617283955), 83583, 9963},  //
      {glm::vec2(0.401234567, 0.00617283955), 83582, 9961},   //
      {glm::vec2(0.413580239, 0.00617283955), 83581, 9960},   //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.0679012313), 83162, 4310},  //
      {glm::vec2(0.339506179, -0.0802469105), 83163, 4309},  //
      {glm::vec2(0.327160507, -0.0802469105), 83164, 4312},  //
      {glm::vec2(0.327160507, -0.0679012313), 83165, 4311},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.117283955), 83166, 4313},  //
      {glm::vec2(0.290123463, -0.117283955), 83168, 1376},  //
      {glm::vec2(0.290123463, -0.104938276), 83169, 4314},  //
      {glm::vec2(0.302469134, -0.104938276), 83167, 1375},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.117283955), 83170, 4315},  //
      {glm::vec2(0.253086448, -0.117283955), 83172, 1378},  //
      {glm::vec2(0.253086448, -0.104938276), 83173, 4316},  //
      {glm::vec2(0.265432119, -0.104938276), 83171, 1377},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.0802469105), 83174, 4317},  //
      {glm::vec2(0.290123463, -0.0802469105), 83176, 4319},  //
      {glm::vec2(0.290123463, -0.0679012313), 83177, 4320},  //
      {glm::vec2(0.302469134, -0.0679012313), 83175, 4318},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.0802469105), 83178, 4321},  //
      {glm::vec2(0.253086448, -0.0802469105), 83181, 4323},  //
      {glm::vec2(0.253086448, -0.0679012313), 83180, 4324},  //
      {glm::vec2(0.265432119, -0.0679012313), 83179, 4322},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, -0.055555556), 83182, 4325},  //
      {glm::vec2(0.277777791, -0.055555556), 83619, 9989},  //
      {glm::vec2(0.277777791, 0.055555556), 83618, 9988},   //
      {glm::vec2(0.388888896, 0.055555556), 83584, 9964},   //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.0432098769), 83183, 4326},  //
      {glm::vec2(0.253086448, -0.0432098769), 83186, 4328},  //
      {glm::vec2(0.253086448, -0.0308641978), 83185, 4329},  //
      {glm::vec2(0.265432119, -0.0308641978), 83184, 4327},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.00617283955), 83187, 4330},  //
      {glm::vec2(0.253086448, -0.00617283955), 83622, 9992},  //
      {glm::vec2(0.253086448, 0.00617283955), 83621, 9991},   //
      {glm::vec2(0.265432119, 0.00617283955), 83620, 9990},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.240740761), 83188, 4369},  //
      {glm::vec2(0.203703731, -0.240740761), 83190, 4371},  //
      {glm::vec2(0.203703731, -0.203703731), 83194, 4373},  //
      {glm::vec2(0.240740761, -0.203703731), 83189, 4370},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.216049403), 83191, 4372},   //
      {glm::vec2(0.19135803, -0.228395075), 83192, 4376},   //
      {glm::vec2(0.179012358, -0.228395075), 83200, 4377},  //
      {glm::vec2(0.179012358, -0.216049403), 83193, 4382},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.19135803), 83195, 4374},   //
      {glm::vec2(0.216049403, -0.19135803), 83197, 4394},   //
      {glm::vec2(0.216049403, -0.179012358), 83210, 4393},  //
      {glm::vec2(0.228395075, -0.179012358), 83196, 4392},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.19135803), 83198, 4383},    //
      {glm::vec2(0.179012358, -0.19135803), 83206, 4402},   //
      {glm::vec2(0.179012358, -0.179012358), 83212, 4403},  //
      {glm::vec2(0.19135803, -0.179012358), 83199, 4395},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.216049403), 83201, 4379},  //
      {glm::vec2(0.154320985, -0.228395075), 83202, 4378},  //
      {glm::vec2(0.141975313, -0.228395075), 83203, 4380},  //
      {glm::vec2(0.141975313, -0.216049403), 83204, 4385},  //
  });
  polys.push_back({
      {glm::vec2(0.129629627, -0.203703731), 83205, 4381},   //
      {glm::vec2(0.129629627, -0.240740761), 83222, 4414},   //
      {glm::vec2(0.0925925896, -0.240740761), 83223, 4416},  //
      {glm::vec2(0.0925925896, -0.203703731), 83228, 4420},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.179012358), 83207, 4404},  //
      {glm::vec2(0.154320985, -0.19135803), 83208, 4386},   //
      {glm::vec2(0.141975313, -0.19135803), 83209, 4405},   //
      {glm::vec2(0.141975313, -0.179012358), 83220, 4406},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.154320985), 83211, 4396},  //
      {glm::vec2(0.216049403, -0.154320985), 83215, 4400},  //
      {glm::vec2(0.216049403, -0.141975313), 83216, 4398},  //
      {glm::vec2(0.228395075, -0.141975313), 83214, 4397},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.154320985), 83213, 4409},   //
      {glm::vec2(0.179012358, -0.154320985), 83221, 4411},  //
      {glm::vec2(0.179012358, -0.141975313), 83219, 4410},  //
      {glm::vec2(0.19135803, -0.141975313), 83218, 4401},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.129629627), 83217, 4399},   //
      {glm::vec2(0.203703731, -0.129629627), 83249, 4491},   //
      {glm::vec2(0.203703731, -0.0925925896), 83248, 4492},  //
      {glm::vec2(0.240740761, -0.0925925896), 83247, 4490},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.216049403), 83224, 4417},  //
      {glm::vec2(0.0802469105, -0.228395075), 83225, 4418},  //
      {glm::vec2(0.0679012313, -0.228395075), 83226, 4419},  //
      {glm::vec2(0.0679012313, -0.216049403), 83227, 4422},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.19135803), 83229, 4421},   //
      {glm::vec2(0.104938276, -0.19135803), 83231, 4445},   //
      {glm::vec2(0.104938276, -0.179012358), 83244, 4444},  //
      {glm::vec2(0.117283955, -0.179012358), 83230, 4443},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.179012358), 83232, 4446},  //
      {glm::vec2(0.0802469105, -0.19135803), 83233, 4423},   //
      {glm::vec2(0.0679012313, -0.19135803), 83234, 4447},   //
      {glm::vec2(0.0679012313, -0.179012358), 83245, 4448},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.216049403), 83235, 4425},  //
      {glm::vec2(0.0432098769, -0.228395075), 83236, 4424},  //
      {glm::vec2(0.0308641978, -0.228395075), 83237, 4426},  //
      {glm::vec2(0.0308641978, -0.216049403), 83238, 4431},  //
  });
  polys.push_back({
      {glm::vec2(0.0185185187, -0.203703731), 83239, 4427},   //
      {glm::vec2(0.0185185187, -0.240740761), 83520, 6105},   //
      {glm::vec2(-0.0185185187, -0.240740761), 83521, 6106},  //
      {glm::vec2(-0.0185185187, -0.203703731), 83526, 6113},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.19135803), 83240, 4432},   //
      {glm::vec2(0.0308641978, -0.19135803), 83242, 4451},   //
      {glm::vec2(0.0308641978, -0.179012358), 83246, 4450},  //
      {glm::vec2(0.0432098769, -0.179012358), 83241, 4449},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.179012358), 83243, 4452},   //
      {glm::vec2(0.00617283955, -0.19135803), 83527, 6114},    //
      {glm::vec2(-0.00617283955, -0.19135803), 83528, 6136},   //
      {glm::vec2(-0.00617283955, -0.179012358), 83545, 6138},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.117283955), 83250, 4496},   //
      {glm::vec2(0.179012358, -0.117283955), 83260, 1470},  //
      {glm::vec2(0.179012358, -0.104938276), 83252, 4497},  //
      {glm::vec2(0.19135803, -0.104938276), 83251, 1467},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.0802469105), 83253, 4493},  //
      {glm::vec2(0.216049403, -0.0802469105), 83255, 1469},  //
      {glm::vec2(0.216049403, -0.0679012313), 83256, 4494},  //
      {glm::vec2(0.228395075, -0.0679012313), 83254, 1468},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.0802469105), 83257, 4498},   //
      {glm::vec2(0.179012358, -0.0802469105), 83261, 4499},  //
      {glm::vec2(0.179012358, -0.0679012313), 83259, 4500},  //
      {glm::vec2(0.19135803, -0.0679012313), 83258, 4495},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.0432098769), 83262, 4501},  //
      {glm::vec2(0.216049403, -0.0432098769), 83265, 4503},  //
      {glm::vec2(0.216049403, -0.0308641978), 83264, 4505},  //
      {glm::vec2(0.228395075, -0.0308641978), 83263, 4502},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.0432098769), 83266, 4507},   //
      {glm::vec2(0.179012358, -0.0432098769), 83271, 4508},  //
      {glm::vec2(0.179012358, -0.0308641978), 83268, 4509},  //
      {glm::vec2(0.19135803, -0.0308641978), 83267, 4504},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.0185185187), 83269, 4506},   //
      {glm::vec2(0.203703731, -0.0185185187), 83740, 10264},  //
      {glm::vec2(0.203703731, 0.0185185187), 83739, 10262},   //
      {glm::vec2(0.240740761, 0.0185185187), 83738, 10261},   //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.00617283955), 83270, 4510},    //
      {glm::vec2(0.179012358, -0.00617283955), 83750, 10271},  //
      {glm::vec2(0.179012358, 0.00617283955), 83742, 10270},   //
      {glm::vec2(0.19135803, 0.00617283955), 83741, 10265},    //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.487654328), 83276, 4988},  //
      {glm::vec2(-0.0432098769, -0.487654328), 83278, 4989},  //
      {glm::vec2(-0.0432098769, -0.475308657), 83279, 4990},  //
      {glm::vec2(-0.0308641978, -0.475308657), 83277, 4987},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.450617284), 83281, 4996},  //
      {glm::vec2(-0.0432098769, -0.450617284), 83283, 4998},  //
      {glm::vec2(-0.0432098769, -0.438271612), 83284, 4997},  //
      {glm::vec2(-0.0308641978, -0.438271612), 83282, 4994},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.475308657), 83285, 5003},  //
      {glm::vec2(-0.0679012388, -0.487654328), 83286, 5002},  //
      {glm::vec2(-0.0802469179, -0.487654328), 83288, 5005},  //
      {glm::vec2(-0.0802469179, -0.475308657), 83287, 5004},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.475308657), 83289, 5007},  //
      {glm::vec2(-0.104938269, -0.487654328), 83290, 5006},  //
      {glm::vec2(-0.117283948, -0.487654328), 83291, 5008},  //
      {glm::vec2(-0.117283948, -0.475308657), 83292, 5009},  //
  });
  polys.push_back({
      {glm::vec2(-0.092592597, -0.462962955), 83293, 5015},  //
      {glm::vec2(-0.129629642, -0.462962955), 83328, 5065},  //
      {glm::vec2(-0.129629642, -0.42592591), 83312, 5041},   //
      {glm::vec2(-0.092592597, -0.42592591), 83298, 5014},   //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.450617284), 83294, 5010},  //
      {glm::vec2(-0.0802469179, -0.450617284), 83296, 5013},  //
      {glm::vec2(-0.0802469179, -0.438271612), 83297, 5012},  //
      {glm::vec2(-0.0679012388, -0.438271612), 83295, 5011},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.413580239), 83303, 5031},  //
      {glm::vec2(-0.0432098769, -0.413580239), 83305, 5032},  //
      {glm::vec2(-0.0432098769, -0.401234567), 83306, 5036},  //
      {glm::vec2(-0.0308641978, -0.401234567), 83304, 5029},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.413580239), 83308, 5039},  //
      {glm::vec2(-0.0802469179, -0.413580239), 83311, 5040},  //
      {glm::vec2(-0.0802469179, -0.401234567), 83310, 5045},  //
      {glm::vec2(-0.0679012388, -0.401234567), 83309, 5038},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.413580239), 83313, 5043},  //
      {glm::vec2(-0.117283948, -0.413580239), 83315, 5044},  //
      {glm::vec2(-0.117283948, -0.401234567), 83316, 5047},  //
      {glm::vec2(-0.104938269, -0.401234567), 83314, 5042},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.376543224), 83317, 5046},  //
      {glm::vec2(-0.0802469179, -0.376543224), 83319, 5156},  //
      {glm::vec2(-0.0802469179, -0.364197552), 83370, 5155},  //
      {glm::vec2(-0.0679012388, -0.364197552), 83318, 5154},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.364197552), 83320, 5157},  //
      {glm::vec2(-0.104938269, -0.376543224), 83321, 5048},  //
      {glm::vec2(-0.117283948, -0.376543224), 83322, 5158},  //
      {glm::vec2(-0.117283948, -0.364197552), 83372, 5159},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.475308657), 83323, 5056},  //
      {glm::vec2(-0.141975313, -0.487654328), 83324, 5055},  //
      {glm::vec2(-0.154320985, -0.487654328), 83326, 5057},  //
      {glm::vec2(-0.154320985, -0.475308657), 83325, 5058},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.475308657), 83327, 5061},  //
      {glm::vec2(-0.179012358, -0.487654328), 83334, 5059},  //
      {glm::vec2(-0.19135803, -0.487654328), 83335, 5073},   //
      {glm::vec2(-0.19135803, -0.475308657), 83336, 5062},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.450617284), 83329, 5066},  //
      {glm::vec2(-0.154320985, -0.450617284), 83332, 5069},  //
      {glm::vec2(-0.154320985, -0.438271612), 83331, 5068},  //
      {glm::vec2(-0.141975313, -0.438271612), 83330, 5067},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.438271612), 83333, 5071},  //
      {glm::vec2(-0.179012358, -0.450617284), 83342, 5070},  //
      {glm::vec2(-0.19135803, -0.450617284), 83343, 5078},   //
      {glm::vec2(-0.19135803, -0.438271612), 83345, 5072},   //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.475308657), 83337, 5074},  //
      {glm::vec2(-0.216049403, -0.487654328), 83338, 5075},  //
      {glm::vec2(-0.228395075, -0.487654328), 83339, 5076},  //
      {glm::vec2(-0.228395075, -0.475308657), 83340, 5077},  //
  });
  polys.push_back({
      {glm::vec2(-0.203703716, -0.462962955), 83341, 5080},  //
      {glm::vec2(-0.240740746, -0.462962955), 83346, 5081},  //
      {glm::vec2(-0.240740746, -0.42592591), 83359, 5102},   //
      {glm::vec2(-0.203703716, -0.42592591), 83344, 5079},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.413580239), 83347, 5087},  //
      {glm::vec2(-0.154320985, -0.413580239), 83350, 5089},  //
      {glm::vec2(-0.154320985, -0.401234567), 83349, 5094},  //
      {glm::vec2(-0.141975313, -0.401234567), 83348, 5088},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.401234567), 83351, 5091},  //
      {glm::vec2(-0.179012358, -0.413580239), 83356, 5092},  //
      {glm::vec2(-0.19135803, -0.413580239), 83357, 5100},   //
      {glm::vec2(-0.19135803, -0.401234567), 83358, 5097},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.376543224), 83352, 5095},  //
      {glm::vec2(-0.154320985, -0.376543224), 83354, 5220},  //
      {glm::vec2(-0.154320985, -0.364197552), 83403, 5221},  //
      {glm::vec2(-0.141975313, -0.364197552), 83353, 5219},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.364197552), 83355, 5222},  //
      {glm::vec2(-0.179012358, -0.376543224), 83364, 5098},  //
      {glm::vec2(-0.19135803, -0.376543224), 83365, 5233},   //
      {glm::vec2(-0.19135803, -0.364197552), 83410, 5223},   //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.413580239), 83360, 5103},  //
      {glm::vec2(-0.228395075, -0.413580239), 83362, 5104},  //
      {glm::vec2(-0.228395075, -0.401234567), 83363, 5105},  //
      {glm::vec2(-0.216049403, -0.401234567), 83361, 5101},  //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.376543224), 83366, 5106},  //
      {glm::vec2(-0.228395075, -0.376543224), 83368, 5235},  //
      {glm::vec2(-0.228395075, -0.364197552), 83412, 5236},  //
      {glm::vec2(-0.216049403, -0.364197552), 83367, 5234},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.339506179), 83371, 5161},  //
      {glm::vec2(-0.0802469179, -0.339506179), 83376, 5163},  //
      {glm::vec2(-0.0802469179, -0.327160507), 83375, 5165},  //
      {glm::vec2(-0.0679012388, -0.327160507), 83374, 5162},  //
  });
  polys.push_back({
      {glm::vec2(-0.092592597, -0.351851851), 83373, 5160},  //
      {glm::vec2(-0.129629642, -0.351851851), 83405, 5224},  //
      {glm::vec2(-0.129629642, -0.314814806), 83378, 5166},  //
      {glm::vec2(-0.092592597, -0.314814806), 83377, 5164},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.265432119), 83383, 5187},  //
      {glm::vec2(-0.0432098769, -0.265432119), 83385, 5188},  //
      {glm::vec2(-0.0432098769, -0.253086448), 83386, 5189},  //
      {glm::vec2(-0.0308641978, -0.253086448), 83384, 5185},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.302469134), 83387, 5191},  //
      {glm::vec2(-0.0802469179, -0.302469134), 83390, 5194},  //
      {glm::vec2(-0.0802469179, -0.290123463), 83389, 5193},  //
      {glm::vec2(-0.0679012388, -0.290123463), 83388, 5192},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.290123463), 83391, 5196},  //
      {glm::vec2(-0.104938269, -0.302469134), 83392, 5195},  //
      {glm::vec2(-0.117283948, -0.302469134), 83394, 5198},  //
      {glm::vec2(-0.117283948, -0.290123463), 83393, 5197},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.253086448), 83395, 5199},  //
      {glm::vec2(-0.0679012388, -0.265432119), 83396, 5200},  //
      {glm::vec2(-0.0802469179, -0.265432119), 83397, 5202},  //
      {glm::vec2(-0.0802469179, -0.253086448), 83398, 5201},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.253086448), 83399, 5203},  //
      {glm::vec2(-0.104938269, -0.265432119), 83400, 5204},  //
      {glm::vec2(-0.117283948, -0.265432119), 83401, 5205},  //
      {glm::vec2(-0.117283948, -0.253086448), 83402, 5206},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.339506179), 83404, 5225},  //
      {glm::vec2(-0.154320985, -0.339506179), 83408, 5227},  //
      {glm::vec2(-0.154320985, -0.327160507), 83407, 5228},  //
      {glm::vec2(-0.141975313, -0.327160507), 83406, 5226},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.327160507), 83409, 5230},  //
      {glm::vec2(-0.179012358, -0.339506179), 83411, 5229},  //
      {glm::vec2(-0.19135803, -0.339506179), 83414, 5238},   //
      {glm::vec2(-0.19135803, -0.327160507), 83415, 5231},   //
  });
  polys.push_back({
      {glm::vec2(-0.203703716, -0.351851851), 83413, 5237},  //
      {glm::vec2(-0.240740746, -0.351851851), 83417, 5240},  //
      {glm::vec2(-0.240740746, -0.314814806), 83418, 5241},  //
      {glm::vec2(-0.203703716, -0.314814806), 83416, 5239},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.302469134), 83419, 5242},  //
      {glm::vec2(-0.154320985, -0.302469134), 83422, 5245},  //
      {glm::vec2(-0.154320985, -0.290123463), 83421, 5244},  //
      {glm::vec2(-0.141975313, -0.290123463), 83420, 5243},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.290123463), 83423, 5247},  //
      {glm::vec2(-0.179012358, -0.302469134), 83429, 5246},  //
      {glm::vec2(-0.19135803, -0.302469134), 83431, 5260},   //
      {glm::vec2(-0.19135803, -0.290123463), 83430, 5248},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.253086448), 83424, 5250},  //
      {glm::vec2(-0.141975313, -0.265432119), 83425, 5249},  //
      {glm::vec2(-0.154320985, -0.265432119), 83427, 5251},  //
      {glm::vec2(-0.154320985, -0.253086448), 83426, 5252},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.253086448), 83428, 5255},  //
      {glm::vec2(-0.179012358, -0.265432119), 83436, 5256},  //
      {glm::vec2(-0.19135803, -0.265432119), 83437, 5265},   //
      {glm::vec2(-0.19135803, -0.253086448), 83438, 5257},   //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.302469134), 83432, 5262},  //
      {glm::vec2(-0.228395075, -0.302469134), 83434, 5264},  //
      {glm::vec2(-0.228395075, -0.290123463), 83435, 5263},  //
      {glm::vec2(-0.216049403, -0.290123463), 83433, 5261},  //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.253086448), 83439, 5266},  //
      {glm::vec2(-0.216049403, -0.265432119), 83440, 5267},  //
      {glm::vec2(-0.228395075, -0.265432119), 83441, 5268},  //
      {glm::vec2(-0.228395075, -0.253086448), 83442, 5269},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.475308657), 83443, 5321},  //
      {glm::vec2(-0.253086448, -0.487654328), 83444, 5320},  //
      {glm::vec2(-0.265432119, -0.487654328), 83445, 5322},  //
      {glm::vec2(-0.265432119, -0.475308657), 83446, 5323},  //
  });
  polys.push_back({
      {glm::vec2(-0.290123463, -0.487654328), 83447, 5324},  //
      {glm::vec2(-0.302469134, -0.487654328), 83449, 5327},  //
      {glm::vec2(-0.302469134, -0.475308657), 83450, 5326},  //
      {glm::vec2(-0.290123463, -0.475308657), 83448, 5325},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.438271612), 83451, 5329},  //
      {glm::vec2(-0.253086448, -0.450617284), 83452, 5328},  //
      {glm::vec2(-0.265432119, -0.450617284), 83453, 5330},  //
      {glm::vec2(-0.265432119, -0.438271612), 83454, 5331},  //
  });
  polys.push_back({
      {glm::vec2(-0.290123463, -0.450617284), 83455, 5332},  //
      {glm::vec2(-0.302469134, -0.450617284), 83457, 5335},  //
      {glm::vec2(-0.302469134, -0.438271612), 83458, 5334},  //
      {glm::vec2(-0.290123463, -0.438271612), 83456, 5333},  //
  });
  polys.push_back({
      {glm::vec2(-0.327160507, -0.475308657), 83459, 5337},  //
      {glm::vec2(-0.327160507, -0.487654328), 83460, 5336},  //
      {glm::vec2(-0.339506179, -0.487654328), 83463, 5338},  //
      {glm::vec2(-0.339506179, -0.475308657), 83461, 5339},  //
  });
  polys.push_back({
      {glm::vec2(-0.314814836, -0.462962955), 83462, 5343},  //
      {glm::vec2(-0.351851881, -0.462962955), 83466, 2056},  //
      {glm::vec2(-0.351851881, -0.42592591), 83479, 5357},   //
      {glm::vec2(-0.314814836, -0.42592591), 83465, 5344},   //
  });
  polys.push_back({
      {glm::vec2(-0.364197552, -0.475308657), 83464, 5341},  //
      {glm::vec2(-0.364197552, -0.487654328), 83486, 5340},  //
      {glm::vec2(-0.376543224, -0.487654328), 83487, 5365},  //
      {glm::vec2(-0.376543224, -0.475308657), 83488, 5342},  //
  });
  polys.push_back({
      {glm::vec2(-0.364197552, -0.438271612), 83467, 5346},  //
      {glm::vec2(-0.364197552, -0.450617284), 83493, 5345},  //
      {glm::vec2(-0.376543224, -0.450617284), 83494, 5370},  //
      {glm::vec2(-0.376543224, -0.438271612), 83495, 5347},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.413580239), 83468, 5349},  //
      {glm::vec2(-0.265432119, -0.413580239), 83470, 5350},  //
      {glm::vec2(-0.265432119, -0.401234567), 83471, 5354},  //
      {glm::vec2(-0.253086448, -0.401234567), 83469, 5348},  //
  });
  polys.push_back({
      {glm::vec2(-0.290123463, -0.413580239), 83472, 5351},  //
      {glm::vec2(-0.302469134, -0.413580239), 83474, 5353},  //
      {glm::vec2(-0.302469134, -0.401234567), 83475, 5356},  //
      {glm::vec2(-0.290123463, -0.401234567), 83473, 5352},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.364197552), 83476, 5543},  //
      {glm::vec2(-0.253086448, -0.376543224), 83477, 5355},  //
      {glm::vec2(-0.265432119, -0.376543224), 83478, 5544},  //
      {glm::vec2(-0.265432119, -0.364197552), 83510, 5545},  //
  });
  polys.push_back({
      {glm::vec2(-0.327160507, -0.413580239), 83480, 5359},  //
      {glm::vec2(-0.339506179, -0.413580239), 83483, 5360},  //
      {glm::vec2(-0.339506179, -0.401234567), 83482, 5363},  //
      {glm::vec2(-0.327160507, -0.401234567), 83481, 5358},  //
  });
  polys.push_back({
      {glm::vec2(-0.364197552, -0.401234567), 83484, 5361},  //
      {glm::vec2(-0.364197552, -0.413580239), 83506, 5362},  //
      {glm::vec2(-0.376543224, -0.413580239), 83507, 5419},  //
      {glm::vec2(-0.376543224, -0.401234567), 83508, 5364},  //
  });
  polys.push_back({
      {glm::vec2(-0.401234567, -0.475308657), 83489, 5366},  //
      {glm::vec2(-0.401234567, -0.487654328), 83490, 5367},  //
      {glm::vec2(-0.413580239, -0.487654328), 83492, 5368},  //
      {glm::vec2(-0.413580239, -0.475308657), 83491, 5369},  //
  });
  polys.push_back({
      {glm::vec2(-0.401234567, -0.450617284), 83496, 5372},  //
      {glm::vec2(-0.413580239, -0.450617284), 83498, 5373},  //
      {glm::vec2(-0.413580239, -0.438271612), 83499, 5374},  //
      {glm::vec2(-0.401234567, -0.438271612), 83497, 5371},  //
  });
  polys.push_back({
      {glm::vec2(-0.438271612, -0.475308657), 83500, 5376},  //
      {glm::vec2(-0.438271612, -0.487654328), 83501, 5375},  //
      {glm::vec2(-0.450617284, -0.487654328), 83502, 5377},  //
      {glm::vec2(-0.450617284, -0.475308657), 83503, 5378},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.339506179), 83511, 5546},  //
      {glm::vec2(-0.265432119, -0.339506179), 83513, 5548},  //
      {glm::vec2(-0.265432119, -0.327160507), 83514, 5549},  //
      {glm::vec2(-0.253086448, -0.327160507), 83512, 5547},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.290123463), 83515, 5551},  //
      {glm::vec2(-0.253086448, -0.302469134), 83516, 5550},  //
      {glm::vec2(-0.265432119, -0.302469134), 83517, 5552},  //
      {glm::vec2(-0.265432119, -0.290123463), 83518, 5553},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.228395075), 83522, 6109},  //
      {glm::vec2(-0.0432098769, -0.228395075), 83524, 6110},  //
      {glm::vec2(-0.0432098769, -0.216049403), 83525, 6116},  //
      {glm::vec2(-0.0308641978, -0.216049403), 83523, 6107},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.19135803), 83529, 6117},   //
      {glm::vec2(-0.0432098769, -0.19135803), 83531, 6141},   //
      {glm::vec2(-0.0432098769, -0.179012358), 83547, 6142},  //
      {glm::vec2(-0.0308641978, -0.179012358), 83530, 6139},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.216049403), 83532, 6120},  //
      {glm::vec2(-0.0679012388, -0.228395075), 83533, 6119},  //
      {glm::vec2(-0.0802469179, -0.228395075), 83535, 6121},  //
      {glm::vec2(-0.0802469179, -0.216049403), 83534, 6124},  //
  });
  polys.push_back({
      {glm::vec2(-0.092592597, -0.240740761), 83536, 6123},  //
      {glm::vec2(-0.129629642, -0.240740761), 83550, 6152},  //
      {glm::vec2(-0.129629642, -0.203703731), 83541, 6126},  //
      {glm::vec2(-0.092592597, -0.203703731), 83537, 6122},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.19135803), 83538, 6125},   //
      {glm::vec2(-0.0802469179, -0.19135803), 83540, 6146},   //
      {glm::vec2(-0.0802469179, -0.179012358), 83548, 6145},  //
      {glm::vec2(-0.0679012388, -0.179012358), 83539, 6144},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.179012358), 83542, 6147},  //
      {glm::vec2(-0.104938269, -0.19135803), 83543, 6128},   //
      {glm::vec2(-0.117283948, -0.19135803), 83544, 6148},   //
      {glm::vec2(-0.117283948, -0.179012358), 83549, 6149},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.216049403), 83551, 6154},  //
      {glm::vec2(-0.141975313, -0.228395075), 83552, 6153},  //
      {glm::vec2(-0.154320985, -0.228395075), 83554, 6155},  //
      {glm::vec2(-0.154320985, -0.216049403), 83553, 6158},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.216049403), 83555, 6157},  //
      {glm::vec2(-0.179012358, -0.228395075), 83559, 6156},  //
      {glm::vec2(-0.19135803, -0.228395075), 83560, 6163},   //
      {glm::vec2(-0.19135803, -0.216049403), 83561, 6161},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.19135803), 83556, 6159},   //
      {glm::vec2(-0.154320985, -0.19135803), 83558, 6172},   //
      {glm::vec2(-0.154320985, -0.179012358), 83566, 6173},  //
      {glm::vec2(-0.141975313, -0.179012358), 83557, 6171},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.0308641978), 83572, 9949},  //
      {glm::vec2(0.475308657, 0.0308641978), 83574, 9954},  //
      {glm::vec2(0.475308657, 0.0432098769), 83575, 9955},  //
      {glm::vec2(0.487654328, 0.0432098769), 83573, 9953},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.0432098769), 83576, 9956},  //
      {glm::vec2(0.450617284, 0.0308641978), 83577, 9952},  //
      {glm::vec2(0.438271612, 0.0308641978), 83579, 9957},  //
      {glm::vec2(0.438271612, 0.0432098769), 83578, 9958},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.0432098769), 83585, 9965},  //
      {glm::vec2(0.413580239, 0.0308641978), 83586, 9962},  //
      {glm::vec2(0.401234567, 0.0308641978), 83587, 9967},  //
      {glm::vec2(0.401234567, 0.0432098769), 83588, 9966},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.0679012388), 83589, 9969},  //
      {glm::vec2(0.475308657, 0.0679012388), 83591, 9970},  //
      {glm::vec2(0.475308657, 0.0802469179), 83592, 9971},  //
      {glm::vec2(0.487654328, 0.0802469179), 83590, 9968},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.0679012388), 83593, 9972},  //
      {glm::vec2(0.438271612, 0.0679012388), 83595, 9974},  //
      {glm::vec2(0.438271612, 0.0802469179), 83596, 9975},  //
      {glm::vec2(0.450617284, 0.0802469179), 83594, 9973},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.104938269), 83597, 9977},  //
      {glm::vec2(0.475308657, 0.104938269), 83599, 7482},  //
      {glm::vec2(0.475308657, 0.117283948), 83600, 9978},  //
      {glm::vec2(0.487654328, 0.117283948), 83598, 7481},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, 0.129629642), 83601, 9979},  //
      {glm::vec2(0.462962955, 0.092592597), 83602, 9976},  //
      {glm::vec2(0.42592591, 0.092592597), 83611, 9986},   //
      {glm::vec2(0.42592591, 0.129629642), 83655, 10069},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.0802469179), 83603, 9981},  //
      {glm::vec2(0.413580239, 0.0679012388), 83604, 9980},  //
      {glm::vec2(0.401234567, 0.0679012388), 83605, 9983},  //
      {glm::vec2(0.401234567, 0.0802469179), 83606, 9982},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.0679012388), 83607, 9997},  //
      {glm::vec2(0.364197552, 0.0679012388), 83627, 9998},  //
      {glm::vec2(0.364197552, 0.0802469179), 83609, 9999},  //
      {glm::vec2(0.376543224, 0.0802469179), 83608, 9984},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.104938269), 83610, 9985},  //
      {glm::vec2(0.401234567, 0.104938269), 83613, 7484},  //
      {glm::vec2(0.401234567, 0.117283948), 83614, 9987},  //
      {glm::vec2(0.413580239, 0.117283948), 83612, 7483},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.104938269), 83615, 10005},  //
      {glm::vec2(0.364197552, 0.104938269), 83632, 7486},   //
      {glm::vec2(0.364197552, 0.117283948), 83617, 10006},  //
      {glm::vec2(0.376543224, 0.117283948), 83616, 7485},   //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.0308641978), 83623, 9993},  //
      {glm::vec2(0.253086448, 0.0308641978), 83626, 9995},  //
      {glm::vec2(0.253086448, 0.0432098769), 83625, 9996},  //
      {glm::vec2(0.265432119, 0.0432098769), 83624, 9994},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.0802469179), 83628, 10001},  //
      {glm::vec2(0.339506179, 0.0679012388), 83629, 10000},  //
      {glm::vec2(0.327160507, 0.0679012388), 83630, 10002},  //
      {glm::vec2(0.327160507, 0.0802469179), 83631, 10003},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, 0.129629642), 83633, 10007},  //
      {glm::vec2(0.351851851, 0.092592597), 83634, 10004},  //
      {glm::vec2(0.314814806, 0.092592597), 83635, 10008},  //
      {glm::vec2(0.314814806, 0.129629642), 83700, 10157},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.0679012388), 83636, 10010},  //
      {glm::vec2(0.290123463, 0.0679012388), 83638, 10011},  //
      {glm::vec2(0.290123463, 0.0802469179), 83639, 10012},  //
      {glm::vec2(0.302469134, 0.0802469179), 83637, 10009},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.0679012388), 83640, 10013},  //
      {glm::vec2(0.253086448, 0.0679012388), 83642, 10015},  //
      {glm::vec2(0.253086448, 0.0802469179), 83643, 10016},  //
      {glm::vec2(0.265432119, 0.0802469179), 83641, 10014},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.104938269), 83644, 10017},  //
      {glm::vec2(0.290123463, 0.104938269), 83646, 10019},  //
      {glm::vec2(0.290123463, 0.117283948), 83647, 10020},  //
      {glm::vec2(0.302469134, 0.117283948), 83645, 10018},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.104938269), 83648, 10021},  //
      {glm::vec2(0.253086448, 0.104938269), 83651, 10023},  //
      {glm::vec2(0.253086448, 0.117283948), 83650, 10024},  //
      {glm::vec2(0.265432119, 0.117283948), 83649, 10022},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.141975313), 83652, 10066},  //
      {glm::vec2(0.475308657, 0.141975313), 83654, 10067},  //
      {glm::vec2(0.475308657, 0.154320985), 83659, 10068},  //
      {glm::vec2(0.487654328, 0.154320985), 83653, 10065},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.141975313), 83656, 10071},  //
      {glm::vec2(0.438271612, 0.141975313), 83658, 10072},  //
      {glm::vec2(0.438271612, 0.154320985), 83661, 10073},  //
      {glm::vec2(0.450617284, 0.154320985), 83657, 10070},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.179012358), 83660, 10076},  //
      {glm::vec2(0.475308657, 0.179012358), 83673, 10077},  //
      {glm::vec2(0.475308657, 0.19135803), 83674, 10102},   //
      {glm::vec2(0.487654328, 0.19135803), 83672, 10075},   //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.179012358), 83662, 10081},  //
      {glm::vec2(0.438271612, 0.179012358), 83677, 10083},  //
      {glm::vec2(0.438271612, 0.19135803), 83676, 10107},   //
      {glm::vec2(0.450617284, 0.19135803), 83675, 10082},   //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.154320985), 83663, 10087},  //
      {glm::vec2(0.413580239, 0.141975313), 83664, 10086},  //
      {glm::vec2(0.401234567, 0.141975313), 83665, 10089},  //
      {glm::vec2(0.401234567, 0.154320985), 83668, 10088},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.141975313), 83666, 10154},  //
      {glm::vec2(0.364197552, 0.141975313), 83698, 10155},  //
      {glm::vec2(0.364197552, 0.154320985), 83670, 10156},  //
      {glm::vec2(0.376543224, 0.154320985), 83667, 10090},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.179012358), 83669, 10091},  //
      {glm::vec2(0.401234567, 0.179012358), 83686, 10093},  //
      {glm::vec2(0.401234567, 0.19135803), 83687, 10128},   //
      {glm::vec2(0.413580239, 0.19135803), 83685, 10092},   //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.179012358), 83671, 10162},  //
      {glm::vec2(0.364197552, 0.179012358), 83715, 10163},  //
      {glm::vec2(0.364197552, 0.19135803), 83689, 10181},   //
      {glm::vec2(0.376543224, 0.19135803), 83688, 10094},   //
  });
  polys.push_back({
      {glm::vec2(0.462962955, 0.203703716), 83678, 10108},  //
      {glm::vec2(0.42592591, 0.203703716), 83690, 10130},   //
      {glm::vec2(0.42592591, 0.240740746), 83684, 10121},   //
      {glm::vec2(0.462962955, 0.240740746), 83683, 10120},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.216049403), 83679, 10104},  //
      {glm::vec2(0.475308657, 0.216049403), 83682, 10114},  //
      {glm::vec2(0.475308657, 0.228395075), 83681, 10115},  //
      {glm::vec2(0.487654328, 0.228395075), 83680, 10113},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.228395075), 83691, 10131},  //
      {glm::vec2(0.413580239, 0.216049403), 83692, 10129},  //
      {glm::vec2(0.401234567, 0.216049403), 83694, 10133},  //
      {glm::vec2(0.401234567, 0.228395075), 83693, 10132},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.216049403), 83695, 10182},  //
      {glm::vec2(0.364197552, 0.216049403), 83720, 10185},  //
      {glm::vec2(0.364197552, 0.228395075), 83697, 10186},  //
      {glm::vec2(0.376543224, 0.228395075), 83696, 10134},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.154320985), 83699, 10158},  //
      {glm::vec2(0.339506179, 0.141975313), 83701, 10159},  //
      {glm::vec2(0.327160507, 0.141975313), 83702, 10161},  //
      {glm::vec2(0.327160507, 0.154320985), 83703, 10160},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.179012358), 83704, 10164},  //
      {glm::vec2(0.327160507, 0.179012358), 83717, 10166},  //
      {glm::vec2(0.327160507, 0.19135803), 83718, 10183},   //
      {glm::vec2(0.339506179, 0.19135803), 83716, 10165},   //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.141975313), 83705, 10168},  //
      {glm::vec2(0.290123463, 0.141975313), 83707, 10169},  //
      {glm::vec2(0.290123463, 0.154320985), 83711, 10170},  //
      {glm::vec2(0.302469134, 0.154320985), 83706, 10167},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.141975313), 83708, 10171},  //
      {glm::vec2(0.253086448, 0.141975313), 83710, 10173},  //
      {glm::vec2(0.253086448, 0.154320985), 83713, 10174},  //
      {glm::vec2(0.265432119, 0.154320985), 83709, 10172},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.179012358), 83712, 10176},  //
      {glm::vec2(0.290123463, 0.179012358), 83725, 10177},  //
      {glm::vec2(0.290123463, 0.19135803), 83726, 10190},   //
      {glm::vec2(0.302469134, 0.19135803), 83724, 10175},   //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.179012358), 83714, 10178},  //
      {glm::vec2(0.253086448, 0.179012358), 83729, 10180},  //
      {glm::vec2(0.253086448, 0.19135803), 83728, 10192},   //
      {glm::vec2(0.265432119, 0.19135803), 83727, 10179},   //
  });
  polys.push_back({
      {glm::vec2(0.351851851, 0.203703716), 83719, 10184},  //
      {glm::vec2(0.314814806, 0.203703716), 83722, 10188},  //
      {glm::vec2(0.314814806, 0.240740746), 83723, 10189},  //
      {glm::vec2(0.351851851, 0.240740746), 83721, 10187},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.216049403), 83730, 10191},  //
      {glm::vec2(0.290123463, 0.216049403), 83732, 10195},  //
      {glm::vec2(0.290123463, 0.228395075), 83733, 10196},  //
      {glm::vec2(0.302469134, 0.228395075), 83731, 10194},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.216049403), 83734, 10193},  //
      {glm::vec2(0.253086448, 0.216049403), 83737, 10198},  //
      {glm::vec2(0.253086448, 0.228395075), 83736, 10199},  //
      {glm::vec2(0.265432119, 0.228395075), 83735, 10197},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.0308641978), 83743, 10263},  //
      {glm::vec2(0.216049403, 0.0308641978), 83745, 10268},  //
      {glm::vec2(0.216049403, 0.0432098769), 83746, 10267},  //
      {glm::vec2(0.228395075, 0.0432098769), 83744, 10266},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.0432098769), 83747, 10269},   //
      {glm::vec2(0.19135803, 0.0308641978), 83748, 10272},   //
      {glm::vec2(0.179012358, 0.0308641978), 83751, 10273},  //
      {glm::vec2(0.179012358, 0.0432098769), 83749, 10274},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.0679012388), 83752, 10275},  //
      {glm::vec2(0.216049403, 0.0679012388), 83754, 10279},  //
      {glm::vec2(0.216049403, 0.0802469179), 83755, 10277},  //
      {glm::vec2(0.228395075, 0.0802469179), 83753, 10276},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.0679012388), 83756, 10284},   //
      {glm::vec2(0.179012358, 0.0679012388), 83765, 10285},  //
      {glm::vec2(0.179012358, 0.0802469179), 83758, 10286},  //
      {glm::vec2(0.19135803, 0.0802469179), 83757, 10280},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, 0.092592597), 83759, 10278},  //
      {glm::vec2(0.203703731, 0.092592597), 83762, 10282},  //
      {glm::vec2(0.203703731, 0.129629642), 83767, 10317},  //
      {glm::vec2(0.240740761, 0.129629642), 83760, 10281},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.104938269), 83761, 10287},   //
      {glm::vec2(0.179012358, 0.104938269), 83766, 10288},  //
      {glm::vec2(0.179012358, 0.117283948), 83764, 10289},  //
      {glm::vec2(0.19135803, 0.117283948), 83763, 10283},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.141975313), 83768, 10319},  //
      {glm::vec2(0.216049403, 0.141975313), 83770, 10321},  //
      {glm::vec2(0.216049403, 0.154320985), 83773, 10320},  //
      {glm::vec2(0.228395075, 0.154320985), 83769, 10318},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.141975313), 83771, 10327},   //
      {glm::vec2(0.179012358, 0.141975313), 83777, 10328},  //
      {glm::vec2(0.179012358, 0.154320985), 83775, 10329},  //
      {glm::vec2(0.19135803, 0.154320985), 83772, 10322},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.179012358), 83774, 10323},  //
      {glm::vec2(0.216049403, 0.179012358), 83779, 10325},  //
      {glm::vec2(0.216049403, 0.19135803), 83780, 10338},   //
      {glm::vec2(0.228395075, 0.19135803), 83778, 10324},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.253086448), 83786, 10857},  //
      {glm::vec2(0.475308657, 0.253086448), 83789, 10859},  //
      {glm::vec2(0.475308657, 0.265432119), 83788, 10858},  //
      {glm::vec2(0.487654328, 0.265432119), 83787, 10856},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.253086448), 83790, 10865},  //
      {glm::vec2(0.438271612, 0.253086448), 83793, 10867},  //
      {glm::vec2(0.438271612, 0.265432119), 83792, 10868},  //
      {glm::vec2(0.450617284, 0.265432119), 83791, 10866},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.290123463), 83794, 10871},  //
      {glm::vec2(0.475308657, 0.290123463), 83796, 10872},  //
      {glm::vec2(0.475308657, 0.302469134), 83797, 10873},  //
      {glm::vec2(0.487654328, 0.302469134), 83795, 10870},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.302469134), 83798, 10874},  //
      {glm::vec2(0.450617284, 0.290123463), 83799, 10875},  //
      {glm::vec2(0.438271612, 0.290123463), 83801, 10876},  //
      {glm::vec2(0.438271612, 0.302469134), 83800, 10877},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.253086448), 83802, 10879},  //
      {glm::vec2(0.401234567, 0.253086448), 83805, 10882},  //
      {glm::vec2(0.401234567, 0.265432119), 83804, 10881},  //
      {glm::vec2(0.413580239, 0.265432119), 83803, 10880},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.253086448), 83806, 10948},  //
      {glm::vec2(0.364197552, 0.253086448), 83829, 10950},  //
      {glm::vec2(0.364197552, 0.265432119), 83808, 10949},  //
      {glm::vec2(0.376543224, 0.265432119), 83807, 10883},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.302469134), 83809, 10885},  //
      {glm::vec2(0.413580239, 0.290123463), 83810, 10884},  //
      {glm::vec2(0.401234567, 0.290123463), 83811, 10887},  //
      {glm::vec2(0.401234567, 0.302469134), 83812, 10886},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.327160507), 83813, 10912},  //
      {glm::vec2(0.475308657, 0.327160507), 83815, 10914},  //
      {glm::vec2(0.475308657, 0.339506179), 83818, 10915},  //
      {glm::vec2(0.487654328, 0.339506179), 83814, 10913},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, 0.351851881), 83816, 10917},  //
      {glm::vec2(0.462962955, 0.314814836), 83817, 10916},  //
      {glm::vec2(0.42592591, 0.314814836), 83822, 10926},   //
      {glm::vec2(0.42592591, 0.351851881), 83820, 10921},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.364197552), 83819, 10919},  //
      {glm::vec2(0.475308657, 0.364197552), 83844, 10920},  //
      {glm::vec2(0.475308657, 0.376543224), 83845, 11040},  //
      {glm::vec2(0.487654328, 0.376543224), 83843, 10918},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.364197552), 83821, 10922},  //
      {glm::vec2(0.438271612, 0.364197552), 83848, 10924},  //
      {glm::vec2(0.438271612, 0.376543224), 83847, 11045},  //
      {glm::vec2(0.450617284, 0.376543224), 83846, 10923},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.339506179), 83823, 10927},  //
      {glm::vec2(0.413580239, 0.327160507), 83824, 10925},  //
      {glm::vec2(0.401234567, 0.327160507), 83825, 10929},  //
      {glm::vec2(0.401234567, 0.339506179), 83827, 10928},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.364197552), 83828, 10931},  //
      {glm::vec2(0.401234567, 0.364197552), 83859, 10933},  //
      {glm::vec2(0.401234567, 0.376543224), 83860, 11064},  //
      {glm::vec2(0.413580239, 0.376543224), 83858, 10932},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.265432119), 83830, 10952},  //
      {glm::vec2(0.339506179, 0.253086448), 83831, 10951},  //
      {glm::vec2(0.327160507, 0.253086448), 83833, 10954},  //
      {glm::vec2(0.327160507, 0.265432119), 83832, 10953},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.253086448), 83835, 10957},  //
      {glm::vec2(0.290123463, 0.253086448), 83837, 10958},  //
      {glm::vec2(0.290123463, 0.265432119), 83838, 10959},  //
      {glm::vec2(0.302469134, 0.265432119), 83836, 10956},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.401234567), 83849, 11042},  //
      {glm::vec2(0.475308657, 0.401234567), 83851, 11052},  //
      {glm::vec2(0.475308657, 0.413580239), 83852, 11051},  //
      {glm::vec2(0.487654328, 0.413580239), 83850, 11048},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.401234567), 83853, 11046},  //
      {glm::vec2(0.438271612, 0.401234567), 83855, 11058},  //
      {glm::vec2(0.438271612, 0.413580239), 83856, 11059},  //
      {glm::vec2(0.450617284, 0.413580239), 83854, 11057},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.438271612), 83864, 11093},  //
      {glm::vec2(0.475308657, 0.438271612), 83866, 11094},  //
      {glm::vec2(0.475308657, 0.450617284), 83867, 11095},  //
      {glm::vec2(0.487654328, 0.450617284), 83865, 11092},  //
  });
  TestPoly(polys, 1771);
}

// void fnExit() { throw std::runtime_error("Someone called Exit()!"); }

int main(int argc, char **argv) {
  // atexit(fnExit);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}