// Copyright 2021 The Manifold Authors.
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
#include <limits>
#include <random>

#include "test.h"

namespace {

using namespace manifold;

void StandardizePoly(SimplePolygonIdx &p) {
  auto start = std::min_element(
      p.begin(), p.end(),
      [](const PolyVert &v1, const PolyVert &v2) { return v1.idx < v2.idx; });
  std::rotate(p.begin(), start, p.end());
}

void StandardizePolys(PolygonsIdx &polys) {
  for (auto &p : polys) StandardizePoly(p);
  std::sort(polys.begin(), polys.end(),
            [](SimplePolygonIdx &p1, SimplePolygonIdx &p2) {
              return p1[0].idx < p2[0].idx;
            });
}

void Identical(PolygonsIdx p1, PolygonsIdx p2) {
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

PolygonsIdx Turn180(PolygonsIdx polys) {
  for (SimplePolygonIdx &poly : polys) {
    for (PolyVert &vert : poly) {
      vert.pos *= -1;
    }
  }
  return polys;
}

PolygonsIdx Duplicate(PolygonsIdx polys) {
  float xMin = std::numeric_limits<float>::infinity();
  float xMax = -std::numeric_limits<float>::infinity();
  int indexMax = 0;
  for (SimplePolygonIdx &poly : polys) {
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
    SimplePolygonIdx poly = polys[i];
    for (PolyVert &vert : poly) {
      vert.pos.x += shift;
      vert.idx += indexMax;
    }
    polys.push_back(poly);
  }
  return polys;
}

void TestPoly(const PolygonsIdx &polys, int expectedNumTri,
              float precision = -1.0f) {
  PolygonParams().verbose = options.params.verbose;

  std::vector<glm::ivec3> triangles;
  EXPECT_NO_THROW(triangles = TriangulateIdx(polys, precision));
  EXPECT_EQ(triangles.size(), expectedNumTri) << "Basic";

  EXPECT_NO_THROW(triangles = TriangulateIdx(Turn180(polys), precision));
  EXPECT_EQ(triangles.size(), expectedNumTri) << "Turn 180";

  EXPECT_NO_THROW(triangles = TriangulateIdx(Duplicate(polys), precision));
  EXPECT_EQ(triangles.size(), 2 * expectedNumTri) << "Duplicate";

  PolygonParams().verbose = false;
}
}  // namespace

/**
 * These polygons are all valid geometry. Some are clearly valid, while many are
 * marginal, but all should produce correct topology and geometry, within
 * tolerance.
 */
TEST(Polygon, SimpleHole) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0, -2), 0},  //
      {glm::vec2(2, 2), 1},   //
      {glm::vec2(0, 4), 2},   //
      {glm::vec2(-3, 3), 3},  //
  });
  polys.push_back({
      {glm::vec2(0, -1), 4},  //
      {glm::vec2(-1, 1), 5},  //
      {glm::vec2(1, 1), 6},   //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, SimpleHole2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0, 1.63299), 0},           //
      {glm::vec2(-1.41421, -0.816496), 1},  //
      {glm::vec2(1.41421, -0.816496), 2},   //
  });
  polys.push_back({
      {glm::vec2(0, 1.02062), 3},           //
      {glm::vec2(0.883883, -0.51031), 4},   //
      {glm::vec2(-0.883883, -0.51031), 5},  //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, MultiMerge) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-7, 0), 0},   //
      {glm::vec2(-6, 3), 1},   //
      {glm::vec2(-5, 1), 2},   //
      {glm::vec2(-4, 6), 3},   //
      {glm::vec2(-3, 2), 4},   //
      {glm::vec2(-2, 5), 5},   //
      {glm::vec2(-1, 4), 6},   //
      {glm::vec2(0, 12), 7},   //
      {glm::vec2(-6, 10), 8},  //
      {glm::vec2(-8, 11), 9},  //
  });
  polys.push_back({
      {glm::vec2(-5, 7), 10},  //
      {glm::vec2(-6, 8), 11},  //
      {glm::vec2(-5, 9), 12},  //
  });
  TestPoly(polys, 13);
}

TEST(Polygon, Colinear) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-5.48368, -3.73905), 0},   //
      {glm::vec2(-4.9881, -4.51552), 1},    //
      {glm::vec2(-4.78988, -4.13186), 2},   //
      {glm::vec2(-4.82012, -4.13999), 3},   //
      {glm::vec2(-4.84314, -4.14617), 4},   //
      {glm::vec2(-4.85738, -4.13581), 5},   //
      {glm::vec2(-4.86772, -4.12831), 6},   //
      {glm::vec2(-4.87337, -4.12422), 7},   //
      {glm::vec2(-4.88097, -4.1187), 8},    //
      {glm::vec2(-4.89799, -4.10634), 9},   //
      {glm::vec2(-4.90219, -4.10329), 10},  //
      {glm::vec2(-4.90826, -4.09887), 11},  //
      {glm::vec2(-4.90846, -4.09873), 12},  //
      {glm::vec2(-4.91227, -4.09597), 13},  //
      {glm::vec2(-4.92199, -4.0889), 14},   //
      {glm::vec2(-5.0245, -4.01443), 15},   //
      {glm::vec2(-5.02494, -4.01412), 16},  //
      {glm::vec2(-5.02536, -4.01381), 17},  //
      {glm::vec2(-5.0316, -4.00927), 18},   //
      {glm::vec2(-5.03211, -4.00891), 19},  //
      {glm::vec2(-5.05197, -3.99448), 20},  //
      {glm::vec2(-5.14757, -3.92504), 21},  //
      {glm::vec2(-5.21287, -3.8776), 22},   //
      {glm::vec2(-5.29419, -3.81853), 23},  //
      {glm::vec2(-5.29907, -3.81499), 24},  //
      {glm::vec2(-5.36732, -3.76541), 25},  //
  });
  TestPoly(polys, 24);
}

TEST(Polygon, Merges) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-3.22039, 10.2769), 0},   //
      {glm::vec2(-3.12437, 10.4147), 1},   //
      {glm::vec2(-3.99093, 10.1781), 2},   //
      {glm::vec2(-3.8154, 10.0716), 3},    //
      {glm::vec2(-3.78982, 10.0893), 4},   //
      {glm::vec2(-3.55033, 10.2558), 5},   //
      {glm::vec2(-3.50073, 10.2549), 6},   //
      {glm::vec2(-3.47018, 10.2572), 7},   //
      {glm::vec2(-3.42633, 10.2605), 8},   //
      {glm::vec2(-3.34311, 10.2604), 9},   //
      {glm::vec2(-3.32096, 10.2633), 10},  //
  });
  TestPoly(polys, 9);
}

TEST(Polygon, ExtraTriangle) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(1.23141634, -0.493547261), 0},  //
      {glm::vec2(1.23142254, -0.493540883), 1},  //
      {glm::vec2(1.23088336, -0.457464248), 2},  //
  });
  polys.push_back({
      {glm::vec2(1.23146737, -0.493494928), 3},   //
      {glm::vec2(1.47253549, -0.24623163), 4},    //
      {glm::vec2(1.47253144, -0.246230021), 5},   //
      {glm::vec2(1.47166216, -0.246238187), 6},   //
      {glm::vec2(1.46963537, -0.24623026), 7},    //
      {glm::vec2(1.46811843, -0.246224999), 8},   //
      {glm::vec2(1.46594918, -0.246223733), 9},   //
      {glm::vec2(1.46594965, -0.246232167), 10},  //
      {glm::vec2(1.46594965, -0.246232241), 11},  //
      {glm::vec2(1.46594083, -0.246223733), 12},  //
      {glm::vec2(1.46591508, -0.246223718), 13},  //
      {glm::vec2(1.46594179, -0.246231437), 14},  //
      {glm::vec2(1.46585774, -0.246223688), 15},  //
      {glm::vec2(1.46577716, -0.246223629), 16},  //
      {glm::vec2(1.46590662, -0.246231213), 17},  //
      {glm::vec2(1.46550393, -0.246223465), 18},  //
      {glm::vec2(1.46081161, -0.246220738), 19},  //
      {glm::vec2(1.46407437, -0.246228904), 20},  //
      {glm::vec2(1.45568836, -0.246227756), 21},  //
      {glm::vec2(1.42199826, -0.260253757), 22},  //
      {glm::vec2(1.40801644, -0.246204734), 23},  //
      {glm::vec2(1.40688479, -0.24620308), 24},   //
      {glm::vec2(1.38571239, -0.246184081), 25},  //
      {glm::vec2(1.30817795, -0.246158242), 26},  //
      {glm::vec2(1.28997684, -0.328056872), 27},  //
      {glm::vec2(1.30974603, -0.315009534), 28},  //
      {glm::vec2(1.31756043, -0.364917517), 29},  //
  });
  TestPoly(polys, 26);
}

TEST(Polygon, SpongeThin) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.5, -0.475308657), 11},          //
      {glm::vec2(-0.487654328, -0.475461066), 12},  //
      {glm::vec2(-0.487654328, -0.475308657), 13},  //
      {glm::vec2(-0.487654328, -0.475308657), 14},  //
      {glm::vec2(-0.487654328, -0.475308657), 15},  //
      {glm::vec2(-0.475308657, -0.475308657), 16},  //
      {glm::vec2(-0.475308657, -0.475613475), 17},  //
      {glm::vec2(-0.462962985, -0.475765914), 18},  //
      {glm::vec2(-0.462962985, -0.475308657), 19},  //
  });
  polys.push_back({
      {glm::vec2(0.42592591, -0.475308657), 20},   //
      {glm::vec2(0.351851851, -0.475308657), 21},  //
      {glm::vec2(0.351851851, -0.475308657), 22},  //
      {glm::vec2(0.351851851, -0.48582533), 23},   //
      {glm::vec2(0.364197552, -0.485977769), 24},  //
      {glm::vec2(0.364197552, -0.475308657), 25},  //
      {glm::vec2(0.364197552, -0.475308657), 26},  //
      {glm::vec2(0.364197552, -0.475308657), 27},  //
      {glm::vec2(0.376543224, -0.475308657), 28},  //
      {glm::vec2(0.376543224, -0.486130178), 29},  //
      {glm::vec2(0.401234567, -0.486434996), 30},  //
      {glm::vec2(0.401234567, -0.475308657), 31},  //
      {glm::vec2(0.401234567, -0.475308657), 32},  //
      {glm::vec2(0.401234567, -0.475308657), 33},  //
      {glm::vec2(0.413580239, -0.475308657), 34},  //
      {glm::vec2(0.413580239, -0.486587405), 35},  //
      {glm::vec2(0.42592591, -0.486739844), 36},   //
  });
  polys.push_back({
      {glm::vec2(0.314814806, -0.475308657), 37},  //
      {glm::vec2(0.240740761, -0.475308657), 38},  //
      {glm::vec2(0.240740761, -0.475308657), 39},  //
      {glm::vec2(0.240740761, -0.484453589), 40},  //
      {glm::vec2(0.253086448, -0.484606028), 41},  //
      {glm::vec2(0.253086448, -0.475308657), 42},  //
      {glm::vec2(0.253086448, -0.475308657), 43},  //
      {glm::vec2(0.253086448, -0.475308657), 44},  //
      {glm::vec2(0.265432119, -0.475308657), 45},  //
      {glm::vec2(0.265432119, -0.484758437), 46},  //
      {glm::vec2(0.290123463, -0.485063255), 47},  //
      {glm::vec2(0.290123463, -0.475308657), 48},  //
      {glm::vec2(0.290123463, -0.475308657), 49},  //
      {glm::vec2(0.290123463, -0.475308657), 50},  //
      {glm::vec2(0.302469134, -0.475308657), 51},  //
      {glm::vec2(0.302469134, -0.485215664), 52},  //
      {glm::vec2(0.314814806, -0.485368103), 53},  //
      {glm::vec2(0.314814806, -0.475308657), 54},  //
  });
  TestPoly(polys, 38);
}

TEST(Polygon, ColinearY) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0, 0), 0},    //
      {glm::vec2(1, 1), 1},    //
      {glm::vec2(2, 1), 2},    //
      {glm::vec2(3, 1), 3},    //
      {glm::vec2(4, 1), 4},    //
      {glm::vec2(4, 2), 5},    //
      {glm::vec2(3, 2), 6},    //
      {glm::vec2(2, 2), 7},    //
      {glm::vec2(1, 2), 8},    //
      {glm::vec2(0, 3), 9},    //
      {glm::vec2(-1, 2), 10},  //
      {glm::vec2(-2, 2), 11},  //
      {glm::vec2(-3, 2), 12},  //
      {glm::vec2(-4, 2), 13},  //
      {glm::vec2(-4, 1), 14},  //
      {glm::vec2(-3, 1), 15},  //
      {glm::vec2(-2, 1), 16},  //
      {glm::vec2(-1, 1), 17},  //
  });
  TestPoly(polys, 16);
}

TEST(Polygon, Concave) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.707107008, -0.707107008), 0},    //
      {glm::vec2(1, 0), 1},                          //
      {glm::vec2(0.683013022, 0), 2},                //
      {glm::vec2(0.379409999, -0.232962996), 3},     //
      {glm::vec2(0.379409999, -0.232962996), 4},     //
      {glm::vec2(1.49012003e-08, -0.183013007), 5},  //
      {glm::vec2(1.49012003e-08, -0.183013007), 6},  //
      {glm::vec2(-0.140431002, 0), 7},               //
      {glm::vec2(-1, 0), 8},                         //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Concave2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(4, 0), 1},    //
      {glm::vec2(3, 2), 3},    //
      {glm::vec2(3, 3), 4},    //
      {glm::vec2(6, 5), 6},    //
      {glm::vec2(6, 14), 13},  //
      {glm::vec2(0, 13), 12},  //
      {glm::vec2(0, 12), 11},  //
      {glm::vec2(3, 11), 10},  //
      {glm::vec2(4, 10), 9},   //
      {glm::vec2(5, 8), 8},    //
      {glm::vec2(1, 7), 7},    //
      {glm::vec2(2, 1), 2},    //
  });
  TestPoly(polys, 10);
}

TEST(Polygon, Sliver) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(2.82002997, 0), 0},             //
      {glm::vec2(2.82002997, 0), 1},             //
      {glm::vec2(2.06105995, 0), 2},             //
      {glm::vec2(2.05792999, 0.0680378973), 3},  //
      {glm::vec2(2.06410003, 0.206908002), 4},   //
      {glm::vec2(2.28446007, 1.04696), 5},       //
      {glm::vec2(2.35005999, 1.24989998), 6},    //
      {glm::vec2(-2.82002997, 15), 7},           //
      {glm::vec2(-2.82002997, 0), 8},            //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Duplicate) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-32.0774002, -10.4309998), 0},   //
      {glm::vec2(-31.7346992, -6.10348988), 1},   //
      {glm::vec2(-31.8645992, -5.61858988), 2},   //
      {glm::vec2(-31.8645992, -5.61858988), 3},   //
      {glm::vec2(-31.8645992, -5.61858988), 4},   //
      {glm::vec2(-31.8645992, -5.61858988), 5},   //
      {glm::vec2(-31.8645992, -5.61858988), 6},   //
      {glm::vec2(-31.8645992, -5.61858988), 7},   //
      {glm::vec2(-31.8645992, -5.61858988), 8},   //
      {glm::vec2(-31.8645992, -5.61858988), 9},   //
      {glm::vec2(-31.8645992, -5.61858988), 10},  //
      {glm::vec2(-31.8645992, -5.61858988), 11},  //
      {glm::vec2(-31.8645992, -5.61858988), 12},  //
      {glm::vec2(-31.8645992, -5.61858988), 13},  //
      {glm::vec2(-31.8645992, -5.61858988), 14},  //
      {glm::vec2(-31.8645992, -5.61858988), 15},  //
      {glm::vec2(-31.8645992, -5.61858988), 16},  //
      {glm::vec2(-31.8645992, -5.61858988), 17},  //
      {glm::vec2(-31.8645992, -5.61858988), 18},  //
      {glm::vec2(-31.8645992, -5.61858988), 19},  //
      {glm::vec2(-31.8645992, -5.61858988), 20},  //
      {glm::vec2(-32.0774002, -3.1865499), 21},   //
  });
  TestPoly(polys, 20);
}

TEST(Polygon, Folded) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(2.82002997, 0), 0},              //
      {glm::vec2(1.23706996, 4.20994997), 1},     //
      {glm::vec2(1.14140999, 4.09090996), 2},     //
      {glm::vec2(1.05895996, 3.94496012), 3},     //
      {glm::vec2(0.00757742021, 2.72726989), 4},  //
      {glm::vec2(-0.468091995, 1.94363999), 5},   //
      {glm::vec2(-1.06106997, 1.36363995), 6},    //
      {glm::vec2(-1.79214001, 0.346489996), 7},   //
      {glm::vec2(-2.27416992, 0), 8},             //
      {glm::vec2(-2.82002997, 0), 9},             //
      {glm::vec2(-2.82002997, 0), 10},            //
  });
  TestPoly(polys, 9);
}

TEST(Polygon, NearlyLinear) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(2.82002997, -8.22814036e-05), 0},    //
      {glm::vec2(2.82002997, -8.22814036e-05), 1},    //
      {glm::vec2(2.31802011, -8.22814036e-05), 2},    //
      {glm::vec2(-0.164566994, -8.22813017e-05), 3},  //
      {glm::vec2(-0.85738802, -8.22814036e-05), 4},   //
      {glm::vec2(-1.01091003, -8.22814036e-05), 5},   //
      {glm::vec2(-1.01091003, -8.22814036e-05), 6},   //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Sliver2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(27.4996014, 8.6873703), 74},    //
      {glm::vec2(28.27701, 9.52887344), 76},     //
      {glm::vec2(27.6687469, 10.8811588), 104},  //
      {glm::vec2(27.5080414, 8.79682922), 242},  //
      {glm::vec2(27.5594807, 8.75218964), 207},  //
      {glm::vec2(27.4996014, 8.6873703), 268},   //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Sliver3) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0, -2.65168381), 369},               //
      {glm::vec2(0, -0.792692184), 1889},             //
      {glm::vec2(0, -0.792692184), 2330},             //
      {glm::vec2(0, -1.04356134), 2430},              //
      {glm::vec2(-0.953957975, -0.768045247), 2331},  //
      {glm::vec2(-1.36363637, -0.757460594), 1892},   //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Sliver4) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.375669807, 8.90489388), 7474},  //
      {glm::vec2(0, 8.39722729), 7459},             //
      {glm::vec2(0, 8.9723053), 7468},              //
      {glm::vec2(0, 8.9723053), 7469},              //
      {glm::vec2(0, 8.96719646), 7467},             //
      {glm::vec2(0, 8.89326191), 7466},             //
      {glm::vec2(0, 8.78047276), 7465},             //
      {glm::vec2(-0.330551624, 8.8897438), 7473},   //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, Sliver5) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-60, 0), 19},               //
      {glm::vec2(-50, 0), 21},               //
      {glm::vec2(-50, 0), 38},               //
      {glm::vec2(-60, 4.37113897e-07), 24},  //
      {glm::vec2(-60, 4.37113897e-07), 37},  //
  });
  polys.push_back({
      {glm::vec2(-60, 100), 20},             //
      {glm::vec2(-60, 4.37113897e-07), 44},  //
      {glm::vec2(-60, 4.37113897e-07), 28},  //
      {glm::vec2(-50, 0), 45},               //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Sliver6) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(10, 0), 5},                //
      {glm::vec2(0, 10), 9},                //
      {glm::vec2(-10, 0), 10},              //
      {glm::vec2(-10, 0), 18},              //
      {glm::vec2(4.37113897e-07, 10), 15},  //
      {glm::vec2(10, 0), 17},               //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Sliver7) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(50, -10), 0},              //
      {glm::vec2(60, 0), 25},               //
      {glm::vec2(50, 0), 31},               //
      {glm::vec2(60, 4.37113897e-07), 32},  //
      {glm::vec2(60, 4.37113897e-07), 33},  //
      {glm::vec2(60, 4.37113897e-07), 24},  //
      {glm::vec2(60, 4.37113897e-07), 2},   //
      {glm::vec2(50, 0), 1},                //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, DISABLED_Sliver8) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(27.9279995, 4.9749999), 4},    //
      {glm::vec2(31.0610008, 2.32299995), 6},   //
      {glm::vec2(31.0610008, 2.32299995), 18},  //
      {glm::vec2(27.9279995, 4.9749999), 16},   //
      {glm::vec2(27.9279995, 4.9749999), 15},   //
      {glm::vec2(31.0610008, 2.32299995), 28},  //
      {glm::vec2(30.4400005, 5.34100008), 17},  //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, DISABLED_Sliver9) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(1.49183154, -0.479596376), 16194},  //
      {glm::vec2(1.71669781, -0.246418983), 16195},  //
      {glm::vec2(1.70233643, -0.246410117), 16196},  //
      {glm::vec2(1.47253978, -0.246277586), 16197},  //
      {glm::vec2(1.47254002, -0.246287003), 17997},  //
      {glm::vec2(1.4869014, -0.246295869), 17996},   //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Colinear2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(11.7864399, -7.4572401), 4176},    //
      {glm::vec2(11.6818037, -7.30982304), 24873},  //
      {glm::vec2(11.6777582, -7.30626202), 28498},  //
      {glm::vec2(11.6789398, -7.30578804), 24872},  //
      {glm::vec2(11.3459997, -6.83671999), 4889},   //
      {glm::vec2(11.25597, -6.9267602), 4888},      //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Split) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.707106769, -0.707106769), 1},     //
      {glm::vec2(1, 0), 14},                          //
      {glm::vec2(0.683012664, 0), 25},                //
      {glm::vec2(0.379409522, -0.232962906), 33},     //
      {glm::vec2(0.379409522, -0.232962906), 32},     //
      {glm::vec2(1.49011612e-08, -0.183012664), 31},  //
      {glm::vec2(1.49011612e-08, -0.183012664), 30},  //
      {glm::vec2(-0.14043057, 0), 24},                //
      {glm::vec2(-1, 0), 4},                          //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Duplicates) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-15, -8.10255623), 1648},         //
      {glm::vec2(-15, -9.02439785), 1650},         //
      {glm::vec2(-13.636364, -9.4640789), 1678},   //
      {glm::vec2(-14.996314, -8.10623646), 1916},  //
      {glm::vec2(-15, -8.10639), 1845},            //
      {glm::vec2(-15, -8.10255623), 1922},         //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Simple1) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(4.04059982, -4.01843977), 2872},   //
      {glm::vec2(3.95867562, -4.25263977), 24604},  //
      {glm::vec2(4.23459578, -4.30138493), 28274},  //
      {glm::vec2(4.235569, -4.30127287), 28273},    //
      {glm::vec2(4.23782539, -4.30141878), 24602},  //
  });
  TestPoly(polys, 3);
}

TEST(Polygon, Simple2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-1, -1), 1},      //
      {glm::vec2(-0.5, -0.5), 9},  //
      {glm::vec2(-1, 0), 11},      //
      {glm::vec2(0, 1), 12},       //
      {glm::vec2(0.5, 0.5), 10},   //
      {glm::vec2(1, 1), 7},        //
      {glm::vec2(-1, 1), 3},       //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Simple3) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(19.7193489, 6.15445995), 19798},  //
      {glm::vec2(20.2308197, 5.64299059), 31187},  //
      {glm::vec2(20.3464642, 5.65459776), 27273},  //
      {glm::vec2(20.3733711, 5.65404081), 27274},  //
      {glm::vec2(20.373394, 5.65404034), 31188},   //
      {glm::vec2(20.8738098, 6.15445995), 19801},  //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Simple4) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(15, -12.7135563), 287},          //
      {glm::vec2(15, -10.6843739), 288},          //
      {glm::vec2(15, -10.6843739), 492},          //
      {glm::vec2(15, -11.0041418), 413},          //
      {glm::vec2(15, -11.4550743), 409},          //
      {glm::vec2(15, -11.4550743), 411},          //
      {glm::vec2(14.9958763, -11.4545326), 408},  //
      {glm::vec2(14.4307623, -11.3802214), 412},  //
      {glm::vec2(13.9298496, -11.2768612), 480},  //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Simple5) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-27.3845081, 0.375669748), 364},  //
      {glm::vec2(-27.6389656, 0), 365},            //
      {glm::vec2(-27.1156006, 0), 355},            //
      {glm::vec2(-27.1156006, 0), 356},            //
      {glm::vec2(-27.1202412, 0), 359},            //
      {glm::vec2(-27.1875362, 0), 360},            //
      {glm::vec2(-27.290184, 0), 362},             //
      {glm::vec2(-27.3733444, 0.330451876), 363},  //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, Simple6) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-7.99813318, 12.8888826), 25009},  //
      {glm::vec2(-7.85714436, 12.9125195), 25006},  //
      {glm::vec2(-7.85714436, 12.9807196), 25005},  //
      {glm::vec2(-7.88929749, 12.9593039), 25007},  //
      {glm::vec2(-7.99812126, 12.8888912), 25008},  //
  });
  TestPoly(polys, 3);
}

TEST(Polygon, TouchingHole) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-2, -1), 0},  //
      {glm::vec2(2, -1), 1},   //
      {glm::vec2(2, 1), 2},    //
      {glm::vec2(-2, 1), 3},   //
  });
  polys.push_back({
      {glm::vec2(-1, -1), 4},  //
      {glm::vec2(-1, 1), 5},   //
      {glm::vec2(1, 1), 6},    //
      {glm::vec2(1, -1), 7},   //
  });
  TestPoly(polys, 8);
}

TEST(Polygon, Degenerate) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(1, -1), 0},   //
      {glm::vec2(1, 1), 1},    //
      {glm::vec2(1, 1), 2},    //
      {glm::vec2(1, -1), 3},   //
      {glm::vec2(1, -1), 4},   //
      {glm::vec2(-1, -1), 5},  //
      {glm::vec2(-1, -1), 6},  //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Degenerate2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0.0740740597, -0.314814836), 4829},  //
      {glm::vec2(0.0925925896, -0.314814806), 4828},  //
      {glm::vec2(0.0925925896, -0.314814806), 4826},  //
      {glm::vec2(0.0740740597, -0.314814836), 4830},  //
  });
  TestPoly(polys, 2);
}

TEST(Polygon, Degenerate3) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.413580239, -0.216049403), 49696},  //
      {glm::vec2(-0.42592591, -0.216049403), 49690},   //
      {glm::vec2(-0.413580239, -0.216049403), 49694},  //
      {glm::vec2(-0.401234567, -0.216049403), 49713},  //
      {glm::vec2(-0.401234567, -0.216049403), 49715},  //
      {glm::vec2(-0.401234567, -0.216049403), 49716},  //
      {glm::vec2(-0.413580239, -0.216049403), 49697},  //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Degenerate4) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0, 10), 213},                      //
      {glm::vec2(-0.0696326792, 9.99390793), 360},  //
      {glm::vec2(4.37113897e-07, 10), 276},         //
      {glm::vec2(0.636729717, 9.94429302), 340},    //
  });
  TestPoly(polys, 2);
}

TEST(Polygon, Degenerate5) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(1, 0), 3},   //
      {glm::vec2(1, 1), 7},   //
      {glm::vec2(1, 1), 15},  //
      {glm::vec2(1, 1), 23},  //
      {glm::vec2(1, 1), 21},  //
      {glm::vec2(0, 1), 22},  //
      {glm::vec2(0, 1), 14},  //
      {glm::vec2(0, 1), 6},   //
  });
  TestPoly(polys, 6);
}

TEST(Polygon, Degenerate6) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(4.37113897e-07, -4.37113897e-07), 0},  //
      {glm::vec2(4.37113897e-07, 0), 18},               //
      {glm::vec2(0, 0), 23},                            //
      {glm::vec2(-1.19421679e-06, 0), 25},              //
      {glm::vec2(-8.66025352, 0), 24},                  //
      {glm::vec2(-8.66025352, 1.339746), 19},           //
      {glm::vec2(-10, -4.37113897e-07), 3},             //
  });
  TestPoly(polys, 5);
}

TEST(Polygon, Tricky) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(1, 0), 0},  //
      {glm::vec2(2, 1), 1},  //
      {glm::vec2(3, 0), 2},  //
      {glm::vec2(3, 5), 3},  //
      {glm::vec2(2, 5), 4},  //
      {glm::vec2(3, 4), 5},  //
      {glm::vec2(3, 2), 6},  //
      {glm::vec2(3, 3), 7},  //
      {glm::vec2(0, 6), 8},  //
  });
  TestPoly(polys, 7);
}

TEST(Polygon, Tricky2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(1, 0), 0},    //
      {glm::vec2(3, 1), 1},    //
      {glm::vec2(3, 3.5), 9},  //
      {glm::vec2(3, 0), 2},    //
      {glm::vec2(3, 5), 3},    //
      {glm::vec2(2, 5), 4},    //
      {glm::vec2(3, 4), 5},    //
      {glm::vec2(3, 2), 6},    //
      {glm::vec2(3, 3), 7},    //
      {glm::vec2(0, 6), 8},    //
  });
  TestPoly(polys, 8);
}

TEST(Polygon, Slit) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(27.7069321, 13.5144091), 286},  //
      {glm::vec2(28.664566, 10.8102894), 267},   //
      {glm::vec2(28.7565536, 10.8183374), 266},  //
      {glm::vec2(25.9535275, 19.4451675), 870},  //
      {glm::vec2(26.0820198, 18.9281673), 865},  //
      {glm::vec2(26.0820198, 18.9281673), 864},  //
      {glm::vec2(26.0820198, 18.9281673), 866},  //
      {glm::vec2(25.8192234, 18.8448315), 867},  //
      {glm::vec2(27.7069321, 13.5144091), 285},  //
      {glm::vec2(27.9789181, 13.2116556), 284},  //
  });
  polys.push_back({
      {glm::vec2(25.6960907, 20.2374783), 891},  //
      {glm::vec2(25.6563644, 20.3597412), 892},  //
      {glm::vec2(25.6467285, 20.3614731), 893},  //
  });
  TestPoly(polys, 9);
}

TEST(Polygon, SharedEdge) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0.265432119, 0.0061728349), 61185},    //
      {glm::vec2(0.277777791, -3.7252903e-09), 61180},  //
      {glm::vec2(0.277777791, 0.0185185187), 61184},    //
      {glm::vec2(0.240740761, 0.0185185187), 76345},    //
      {glm::vec2(0.265432119, 0.00617283955), 61186},   //
      {glm::vec2(0.265432119, 0.00617283955), 61187},   //
  });
  TestPoly(polys, 4);
}

TEST(Polygon, Precision) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.98486793, -0.492948532), 0},   //
      {glm::vec2(-0.984859049, -0.492013603), 1},  //
      {glm::vec2(-0.984966695, -0.489926398), 2},  //
      {glm::vec2(-0.984955609, -0.490281343), 3},  //
      {glm::vec2(-0.985008538, -0.489676297), 4},  //
      {glm::vec2(-0.98491329, -0.491925418), 5},   //
      {glm::vec2(-0.984878719, -0.492937535), 6},  //
  });
  TestPoly(polys, 5, 0.0001);
};

TEST(Polygon, Precision2) {
  PolygonParams().processOverlaps = true;
  const bool intermediateChecks = PolygonParams().intermediateChecks;
  PolygonParams().intermediateChecks = false;

  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(4.98093176, -0.247938812), 11113},   //
      {glm::vec2(4.94630527, -0.0826399028), 22736},  //
      {glm::vec2(4.98092985, -0.247938812), 22735},   //
  });
  polys.push_back({
      {glm::vec2(4.76215458, -0.247848436), 17566},  //
      {glm::vec2(4.76215267, -0.247860417), 22640},  //
      {glm::vec2(4.76215553, -0.247860417), 22639},  //
  });
  polys.push_back({
      {glm::vec2(4.95041943, -0.241741896), 17815},  //
      {glm::vec2(4.85906506, -0.223121181), 17816},  //
      {glm::vec2(4.90268326, -0.152885556), 17824},  //
      {glm::vec2(4.82208872, -0.18590945), 17823},   //
      {glm::vec2(4.79133606, -0.247870877), 22638},  //
      {glm::vec2(4.98092985, -0.247938812), 22733},  //
      {glm::vec2(4.90268326, -0.152885556), 17822},  //
      {glm::vec2(4.95041943, -0.241741896), 17819},  //
  });
  TestPoly(polys, 8);

  PolygonParams().processOverlaps = false;
  PolygonParams().intermediateChecks = intermediateChecks;
};

TEST(Polygon, Comb) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0.462962955, -0.427297652), 18},      //
      {glm::vec2(0.5, -0.42592591), 17},               //
      {glm::vec2(-0.5, -0.42592591), 1283},            //
      {glm::vec2(-0.5, -0.462962955), 1269},           //
      {glm::vec2(-0.462962985, -0.461591214), 1268},   //
      {glm::vec2(-0.462962985, -0.42592591), 1282},    //
      {glm::vec2(-0.42592594, -0.42592591), 1280},     //
      {glm::vec2(-0.42592594, -0.460219473), 1266},    //
      {glm::vec2(-0.351851881, -0.45747599), 1247},    //
      {glm::vec2(-0.351851881, -0.42592591), 1257},    //
      {glm::vec2(-0.314814836, -0.42592591), 1256},    //
      {glm::vec2(-0.314814836, -0.456104249), 1245},   //
      {glm::vec2(-0.240740746, -0.453360766), 1132},   //
      {glm::vec2(-0.240740746, -0.42592591), 1143},    //
      {glm::vec2(-0.203703716, -0.42592591), 1142},    //
      {glm::vec2(-0.203703716, -0.451989025), 1130},   //
      {glm::vec2(-0.129629642, -0.449245542), 1128},   //
      {glm::vec2(-0.129629642, -0.42592591), 1141},    //
      {glm::vec2(-0.092592597, -0.42592591), 1119},    //
      {glm::vec2(-0.092592597, -0.447873801), 1111},   //
      {glm::vec2(-0.0185185187, -0.445130289), 1109},  //
      {glm::vec2(-0.0185185187, -0.42592591), 1117},   //
      {glm::vec2(0.0185185187, -0.42592591), 169},     //
      {glm::vec2(0.0185185187, -0.443758547), 162},    //
      {glm::vec2(0.0925925896, -0.441015065), 160},    //
      {glm::vec2(0.0925925896, -0.42592591), 168},     //
      {glm::vec2(0.129629627, -0.42592591), 152},      //
      {glm::vec2(0.129629627, -0.439643323), 141},     //
      {glm::vec2(0.203703731, -0.436899841), 150},     //
      {glm::vec2(0.203703731, -0.42592591), 151},      //
      {glm::vec2(0.240740761, -0.42592591), 149},      //
      {glm::vec2(0.240740761, -0.4355281), 148},       //
      {glm::vec2(0.314814806, -0.432784617), 40},      //
      {glm::vec2(0.314814806, -0.42592591), 41},       //
      {glm::vec2(0.351851851, -0.42592591), 39},       //
      {glm::vec2(0.351851851, -0.431412876), 38},      //
      {glm::vec2(0.42592591, -0.428669393), 21},       //
      {glm::vec2(0.42592591, -0.42592591), 22},        //
      {glm::vec2(0.462962955, -0.42592591), 19},       //
  });
  TestPoly(polys, 37);
}

TEST(Polygon, Comb2) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.5, -0.462962955), 2},             //
      {glm::vec2(-0.462962955, -0.461591214), 4},     //
      {glm::vec2(-0.462962955, -0.42592591), 13},     //
      {glm::vec2(-0.42592591, -0.42592591), 15},      //
      {glm::vec2(-0.42592591, -0.460219473), 7},      //
      {glm::vec2(-0.351851851, -0.45747599), 28},     //
      {glm::vec2(-0.351851851, -0.42592591), 35},     //
      {glm::vec2(-0.314814806, -0.42592591), 36},     //
      {glm::vec2(-0.314814806, -0.456104249), 30},    //
      {glm::vec2(-0.240740761, -0.453360766), 132},   //
      {glm::vec2(-0.240740761, -0.42592591), 144},    //
      {glm::vec2(-0.203703731, -0.42592591), 145},    //
      {glm::vec2(-0.203703731, -0.451989025), 134},   //
      {glm::vec2(-0.129629627, -0.449245542), 137},   //
      {glm::vec2(-0.129629627, -0.42592591), 147},    //
      {glm::vec2(-0.0925925896, -0.42592591), 164},   //
      {glm::vec2(-0.0925925896, -0.447873801), 155},  //
      {glm::vec2(-0.0185185187, -0.445130289), 158},  //
      {glm::vec2(-0.0185185187, -0.42592591), 166},   //
      {glm::vec2(0.0185185187, -0.42592591), 1113},   //
      {glm::vec2(0.0185185187, -0.443758547), 1104},  //
      {glm::vec2(0.092592597, -0.441015065), 1107},   //
      {glm::vec2(0.092592597, -0.42592591), 1116},    //
      {glm::vec2(0.129629642, -0.42592591), 1134},    //
      {glm::vec2(0.129629642, -0.439643323), 1122},   //
      {glm::vec2(0.203703716, -0.436899841), 1137},   //
      {glm::vec2(0.203703716, -0.42592591), 1138},    //
      {glm::vec2(0.240740746, -0.42592591), 1140},    //
      {glm::vec2(0.240740746, -0.4355281), 1139},     //
      {glm::vec2(0.314814836, -0.432784617), 1251},   //
      {glm::vec2(0.314814836, -0.42592591), 1252},    //
      {glm::vec2(0.351851881, -0.42592591), 1254},    //
      {glm::vec2(0.351851881, -0.431412876), 1253},   //
      {glm::vec2(0.42592594, -0.428669393), 1272},    //
      {glm::vec2(0.42592594, -0.42592591), 1273},     //
      {glm::vec2(0.462962985, -0.42592591), 1278},    //
      {glm::vec2(0.462962985, -0.427297652), 1277},   //
      {glm::vec2(0.5, -0.42592591), 1279},            //
      {glm::vec2(-0.5, -0.42592591), 12},             //
  });
  TestPoly(polys, 37);
}

TEST(Polygon, PointPoly) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0, -15.9780979), 32},            //
      {glm::vec2(5.08144999, -14.2678728), 244},  //
      {glm::vec2(4.83870935, -14.0789623), 243},  //
      {glm::vec2(4.38336992, -13.7492008), 237},  //
      {glm::vec2(4.35483837, -13.7284746), 238},  //
      {glm::vec2(4.33122683, -13.712779), 235},   //
      {glm::vec2(3.87096763, -13.3689337), 230},  //
      {glm::vec2(3.52637458, -13.1333551), 81},   //
      {glm::vec2(3.38709664, -13.0251188), 79},   //
      {glm::vec2(3.10755324, -12.8263216), 75},   //
      {glm::vec2(2.90322566, -12.6806841), 73},   //
      {glm::vec2(2.80962205, -12.6208401), 71},   //
      {glm::vec2(2.41935468, -12.3280048), 69},   //
      {glm::vec2(2.16151524, -12.1544657), 68},   //
      {glm::vec2(1.93548381, -11.9734631), 86},   //
      {glm::vec2(1.56781006, -11.7093639), 47},   //
      {glm::vec2(1.45161283, -11.6084995), 46},   //
      {glm::vec2(1.02412188, -11.2756453), 43},   //
      {glm::vec2(0.967741907, -11.2216129), 44},  //
      {glm::vec2(0.628127813, -10.9296618), 40},  //
      {glm::vec2(0, -10.9296618), 33},            //
  });
  polys.push_back({
      {glm::vec2(15, -10.9296618), 1052},  //
      {glm::vec2(15, -10.9296618), 1051},  //
      {glm::vec2(15, -10.9296618), 1053},  //
  });
  TestPoly(polys, 20);
};

TEST(Polygon, KissingZigzag) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(37.4667358, 0), 0},              //
      {glm::vec2(27.8904877, -3.04520559), 1},    //
      {glm::vec2(27.8904877, 3.04520559), 2},     //
      {glm::vec2(37.4667358, 0), 3},              //
      {glm::vec2(36.4568138, 8.64042377), 4},     //
      {glm::vec2(27.8409691, 3.46886754), 5},     //
      {glm::vec2(26.4364243, 9.39511108), 6},     //
      {glm::vec2(36.4568138, 8.64042377), 7},     //
      {glm::vec2(33.4814987, 16.8150406), 8},     //
      {glm::vec2(26.2905369, 9.79593277), 9},     //
      {glm::vec2(23.5571651, 15.2385235), 10},    //
      {glm::vec2(33.4814987, 16.8150406), 11},    //
      {glm::vec2(28.7011852, 24.0831528), 12},    //
      {glm::vec2(23.322773, 15.5948954), 13},     //
      {glm::vec2(19.4079323, 20.2604218), 14},    //
      {glm::vec2(28.7011852, 24.0831528), 15},    //
      {glm::vec2(22.3735847, 30.0529385), 16},    //
      {glm::vec2(19.0976753, 20.5531368), 17},    //
      {glm::vec2(14.2124147, 24.1900768), 18},    //
      {glm::vec2(22.3735847, 30.0529385), 19},    //
      {glm::vec2(14.8398142, 34.4025612), 20},    //
      {glm::vec2(13.8430138, 24.4033508), 21},    //
      {glm::vec2(8.2507, 26.8156395), 22},        //
      {glm::vec2(14.8398142, 34.4025612), 23},    //
      {glm::vec2(6.50603199, 36.8975296), 24},    //
      {glm::vec2(7.84207535, 26.937973), 25},     //
      {glm::vec2(1.84419155, 27.9955635), 26},    //
      {glm::vec2(6.50603199, 36.8975296), 27},    //
      {glm::vec2(-2.1784966, 37.4033508), 28},    //
      {glm::vec2(1.41836619, 28.0203648), 29},    //
      {glm::vec2(-4.66174126, 27.6662388), 30},   //
      {glm::vec2(-2.1784966, 37.4033508), 31},    //
      {glm::vec2(-10.7455816, 35.8927383), 32},   //
      {glm::vec2(-5.08180761, 27.5921688), 33},   //
      {glm::vec2(-10.916357, 25.8454189), 34},    //
      {glm::vec2(-10.7455816, 35.8927383), 35},   //
      {glm::vec2(-18.7333698, 32.4471436), 36},   //
      {glm::vec2(-11.3080206, 25.6764717), 37},   //
      {glm::vec2(-16.5824718, 22.6312675), 38},   //
      {glm::vec2(-18.7333698, 32.4471436), 39},   //
      {glm::vec2(-25.711235, 27.2523136), 40},    //
      {glm::vec2(-16.924614, 22.3765507), 41},    //
      {glm::vec2(-21.3546181, 18.1970577), 42},   //
      {glm::vec2(-25.711235, 27.2523136), 43},    //
      {glm::vec2(-31.3030052, 20.5883045), 44},   //
      {glm::vec2(-21.6287994, 17.8703041), 45},   //
      {glm::vec2(-24.9755325, 12.7818384), 46},   //
      {glm::vec2(-31.3030052, 20.5883045), 47},   //
      {glm::vec2(-35.2072144, 12.8143806), 48},   //
      {glm::vec2(-25.1669636, 12.4006672), 49},   //
      {glm::vec2(-27.2500057, 6.67755318), 50},   //
      {glm::vec2(-35.2072144, 12.8143806), 51},   //
      {glm::vec2(-37.213398, 4.34962463), 52},    //
      {glm::vec2(-27.3483734, 6.26250458), 53},   //
      {glm::vec2(-28.0554276, 0.213274717), 54},  //
      {glm::vec2(-37.213398, 4.34962463), 55},    //
      {glm::vec2(-37.213398, -4.34962177), 56},   //
      {glm::vec2(-28.0554276, -0.21327281), 57},  //
      {glm::vec2(-27.3483734, -6.26250267), 58},  //
      {glm::vec2(-37.213398, -4.34962177), 59},   //
      {glm::vec2(-35.2072144, -12.8143787), 60},  //
      {glm::vec2(-27.2500057, -6.67755222), 61},  //
      {glm::vec2(-25.1669636, -12.4006662), 62},  //
      {glm::vec2(-35.2072144, -12.8143787), 63},  //
      {glm::vec2(-31.3029995, -20.5883102), 64},  //
      {glm::vec2(-24.9755306, -12.7818432), 65},  //
      {glm::vec2(-21.6287937, -17.8703079), 66},  //
      {glm::vec2(-31.3029995, -20.5883102), 67},  //
      {glm::vec2(-25.7112312, -27.2523193), 68},  //
      {glm::vec2(-21.3546143, -18.1970615), 69},  //
      {glm::vec2(-16.9246101, -22.3765545), 70},  //
      {glm::vec2(-25.7112312, -27.2523193), 71},  //
      {glm::vec2(-18.7333641, -32.4471474), 72},  //
      {glm::vec2(-16.5824661, -22.6312695), 73},  //
      {glm::vec2(-11.3080158, -25.6764736), 74},  //
      {glm::vec2(-18.7333641, -32.4471474), 75},  //
      {glm::vec2(-10.7455835, -35.8927383), 76},  //
      {glm::vec2(-10.9163589, -25.8454189), 77},  //
      {glm::vec2(-5.08180904, -27.5921688), 78},  //
      {glm::vec2(-10.7455835, -35.8927383), 79},  //
      {glm::vec2(-2.17849016, -37.4033508), 80},  //
      {glm::vec2(-4.66173601, -27.6662388), 81},  //
      {glm::vec2(1.41837108, -28.0203648), 82},   //
      {glm::vec2(-2.17849016, -37.4033508), 83},  //
      {glm::vec2(6.50602913, -36.8975296), 84},   //
      {glm::vec2(1.84418964, -27.9955635), 85},   //
      {glm::vec2(7.84207344, -26.937973), 86},    //
      {glm::vec2(6.50602913, -36.8975296), 87},   //
      {glm::vec2(14.8398247, -34.4025574), 88},   //
      {glm::vec2(8.25070763, -26.8156357), 89},   //
      {glm::vec2(13.8430195, -24.403347), 90},    //
      {glm::vec2(14.8398247, -34.4025574), 91},   //
      {glm::vec2(22.3735847, -30.0529385), 92},   //
      {glm::vec2(14.2124147, -24.1900768), 93},   //
      {glm::vec2(19.0976753, -20.5531368), 94},   //
      {glm::vec2(22.3735847, -30.0529385), 95},   //
      {glm::vec2(28.7011795, -24.0831585), 96},   //
      {glm::vec2(19.4079285, -20.2604256), 97},   //
      {glm::vec2(23.3227692, -15.5949011), 98},   //
      {glm::vec2(28.7011795, -24.0831585), 99},   //
      {glm::vec2(33.4815025, -16.8150368), 100},  //
      {glm::vec2(23.5571671, -15.2385206), 101},  //
      {glm::vec2(26.2905388, -9.79592991), 102},  //
      {glm::vec2(33.4815025, -16.8150368), 103},  //
      {glm::vec2(36.4568138, -8.64042759), 104},  //
      {glm::vec2(26.4364243, -9.39511299), 105},  //
      {glm::vec2(27.8409691, -3.46886992), 106},  //
      {glm::vec2(36.4568138, -8.64042759), 107},  //
  });
  TestPoly(polys, 106);
}

TEST(Polygon, Sponge) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(-0.5, -0.5), 22},                  //
      {glm::vec2(-0.388888896, -0.388888896), 23},  //
      {glm::vec2(-0.388888896, -0.388888896), 24},  //
      {glm::vec2(-0.388888896, -0.277777791), 26},  //
      {glm::vec2(-0.388888896, -0.277777791), 27},  //
      {glm::vec2(-0.277777791, -0.277777791), 30},  //
      {glm::vec2(-0.166666657, -0.166666672), 41},  //
      {glm::vec2(-0.166666672, -0.166666672), 42},  //
      {glm::vec2(-0.166666672, -0.166666672), 43},  //
      {glm::vec2(-0.166666672, 0.166666672), 76},   //
      {glm::vec2(-0.166666672, 0.166666672), 77},   //
      {glm::vec2(0.166666657, 0.166666672), 142},   //
      {glm::vec2(0.277777791, 0.277777791), 153},   //
      {glm::vec2(0.277777791, 0.277777791), 154},   //
      {glm::vec2(0.277777791, 0.277777791), 155},   //
      {glm::vec2(0.277777791, 0.388888896), 156},   //
      {glm::vec2(0.277777791, 0.388888896), 157},   //
      {glm::vec2(0.388888896, 0.388888896), 161},   //
      {glm::vec2(0.388888896, 0.388888896), 160},   //
      {glm::vec2(0.5, 0.5), 163},                   //
      {glm::vec2(-0.5, 0.5), 69},                   //
  });
  polys.push_back({
      {glm::vec2(-0.388888896, -0.055555556), 33},  //
      {glm::vec2(-0.388888896, -0.055555556), 34},  //
      {glm::vec2(-0.388888896, 0.055555556), 63},   //
      {glm::vec2(-0.388888896, 0.055555556), 64},   //
      {glm::vec2(-0.277777791, 0.055555556), 66},   //
      {glm::vec2(-0.277777791, 0.055555556), 65},   //
      {glm::vec2(-0.277777791, -0.055555556), 35},  //
      {glm::vec2(-0.277777791, -0.055555556), 36},  //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, 0.277777791), 72},  //
      {glm::vec2(-0.277777791, 0.277777791), 73},  //
      {glm::vec2(-0.388888896, 0.277777791), 67},  //
      {glm::vec2(-0.388888896, 0.277777791), 68},  //
      {glm::vec2(-0.388888896, 0.388888896), 70},  //
      {glm::vec2(-0.388888896, 0.388888896), 71},  //
      {glm::vec2(-0.277777791, 0.388888896), 74},  //
      {glm::vec2(-0.277777791, 0.388888896), 75},  //
  });
  polys.push_back({
      {glm::vec2(-0.055555556, 0.277777791), 78},  //
      {glm::vec2(-0.055555556, 0.277777791), 79},  //
      {glm::vec2(-0.055555556, 0.388888896), 80},  //
      {glm::vec2(-0.055555556, 0.388888896), 81},  //
      {glm::vec2(0.055555556, 0.388888896), 147},  //
      {glm::vec2(0.055555556, 0.388888896), 148},  //
      {glm::vec2(0.055555556, 0.277777791), 146},  //
      {glm::vec2(0.055555556, 0.277777791), 145},  //
  });
  TestPoly(polys, 49);
}

TEST(Polygon, SquareHoles) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0.388888896, -0.277777791), 10},  //
      {glm::vec2(0.388888896, -0.388888896), 8},   //
      {glm::vec2(0.277777791, -0.388888896), 9},   //
      {glm::vec2(0.277777791, -0.277777791), 11},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, -0.055555556), 14},  //
      {glm::vec2(0.277777791, -0.055555556), 12},  //
      {glm::vec2(0.277777791, 0.055555556), 16},   //
      {glm::vec2(0.388888896, 0.055555556), 15},   //
  });
  polys.push_back({
      {glm::vec2(0.055555556, -0.277777791), 25},   //
      {glm::vec2(0.055555556, -0.388888896), 13},   //
      {glm::vec2(-0.055555556, -0.388888896), 23},  //
      {glm::vec2(-0.055555556, -0.277777791), 24},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, 0.277777791), 18},  //
      {glm::vec2(0.277777791, 0.277777791), 17},  //
      {glm::vec2(0.277777791, 0.388888896), 20},  //
      {glm::vec2(0.388888896, 0.388888896), 19},  //
  });
  polys.push_back({
      {glm::vec2(0.166666672, 0.166666672), 32},    //
      {glm::vec2(0.166666672, -0.166666672), 21},   //
      {glm::vec2(-0.166666672, -0.166666672), 26},  //
      {glm::vec2(-0.166666672, 0.166666672), 33},   //
  });
  polys.push_back({
      {glm::vec2(0.055555556, 0.388888896), 35},   //
      {glm::vec2(0.055555556, 0.277777791), 22},   //
      {glm::vec2(-0.055555556, 0.277777791), 34},  //
      {glm::vec2(-0.055555556, 0.388888896), 36},  //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, -0.277777791), 30},  //
      {glm::vec2(-0.277777791, -0.388888896), 27},  //
      {glm::vec2(-0.388888896, -0.388888896), 28},  //
      {glm::vec2(-0.388888896, -0.277777791), 29},  //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, -0.055555556), 37},  //
      {glm::vec2(-0.388888896, -0.055555556), 31},  //
      {glm::vec2(-0.388888896, 0.055555556), 39},   //
      {glm::vec2(-0.277777791, 0.055555556), 38},   //
  });
  polys.push_back({
      {glm::vec2(-0.277777791, 0.388888896), 42},  //
      {glm::vec2(-0.277777791, 0.277777791), 40},  //
      {glm::vec2(-0.388888896, 0.277777791), 41},  //
      {glm::vec2(-0.388888896, 0.388888896), 43},  //
  });
  polys.push_back({
      {glm::vec2(-0.5, -0.5), 1},  //
      {glm::vec2(0.5, -0.5), 0},   //
      {glm::vec2(0.5, 0.5), 3},    //
      {glm::vec2(-0.5, 0.5), 2},   //
  });
  TestPoly(polys, 56);
}

TEST(Polygon, BigSponge) {
  PolygonsIdx polys;
  polys.push_back({
      {glm::vec2(0.5, 0.5), 1},                        //
      {glm::vec2(0.487654328, 0.487654328), 13834},    //
      {glm::vec2(0.487654328, 0.487654328), 83869},    //
      {glm::vec2(0.487654328, 0.475308657), 83868},    //
      {glm::vec2(0.475308657, 0.475308657), 83870},    //
      {glm::vec2(0.475308657, 0.475308657), 13835},    //
      {glm::vec2(0.462962955, 0.462962955), 13833},    //
      {glm::vec2(0.462962955, 0.42592594), 83857},     //
      {glm::vec2(0.42592594, 0.42592594), 13830},      //
      {glm::vec2(0.413580239, 0.413580239), 13832},    //
      {glm::vec2(0.413580239, 0.413580239), 83861},    //
      {glm::vec2(0.413580239, 0.401234567), 83862},    //
      {glm::vec2(0.401234567, 0.401234567), 83863},    //
      {glm::vec2(0.401234567, 0.401234567), 13831},    //
      {glm::vec2(0.388888896, 0.388888896), 13836},    //
      {glm::vec2(0.388888896, 0.388888896), 83826},    //
      {glm::vec2(0.388888896, 0.277777791), 83834},    //
      {glm::vec2(0.277777791, 0.277777791), 83842},    //
      {glm::vec2(0.277777791, 0.277777791), 13829},    //
      {glm::vec2(0.265432119, 0.265432119), 13827},    //
      {glm::vec2(0.265432119, 0.265432119), 83840},    //
      {glm::vec2(0.265432119, 0.253086448), 83839},    //
      {glm::vec2(0.253086448, 0.253086448), 83841},    //
      {glm::vec2(0.253086448, 0.253086448), 13828},    //
      {glm::vec2(0.240740746, 0.240740746), 13824},    //
      {glm::vec2(0.240740761, 0.240740746), 83783},    //
      {glm::vec2(0.240740761, 0.203703716), 83781},    //
      {glm::vec2(0.203703731, 0.203703716), 83784},    //
      {glm::vec2(0.203703731, 0.203703746), 13825},    //
      {glm::vec2(0.19135803, 0.19135803), 13823},      //
      {glm::vec2(0.19135803, 0.19135803), 83782},      //
      {glm::vec2(0.19135803, 0.179012358), 83776},     //
      {glm::vec2(0.179012358, 0.179012358), 83785},    //
      {glm::vec2(0.179012358, 0.179012358), 13826},    //
      {glm::vec2(0.166666672, 0.166666657), 13822},    //
      {glm::vec2(0.166666672, -0.166666672), 83546},   //
      {glm::vec2(-0.166666672, -0.166666672), 83871},  //
      {glm::vec2(-0.166666672, -0.166666657), 13837},  //
      {glm::vec2(-0.179012358, -0.179012358), 13818},  //
      {glm::vec2(-0.179012358, -0.19135803), 83565},   //
      {glm::vec2(-0.19135803, -0.19135803), 13820},    //
      {glm::vec2(-0.203703731, -0.203703731), 13821},  //
      {glm::vec2(-0.203703716, -0.203703731), 83562},  //
      {glm::vec2(-0.203703716, -0.240740761), 83563},  //
      {glm::vec2(-0.240740746, -0.240740761), 83564},  //
      {glm::vec2(-0.240740746, -0.240740731), 13819},  //
      {glm::vec2(-0.253086448, -0.253086448), 13816},  //
      {glm::vec2(-0.253086448, -0.265432119), 83519},  //
      {glm::vec2(-0.265432119, -0.265432119), 13817},  //
      {glm::vec2(-0.277777791, -0.277777791), 13815},  //
      {glm::vec2(-0.277777791, -0.388888896), 83485},  //
      {glm::vec2(-0.388888896, -0.388888896), 13808},  //
      {glm::vec2(-0.401234567, -0.401234567), 13814},  //
      {glm::vec2(-0.401234567, -0.413580239), 83509},  //
      {glm::vec2(-0.413580239, -0.413580239), 13813},  //
      {glm::vec2(-0.42592594, -0.42592594), 13809},    //
      {glm::vec2(-0.42592594, -0.462962955), 83504},   //
      {glm::vec2(-0.462962955, -0.462962955), 13810},  //
      {glm::vec2(-0.475308657, -0.475308657), 13811},  //
      {glm::vec2(-0.475308657, -0.487654328), 83505},  //
      {glm::vec2(-0.487654328, -0.487654328), 13812},  //
      {glm::vec2(-0.5, -0.5), 4},                      //
      {glm::vec2(0.5, -0.5), 0},                       //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.487654328), 82702},  //
      {glm::vec2(0.475308657, -0.487654328), 82704},  //
      {glm::vec2(0.475308657, -0.475308657), 82705},  //
      {glm::vec2(0.487654328, -0.475308657), 82703},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.475308657), 82706},  //
      {glm::vec2(0.450617284, -0.487654328), 82707},  //
      {glm::vec2(0.438271612, -0.487654328), 82709},  //
      {glm::vec2(0.438271612, -0.475308657), 82708},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.462962955), 82710},  //
      {glm::vec2(0.42592591, -0.462962955), 82723},   //
      {glm::vec2(0.42592591, -0.42592591), 82735},    //
      {glm::vec2(0.462962955, -0.42592591), 82714},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.450617284), 82711},  //
      {glm::vec2(0.475308657, -0.450617284), 82713},  //
      {glm::vec2(0.475308657, -0.438271612), 82715},  //
      {glm::vec2(0.487654328, -0.438271612), 82712},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.475308657), 82716},  //
      {glm::vec2(0.413580239, -0.487654328), 82717},  //
      {glm::vec2(0.401234567, -0.487654328), 82718},  //
      {glm::vec2(0.401234567, -0.475308657), 82719},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.475308657), 82720},  //
      {glm::vec2(0.376543224, -0.487654328), 82721},  //
      {glm::vec2(0.364197552, -0.487654328), 82756},  //
      {glm::vec2(0.364197552, -0.475308657), 82722},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.438271612), 82724},  //
      {glm::vec2(0.413580239, -0.450617284), 82725},  //
      {glm::vec2(0.401234567, -0.450617284), 82726},  //
      {glm::vec2(0.401234567, -0.438271612), 82727},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.450617284), 82728},  //
      {glm::vec2(0.364197552, -0.450617284), 82762},  //
      {glm::vec2(0.364197552, -0.438271612), 82730},  //
      {glm::vec2(0.376543224, -0.438271612), 82729},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.413580239), 82731},  //
      {glm::vec2(0.475308657, -0.413580239), 82733},  //
      {glm::vec2(0.475308657, -0.401234567), 82734},  //
      {glm::vec2(0.487654328, -0.401234567), 82732},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.413580239), 82736},  //
      {glm::vec2(0.438271612, -0.413580239), 82739},  //
      {glm::vec2(0.438271612, -0.401234567), 82738},  //
      {glm::vec2(0.450617284, -0.401234567), 82737},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.376543224), 82740},  //
      {glm::vec2(0.475308657, -0.376543224), 82742},  //
      {glm::vec2(0.475308657, -0.364197552), 82799},  //
      {glm::vec2(0.487654328, -0.364197552), 82741},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.376543224), 82743},  //
      {glm::vec2(0.438271612, -0.376543224), 82745},  //
      {glm::vec2(0.438271612, -0.364197552), 82801},  //
      {glm::vec2(0.450617284, -0.364197552), 82744},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.413580239), 82746},  //
      {glm::vec2(0.401234567, -0.413580239), 82748},  //
      {glm::vec2(0.401234567, -0.401234567), 82749},  //
      {glm::vec2(0.413580239, -0.401234567), 82747},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.413580239), 82750},  //
      {glm::vec2(0.364197552, -0.413580239), 82781},  //
      {glm::vec2(0.364197552, -0.401234567), 82752},  //
      {glm::vec2(0.376543224, -0.401234567), 82751},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.364197552), 82753},  //
      {glm::vec2(0.413580239, -0.376543224), 82754},  //
      {glm::vec2(0.401234567, -0.376543224), 82755},  //
      {glm::vec2(0.401234567, -0.364197552), 82808},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.475308657), 82757},  //
      {glm::vec2(0.339506179, -0.487654328), 82758},  //
      {glm::vec2(0.327160507, -0.487654328), 82759},  //
      {glm::vec2(0.327160507, -0.475308657), 82760},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, -0.462962955), 82761},  //
      {glm::vec2(0.314814806, -0.462962955), 82764},  //
      {glm::vec2(0.314814806, -0.42592591), 82783},   //
      {glm::vec2(0.351851851, -0.42592591), 82763},   //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.475308657), 82765},  //
      {glm::vec2(0.302469134, -0.487654328), 82766},  //
      {glm::vec2(0.290123463, -0.487654328), 82767},  //
      {glm::vec2(0.290123463, -0.475308657), 82768},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.487654328), 82769},  //
      {glm::vec2(0.253086448, -0.487654328), 82772},  //
      {glm::vec2(0.253086448, -0.475308657), 82771},  //
      {glm::vec2(0.265432119, -0.475308657), 82770},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.450617284), 82773},  //
      {glm::vec2(0.290123463, -0.450617284), 82775},  //
      {glm::vec2(0.290123463, -0.438271612), 82776},  //
      {glm::vec2(0.302469134, -0.438271612), 82774},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.450617284), 82777},  //
      {glm::vec2(0.253086448, -0.450617284), 82779},  //
      {glm::vec2(0.253086448, -0.438271612), 82780},  //
      {glm::vec2(0.265432119, -0.438271612), 82778},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.401234567), 82782},  //
      {glm::vec2(0.339506179, -0.413580239), 82784},  //
      {glm::vec2(0.327160507, -0.413580239), 82785},  //
      {glm::vec2(0.327160507, -0.401234567), 82786},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, -0.388888896), 82787},  //
      {glm::vec2(0.277777791, -0.388888896), 82844},  //
      {glm::vec2(0.277777791, -0.277777791), 82848},  //
      {glm::vec2(0.388888896, -0.277777791), 82814},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.413580239), 82788},  //
      {glm::vec2(0.290123463, -0.413580239), 82790},  //
      {glm::vec2(0.290123463, -0.401234567), 82791},  //
      {glm::vec2(0.302469134, -0.401234567), 82789},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.413580239), 82792},  //
      {glm::vec2(0.253086448, -0.413580239), 82795},  //
      {glm::vec2(0.253086448, -0.401234567), 82794},  //
      {glm::vec2(0.265432119, -0.401234567), 82793},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.376543224), 82796},  //
      {glm::vec2(0.253086448, -0.376543224), 82798},  //
      {glm::vec2(0.253086448, -0.364197552), 82842},  //
      {glm::vec2(0.265432119, -0.364197552), 82797},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.339506179), 82800},  //
      {glm::vec2(0.475308657, -0.339506179), 82804},  //
      {glm::vec2(0.475308657, -0.327160507), 82805},  //
      {glm::vec2(0.487654328, -0.327160507), 82803},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.351851851), 82802},  //
      {glm::vec2(0.42592591, -0.351851851), 82810},   //
      {glm::vec2(0.42592591, -0.314814806), 82807},   //
      {glm::vec2(0.462962955, -0.314814806), 82806},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.339506179), 82809},  //
      {glm::vec2(0.401234567, -0.339506179), 82812},  //
      {glm::vec2(0.401234567, -0.327160507), 82813},  //
      {glm::vec2(0.413580239, -0.327160507), 82811},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.302469134), 82815},  //
      {glm::vec2(0.475308657, -0.302469134), 82818},  //
      {glm::vec2(0.475308657, -0.290123463), 82817},  //
      {glm::vec2(0.487654328, -0.290123463), 82816},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.302469134), 82819},  //
      {glm::vec2(0.438271612, -0.302469134), 82822},  //
      {glm::vec2(0.438271612, -0.290123463), 82821},  //
      {glm::vec2(0.450617284, -0.290123463), 82820},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.265432119), 82823},  //
      {glm::vec2(0.475308657, -0.265432119), 82825},  //
      {glm::vec2(0.475308657, -0.253086448), 82826},  //
      {glm::vec2(0.487654328, -0.253086448), 82824},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.253086448), 82827},  //
      {glm::vec2(0.450617284, -0.265432119), 82828},  //
      {glm::vec2(0.438271612, -0.265432119), 82829},  //
      {glm::vec2(0.438271612, -0.253086448), 82830},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.290123463), 82831},  //
      {glm::vec2(0.413580239, -0.302469134), 82832},  //
      {glm::vec2(0.401234567, -0.302469134), 82834},  //
      {glm::vec2(0.401234567, -0.290123463), 82833},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.253086448), 82835},  //
      {glm::vec2(0.413580239, -0.265432119), 82836},  //
      {glm::vec2(0.401234567, -0.265432119), 82837},  //
      {glm::vec2(0.401234567, -0.253086448), 82838},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.253086448), 82839},  //
      {glm::vec2(0.376543224, -0.265432119), 82840},  //
      {glm::vec2(0.364197552, -0.265432119), 82849},  //
      {glm::vec2(0.364197552, -0.253086448), 82841},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.339506179), 82843},  //
      {glm::vec2(0.253086448, -0.339506179), 82847},  //
      {glm::vec2(0.253086448, -0.327160507), 82846},  //
      {glm::vec2(0.265432119, -0.327160507), 82845},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.253086448), 82850},  //
      {glm::vec2(0.339506179, -0.265432119), 82851},  //
      {glm::vec2(0.327160507, -0.265432119), 82852},  //
      {glm::vec2(0.327160507, -0.253086448), 82853},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.302469134), 82854},  //
      {glm::vec2(0.253086448, -0.302469134), 82857},  //
      {glm::vec2(0.253086448, -0.290123463), 82856},  //
      {glm::vec2(0.265432119, -0.290123463), 82855},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.253086448), 82858},  //
      {glm::vec2(0.302469134, -0.265432119), 82859},  //
      {glm::vec2(0.290123463, -0.265432119), 82860},  //
      {glm::vec2(0.290123463, -0.253086448), 82861},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.265432119), 82862},  //
      {glm::vec2(0.253086448, -0.265432119), 82864},  //
      {glm::vec2(0.253086448, -0.253086448), 82865},  //
      {glm::vec2(0.265432119, -0.253086448), 82863},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.487654328), 82866},  //
      {glm::vec2(0.216049403, -0.487654328), 82868},  //
      {glm::vec2(0.216049403, -0.475308657), 82869},  //
      {glm::vec2(0.228395075, -0.475308657), 82867},  //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.462962955), 82870},  //
      {glm::vec2(0.203703731, -0.462962955), 82875},  //
      {glm::vec2(0.203703731, -0.42592591), 82890},   //
      {glm::vec2(0.240740761, -0.42592591), 82874},   //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.475308657), 82871},   //
      {glm::vec2(0.19135803, -0.487654328), 82872},   //
      {glm::vec2(0.179012358, -0.487654328), 82879},  //
      {glm::vec2(0.179012358, -0.475308657), 82873},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.450617284), 82876},   //
      {glm::vec2(0.179012358, -0.450617284), 82884},  //
      {glm::vec2(0.179012358, -0.438271612), 82878},  //
      {glm::vec2(0.19135803, -0.438271612), 82877},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.475308657), 82880},  //
      {glm::vec2(0.154320985, -0.487654328), 82881},  //
      {glm::vec2(0.141975313, -0.487654328), 82882},  //
      {glm::vec2(0.141975313, -0.475308657), 82883},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.438271612), 82885},  //
      {glm::vec2(0.154320985, -0.450617284), 82886},  //
      {glm::vec2(0.141975313, -0.450617284), 82887},  //
      {glm::vec2(0.141975313, -0.438271612), 82889},  //
  });
  polys.push_back({
      {glm::vec2(0.129629627, -0.42592591), 82888},    //
      {glm::vec2(0.129629627, -0.462962955), 82916},   //
      {glm::vec2(0.0925925896, -0.462962955), 82921},  //
      {glm::vec2(0.0925925896, -0.42592591), 82936},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.413580239), 82891},  //
      {glm::vec2(0.216049403, -0.413580239), 82893},  //
      {glm::vec2(0.216049403, -0.401234567), 82894},  //
      {glm::vec2(0.228395075, -0.401234567), 82892},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.413580239), 82895},   //
      {glm::vec2(0.179012358, -0.413580239), 82903},  //
      {glm::vec2(0.179012358, -0.401234567), 82897},  //
      {glm::vec2(0.19135803, -0.401234567), 82896},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.376543224), 82898},  //
      {glm::vec2(0.216049403, -0.376543224), 82900},  //
      {glm::vec2(0.216049403, -0.364197552), 82956},  //
      {glm::vec2(0.228395075, -0.364197552), 82899},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.376543224), 82901},   //
      {glm::vec2(0.179012358, -0.376543224), 82908},  //
      {glm::vec2(0.179012358, -0.364197552), 82958},  //
      {glm::vec2(0.19135803, -0.364197552), 82902},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.401234567), 82904},  //
      {glm::vec2(0.154320985, -0.413580239), 82905},  //
      {glm::vec2(0.141975313, -0.413580239), 82906},  //
      {glm::vec2(0.141975313, -0.401234567), 82907},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.364197552), 82909},  //
      {glm::vec2(0.154320985, -0.376543224), 82910},  //
      {glm::vec2(0.141975313, -0.376543224), 82911},  //
      {glm::vec2(0.141975313, -0.364197552), 82965},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.475308657), 82912},  //
      {glm::vec2(0.117283955, -0.487654328), 82913},  //
      {glm::vec2(0.104938276, -0.487654328), 82915},  //
      {glm::vec2(0.104938276, -0.475308657), 82914},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.475308657), 82917},  //
      {glm::vec2(0.0802469105, -0.487654328), 82918},  //
      {glm::vec2(0.0679012313, -0.487654328), 82919},  //
      {glm::vec2(0.0679012313, -0.475308657), 82920},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.438271612), 82922},  //
      {glm::vec2(0.0802469105, -0.450617284), 82923},  //
      {glm::vec2(0.0679012313, -0.450617284), 82924},  //
      {glm::vec2(0.0679012313, -0.438271612), 82925},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.475308657), 82926},  //
      {glm::vec2(0.0432098769, -0.487654328), 82927},  //
      {glm::vec2(0.0308641978, -0.487654328), 82928},  //
      {glm::vec2(0.0308641978, -0.475308657), 82929},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.475308657), 82930},   //
      {glm::vec2(0.00617283955, -0.487654328), 83272},   //
      {glm::vec2(-0.00617283955, -0.487654328), 83273},  //
      {glm::vec2(-0.00617283955, -0.475308657), 83274},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.450617284), 82931},  //
      {glm::vec2(0.0308641978, -0.450617284), 82933},  //
      {glm::vec2(0.0308641978, -0.438271612), 82934},  //
      {glm::vec2(0.0432098769, -0.438271612), 82932},  //
  });
  polys.push_back({
      {glm::vec2(0.0185185187, -0.42592591), 82935},    //
      {glm::vec2(0.0185185187, -0.462962955), 83275},   //
      {glm::vec2(-0.0185185187, -0.462962955), 83280},  //
      {glm::vec2(-0.0185185187, -0.42592591), 83299},   //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.413580239), 82937},  //
      {glm::vec2(0.104938276, -0.413580239), 82940},  //
      {glm::vec2(0.104938276, -0.401234567), 82939},  //
      {glm::vec2(0.117283955, -0.401234567), 82938},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.413580239), 82941},  //
      {glm::vec2(0.0679012313, -0.413580239), 82943},  //
      {glm::vec2(0.0679012313, -0.401234567), 82944},  //
      {glm::vec2(0.0802469105, -0.401234567), 82942},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.376543224), 82945},  //
      {glm::vec2(0.104938276, -0.376543224), 82947},  //
      {glm::vec2(0.104938276, -0.364197552), 82996},  //
      {glm::vec2(0.117283955, -0.364197552), 82946},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.364197552), 82948},  //
      {glm::vec2(0.0802469105, -0.376543224), 82949},  //
      {glm::vec2(0.0679012313, -0.376543224), 82950},  //
      {glm::vec2(0.0679012313, -0.364197552), 82998},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.413580239), 82951},  //
      {glm::vec2(0.0308641978, -0.413580239), 82953},  //
      {glm::vec2(0.0308641978, -0.401234567), 82954},  //
      {glm::vec2(0.0432098769, -0.401234567), 82952},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.401234567), 82955},   //
      {glm::vec2(0.00617283955, -0.413580239), 83300},   //
      {glm::vec2(-0.00617283955, -0.413580239), 83301},  //
      {glm::vec2(-0.00617283955, -0.401234567), 83302},  //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.351851851), 82957},  //
      {glm::vec2(0.203703731, -0.351851851), 82962},  //
      {glm::vec2(0.203703731, -0.314814806), 82961},  //
      {glm::vec2(0.240740761, -0.314814806), 82960},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.339506179), 82959},   //
      {glm::vec2(0.179012358, -0.339506179), 82967},  //
      {glm::vec2(0.179012358, -0.327160507), 82964},  //
      {glm::vec2(0.19135803, -0.327160507), 82963},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.339506179), 82966},  //
      {glm::vec2(0.141975313, -0.339506179), 82969},  //
      {glm::vec2(0.141975313, -0.327160507), 82970},  //
      {glm::vec2(0.154320985, -0.327160507), 82968},  //
  });
  polys.push_back({
      {glm::vec2(0.129629627, -0.314814806), 82971},   //
      {glm::vec2(0.129629627, -0.351851851), 82997},   //
      {glm::vec2(0.0925925896, -0.351851851), 83000},  //
      {glm::vec2(0.0925925896, -0.314814806), 83001},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.302469134), 82972},  //
      {glm::vec2(0.216049403, -0.302469134), 82975},  //
      {glm::vec2(0.216049403, -0.290123463), 82974},  //
      {glm::vec2(0.228395075, -0.290123463), 82973},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.302469134), 82976},   //
      {glm::vec2(0.179012358, -0.302469134), 82986},  //
      {glm::vec2(0.179012358, -0.290123463), 82978},  //
      {glm::vec2(0.19135803, -0.290123463), 82977},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.265432119), 82979},  //
      {glm::vec2(0.216049403, -0.265432119), 82981},  //
      {glm::vec2(0.216049403, -0.253086448), 82982},  //
      {glm::vec2(0.228395075, -0.253086448), 82980},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.253086448), 82983},   //
      {glm::vec2(0.19135803, -0.265432119), 82984},   //
      {glm::vec2(0.179012358, -0.265432119), 82991},  //
      {glm::vec2(0.179012358, -0.253086448), 82985},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.290123463), 82987},  //
      {glm::vec2(0.154320985, -0.302469134), 82988},  //
      {glm::vec2(0.141975313, -0.302469134), 82990},  //
      {glm::vec2(0.141975313, -0.290123463), 82989},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.253086448), 82992},  //
      {glm::vec2(0.154320985, -0.265432119), 82993},  //
      {glm::vec2(0.141975313, -0.265432119), 82994},  //
      {glm::vec2(0.141975313, -0.253086448), 82995},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.339506179), 82999},  //
      {glm::vec2(0.0679012313, -0.339506179), 83003},  //
      {glm::vec2(0.0679012313, -0.327160507), 83004},  //
      {glm::vec2(0.0802469105, -0.327160507), 83002},  //
  });
  polys.push_back({
      {glm::vec2(0.055555556, -0.277777791), 83005},   //
      {glm::vec2(0.055555556, -0.388888896), 83307},   //
      {glm::vec2(-0.055555556, -0.388888896), 83369},  //
      {glm::vec2(-0.055555556, -0.277777791), 83379},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.302469134), 83006},  //
      {glm::vec2(0.104938276, -0.302469134), 83009},  //
      {glm::vec2(0.104938276, -0.290123463), 83008},  //
      {glm::vec2(0.117283955, -0.290123463), 83007},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.290123463), 83010},  //
      {glm::vec2(0.0802469105, -0.302469134), 83011},  //
      {glm::vec2(0.0679012313, -0.302469134), 83013},  //
      {glm::vec2(0.0679012313, -0.290123463), 83012},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.253086448), 83014},  //
      {glm::vec2(0.117283955, -0.265432119), 83015},  //
      {glm::vec2(0.104938276, -0.265432119), 83016},  //
      {glm::vec2(0.104938276, -0.253086448), 83017},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.253086448), 83018},  //
      {glm::vec2(0.0802469105, -0.265432119), 83019},  //
      {glm::vec2(0.0679012313, -0.265432119), 83020},  //
      {glm::vec2(0.0679012313, -0.253086448), 83021},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.253086448), 83022},  //
      {glm::vec2(0.0432098769, -0.265432119), 83023},  //
      {glm::vec2(0.0308641978, -0.265432119), 83024},  //
      {glm::vec2(0.0308641978, -0.253086448), 83025},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.253086448), 83026},   //
      {glm::vec2(0.00617283955, -0.265432119), 83380},   //
      {glm::vec2(-0.00617283955, -0.265432119), 83381},  //
      {glm::vec2(-0.00617283955, -0.253086448), 83382},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.228395075), 83027},  //
      {glm::vec2(0.475308657, -0.228395075), 83029},  //
      {glm::vec2(0.475308657, -0.216049403), 83030},  //
      {glm::vec2(0.487654328, -0.216049403), 83028},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.203703731), 83031},  //
      {glm::vec2(0.462962955, -0.240740761), 83032},  //
      {glm::vec2(0.42592591, -0.240740761), 83040},   //
      {glm::vec2(0.42592591, -0.203703731), 83036},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.19135803), 83033},   //
      {glm::vec2(0.475308657, -0.19135803), 83035},   //
      {glm::vec2(0.475308657, -0.179012358), 83053},  //
      {glm::vec2(0.487654328, -0.179012358), 83034},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.19135803), 83037},   //
      {glm::vec2(0.438271612, -0.19135803), 83039},   //
      {glm::vec2(0.438271612, -0.179012358), 83055},  //
      {glm::vec2(0.450617284, -0.179012358), 83038},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.216049403), 83041},  //
      {glm::vec2(0.413580239, -0.228395075), 83042},  //
      {glm::vec2(0.401234567, -0.228395075), 83043},  //
      {glm::vec2(0.401234567, -0.216049403), 83044},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.216049403), 83045},  //
      {glm::vec2(0.376543224, -0.228395075), 83046},  //
      {glm::vec2(0.364197552, -0.228395075), 83073},  //
      {glm::vec2(0.364197552, -0.216049403), 83047},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.179012358), 83048},  //
      {glm::vec2(0.413580239, -0.19135803), 83049},   //
      {glm::vec2(0.401234567, -0.19135803), 83050},   //
      {glm::vec2(0.401234567, -0.179012358), 83064},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.19135803), 83051},   //
      {glm::vec2(0.364197552, -0.19135803), 83077},   //
      {glm::vec2(0.364197552, -0.179012358), 83066},  //
      {glm::vec2(0.376543224, -0.179012358), 83052},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.154320985), 83054},  //
      {glm::vec2(0.475308657, -0.154320985), 83058},  //
      {glm::vec2(0.475308657, -0.141975313), 83059},  //
      {glm::vec2(0.487654328, -0.141975313), 83057},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.154320985), 83056},  //
      {glm::vec2(0.438271612, -0.154320985), 83062},  //
      {glm::vec2(0.438271612, -0.141975313), 83061},  //
      {glm::vec2(0.450617284, -0.141975313), 83060},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.129629627), 83063},   //
      {glm::vec2(0.42592591, -0.129629627), 83127},    //
      {glm::vec2(0.42592591, -0.0925925896), 83118},   //
      {glm::vec2(0.462962955, -0.0925925896), 83116},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.154320985), 83065},  //
      {glm::vec2(0.401234567, -0.154320985), 83069},  //
      {glm::vec2(0.401234567, -0.141975313), 83070},  //
      {glm::vec2(0.413580239, -0.141975313), 83068},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.154320985), 83067},  //
      {glm::vec2(0.364197552, -0.154320985), 83098},  //
      {glm::vec2(0.364197552, -0.141975313), 83072},  //
      {glm::vec2(0.376543224, -0.141975313), 83071},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, -0.203703731), 83074},  //
      {glm::vec2(0.351851851, -0.240740761), 83075},  //
      {glm::vec2(0.314814806, -0.240740761), 83076},  //
      {glm::vec2(0.314814806, -0.203703731), 83079},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.179012358), 83078},  //
      {glm::vec2(0.339506179, -0.19135803), 83080},   //
      {glm::vec2(0.327160507, -0.19135803), 83081},   //
      {glm::vec2(0.327160507, -0.179012358), 83096},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.216049403), 83082},  //
      {glm::vec2(0.302469134, -0.228395075), 83083},  //
      {glm::vec2(0.290123463, -0.228395075), 83084},  //
      {glm::vec2(0.290123463, -0.216049403), 83085},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.228395075), 83086},  //
      {glm::vec2(0.253086448, -0.228395075), 83089},  //
      {glm::vec2(0.253086448, -0.216049403), 83088},  //
      {glm::vec2(0.265432119, -0.216049403), 83087},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.19135803), 83090},   //
      {glm::vec2(0.290123463, -0.19135803), 83092},   //
      {glm::vec2(0.290123463, -0.179012358), 83103},  //
      {glm::vec2(0.302469134, -0.179012358), 83091},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.19135803), 83093},   //
      {glm::vec2(0.253086448, -0.19135803), 83095},   //
      {glm::vec2(0.253086448, -0.179012358), 83105},  //
      {glm::vec2(0.265432119, -0.179012358), 83094},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.154320985), 83097},  //
      {glm::vec2(0.327160507, -0.154320985), 83100},  //
      {glm::vec2(0.327160507, -0.141975313), 83101},  //
      {glm::vec2(0.339506179, -0.141975313), 83099},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, -0.129629627), 83102},   //
      {glm::vec2(0.314814806, -0.129629627), 83159},   //
      {glm::vec2(0.314814806, -0.0925925896), 83160},  //
      {glm::vec2(0.351851851, -0.0925925896), 83158},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.154320985), 83104},  //
      {glm::vec2(0.290123463, -0.154320985), 83108},  //
      {glm::vec2(0.290123463, -0.141975313), 83109},  //
      {glm::vec2(0.302469134, -0.141975313), 83107},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.154320985), 83106},  //
      {glm::vec2(0.253086448, -0.154320985), 83112},  //
      {glm::vec2(0.253086448, -0.141975313), 83111},  //
      {glm::vec2(0.265432119, -0.141975313), 83110},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.117283955), 83113},  //
      {glm::vec2(0.475308657, -0.117283955), 83115},  //
      {glm::vec2(0.475308657, -0.104938276), 83117},  //
      {glm::vec2(0.487654328, -0.104938276), 83114},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.0802469105), 83119},  //
      {glm::vec2(0.475308657, -0.0802469105), 83121},  //
      {glm::vec2(0.475308657, -0.0679012313), 83122},  //
      {glm::vec2(0.487654328, -0.0679012313), 83120},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.0802469105), 83123},  //
      {glm::vec2(0.438271612, -0.0802469105), 83126},  //
      {glm::vec2(0.438271612, -0.0679012313), 83125},  //
      {glm::vec2(0.450617284, -0.0679012313), 83124},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.104938276), 83128},  //
      {glm::vec2(0.413580239, -0.117283955), 83129},  //
      {glm::vec2(0.401234567, -0.117283955), 83130},  //
      {glm::vec2(0.401234567, -0.104938276), 83131},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.117283955), 83132},  //
      {glm::vec2(0.364197552, -0.117283955), 83157},  //
      {glm::vec2(0.364197552, -0.104938276), 83134},  //
      {glm::vec2(0.376543224, -0.104938276), 83133},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.0802469105), 83135},  //
      {glm::vec2(0.401234567, -0.0802469105), 83137},  //
      {glm::vec2(0.401234567, -0.0679012313), 83138},  //
      {glm::vec2(0.413580239, -0.0679012313), 83136},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, -0.0802469105), 83139},  //
      {glm::vec2(0.364197552, -0.0802469105), 83161},  //
      {glm::vec2(0.364197552, -0.0679012313), 83141},  //
      {glm::vec2(0.376543224, -0.0679012313), 83140},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.0432098769), 83142},  //
      {glm::vec2(0.475308657, -0.0432098769), 83145},  //
      {glm::vec2(0.475308657, -0.0308641978), 83144},  //
      {glm::vec2(0.487654328, -0.0308641978), 83143},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, -0.0432098769), 83146},  //
      {glm::vec2(0.438271612, -0.0432098769), 83149},  //
      {glm::vec2(0.438271612, -0.0308641978), 83148},  //
      {glm::vec2(0.450617284, -0.0308641978), 83147},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, -0.00617283955), 83150},  //
      {glm::vec2(0.475308657, -0.00617283955), 83569},  //
      {glm::vec2(0.475308657, 0.00617283955), 83568},   //
      {glm::vec2(0.487654328, 0.00617283955), 83567},   //
  });
  polys.push_back({
      {glm::vec2(0.462962955, -0.0185185187), 83151},  //
      {glm::vec2(0.42592591, -0.0185185187), 83580},   //
      {glm::vec2(0.42592591, 0.0185185187), 83571},    //
      {glm::vec2(0.462962955, 0.0185185187), 83570},   //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.0308641978), 83152},  //
      {glm::vec2(0.413580239, -0.0432098769), 83153},  //
      {glm::vec2(0.401234567, -0.0432098769), 83155},  //
      {glm::vec2(0.401234567, -0.0308641978), 83154},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, -0.00617283955), 83156},  //
      {glm::vec2(0.401234567, -0.00617283955), 83583},  //
      {glm::vec2(0.401234567, 0.00617283955), 83582},   //
      {glm::vec2(0.413580239, 0.00617283955), 83581},   //
  });
  polys.push_back({
      {glm::vec2(0.339506179, -0.0679012313), 83162},  //
      {glm::vec2(0.339506179, -0.0802469105), 83163},  //
      {glm::vec2(0.327160507, -0.0802469105), 83164},  //
      {glm::vec2(0.327160507, -0.0679012313), 83165},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.117283955), 83166},  //
      {glm::vec2(0.290123463, -0.117283955), 83168},  //
      {glm::vec2(0.290123463, -0.104938276), 83169},  //
      {glm::vec2(0.302469134, -0.104938276), 83167},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.117283955), 83170},  //
      {glm::vec2(0.253086448, -0.117283955), 83172},  //
      {glm::vec2(0.253086448, -0.104938276), 83173},  //
      {glm::vec2(0.265432119, -0.104938276), 83171},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, -0.0802469105), 83174},  //
      {glm::vec2(0.290123463, -0.0802469105), 83176},  //
      {glm::vec2(0.290123463, -0.0679012313), 83177},  //
      {glm::vec2(0.302469134, -0.0679012313), 83175},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.0802469105), 83178},  //
      {glm::vec2(0.253086448, -0.0802469105), 83181},  //
      {glm::vec2(0.253086448, -0.0679012313), 83180},  //
      {glm::vec2(0.265432119, -0.0679012313), 83179},  //
  });
  polys.push_back({
      {glm::vec2(0.388888896, -0.055555556), 83182},  //
      {glm::vec2(0.277777791, -0.055555556), 83619},  //
      {glm::vec2(0.277777791, 0.055555556), 83618},   //
      {glm::vec2(0.388888896, 0.055555556), 83584},   //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.0432098769), 83183},  //
      {glm::vec2(0.253086448, -0.0432098769), 83186},  //
      {glm::vec2(0.253086448, -0.0308641978), 83185},  //
      {glm::vec2(0.265432119, -0.0308641978), 83184},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, -0.00617283955), 83187},  //
      {glm::vec2(0.253086448, -0.00617283955), 83622},  //
      {glm::vec2(0.253086448, 0.00617283955), 83621},   //
      {glm::vec2(0.265432119, 0.00617283955), 83620},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.240740761), 83188},  //
      {glm::vec2(0.203703731, -0.240740761), 83190},  //
      {glm::vec2(0.203703731, -0.203703731), 83194},  //
      {glm::vec2(0.240740761, -0.203703731), 83189},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.216049403), 83191},   //
      {glm::vec2(0.19135803, -0.228395075), 83192},   //
      {glm::vec2(0.179012358, -0.228395075), 83200},  //
      {glm::vec2(0.179012358, -0.216049403), 83193},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.19135803), 83195},   //
      {glm::vec2(0.216049403, -0.19135803), 83197},   //
      {glm::vec2(0.216049403, -0.179012358), 83210},  //
      {glm::vec2(0.228395075, -0.179012358), 83196},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.19135803), 83198},    //
      {glm::vec2(0.179012358, -0.19135803), 83206},   //
      {glm::vec2(0.179012358, -0.179012358), 83212},  //
      {glm::vec2(0.19135803, -0.179012358), 83199},   //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.216049403), 83201},  //
      {glm::vec2(0.154320985, -0.228395075), 83202},  //
      {glm::vec2(0.141975313, -0.228395075), 83203},  //
      {glm::vec2(0.141975313, -0.216049403), 83204},  //
  });
  polys.push_back({
      {glm::vec2(0.129629627, -0.203703731), 83205},   //
      {glm::vec2(0.129629627, -0.240740761), 83222},   //
      {glm::vec2(0.0925925896, -0.240740761), 83223},  //
      {glm::vec2(0.0925925896, -0.203703731), 83228},  //
  });
  polys.push_back({
      {glm::vec2(0.154320985, -0.179012358), 83207},  //
      {glm::vec2(0.154320985, -0.19135803), 83208},   //
      {glm::vec2(0.141975313, -0.19135803), 83209},   //
      {glm::vec2(0.141975313, -0.179012358), 83220},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.154320985), 83211},  //
      {glm::vec2(0.216049403, -0.154320985), 83215},  //
      {glm::vec2(0.216049403, -0.141975313), 83216},  //
      {glm::vec2(0.228395075, -0.141975313), 83214},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.154320985), 83213},   //
      {glm::vec2(0.179012358, -0.154320985), 83221},  //
      {glm::vec2(0.179012358, -0.141975313), 83219},  //
      {glm::vec2(0.19135803, -0.141975313), 83218},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.129629627), 83217},   //
      {glm::vec2(0.203703731, -0.129629627), 83249},   //
      {glm::vec2(0.203703731, -0.0925925896), 83248},  //
      {glm::vec2(0.240740761, -0.0925925896), 83247},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.216049403), 83224},  //
      {glm::vec2(0.0802469105, -0.228395075), 83225},  //
      {glm::vec2(0.0679012313, -0.228395075), 83226},  //
      {glm::vec2(0.0679012313, -0.216049403), 83227},  //
  });
  polys.push_back({
      {glm::vec2(0.117283955, -0.19135803), 83229},   //
      {glm::vec2(0.104938276, -0.19135803), 83231},   //
      {glm::vec2(0.104938276, -0.179012358), 83244},  //
      {glm::vec2(0.117283955, -0.179012358), 83230},  //
  });
  polys.push_back({
      {glm::vec2(0.0802469105, -0.179012358), 83232},  //
      {glm::vec2(0.0802469105, -0.19135803), 83233},   //
      {glm::vec2(0.0679012313, -0.19135803), 83234},   //
      {glm::vec2(0.0679012313, -0.179012358), 83245},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.216049403), 83235},  //
      {glm::vec2(0.0432098769, -0.228395075), 83236},  //
      {glm::vec2(0.0308641978, -0.228395075), 83237},  //
      {glm::vec2(0.0308641978, -0.216049403), 83238},  //
  });
  polys.push_back({
      {glm::vec2(0.0185185187, -0.203703731), 83239},   //
      {glm::vec2(0.0185185187, -0.240740761), 83520},   //
      {glm::vec2(-0.0185185187, -0.240740761), 83521},  //
      {glm::vec2(-0.0185185187, -0.203703731), 83526},  //
  });
  polys.push_back({
      {glm::vec2(0.0432098769, -0.19135803), 83240},   //
      {glm::vec2(0.0308641978, -0.19135803), 83242},   //
      {glm::vec2(0.0308641978, -0.179012358), 83246},  //
      {glm::vec2(0.0432098769, -0.179012358), 83241},  //
  });
  polys.push_back({
      {glm::vec2(0.00617283955, -0.179012358), 83243},   //
      {glm::vec2(0.00617283955, -0.19135803), 83527},    //
      {glm::vec2(-0.00617283955, -0.19135803), 83528},   //
      {glm::vec2(-0.00617283955, -0.179012358), 83545},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.117283955), 83250},   //
      {glm::vec2(0.179012358, -0.117283955), 83260},  //
      {glm::vec2(0.179012358, -0.104938276), 83252},  //
      {glm::vec2(0.19135803, -0.104938276), 83251},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.0802469105), 83253},  //
      {glm::vec2(0.216049403, -0.0802469105), 83255},  //
      {glm::vec2(0.216049403, -0.0679012313), 83256},  //
      {glm::vec2(0.228395075, -0.0679012313), 83254},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.0802469105), 83257},   //
      {glm::vec2(0.179012358, -0.0802469105), 83261},  //
      {glm::vec2(0.179012358, -0.0679012313), 83259},  //
      {glm::vec2(0.19135803, -0.0679012313), 83258},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, -0.0432098769), 83262},  //
      {glm::vec2(0.216049403, -0.0432098769), 83265},  //
      {glm::vec2(0.216049403, -0.0308641978), 83264},  //
      {glm::vec2(0.228395075, -0.0308641978), 83263},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.0432098769), 83266},   //
      {glm::vec2(0.179012358, -0.0432098769), 83271},  //
      {glm::vec2(0.179012358, -0.0308641978), 83268},  //
      {glm::vec2(0.19135803, -0.0308641978), 83267},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, -0.0185185187), 83269},  //
      {glm::vec2(0.203703731, -0.0185185187), 83740},  //
      {glm::vec2(0.203703731, 0.0185185187), 83739},   //
      {glm::vec2(0.240740761, 0.0185185187), 83738},   //
  });
  polys.push_back({
      {glm::vec2(0.19135803, -0.00617283955), 83270},   //
      {glm::vec2(0.179012358, -0.00617283955), 83750},  //
      {glm::vec2(0.179012358, 0.00617283955), 83742},   //
      {glm::vec2(0.19135803, 0.00617283955), 83741},    //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.487654328), 83276},  //
      {glm::vec2(-0.0432098769, -0.487654328), 83278},  //
      {glm::vec2(-0.0432098769, -0.475308657), 83279},  //
      {glm::vec2(-0.0308641978, -0.475308657), 83277},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.450617284), 83281},  //
      {glm::vec2(-0.0432098769, -0.450617284), 83283},  //
      {glm::vec2(-0.0432098769, -0.438271612), 83284},  //
      {glm::vec2(-0.0308641978, -0.438271612), 83282},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.475308657), 83285},  //
      {glm::vec2(-0.0679012388, -0.487654328), 83286},  //
      {glm::vec2(-0.0802469179, -0.487654328), 83288},  //
      {glm::vec2(-0.0802469179, -0.475308657), 83287},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.475308657), 83289},  //
      {glm::vec2(-0.104938269, -0.487654328), 83290},  //
      {glm::vec2(-0.117283948, -0.487654328), 83291},  //
      {glm::vec2(-0.117283948, -0.475308657), 83292},  //
  });
  polys.push_back({
      {glm::vec2(-0.092592597, -0.462962955), 83293},  //
      {glm::vec2(-0.129629642, -0.462962955), 83328},  //
      {glm::vec2(-0.129629642, -0.42592591), 83312},   //
      {glm::vec2(-0.092592597, -0.42592591), 83298},   //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.450617284), 83294},  //
      {glm::vec2(-0.0802469179, -0.450617284), 83296},  //
      {glm::vec2(-0.0802469179, -0.438271612), 83297},  //
      {glm::vec2(-0.0679012388, -0.438271612), 83295},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.413580239), 83303},  //
      {glm::vec2(-0.0432098769, -0.413580239), 83305},  //
      {glm::vec2(-0.0432098769, -0.401234567), 83306},  //
      {glm::vec2(-0.0308641978, -0.401234567), 83304},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.413580239), 83308},  //
      {glm::vec2(-0.0802469179, -0.413580239), 83311},  //
      {glm::vec2(-0.0802469179, -0.401234567), 83310},  //
      {glm::vec2(-0.0679012388, -0.401234567), 83309},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.413580239), 83313},  //
      {glm::vec2(-0.117283948, -0.413580239), 83315},  //
      {glm::vec2(-0.117283948, -0.401234567), 83316},  //
      {glm::vec2(-0.104938269, -0.401234567), 83314},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.376543224), 83317},  //
      {glm::vec2(-0.0802469179, -0.376543224), 83319},  //
      {glm::vec2(-0.0802469179, -0.364197552), 83370},  //
      {glm::vec2(-0.0679012388, -0.364197552), 83318},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.364197552), 83320},  //
      {glm::vec2(-0.104938269, -0.376543224), 83321},  //
      {glm::vec2(-0.117283948, -0.376543224), 83322},  //
      {glm::vec2(-0.117283948, -0.364197552), 83372},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.475308657), 83323},  //
      {glm::vec2(-0.141975313, -0.487654328), 83324},  //
      {glm::vec2(-0.154320985, -0.487654328), 83326},  //
      {glm::vec2(-0.154320985, -0.475308657), 83325},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.475308657), 83327},  //
      {glm::vec2(-0.179012358, -0.487654328), 83334},  //
      {glm::vec2(-0.19135803, -0.487654328), 83335},   //
      {glm::vec2(-0.19135803, -0.475308657), 83336},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.450617284), 83329},  //
      {glm::vec2(-0.154320985, -0.450617284), 83332},  //
      {glm::vec2(-0.154320985, -0.438271612), 83331},  //
      {glm::vec2(-0.141975313, -0.438271612), 83330},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.438271612), 83333},  //
      {glm::vec2(-0.179012358, -0.450617284), 83342},  //
      {glm::vec2(-0.19135803, -0.450617284), 83343},   //
      {glm::vec2(-0.19135803, -0.438271612), 83345},   //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.475308657), 83337},  //
      {glm::vec2(-0.216049403, -0.487654328), 83338},  //
      {glm::vec2(-0.228395075, -0.487654328), 83339},  //
      {glm::vec2(-0.228395075, -0.475308657), 83340},  //
  });
  polys.push_back({
      {glm::vec2(-0.203703716, -0.462962955), 83341},  //
      {glm::vec2(-0.240740746, -0.462962955), 83346},  //
      {glm::vec2(-0.240740746, -0.42592591), 83359},   //
      {glm::vec2(-0.203703716, -0.42592591), 83344},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.413580239), 83347},  //
      {glm::vec2(-0.154320985, -0.413580239), 83350},  //
      {glm::vec2(-0.154320985, -0.401234567), 83349},  //
      {glm::vec2(-0.141975313, -0.401234567), 83348},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.401234567), 83351},  //
      {glm::vec2(-0.179012358, -0.413580239), 83356},  //
      {glm::vec2(-0.19135803, -0.413580239), 83357},   //
      {glm::vec2(-0.19135803, -0.401234567), 83358},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.376543224), 83352},  //
      {glm::vec2(-0.154320985, -0.376543224), 83354},  //
      {glm::vec2(-0.154320985, -0.364197552), 83403},  //
      {glm::vec2(-0.141975313, -0.364197552), 83353},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.364197552), 83355},  //
      {glm::vec2(-0.179012358, -0.376543224), 83364},  //
      {glm::vec2(-0.19135803, -0.376543224), 83365},   //
      {glm::vec2(-0.19135803, -0.364197552), 83410},   //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.413580239), 83360},  //
      {glm::vec2(-0.228395075, -0.413580239), 83362},  //
      {glm::vec2(-0.228395075, -0.401234567), 83363},  //
      {glm::vec2(-0.216049403, -0.401234567), 83361},  //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.376543224), 83366},  //
      {glm::vec2(-0.228395075, -0.376543224), 83368},  //
      {glm::vec2(-0.228395075, -0.364197552), 83412},  //
      {glm::vec2(-0.216049403, -0.364197552), 83367},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.339506179), 83371},  //
      {glm::vec2(-0.0802469179, -0.339506179), 83376},  //
      {glm::vec2(-0.0802469179, -0.327160507), 83375},  //
      {glm::vec2(-0.0679012388, -0.327160507), 83374},  //
  });
  polys.push_back({
      {glm::vec2(-0.092592597, -0.351851851), 83373},  //
      {glm::vec2(-0.129629642, -0.351851851), 83405},  //
      {glm::vec2(-0.129629642, -0.314814806), 83378},  //
      {glm::vec2(-0.092592597, -0.314814806), 83377},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.265432119), 83383},  //
      {glm::vec2(-0.0432098769, -0.265432119), 83385},  //
      {glm::vec2(-0.0432098769, -0.253086448), 83386},  //
      {glm::vec2(-0.0308641978, -0.253086448), 83384},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.302469134), 83387},  //
      {glm::vec2(-0.0802469179, -0.302469134), 83390},  //
      {glm::vec2(-0.0802469179, -0.290123463), 83389},  //
      {glm::vec2(-0.0679012388, -0.290123463), 83388},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.290123463), 83391},  //
      {glm::vec2(-0.104938269, -0.302469134), 83392},  //
      {glm::vec2(-0.117283948, -0.302469134), 83394},  //
      {glm::vec2(-0.117283948, -0.290123463), 83393},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.253086448), 83395},  //
      {glm::vec2(-0.0679012388, -0.265432119), 83396},  //
      {glm::vec2(-0.0802469179, -0.265432119), 83397},  //
      {glm::vec2(-0.0802469179, -0.253086448), 83398},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.253086448), 83399},  //
      {glm::vec2(-0.104938269, -0.265432119), 83400},  //
      {glm::vec2(-0.117283948, -0.265432119), 83401},  //
      {glm::vec2(-0.117283948, -0.253086448), 83402},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.339506179), 83404},  //
      {glm::vec2(-0.154320985, -0.339506179), 83408},  //
      {glm::vec2(-0.154320985, -0.327160507), 83407},  //
      {glm::vec2(-0.141975313, -0.327160507), 83406},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.327160507), 83409},  //
      {glm::vec2(-0.179012358, -0.339506179), 83411},  //
      {glm::vec2(-0.19135803, -0.339506179), 83414},   //
      {glm::vec2(-0.19135803, -0.327160507), 83415},   //
  });
  polys.push_back({
      {glm::vec2(-0.203703716, -0.351851851), 83413},  //
      {glm::vec2(-0.240740746, -0.351851851), 83417},  //
      {glm::vec2(-0.240740746, -0.314814806), 83418},  //
      {glm::vec2(-0.203703716, -0.314814806), 83416},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.302469134), 83419},  //
      {glm::vec2(-0.154320985, -0.302469134), 83422},  //
      {glm::vec2(-0.154320985, -0.290123463), 83421},  //
      {glm::vec2(-0.141975313, -0.290123463), 83420},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.290123463), 83423},  //
      {glm::vec2(-0.179012358, -0.302469134), 83429},  //
      {glm::vec2(-0.19135803, -0.302469134), 83431},   //
      {glm::vec2(-0.19135803, -0.290123463), 83430},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.253086448), 83424},  //
      {glm::vec2(-0.141975313, -0.265432119), 83425},  //
      {glm::vec2(-0.154320985, -0.265432119), 83427},  //
      {glm::vec2(-0.154320985, -0.253086448), 83426},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.253086448), 83428},  //
      {glm::vec2(-0.179012358, -0.265432119), 83436},  //
      {glm::vec2(-0.19135803, -0.265432119), 83437},   //
      {glm::vec2(-0.19135803, -0.253086448), 83438},   //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.302469134), 83432},  //
      {glm::vec2(-0.228395075, -0.302469134), 83434},  //
      {glm::vec2(-0.228395075, -0.290123463), 83435},  //
      {glm::vec2(-0.216049403, -0.290123463), 83433},  //
  });
  polys.push_back({
      {glm::vec2(-0.216049403, -0.253086448), 83439},  //
      {glm::vec2(-0.216049403, -0.265432119), 83440},  //
      {glm::vec2(-0.228395075, -0.265432119), 83441},  //
      {glm::vec2(-0.228395075, -0.253086448), 83442},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.475308657), 83443},  //
      {glm::vec2(-0.253086448, -0.487654328), 83444},  //
      {glm::vec2(-0.265432119, -0.487654328), 83445},  //
      {glm::vec2(-0.265432119, -0.475308657), 83446},  //
  });
  polys.push_back({
      {glm::vec2(-0.290123463, -0.487654328), 83447},  //
      {glm::vec2(-0.302469134, -0.487654328), 83449},  //
      {glm::vec2(-0.302469134, -0.475308657), 83450},  //
      {glm::vec2(-0.290123463, -0.475308657), 83448},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.438271612), 83451},  //
      {glm::vec2(-0.253086448, -0.450617284), 83452},  //
      {glm::vec2(-0.265432119, -0.450617284), 83453},  //
      {glm::vec2(-0.265432119, -0.438271612), 83454},  //
  });
  polys.push_back({
      {glm::vec2(-0.290123463, -0.450617284), 83455},  //
      {glm::vec2(-0.302469134, -0.450617284), 83457},  //
      {glm::vec2(-0.302469134, -0.438271612), 83458},  //
      {glm::vec2(-0.290123463, -0.438271612), 83456},  //
  });
  polys.push_back({
      {glm::vec2(-0.327160507, -0.475308657), 83459},  //
      {glm::vec2(-0.327160507, -0.487654328), 83460},  //
      {glm::vec2(-0.339506179, -0.487654328), 83463},  //
      {glm::vec2(-0.339506179, -0.475308657), 83461},  //
  });
  polys.push_back({
      {glm::vec2(-0.314814836, -0.462962955), 83462},  //
      {glm::vec2(-0.351851881, -0.462962955), 83466},  //
      {glm::vec2(-0.351851881, -0.42592591), 83479},   //
      {glm::vec2(-0.314814836, -0.42592591), 83465},   //
  });
  polys.push_back({
      {glm::vec2(-0.364197552, -0.475308657), 83464},  //
      {glm::vec2(-0.364197552, -0.487654328), 83486},  //
      {glm::vec2(-0.376543224, -0.487654328), 83487},  //
      {glm::vec2(-0.376543224, -0.475308657), 83488},  //
  });
  polys.push_back({
      {glm::vec2(-0.364197552, -0.438271612), 83467},  //
      {glm::vec2(-0.364197552, -0.450617284), 83493},  //
      {glm::vec2(-0.376543224, -0.450617284), 83494},  //
      {glm::vec2(-0.376543224, -0.438271612), 83495},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.413580239), 83468},  //
      {glm::vec2(-0.265432119, -0.413580239), 83470},  //
      {glm::vec2(-0.265432119, -0.401234567), 83471},  //
      {glm::vec2(-0.253086448, -0.401234567), 83469},  //
  });
  polys.push_back({
      {glm::vec2(-0.290123463, -0.413580239), 83472},  //
      {glm::vec2(-0.302469134, -0.413580239), 83474},  //
      {glm::vec2(-0.302469134, -0.401234567), 83475},  //
      {glm::vec2(-0.290123463, -0.401234567), 83473},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.364197552), 83476},  //
      {glm::vec2(-0.253086448, -0.376543224), 83477},  //
      {glm::vec2(-0.265432119, -0.376543224), 83478},  //
      {glm::vec2(-0.265432119, -0.364197552), 83510},  //
  });
  polys.push_back({
      {glm::vec2(-0.327160507, -0.413580239), 83480},  //
      {glm::vec2(-0.339506179, -0.413580239), 83483},  //
      {glm::vec2(-0.339506179, -0.401234567), 83482},  //
      {glm::vec2(-0.327160507, -0.401234567), 83481},  //
  });
  polys.push_back({
      {glm::vec2(-0.364197552, -0.401234567), 83484},  //
      {glm::vec2(-0.364197552, -0.413580239), 83506},  //
      {glm::vec2(-0.376543224, -0.413580239), 83507},  //
      {glm::vec2(-0.376543224, -0.401234567), 83508},  //
  });
  polys.push_back({
      {glm::vec2(-0.401234567, -0.475308657), 83489},  //
      {glm::vec2(-0.401234567, -0.487654328), 83490},  //
      {glm::vec2(-0.413580239, -0.487654328), 83492},  //
      {glm::vec2(-0.413580239, -0.475308657), 83491},  //
  });
  polys.push_back({
      {glm::vec2(-0.401234567, -0.450617284), 83496},  //
      {glm::vec2(-0.413580239, -0.450617284), 83498},  //
      {glm::vec2(-0.413580239, -0.438271612), 83499},  //
      {glm::vec2(-0.401234567, -0.438271612), 83497},  //
  });
  polys.push_back({
      {glm::vec2(-0.438271612, -0.475308657), 83500},  //
      {glm::vec2(-0.438271612, -0.487654328), 83501},  //
      {glm::vec2(-0.450617284, -0.487654328), 83502},  //
      {glm::vec2(-0.450617284, -0.475308657), 83503},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.339506179), 83511},  //
      {glm::vec2(-0.265432119, -0.339506179), 83513},  //
      {glm::vec2(-0.265432119, -0.327160507), 83514},  //
      {glm::vec2(-0.253086448, -0.327160507), 83512},  //
  });
  polys.push_back({
      {glm::vec2(-0.253086448, -0.290123463), 83515},  //
      {glm::vec2(-0.253086448, -0.302469134), 83516},  //
      {glm::vec2(-0.265432119, -0.302469134), 83517},  //
      {glm::vec2(-0.265432119, -0.290123463), 83518},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.228395075), 83522},  //
      {glm::vec2(-0.0432098769, -0.228395075), 83524},  //
      {glm::vec2(-0.0432098769, -0.216049403), 83525},  //
      {glm::vec2(-0.0308641978, -0.216049403), 83523},  //
  });
  polys.push_back({
      {glm::vec2(-0.0308641978, -0.19135803), 83529},   //
      {glm::vec2(-0.0432098769, -0.19135803), 83531},   //
      {glm::vec2(-0.0432098769, -0.179012358), 83547},  //
      {glm::vec2(-0.0308641978, -0.179012358), 83530},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.216049403), 83532},  //
      {glm::vec2(-0.0679012388, -0.228395075), 83533},  //
      {glm::vec2(-0.0802469179, -0.228395075), 83535},  //
      {glm::vec2(-0.0802469179, -0.216049403), 83534},  //
  });
  polys.push_back({
      {glm::vec2(-0.092592597, -0.240740761), 83536},  //
      {glm::vec2(-0.129629642, -0.240740761), 83550},  //
      {glm::vec2(-0.129629642, -0.203703731), 83541},  //
      {glm::vec2(-0.092592597, -0.203703731), 83537},  //
  });
  polys.push_back({
      {glm::vec2(-0.0679012388, -0.19135803), 83538},   //
      {glm::vec2(-0.0802469179, -0.19135803), 83540},   //
      {glm::vec2(-0.0802469179, -0.179012358), 83548},  //
      {glm::vec2(-0.0679012388, -0.179012358), 83539},  //
  });
  polys.push_back({
      {glm::vec2(-0.104938269, -0.179012358), 83542},  //
      {glm::vec2(-0.104938269, -0.19135803), 83543},   //
      {glm::vec2(-0.117283948, -0.19135803), 83544},   //
      {glm::vec2(-0.117283948, -0.179012358), 83549},  //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.216049403), 83551},  //
      {glm::vec2(-0.141975313, -0.228395075), 83552},  //
      {glm::vec2(-0.154320985, -0.228395075), 83554},  //
      {glm::vec2(-0.154320985, -0.216049403), 83553},  //
  });
  polys.push_back({
      {glm::vec2(-0.179012358, -0.216049403), 83555},  //
      {glm::vec2(-0.179012358, -0.228395075), 83559},  //
      {glm::vec2(-0.19135803, -0.228395075), 83560},   //
      {glm::vec2(-0.19135803, -0.216049403), 83561},   //
  });
  polys.push_back({
      {glm::vec2(-0.141975313, -0.19135803), 83556},   //
      {glm::vec2(-0.154320985, -0.19135803), 83558},   //
      {glm::vec2(-0.154320985, -0.179012358), 83566},  //
      {glm::vec2(-0.141975313, -0.179012358), 83557},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.0308641978), 83572},  //
      {glm::vec2(0.475308657, 0.0308641978), 83574},  //
      {glm::vec2(0.475308657, 0.0432098769), 83575},  //
      {glm::vec2(0.487654328, 0.0432098769), 83573},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.0432098769), 83576},  //
      {glm::vec2(0.450617284, 0.0308641978), 83577},  //
      {glm::vec2(0.438271612, 0.0308641978), 83579},  //
      {glm::vec2(0.438271612, 0.0432098769), 83578},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.0432098769), 83585},  //
      {glm::vec2(0.413580239, 0.0308641978), 83586},  //
      {glm::vec2(0.401234567, 0.0308641978), 83587},  //
      {glm::vec2(0.401234567, 0.0432098769), 83588},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.0679012388), 83589},  //
      {glm::vec2(0.475308657, 0.0679012388), 83591},  //
      {glm::vec2(0.475308657, 0.0802469179), 83592},  //
      {glm::vec2(0.487654328, 0.0802469179), 83590},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.0679012388), 83593},  //
      {glm::vec2(0.438271612, 0.0679012388), 83595},  //
      {glm::vec2(0.438271612, 0.0802469179), 83596},  //
      {glm::vec2(0.450617284, 0.0802469179), 83594},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.104938269), 83597},  //
      {glm::vec2(0.475308657, 0.104938269), 83599},  //
      {glm::vec2(0.475308657, 0.117283948), 83600},  //
      {glm::vec2(0.487654328, 0.117283948), 83598},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, 0.129629642), 83601},  //
      {glm::vec2(0.462962955, 0.092592597), 83602},  //
      {glm::vec2(0.42592591, 0.092592597), 83611},   //
      {glm::vec2(0.42592591, 0.129629642), 83655},   //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.0802469179), 83603},  //
      {glm::vec2(0.413580239, 0.0679012388), 83604},  //
      {glm::vec2(0.401234567, 0.0679012388), 83605},  //
      {glm::vec2(0.401234567, 0.0802469179), 83606},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.0679012388), 83607},  //
      {glm::vec2(0.364197552, 0.0679012388), 83627},  //
      {glm::vec2(0.364197552, 0.0802469179), 83609},  //
      {glm::vec2(0.376543224, 0.0802469179), 83608},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.104938269), 83610},  //
      {glm::vec2(0.401234567, 0.104938269), 83613},  //
      {glm::vec2(0.401234567, 0.117283948), 83614},  //
      {glm::vec2(0.413580239, 0.117283948), 83612},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.104938269), 83615},  //
      {glm::vec2(0.364197552, 0.104938269), 83632},  //
      {glm::vec2(0.364197552, 0.117283948), 83617},  //
      {glm::vec2(0.376543224, 0.117283948), 83616},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.0308641978), 83623},  //
      {glm::vec2(0.253086448, 0.0308641978), 83626},  //
      {glm::vec2(0.253086448, 0.0432098769), 83625},  //
      {glm::vec2(0.265432119, 0.0432098769), 83624},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.0802469179), 83628},  //
      {glm::vec2(0.339506179, 0.0679012388), 83629},  //
      {glm::vec2(0.327160507, 0.0679012388), 83630},  //
      {glm::vec2(0.327160507, 0.0802469179), 83631},  //
  });
  polys.push_back({
      {glm::vec2(0.351851851, 0.129629642), 83633},  //
      {glm::vec2(0.351851851, 0.092592597), 83634},  //
      {glm::vec2(0.314814806, 0.092592597), 83635},  //
      {glm::vec2(0.314814806, 0.129629642), 83700},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.0679012388), 83636},  //
      {glm::vec2(0.290123463, 0.0679012388), 83638},  //
      {glm::vec2(0.290123463, 0.0802469179), 83639},  //
      {glm::vec2(0.302469134, 0.0802469179), 83637},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.0679012388), 83640},  //
      {glm::vec2(0.253086448, 0.0679012388), 83642},  //
      {glm::vec2(0.253086448, 0.0802469179), 83643},  //
      {glm::vec2(0.265432119, 0.0802469179), 83641},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.104938269), 83644},  //
      {glm::vec2(0.290123463, 0.104938269), 83646},  //
      {glm::vec2(0.290123463, 0.117283948), 83647},  //
      {glm::vec2(0.302469134, 0.117283948), 83645},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.104938269), 83648},  //
      {glm::vec2(0.253086448, 0.104938269), 83651},  //
      {glm::vec2(0.253086448, 0.117283948), 83650},  //
      {glm::vec2(0.265432119, 0.117283948), 83649},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.141975313), 83652},  //
      {glm::vec2(0.475308657, 0.141975313), 83654},  //
      {glm::vec2(0.475308657, 0.154320985), 83659},  //
      {glm::vec2(0.487654328, 0.154320985), 83653},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.141975313), 83656},  //
      {glm::vec2(0.438271612, 0.141975313), 83658},  //
      {glm::vec2(0.438271612, 0.154320985), 83661},  //
      {glm::vec2(0.450617284, 0.154320985), 83657},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.179012358), 83660},  //
      {glm::vec2(0.475308657, 0.179012358), 83673},  //
      {glm::vec2(0.475308657, 0.19135803), 83674},   //
      {glm::vec2(0.487654328, 0.19135803), 83672},   //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.179012358), 83662},  //
      {glm::vec2(0.438271612, 0.179012358), 83677},  //
      {glm::vec2(0.438271612, 0.19135803), 83676},   //
      {glm::vec2(0.450617284, 0.19135803), 83675},   //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.154320985), 83663},  //
      {glm::vec2(0.413580239, 0.141975313), 83664},  //
      {glm::vec2(0.401234567, 0.141975313), 83665},  //
      {glm::vec2(0.401234567, 0.154320985), 83668},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.141975313), 83666},  //
      {glm::vec2(0.364197552, 0.141975313), 83698},  //
      {glm::vec2(0.364197552, 0.154320985), 83670},  //
      {glm::vec2(0.376543224, 0.154320985), 83667},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.179012358), 83669},  //
      {glm::vec2(0.401234567, 0.179012358), 83686},  //
      {glm::vec2(0.401234567, 0.19135803), 83687},   //
      {glm::vec2(0.413580239, 0.19135803), 83685},   //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.179012358), 83671},  //
      {glm::vec2(0.364197552, 0.179012358), 83715},  //
      {glm::vec2(0.364197552, 0.19135803), 83689},   //
      {glm::vec2(0.376543224, 0.19135803), 83688},   //
  });
  polys.push_back({
      {glm::vec2(0.462962955, 0.203703716), 83678},  //
      {glm::vec2(0.42592591, 0.203703716), 83690},   //
      {glm::vec2(0.42592591, 0.240740746), 83684},   //
      {glm::vec2(0.462962955, 0.240740746), 83683},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.216049403), 83679},  //
      {glm::vec2(0.475308657, 0.216049403), 83682},  //
      {glm::vec2(0.475308657, 0.228395075), 83681},  //
      {glm::vec2(0.487654328, 0.228395075), 83680},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.228395075), 83691},  //
      {glm::vec2(0.413580239, 0.216049403), 83692},  //
      {glm::vec2(0.401234567, 0.216049403), 83694},  //
      {glm::vec2(0.401234567, 0.228395075), 83693},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.216049403), 83695},  //
      {glm::vec2(0.364197552, 0.216049403), 83720},  //
      {glm::vec2(0.364197552, 0.228395075), 83697},  //
      {glm::vec2(0.376543224, 0.228395075), 83696},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.154320985), 83699},  //
      {glm::vec2(0.339506179, 0.141975313), 83701},  //
      {glm::vec2(0.327160507, 0.141975313), 83702},  //
      {glm::vec2(0.327160507, 0.154320985), 83703},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.179012358), 83704},  //
      {glm::vec2(0.327160507, 0.179012358), 83717},  //
      {glm::vec2(0.327160507, 0.19135803), 83718},   //
      {glm::vec2(0.339506179, 0.19135803), 83716},   //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.141975313), 83705},  //
      {glm::vec2(0.290123463, 0.141975313), 83707},  //
      {glm::vec2(0.290123463, 0.154320985), 83711},  //
      {glm::vec2(0.302469134, 0.154320985), 83706},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.141975313), 83708},  //
      {glm::vec2(0.253086448, 0.141975313), 83710},  //
      {glm::vec2(0.253086448, 0.154320985), 83713},  //
      {glm::vec2(0.265432119, 0.154320985), 83709},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.179012358), 83712},  //
      {glm::vec2(0.290123463, 0.179012358), 83725},  //
      {glm::vec2(0.290123463, 0.19135803), 83726},   //
      {glm::vec2(0.302469134, 0.19135803), 83724},   //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.179012358), 83714},  //
      {glm::vec2(0.253086448, 0.179012358), 83729},  //
      {glm::vec2(0.253086448, 0.19135803), 83728},   //
      {glm::vec2(0.265432119, 0.19135803), 83727},   //
  });
  polys.push_back({
      {glm::vec2(0.351851851, 0.203703716), 83719},  //
      {glm::vec2(0.314814806, 0.203703716), 83722},  //
      {glm::vec2(0.314814806, 0.240740746), 83723},  //
      {glm::vec2(0.351851851, 0.240740746), 83721},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.216049403), 83730},  //
      {glm::vec2(0.290123463, 0.216049403), 83732},  //
      {glm::vec2(0.290123463, 0.228395075), 83733},  //
      {glm::vec2(0.302469134, 0.228395075), 83731},  //
  });
  polys.push_back({
      {glm::vec2(0.265432119, 0.216049403), 83734},  //
      {glm::vec2(0.253086448, 0.216049403), 83737},  //
      {glm::vec2(0.253086448, 0.228395075), 83736},  //
      {glm::vec2(0.265432119, 0.228395075), 83735},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.0308641978), 83743},  //
      {glm::vec2(0.216049403, 0.0308641978), 83745},  //
      {glm::vec2(0.216049403, 0.0432098769), 83746},  //
      {glm::vec2(0.228395075, 0.0432098769), 83744},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.0432098769), 83747},   //
      {glm::vec2(0.19135803, 0.0308641978), 83748},   //
      {glm::vec2(0.179012358, 0.0308641978), 83751},  //
      {glm::vec2(0.179012358, 0.0432098769), 83749},  //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.0679012388), 83752},  //
      {glm::vec2(0.216049403, 0.0679012388), 83754},  //
      {glm::vec2(0.216049403, 0.0802469179), 83755},  //
      {glm::vec2(0.228395075, 0.0802469179), 83753},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.0679012388), 83756},   //
      {glm::vec2(0.179012358, 0.0679012388), 83765},  //
      {glm::vec2(0.179012358, 0.0802469179), 83758},  //
      {glm::vec2(0.19135803, 0.0802469179), 83757},   //
  });
  polys.push_back({
      {glm::vec2(0.240740761, 0.092592597), 83759},  //
      {glm::vec2(0.203703731, 0.092592597), 83762},  //
      {glm::vec2(0.203703731, 0.129629642), 83767},  //
      {glm::vec2(0.240740761, 0.129629642), 83760},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.104938269), 83761},   //
      {glm::vec2(0.179012358, 0.104938269), 83766},  //
      {glm::vec2(0.179012358, 0.117283948), 83764},  //
      {glm::vec2(0.19135803, 0.117283948), 83763},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.141975313), 83768},  //
      {glm::vec2(0.216049403, 0.141975313), 83770},  //
      {glm::vec2(0.216049403, 0.154320985), 83773},  //
      {glm::vec2(0.228395075, 0.154320985), 83769},  //
  });
  polys.push_back({
      {glm::vec2(0.19135803, 0.141975313), 83771},   //
      {glm::vec2(0.179012358, 0.141975313), 83777},  //
      {glm::vec2(0.179012358, 0.154320985), 83775},  //
      {glm::vec2(0.19135803, 0.154320985), 83772},   //
  });
  polys.push_back({
      {glm::vec2(0.228395075, 0.179012358), 83774},  //
      {glm::vec2(0.216049403, 0.179012358), 83779},  //
      {glm::vec2(0.216049403, 0.19135803), 83780},   //
      {glm::vec2(0.228395075, 0.19135803), 83778},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.253086448), 83786},  //
      {glm::vec2(0.475308657, 0.253086448), 83789},  //
      {glm::vec2(0.475308657, 0.265432119), 83788},  //
      {glm::vec2(0.487654328, 0.265432119), 83787},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.253086448), 83790},  //
      {glm::vec2(0.438271612, 0.253086448), 83793},  //
      {glm::vec2(0.438271612, 0.265432119), 83792},  //
      {glm::vec2(0.450617284, 0.265432119), 83791},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.290123463), 83794},  //
      {glm::vec2(0.475308657, 0.290123463), 83796},  //
      {glm::vec2(0.475308657, 0.302469134), 83797},  //
      {glm::vec2(0.487654328, 0.302469134), 83795},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.302469134), 83798},  //
      {glm::vec2(0.450617284, 0.290123463), 83799},  //
      {glm::vec2(0.438271612, 0.290123463), 83801},  //
      {glm::vec2(0.438271612, 0.302469134), 83800},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.253086448), 83802},  //
      {glm::vec2(0.401234567, 0.253086448), 83805},  //
      {glm::vec2(0.401234567, 0.265432119), 83804},  //
      {glm::vec2(0.413580239, 0.265432119), 83803},  //
  });
  polys.push_back({
      {glm::vec2(0.376543224, 0.253086448), 83806},  //
      {glm::vec2(0.364197552, 0.253086448), 83829},  //
      {glm::vec2(0.364197552, 0.265432119), 83808},  //
      {glm::vec2(0.376543224, 0.265432119), 83807},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.302469134), 83809},  //
      {glm::vec2(0.413580239, 0.290123463), 83810},  //
      {glm::vec2(0.401234567, 0.290123463), 83811},  //
      {glm::vec2(0.401234567, 0.302469134), 83812},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.327160507), 83813},  //
      {glm::vec2(0.475308657, 0.327160507), 83815},  //
      {glm::vec2(0.475308657, 0.339506179), 83818},  //
      {glm::vec2(0.487654328, 0.339506179), 83814},  //
  });
  polys.push_back({
      {glm::vec2(0.462962955, 0.351851881), 83816},  //
      {glm::vec2(0.462962955, 0.314814836), 83817},  //
      {glm::vec2(0.42592591, 0.314814836), 83822},   //
      {glm::vec2(0.42592591, 0.351851881), 83820},   //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.364197552), 83819},  //
      {glm::vec2(0.475308657, 0.364197552), 83844},  //
      {glm::vec2(0.475308657, 0.376543224), 83845},  //
      {glm::vec2(0.487654328, 0.376543224), 83843},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.364197552), 83821},  //
      {glm::vec2(0.438271612, 0.364197552), 83848},  //
      {glm::vec2(0.438271612, 0.376543224), 83847},  //
      {glm::vec2(0.450617284, 0.376543224), 83846},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.339506179), 83823},  //
      {glm::vec2(0.413580239, 0.327160507), 83824},  //
      {glm::vec2(0.401234567, 0.327160507), 83825},  //
      {glm::vec2(0.401234567, 0.339506179), 83827},  //
  });
  polys.push_back({
      {glm::vec2(0.413580239, 0.364197552), 83828},  //
      {glm::vec2(0.401234567, 0.364197552), 83859},  //
      {glm::vec2(0.401234567, 0.376543224), 83860},  //
      {glm::vec2(0.413580239, 0.376543224), 83858},  //
  });
  polys.push_back({
      {glm::vec2(0.339506179, 0.265432119), 83830},  //
      {glm::vec2(0.339506179, 0.253086448), 83831},  //
      {glm::vec2(0.327160507, 0.253086448), 83833},  //
      {glm::vec2(0.327160507, 0.265432119), 83832},  //
  });
  polys.push_back({
      {glm::vec2(0.302469134, 0.253086448), 83835},  //
      {glm::vec2(0.290123463, 0.253086448), 83837},  //
      {glm::vec2(0.290123463, 0.265432119), 83838},  //
      {glm::vec2(0.302469134, 0.265432119), 83836},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.401234567), 83849},  //
      {glm::vec2(0.475308657, 0.401234567), 83851},  //
      {glm::vec2(0.475308657, 0.413580239), 83852},  //
      {glm::vec2(0.487654328, 0.413580239), 83850},  //
  });
  polys.push_back({
      {glm::vec2(0.450617284, 0.401234567), 83853},  //
      {glm::vec2(0.438271612, 0.401234567), 83855},  //
      {glm::vec2(0.438271612, 0.413580239), 83856},  //
      {glm::vec2(0.450617284, 0.413580239), 83854},  //
  });
  polys.push_back({
      {glm::vec2(0.487654328, 0.438271612), 83864},  //
      {glm::vec2(0.475308657, 0.438271612), 83866},  //
      {glm::vec2(0.475308657, 0.450617284), 83867},  //
      {glm::vec2(0.487654328, 0.450617284), 83865},  //
  });
  TestPoly(polys, 1771);
}
