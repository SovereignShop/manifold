#pragma once

extern "C" {
#include "libqhull_r/libqhull_r.h"
}

#include <stdexcept>
#include "public.h"
#include <glm/glm.hpp>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>

namespace manifold {

using namespace manifold;

std::vector<std::vector<int>> buildAdjacency(const Mesh &mesh) {
    std::map<std::pair<int, int>, std::vector<int>> edgeToFace;
    std::vector<std::vector<int>> adjacency(mesh.triVerts.size());

    for (int i = 0; i < mesh.triVerts.size(); ++i) {
        glm::ivec3 face = mesh.triVerts[i];
        for (int j = 0; j < 3; ++j) {
            std::pair<int, int> edge(face[j], face[(j+1)%3]);
            if (edge.first > edge.second) std::swap(edge.first, edge.second);

            edgeToFace[edge].push_back(i);
        }
    }

    for (const auto &pair : edgeToFace) {
        for (int i = 0; i < pair.second.size(); ++i) {
            for (int j = i+1; j < pair.second.size(); ++j) {
                adjacency[pair.second[i]].push_back(pair.second[j]);
                adjacency[pair.second[j]].push_back(pair.second[i]);
            }
        }
    }

    return adjacency;
}

void orientMesh(Mesh &mesh) {
    std::vector<std::vector<int>> adjacency = buildAdjacency(mesh);
    std::vector<bool> visited(mesh.triVerts.size(), false);
    std::queue<int> q;

    q.push(0);
    visited[0] = true;

    while (!q.empty()) {
        int currentFaceIndex = q.front();
        q.pop();
        glm::ivec3 currentFace = mesh.triVerts[currentFaceIndex];

        for (int neighbour : adjacency[currentFaceIndex]) {
            if (visited[neighbour]) continue;

            glm::ivec3 neighbourFace = mesh.triVerts[neighbour];

            for (int i = 0; i < 3; ++i) {
                if ((neighbourFace[i] == currentFace[0] && neighbourFace[(i+1)%3] == currentFace[1]) ||
                    (neighbourFace[i] == currentFace[1] && neighbourFace[(i+1)%3] == currentFace[2]) ||
                    (neighbourFace[i] == currentFace[2] && neighbourFace[(i+1)%3] == currentFace[0])) {
                    std::swap(mesh.triVerts[neighbour][(i+1)%3], mesh.triVerts[neighbour][(i+2)%3]);
                    break;
                }
            }

            visited[neighbour] = true;
            q.push(neighbour);
        }
    }
}


Mesh computeConvexHull3D(const std::vector<glm::vec3>& combinedVerts) {
  // Convert vertices to coordinate array for Qhull
  int dim = 3;
  int n = combinedVerts.size();

  coordT* points = new coordT[n*dim];
  for(int i = 0; i < n; i++) {
    points[i*dim] = combinedVerts[i].x;
    points[i*dim+1] = combinedVerts[i].y;
    points[i*dim+2] = combinedVerts[i].z;
  }

  boolT ismalloc = False;  // True if qhull should free points in qh_freeqhull() or reallocation
  char flags[] = "qhull Qt";  // option flags for qhull, see qh_opt.htm

  // create a new instance of Qhull
  qhT qh_qh;
  qh_zero(&qh_qh, stderr);

  int exitcode = qh_new_qhull(&qh_qh, dim, n, points, ismalloc, flags, NULL, NULL);
  if(exitcode != 0) {
    //std::cout << "Convex Hull failled! Returning first mesh." << std::endl;
    throw std::runtime_error("Convex Hull failled!");
  }

  // Create a new mesh for the convex hull
  Mesh convexHull;

  std::map<int, int> vertexIndexMap;
  //// Iterate over the facets of the hull
  qhT* qh = &qh_qh;
  facetT *facet;
  vertexT *vertex, **vertexp;
  FORALLfacets {
    glm::ivec3 tri;
    int i = 0;

    FOREACHvertex_(facet->vertices) {
      int id = qh_pointid(&qh_qh, vertex->point);

      // Check if the vertex is already added
      if(vertexIndexMap.find(id) == vertexIndexMap.end()) {
        convexHull.vertPos.push_back(combinedVerts[id]);
        vertexIndexMap[id] = convexHull.vertPos.size() - 1;
      }

      // Add the index to the triangle
      tri[i] = vertexIndexMap[id];
      i++;
    }

    convexHull.triVerts.push_back(tri);
  }

  qh_freeqhull(&qh_qh, !qh_ALL);
  delete[] points;

  // Orient faces by right-hand rule.
  orientMesh(convexHull);

  return convexHull;
}

int findMinYPointIndex(const std::vector<glm::vec2>& points) {
    int minYPointIndex = 0;
    for (size_t i = 1; i < points.size(); i++) {
        if ((points[i].y < points[minYPointIndex].y) ||
            (points[i].y == points[minYPointIndex].y && points[i].x > points[minYPointIndex].x)) {
            minYPointIndex = i;
        }
    }
    return minYPointIndex;
}

std::vector<glm::vec2> sortPointsCounterClockwise(const std::vector<glm::vec2>& points) {
    std::vector<glm::vec2> sortedPoints(points);

    // Find the bottom-most point (or one of them, if multiple)
    int minYPointIndex = findMinYPointIndex(sortedPoints);

    // Sort the points by angle from the line horizontal to minYPoint, counter-clockwise
    glm::vec2 minYPoint = points[minYPointIndex];
    std::sort(sortedPoints.begin(), sortedPoints.end(),
        [minYPoint](const glm::vec2& p1, const glm::vec2& p2) -> bool {
            double angle1 = atan2(p1.y - minYPoint.y, p1.x - minYPoint.x);
            double angle2 = atan2(p2.y - minYPoint.y, p2.x - minYPoint.x);
            if (angle1 < 0) angle1 += 2 * 3.141592653589;
            if (angle2 < 0) angle2 += 2 * 3.141592653589;
            return angle1 < angle2;
        }
    );

    return sortedPoints;
}

SimplePolygon computeConvexHull2D(const SimplePolygon& allPoints) {
  // Convert points to coordinate array for Qhull
  int dim = 2;  // We're now in 2D
  int n = allPoints.size();

  coordT* points = new coordT[n*dim];
  for(int i = 0; i < n; i++) {
    points[i*dim] = allPoints[i].x;
    points[i*dim+1] = allPoints[i].y;
  }

  boolT ismalloc = False;
  char flags[] = "qhull Qt";

  // create a new instance of Qhull
  qhT qh_qh;
  qh_zero(&qh_qh, stderr);

  int exitcode = qh_new_qhull(&qh_qh, dim, n, points, ismalloc, flags, NULL, NULL);
  if(exitcode != 0) {
    //std::cout << "Convex Hull failed! Returning first polygon." << std::endl;
    return allPoints;
  }

  // Create a new polygon for the convex hull
  SimplePolygon convexHull;

  qhT* qh = &qh_qh;
  vertexT *vertex, **vertexp;
  FORALLvertices {
    int id = qh_pointid(&qh_qh, vertex->point);
    convexHull.push_back(allPoints[id]);
  }

  qh_freeqhull(&qh_qh, !qh_ALL);
  delete[] points;

  // It's not clear why qhull does not sanely order vertices. For now we sort them ourselves.
  SimplePolygon sortedPoints = sortPointsCounterClockwise(convexHull);
  return sortedPoints;
}

}
