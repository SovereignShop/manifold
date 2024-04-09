#pragma once

#include <stdexcept>
#include <glm/glm.hpp>
#include <vector>
#include "polygon.h"
#include "manifold.h"
#include "cross_section.h"
#include "buffer_utils.hpp"
#include "matrix_transforms.hpp"

namespace MeshUtils {

std::vector<glm::ivec3> TriangulateFaces(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<uint32_t>>& faces, float precision) {
    std::vector<glm::ivec3> result;
    for (const auto& face : faces) {
        // If the face only has 3 vertices, no need to triangulate, just add it to result
        if (face.size() == 3) {
            result.push_back(glm::ivec3(face[0], face[1], face[2]));
            continue;
        }

        // Compute face normal
        glm::vec3 normal = glm::cross(vertices[face[1]] - vertices[face[0]], vertices[face[2]] - vertices[face[0]]);
        normal = glm::normalize(normal);

        // Compute reference right vector
        glm::vec3 right = glm::normalize(vertices[face[1]] - vertices[face[0]]);

        // Compute up vector
        glm::vec3 up = glm::cross(right, normal);

        // Project vertices onto plane
        std::vector<glm::vec2> face2D;
        for (const auto& index : face) {
            glm::vec3 local = vertices[index] - vertices[face[0]];
            face2D.push_back(glm::vec2(glm::dot(local, right), glm::dot(local, up)));
        }

        // Triangulate and remap the triangulated vertices back to the original indices
        std::vector<glm::ivec3> triVerts = manifold::Triangulate({face2D}, precision);
        for (auto& tri : triVerts) {
            tri.x = face[tri.x];
            tri.y = face[tri.y];
            tri.z = face[tri.z];
        }

        // Append to result
        result.insert(result.end(), triVerts.begin(), triVerts.end());
    }
    return result;
}

manifold::Manifold Polyhedron(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<uint32_t>>& faces) {
    manifold::Mesh mesh;
    mesh.triVerts = TriangulateFaces(vertices, faces, -1.0);
    mesh.vertPos = vertices;

    return manifold::Manifold(mesh);
}

manifold::Manifold Polyhedron(double* vertices, std::size_t nVertices, int* faceBuf, int* faceLengths, std::size_t nFaces) {

    std::vector<glm::vec3> verts = BufferUtils::createDoubleVec3Vector(vertices, nVertices*3);

    std::vector<std::vector<uint32_t>> faces;
    for (std::size_t faceIdx = 0, faceBufIndex = 0; faceIdx < nFaces; faceIdx++) {
        std::size_t faceLength = (std::size_t) faceLengths[faceIdx];
        std::vector<uint32_t> face;
        for (size_t j = 0; j < faceLength; j++) {
            face.push_back((uint32_t) faceBuf[faceBufIndex]);
            faceBufIndex++;
        }
        faces.push_back(face);
    }

    return Polyhedron(verts, faces);
}

enum class LoftAlgorithm: long {
   EagerNearestNeighbor,
   Isomorphic
};

manifold::Manifold EagerNearestNeighborLoft(const std::vector<manifold::Polygons>& sections, const std::vector<glm::mat4x3>& transforms) {
    if (sections.size() != transforms.size()) {
      throw std::runtime_error("Mismatched number of sections and transforms");
    }
    if (sections.size() < 2) {
      throw std::runtime_error("Loft requires at least two sections.");
    }

    std::vector<glm::vec3> vertPos;
    std::vector<glm::ivec3> triVerts;

    size_t botSectionOffset = 0;
    for (std::size_t i = 0; i < sections.size() - 1; ++i) {
        const manifold::Polygons& botPolygons = sections[i];
        const manifold::Polygons& topPolygons = sections[i + 1];
        const glm::mat4x3& currentTransform = transforms[i];
        const glm::mat4x3& nextTransform = transforms[i + 1];

        if (botPolygons.size() != topPolygons.size()) {
          throw std::runtime_error("Cross sections must be composed of euqal number of polygons.");
        }

        size_t botSectionSize = 0;
        for (auto& poly : botPolygons)
          botSectionSize += poly.size();

        size_t topSectionOffset = botSectionOffset + botSectionSize;

        size_t botPolyOffset = 0;
        size_t topPolyOffset = 0;
        auto currPolyIt = botPolygons.begin();
        auto nextPolyIt = topPolygons.begin();
        for (int idx = 0; currPolyIt != botPolygons.end(); idx++, currPolyIt++, nextPolyIt++) {
          auto botPolygon = *currPolyIt;
          auto topPolygon = *nextPolyIt;

          std::vector<glm::vec3> botTransformed;
          std::vector<glm::vec3> topTransformed;

          for (const auto& vertex : botPolygon) {
              botTransformed.push_back(MatrixTransforms::Translate(currentTransform, glm::vec3(vertex.x, vertex.y, 0))[3]);
          }
          for (const auto& vertex : topPolygon) {
              topTransformed.push_back(MatrixTransforms::Translate(nextTransform, glm::vec3(vertex.x, vertex.y, 0))[3]);
          }

          vertPos.insert(vertPos.end(), botTransformed.begin(), botTransformed.end());

          float minDistance = std::numeric_limits<float>::max();
          size_t botStartVertOffset = 0,
            topStartVertOffset = 0;
          for (size_t j = 0; j < topTransformed.size(); ++j) {
            float dist = glm::distance(botTransformed[0], topTransformed[j]);
            if (dist < minDistance) {
              minDistance = dist;
              topStartVertOffset = j;
            }
          }

          bool botHasMoved = false,
            topHasMoved = false;
          size_t botVertOffset = botStartVertOffset,
            topVertOffset = topStartVertOffset;
          do {
              size_t botNextVertOffset = (botVertOffset + 1) % botTransformed.size();
              size_t topNextVertOffset = (topVertOffset + 1) % topTransformed.size();

              float distBotNextToTop = glm::distance(botTransformed[botNextVertOffset], topTransformed[topVertOffset]);
              float distBotToTopNext = glm::distance(botTransformed[botVertOffset], topTransformed[topNextVertOffset]);
              float distBotNextToTopNext = glm::distance(botTransformed[botNextVertOffset], topTransformed[topNextVertOffset]);

              bool botHasNext = botNextVertOffset != (botStartVertOffset + 1) % botTransformed.size() || !botHasMoved;
              bool topHasNext = topNextVertOffset != (topStartVertOffset + 1) % topTransformed.size() || !topHasMoved;

              if (distBotNextToTopNext < distBotNextToTop && distBotNextToTopNext <= distBotToTopNext && botHasNext && topHasNext) {
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        topSectionOffset + topPolyOffset + topNextVertOffset,
                                        topSectionOffset + topPolyOffset + topVertOffset);
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        botSectionOffset + botPolyOffset + botNextVertOffset,
                                        topSectionOffset + topPolyOffset + topNextVertOffset);
                  botVertOffset = botNextVertOffset;
                  topVertOffset = topNextVertOffset;
                  botHasMoved = true;
                  topHasMoved = true;
              } else if (distBotNextToTop < distBotToTopNext && botHasNext) {
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        botSectionOffset + botPolyOffset + botNextVertOffset,
                                        topSectionOffset + topPolyOffset + topVertOffset);
                  botVertOffset = botNextVertOffset;
                  botHasMoved = true;
              } else {
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        topSectionOffset + topPolyOffset + topNextVertOffset,
                                        topSectionOffset + topPolyOffset + topVertOffset);
                  topVertOffset = topNextVertOffset;
                  topHasMoved = true;
              }

          } while (botVertOffset != botStartVertOffset || topVertOffset != topStartVertOffset);
          botPolyOffset += botPolygon.size();
          topPolyOffset += topPolygon.size();
        }
        botSectionOffset += botSectionSize;
    }

    auto frontPolygons = sections.front();
    auto frontTriangles = manifold::Triangulate(frontPolygons, -1.0);
    for (auto& tri : frontTriangles) {
      triVerts.push_back({tri[2], tri[1], tri[0]});
    }

    auto backPolygons = sections.back();
    auto backTransform = transforms.back();
    for (const auto& poly: backPolygons) {
      for (const auto& vertex : poly) {
        vertPos.push_back(MatrixTransforms::Translate(backTransform, glm::vec3(vertex.x, vertex.y, 0))[3]);
      }
    }
    auto backTriangles = manifold::Triangulate(backPolygons, -1.0);

    for (auto& triangle : backTriangles) {
        triangle[0] += botSectionOffset;
        triangle[1] += botSectionOffset;
        triangle[2] += botSectionOffset;
        triVerts.push_back(triangle);
    }

    manifold::Mesh mesh;
    mesh.triVerts = triVerts;
    mesh.vertPos = vertPos;
    auto man = manifold::Manifold(mesh);
    return man;
}

manifold::Manifold IsomorphicLoft(const std::vector<manifold::Polygons>& sections, const std::vector<glm::mat4x3>& transforms) {
    std::vector<glm::vec3> vertPos;
    std::vector<glm::ivec3> triVerts;

    if (sections.size() != transforms.size()) {
        throw std::runtime_error("Mismatched number of sections and transforms");
    }

    std::size_t offset = 0;
    std::size_t nVerticesInEachSection = 0;

    for (std::size_t i = 0; i < sections.size(); ++i) {
        const manifold::Polygons polygons = sections[i];
        glm::mat4x3 transform = transforms[i];

        for (const auto& polygon : polygons) {
            for (const glm::vec2& vertex : polygon) {
                glm::vec3 translatedVertex = MatrixTransforms::Translate(transform, glm::vec3(vertex.x, vertex.y, 0))[3];
                vertPos.push_back(translatedVertex);
            }
        }

        if (i == 0) {
            nVerticesInEachSection = vertPos.size();
        } else if ((vertPos.size() % nVerticesInEachSection) != 0)  {
            throw std::runtime_error("Recieved CrossSection with different number of vertices");
        }

        if (i < sections.size() - 1) {
            std::size_t currentOffset = offset;
            std::size_t nextOffset = offset + nVerticesInEachSection;

            for (std::size_t j = 0; j < polygons.size(); ++j) {
                const auto& polygon = polygons[j];

                for (std::size_t k = 0; k < polygon.size(); ++k) {
                    std::size_t nextIndex = (k + 1) % polygon.size();

                    glm::ivec3 triangle1(currentOffset + k, currentOffset + nextIndex, nextOffset + k);
                    glm::ivec3 triangle2(currentOffset + nextIndex, nextOffset + nextIndex, nextOffset + k);

                    triVerts.push_back(triangle1);
                    triVerts.push_back(triangle2);
                }
                currentOffset += polygon.size();
                nextOffset += polygon.size();
            }
        }

        offset += nVerticesInEachSection;
    }

    auto frontPolygons = sections.front();
    auto frontTriangles = manifold::Triangulate(frontPolygons, -1.0);
    for (auto& tri : frontTriangles) {
        triVerts.push_back({tri.z, tri.y, tri.x});
    }

    auto backPolygons = sections.back();
    auto backTriangles = manifold::Triangulate(backPolygons, -1.0);
    for (auto& triangle : backTriangles) {
        triangle.x += offset - nVerticesInEachSection;
        triangle.y += offset - nVerticesInEachSection;
        triangle.z += offset - nVerticesInEachSection;
        triVerts.push_back(triangle);
    }

    manifold::Mesh mesh;
    mesh.triVerts = triVerts;
    mesh.vertPos = vertPos;
    return manifold::Manifold(mesh);
}

manifold::Manifold Loft(const std::vector<manifold::Polygons>& sections, const std::vector<glm::mat4x3>& transforms, LoftAlgorithm algorithm) {
    switch (algorithm) {
        case LoftAlgorithm::EagerNearestNeighbor:
            return EagerNearestNeighborLoft(sections, transforms);
        case LoftAlgorithm::Isomorphic:
            return IsomorphicLoft(sections, transforms);
        default:
            return EagerNearestNeighborLoft(sections, transforms);
    }
}

manifold::Manifold Loft(const std::vector<manifold::Polygons>& sections, const std::vector<glm::mat4x3>& transforms) {
    return EagerNearestNeighborLoft(sections, transforms);
}

manifold::Manifold Loft(const manifold::Polygons& sections, const std::vector<glm::mat4x3>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back({section});
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const manifold::Polygons& sections, const std::vector<glm::mat4x3>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back({section});
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const manifold::SimplePolygon& section, const std::vector<glm::mat4x3>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (std::size_t i = 0; i < transforms.size(); i++) {
        polys.push_back({section});
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const manifold::SimplePolygon& section, const std::vector<glm::mat4x3>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (std::size_t i = 0; i < transforms.size(); i++) {
        polys.push_back({section});
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const std::vector<manifold::CrossSection>& sections, const std::vector<glm::mat4x3>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back(section.ToPolygons());
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const std::vector<manifold::CrossSection>& sections, const std::vector<glm::mat4x3>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back(section.ToPolygons());
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const manifold::CrossSection section, const std::vector<glm::mat4x3>& transforms) {
    std::vector<manifold::Polygons> sections(transforms.size());
    auto polys = section.ToPolygons();
    for (std::size_t i = 0; i < transforms.size(); i++) {
        sections[i] = polys;
    }
    return Loft(sections, transforms);
}

manifold::Manifold Loft(const manifold::CrossSection section, const std::vector<glm::mat4x3>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> sections(transforms.size());
    auto polys = section.ToPolygons();
    for (std::size_t i = 0; i < transforms.size(); i++) {
        sections[i] = polys;
    }
    return Loft(sections, transforms, algorithm);
}

}
