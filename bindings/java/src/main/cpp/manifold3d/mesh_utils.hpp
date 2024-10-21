#pragma once

#include <stdexcept>
#include <glm/glm.hpp>
#include <vector>
#include "polygon.h"
#include "manifold.h"
#include "cross_section.h"
#include "buffer_utils.hpp"
#include "matrix_transforms.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
namespace MeshUtils {


//#include <glm/vec3.hpp>     // Include GLM for vector operations

manifold::Manifold CreateSurface(const double* heightMap, int width, int height, double pixelWidth = 1.0) {
    // Calculate the number of vertices and triangles in advance
    int numVertices = 2 * width * height; // Top and bottom vertices
    int numTriangles = 2 * (width - 1) * (height - 1) * 2 + 2 * (width - 1 + height - 1) * 2; // Top, bottom, and sides

    // Preallocate vectors
    std::vector<glm::vec3> vertices;
    vertices.reserve(numVertices);
    std::vector<glm::ivec3> triangles;
    triangles.reserve(numTriangles);

    // Generate top surface vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            double heightValue = heightMap[i * width + j];
            vertices.emplace_back(j * pixelWidth, i * pixelWidth, heightValue); // Top vertex scaled by pixelWidth
        }
    }

    // Generate bottom surface vertices (z = 0)
    int bottomOffset = vertices.size();  // Bottom vertices start after the top vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            vertices.emplace_back(j * pixelWidth, i * pixelWidth, 0.0);
        }
    }

    // Generate triangles for the top and bottom surfaces
    for (int i = 0; i < height - 1; ++i) {
        for (int j = 0; j < width - 1; ++j) {
            int topLeft = i * width + j;
            int topRight = i * width + (j + 1);
            int bottomLeft = (i + 1) * width + j;
            int bottomRight = (i + 1) * width + (j + 1);

            // Top surface triangles (counterclockwise)
            triangles.emplace_back(bottomRight, bottomLeft, topLeft);
            triangles.emplace_back(topRight, bottomRight, topLeft);

            // Bottom surface triangles (clockwise)
            int bTopLeft = bottomOffset + topLeft;
            int bTopRight = bottomOffset + topRight;
            int bBottomLeft = bottomOffset + bottomLeft;
            int bBottomRight = bottomOffset + bottomRight;
            triangles.emplace_back(bBottomLeft, bBottomRight, bTopLeft);
            triangles.emplace_back(bTopLeft, bBottomRight, bTopRight);
        }
    }

    // Left edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width;
        int tBottom = (i + 1) * width;
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        // Left side triangles
        triangles.emplace_back(tTop, tBottom, bBottom);
        triangles.emplace_back(tTop, bBottom, bTop);
    }

    // Right edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width + (width - 1);
        int tBottom = (i + 1) * width + (width - 1);
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        // Right side triangles
        triangles.emplace_back(tTop, bBottom, tBottom);
        triangles.emplace_back(tTop, bTop, bBottom);
    }

    // Top edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = j;
        int tRight = j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        // Top side triangles
        triangles.emplace_back(bLeft, bRight, tRight);
        triangles.emplace_back(tLeft, bLeft, tRight);
    }

    // Bottom edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = (height - 1) * width + j;
        int tRight = (height - 1) * width + j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        // Bottom side triangles
        triangles.emplace_back(tLeft, tRight, bRight);
        triangles.emplace_back(tLeft, bRight, bLeft);
    }

    // Step 5: Create and validate the manifold
    manifold::Manifold solid = manifold::Manifold({vertices, triangles});
    manifold::Manifold::Error status = solid.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    return solid;
}

manifold::Manifold CreateSurface(const std::string& texturePath, double pixelWidth = 1.0) {
    // Load the texture image as a grayscale image
    int width, height, channels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &channels, 1);
    if (!data) {
        throw std::runtime_error("Failed to load texture image.");
    }

    // Create a height map from the texture data
    std::vector<double> heightMap(width * height);
    for (int i = 0; i < width * height; ++i) {
        // Copy pixel value directly into the height map
        heightMap[i] = static_cast<double>(data[i]);
    }
    stbi_image_free(data); // Free the image data

    // Invoke the overloaded function with the height map
    return CreateSurface(heightMap.data(), width, height, pixelWidth);
}

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

glm::vec2 calculatePolygonCentroid(const std::vector<glm::vec2>& vertices) {
    if (vertices.size() < 3) {
        throw std::invalid_argument("A polygon must have at least 3 vertices.");
    }

    double centroidX = 0.0;
    double centroidY = 0.0;
    double signedArea = 0.0;
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double a = 0.0;  // Partial signed area

    size_t i = 0;
    for (i = 0; i < vertices.size() - 1; ++i) {
        x0 = vertices[i].x;
        y0 = vertices[i].y;
        x1 = vertices[i+1].x;
        y1 = vertices[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroidX += (x0 + x1) * a;
        centroidY += (y0 + y1) * a;
    }

    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[0].x;
    y1 = vertices[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroidX += (x0 + x1) * a;
    centroidY += (y0 + y1) * a;

    signedArea *= 0.5;
    centroidX /= (6.0 * signedArea);
    centroidY /= (6.0 * signedArea);

    return glm::vec2(centroidX, centroidY);
}

manifold::Manifold EagerNearestNeighborLoft(const std::vector<manifold::Polygons>& sections, const std::vector<glm::mat4x3>& transforms) {
    if (sections.size() != transforms.size()) {
      throw std::runtime_error("Mismatched number of sections and transforms");
    }
    if (sections.size() < 2) {
      throw std::runtime_error("Loft requires at least two sections.");
    }

    size_t nVerts = 0;
    std::vector<size_t> sectionSizes;
    sectionSizes.reserve(sections.size());
    for (auto& section: sections) {
        size_t sectionSize = 0;
        for (auto& poly: section) {
            sectionSize += poly.size();
        }
        nVerts += sectionSize;
        sectionSizes.push_back(sectionSize);
    }

    std::vector<glm::vec3> vertPos;
    vertPos.reserve(nVerts);
    std::vector<glm::ivec3> triVerts;
    triVerts.reserve(2*nVerts);

    size_t botSectionOffset = 0;
    for (std::size_t i = 0; i < sections.size() - 1; ++i) {
        const manifold::Polygons& botPolygons = sections[i];
        const manifold::Polygons& topPolygons = sections[i + 1];
        const glm::mat4x3& botTransform = transforms[i];

        if (botPolygons.size() != topPolygons.size()) {
          throw std::runtime_error("Cross sections must be composed of euqal number of polygons.");
        }

        size_t botSectionSize = sectionSizes[i];
        size_t topSectionOffset = botSectionOffset + botSectionSize;

        size_t botPolyOffset = 0;
        size_t topPolyOffset = 0;
        auto currPolyIt = botPolygons.begin();
        auto nextPolyIt = topPolygons.begin();
        for (int idx = 0; currPolyIt != botPolygons.end(); idx++, currPolyIt++, nextPolyIt++) {
          auto botPolygon = *currPolyIt;
          auto topPolygon = *nextPolyIt;

          glm::vec2 botCentroid = calculatePolygonCentroid(botPolygon);
          glm::vec2 topCentroid = calculatePolygonCentroid(topPolygon);
          glm::vec2 centroidOffset = topCentroid - botCentroid;

          for (const auto& vertex : botPolygon) {
              vertPos.push_back(MatrixTransforms::Translate(botTransform, glm::vec3(vertex.x, vertex.y, 0))[3]);
          }

          float minDistance = std::numeric_limits<float>::max();
          size_t botStartVertOffset = 0,
            topStartVertOffset = 0;
          for (size_t j = 0; j < topPolygon.size(); ++j) {
            float dist = glm::distance(botPolygon[0], topPolygon[j] - centroidOffset);
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
              size_t botNextVertOffset = (botVertOffset + 1) % botPolygon.size();
              size_t topNextVertOffset = (topVertOffset + 1) % topPolygon.size();

              float distBotNextToTop = glm::distance(botPolygon[botNextVertOffset], topPolygon[topVertOffset] - centroidOffset);
              float distBotToTopNext = glm::distance(botPolygon[botVertOffset], topPolygon[topNextVertOffset] - centroidOffset);
              float distBotNextToTopNext = glm::distance(botPolygon[botNextVertOffset], topPolygon[topNextVertOffset] - centroidOffset);

              bool botHasNext = botNextVertOffset != (botStartVertOffset + 1) % botPolygon.size() || !botHasMoved;
              bool topHasNext = topNextVertOffset != (topStartVertOffset + 1) % topPolygon.size() || !topHasMoved;

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
    for (auto& section : sections) {
        polys.push_back({section});
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const manifold::Polygons& sections, const std::vector<glm::mat4x3>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (auto& section : sections) {
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
