#pragma once

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "manifold/linalg.h"
#include "manifold/polygon.h"
#include "manifold/manifold.h"
#include "manifold/cross_section.h"
#include "happly.h"
#include "buffer_utils.hpp"
#include "matrix_transforms.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace MeshUtils {

using vec2   = linalg::vec<double, 2>;
using vec3   = linalg::vec<double, 3>;
using vec4   = linalg::vec<double, 4>;
using ivec3  = linalg::vec<int, 3>;
using mat2x2 = linalg::mat<double, 2, 2>;
using mat3x3 = linalg::mat<double, 3, 3>;
using mat4x3 = linalg::mat<double, 4, 3>;
using mat3x4 = linalg::mat<double, 3, 4>;

manifold::Manifold CreateManifold(const std::vector<vec3>& vertices, const std::vector<ivec3>& triVerts) {
    manifold::MeshGL mesh;
    mesh.numProp = 3;
    mesh.triVerts.reserve(3 * triVerts.size());
    for (auto& triVert: triVerts) {
        mesh.triVerts.push_back(triVert[0]);
        mesh.triVerts.push_back(triVert[1]);
        mesh.triVerts.push_back(triVert[2]);
    }
    mesh.vertProperties.reserve(3 * vertices.size());
    for (auto& vert: vertices) {
        mesh.vertProperties.push_back(vert[0]);
        mesh.vertProperties.push_back(vert[1]);
        mesh.vertProperties.push_back(vert[2]);
    }

    auto man = manifold::Manifold(mesh);
    manifold::Manifold::Error status = man.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }
    return man;
}

manifold::Manifold CreateSurface(const float* vertProperties, int numProps, int width, int height, float pixelWidth = 1.0) {
    // Create the MeshGL structure
    manifold::MeshGL meshGL;

    // Set number of vertex properties based on numProps
    meshGL.numProp = numProps + 2;
    int numVerts = width * height * meshGL.numProp * 2;
    meshGL.vertProperties.reserve(numVerts);
    int numTopBottomTriangles = 4 * (width - 1) * (height - 1);
    int numEdgeTriangles = 4 * (height - 1) + 4 * (width - 1);
    int numTriangles = numTopBottomTriangles + numEdgeTriangles;
    meshGL.triVerts.reserve(3 * numTriangles);

    // Generate top surface vertices and properties
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            const float* props = &vertProperties[(i * width + j) * numProps];
            float x = j * pixelWidth;
            float y = i * pixelWidth;
            float z = props[0];  // Height (z)

            // Add vertex properties (x, y, z)
            meshGL.vertProperties.push_back(x);
            meshGL.vertProperties.push_back(y);
            meshGL.vertProperties.push_back(z);

            // Add additional properties from index 3 to numProps
            for (int k = 1; k < numProps; ++k) {
                meshGL.vertProperties.push_back(props[k]);
            }
        }
    }

    // Generate bottom surface vertices (z = 0)
    int bottomOffset = width * height;  // Bottom vertices start after the top vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            // Add bottom vertex properties (x, y, z=0)
            meshGL.vertProperties.push_back(j * pixelWidth);
            meshGL.vertProperties.push_back(i * pixelWidth);
            meshGL.vertProperties.push_back(0.0);

            // Set remaining properties to 0 from index 3 to numProps
            for (int k = 1; k < numProps; ++k) {
                meshGL.vertProperties.push_back(0.0);
            }
        }
    }

    // Step 3: Generate triangles for the top and bottom surfaces
    for (int i = 0; i < height - 1; ++i) {
        for (int j = 0; j < width - 1; ++j) {
            int topLeft = i * width + j;
            int topRight = i * width + (j + 1);
            int bottomLeft = (i + 1) * width + j;
            int bottomRight = (i + 1) * width + (j + 1);

            // Top surface triangles (counterclockwise)
            meshGL.triVerts.push_back(bottomRight);
            meshGL.triVerts.push_back(bottomLeft);
            meshGL.triVerts.push_back(topLeft);

            meshGL.triVerts.push_back(topRight);
            meshGL.triVerts.push_back(bottomRight);
            meshGL.triVerts.push_back(topLeft);

            // Bottom surface triangles (clockwise)
            int bTopLeft = bottomOffset + topLeft;
            int bTopRight = bottomOffset + topRight;
            int bBottomLeft = bottomOffset + bottomLeft;
            int bBottomRight = bottomOffset + bottomRight;
            meshGL.triVerts.push_back(bBottomLeft);
            meshGL.triVerts.push_back(bBottomRight);
            meshGL.triVerts.push_back(bTopLeft);

            meshGL.triVerts.push_back(bTopLeft);
            meshGL.triVerts.push_back(bBottomRight);
            meshGL.triVerts.push_back(bTopRight);
        }
    }

    // Step 4: Generate triangles for the sides (left, right, top, bottom)
    // Left edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width;
        int tBottom = (i + 1) * width;
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(tBottom);
        meshGL.triVerts.push_back(bBottom);

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(bBottom);
        meshGL.triVerts.push_back(bTop);
    }

    // Right edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width + (width - 1);
        int tBottom = (i + 1) * width + (width - 1);
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(bBottom);
        meshGL.triVerts.push_back(tBottom);

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(bTop);
        meshGL.triVerts.push_back(bBottom);
    }

    // Top edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = j;
        int tRight = j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        meshGL.triVerts.push_back(bLeft);
        meshGL.triVerts.push_back(bRight);
        meshGL.triVerts.push_back(tRight);

        meshGL.triVerts.push_back(tLeft);
        meshGL.triVerts.push_back(bLeft);
        meshGL.triVerts.push_back(tRight);
    }

    // Bottom edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = (height - 1) * width + j;
        int tRight = (height - 1) * width + j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        meshGL.triVerts.push_back(tLeft);
        meshGL.triVerts.push_back(tRight);
        meshGL.triVerts.push_back(bRight);

        meshGL.triVerts.push_back(tLeft);
        meshGL.triVerts.push_back(bRight);
        meshGL.triVerts.push_back(bLeft);
    }

    // Create and validate the manifold
    manifold::Manifold solid = manifold::Manifold(meshGL);
    manifold::Manifold::Error status = solid.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    return solid;
}


manifold::Manifold PlyToSurface(const std::string &filepath, double cellSize, double zOffset, double scaleFactor) {
    // Create a reader for the PLY file
    happly::PLYData plyIn(filepath);

    std::vector<float> vX = plyIn.getElement("vertex").getProperty<float>("x");
    std::vector<float> vY = plyIn.getElement("vertex").getProperty<float>("y");
    std::vector<float> vZ = plyIn.getElement("vertex").getProperty<float>("z");

    std::vector<uint8_t> vR = plyIn.getElement("vertex").getProperty<uint8_t>("red");
    std::vector<uint8_t> vG = plyIn.getElement("vertex").getProperty<uint8_t>("green");
    std::vector<uint8_t> vB = plyIn.getElement("vertex").getProperty<uint8_t>("blue");

    float min_x = *std::min_element(vX.begin(), vX.end());
    float max_x = *std::max_element(vX.begin(), vX.end());
    float min_y = *std::min_element(vY.begin(), vY.end());
    float max_y = *std::max_element(vY.begin(), vY.end());
    float min_z = *std::min_element(vZ.begin(), vZ.end());

    // Calculate the spans for x and y
    double x_span = (max_x - min_x) * scaleFactor;
    double y_span = (max_y - min_y) * scaleFactor;

    int grid_resolution_x = static_cast<int>(x_span / cellSize);
    int grid_resolution_y = static_cast<int>(y_span / cellSize);

    // Ensure at least one cell is created in both directions
    grid_resolution_x = std::max(1, grid_resolution_x);
    grid_resolution_y = std::max(1, grid_resolution_y);

    // Initialize grid structures for z-value sums, color sums, and point counts
    std::vector<std::vector<float>> z_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<float>> r_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<float>> g_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<float>> b_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<int>> point_count(grid_resolution_x, std::vector<int>(grid_resolution_y, 0));

    // Process each point in the PLY file
    for (size_t i = 0; i < vX.size(); ++i) {
        float x = vX[i];
        float y = vY[i];
        float z = vZ[i];
        float r = static_cast<float>(vR[i]) / 255.0;
        float g = static_cast<float>(vG[i]) / 255.0;
        float b = static_cast<float>(vB[i]) / 255.0;

        // Find the corresponding grid cell indices
        int grid_x = static_cast<int>(((x - min_x) * scaleFactor) / cellSize);
        int grid_y = static_cast<int>(((y - min_y) * scaleFactor) / cellSize);

        // Ensure the point falls within the grid bounds
        if (grid_x >= 0 && grid_x < grid_resolution_x && grid_y >= 0 && grid_y < grid_resolution_y) {
            // Accumulate the z and color values in the corresponding grid cell
            z_sum[grid_x][grid_y] += (z - min_z) * scaleFactor;
            r_sum[grid_x][grid_y] += r;
            g_sum[grid_x][grid_y] += g;
            b_sum[grid_x][grid_y] += b;
            point_count[grid_x][grid_y] += 1;
        }
    }
    int nProp = 4;

    // Initialize vertProperties to store the flattened vertex data
    std::vector<float> vertProperties;
    vertProperties.reserve(grid_resolution_x * grid_resolution_y * nProp);  // Reserve space for z, r, g, b per cell

    // Compute the average height and color for each grid cell and populate vertProperties
    for (int i = 0; i < grid_resolution_x; ++i) {
        for (int j = 0; j < grid_resolution_y; ++j) {
            if (point_count[i][j] > 0) {
                // Calculate the average height, apply offset, and set default color
                float avg_z = (z_sum[i][j] / point_count[i][j]) + zOffset;
                float avg_r = r_sum[i][j] / point_count[i][j];
                float avg_g = g_sum[i][j] / point_count[i][j];
                float avg_b = b_sum[i][j] / point_count[i][j];
                // Push x, y, z, r, g, b for each grid cell in row-major order
                float* props = &vertProperties[(j * grid_resolution_x + i) * nProp];
                props[0] = avg_z;
                props[1] = avg_r;
                props[2] = avg_g;
                props[3] = avg_b;
            } else {
                float* props = &vertProperties[(j * grid_resolution_x + i) * nProp];
                props[0] = 10.0;
                props[1] = 0.0;
                props[2] = 0.0;
                props[3] = 0.0;
            }

        }
    }

    // Pass vertProperties to CreateSurface as a pointer and specify numProps = 6
    return CreateSurface(vertProperties.data(), nProp, grid_resolution_x, grid_resolution_y, cellSize);
}

manifold::Manifold ColorVertices(const manifold::Manifold& man, const vec4 color, size_t propIndex = 3) {
    manifold::MeshGL mesh = man.GetMeshGL();
    const std::vector<float>& vertProps = mesh.vertProperties;
    size_t numProps = mesh.numProp;

    size_t numNewProps = std::max(propIndex + 4, static_cast<size_t>(numProps));
    std::vector<float> newVertProps;
    newVertProps.resize(numNewProps * mesh.NumVert());

    for (size_t i = 0; i < mesh.NumVert(); ++i) {
        for (size_t j = 0; j < numProps; ++j) {
            newVertProps[i * numNewProps + j] = vertProps[i * numProps + j];
        }

        for (size_t j = 0; j < 4; ++j) {
            newVertProps[i * numNewProps + propIndex + j] = color[j];
        }
    }

    manifold::MeshGL newMesh;
    newMesh.vertProperties = std::move(newVertProps);
    newMesh.numProp = numNewProps;
    newMesh.triVerts = mesh.triVerts;

    return manifold::Manifold(newMesh);
}


manifold::Manifold CreateSurface(const std::string& texturePath, double pixelWidth = 1.0) {
    // Load the texture image in its original format to determine channels
    int width, height, channels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &channels, 0);  // 0 keeps original channels
    if (!data) {
        throw std::runtime_error("Failed to load texture image.");
    }

    // Set the number of properties based on the image format
    int numProps = channels;  // 1 for grayscale, 3 for RGB, 4 for RGBA

    // Create a property map from the texture data
    std::vector<float> propertyMap(width * height * numProps);
    for (int i = 0; i < width * height; ++i) {
        for (int c = 0; c < channels; ++c) {
            propertyMap[i * numProps + c] = static_cast<float>(data[i * channels + c]) / 255.0f;  // Normalize to [0, 1]
        }
    }
    stbi_image_free(data); // Free the image data

    // Invoke the overloaded CreateSurface function with the property map and numProps
    return CreateSurface(propertyMap.data(), numProps, width, height, pixelWidth);
}

manifold::Manifold LoadImage(const std::string& texturePath, const float depth, double pixelWidth = 1.0) {
    // Load the texture image in its original format to determine channels
    int width, height, channels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &channels, 0);  // 0 keeps original channels
    if (!data) {
        throw std::runtime_error("Failed to load texture image.");
    }

    // Set the number of properties based on the image format
    int numProps = channels + 1;

    // Create a property map from the texture data
    std::vector<float> propertyMap(width * height * numProps);
    for (int i = 0; i < width * height; ++i) {
        propertyMap[i * numProps] = depth;
        for (int c = 1; c < channels; ++c) {
            propertyMap[i * numProps + c] = static_cast<float>(data[i * channels + c]) / 255.0f;  // Normalize to [0, 1]
        }
    }
    stbi_image_free(data); // Free the image data

    // Invoke the overloaded CreateSurface function with the property map and numProps
    return CreateSurface(propertyMap.data(), numProps, width, height, pixelWidth);
}

std::vector<ivec3> TriangulateFaces(const std::vector<vec3>& vertices, const std::vector<std::vector<uint32_t>>& faces, float precision) {
    std::vector<ivec3> result;
    for (const auto& face : faces) {
        // If the face only has 3 vertices, no need to triangulate, just add it to result
        if (face.size() == 3) {
            result.push_back(ivec3(face[0], face[1], face[2]));
            continue;
        }

        // Compute face normal
        vec3 normal = linalg::cross(vertices[face[1]] - vertices[face[0]], vertices[face[2]] - vertices[face[0]]);
        normal = linalg::normalize(normal);

        // Compute reference right vector
        vec3 right = linalg::normalize(vertices[face[1]] - vertices[face[0]]);

        // Compute up vector
        vec3 up = linalg::cross(right, normal);

        // Project vertices onto plane
        std::vector<vec2> face2D;
        for (const auto& index : face) {
            vec3 local = vertices[index] - vertices[face[0]];
            face2D.push_back(vec2(linalg::dot(local, right), linalg::dot(local, up)));
        }

        // Triangulate and remap the triangulated vertices back to the original indices
        std::vector<ivec3> triVerts = manifold::Triangulate({face2D}, precision);
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

manifold::Manifold Polyhedron(const std::vector<vec3>& vertices, const std::vector<std::vector<uint32_t>>& faces) {
    std::vector<ivec3> triVerts = TriangulateFaces(vertices, faces, -1.0);
    return CreateManifold(vertices, triVerts);
}

manifold::Manifold Polyhedron(double* vertices, std::size_t nVertices, int* faceBuf, int* faceLengths, std::size_t nFaces) {

    std::vector<vec3> verts = BufferUtils::createDoubleVec3Vector(vertices, nVertices*3);

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

vec2 calculatePolygonCentroid(const std::vector<vec2>& vertices) {
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

    return vec2(centroidX, centroidY);
}

manifold::Manifold EagerNearestNeighborLoft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms) {
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

    std::vector<vec3> vertPos;
    vertPos.reserve(nVerts);
    std::vector<ivec3> triVerts;
    triVerts.reserve(2*nVerts);

    size_t botSectionOffset = 0;
    for (std::size_t i = 0; i < sections.size() - 1; ++i) {
        const manifold::Polygons& botPolygons = sections[i];
        const manifold::Polygons& topPolygons = sections[i + 1];
        const mat3x4& botTransform = transforms[i];
        const mat3x4& topTransform = transforms[i + 1];
        std::cout << "Bot transform: " << botTransform[3][0] << " " << botTransform[3][1] << " " << botTransform[3][2] << std::endl;
        std::cout << "Top transform: " << topTransform[3][0] << " " << topTransform[3][1] << " " << topTransform[3][2] << std::endl;

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

          vec2 botCentroid = calculatePolygonCentroid(botPolygon);
          vec2 topCentroid = calculatePolygonCentroid(topPolygon);
          vec2 centroidOffset = topCentroid - botCentroid;

          for (const auto& vertex : botPolygon) {
              vertPos.push_back(MatrixTransforms::Translate(botTransform, vec3(vertex.x, vertex.y, 0))[3]);
          }

          float minDistance = std::numeric_limits<float>::max();
          size_t botStartVertOffset = 0,
            topStartVertOffset = 0;
          for (size_t j = 0; j < topPolygon.size(); ++j) {
            float dist = linalg::distance(botPolygon[0], topPolygon[j] - centroidOffset);
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

              float distBotNextToTop = linalg::distance(botPolygon[botNextVertOffset], topPolygon[topVertOffset] - centroidOffset);
              float distBotToTopNext = linalg::distance(botPolygon[botVertOffset], topPolygon[topNextVertOffset] - centroidOffset);
              float distBotNextToTopNext = linalg::distance(botPolygon[botNextVertOffset], topPolygon[topNextVertOffset] - centroidOffset);

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
        vertPos.push_back(MatrixTransforms::Translate(backTransform, vec3(vertex.x, vertex.y, 0))[3]);
      }
    }
    auto backTriangles = manifold::Triangulate(backPolygons, -1.0);

    for (auto& triangle : backTriangles) {
        triangle[0] += botSectionOffset;
        triangle[1] += botSectionOffset;
        triangle[2] += botSectionOffset;
        triVerts.push_back(triangle);
    }

    auto man = CreateManifold(vertPos, triVerts);
    return man;
}

manifold::Manifold IsomorphicLoft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms) {
    std::vector<vec3> vertPos;
    std::vector<ivec3> triVerts;

    if (sections.size() != transforms.size()) {
        throw std::runtime_error("Mismatched number of sections and transforms");
    }

    std::size_t offset = 0;
    std::size_t nVerticesInEachSection = 0;

    for (std::size_t i = 0; i < sections.size(); ++i) {
        const manifold::Polygons polygons = sections[i];
        mat3x4 transform = transforms[i];

        for (const auto& polygon : polygons) {
            for (const vec2& vertex : polygon) {
                vec3 translatedVertex = MatrixTransforms::Translate(transform, vec3(vertex.x, vertex.y, 0))[3];
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

                    ivec3 triangle1(currentOffset + k, currentOffset + nextIndex, nextOffset + k);
                    ivec3 triangle2(currentOffset + nextIndex, nextOffset + nextIndex, nextOffset + k);

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
    return CreateManifold(vertPos, triVerts);
}

manifold::Manifold Loft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    switch (algorithm) {
        case LoftAlgorithm::EagerNearestNeighbor:
            return EagerNearestNeighborLoft(sections, transforms);
        case LoftAlgorithm::Isomorphic:
            return IsomorphicLoft(sections, transforms);
        default:
            return EagerNearestNeighborLoft(sections, transforms);
    }
}

manifold::Manifold Loft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms) {
    return EagerNearestNeighborLoft(sections, transforms);
}

manifold::Manifold Loft(const manifold::Polygons& sections, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (auto& section : sections) {
        polys.push_back({section});
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const manifold::Polygons& sections, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (auto& section : sections) {
        polys.push_back({section});
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const manifold::SimplePolygon& section, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (std::size_t i = 0; i < transforms.size(); i++) {
        polys.push_back({section});
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const manifold::SimplePolygon& section, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (std::size_t i = 0; i < transforms.size(); i++) {
        polys.push_back({section});
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const std::vector<manifold::CrossSection>& sections, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back(section.ToPolygons());
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const std::vector<manifold::CrossSection>& sections, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back(section.ToPolygons());
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const manifold::CrossSection section, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> sections(transforms.size());
    auto polys = section.ToPolygons();
    for (std::size_t i = 0; i < transforms.size(); i++) {
        sections[i] = polys;
    }
    return Loft(sections, transforms);
}

manifold::Manifold Loft(const manifold::CrossSection section, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> sections(transforms.size());
    auto polys = section.ToPolygons();
    for (std::size_t i = 0; i < transforms.size(); i++) {
        sections[i] = polys;
    }
    return Loft(sections, transforms, algorithm);
}

}
