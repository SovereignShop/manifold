#pragma once

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

manifold::Manifold Loft(const std::vector<manifold::CrossSection>& sections, const std::vector<glm::mat4x3>& transforms) {
    std::vector<glm::vec3> vertPos;
    std::vector<glm::ivec3> triVerts;

    if (sections.size() != transforms.size()) {
        throw std::runtime_error("Mismatched number of sections and transforms");
    }

    std::size_t offset = 0;
    std::size_t nVerticesInEachSection = 0;

    for (std::size_t i = 0; i < sections.size(); ++i) {
        const auto& polygons = sections[i].ToPolygons();
        glm::mat4x3 transform = transforms[i];

        for (const auto& polygon : polygons) {
            for (const glm::vec2& vertex : polygon) {
                glm::vec3 translatedVertex = MatrixTransforms::Translate(transform, glm::vec3(vertex.x, vertex.y, 0))[3];
                vertPos.push_back(translatedVertex);
            }
        }

        if (i == 0) {
            nVerticesInEachSection = vertPos.size();
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

    auto frontPolygons = sections.front().ToPolygons();
    auto frontTriangles = manifold::Triangulate(frontPolygons, -1.0);
    for (auto& tri : frontTriangles) {
        triVerts.push_back({tri.z, tri.y, tri.x});
    }

    auto backPolygons = sections.back().ToPolygons();
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

}
