#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Vector_3.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <stdexcept>
#include <glm/glm.hpp>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include "polygon.h"
#include "manifold.h"
#include "meshIO.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "cross_section.h"
#include "public.h"
//#include "buffer_utils.hpp"
//#include "matrix_transforms.hpp"
//#include "stb_image.h"      // Include stb_image for image loading
#define STB_IMAGE_IMPLEMENTATION
#include "happly.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


// Define the point type
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Advancing_front_surface_reconstruction<> Reconstruction;
typedef Reconstruction::Triangulation_3 Triangulation_3;
typedef Reconstruction::Triangulation_data_structure_2 TDS_2;

namespace MeshUtils {

manifold::Manifold CreateSurface(const std::vector<std::array<double, 5>>& heightMap, int width, int height, double pixelWidth = 1.0) {
    // Create the MeshGL structure
    manifold::MeshGL meshGL;

    // We have 7 properties: x, y, z for position and r, g, b, a for color with alpha
    meshGL.numProp = 7;

    // Step 1: Generate top surface vertices with RGBA colors
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            const std::array<double, 5>& data = heightMap[i * width + j];
            double heightValue = data[0]; // Height
            float r = static_cast<float>(data[1]); // Red
            float g = static_cast<float>(data[2]); // Green
            float b = static_cast<float>(data[3]); // Blue
            float a = static_cast<float>(data[4]); // Alpha

            // Add vertex properties (x, y, z, r, g, b, a)
            meshGL.vertProperties.push_back(j * pixelWidth); // x
            meshGL.vertProperties.push_back(i * pixelWidth); // y
            meshGL.vertProperties.push_back(heightValue);    // z
            meshGL.vertProperties.push_back(r);              // r
            meshGL.vertProperties.push_back(g);              // g
            meshGL.vertProperties.push_back(b);              // b
            meshGL.vertProperties.push_back(a);              // a
        }
    }

    // Step 2: Generate bottom surface vertices (z = 0) and default color (opaque black)
    int bottomOffset = width * height;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            meshGL.vertProperties.push_back(j * pixelWidth); // x
            meshGL.vertProperties.push_back(i * pixelWidth); // y
            meshGL.vertProperties.push_back(0.0);            // z
            meshGL.vertProperties.push_back(0.0);            // r
            meshGL.vertProperties.push_back(0.0);            // g
            meshGL.vertProperties.push_back(0.0);            // b
            meshGL.vertProperties.push_back(1.0);            // a (fully opaque)
        }
    }

    // Step 3: Generate triangles for the top and bottom surfaces
    for (int i = 0; i < height - 1; ++i) {
        for (int j = 0; j < width - 1; ++j) {
            int topLeft = i * width + j;
            int topRight = i * width + (j + 1);
            int bottomLeft = (i + 1) * width + j;
            int bottomRight = (i + 1) * width + (j + 1);

            // Top surface triangles
            meshGL.triVerts.push_back(bottomRight);
            meshGL.triVerts.push_back(bottomLeft);
            meshGL.triVerts.push_back(topLeft);
            meshGL.triVerts.push_back(topRight);
            meshGL.triVerts.push_back(bottomRight);
            meshGL.triVerts.push_back(topLeft);

            // Bottom surface triangles
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

    // Side triangles (left, right, top, bottom) generated similarly as before (omitted for brevity)

    // Step 5: Create and validate the manifold
    manifold::Manifold solid = manifold::Manifold(meshGL);
    manifold::Manifold::Error status = solid.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    return solid;
}

manifold::Manifold CreateSurface(const double* heightMap, const std::vector<glm::vec4> vertColor, int width, int height, double pixelWidth = 1.0) {
    // Calculate the number of vertices and triangles in advance
    int numVertices = 2 * width * height; // Top and bottom vertices
    int numTriangles = 2 * (width - 1) * (height - 1) * 2 + 2 * (width - 1 + height - 1) * 2; // Top, bottom, and sides

    // Preallocate vectors
    std::vector<glm::vec3> vertices;
    vertices.reserve(numVertices);
    std::vector<glm::ivec3> triangles;
    triangles.reserve(numTriangles);
    std::vector<glm::vec4> colors;
    colors.reserve(numVertices);

    // Step 1: Generate top surface vertices with colors
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int idx = (i * width + j);
            double heightValue = heightMap[idx];

            // Position of the vertex
            vertices.emplace_back(j * pixelWidth, i * pixelWidth, heightValue);

            // Color of the vertex (RGBA)
            colors.push_back(vertColor[idx]);
        }
    }

    // Step 2: Generate bottom surface vertices (z = 0) with default color (opaque black)
    int bottomOffset = vertices.size();  // Bottom vertices start after the top vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            vertices.emplace_back(j * pixelWidth, i * pixelWidth, 0.0); // Position
            colors.emplace_back(0.0f, 0.0f, 0.0f, 1.0f);                // Default color: black and fully opaque
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

    // Step 4: Generate triangles for the sides (left, right, top, bottom)
    // Left edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width;
        int tBottom = (i + 1) * width;
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        triangles.emplace_back(tTop, tBottom, bBottom);  // Left side triangles
        triangles.emplace_back(tTop, bBottom, bTop);
    }

    // Right edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width + (width - 1);
        int tBottom = (i + 1) * width + (width - 1);
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        triangles.emplace_back(tTop, bBottom, tBottom);  // Right side triangles
        triangles.emplace_back(tTop, bTop, bBottom);
    }

    // Top edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = j;
        int tRight = j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        triangles.emplace_back(bLeft, bRight, tRight);  // Top side triangles
        triangles.emplace_back(tLeft, bLeft, tRight);
    }

    // Bottom edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = (height - 1) * width + j;
        int tRight = (height - 1) * width + j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        triangles.emplace_back(tLeft, tRight, bRight);  // Bottom side triangles
        triangles.emplace_back(tLeft, bRight, bLeft);
    }

    // Step 5: Construct and validate the manifold with vertex colors
    manifold::Mesh mesh;
    mesh.vertPos = vertices;
    mesh.triVerts = triangles;

    manifold::Manifold solid = manifold::Manifold(mesh);
    manifold::Manifold::Error status = solid.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    ExportOptions opts;
    Material mat;
    mat.color = {0.0, 0.7, 0.7, 1};
    mat.metalness = 0.3;
    mat.vertColor = colors;
    opts.mat = mat;
    ExportMesh("test.gltf", solid.GetMesh(), opts);

    return solid;
}


manifold::Manifold CreateSurface(const std::vector<std::array<double, 4>>& heightMap, int width, int height, double pixelWidth = 1.0) {
    // Create the MeshGL structure
    manifold::MeshGL meshGL;

    // We have 6 properties: x, y, z for position and r, g, b for color
    meshGL.numProp = 6;

    // Step 1: Generate top surface vertices and colors
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            const std::array<double, 4>& data = heightMap[i * width + j];
            double heightValue = data[0]; // Height
            float r = static_cast<float>(data[1]); // Red
            float g = static_cast<float>(data[2]); // Green
            float b = static_cast<float>(data[3]); // Blue

            // Add vertex properties (x, y, z, r, g, b)
            meshGL.vertProperties.push_back(j * pixelWidth); // x
            meshGL.vertProperties.push_back(i * pixelWidth); // y
            meshGL.vertProperties.push_back(heightValue);    // z
            meshGL.vertProperties.push_back(r);              // r
            meshGL.vertProperties.push_back(g);              // g
            meshGL.vertProperties.push_back(b);              // b
        }
    }

    // Step 2: Generate bottom surface vertices (z = 0)
    int bottomOffset = width * height;  // Bottom vertices start after the top vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            // Add bottom vertex properties (x, y, z=0, r=0, g=0, b=0)
            meshGL.vertProperties.push_back(j * pixelWidth); // x
            meshGL.vertProperties.push_back(i * pixelWidth); // y
            meshGL.vertProperties.push_back(0.0);            // z
            meshGL.vertProperties.push_back(0.0);            // r
            meshGL.vertProperties.push_back(0.0);            // g
            meshGL.vertProperties.push_back(0.0);            // b
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

    // Step 5: Create and validate the manifold
    manifold::Manifold solid = manifold::Manifold(meshGL);
    manifold::Manifold::Error status = solid.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    return solid;
}

manifold::Manifold CreateSurface(const double* heightMap, int width, int height, double pixelWidth = 1.0) {
    // Calculate the number of vertices and triangles in advance
    int numVertices = 2 * width * height; // Top and bottom vertices
    int numTriangles = 2 * (width - 1) * (height - 1) * 2 + 2 * (width - 1 + height - 1) * 2; // Top, bottom, and sides

    // Preallocate vectors
    std::vector<glm::vec3> vertices;
    vertices.reserve(numVertices);
    std::vector<glm::ivec3> triangles;
    triangles.reserve(numTriangles);

    // Step 1: Generate top surface vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            double heightValue = heightMap[i * width + j];
            vertices.emplace_back(j * pixelWidth, i * pixelWidth, heightValue); // Top vertex scaled by pixelWidth
        }
    }

    // Step 2: Generate bottom surface vertices (z = 0)
    int bottomOffset = vertices.size();  // Bottom vertices start after the top vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            vertices.emplace_back(j * pixelWidth, i * pixelWidth, 0.0); // Bottom vertex scaled by pixelWidth
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

    // Step 4: Generate triangles for the sides (left, right, top, bottom)
    // Left edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width;
        int tBottom = (i + 1) * width;
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        triangles.emplace_back(tTop, tBottom, bBottom);  // Left side triangles
        triangles.emplace_back(tTop, bBottom, bTop);
    }

    // Right edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width + (width - 1);
        int tBottom = (i + 1) * width + (width - 1);
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        triangles.emplace_back(tTop, bBottom, tBottom);  // Right side triangles
        triangles.emplace_back(tTop, bTop, bBottom);
    }

    // Top edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = j;
        int tRight = j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        triangles.emplace_back(bLeft, bRight, tRight);  // Top side triangles
        triangles.emplace_back(tLeft, bLeft, tRight);
    }

    // Bottom edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = (height - 1) * width + j;
        int tRight = (height - 1) * width + j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        triangles.emplace_back(tLeft, tRight, bRight);  // Bottom side triangles
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

// Updated original function that takes the texture file path
manifold::Manifold CreateSurface(const std::string& texturePath, float pixelWidth = 1.0) {
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

// manifold::Manifold readPlyFile(const std::string &filepath, double cell_size, double z_offset, double scale_factor) {
//     // Create a reader for the PLY file
//     happly::PLYData plyIn(filepath);
//
//     // Get vertex positions and color channels
//     std::vector<float> vX = plyIn.getElement("vertex").getProperty<float>("x");
//     std::vector<float> vY = plyIn.getElement("vertex").getProperty<float>("y");
//     std::vector<float> vZ = plyIn.getElement("vertex").getProperty<float>("z");
//     std::vector<uint8_t> vR = plyIn.getElement("vertex").getProperty<uint8_t>("red");
//     std::vector<uint8_t> vG = plyIn.getElement("vertex").getProperty<uint8_t>("green");
//     std::vector<uint8_t> vB = plyIn.getElement("vertex").getProperty<uint8_t>("blue");
//     std::vector<uint8_t> vA;  // Alpha channel
//
//     // Check if an alpha channel exists
//     if (plyIn.getElement("vertex").hasProperty("alpha")) {
//         vA = plyIn.getElement("vertex").getProperty<uint8_t>("alpha");
//     } else {
//         // Default alpha channel to fully opaque if not present in the PLY file
//         vA.resize(vX.size(), 255);  // Full opacity
//     }
//
//     // Find min and max values for x, y, and z
//     float min_x = *std::min_element(vX.begin(), vX.end());
//     float max_x = *std::max_element(vX.begin(), vX.end());
//     float min_y = *std::min_element(vY.begin(), vY.end());
//     float max_y = *std::max_element(vY.begin(), vY.end());
//     float min_z = *std::min_element(vZ.begin(), vZ.end());
//
//     // Calculate grid resolution based on spans and cell size
//     double x_span = (max_x - min_x) * scale_factor;
//     double y_span = (max_y - min_y) * scale_factor;
//     int grid_resolution_x = std::max(1, static_cast<int>(x_span / cell_size));
//     int grid_resolution_y = std::max(1, static_cast<int>(y_span / cell_size));
//
//     // Initialize heightMap to store height and RGBA color data
//     std::vector<std::array<double, 5>> heightMap(grid_resolution_x * grid_resolution_y, {10.0, 0.0, 0.0, 0.0, 1.0});
//
//     // Accumulation arrays for height and color channels
//     std::vector<std::vector<double>> z_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<double>> r_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<double>> g_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<double>> b_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<double>> a_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<int>> point_count(grid_resolution_x, std::vector<int>(grid_resolution_y, 0));
//
//     // Process each point
//     for (size_t i = 0; i < vX.size(); ++i) {
//         double x = vX[i];
//         double y = vY[i];
//         double z = vZ[i];
//         double r = static_cast<double>(vR[i]) / 255.0;
//         double g = static_cast<double>(vG[i]) / 255.0;
//         double b = static_cast<double>(vB[i]) / 255.0;
//         double a = static_cast<double>(vA[i]) / 255.0;
//
//         // Find the corresponding grid cell indices
//         int grid_x = static_cast<int>(((x - min_x) * scale_factor) / cell_size);
//         int grid_y = static_cast<int>(((y - min_y) * scale_factor) / cell_size);
//
//         // Ensure the point falls within the grid bounds
//         if (grid_x >= 0 && grid_x < grid_resolution_x && grid_y >= 0 && grid_y < grid_resolution_y) {
//             // Accumulate the z-value and RGBA color components in the corresponding grid cell
//             z_sum[grid_x][grid_y] += (z - min_z) * scale_factor;
//             r_sum[grid_x][grid_y] += r;
//             g_sum[grid_x][grid_y] += g;
//             b_sum[grid_x][grid_y] += b;
//             a_sum[grid_x][grid_y] += a;
//             point_count[grid_x][grid_y] += 1;
//         }
//     }
//
//     // Compute the average height and RGBA color for each grid cell
//     for (int i = 0; i < grid_resolution_x; ++i) {
//         for (int j = 0; j < grid_resolution_y; ++j) {
//             if (point_count[i][j] > 0) {
//                 // Calculate the average height, apply offset, and compute average color
//                 double avg_z = (z_sum[i][j] / point_count[i][j]) + z_offset;
//                 double avg_r = r_sum[i][j] / point_count[i][j];
//                 double avg_g = g_sum[i][j] / point_count[i][j];
//                 double avg_b = b_sum[i][j] / point_count[i][j];
//                 double avg_a = a_sum[i][j] / point_count[i][j];
//
//                 // Store the results in heightMap in row-major order
//                 heightMap[j * grid_resolution_x + i] = {avg_z, avg_r, avg_g, avg_b, avg_a};
//             }
//         }
//     }
//
//     // Pass heightMap to CreateSurface function with color and alpha channel support
//     return CreateSurface(heightMap, grid_resolution_x, grid_resolution_y, cell_size);
// }

manifold::Manifold readPlyFile(const std::string &filepath, double cell_size, double z_offset, double scale_factor) {
    // Create a reader for the PLY file
    happly::PLYData plyIn(filepath);

    // Get vertex positions
    std::vector<float> vX = plyIn.getElement("vertex").getProperty<float>("x");
    std::vector<float> vY = plyIn.getElement("vertex").getProperty<float>("y");
    std::vector<float> vZ = plyIn.getElement("vertex").getProperty<float>("z");

    // Get color channels if available
    std::vector<uint8_t> vR = plyIn.getElement("vertex").getProperty<uint8_t>("red");
    std::vector<uint8_t> vG = plyIn.getElement("vertex").getProperty<uint8_t>("green");
    std::vector<uint8_t> vB = plyIn.getElement("vertex").getProperty<uint8_t>("blue");

    // Find min and max values for x, y, and z to define the grid boundaries
    float min_x = *std::min_element(vX.begin(), vX.end());
    float max_x = *std::max_element(vX.begin(), vX.end());
    float min_y = *std::min_element(vY.begin(), vY.end());
    float max_y = *std::max_element(vY.begin(), vY.end());
    float min_z = *std::min_element(vZ.begin(), vZ.end());

    // Calculate the spans for x and y
    double x_span = (max_x - min_x) * scale_factor;
    double y_span = (max_y - min_y) * scale_factor;

    int grid_resolution_x = static_cast<int>(x_span / cell_size);
    int grid_resolution_y = static_cast<int>(y_span / cell_size);

    // Ensure at least one cell is created in both directions
    grid_resolution_x = std::max(1, grid_resolution_x);
    grid_resolution_y = std::max(1, grid_resolution_y);

    // Initialize arrays to store height and color data
    std::vector<double> heightMap(grid_resolution_x * grid_resolution_y, 10.0);  // Default height value
    std::vector<glm::vec4> vertColor(grid_resolution_x * grid_resolution_y, {0.0, 0.0, 0.0, 0.0}); // RGBA colors

    // Initialize structures for z-value sums, color sums, and point counts
    std::vector<std::vector<double>> z_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
    std::vector<std::vector<double>> r_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
    std::vector<std::vector<double>> g_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
    std::vector<std::vector<double>> b_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
    std::vector<std::vector<int>> point_count(grid_resolution_x, std::vector<int>(grid_resolution_y, 0));

    // Process each point in the PLY file
    for (size_t i = 0; i < vX.size(); ++i) {
        double x = vX[i];
        double y = vY[i];
        double z = vZ[i];
        double r = static_cast<double>(vR[i]) / 255.0;
        double g = static_cast<double>(vG[i]) / 255.0;
        double b = static_cast<double>(vB[i]) / 255.0;

        // Find the corresponding grid cell indices
        int grid_x = static_cast<int>(((x - min_x) * scale_factor) / cell_size);
        int grid_y = static_cast<int>(((y - min_y) * scale_factor) / cell_size);

        // Ensure the point falls within the grid bounds
        if (grid_x >= 0 && grid_x < grid_resolution_x && grid_y >= 0 && grid_y < grid_resolution_y) {
            // Accumulate the z and color values in the corresponding grid cell
            z_sum[grid_x][grid_y] += (z - min_z) * scale_factor;
            r_sum[grid_x][grid_y] += r;
            g_sum[grid_x][grid_y] += g;
            b_sum[grid_x][grid_y] += b;
            point_count[grid_x][grid_y] += 1;
        }
    }

    // Compute the average height and color for each grid cell and store RGB for image output
    //std::vector<uint8_t> image_data(grid_resolution_x * grid_resolution_y * 3, 0);  // RGB image data

    for (int i = 0; i < grid_resolution_x; ++i) {
        for (int j = 0; j < grid_resolution_y; ++j) {
            int idx = j * grid_resolution_x + i;
            if (point_count[i][j] > 0) {
                // Calculate the average height, apply offset, and store in heightMap
                heightMap[idx] = (z_sum[i][j] / point_count[i][j]) + z_offset;

                // Calculate the average color and store in vertColor
                vertColor[idx] = {
                    r_sum[i][j] / point_count[i][j],
                    g_sum[i][j] / point_count[i][j],
                    b_sum[i][j] / point_count[i][j],
                    1.0
                };
                //vertColor[idx * 4 + 0] = r_sum[i][j] / point_count[i][j]; // Red
                //vertColor[idx * 4 + 1] = g_sum[i][j] / point_count[i][j]; // Green
                //vertColor[idx * 4 + 2] = b_sum[i][j] / point_count[i][j]; // Blue
                //vertColor[idx * 4 + 3] = 1.0;                             // Alpha (fully opaque)

                // Convert colors to 0-255 range for image data
                // image_data[idx * 3 + 0] = static_cast<uint8_t>(vertColor[idx * 4 + 0] * 255); // Red
                // image_data[idx * 3 + 1] = static_cast<uint8_t>(vertColor[idx * 4 + 1] * 255); // Green
                // image_data[idx * 3 + 2] = static_cast<uint8_t>(vertColor[idx * 4 + 2] * 255); // Blue
            }
        }
    }

    // Save the RGB image as a PNG
    //stbi_write_png("output_image.png", grid_resolution_x, grid_resolution_y, 3, image_data.data(), grid_resolution_x * 3);

    // Call CreateSurface with heightMap and vertColor
    return CreateSurface(heightMap.data(), vertColor, grid_resolution_x, grid_resolution_y, cell_size);
}

// Updated readPlyFile function
// manifold::Manifold readPlyFile(const std::string &filepath, double cell_size, double z_offset, double scale_factor) {
//     // Create a reader for the PLY file
//     happly::PLYData plyIn(filepath);
//
//     // Get vertex positions
//     std::vector<float> vX = plyIn.getElement("vertex").getProperty<float>("x");
//     std::vector<float> vY = plyIn.getElement("vertex").getProperty<float>("y");
//     std::vector<float> vZ = plyIn.getElement("vertex").getProperty<float>("z");
//
//     // Get color channels if available
//     std::vector<uint8_t> vR = plyIn.getElement("vertex").getProperty<uint8_t>("red");
//     std::vector<uint8_t> vG = plyIn.getElement("vertex").getProperty<uint8_t>("green");
//     std::vector<uint8_t> vB = plyIn.getElement("vertex").getProperty<uint8_t>("blue");
//
//     // Find min and max values for x, y, and z to define the grid boundaries
//     float min_x = *std::min_element(vX.begin(), vX.end());
//     float max_x = *std::max_element(vX.begin(), vX.end());
//     float min_y = *std::min_element(vY.begin(), vY.end());
//     float max_y = *std::max_element(vY.begin(), vY.end());
//     float min_z = *std::min_element(vZ.begin(), vZ.end());
//
//     // Calculate the spans for x and y
//     double x_span = (max_x - min_x) * scale_factor;
//     double y_span = (max_y - min_y) * scale_factor;
//
//     int grid_resolution_x = static_cast<int>(x_span / cell_size);
//     int grid_resolution_y = static_cast<int>(y_span / cell_size);
//
//     // Ensure at least one cell is created in both directions
//     grid_resolution_x = std::max(1, grid_resolution_x);
//     grid_resolution_y = std::max(1, grid_resolution_y);
//
//     // Initialize arrays to store height and color data
//     std::vector<double> heightMap(grid_resolution_x * grid_resolution_y, 10.0);  // Default height value
//     std::vector<double> vertColor(grid_resolution_x * grid_resolution_y * 4, 0.0); // RGBA colors
//
//     // Initialize structures for z-value sums, color sums, and point counts
//     std::vector<std::vector<double>> z_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<double>> r_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<double>> g_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<double>> b_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<int>> point_count(grid_resolution_x, std::vector<int>(grid_resolution_y, 0));
//
//     // Process each point in the PLY file
//     for (size_t i = 0; i < vX.size(); ++i) {
//         double x = vX[i];
//         double y = vY[i];
//         double z = vZ[i];
//         double r = static_cast<double>(vR[i]) / 255.0;
//         double g = static_cast<double>(vG[i]) / 255.0;
//         double b = static_cast<double>(vB[i]) / 255.0;
//
//         // Find the corresponding grid cell indices
//         int grid_x = static_cast<int>(((x - min_x) * scale_factor) / cell_size);
//         int grid_y = static_cast<int>(((y - min_y) * scale_factor) / cell_size);
//
//         if (i < 200) {
//             cout << r << " " << g << " " << b << endl;
//         }
//
//         // Ensure the point falls within the grid bounds
//         if (grid_x >= 0 && grid_x < grid_resolution_x && grid_y >= 0 && grid_y < grid_resolution_y) {
//             // Accumulate the z and color values in the corresponding grid cell
//             z_sum[grid_x][grid_y] += (z - min_z) * scale_factor;
//             r_sum[grid_x][grid_y] += r;
//             g_sum[grid_x][grid_y] += g;
//             b_sum[grid_x][grid_y] += b;
//             point_count[grid_x][grid_y] += 1;
//         }
//     }
//
//     // Compute the average height and color for each grid cell
//     for (int i = 0; i < grid_resolution_x; ++i) {
//         for (int j = 0; j < grid_resolution_y; ++j) {
//             int idx = j * grid_resolution_x + i;
//             if (point_count[i][j] > 0) {
//                 // Calculate the average height, apply offset, and store in heightMap
//                 heightMap[idx] = (z_sum[i][j] / point_count[i][j]) + z_offset;
//
//                 // Calculate the average color and store in vertColor
//                 vertColor[idx * 4 + 0] = r_sum[i][j] / point_count[i][j]; // Red
//                 vertColor[idx * 4 + 1] = g_sum[i][j] / point_count[i][j]; // Green
//                 vertColor[idx * 4 + 2] = b_sum[i][j] / point_count[i][j]; // Blue
//                 vertColor[idx * 4 + 3] = 1.0;                             // Alpha (fully opaque)
//             }
//         }
//     }
//
//     // Call CreateSurface with heightMap and vertColor
//     Manifold man = CreateSurface(heightMap.data(), vertColor.data(), grid_resolution_x, grid_resolution_y, cell_size);
//
//     std::vector<glm::vec4> glmVertColor;
//     glmVertColor.reserve(grid_resolution_x * grid_resolution_y);
//
//     for (size_t i = 0; i < vertColor.size(); i += 4) {
//         glmVertColor.emplace_back(
//             static_cast<float>(vertColor[i]),       // R
//             static_cast<float>(vertColor[i + 1]),   // G
//             static_cast<float>(vertColor[i + 2]),   // B
//             static_cast<float>(vertColor[i + 3])    // A
//         );
//     }
//
//     return man;
// }

//manifold::Manifold readPlyFile(const std::string &filepath, double cell_size, double z_offset, double scale_factor) {
//    // Create a reader for the PLY file
//    happly::PLYData plyIn(filepath);
//
//    // Get vertex positions
//    std::vector<float> vX = plyIn.getElement("vertex").getProperty<float>("x");
//    std::vector<float> vY = plyIn.getElement("vertex").getProperty<float>("y");
//    std::vector<float> vZ = plyIn.getElement("vertex").getProperty<float>("z");
//
//    // Get color channels if available
//    std::vector<uint8_t> vR = plyIn.getElement("vertex").getProperty<uint8_t>("red");
//    std::vector<uint8_t> vG = plyIn.getElement("vertex").getProperty<uint8_t>("green");
//    std::vector<uint8_t> vB = plyIn.getElement("vertex").getProperty<uint8_t>("blue");
//
//    // Find min and max values for x, y, and z to define the grid boundaries
//    float min_x = *std::min_element(vX.begin(), vX.end());
//    float max_x = *std::max_element(vX.begin(), vX.end());
//    float min_y = *std::min_element(vY.begin(), vY.end());
//    float max_y = *std::max_element(vY.begin(), vY.end());
//    float min_z = *std::min_element(vZ.begin(), vZ.end());
//
//    // Calculate the spans for x and y
//    double x_span = (max_x - min_x) * scale_factor;
//    double y_span = (max_y - min_y) * scale_factor;
//
//    int grid_resolution_x = static_cast<int>(x_span / cell_size);
//    int grid_resolution_y = static_cast<int>(y_span / cell_size);
//
//    // Ensure at least one cell is created in both directions
//    grid_resolution_x = std::max(1, grid_resolution_x);
//    grid_resolution_y = std::max(1, grid_resolution_y);
//
//    // Initialize grid to store height and color data
//    std::vector<std::array<double, 4>> heightMap(grid_resolution_x * grid_resolution_y, {10.0, 0.0, 0.0, 0.0});
//
//    // Initialize structures for z-value sums, color sums, and point counts
//    std::vector<std::vector<double>> z_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//    std::vector<std::vector<double>> r_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//    std::vector<std::vector<double>> g_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//    std::vector<std::vector<double>> b_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//    std::vector<std::vector<int>> point_count(grid_resolution_x, std::vector<int>(grid_resolution_y, 0));
//
//    // Process each point in the PLY file
//    for (size_t i = 0; i < vX.size(); ++i) {
//        double x = vX[i];
//        double y = vY[i];
//        double z = vZ[i];
//        double r = static_cast<double>(vR[i]) / 255.0;
//        double g = static_cast<double>(vG[i]) / 255.0;
//        double b = static_cast<double>(vB[i]) / 255.0;
//
//        // Find the corresponding grid cell indices
//        int grid_x = static_cast<int>(((x - min_x) * scale_factor) / cell_size);
//        int grid_y = static_cast<int>(((y - min_y) * scale_factor) / cell_size);
//
//        // Ensure the point falls within the grid bounds
//        if (grid_x >= 0 && grid_x < grid_resolution_x && grid_y >= 0 && grid_y < grid_resolution_y) {
//            // Accumulate the z and color values in the corresponding grid cell
//            z_sum[grid_x][grid_y] += (z - min_z) * scale_factor;
//            r_sum[grid_x][grid_y] += r;
//            g_sum[grid_x][grid_y] += g;
//            b_sum[grid_x][grid_y] += b;
//            point_count[grid_x][grid_y] += 1;
//        }
//    }
//
//    // Compute the average height and color for each grid cell
//    for (int i = 0; i < grid_resolution_x; ++i) {
//        for (int j = 0; j < grid_resolution_y; ++j) {
//            if (point_count[i][j] > 0) {
//                // Calculate the average height, apply offset, and set default color
//                double avg_z = (z_sum[i][j] / point_count[i][j]) + z_offset;
//                double avg_r = r_sum[i][j] / point_count[i][j];
//                double avg_g = g_sum[i][j] / point_count[i][j];
//                double avg_b = b_sum[i][j] / point_count[i][j];
//
//                // Store the results in heightMap in row-major order
//                heightMap[j * grid_resolution_x + i] = {avg_z, avg_r, avg_g, avg_b};
//            }
//        }
//    }
//
//    // Pass heightMap to CreateSurface function with color support
//    return CreateSurface(heightMap, grid_resolution_x, grid_resolution_y, cell_size);
//}

manifold::Manifold createRedBlueTetrahedrons() {
    // Define the vertices for two tetrahedrons
    std::vector<float> vertProperties = {
        // Red Tetrahedron vertices (Position and Color RGBA)
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 0 (Red)
        1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 1 (Red)
        0.5f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 2 (Red)
        0.5f, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 3 (Red)

        // Blue Tetrahedron vertices (Position and Color RGBA)
        2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,   // Vertex 4 (Blue)
        3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,   // Vertex 5 (Blue)
        2.5f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,   // Vertex 6 (Blue)
        2.5f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f    // Vertex 7 (Blue)
    };

    // Define the indices for the triangles of each tetrahedron
    std::vector<uint32_t> triVerts = {
        // Red Tetrahedron faces
        0, 1, 2,   // Face 1
        0, 1, 3,   // Face 2
        1, 2, 3,   // Face 3
        2, 0, 3,   // Face 4

        // Blue Tetrahedron faces
        4, 5, 6,   // Face 1
        4, 5, 7,   // Face 2
        5, 6, 7,   // Face 3
        6, 4, 7    // Face 4
    };

    // Configure the MeshGL object
    MeshGL meshGL;
    meshGL.numProp = 7;                  // 3 for position (x, y, z) + 4 for color (r, g, b, a)
    meshGL.vertProperties = vertProperties;
    meshGL.triVerts = triVerts;

    return manifold::Manifold(meshGL);
}


manifold::Manifold createRedBlueCube() {
    // Define the vertices for a cube with color (red and blue) assigned per vertex
    std::vector<float> vertProperties = {
        // Vertex positions (x, y, z) and colors (r, g, b, a)
        -1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 0 (Red)
         1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 1 (Red)
         1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f,   // Vertex 2 (Blue)
        -1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f,   // Vertex 3 (Blue)
        -1.0f, -1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 4 (Red)
         1.0f, -1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 1.0f,   // Vertex 5 (Red)
         1.0f,  1.0f,  1.0f, 0.0f, 0.0f, 1.0f, 1.0f,   // Vertex 6 (Blue)
        -1.0f,  1.0f,  1.0f, 0.0f, 0.0f, 1.0f, 1.0f    // Vertex 7 (Blue)
    };

    // Define the indices for each face of the cube (2 triangles per face)
    std::vector<uint32_t> triVerts = {
        0, 1, 2, 0, 2, 3,   // Front face
        1, 5, 6, 1, 6, 2,   // Right face
        5, 4, 7, 5, 7, 6,   // Back face
        4, 0, 3, 4, 3, 7,   // Left face
        3, 2, 6, 3, 6, 7,   // Top face
        4, 5, 1, 4, 1, 0    // Bottom face
    };

    // Configure the MeshGL object
    MeshGL meshGL;
    meshGL.numProp = 7;                  // 3 for position (x, y, z) + 4 for color (r, g, b, a)
    meshGL.vertProperties = vertProperties;
    meshGL.triVerts = triVerts;

    manifold::Manifold man = manifold::Manifold(meshGL);

    manifold::Manifold::Error status = man.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    return man;
}

// manifold::Manifold readPlyFile(const std::string &filepath, double cell_size, double z_offset, double scale_factor) {
//     // Create a reader for the PLY file
//     happly::PLYData plyIn(filepath);
//
//     // Get vertex positions
//     std::vector<float> vX = plyIn.getElement("vertex").getProperty<float>("x");
//     std::vector<float> vY = plyIn.getElement("vertex").getProperty<float>("y");
//     std::vector<float> vZ = plyIn.getElement("vertex").getProperty<float>("z");
//
//     // Find min and max values for x, y, and z to define the grid boundaries
//     float min_x = *std::min_element(vX.begin(), vX.end());
//     float max_x = *std::max_element(vX.begin(), vX.end());
//     float min_y = *std::min_element(vY.begin(), vY.end());
//     float max_y = *std::max_element(vY.begin(), vY.end());
//     float min_z = *std::min_element(vZ.begin(), vZ.end());
//
//     // Calculate the spans for x and y
//     double x_span = (max_x - min_x) * scale_factor;
//     double y_span = (max_y - min_y) * scale_factor;
//
//     int grid_resolution_x = static_cast<int>(x_span / cell_size);
//     int grid_resolution_y = static_cast<int>(y_span / cell_size);
//
//     // Ensure at least one cell is created in both directions
//     grid_resolution_x = std::max(1, grid_resolution_x);
//     grid_resolution_y = std::max(1, grid_resolution_y);
//
//     // Initialize flat heightmap (1D array in row-major order)
//     std::vector<double> heightmap(grid_resolution_x * grid_resolution_y, 10.0);  // Default value of 10
//
//     // Initialize structures for z-value sums and point counts
//     std::vector<std::vector<double>> z_sum(grid_resolution_x, std::vector<double>(grid_resolution_y, 0.0));
//     std::vector<std::vector<int>> point_count(grid_resolution_x, std::vector<int>(grid_resolution_y, 0));
//
//     // Process each point in the PLY file
//     for (size_t i = 0; i < vX.size(); ++i) {
//         double x = vX[i];
//         double y = vY[i];
//         double z = vZ[i];
//
//         // Find the corresponding grid cell indices
//         int grid_x = static_cast<int>(((x - min_x) * scale_factor) / cell_size);
//         int grid_y = static_cast<int>(((y - min_y) * scale_factor) / cell_size);
//
//         // Ensure the point falls within the grid bounds
//         if (grid_x >= 0 && grid_x < grid_resolution_x && grid_y >= 0 && grid_y < grid_resolution_y) {
//             // Accumulate the z-value in the corresponding grid cell
//             z_sum[grid_x][grid_y] += (z - min_z) * scale_factor;
//             point_count[grid_x][grid_y] += 1;
//         }
//     }
//
//     // Compute the average z-value for each grid cell, subtract min_z, and add the constant offset
//     for (int i = 0; i < grid_resolution_x; ++i) {
//         for (int j = 0; j < grid_resolution_y; ++j) {
//             if (point_count[i][j] > 0) {
//                 // Compute the average z-value and add the offset
//                 double avg_z = (z_sum[i][j] / point_count[i][j]) + z_offset;
//
//                 // Map to the flat array in row-major order
//                 heightmap[j * grid_resolution_x + i] = avg_z;
//             }
//         }
//     }
//
//     // Cast the heightmap to a double* for manifold creation
//     double* heightmap_ptr = heightmap.data();
//
//     // Return the created surface using the manifold library
//     return CreateSurface(heightmap_ptr, grid_resolution_x, grid_resolution_y, cell_size);
// }

}
