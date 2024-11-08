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

    cout << "width: " << width << " height: " << height << endl;
    cout << "VERT SIZE: " << meshGL.vertProperties.size() << " Expected: " << numVerts << endl;
    cout << "TRI SIZE: " << meshGL.triVerts.size() << " Excpeted: " << numTriangles << endl;

    // Create and validate the manifold
    manifold::Manifold solid = manifold::Manifold(meshGL);
    manifold::Manifold::Error status = solid.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    return solid;
}


manifold::Manifold PlyToSurface(const std::string &filepath, double cell_size, double z_offset, double scale_factor) {
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
    double x_span = (max_x - min_x) * scale_factor;
    double y_span = (max_y - min_y) * scale_factor;

    int grid_resolution_x = static_cast<int>(x_span / cell_size);
    int grid_resolution_y = static_cast<int>(y_span / cell_size);

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
    int nProp = 4;

    // Initialize vertProperties to store the flattened vertex data
    std::vector<float> vertProperties;
    vertProperties.reserve(grid_resolution_x * grid_resolution_y * nProp);  // Reserve space for z, r, g, b per cell

    // Compute the average height and color for each grid cell and populate vertProperties
    for (int i = 0; i < grid_resolution_x; ++i) {
        for (int j = 0; j < grid_resolution_y; ++j) {
            if (point_count[i][j] > 0) {
                // Calculate the average height, apply offset, and set default color
                float avg_z = (z_sum[i][j] / point_count[i][j]) + z_offset;
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
    return CreateSurface(vertProperties.data(), nProp, grid_resolution_x, grid_resolution_y, cell_size);
}



}
