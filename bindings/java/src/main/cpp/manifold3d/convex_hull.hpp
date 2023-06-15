#pragma once

#include <glm/glm.hpp>

#include <convex_hull_qhull_impl.hpp>
#include <vector>
#include "manifold.h"
#include "cross_section.h"

namespace ConvexHull {

manifold::Manifold ConvexHull(const manifold::Manifold manifold, const float precision = 0.0001) {
    manifold::Mesh inputMesh = manifold.GetMesh();
    manifold::Mesh outputMesh = manifold::computeConvexHull3D(inputMesh.vertPos);;
    return manifold::Manifold(outputMesh);
}

manifold::Manifold ConvexHull(const manifold::Manifold manifold, const manifold::Manifold other, const float precision = 0.0001) {
    manifold::Mesh inputMesh1 = manifold.GetMesh();
    manifold::Mesh inputMesh2 = other.GetMesh();

    // Combine vertices from input meshes
    std::vector<glm::vec3> combinedVerts;
    for (auto& vert: inputMesh1.vertPos) {
        combinedVerts.push_back(vert);
    }

    for (auto& vert: inputMesh2.vertPos) {
        combinedVerts.push_back(vert);
    }

    manifold::Mesh outputMesh = manifold::computeConvexHull3D(combinedVerts);

    return manifold::Manifold(outputMesh);
}

manifold::CrossSection ConvexHull(const manifold::CrossSection& cross_section, const float precision = 0.0001) {
    manifold::SimplePolygon hullPoints;
    for (auto& poly: cross_section.ToPolygons()) {
        for (auto& pt: poly) {
            hullPoints.push_back(glm::vec2(pt.x, pt.y));
        }
    }
    manifold::SimplePolygon res =  manifold::computeConvexHull2D(hullPoints);
    return manifold::CrossSection(res);
}


manifold::CrossSection ConvexHull(const manifold::CrossSection& cross_section, const manifold::CrossSection& other, const float precision = 0.0001) {
    manifold::SimplePolygon hullPoints;

    for (auto& poly: cross_section.ToPolygons()) {
        for (auto& pt: poly) {
            hullPoints.push_back(glm::vec2(pt.x, pt.y));
        }
    }

    for (auto& poly: other.ToPolygons()) {
        for (auto& pt: poly) {
            hullPoints.push_back(glm::vec2(pt.x, pt.y));
        }
    }

    manifold::SimplePolygon res = manifold::computeConvexHull2D(hullPoints);
    return manifold::CrossSection(res);
}

}
