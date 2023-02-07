#pragma once
#include <stddef.h>

typedef struct ManifoldManifold ManifoldManifold;
typedef struct ManifoldSimplePolygon ManifoldSimplePolygon;
typedef struct ManifoldPolygons ManifoldPolygons;
typedef struct ManifoldMesh ManifoldMesh;
typedef struct ManifoldMeshGL ManifoldMeshGL;
typedef struct ManifoldCurvature ManifoldCurvature;
typedef struct ManifoldComponents ManifoldComponents;
typedef struct ManifoldBox ManifoldBox;
typedef struct ManifoldMaterial ManifoldMaterial;
typedef struct ManifoldExportOptions ManifoldExportOptions;

typedef struct ManifoldManifoldPair {
  ManifoldManifold* first;
  ManifoldManifold* second;
} ManifoldManifoldPair;

typedef struct ManifoldVec2 {
  float x;
  float y;
} ManifoldVec2;

typedef struct ManifoldVec3 {
  float x;
  float y;
  float z;
} ManifoldVec3;

typedef struct ManifoldIVec3 {
  int x;
  int y;
  int z;
} ManifoldIVec3;

typedef struct ManifoldVec4 {
  float x;
  float y;
  float z;
  float w;
} ManifoldVec4;

typedef struct ManifoldProperties {
  float surface_area;
  float volume;
} ManifoldProperties;

typedef struct ManifoldPolyVert {
  ManifoldVec2 pos;
  int idx;

} ManifoldPolyVert;

typedef struct ManifoldCurvatureBounds {
  float max_mean_curvature;
  float min_mean_curvature;
  float max_gaussian_curvature;
  float min_gaussian_curvature;
} ManifoldCurvatureBounds;

typedef enum ManifoldError {
  NO_ERROR,
  NON_FINITE_VERTEX,
  NOT_MANIFOLD,
  VERTEX_INDEX_OUT_OF_BOUNDS,
  PROPERTIES_WRONG_LENGTH,
  MISSING_POSITION_PROPERTIES,
  MERGE_VECTORS_DIFFERENT_LENGTHS,
  MERGE_INDEX_OUT_OF_BOUNDS,
  TRANSFORM_WRONG_LENGTH,
  RUN_INDEX_WRONG_LENGTH,
  FACE_ID_WRONG_LENGTH,
} ManifoldError;
