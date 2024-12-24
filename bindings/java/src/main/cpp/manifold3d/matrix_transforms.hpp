#pragma once

#include <linalg.h>    // https://github.com/sgorsten/linalg
#include <cmath>       // for std::sin, std::cos

namespace MatrixTransforms {

using namespace linalg::aliases;

// ------------------------------------------------------------
// Type aliases for convenience (double-precision):
using double2   = linalg::vec<double, 2>;
using double3   = linalg::vec<double, 3>;
using double2x2 = linalg::mat<double, 2, 2>;
using double3x3 = linalg::mat<double, 3, 3>;

// 4x3 matrix in row-major form => 3 rows, 4 columns
// We'll treat the "columns" as 3-element vectors (like glm::mat4x3).
using double4x3 = linalg::mat<double, 3, 4>;

// 3x2 matrix in row-major form => 2 rows, 3 columns
// We'll treat the "columns" as 2-element vectors (like glm::mat3x2).
using double3x2 = linalg::mat<double, 2, 3>;

// ------------------------------------------------------------
// Helper functions to mimic column-based access like GLM.
// linalg::mat<T, R, C> has operator()(row, col) for element access.
template<typename T, int R, int C>
linalg::vec<T,R> get_col(const linalg::mat<T,R,C> &M, int col_index)
{
    linalg::vec<T,R> column{};
    for (int r = 0; r < R; ++r) {
        column[r] = M(r, col_index);
    }
    return column;
}

template<typename T, int R, int C>
void set_col(linalg::mat<T,R,C> &M, int col_index, const linalg::vec<T,R> &col_vec)
{
    for (int r = 0; r < R; ++r) {
        M(r, col_index) = col_vec[r];
    }
}

// ------------------------------------------------------------
// Rodrigues Rotation: rotate vector v around (unit) axis k by angle a.
inline double3 rodrigues_rotation(const double3 &v, const double3 &k, double a)
{
    using std::sin;
    using std::cos;

    double c = cos(a);
    double s = sin(a);

    // v*cos(a) + (k x v)*sin(a) + k*(k·v)*(1 - cos(a))
    return v * c
         + linalg::cross(k, v) * s
         + k * (linalg::dot(k, v) * (1.0 - c));
}

// ------------------------------------------------------------
// Yaw: rotate columns [0], [2] about column [1].
inline double4x3 Yaw(const double4x3 &m, double a)
{
    auto c0 = get_col(m, 0);
    auto c1 = get_col(m, 1);
    auto c2 = get_col(m, 2);
    auto c3 = get_col(m, 3);

    auto d0 = rodrigues_rotation(c0, c1, a);
    auto d1 = c1;  // unchanged
    auto d2 = rodrigues_rotation(c2, c1, a);

    double4x3 result = m;
    set_col(result, 0, d0);
    set_col(result, 1, d1);
    set_col(result, 2, d2);
    set_col(result, 3, c3);

    return result;
}

// Pitch: rotate columns [1], [2] about column [0].
inline double4x3 Pitch(const double4x3 &m, double a)
{
    auto c0 = get_col(m, 0);
    auto c1 = get_col(m, 1);
    auto c2 = get_col(m, 2);
    auto c3 = get_col(m, 3);

    auto d0 = c0;  // unchanged
    auto d1 = rodrigues_rotation(c1, c0, a);
    auto d2 = rodrigues_rotation(c2, c0, a);

    double4x3 result = m;
    set_col(result, 0, d0);
    set_col(result, 1, d1);
    set_col(result, 2, d2);
    set_col(result, 3, c3);

    return result;
}

// Roll: rotate columns [0], [1] about column [2].
inline double4x3 Roll(const double4x3 &m, double a)
{
    auto c0 = get_col(m, 0);
    auto c1 = get_col(m, 1);
    auto c2 = get_col(m, 2);
    auto c3 = get_col(m, 3);

    auto d0 = rodrigues_rotation(c0, c2, a);
    auto d1 = rodrigues_rotation(c1, c2, a);
    auto d2 = c2; // unchanged

    double4x3 result = m;
    set_col(result, 0, d0);
    set_col(result, 1, d1);
    set_col(result, 2, d2);
    set_col(result, 3, c3);

    return result;
}

// Rotate: rotate columns [0..2] around an arbitrary axis.
inline double4x3 Rotate(const double4x3 &m, const double3 &axis, double a)
{
    auto c0 = get_col(m, 0);
    auto c1 = get_col(m, 1);
    auto c2 = get_col(m, 2);
    auto c3 = get_col(m, 3);

    double4x3 result = m;
    set_col(result, 0, rodrigues_rotation(c0, axis, a));
    set_col(result, 1, rodrigues_rotation(c1, axis, a));
    set_col(result, 2, rodrigues_rotation(c2, axis, a));
    set_col(result, 3, c3);

    return result;
}

// ------------------------------------------------------------
// 2D rotation of a vector
inline double2 RotateVec2(const double2 &v, double angleRadians)
{
    using std::sin;
    using std::cos;

    double c = cos(angleRadians);
    double s = sin(angleRadians);

    return {
        c*v.x - s*v.y,
        s*v.x + c*v.y
    };
}

// Combined 3D rotation (pitch->yaw->roll) given angles.x/y/z
inline double4x3 Rotate(const double4x3 &m, const double3 &angles)
{
    double4x3 res = m;
    if (angles.x != 0.0) {
        res = Pitch(res, angles.x);
    }
    if (angles.y != 0.0) {
        res = Yaw(res, angles.y);
    }
    if (angles.z != 0.0) {
        res = Roll(res, angles.z);
    }
    return res;
}

// ------------------------------------------------------------
// 2D transform matrix is 2 rows × 3 columns => double3x2
// Rotate basis vectors in 2D
inline double3x2 Rotate(const double3x2 &m, double angleRadians)
{
    // columns: [0]=xAxis, [1]=yAxis, [2]=translation
    auto xAxis = get_col(m, 0);
    auto yAxis = get_col(m, 1);
    auto trans = get_col(m, 2);

    double2 xRot = RotateVec2(xAxis, angleRadians);
    double2 yRot = RotateVec2(yAxis, angleRadians);

    double3x2 result = m;
    set_col(result, 0, xRot);
    set_col(result, 1, yRot);
    set_col(result, 2, trans);

    return result;
}

// ------------------------------------------------------------
// Replace the upper-left 3x3 with a given rotation
inline double4x3 SetRotation(const double4x3 &m, const double3x3 &rotation)
{
    double4x3 result = m;
    for (int i = 0; i < 3; ++i) {
        auto col = get_col(rotation, i); // each column is double3
        set_col(result, i, col);
    }
    return result;
}

// 2D version
inline double3x2 SetRotation(const double3x2 &m, const double2x2 &rot2x2)
{
    double3x2 result = m;
    for (int i = 0; i < 2; ++i) {
        auto col = get_col(rot2x2, i); // each column is double2
        set_col(result, i, col);
    }
    return result;
}

// ------------------------------------------------------------
// Translation: T += offset.x * col0 + offset.y * col1 + offset.z * col2
inline double4x3 Translate(const double4x3 &m, const double3 &offset)
{
    double4x3 result = m;

    if (offset.x != 0.0) {
        set_col(result, 3, get_col(result, 3) + get_col(m, 0)*offset.x);
    }
    if (offset.y != 0.0) {
        set_col(result, 3, get_col(result, 3) + get_col(m, 1)*offset.y);
    }
    if (offset.z != 0.0) {
        set_col(result, 3, get_col(result, 3) + get_col(m, 2)*offset.z);
    }

    return result;
}

// 2D version
inline double3x2 Translate(const double3x2 &m, const double2 &offset)
{
    double3x2 result = m;

    if (offset.x != 0.0) {
        set_col(result, 2, get_col(result, 2) + get_col(m, 0)*offset.x);
    }
    if (offset.y != 0.0) {
        set_col(result, 2, get_col(result, 2) + get_col(m, 1)*offset.y);
    }
    return result;
}

// ------------------------------------------------------------
// Set absolute translation
inline double4x3 SetTranslation(const double4x3 &m, const double3 &translation)
{
    double4x3 result = m;
    set_col(result, 3, translation);
    return result;
}

// 2D version
inline double3x2 SetTranslation(const double3x2 &m, const double2 &translation)
{
    double3x2 result = m;
    set_col(result, 2, translation);
    return result;
}

// ------------------------------------------------------------
// Multiply two 4x3 transforms (with normalization of the first 3 columns).
inline double4x3 Transform(const double4x3 &a, const double4x3 &b)
{
    double4x3 result; // 3 rows, 4 columns

    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 3; ++row)
        {
            double sum = a(row,0)*b(0,col)
                       + a(row,1)*b(1,col)
                       + a(row,2)*b(2,col);
            if (col == 3) {
                sum += a(row,3);
            }
            result(row,col) = sum;
        }
    }

    // Normalize each of the first 3 columns
    for (int i = 0; i < 3; ++i) {
        auto c = get_col(result, i);
        auto norm_c = linalg::normalize(c);
        set_col(result, i, norm_c);
    }

    return result;
}

// Invert a 4x3 transform (rotation + translation)
inline double4x3 InvertTransform(const double4x3 &m)
{
    // Extract rotation part as a 3x3
    double3x3 rot;
    for (int i = 0; i < 3; ++i) {
        auto col = get_col(m, i);
        set_col(rot, i, col);
    }

    // Transpose to invert rotation
    auto rotT = linalg::transpose(rot);

    // Put transposed rotation back
    double4x3 unRotated = SetRotation(m, rotT);

    // The translation is column 3 => invert it
    auto trans = get_col(m, 3);
    auto invTrans = -linalg::mul(rotT, trans);

    return SetTranslation(unRotated, invTrans);
}

// ------------------------------------------------------------
// 2D versions
inline double3x2 Transform(const double3x2 &a, const double3x2 &b)
{
    double3x2 result;

    for (int col = 0; col < 3; ++col)
    {
        for (int row = 0; row < 2; ++row)
        {
            double sum = a(row,0)*b(0,col)
                       + a(row,1)*b(1,col);
            if (col == 2) {
                sum += a(row,2);
            }
            result(row,col) = sum;
        }
    }

    // Normalize first 2 columns
    for (int i = 0; i < 2; ++i) {
        auto c = get_col(result, i);
        auto norm_c = linalg::normalize(c);
        set_col(result, i, norm_c);
    }

    return result;
}

inline double3x2 InvertTransform(const double3x2 &m)
{
    // Extract 2x2 rotation
    double2x2 rot2;
    for (int i = 0; i < 2; ++i) {
        auto col = get_col(m, i);
        set_col(rot2, i, col);
    }

    // Transpose to invert
    auto rot2T = linalg::transpose(rot2);

    // Rebuild partial matrix
    double3x2 unRotated = SetRotation(m, rot2T);

    // Invert translation
    auto trans = get_col(m, 2);
    auto invTrans = -linalg::mul(rot2T, trans);

    return SetTranslation(unRotated, invTrans);
}

// ------------------------------------------------------------
// CombineTransforms variant (like Transform but different math).
inline double4x3 CombineTransforms(const double4x3 &a, const double4x3 &b)
{
    double4x3 r;

    auto a0 = get_col(a, 0);
    auto a1 = get_col(a, 1);
    auto a2 = get_col(a, 2);
    auto a3 = get_col(a, 3);

    // For each rotation column i in b, combine with a, then normalize.
    for(int i = 0; i < 3; ++i) {
        auto bi = get_col(b, i);
        double3 c = a0*bi.x + a1*bi.y + a2*bi.z;
        c = linalg::normalize(c);
        set_col(r, i, c);
    }

    // Combine translation
    auto b3 = get_col(b, 3);
    double3 c3 = a0*b3.x + a1*b3.y + a2*b3.z + a3;
    set_col(r, 3, c3);

    return r;
}

inline double3x2 CombineTransforms(const double3x2 &a, const double3x2 &b)
{
    double3x2 r;

    auto a0 = get_col(a, 0);
    auto a1 = get_col(a, 1);
    auto a2 = get_col(a, 2);

    for (int i = 0; i < 2; ++i) {
        auto bi = get_col(b, i);
        double2 c = a0*bi.x + a1*bi.y;
        c = linalg::normalize(c);
        set_col(r, i, c);
    }

    auto b2 = get_col(b, 2);
    double2 c2 = a0*b2.x + a1*b2.y + a2;
    set_col(r, 2, c2);

    return r;
}

} // namespace MatrixTransforms
