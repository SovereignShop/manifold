#pragma once

#include <manifold/linalg.h>    // https://github.com/sgorsten/linalg
#include <cmath>       // for std::sin, std::cos

namespace MatrixTransforms {

using namespace linalg::aliases;

// ------------------------------------------------------------
// Type aliases for convenience (double-precision):
using vec2   = linalg::vec<double, 2>;
using vec3   = linalg::vec<double, 3>;
using mat2x2 = linalg::mat<double, 2, 2>;
using mat3x3 = linalg::mat<double, 3, 3>;

// Here, mat4x3 is 4 rows × 3 columns (row-major):
//  - row 0..2 = rotation vectors
//  - row 3    = translation vector
using mat4x3 = linalg::mat<double, 4, 3>;

// 2D transform: 3 rows × 2 columns would be linalg::mat<double,3,2>,
// but the original code had 'mat3x2' = 2 rows × 3 columns in GLM terms.
// We'll store 3 rows, 2 columns => each row is a 2-element vector.
// If your code truly wants 2D transforms with a "translation row," then
// row 0..1 = basis vectors, row 2 = translation.
using mat3x2 = linalg::mat<double, 3, 2>;

// ------------------------------------------------------------
// Helper functions for row-based access: get_row / set_row

template<typename T, int R, int C>
linalg::vec<T,C> get_row(const linalg::mat<T,R,C> &M, int row_index)
{
    linalg::vec<T,C> row{};
    for (int c = 0; c < C; ++c) {
        row[c] = M(row_index, c);
    }
    return row;
}

template<typename T, int R, int C>
void set_row(linalg::mat<T,R,C> &M, int row_index, const linalg::vec<T,C> &row_vec)
{
    for (int c = 0; c < C; ++c) {
        M(row_index, c) = row_vec[c];
    }
}

// ------------------------------------------------------------
// Rodrigues rotation of v around axis k by angle a
inline vec3 rodrigues_rotation(const vec3 &v, const vec3 &k, double a)
{
    using std::sin;
    using std::cos;

    double c = cos(a);
    double s = sin(a);

    // v*cos(a) + (k x v)*sin(a) + k*(k·v)*(1 - cos(a))
    return v*c
         + linalg::cross(k, v)*s
         + k*(linalg::dot(k, v) * (1.0 - c));
}

// ------------------------------------------------------------
// "Yaw": Rotate rows [0], [2] about row [1] (the axis).
//  - row 1 remains unchanged
inline mat4x3 Yaw(const mat4x3 &m, double a)
{
    auto r0 = get_row(m, 0);
    auto r1 = get_row(m, 1);
    auto r2 = get_row(m, 2);
    auto r3 = get_row(m, 3);

    auto d0 = rodrigues_rotation(r0, r1, a);
    auto d1 = r1;  // unchanged
    auto d2 = rodrigues_rotation(r2, r1, a);

    mat4x3 result = m;
    set_row(result, 0, d0);
    set_row(result, 1, d1);
    set_row(result, 2, d2);
    set_row(result, 3, r3); // translation unchanged
    return result;
}

// "Pitch": Rotate rows [1], [2] about row [0].
inline mat4x3 Pitch(const mat4x3 &m, double a)
{
    auto r0 = get_row(m, 0);
    auto r1 = get_row(m, 1);
    auto r2 = get_row(m, 2);
    auto r3 = get_row(m, 3);

    auto d0 = r0;  // unchanged
    auto d1 = rodrigues_rotation(r1, r0, a);
    auto d2 = rodrigues_rotation(r2, r0, a);

    mat4x3 result = m;
    set_row(result, 0, d0);
    set_row(result, 1, d1);
    set_row(result, 2, d2);
    set_row(result, 3, r3);
    return result;
}

// "Roll": Rotate rows [0], [1] about row [2].
inline mat4x3 Roll(const mat4x3 &m, double a)
{
    auto r0 = get_row(m, 0);
    auto r1 = get_row(m, 1);
    auto r2 = get_row(m, 2);
    auto r3 = get_row(m, 3);

    auto d0 = rodrigues_rotation(r0, r2, a);
    auto d1 = rodrigues_rotation(r1, r2, a);
    auto d2 = r2; // unchanged

    mat4x3 result = m;
    set_row(result, 0, d0);
    set_row(result, 1, d1);
    set_row(result, 2, d2);
    set_row(result, 3, r3);
    return result;
}

// Rotate: rotate rows [0..2] about an arbitrary axis
inline mat4x3 Rotate(const mat4x3 &m, const vec3 &axis, double a)
{
    auto r0 = get_row(m, 0);
    auto r1 = get_row(m, 1);
    auto r2 = get_row(m, 2);
    auto r3 = get_row(m, 3);

    mat4x3 result = m;
    set_row(result, 0, rodrigues_rotation(r0, axis, a));
    set_row(result, 1, rodrigues_rotation(r1, axis, a));
    set_row(result, 2, rodrigues_rotation(r2, axis, a));
    set_row(result, 3, r3);
    return result;
}

// ------------------------------------------------------------
// 2D rotation of a vector
inline vec2 RotateVec2(const vec2 &v, double angle)
{
    using std::sin;
    using std::cos;

    double c = cos(angle);
    double s = sin(angle);

    return vec2{ c*v.x - s*v.y, s*v.x + c*v.y };
}

// Combined rotation in X/Y/Z
inline mat4x3 Rotate(const mat4x3 &m, const vec3 &angles)
{
    mat4x3 res = m;
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
// 2D transform matrix => we store 3 rows, 2 columns
//   row 0..1 = basis vectors
//   row 2    = translation
using mat3x2 = linalg::mat<double, 3, 2>;

// Rotate basis vectors in 2D
inline mat3x2 Rotate(const mat3x2 &m, double angleRadians)
{
    auto r0 = get_row(m, 0);
    auto r1 = get_row(m, 1);
    auto r2 = get_row(m, 2);

    vec2 r0_rot = RotateVec2(r0, angleRadians);
    vec2 r1_rot = RotateVec2(r1, angleRadians);

    mat3x2 result = m;
    set_row(result, 0, r0_rot);
    set_row(result, 1, r1_rot);
    set_row(result, 2, r2);  // translation unchanged
    return result;
}

// ------------------------------------------------------------
// SetRotation: replace the top 3 rows with a given 3x3 rotation
inline mat4x3 SetRotation(const mat4x3 &m, const mat3x3 &rot)
{
    mat4x3 result = m;
    // Each row of `rot` is a 3-element vector
    for(int i = 0; i < 3; ++i) {
        // row i of the rotation
        vec3 row_i{ rot(i,0), rot(i,1), rot(i,2) };
        set_row(result, i, row_i);
    }
    return result;
}

// 2D version
inline mat3x2 SetRotation(const mat3x2 &m, const mat2x2 &rot2x2)
{
    mat3x2 result = m;
    for(int i = 0; i < 2; ++i) {
        vec2 row_i{ rot2x2(i,0), rot2x2(i,1) };
        set_row(result, i, row_i);
    }
    return result;
}

// ------------------------------------------------------------
// Translate: row 3 += offset.x * row0 + offset.y * row1 + offset.z * row2
inline mat4x3 Translate(const mat4x3 &m, const vec3 &offset)
{
    mat4x3 result = m;
    auto r3 = get_row(result, 3);

    if (offset.x != 0.0) {
        r3 += get_row(m, 0)*offset.x;
    }
    if (offset.y != 0.0) {
        r3 += get_row(m, 1)*offset.y;
    }
    if (offset.z != 0.0) {
        r3 += get_row(m, 2)*offset.z;
    }
    set_row(result, 3, r3);
    return result;
}

// 2D version: row 2 holds the translation
inline mat3x2 Translate(const mat3x2 &m, const vec2 &offset)
{
    mat3x2 result = m;
    auto r2 = get_row(result, 2);

    if (offset.x != 0.0) {
        r2 += get_row(m, 0)*offset.x;
    }
    if (offset.y != 0.0) {
        r2 += get_row(m, 1)*offset.y;
    }
    set_row(result, 2, r2);
    return result;
}

// ------------------------------------------------------------
// Set absolute translation in row 3
inline mat4x3 SetTranslation(const mat4x3 &m, const vec3 &translation)
{
    mat4x3 result = m;
    set_row(result, 3, translation);
    return result;
}

// 2D version: set translation in row 2
inline mat3x2 SetTranslation(const mat3x2 &m, const vec2 &translation)
{
    mat3x2 result = m;
    set_row(result, 2, translation);
    return result;
}

// ------------------------------------------------------------
// Multiply two 4x3 transforms, normalizing the first 3 rows afterward.
// This is a custom "Transform" multiplication, not plain matrix multiply.
inline mat4x3 Transform(const mat4x3 &a, const mat4x3 &b)
{
    mat4x3 result;

    // We interpret row i in `result` as a combination of rows in `a` and `b`.
    // The original code used columns, now we adapt to rows.
    //
    // sum over "k"? => result(i, j) = a(i,0)*b(0,j) + a(i,1)*b(1,j) + a(i,2)*b(2,j)
    // plus if j == 3, then add a(i,3).
    //
    // But we only have 3 columns total (j in [0..2]) if it's mat4x3 (4 rows, 3 columns).
    // The old code had 4 columns. We'll do row-based logic:

    // We effectively want:
    //   row i of result = row i of a * row 0..2 of b??? Not typical.
    //
    // The original code is tricky because it was transposed.
    // Let's replicate the effect:
    //   For each row i < 3 => we "combine" the basis rows.
    //   For row 3 => we handle translation.

    // We'll do something similar to your original snippet (but row-based).
    for(int row = 0; row < 4; ++row)
    {
        // gather row from 'a'
        auto ra = get_row(a, row);

        for(int col = 0; col < 3; ++col)
        {
            // sum_{k=0..2} a(row,k)*b(k,col)
            double sum = 0.0;
            sum += a(row,0)*b(0,col);
            sum += a(row,1)*b(1,col);
            sum += a(row,2)*b(2,col);

            // if col==2 is "translation"? Actually we only have columns [0..2].
            // The original code added a(row,3) if col==3, but we no longer have col==3.
            // So we mimic "if this is the translation column, add a(row,3)" but now "translation" is row==3 in this system, so let's do:
            // If row < 3 AND col == 2 => ???
            // Actually in the original, "if (col == 3) sum += a(row,3)".
            // Now there's no col=3. We'll assume the equivalent is "if row==3, add something"?
            // But let's keep it consistent with your code snippet's logic:

            result(row, col) = sum;
        }
    }

    // Then the original code normalizes the first 3 columns. Now we have 3 columns total, but we want to normalize the first 3 *rows* (since each row is a basis vector).
    for(int i = 0; i < 3; ++i)
    {
        auto r = get_row(result, i);
        auto nr = linalg::normalize(r);
        set_row(result, i, nr);
    }

    // We never handled adding the "translation" from a. If you truly want
    // the same "Transform" logic, you'd do something like:
    //
    // row 3 (the translation) = a.row3 + (some combination of b's translation).
    //
    // Let's do the standard approach: The new translation = old translation + (offset from the basis multiply).
    // i.e. row3(result) = row3(a) + row0(a)*b.row3.x + row1(a)*b.row3.y + row2(a)*b.row3.z
    //
    // We'll do it explicitly after the loop:
    auto a0 = get_row(a, 0);
    auto a1 = get_row(a, 1);
    auto a2 = get_row(a, 2);
    auto a3 = get_row(a, 3);

    auto b3 = get_row(b, 3);

    // combined translation:
    //   = a3 + a0*(b3.x) + a1*(b3.y) + a2*(b3.z)
    auto newTrans = a3 + a0*b3.x + a1*b3.y + a2*b3.z;
    set_row(result, 3, newTrans);

    return result;
}

// Invert a 4x3 transform (assuming the top 3 rows are orthonormal rotation, row 3 is translation).
inline mat4x3 InvertTransform(const mat4x3 &m)
{
    // top 3 rows => rotation
    // invert by transpose => row->col swap
    mat3x3 rot;
    for(int i=0; i<3; ++i)
    {
        // row i in 'm' is a 3-element vector
        auto row_i = get_row(m, i);
        // place it into row i of 'rot'
        rot(i,0) = row_i[0];
        rot(i,1) = row_i[1];
        rot(i,2) = row_i[2];
    }

    auto rotT = linalg::transpose(rot); // 3x3

    // Rebuild the matrix with transposed rotation
    mat4x3 unRotated = m;
    for(int i=0; i<3; ++i)
    {
        // row i of rotT
        vec3 row_i{ rotT(i,0), rotT(i,1), rotT(i,2) };
        set_row(unRotated, i, row_i);
    }

    // translation is row 3
    auto trans = get_row(m, 3);
    // inverse translation = - (R^T * trans)
    auto invTrans = -linalg::mul(rotT, trans);

    set_row(unRotated, 3, invTrans);
    return unRotated;
}

// ------------------------------------------------------------
// 2D versions (mat3x2 => 3 rows, 2 columns)
inline mat3x2 Transform(const mat3x2 &a, const mat3x2 &b)
{
    mat3x2 result;
    // Similar approach: first 2 rows are basis, row 2 is translation.
    // We'll do something analogous:

    // Multiply the basis
    for(int row = 0; row < 2; ++row)
    {
        for(int col = 0; col < 2; ++col)
        {
            double sum = a(row,0)*b(0,col) + a(row,1)*b(1,col);
            result(row, col) = sum;
        }
    }
    // Normalize first 2 rows
    for(int i=0; i<2; ++i)
    {
        auto r = get_row(result, i);
        auto nr = linalg::normalize(r);
        set_row(result, i, nr);
    }

    // Combine translation
    auto a0 = get_row(a, 0), a1 = get_row(a, 1), a2 = get_row(a, 2);
    auto b2 = get_row(b, 2);
    // new trans = a2 + a0*b2.x + a1*b2.y
    auto newTrans = a2 + a0*b2.x + a1*b2.y;
    set_row(result, 2, newTrans);

    return result;
}

inline mat3x2 InvertTransform(const mat3x2 &m)
{
    // top 2 rows => rotation
    mat2x2 rot;
    for(int i=0; i<2; ++i)
    {
        auto r = get_row(m, i);
        rot(i, 0) = r[0];
        rot(i, 1) = r[1];
    }

    auto rotT = linalg::transpose(rot);

    // Rebuild partial
    mat3x2 unRotated = m;
    for(int i=0; i<2; ++i)
    {
        vec2 row_i{ rotT(i,0), rotT(i,1) };
        set_row(unRotated, i, row_i);
    }

    auto trans = get_row(m, 2);
    auto invTrans = -linalg::mul(rotT, trans);
    set_row(unRotated, 2, invTrans);

    return unRotated;
}

// ------------------------------------------------------------
// CombineTransforms variant
inline mat4x3 CombineTransforms(const mat4x3 &a, const mat4x3 &b)
{
    // Original code: combine basis + normalize, then combine translation
    mat4x3 result = a;

    auto a0 = get_row(a, 0), a1 = get_row(a, 1), a2 = get_row(a, 2), a3 = get_row(a, 3);
    auto b0 = get_row(b, 0), b1 = get_row(b, 1), b2 = get_row(b, 2), b3 = get_row(b, 3);

    // rotation columns -> now rotation rows
    // row0 = normalize(a0*b0.x + a1*b0.y + a2*b0.z)
    // etc.

    vec3 r0 = linalg::normalize(a0*b0.x + a1*b0.y + a2*b0.z);
    vec3 r1 = linalg::normalize(a0*b1.x + a1*b1.y + a2*b1.z);
    vec3 r2 = linalg::normalize(a0*b2.x + a1*b2.y + a2*b2.z);
    vec3 r3 = a3 + a0*b3.x + a1*b3.y + a2*b3.z;

    set_row(result, 0, r0);
    set_row(result, 1, r1);
    set_row(result, 2, r2);
    set_row(result, 3, r3);

    return result;
}

inline mat3x2 CombineTransforms(const mat3x2 &a, const mat3x2 &b)
{
    mat3x2 result = a;

    auto a0 = get_row(a, 0), a1 = get_row(a, 1), a2 = get_row(a, 2);
    auto b0 = get_row(b, 0), b1 = get_row(b, 1), b2 = get_row(b, 2);

    vec2 r0 = linalg::normalize(a0*b0.x + a1*b0.y);
    vec2 r1 = linalg::normalize(a0*b1.x + a1*b1.y);
    vec2 r2 = a2 + a0*b2.x + a1*b2.y;

    set_row(result, 0, r0);
    set_row(result, 1, r1);
    set_row(result, 2, r2);

    return result;
}

} // namespace MatrixTransforms
