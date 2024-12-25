#pragma once

#include "manifold/linalg.h"
#include <vector>

namespace BufferUtils {

using vec2   = linalg::vec<double, 2>;
using vec3   = linalg::vec<double, 3>;
using vec4   = linalg::vec<double, 4>;
using ivec3  = linalg::vec<int, 3>;
using ivec4  = linalg::vec<int, 4>;
using u_int32_t = unsigned int;

std::vector<u_int32_t> uIntVectorFromPointer(long long int* data, size_t count) {
    std::vector<u_int32_t> vec(data, data + count);
    return vec;
}

std::vector<int> intVectorFromPointer(int* data, size_t count) {
    std::vector<int> vec(data, data + count);
    return vec;
}

std::vector<float> floatVectorFromPointer(float* data, size_t count) {
    std::vector<float> vec(data, data + count);
    return vec;
}

std::vector<vec2> createDoubleVec2Vector(double* values, std::size_t count) {
    std::vector<vec2> result(count / 2);

    for (std::size_t i = 0; i < count; i += 2) {
        result[i/2] = vec2(values[i], values[i + 1]);
    }
    return result;
}

std::vector<vec3> createDoubleVec3Vector(double* values, std::size_t count) {
    std::vector<vec3> result(count / 3);

    for (std::size_t i = 0; i < count; i += 3) {
        result[i/3] = vec3(values[i], values[i + 1], values[i + 2]);
    }

    return result;
}

std::vector<vec3> createFloatVec3Vector(float* values, std::size_t count) {
    std::vector<vec3> result(count / 3);

    for (std::size_t i = 0; i < count; i += 3) {
        result[i/3] = vec3(values[i], values[i + 1], values[i + 2]);
    }

    return result;
}

std::vector<ivec3> createIntegerVec3Vector(int* values, std::size_t count) {
    std::vector<ivec3> result(count / 3);

    for (std::size_t i = 0; i < count; i += 3) {
        result[i/3] = ivec3(values[i], values[i + 1], values[i + 2]);
    }

    return result;
}

std::vector<vec4> createDoubleVec4Vector(double* values, std::size_t count) {
    std::vector<vec4> result(count / 4);

    for (std::size_t i = 0; i < count; i += 4) {
        result[i/4] = vec4(values[i], values[i + 1], values[i + 2], values[i + 3]);
    }

    return result;
}

std::vector<vec4> createFloatVec4Vector(float* values, std::size_t count) {
    std::vector<vec4> result(count / 4);

    for (std::size_t i = 0; i < count; i += 4) {
        result[count/4] = vec4(values[i], values[i + 1], values[i + 2], values[i + 3]);
    }

    return result;
}

std::vector<ivec4> createIntegerVec4Vector(int* values, std::size_t count) {
    std::vector<ivec4> result(count / 4);

    for (std::size_t i = 0; i < count; i += 4) {
        result[i/4] = ivec4(values[i], values[i + 1], values[i + 2], values[i + 4]);
    }

    return result;
}

} // namespace BufferUtils
