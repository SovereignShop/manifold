#pragma once

#include "glm/glm.hpp"
#include <vector>

namespace BufferUtils {


std::vector<int> intVectorFromPointer(int* data, size_t count) {
    std::vector<int> vec(data, data + count);
    return vec;
}



std::vector<float> floatVectorFromPointer(float* data, size_t count) {
    std::vector<float> vec(data, data + count);
    return vec;
}

std::vector<glm::vec2> createDoubleVec2Vector(double* values, std::size_t count) {
    std::vector<glm::vec2> result(count / 2);

    for (std::size_t i = 0; i < count; i += 2) {
        result[i/2] = glm::vec2(values[i], values[i + 1]);
    }
    return result;
}

std::vector<glm::vec3> createDoubleVec3Vector(double* values, std::size_t count) {
    std::vector<glm::vec3> result(count / 3);

    for (std::size_t i = 0; i < count; i += 3) {
        result[i/3] = glm::vec3(values[i], values[i + 1], values[i + 2]);
    }

    return result;
}

std::vector<glm::vec3> createFloatVec3Vector(float* values, std::size_t count) {
    std::vector<glm::vec3> result(count / 3);

    for (std::size_t i = 0; i < count; i += 3) {
        result[i/3] = glm::vec3(values[i], values[i + 1], values[i + 2]);
    }

    return result;
}

std::vector<glm::ivec3> createIntegerVec3Vector(int* values, std::size_t count) {
    std::vector<glm::ivec3> result(count / 3);

    for (std::size_t i = 0; i < count; i += 3) {
        result[i/3] = glm::ivec3(values[i], values[i + 1], values[i + 2]);
    }

    return result;
}

std::vector<glm::vec4> createDoubleVec4Vector(double* values, std::size_t count) {
    std::vector<glm::vec4> result(count / 4);

    for (std::size_t i = 0; i < count; i += 4) {
        result[i/4] = glm::vec4(values[i], values[i + 1], values[i + 2], values[i + 3]);
    }

    return result;
}

std::vector<glm::vec4> createFloatVec4Vector(float* values, std::size_t count) {
    std::vector<glm::vec4> result(count / 4);

    for (std::size_t i = 0; i < count; i += 4) {
        result[count/4] = glm::vec4(values[i], values[i + 1], values[i + 2], values[i + 3]);
    }

    return result;
}

std::vector<glm::ivec4> createIntegerVec4Vector(int* values, std::size_t count) {
    std::vector<glm::ivec4> result(count / 4);

    for (std::size_t i = 0; i < count; i += 4) {
        result[i/4] = glm::ivec4(values[i], values[i + 1], values[i + 2], values[i + 4]);
    }

    return result;
}

} // namespace BufferUtils
