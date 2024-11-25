#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include "mesh.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cfloat>

class Quadric
{
public:
    glm::mat4 A;
    glm::vec4 b;
    float c;

    Quadric();
    Quadric(const glm::vec3& faceOrigin, const glm::vec3& faceNormal);
    Quadric(const Quadric& other)
    {
        this->A = other.A;
        this->b = other.b;
        this->c = other.c;
    }

    static Quadric initializeQuadricFromVertex(const Vertex& vertex, float targetSphereRadius = 1.0f)
    {
        Quadric q;

        // (p * I * p - 2 * t * p + t * t) * weight -> con t sfera target, ovvero mi sposto sulla normale nella direzione negativa t = (vertex.position - k * vertex.normal, k) -> (sfera)
        glm::vec3 n = vertex.normal;
        n = glm::normalize(n);

        if (glm::length2(n) - 0.99f < 0)
        {
            std::cerr << "Normal not normalized: " << glm::length(n) << std::endl;
        }

        glm::vec4 t = glm::vec4(vertex.position - targetSphereRadius * n, targetSphereRadius);

        q.A = glm::mat4(0.0f);
        q.b = -2.0f * t;
        q.c = glm::dot(t, t);

        return q;
    }

    [[nodiscard]] glm::mat4 getA() const
    {
        return A;
    }

    [[nodiscard]] glm::vec4 getB() const
    {
        return b;
    }

    [[nodiscard]] float getC() const
    {
        return c;
    }

    Quadric operator+(const Quadric& quadric) const;
    Quadric operator*(const float& multiplier);

    void operator+=(const Quadric& quadric);
    void operator*=(const float& multiplier);

    [[nodiscard]] float evaluateSQEM(const glm::vec4& sphere) const;
    [[nodiscard]] glm::vec4 minimizer(float minimumRadius, float maximumRadius = DBL_MAX) const;
    void getMinimumAndMinimizer(float& min, glm::vec4& _minimizer, float maximumRadius = DBL_MAX, float minimumRadius = 0.01f) const;

    glm::vec4 constrainIntoVector(const glm::vec3& start, const glm::vec3& end, const float& radius);
    glm::vec4 constrainR(const float& radius);

    void addQuadricToTargetRadius(const float& t);

    void print() const;
};
