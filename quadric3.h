#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <iostream>

class Quadric;

class Quadric3
{
private:
    glm::mat3 A;
    glm::vec3 b;
    float c;

public:
    Quadric3();
    Quadric3(const Quadric& q, const float& constR);

    glm::mat3 getA() const
    {
        return A;
    }

    glm::vec3 getB() const
    {
        return b;
    }

    float getC() const
    {
        return c;
    }

    Quadric3 operator+(const Quadric3& quadric);
    Quadric3 operator*(const float& multiplier);

    void operator+=(const Quadric3& quadric);
    void operator*=(const float& multiplier);

    float evaluateSQEM(const glm::vec3& sphereOrigin) const;
    glm::vec3 minimizer() const;

    friend std::ostream& operator<<(std::ostream& os, const Quadric3& q);
    void print();
};
