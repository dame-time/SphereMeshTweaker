#pragma once

#include <glm/glm.hpp>
#include <iostream>

class Quadric;

class Quadric2
{
public:
    glm::mat2 A;
    glm::vec2 b;
    float c;

    Quadric2();
    Quadric2(const Quadric& q, const glm::vec3& u, const glm::vec3& v);

    glm::mat2 getA() const
    {
        return A;
    }

    glm::vec2 getB() const
    {
        return b;
    }

    float getC() const
    {
        return c;
    }

    Quadric2 operator+(const Quadric2& quadric);
    Quadric2 operator*(const float& multiplier);

    void operator+=(const Quadric2& quadric);
    void operator*=(const float& multiplier);

    glm::vec2 minimizer() const;
    glm::vec2 constrainR(const float& radius) const;

    void print();
};
