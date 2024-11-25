#include "quadric3.h"
#include "quadric.h"
#include <glm/gtc/matrix_inverse.hpp>

Quadric3::Quadric3()
{
    this->A = glm::mat3(0.0f);
    this->b = glm::vec3(0.0f);
    this->c = 0.0f;
}

Quadric3::Quadric3(const Quadric& q, const float& R)
{
    auto M = q.getA();
    this->A = glm::mat3(M);

    glm::vec3 b3 = glm::vec3(q.getB());
    glm::vec3 M13 = glm::vec3(M[3][0], M[3][1], M[3][2]);
    this->b = b3 + 2.0f * R * M13;

    float b1 = q.getB().w;
    float M11 = M[3][3];
    this->c = q.getC() + R * R * M11 + R * b1;
}

Quadric3 Quadric3::operator+(const Quadric3 &quadric)
{
    Quadric3 result;

    result.A = this->A + quadric.A;
    result.b = this->b + quadric.b;
    result.c = this->c + quadric.c;

    return result;
}

Quadric3 Quadric3::operator*(const float &multiplier)
{
    Quadric3 result;

    result.A = this->A * multiplier;
    result.b = this->b * multiplier;
    result.c = this->c * multiplier;

    return result;
}

void Quadric3::operator+=(const Quadric3& quadric)
{
    this->A += quadric.A;
    this->b += quadric.b;
    this->c += quadric.c;
}

void Quadric3::operator*=(const float& multiplier)
{
    this->A *= multiplier;
    this->b *= multiplier;
    this->c *= multiplier;
}

float Quadric3::evaluateSQEM(const glm::vec3 &s) const
{
    return glm::dot(s, A * s) + glm::dot(b, s) + c;
}

glm::vec3 Quadric3::minimizer() const
{
    glm::vec3 result;

    try
    {
        result = glm::inverse(A) * (-b / 2.0f);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Matrix has determinant 0 for this Quadric3" << std::endl;
    }

    return result;
}
