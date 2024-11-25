#include "quadric2.h"
#include "quadric.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

Quadric2::Quadric2()
{
    this->A = glm::mat2(0.0f);
    this->b = glm::vec2(0.0f);
    this->c = 0.0f;
}

Quadric2::Quadric2(const Quadric& q, const glm::vec3& u, const glm::vec3& v)
{
    this->A = glm::mat2(0.0f);

    auto qA1133 = glm::mat3(q.getA());
    glm::vec3 qA4143 = glm::vec3(q.getA()[0][3], q.getA()[1][3], q.getA()[2][3]);
    float qA44 = q.getA()[3][3];

    glm::vec3 mu = u - v;

    float mux = mu.x;
    float muy = mu.y;
    float muz = mu.z;

    float a = qA1133[0][0];
    float b = qA1133[0][1];
    float c = qA1133[0][2];
    float d = qA1133[1][0];
    float e = qA1133[1][1];
    float f = qA1133[1][2];
    float g = qA1133[2][0];
    float h = qA1133[2][1];
    float i = qA1133[2][2];

    float A00 = glm::dot(glm::vec3(mux * a + muy * b + muz * c, mux * d + muy * e + muz * f, mux * g + muy * h + muz * i), mu);
    float A01 = glm::dot(qA4143, mu);
    float A10 = A01;
    float A11 = qA44;

    this->A[0][0] = A00;
    this->A[0][1] = A01;
    this->A[1][0] = A10;
    this->A[1][1] = A11;

    this->b = glm::vec2(0.0f);

    glm::vec3 qb13 = glm::vec3(q.getB());  // Truncate glm::vec4 to glm::vec3
    float qb44 = q.getB().w;

    float b0 = glm::dot(qb13, mu) - glm::dot(glm::vec3(mux * a + muy * b + muz * c, mux * d + muy * e + muz * f, mux * g + muy * h + muz * i), mu);
    float b1 = qb44 - glm::dot(qA4143, mu);

    this->b = glm::vec2(b0, b1);

    this->c = q.getC() - glm::dot(qb13, mu) + 0.5f * glm::dot(glm::vec3(mux * a + muy * b + muz * c, mux * d + muy * e + muz * f, mux * g + muy * h + muz * i), mu);
}

Quadric2 Quadric2::operator+(const Quadric2& quadric)
{
    Quadric2 result;

    result.A = this->A + quadric.A;
    result.b = this->b + quadric.b;
    result.c = this->c + quadric.c;

    return result;
}

Quadric2 Quadric2::operator*(const float& multiplier)
{
    Quadric2 result;

    result.A = this->A * multiplier;
    result.b = this->b * multiplier;
    result.c = this->c * multiplier;

    return result;
}

void Quadric2::operator+=(const Quadric2& quadric)
{
    this->A += quadric.A;
    this->b += quadric.b;
    this->c += quadric.c;
}

void Quadric2::operator*=(const float& multiplier)
{
    this->A *= multiplier;
    this->b *= multiplier;
    this->c *= multiplier;
}

glm::vec2 Quadric2::minimizer() const
{
    glm::vec2 result;

    const float minRadius = 0.01f;

    try
    {
        result = glm::inverse(A) * (-b / 2.0f);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Matrix has determinant 0 for this quadric" << std::endl;

        float Ahat = 0.5f * this->A[1][1];
        float bhat = (this->A[0][1] / 4.0f) + (this->A[1][0] / 4.0f) + this->b.y;

        float radius = (-bhat / 2.0f) / Ahat;
        return glm::vec2(0.5f, radius);
    }

    return result;
}

glm::vec2 Quadric2::constrainR(const float& radius) const
{
    float Ahat = 0.5f * this->A[0][0] * this->A[1][1];
    float bhat = 0.5f * radius * this->A[0][1] - this->b.x;

    float lambda = (-bhat / 2.0f) / Ahat;
    return glm::vec2(lambda, radius);
}

void Quadric2::print()
{
    std::cout << "Matrix A: " << glm::to_string(A) << std::endl;
    std::cout << "Vector b: " << glm::to_string(b) << std::endl;
    std::cout << "Scalar c: " << c << std::endl;
}
