#include "quadric.h"
#include "quadric3.h"
#include "quadric2.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/string_cast.hpp>
#include <cfloat>

Quadric::Quadric()
{
    A = glm::mat4(0.0f);
    b = glm::vec4(0, 0, 0, 0);
    c = 0;
}

Quadric::Quadric(const glm::vec3 &faceOrigin, const glm::vec3 &faceNormal)
{
    A = glm::mat4(0.0f);
    glm::mat3 faceNormalOuter = glm::outerProduct(faceNormal, faceNormal);
    A = glm::mat4(faceNormalOuter);

    glm::vec4 homogenousFaceNormal = glm::vec4(faceNormal, 1.0f);
    A[3] = homogenousFaceNormal;
    A = glm::transpose(A);

    b = homogenousFaceNormal * (-2.0f * glm::dot(faceNormal, faceOrigin));
    c = glm::dot(faceNormal, faceOrigin) * glm::dot(faceNormal, faceOrigin);
}

Quadric Quadric::operator+(const Quadric &quadric) const
{
    Quadric result;

    result.A = this->A + quadric.A;
    result.b = this->b + quadric.b;
    result.c = this->c + quadric.c;

    return result;
}

Quadric Quadric::operator*(const float &multiplier)
{
    Quadric result;

    result.A = this->A * multiplier;
    result.b = this->b * multiplier;
    result.c = this->c * multiplier;

    return result;
}

void Quadric::operator+=(const Quadric &quadric)
{
    this->A += quadric.A;
    this->b += quadric.b;
    this->c += quadric.c;
}

void Quadric::operator*=(const float &multiplier)
{
    this->A *= multiplier;
    this->b *= multiplier;
    this->c *= multiplier;
}

float Quadric::evaluateSQEM(const glm::vec4 &s) const
{
    return glm::dot(s, A * s) + glm::dot(b, s) + c;
}

glm::vec4 Quadric::minimizer(float minimumRadius, float maximumRadius) const
{
    glm::vec4 result;

    try
    {
        result = glm::inverse(A) * (-b / 2.0f);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Matrix has determinant 0 for this quadric" << std::endl;
        return glm::vec4(-1, -1, -1, 0);
    }

    if (result.w < minimumRadius)
    {
        Quadric3 q3 = Quadric3(*this, minimumRadius);
        auto newOrigin = q3.minimizer();
        result = glm::vec4(newOrigin, minimumRadius);
    }
    else if (result.w > maximumRadius)
    {
        Quadric3 q3 = Quadric3(*this, maximumRadius);
        auto newOrigin = q3.minimizer();
        result = glm::vec4(newOrigin, maximumRadius);
    }

    return result;
}

void Quadric::getMinimumAndMinimizer(float &min, glm::vec4 &_minimizer, float maximumRadius, float minimumRadius) const
{
    _minimizer = minimizer(minimumRadius, maximumRadius);
    min = evaluateSQEM(_minimizer);
}

glm::vec4 Quadric::constrainIntoVector(const glm::vec3 &u, const glm::vec3 &v, const float &radius)
{
    auto r = radius;

    Quadric2 q2 = Quadric2(*this, u, v);
    auto minimizer = q2.minimizer();

    if (minimizer.y > r)
        minimizer = q2.constrainR(r);

    auto mu = u - v;
    auto newOrigin = u + (minimizer.x * mu);
    auto newRadius = minimizer.y;

    return glm::vec4(newOrigin, newRadius);
}

glm::vec4 Quadric::constrainR(const float &radius)
{
    auto r = radius;

    Quadric3 q3 = Quadric3(*this, r);
    auto newOrigin = q3.minimizer();

    return glm::vec4(newOrigin, radius);
}

void Quadric::addQuadricToTargetRadius(const float &t)
{
    const int newQuadricWeight = 1000;

    Quadric q = Quadric();

    q.A = glm::mat4(0.0f);
    q.b = glm::vec4(0, 0, 0, -t * 2);
    q.c = t;

    *this = *this + q * newQuadricWeight;
}

void Quadric::print() const
{
    std::cout << "Matrix A:\n" << glm::to_string(A) << std::endl;
    std::cout << "Vector b:\n" << glm::to_string(b) << std::endl;
    std::cout << "Scalar c: " << c << std::endl;
}
