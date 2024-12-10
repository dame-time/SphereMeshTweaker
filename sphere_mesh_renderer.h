#pragma once

#include "sphere_mesh.h"
#include "shader.h"

using namespace SM;

class RenderableSphere
{
public:
    bool isSelected = false;
    bool isRecentSelected = false;

    RenderableSphere() {}

    glm::vec3 getColor() const { return isSelected ? selectedSphereColor : isRecentSelected ? recentSelectSphereColor : unselectedSphereColor; }

private:
    const glm::vec3 unselectedSphereColor {1.0f, 0.0f, 0.0f};
    const glm::vec3 selectedSphereColor {0.9f, 0.9f, 0.0f};
    const glm::vec3 recentSelectSphereColor {0.5f, 0.5f, 0.8f};
};

struct PrysmTri
{
    glm::vec3 a;
    glm::vec3 b;
    glm::vec3 c;
};

class SphereMeshRenderer
{
public:
    Shader* sphereShader;
    Shader* meshShader;

    bool visible {};
    bool filled {};

    int numberOfSpheres {0};
    float connectivitySize {1.0f};
    float sphereSize {1.0f};

    SphereMeshRenderer(SphereMesh* referenceSM);

    void updateSphereMesh(SphereMesh* sm);

    void render() const;

    void selectSphere(int i);
    void unselectSphere(int i);

    void scale(float scale);

    void selectRecentSphere(int i);
    void unselectRecentSphere(int i);

    std::string exportGeometryAsString() const;

private:
    std::vector<RenderableSphere> renderableSpheres;
    static std::vector<float> originalRadii;

    std::vector<PrysmTri> tris;

    SphereMesh* sm;

    void renderSphere(const glm::vec3& center, float radius, const glm::vec3& color) const;

    void renderSpheres() const;
    void renderConnectivity() const;
    void renderFull() const;

    glm::vec3 computeUpperPlaneNormal(const Sphere &sa, const Sphere &sb, const Sphere &sc, int direction) const;

    void regeneratePrysmoidGeometry();
    void regenerateCapsuloidGeometry();
    void renderCachedPrysmoids() const;
    void renderCachedCapsuloids() const;

    void renderTriangle(int index) const;
    void renderPrysmoidCapsule(int index, const glm::vec3& color) const;
    void renderCapsuloidCapsule(int index, const glm::vec3& color) const;
    void renderCapsuleBetweenSpheres(int sphereIndex1, int sphereIndex2, const glm::vec3& color) const;

    void drawSpheresOverPrysmoid(int index) const;
    void drawSpheresOverCapsuloid(int index) const;
};
