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

class SphereMeshRenderer
{
public:
    bool visible {};
    bool filled {};

    int numberOfSpheres {0};
    float sphereSize {1.0f};

    SphereMeshRenderer(SphereMesh* referenceSM);

    void updateSphereMesh(SphereMesh* sm);
    void useShader(Shader* shader);

    void render() const;

    void selectSphere(int i);
    void unselectSphere(int i);

    void selectRecentSphere(int i);
    void unselectRecentSphere(int i);

private:
    std::vector<RenderableSphere> renderableSpheres;

    SphereMesh* sm;
    Shader* sphereShader;

    void renderSphere(const glm::vec3& center, float radius, const glm::vec3& color) const;

    void renderSpheres() const;
    void renderConnectivity() const;
    void renderFull() const;

    void drawSpheresOverPrysmoid(int index) const;
    void drawSpheresOverCapsuloid(int index) const;
};
