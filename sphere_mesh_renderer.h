#pragma once

#include "sphere_mesh.h"
#include "shader.h"
#include "mesh.h"

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

    void render();

    void selectSphere(int i);
    void unselectSphere(int i);

    void selectRecentSphere(int i);
    void unselectRecentSphere(int i);

    std::string exportGeometryAsString() const;

private:
    std::vector<RenderableSphere> renderableSpheres;
    std::vector<PrysmTri> tris;

    std::vector<Vertex> m_vertices;
    std::vector<unsigned int> m_indices;

    struct SubMesh {
        size_t indexOffset;
        size_t indexCount;
        glm::vec3 color;
    };
    std::vector<SubMesh> m_subMeshes;

    QOpenGLVertexArrayObject m_VAO;
    QOpenGLBuffer m_VBO { QOpenGLBuffer::VertexBuffer };
    QOpenGLBuffer m_EBO { QOpenGLBuffer::IndexBuffer };

    SphereMesh* sm;

    void buildPrysmoidGeometry(int index, const glm::vec3 &color);
    void buildQuadGeometry(int index, const glm::vec3 &color);
    void buildCapsuloidGeometry(int index, const glm::vec3 &color);
    void buildCapsuleBetweenSpheres(int sphereIndex1, int sphereIndex2,
                                    const glm::vec3& color);

    void appendTriangle(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3,
                        const glm::vec3& n1, const glm::vec3& n2, const glm::vec3& n3);
    void uploadGeometryToGPU();

    void renderSphere(const glm::vec3& center, float radius, const glm::vec3& color) const;

    void renderSpheres() const;
    void renderFull() const;

    glm::vec3 computeUpperPlaneNormal(const Sphere &sa, const Sphere &sb, const Sphere &sc, int direction) const;
};
