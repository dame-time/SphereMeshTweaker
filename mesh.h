#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <assimp/scene.h>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

struct Vertex {
    glm::vec3 position{};
    glm::vec3 normal{};
    glm::vec2 texCoords{};

    int bumperIndex = -1;
};

class Mesh {
public:
    glm::vec3 color{1.0f};
    float alpha {1.0f};
    bool isWireframe {false};
    bool isVisible {true};

    Mesh();
    ~Mesh();

    bool loadFromFile(const std::string& filePath);
    [[nodiscard]] bool saveToOBJ(const std::string& outputFolder, const std::string& fileName = "outMesh.obj") const;

    [[nodiscard]] glm::vec3 getCenter() const;

    void deflate(float scale);

    void render(bool blend = false) const;
    void renderWireframe() const;

    void resetToOriginal();

    void updateNormals();

private:
    std::vector<Vertex> vertices;
    std::vector<Vertex> originalVertices;
    std::vector<unsigned int> indices;

    QOpenGLVertexArrayObject* vao;
    QOpenGLBuffer vbo;
    QOpenGLBuffer ebo;

    float BDD = 0.0f;

    void setupMesh();

    void processMesh(aiMesh* mesh, const aiScene* scene);
};
