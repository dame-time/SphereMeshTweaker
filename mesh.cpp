#include "mesh.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <iostream>
#include <fstream>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFunctions>

Mesh::Mesh() : vao(new QOpenGLVertexArrayObject()), vbo(QOpenGLBuffer::VertexBuffer), ebo(QOpenGLBuffer::IndexBuffer) {
    vao->create();
    vbo.create();
    ebo.create();
}

Mesh::~Mesh() {
    vao->destroy();
    vbo.destroy();
    ebo.destroy();
}

bool Mesh::loadFromFile(const std::string& filePath) {
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(filePath,
                                             aiProcess_Triangulate |
                                                 aiProcess_FlipUVs |
                                                 aiProcess_CalcTangentSpace |
                                                 aiProcess_GenSmoothNormals |
                                                 aiProcess_JoinIdenticalVertices);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cerr << "Error loading mesh with Assimp: " << importer.GetErrorString() << std::endl;
        return false;
    }

    aiMesh* mesh = scene->mMeshes[0];
    processMesh(mesh, scene);

    setupMesh();

    return true;
}

void Mesh::processMesh(aiMesh* mesh, const aiScene* scene) {
    vertices.clear();
    indices.clear();
    originalVertices.clear();

    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;

        vertex.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        vertex.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);

        if (mesh->mTextureCoords[0])
            vertex.texCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
        else
            vertex.texCoords = glm::vec2(0.0f, 0.0f);

        originalVertices.push_back(vertex);
        vertices.push_back(vertex);
    }

    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);
    }
}

void Mesh::setupMesh() {
    vao->bind();

    vbo.bind();
    vbo.allocate(&vertices[0], vertices.size() * sizeof(Vertex));

    ebo.bind();
    ebo.allocate(&indices[0], indices.size() * sizeof(unsigned int));

    QOpenGLFunctions f;
    f.initializeOpenGLFunctions();

    f.glEnableVertexAttribArray(0);
    f.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    f.glEnableVertexAttribArray(1);
    f.glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    f.glEnableVertexAttribArray(2);
    f.glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));

    vao->release();

    glm::vec3 minCorner = glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    glm::vec3 maxCorner = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& vertex : vertices)
        for (int i = 0; i < 3; i++) {
            if (vertex.position[i] > maxCorner[i])
                maxCorner[i] = vertex.position[i];
            if (vertex.position[i] < minCorner[i])
                minCorner[i] = vertex.position[i];
        }

    glm::vec3 diagonal = maxCorner - minCorner;
    BDD = glm::max(diagonal.x, glm::max(diagonal.y, diagonal.z));
}

glm::vec3 Mesh::getCenter() const {
    glm::vec3 center(0.0f);

    for (const auto& vertex : vertices)
        center += vertex.position;

    center /= (float)vertices.size();

    return center;
}

void Mesh::renderWireframe() const {
    if (!isWireframe || !isVisible) return;

    glStencilFunc(GL_EQUAL, 1, 0xFF);  // Render only where the stencil buffer is 1
    glStencilMask(0x00);  // Disable writing to the stencil buffer during wireframe rendering

    glEnable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(-1.0f, -1.0f);

    vao->bind();
    glLineWidth(1.5f);
    glDrawElements(GL_TRIANGLES, static_cast<int>(indices.size()), GL_UNSIGNED_INT, 0);
    vao->release();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilMask(0xFF);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_POLYGON_OFFSET_LINE);
    glDisable(GL_CULL_FACE);
}

void Mesh::render(bool blend) const {
    if (!isVisible) return;

    glEnable(GL_STENCIL_TEST);

    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilMask(0xFF);

    if (blend)
    {
        glEnable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT, GL_FILL);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    vao->bind();
    glDrawElements(GL_TRIANGLES, static_cast<int>(indices.size()), GL_UNSIGNED_INT, 0);
    vao->release();
    if (blend)
    {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }

    glStencilMask(0x00);
}

bool Mesh::saveToOBJ(const std::string& outputFolder, const std::string& outName) const {
    std::ofstream objFile(outputFolder + outName);

    if (!objFile.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << outputFolder + outName << std::endl;
        return false;
    }

    for (const auto& vertex : vertices)
        objFile << "v " << vertex.position.x << " " << vertex.position.y << " " << vertex.position.z << "\n";

    for (const auto& vertex : vertices)
        objFile << "vn " << vertex.normal.x << " " << vertex.normal.y << " " << vertex.normal.z << "\n";

    for (unsigned int i = 0; i < indices.size(); i += 3)
        objFile << "f "
                << indices[i] + 1 << "/" << indices[i] + 1 << "/" << indices[i] + 1 << " "
                << indices[i + 1] + 1 << "/" << indices[i + 1] + 1 << "/" << indices[i + 1] + 1 << " "
                << indices[i + 2] + 1 << "/" << indices[i + 2] + 1 << "/" << indices[i + 2] + 1 << "\n";

    objFile.close();
    return true;
}

void Mesh::resetToOriginal() {
    if (originalVertices.empty()) {
        std::cerr << "Error: Original vertices not available for reset." << std::endl;
        return;
    }

    vertices = originalVertices;

    vbo.bind();
    vbo.write(0, &vertices[0], vertices.size() * sizeof(Vertex));
}

void Mesh::updateNormals() {
    for (auto & vertex : vertices)
        vertex.normal = glm::vec3(0.0f);

    for (unsigned int i = 0; i < indices.size(); i += 3) {
        glm::vec3 v1 = vertices[indices[i]].position;
        glm::vec3 v2 = vertices[indices[i + 1]].position;
        glm::vec3 v3 = vertices[indices[i + 2]].position;

        glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));

        vertices[indices[i]].normal += normal;
        vertices[indices[i + 1]].normal += normal;
        vertices[indices[i + 2]].normal += normal;
    }

    for (auto & vertex : vertices)
        vertex.normal = glm::normalize(vertex.normal);

    vbo.bind();
    vbo.write(0, &vertices[0], vertices.size() * sizeof(Vertex));
}
