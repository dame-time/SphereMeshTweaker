#include "sphere_mesh_renderer.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <functional>
#include <utility>

struct hash_pair {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);

        return hash1 ^ (hash2 << 1);
    }
};

std::vector<float> SphereMeshRenderer::originalRadii;

SphereMeshRenderer::SphereMeshRenderer(SphereMesh* referenceSM) : sm(referenceSM)
{
    updateSphereMesh(sm);
}

void SphereMeshRenderer::updateSphereMesh(SphereMesh* sm)
{
    this->sm = sm;
    renderableSpheres.clear();

    for (int i = 0; i < sm->spheres.size(); i++)
        renderableSpheres.push_back({});

    originalRadii.resize(sm->spheres.size());
    for (size_t i = 0; i < sm->spheres.size(); ++i)
        originalRadii[i] = sm->spheres[i].radius;
}

glm::vec3 SphereMeshRenderer::computeUpperPlaneNormal(const Sphere &sa, const Sphere &sb, const Sphere &sc, int direction) const
{
    glm::vec3 a = sa.center;
    glm::vec3 b = sb.center;
    glm::vec3 c = sc.center;

    auto sign = static_cast<float>(direction);

    glm::vec3 n = sign * glm::cross(b - a, c - a);
    n = glm::normalize(n);
    glm::vec3 startN = n;

    for (int i = 0; i < 1000; i++){
        a = sa.center + n * sa.radius;
        b = sb.center + n * sb.radius;
        c = sc.center + n * sc.radius;

        glm::vec3 new_n = glm::normalize(sign * glm::cross(b - a, c - a));
        if (glm::dot(n, new_n) >= 0.999f)
        {
            if (glm::dot(startN, new_n) < 0)

            return new_n;
        }
        n = new_n;
    }

    return n;
}

void SphereMeshRenderer::render() const
{
    if (!visible)
        return;

    renderSpheres();

    if (!filled)
        return;

    renderFull();
}

void SphereMeshRenderer::selectSphere(int i)
{
    renderableSpheres[i].isSelected = true;
}

void SphereMeshRenderer::unselectSphere(int i)
{
    renderableSpheres[i].isSelected = false;
}

void SphereMeshRenderer::scale(float scale)
{
    float targetRadius = 0.2f;

    for (size_t i = 0; i < sm->spheres.size(); ++i) {
        float originalRadius = originalRadii[i];
        sm->spheres[i].radius = scale * originalRadius + (1.0f - scale) * targetRadius;
    }
}


void SphereMeshRenderer::selectRecentSphere(int i) {
    renderableSpheres[i].isSelected = false;
    renderableSpheres[i].isRecentSelected = true;
}

void SphereMeshRenderer::unselectRecentSphere(int i) {
    renderableSpheres[i].isSelected = false;
    renderableSpheres[i].isRecentSelected = false;
}

void SphereMeshRenderer::renderSpheres() const
{
    for(int i = 0; i < sm->spheres.size(); i++)
        renderSphere(sm->spheres[i].center, sm->spheres[i].radius * sphereSize, renderableSpheres[i].getColor());
}

void SphereMeshRenderer::renderConnectivity() const
{
    for (int i = 0; i < sm->prysmoids.size(); i++)
        drawSpheresOverPrysmoid(i);

    for (int i = 0; i < sm->capsuloids.size(); i++)
        drawSpheresOverCapsuloid(i);
}

void SphereMeshRenderer::renderFull() const
{
    for (int i = 0; i < sm->prysmoids.size(); i++)
    {
        renderPrysmoidCapsule(i, glm::vec3(0.1f, 0.75f, 0.0f));
        renderTriangle(i);
    }

    for (int i = 0; i < sm->capsuloids.size(); i++)
        renderCapsuloidCapsule(i, glm::vec3(0.0f, 0.0f, 0.75f));
}

void SphereMeshRenderer::renderTriangle(int index) const {
    const auto& prysmoid = sm->prysmoids[index];

    glm::vec3 C1 = sm->spheres[prysmoid.indices[0]].center;
    glm::vec3 C2 = sm->spheres[prysmoid.indices[1]].center;
    glm::vec3 C3 = sm->spheres[prysmoid.indices[2]].center;

    float R1 = originalRadii[prysmoid.indices[0]] * connectivitySize;
    float R2 = originalRadii[prysmoid.indices[1]] * connectivitySize;
    float R3 = originalRadii[prysmoid.indices[2]] * connectivitySize;

    glm::vec3 N = computeUpperPlaneNormal(sm->spheres[prysmoid.indices[0]],
                                          sm->spheres[prysmoid.indices[1]],
                                          sm->spheres[prysmoid.indices[2]], 1);
    glm::vec3 NInv = computeUpperPlaneNormal(sm->spheres[prysmoid.indices[0]],
                                             sm->spheres[prysmoid.indices[1]],
                                             sm->spheres[prysmoid.indices[2]], -1);

    glm::vec3 V1_top = C1 + R1 * N;
    glm::vec3 V2_top = C2 + R2 * N;
    glm::vec3 V3_top = C3 + R3 * N;

    glm::vec3 V1_bottom = C1 + R1 * NInv;
    glm::vec3 V2_bottom = C2 + R2 * NInv;
    glm::vec3 V3_bottom = C3 + R3 * NInv;

    glm::vec3 normalTop = glm::normalize(glm::cross(V2_top - V1_top, V3_top - V1_top));
    glm::vec3 normalBottom = glm::normalize(glm::cross(V3_bottom - V1_bottom, V2_bottom - V1_bottom));

    static QOpenGLVertexArrayObject VAO;
    static QOpenGLBuffer VBO(QOpenGLBuffer::VertexBuffer);
    static bool isInitialized = false;

    if (!isInitialized) {
        VAO.create();
        VBO.create();
        isInitialized = true;
    }

    VAO.bind();
    VBO.bind();

    QOpenGLFunctions f;
    f.initializeOpenGLFunctions();

    meshShader->use();
    meshShader->setVec3("material.ambient", glm::vec3(0.8f, 0.5f, 0.3f));
    meshShader->setVec3("material.diffuse", glm::vec3(0.8f, 0.5f, 0.3f));
    meshShader->setVec3("material.specular", glm::vec3(0.1f, 0.1f, 0.1f));
    meshShader->setFloat("material.shininess", 32.0f);

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    std::vector<float> verticesTop = {
        V1_top.x, V1_top.y, V1_top.z, normalTop.x, normalTop.y, normalTop.z,
        V2_top.x, V2_top.y, V2_top.z, normalTop.x, normalTop.y, normalTop.z,
        V3_top.x, V3_top.y, V3_top.z, normalTop.x, normalTop.y, normalTop.z,
    };
    VBO.allocate(verticesTop.data(), verticesTop.size() * sizeof(float));
    f.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    f.glEnableVertexAttribArray(0);
    f.glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    f.glEnableVertexAttribArray(1);
    f.glDrawArrays(GL_TRIANGLES, 0, 3);

    VAO.release();
    VBO.release();

    VAO.bind();
    VBO.bind();

    std::vector<float> verticesBottom = {
        V1_bottom.x, V1_bottom.y, V1_bottom.z, normalBottom.x, normalBottom.y, normalBottom.z,
        V2_bottom.x, V2_bottom.y, V2_bottom.z, normalBottom.x, normalBottom.y, normalBottom.z,
        V3_bottom.x, V3_bottom.y, V3_bottom.z, normalBottom.x, normalBottom.y, normalBottom.z,
    };
    VBO.allocate(verticesBottom.data(), verticesBottom.size() * sizeof(float));
    f.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    f.glEnableVertexAttribArray(0);
    f.glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    f.glEnableVertexAttribArray(1);
    f.glDrawArrays(GL_TRIANGLES, 0, 3);

    VAO.release();
    VBO.release();
    glUseProgram(0);
}

void SphereMeshRenderer::drawSpheresOverPrysmoid(int index) const
{
    glm::vec3 color [3] = {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)};
    Prysmoid p = sm->prysmoids[index];

    for (int i = 0; i < numberOfSpheres; i++)
        for (int j = 0; j < numberOfSpheres - i; j++)
        {
            int k = numberOfSpheres - 1 - i - j;

            // if (i == numberOfSpheres - 1 || j == numberOfSpheres - 1 || k == numberOfSpheres - 1) continue;

            float ci = i * 1.0 / (numberOfSpheres - 1);
            float cj = j * 1.0 / (numberOfSpheres - 1);
            float ck = k * 1.0 / (numberOfSpheres - 1);

            int dim = 2;
            if (i == 0) dim--;
            if (j == 0) dim--;
            if (k == 0) dim--;

            glm::vec3 origin = {sm->spheres[p.indices[0]].center * ci + sm->spheres[p.indices[1]].center * cj
                                + sm->spheres[p.indices[2]].center * ck};
            float radius = glm::mix(0.001f,
                                        sm->spheres[p.indices[0]].radius * ci +
                                        sm->spheres[p.indices[1]].radius * cj +
                                        sm->spheres[p.indices[2]].radius * ck,
                                    (float)connectivitySize);

            renderSphere(origin, radius, color[dim]);
        }
}

void SphereMeshRenderer::drawSpheresOverCapsuloid(int index) const
{
    const glm::vec3 color = glm::vec3(0.1, 0.7, 1);
    Capsuloid c = sm->capsuloids[index];

    for (int i = 1; i < numberOfSpheres - 1; i++)
    {
        renderSphere(glm::mix(sm->spheres[c.indices[0]].center, sm->spheres[c.indices[1]].center, i * 1.0 / (numberOfSpheres - 1)),
                     glm::mix(
                         0.001f,
                         glm::mix(sm->spheres[c.indices[0]].radius, sm->spheres[c.indices[1]].radius, i * 1.0 / (numberOfSpheres - 1)),
                         (float)connectivitySize
                         ),
                     color);
    }
}

void SphereMeshRenderer::renderSphere(const glm::vec3& center, float radius, const glm::vec3& color) const
{
    static QOpenGLVertexArrayObject VAO;
    static QOpenGLBuffer VBO(QOpenGLBuffer::VertexBuffer);
    static QOpenGLBuffer EBO(QOpenGLBuffer::IndexBuffer);
    static std::vector<float> vertices;
    static std::vector<unsigned int> indices;
    static int vertexCount = 0;

    if (!VAO.isCreated()) {
        vertices = {
            -1.0f,  1.0f,
            1.0f,  1.0f,
            1.0f, -1.0f,
            -1.0f, -1.0f
        };

        indices = {
            0, 1, 2,
            2, 3, 0
        };

        vertexCount = static_cast<int>(vertices.size());

        VAO.create();
        VAO.bind();

        VBO.create();
        VBO.bind();
        VBO.allocate(vertices.data(), vertices.size() * sizeof(float));

        EBO.create();
        EBO.bind();
        EBO.allocate(indices.data(), indices.size() * sizeof(unsigned int));

        QOpenGLFunctions f;
        f.initializeOpenGLFunctions();

        f.glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        f.glEnableVertexAttribArray(0);

        VAO.release();
    }

    sphereShader->use();
    sphereShader->setVec3("center", center);
    sphereShader->setVec3("material.ambient", color);
    sphereShader->setVec3("material.diffuse", glm::vec3(0.9f, 0.9f, 0.9f));
    sphereShader->setVec3("material.specular", glm::vec3(0.0f, 0.0f, 0.0f));
    sphereShader->setFloat("material.shininess", 0.0f);
    sphereShader->setFloat("radius", radius);

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    VAO.bind();
    glDrawElements(GL_TRIANGLES, static_cast<int>(indices.size()), GL_UNSIGNED_INT, nullptr);
    VAO.release();

    glUseProgram(0);
}

void SphereMeshRenderer::renderPrysmoidCapsule(int index, const glm::vec3& color) const {
    const auto& prysmoid = sm->prysmoids[index];

    for (int side = 0; side < 3; ++side) {
        int sphereIndex1 = prysmoid.indices[side];
        int sphereIndex2 = prysmoid.indices[(side + 1) % 3];

        renderCapsuleBetweenSpheres(sphereIndex1, sphereIndex2, color);
    }
}

void SphereMeshRenderer::renderCapsuloidCapsule(int index, const glm::vec3& color) const {
    const auto& capsuloid = sm->capsuloids[index];

    renderCapsuleBetweenSpheres(capsuloid.indices[0], capsuloid.indices[1], color);
}

void SphereMeshRenderer::renderCapsuleBetweenSpheres(int sphereIndex1, int sphereIndex2, const glm::vec3& color) const {
    glm::vec3 v0 = sm->spheres[sphereIndex1].center;
    glm::vec3 v1 = sm->spheres[sphereIndex2].center;
    float r0 = originalRadii[sphereIndex1] * connectivitySize;
    float r1 = originalRadii[sphereIndex2] * connectivitySize;

    if (r1 < r0) {
        std::swap(v0, v1);
        std::swap(r0, r1);
    }

    const int SEGMENTS = 128;

    glm::vec3 d = v0 - v1;
    float dLength = glm::length(d);

    float l = std::sqrt(glm::dot(d, d) - std::pow(r1 - r0, 2));

    float r0Bis = r0 * l / dLength;
    float r1Bis = r1 * l / dLength;

    float d0 = (r1 - r0) * r0 / dLength;
    float d1 = (r1 - r0) * r1 / dLength;

    glm::vec3 v0Bis = v0 + glm::normalize(d) * d0;
    glm::vec3 v1Bis = v1 + glm::normalize(d) * d1;

    glm::vec3 arbitrary = (glm::abs(d.x) < 0.99) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(d, arbitrary));
    glm::vec3 up = glm::normalize(glm::cross(right, d));

    std::vector<glm::vec3> circlePoints1;
    std::vector<glm::vec3> circleNormals1;
    std::vector<glm::vec3> circlePoints2;
    std::vector<glm::vec3> circleNormals2;

    for (int i = 0; i < SEGMENTS; ++i) {
        float theta = (2.0f * M_PI * i) / SEGMENTS;

        glm::vec3 offset1 = (right * cos(theta) + up * sin(theta)) * r0Bis;
        circlePoints1.push_back(v0Bis + offset1);
        circleNormals1.push_back(glm::normalize(offset1));

        glm::vec3 offset2 = (right * cos(theta) + up * sin(theta)) * r1Bis;
        circlePoints2.push_back(v1Bis + offset2);
        circleNormals2.push_back(glm::normalize(offset2));
    }

    static QOpenGLVertexArrayObject VAO;
    static QOpenGLBuffer VBO(QOpenGLBuffer::VertexBuffer);
    static bool isInitialized = false;

    if (!isInitialized) {
        VAO.create();
        VBO.create();
        isInitialized = true;
    }

    VAO.bind();
    VBO.bind();

    QOpenGLFunctions f;
    f.initializeOpenGLFunctions();

    meshShader->use();
    meshShader->setVec3("material.ambient", color);
    meshShader->setVec3("material.diffuse", color);
    meshShader->setVec3("material.specular", glm::vec3(0.1f, 0.1f, 0.1f));
    meshShader->setFloat("material.shininess", 32.0f);

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    for (int i = 0; i < SEGMENTS; ++i) {
        int nextIndex = (i + 1) % SEGMENTS;

        std::vector<float> vertices1 = {
            circlePoints1[i].x, circlePoints1[i].y, circlePoints1[i].z,
            circleNormals1[i].x, circleNormals1[i].y, circleNormals1[i].z,

            circlePoints2[i].x, circlePoints2[i].y, circlePoints2[i].z,
            circleNormals2[i].x, circleNormals2[i].y, circleNormals2[i].z,

            circlePoints1[nextIndex].x, circlePoints1[nextIndex].y, circlePoints1[nextIndex].z,
            circleNormals1[nextIndex].x, circleNormals1[nextIndex].y, circleNormals1[nextIndex].z,
        };
        VBO.allocate(vertices1.data(), vertices1.size() * sizeof(float));
        f.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        f.glEnableVertexAttribArray(0);
        f.glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        f.glEnableVertexAttribArray(1);
        f.glDrawArrays(GL_TRIANGLES, 0, 3);

        std::vector<float> vertices2 = {
            circlePoints2[i].x, circlePoints2[i].y, circlePoints2[i].z,
            circleNormals2[i].x, circleNormals2[i].y, circleNormals2[i].z,

            circlePoints2[nextIndex].x, circlePoints2[nextIndex].y, circlePoints2[nextIndex].z,
            circleNormals2[nextIndex].x, circleNormals2[nextIndex].y, circleNormals2[nextIndex].z,

            circlePoints1[nextIndex].x, circlePoints1[nextIndex].y, circlePoints1[nextIndex].z,
            circleNormals1[nextIndex].x, circleNormals1[nextIndex].y, circleNormals1[nextIndex].z,
        };
        VBO.allocate(vertices2.data(), vertices2.size() * sizeof(float));
        f.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        f.glEnableVertexAttribArray(0);
        f.glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        f.glEnableVertexAttribArray(1);
        f.glDrawArrays(GL_TRIANGLES, 0, 3);
    }

    VAO.release();
    VBO.release();
    glUseProgram(0);
}

int getOrAddMidpoint(int v1, int v2, std::vector<glm::vec3>& vertices,
                     std::unordered_map<std::pair<int, int>, int, hash_pair>& edgeMidpointCache,
                     const glm::vec3& center, float radius) {
    auto edge = std::minmax(v1, v2);
    auto it = edgeMidpointCache.find(edge);
    if (it != edgeMidpointCache.end()) {
        return it->second;
    }

    glm::vec3 midpoint = glm::normalize((vertices[v1] + vertices[v2]) * 0.5f - center) * radius + center;
    int index = vertices.size();
    vertices.push_back(midpoint);
    edgeMidpointCache[edge] = index;

    return index;
}

std::pair<std::vector<glm::vec3>, std::vector<unsigned int>> generateIcosphere(const glm::vec3& center, float radius, int subdivisions) {
    const float t = (1.0f + sqrt(5.0f)) / 2.0f;

    std::vector<glm::vec3> vertices = {
        glm::normalize(glm::vec3(-1,  t,  0)) * radius + center,
        glm::normalize(glm::vec3( 1,  t,  0)) * radius + center,
        glm::normalize(glm::vec3(-1, -t,  0)) * radius + center,
        glm::normalize(glm::vec3( 1, -t,  0)) * radius + center,
        glm::normalize(glm::vec3( 0, -1,  t)) * radius + center,
        glm::normalize(glm::vec3( 0,  1,  t)) * radius + center,
        glm::normalize(glm::vec3( 0, -1, -t)) * radius + center,
        glm::normalize(glm::vec3( 0,  1, -t)) * radius + center,
        glm::normalize(glm::vec3( t,  0, -1)) * radius + center,
        glm::normalize(glm::vec3( t,  0,  1)) * radius + center,
        glm::normalize(glm::vec3(-t,  0, -1)) * radius + center,
        glm::normalize(glm::vec3(-t,  0,  1)) * radius + center,
    };

    std::vector<unsigned int> indices = {
        0, 11, 5,  0, 5, 1,  0, 1, 7,  0, 7, 10,  0, 10, 11,
        1, 5, 9,  5, 11, 4,  11, 10, 2,  10, 7, 6,  7, 1, 8,
        3, 9, 4,  3, 4, 2,  3, 2, 6,  3, 6, 8,  3, 8, 9,
        4, 9, 5,  2, 4, 11,  6, 2, 10,  8, 6, 7,  9, 8, 1,
    };

    for (int i = 0; i < subdivisions; ++i) {
        std::unordered_map<std::pair<int, int>, int, hash_pair> edgeMidpointCache;
        std::vector<unsigned int> newIndices;

        for (size_t j = 0; j < indices.size(); j += 3) {
            int v1 = indices[j];
            int v2 = indices[j + 1];
            int v3 = indices[j + 2];

            int a = getOrAddMidpoint(v1, v2, vertices, edgeMidpointCache, center, radius);
            int b = getOrAddMidpoint(v2, v3, vertices, edgeMidpointCache, center, radius);
            int c = getOrAddMidpoint(v3, v1, vertices, edgeMidpointCache, center, radius);

            newIndices.insert(newIndices.end(), {static_cast<unsigned int>(v1), static_cast<unsigned int>(a), static_cast<unsigned int>(c)});
            newIndices.insert(newIndices.end(), {static_cast<unsigned int>(v2), static_cast<unsigned int>(b), static_cast<unsigned int>(a)});
            newIndices.insert(newIndices.end(), {static_cast<unsigned int>(v3), static_cast<unsigned int>(c), static_cast<unsigned int>(b)});
            newIndices.insert(newIndices.end(), {static_cast<unsigned int>(a), static_cast<unsigned int>(b), static_cast<unsigned int>(c)});
        }

        indices = std::move(newIndices);
    }

    return {vertices, indices};
}

std::string SphereMeshRenderer::exportGeometryAsString() const {
    std::ostringstream oss;

    int vertexOffset = 0;

    auto generateCapsuleGeometry = [&](const glm::vec3& v0, const glm::vec3& v1, float r0, float r1) {
        const int SEGMENTS = 128;

        glm::vec3 d = v1 - v0;
        float dLength = glm::length(d);
        glm::vec3 dir = glm::normalize(d);

        float l = std::sqrt(glm::dot(d, d) - std::pow(r1 - r0, 2));

        float r0Bis = r0 * l / dLength;
        float r1Bis = r1 * l / dLength;

        float d0 = (r1 - r0) * r0 / dLength;
        float d1 = (r1 - r0) * r1 / dLength;

        glm::vec3 v0Bis = v0 + dir * d0;
        glm::vec3 v1Bis = v1 - dir * d1;

        glm::vec3 arbitrary = (glm::abs(dir.x) < 0.99) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::normalize(glm::cross(dir, arbitrary));
        glm::vec3 up = glm::normalize(glm::cross(right, dir));

        std::vector<glm::vec3> circlePoints1;
        std::vector<glm::vec3> circlePoints2;

        for (int i = 0; i < SEGMENTS; ++i) {
            float theta = (2.0f * M_PI * i) / SEGMENTS;

            glm::vec3 offset1 = (right * cos(theta) + up * sin(theta)) * r0Bis;
            glm::vec3 offset2 = (right * cos(theta) + up * sin(theta)) * r1Bis;

            circlePoints1.push_back(v0Bis + offset1);
            circlePoints2.push_back(v1Bis + offset2);
        }

        for (int i = 0; i < SEGMENTS; ++i) {
            int nextIndex = (i + 1) % SEGMENTS;

            glm::vec3 normal1 = glm::normalize(circlePoints1[i] - v0Bis);
            glm::vec3 normal2 = glm::normalize(circlePoints2[i] - v1Bis);
            glm::vec3 normal3 = glm::normalize(circlePoints1[nextIndex] - v0Bis);
            glm::vec3 normal4 = glm::normalize(circlePoints2[nextIndex] - v1Bis);

            oss << "v " << circlePoints1[i].x << " " << circlePoints1[i].y << " " << circlePoints1[i].z << "\n";
            oss << "vn " << normal1.x << " " << normal1.y << " " << normal1.z << "\n";

            oss << "v " << circlePoints2[i].x << " " << circlePoints2[i].y << " " << circlePoints2[i].z << "\n";
            oss << "vn " << normal2.x << " " << normal2.y << " " << normal2.z << "\n";

            oss << "v " << circlePoints1[nextIndex].x << " " << circlePoints1[nextIndex].y << " " << circlePoints1[nextIndex].z << "\n";
            oss << "vn " << normal3.x << " " << normal3.y << " " << normal3.z << "\n";

            oss << "f " << vertexOffset + 1 << "//" << vertexOffset + 1 << " "
                << vertexOffset + 2 << "//" << vertexOffset + 2 << " "
                << vertexOffset + 3 << "//" << vertexOffset + 3 << "\n";

            vertexOffset += 3;

            oss << "v " << circlePoints2[i].x << " " << circlePoints2[i].y << " " << circlePoints2[i].z << "\n";
            oss << "vn " << normal2.x << " " << normal2.y << " " << normal2.z << "\n";

            oss << "v " << circlePoints2[nextIndex].x << " " << circlePoints2[nextIndex].y << " " << circlePoints2[nextIndex].z << "\n";
            oss << "vn " << normal4.x << " " << normal4.y << " " << normal4.z << "\n";

            oss << "v " << circlePoints1[nextIndex].x << " " << circlePoints1[nextIndex].y << " " << circlePoints1[nextIndex].z << "\n";
            oss << "vn " << normal3.x << " " << normal3.y << " " << normal3.z << "\n";

            oss << "f " << vertexOffset + 1 << "//" << vertexOffset + 1 << " "
                << vertexOffset + 2 << "//" << vertexOffset + 2 << " "
                << vertexOffset + 3 << "//" << vertexOffset + 3 << "\n";

            vertexOffset += 3;
        }
    };

    for (size_t i = 0; i < sm->spheres.size(); ++i) {
        const auto& sphere = sm->spheres[i];
        auto [vertices, indices] = generateIcosphere(sphere.center, sphere.radius, 2);

        for (const auto& vertex : vertices) {
            oss << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
            oss << "vn " << glm::normalize(vertex - sphere.center).x << " "
                << glm::normalize(vertex - sphere.center).y << " "
                << glm::normalize(vertex - sphere.center).z << "\n";
        }

        for (size_t j = 0; j < indices.size(); j += 3) {
            oss << "f " << indices[j] + 1 + vertexOffset << "//" << indices[j] + 1 + vertexOffset << " "
                << indices[j + 1] + 1 + vertexOffset << "//" << indices[j + 1] + 1 + vertexOffset << " "
                << indices[j + 2] + 1 + vertexOffset << "//" << indices[j + 2] + 1 + vertexOffset << "\n";
        }
        vertexOffset += vertices.size();
    }

    for (const auto& prysmoid : sm->prysmoids) {
        glm::vec3 C1 = sm->spheres[prysmoid.indices[0]].center;
        glm::vec3 C2 = sm->spheres[prysmoid.indices[1]].center;
        glm::vec3 C3 = sm->spheres[prysmoid.indices[2]].center;

        float R1 = originalRadii[prysmoid.indices[0]] * connectivitySize;
        float R2 = originalRadii[prysmoid.indices[1]] * connectivitySize;
        float R3 = originalRadii[prysmoid.indices[2]] * connectivitySize;

        glm::vec3 N = computeUpperPlaneNormal(sm->spheres[prysmoid.indices[0]],
                                              sm->spheres[prysmoid.indices[1]],
                                              sm->spheres[prysmoid.indices[2]], 1);
        glm::vec3 NInv = computeUpperPlaneNormal(sm->spheres[prysmoid.indices[0]],
                                                 sm->spheres[prysmoid.indices[1]],
                                                 sm->spheres[prysmoid.indices[2]], -1);

        glm::vec3 V1_top = C1 + R1 * N;
        glm::vec3 V2_top = C2 + R2 * N;
        glm::vec3 V3_top = C3 + R3 * N;

        glm::vec3 V1_bottom = C1 + R1 * NInv;
        glm::vec3 V2_bottom = C2 + R2 * NInv;
        glm::vec3 V3_bottom = C3 + R3 * NInv;

        glm::vec3 normalTop = glm::normalize(glm::cross(V2_top - V1_top, V3_top - V1_top));
        glm::vec3 normalBottom = glm::normalize(glm::cross(V2_bottom - V3_bottom, V1_bottom - V3_bottom));

        oss << "v " << V1_top.x << " " << V1_top.y << " " << V1_top.z << "\n";
        oss << "vn " << normalTop.x << " " << normalTop.y << " " << normalTop.z << "\n";

        oss << "v " << V2_top.x << " " << V2_top.y << " " << V2_top.z << "\n";
        oss << "vn " << normalTop.x << " " << normalTop.y << " " << normalTop.z << "\n";

        oss << "v " << V3_top.x << " " << V3_top.y << " " << V3_top.z << "\n";
        oss << "vn " << normalTop.x << " " << normalTop.y << " " << normalTop.z << "\n";

        oss << "f " << vertexOffset + 1 << "//" << vertexOffset + 1 << " "
            << vertexOffset + 2 << "//" << vertexOffset + 2 << " "
            << vertexOffset + 3 << "//" << vertexOffset + 3 << "\n";

        vertexOffset += 3;

        oss << "v " << V3_bottom.x << " " << V3_bottom.y << " " << V3_bottom.z << "\n";
        oss << "vn " << normalBottom.x << " " << normalBottom.y << " " << normalBottom.z << "\n";

        oss << "v " << V2_bottom.x << " " << V2_bottom.y << " " << V2_bottom.z << "\n";
        oss << "vn " << normalBottom.x << " " << normalBottom.y << " " << normalBottom.z << "\n";

        oss << "v " << V1_bottom.x << " " << V1_bottom.y << " " << V1_bottom.z << "\n";
        oss << "vn " << normalBottom.x << " " << normalBottom.y << " " << normalBottom.z << "\n";

        oss << "f " << vertexOffset + 1 << "//" << vertexOffset + 1 << " "
            << vertexOffset + 2 << "//" << vertexOffset + 2 << " "
            << vertexOffset + 3 << "//" << vertexOffset + 3 << "\n";

        vertexOffset += 3;

        generateCapsuleGeometry(C1, C2, R1, R2);
        generateCapsuleGeometry(C2, C3, R2, R3);
        generateCapsuleGeometry(C3, C1, R3, R1);
    }

    for (const auto& capsule : sm->capsuloids) {
        glm::vec3 v0 = sm->spheres[capsule.indices[0]].center;
        glm::vec3 v1 = sm->spheres[capsule.indices[1]].center;

        float r0 = originalRadii[capsule.indices[0]] * connectivitySize;
        float r1 = originalRadii[capsule.indices[1]] * connectivitySize;

        generateCapsuleGeometry(v0, v1, r0, r1);
    }

    return oss.str();
}
