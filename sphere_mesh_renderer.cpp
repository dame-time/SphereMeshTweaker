#include "sphere_mesh_renderer.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <functional>
#include <utility>

#define MIN_RADIUS 0.1f

struct hash_pair {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);

        return hash1 ^ (hash2 << 1);
    }
};

SphereMeshRenderer::SphereMeshRenderer(SphereMesh* referenceSM) : sm(referenceSM)
{
    updateSphereMesh(sm);
}

void SphereMeshRenderer::updateSphereMesh(SphereMesh* sm)
{
    this->sm = sm;

    renderableSpheres.clear();
    renderableSpheres.resize(sm->spheres.size());

    m_vertices.clear();
    m_indices.clear();
    m_subMeshes.clear();

    SubMesh prysSub;
    prysSub.indexOffset = m_indices.size();
    prysSub.color = glm::vec3(0.8f, 0.5f, 0.3f);

    for (int i = 0; i < sm->prysmoids.size(); i++)
        buildPrysmoidGeometry(i, prysSub.color);

    prysSub.indexCount = m_indices.size() - prysSub.indexOffset;
    m_subMeshes.push_back(prysSub);

    SubMesh quadSub;
    quadSub.indexOffset = m_indices.size();
    quadSub.color = glm::vec3(0.8f, 0.3f, 0.5f);

    for (int i = 0; i < sm->quadrilaterals.size(); i++)
        buildQuadGeometry(i, quadSub.color);

    quadSub.indexCount = m_indices.size() - quadSub.indexOffset;
    m_subMeshes.push_back(quadSub);

    SubMesh capsSub;
    capsSub.indexOffset = m_indices.size();
    capsSub.color = glm::vec3(0.0f, 0.0f, 0.75f);

    for (int i = 0; i < sm->capsuloids.size(); i++)
        buildCapsuloidGeometry(i, capsSub.color);

    capsSub.indexCount = m_indices.size() - capsSub.indexOffset;
    m_subMeshes.push_back(capsSub);

    uploadGeometryToGPU();
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

void SphereMeshRenderer::render()
{
    if (!visible)
        return;

    renderSpheres();

    if (!filled)
        return;

    m_VAO.bind();
    meshShader->use();

    for (auto &sub : m_subMeshes)
    {
        meshShader->setVec3("material.ambient",  sub.color);
        meshShader->setVec3("material.diffuse",  sub.color);
        meshShader->setVec3("material.specular", glm::vec3(0.1f, 0.1f, 0.1f));
        meshShader->setFloat("material.shininess", 32.0f);

        size_t offsetBytes = sub.indexOffset * sizeof(unsigned int);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(sub.indexCount),
                       GL_UNSIGNED_INT,
                       reinterpret_cast<void*>(offsetBytes));
    }

    m_VAO.release();
    glUseProgram(0);
}

void SphereMeshRenderer::selectSphere(int i)
{
    renderableSpheres[i].isSelected = true;
}

void SphereMeshRenderer::unselectSphere(int i)
{
    renderableSpheres[i].isSelected = false;
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
        renderSphere(sm->spheres[i].center, glm::mix(MIN_RADIUS, sm->spheres[i].radius, sphereSize), renderableSpheres[i].getColor());
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

        oss << "v " << v0.x << " " << v0.y << " " << v0.z << "\n";
        oss << "vn " << glm::normalize(v0 - v0).x << " " << glm::normalize(v0 - v0).y << " " << glm::normalize(v0 - v0).z << "\n";
        int v0CenterIndex = vertexOffset + 1;
        vertexOffset++;

        oss << "v " << v1.x << " " << v1.y << " " << v1.z << "\n";
        oss << "vn " << glm::normalize(v1 - v1).x << " " << glm::normalize(v1 - v1).y << " " << glm::normalize(v1 - v1).z << "\n";
        int v1CenterIndex = vertexOffset + 1;
        vertexOffset++;

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

        for (int i = 0; i < SEGMENTS; ++i) {
            int nextIndex = (i + 1) % SEGMENTS;

            oss << "f " << v0CenterIndex << "//" << v0CenterIndex << " "
                << vertexOffset + 1 + i << "//" << vertexOffset + 1 + i << " "
                << vertexOffset + 1 + nextIndex << "//" << vertexOffset + 1 + nextIndex << "\n";

            oss << "f " << v1CenterIndex << "//" << v1CenterIndex << " "
                << vertexOffset + SEGMENTS + i + 1 << "//" << vertexOffset + SEGMENTS + i + 1 << " "
                << vertexOffset + SEGMENTS + nextIndex + 1 << "//" << vertexOffset + SEGMENTS + nextIndex + 1 << "\n";
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

        float R1 = glm::mix(MIN_RADIUS, sm->spheres[prysmoid.indices[0]].radius, connectivitySize);
        float R2 = glm::mix(MIN_RADIUS, sm->spheres[prysmoid.indices[1]].radius, connectivitySize);
        float R3 = glm::mix(MIN_RADIUS, sm->spheres[prysmoid.indices[2]].radius, connectivitySize);

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

        float r0 = glm::mix(MIN_RADIUS, sm->spheres[capsule.indices[0]].radius, connectivitySize);
        float r1 = glm::mix(MIN_RADIUS, sm->spheres[capsule.indices[1]].radius, connectivitySize);

        generateCapsuleGeometry(v0, v1, r0, r1);
    }

    return oss.str();
}

void SphereMeshRenderer::appendTriangle(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3,
                                        const glm::vec3& n1, const glm::vec3& n2, const glm::vec3& n3)
{
    unsigned int startIndex = static_cast<unsigned int>(m_vertices.size());

    m_vertices.push_back(Vertex{p1, n1});
    m_vertices.push_back(Vertex{p2, n2});
    m_vertices.push_back(Vertex{p3, n3});

    m_indices.push_back(startIndex + 0);
    m_indices.push_back(startIndex + 1);
    m_indices.push_back(startIndex + 2);
}

void SphereMeshRenderer::buildPrysmoidGeometry(int index, const glm::vec3 &color)
{
    const auto& prysmoid = sm->prysmoids[index];

    const Sphere& s0 = sm->spheres[prysmoid.indices[0]];
    const Sphere& s1 = sm->spheres[prysmoid.indices[1]];
    const Sphere& s2 = sm->spheres[prysmoid.indices[2]];

    glm::vec3 C1 = s0.center;
    glm::vec3 C2 = s1.center;
    glm::vec3 C3 = s2.center;

    float R1 = glm::mix(MIN_RADIUS, s0.radius, connectivitySize);
    float R2 = glm::mix(MIN_RADIUS, s1.radius, connectivitySize);
    float R3 = glm::mix(MIN_RADIUS, s2.radius, connectivitySize);

    glm::vec3 nTop    = computeUpperPlaneNormal(s0, s1, s2,  1);
    glm::vec3 nBottom = computeUpperPlaneNormal(s0, s1, s2, -1);

    glm::vec3 V1_top = C1 + R1 * nTop;
    glm::vec3 V2_top = C2 + R2 * nTop;
    glm::vec3 V3_top = C3 + R3 * nTop;
    glm::vec3 V1_bottom = C1 + R1 * nBottom;
    glm::vec3 V2_bottom = C2 + R2 * nBottom;
    glm::vec3 V3_bottom = C3 + R3 * nBottom;

    appendTriangle(V1_top, V2_top, V3_top, nTop, nTop, nTop);
    appendTriangle(V1_bottom, V2_bottom, V3_bottom, nBottom, nBottom, nBottom);

    buildCapsuleBetweenSpheres(prysmoid.indices[0], prysmoid.indices[1], color);
    buildCapsuleBetweenSpheres(prysmoid.indices[1], prysmoid.indices[2], color);
    buildCapsuleBetweenSpheres(prysmoid.indices[2], prysmoid.indices[0], color);
}

void SphereMeshRenderer::buildQuadGeometry(int index, const glm::vec3 &color)
{
    const auto& quad = sm->quadrilaterals[index];

    const Sphere& s0 = sm->spheres[quad.indices[0]];
    const Sphere& s1 = sm->spheres[quad.indices[1]];
    const Sphere& s2 = sm->spheres[quad.indices[2]];
    const Sphere& s3 = sm->spheres[quad.indices[3]];

    glm::vec3 C1 = s0.center;
    glm::vec3 C2 = s1.center;
    glm::vec3 C3 = s2.center;
    glm::vec3 C4 = s3.center;

    float R1 = glm::mix(MIN_RADIUS, s0.radius, connectivitySize);
    float R2 = glm::mix(MIN_RADIUS, s1.radius, connectivitySize);
    float R3 = glm::mix(MIN_RADIUS, s2.radius, connectivitySize);
    float R4 = glm::mix(MIN_RADIUS, s3.radius, connectivitySize);

    glm::vec3 nTop    = computeUpperPlaneNormal(s0, s1, s2,  1);
    glm::vec3 nBottom = computeUpperPlaneNormal(s0, s1, s2, -1);

    glm::vec3 V1_top = C1 + R1 * nTop;
    glm::vec3 V2_top = C2 + R2 * nTop;
    glm::vec3 V3_top = C3 + R3 * nTop;
    glm::vec3 V4_top = C4 + R4 * nTop;

    glm::vec3 V1_bottom = C1 + R1 * nBottom;
    glm::vec3 V2_bottom = C2 + R2 * nBottom;
    glm::vec3 V3_bottom = C3 + R3 * nBottom;
    glm::vec3 V4_bottom = C4 + R4 * nBottom;

    appendTriangle(V1_top, V2_top, V3_top, nTop, nTop, nTop);
    appendTriangle(V3_top, V4_top, V1_top, nTop, nTop, nTop);

    appendTriangle(V1_bottom, V2_bottom, V3_bottom, nBottom, nBottom, nBottom);
    appendTriangle(V3_bottom, V4_bottom, V1_bottom, nBottom, nBottom, nBottom);

    buildCapsuleBetweenSpheres(quad.indices[0], quad.indices[1], color);
    buildCapsuleBetweenSpheres(quad.indices[1], quad.indices[2], color);
    buildCapsuleBetweenSpheres(quad.indices[2], quad.indices[3], color);
    buildCapsuleBetweenSpheres(quad.indices[3], quad.indices[0], color);
}

void SphereMeshRenderer::buildCapsuleBetweenSpheres(int sphereIndex1, int sphereIndex2,
                                                    const glm::vec3& color)
{
    const Sphere& s0 = sm->spheres[sphereIndex1];
    const Sphere& s1 = sm->spheres[sphereIndex2];

    glm::vec3 v0 = s0.center;
    glm::vec3 v1 = s1.center;
    float r0 = glm::mix(MIN_RADIUS, s0.radius, connectivitySize);
    float r1 = glm::mix(MIN_RADIUS, s1.radius, connectivitySize);

    if (r1 < r0) {
        std::swap(v0, v1);
        std::swap(r0, r1);
    }

    const int SEGMENTS = 32;
    glm::vec3 d = v0 - v1;
    float dLength = glm::length(d);

    float l = sqrt(glm::dot(d, d) - (r1 - r0)*(r1 - r0));

    float r0Bis = r0 * (l / dLength);
    float r1Bis = r1 * (l / dLength);

    float d0 = (r1 - r0)*(r0/dLength);
    float d1 = (r1 - r0)*(r1/dLength);

    glm::vec3 v0Bis = v0 + glm::normalize(d)*d0;
    glm::vec3 v1Bis = v1 + glm::normalize(d)*d1;

    glm::vec3 arbitrary = (fabs(d.x) < 0.99f) ? glm::vec3(1.0f, 0.0f, 0.0f)
                                              : glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(d, arbitrary));
    glm::vec3 up    = glm::normalize(glm::cross(right, d));

    std::vector<glm::vec3> circle1(SEGMENTS), circle2(SEGMENTS);
    std::vector<glm::vec3> normal1(SEGMENTS), normal2(SEGMENTS);

    for (int i = 0; i < SEGMENTS; ++i) {
        float theta = 2.0f * M_PI * float(i) / float(SEGMENTS);

        glm::vec3 offset1 = (right * cos(theta) + up * sin(theta)) * r0Bis;
        circle1[i] = v0Bis + offset1;
        normal1[i] = glm::normalize(offset1);

        glm::vec3 offset2 = (right * cos(theta) + up * sin(theta)) * r1Bis;
        circle2[i] = v1Bis + offset2;
        normal2[i] = glm::normalize(offset2);
    }

    for (int i = 0; i < SEGMENTS; ++i) {
        int next = (i + 1) % SEGMENTS;

        glm::vec3 p1 = circle1[i];
        glm::vec3 p2 = circle2[i];
        glm::vec3 p3 = circle1[next];
        glm::vec3 p4 = circle2[next];

        glm::vec3 n1 = normal1[i];
        glm::vec3 n2 = normal2[i];
        glm::vec3 n3 = normal1[next];
        glm::vec3 n4 = normal2[next];

        appendTriangle(p1, p2, p3, n1, n2, n3);
        appendTriangle(p2, p4, p3, n2, n4, n3);
    }
}

void SphereMeshRenderer::buildCapsuloidGeometry(int index, const glm::vec3 &color)
{
    const auto &caps = sm->capsuloids[index];

    buildCapsuleBetweenSpheres(caps.indices[0], caps.indices[1], color);
}

void SphereMeshRenderer::uploadGeometryToGPU()
{
    if (!m_VAO.isCreated()) {
        m_VAO.create();
    }
    m_VAO.bind();

    if (!m_VBO.isCreated()) {
        m_VBO.create();
    }
    m_VBO.bind();
    m_VBO.allocate(m_vertices.data(),
                   static_cast<int>(m_vertices.size() * sizeof(Vertex)));

    if (!m_EBO.isCreated()) {
        m_EBO.create();
    }
    m_EBO.bind();
    m_EBO.allocate(m_indices.data(),
                   static_cast<int>(m_indices.size() * sizeof(unsigned int)));

    QOpenGLFunctions f;
    f.initializeOpenGLFunctions();

    f.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                            reinterpret_cast<void*>(offsetof(Vertex, position)));
    f.glEnableVertexAttribArray(0);

    f.glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                            reinterpret_cast<void*>(offsetof(Vertex, normal)));
    f.glEnableVertexAttribArray(1);

    m_VAO.release();
    m_VBO.release();
    m_EBO.release();
}
