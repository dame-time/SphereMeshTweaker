#include "sphere_mesh_renderer.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

SphereMeshRenderer::SphereMeshRenderer(SphereMesh* referenceSM) : sm(referenceSM)
{
    for (int i = 0; i < sm->spheres.size(); i++)
        renderableSpheres.push_back({});
}

void SphereMeshRenderer::updateSphereMesh(SphereMesh* sm)
{
    this->sm = sm;
    renderableSpheres.clear();

    for (int i = 0; i < sm->spheres.size(); i++)
        renderableSpheres.push_back({});
}

void SphereMeshRenderer::useShader(Shader* shader)
{
    this->sphereShader = shader;
}

void SphereMeshRenderer::render() const
{
    if (!visible)
        return;

    renderSpheres();

    if (!filled)
        return;

    if (connectivitySize < 1.0f)
    {
        renderConnectivity();
        return;
    }

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
        drawSpheresOverPrysmoid(i);

    for (int i = 0; i < sm->capsuloids.size(); i++)
        drawSpheresOverCapsuloid(i);
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

