#include "openglwidget.h"
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QDebug>
#include <QFile>
#include <QOpenGLDebugLogger>
#include <QKeyEvent>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

using namespace SM;

OpenGLWidget::OpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent), camera(), shader(nullptr), mesh(nullptr), sphereMesh(nullptr), smRenderer(nullptr)
{
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);
    setFocusPolicy(Qt::StrongFocus);
}

OpenGLWidget::~OpenGLWidget() {
    delete mesh;
    delete sphereMesh;
    delete smRenderer;
}

void OpenGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    makeCurrent();

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    shader = new Shader(":/shaders/vertex.vert", ":/shaders/fragment.frag");
    shader->bindAttribute("aPos", 0);
    shader->bindAttribute("aNormal", 1);
    shader->linkProgram();

    sphereMeshShader = new Shader(":/shaders/impostor.vert", ":/shaders/impostor.frag");
    sphereMeshShader->bindAttribute("aPos", 0);
    sphereMeshShader->linkProgram();

    mesh = new Mesh();
    mesh->loadFromFile("/Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/assets/foot.obj");
    mesh->color = glm::vec3(1.0f, 0.0f, 1.0f);
    mesh->isVisible = true;

    projectOnMesh = new Mesh();
    projectOnMesh->loadFromFile("/Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/assets/foot.obj");
    projectOnMesh->color = glm::vec3(0.5f, 0.5f, 0.5f);
    projectOnMesh->isWireframe = true;
    projectOnMesh->isVisible = false;

    sphereMesh = new SphereMesh();
    sphereMesh->loadFromFile("/Users/davidepaollilo/Workspaces/C++/SMToMeshFitter/assets/foot.sm");

    smRenderer = new SphereMeshRenderer(sphereMesh);
    smRenderer->useShader(sphereMeshShader);

    camera.setTarget(mesh->getCenter());
    setFocus();
}

void checkOpenGLError() {
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
        qDebug() << "OpenGL error: " << err;
}

void OpenGLWidget::paintGL() {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (smRenderer != nullptr) {
        sphereMeshShader->use();
        sphereMeshShader->setMat4("view", camera.getViewMatrix());
        sphereMeshShader->setMat4("projection", camera.getOrthographicMatrix(width(), height(), 0.0001f, 100000.0f));
        sphereMeshShader->setVec3("light.position", glm::vec3(-10.0f, 1.0f, 0.0f));
        sphereMeshShader->setVec3("light.ambient", glm::vec3(.5f, .5f, .5f));
        sphereMeshShader->setVec3("light.diffuse", glm::vec3(0.3f, 0.3f, 0.3f));
        sphereMeshShader->setVec3("light.specular", glm::vec3(0.3f, 0.3f, 0.3f));
        sphereMeshShader->setVec3("material.specular", glm::vec3(1.0f));
        sphereMeshShader->setFloat("material.shininess", 1);
        smRenderer->render();
    }

    if (mesh != nullptr) {
        shader->use();
        shader->setMat4("model", glm::mat4(1.0f));
        shader->setMat4("view", camera.getViewMatrix());
        shader->setMat4("projection", camera.getOrthographicMatrix(width(), height(), 0.0001f, 100000.0f));
        shader->setVec3("material.ambient", mesh->color);
        shader->setVec3("material.diffuse", glm::vec3(0.9f, 0.9f, 0.9f));
        shader->setVec3("material.specular", glm::vec3(1.0f));
        shader->setFloat("material.shininess", 1);

        shader->setVec3("light.position", glm::vec3(-1.f, 1.f, 0.f));
        shader->setVec3("light.ambient", glm::vec3(.5f, .5f, .5f));
        shader->setVec3("light.diffuse", glm::vec3(0.3f, 0.3f, 0.3f));
        shader->setVec3("light.specular", glm::vec3(0.3f, 0.3f, 0.3f));
        mesh->render(true);
    }

    if (projectOnMesh != nullptr) {
        shader->use();
        shader->setMat4("model", glm::mat4(1.0f));
        shader->setMat4("view", camera.getViewMatrix());
        shader->setMat4("projection", camera.getOrthographicMatrix(width(), height(), 0.0001f, 100000.0f));
        shader->setVec3("material.ambient", projectOnMesh->color);
        shader->setVec3("material.diffuse", glm::vec3(0.9f, 0.9f, 0.9f));
        shader->setVec3("material.specular", glm::vec3(1.0f));
        shader->setFloat("material.shininess", 1);

        shader->setVec3("light.position", glm::vec3(-1.f, 1.f, 0.f));
        shader->setVec3("light.ambient", glm::vec3(.5f, .5f, .5f));
        shader->setVec3("light.diffuse", glm::vec3(0.3f, 0.3f, 0.3f));
        shader->setVec3("light.specular", glm::vec3(0.3f, 0.3f, 0.3f));
        projectOnMesh->render(false);
        shader->setVec3("material.ambient", {0.0f, 0.0f, 0.0f});
        projectOnMesh->renderWireframe();
    }

    checkOpenGLError();
}

void OpenGLWidget::resizeGL(int w, int h) {
    if (h == 0) h = 1;

    glViewport(0, 0, w, h);
}

void OpenGLWidget::mousePressEvent(QMouseEvent *event) {
    QPoint currentPos = event->pos();
    lastMousePos = currentPos;

    if (event->button() == Qt::LeftButton && enableGeometricOperations) {
        if (!sphereMesh) return;

        Ray ray = rayOfPixel(currentPos.x(), currentPos.y());

        if (selectedSphere != -1)
            smRenderer->unselectSphere(selectedSphere);

        int i = sphereMesh->intersectedSphereAlongRay(ray.origin, ray.direction, posUnderMouse);

        if (selectedSphere == i && i > -1) {
            selectedSphere = -1;
            dragging = false;
        } else {
            if (selectedSphere != -1) {
                recentSelectedSpheres.push_back(selectedSphere);
                smRenderer->selectRecentSphere(selectedSphere);

                if (recentSelectedSpheres.size() > MAX_RECENT_SELECTIONS) {
                    int removedSphere = recentSelectedSpheres.front();
                    recentSelectedSpheres.pop_front();
                    smRenderer->unselectRecentSphere(removedSphere);
                }
            }

            selectedSphere = i;
            if (selectedSphere > -1) {
                smRenderer->selectSphere(selectedSphere);
                dragging = true;
            } else
                dragging = false;
        }

        initialMousePos = event->pos();
    }

    update();
}

void OpenGLWidget::mouseReleaseEvent(QMouseEvent *event) {
    dragging = false;
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent *event) {
    QPoint currentPos = event->pos();
    QPoint deltaPos = currentPos - lastMousePos;
    float xOffset = event->position().x() - lastMousePos.x();
    float yOffset = lastMousePos.y() - event->position().y();
    lastMousePos = currentPos;

    if (enableGeometricOperations)
    {
        if (event->buttons() & Qt::RightButton)
            scaleSelectedSphere(currentPos);

        if (dragging)
            translateSelectedSphere(currentPos);

        return;
    }

    if (event->buttons() & Qt::RightButton)
    {
        panCamera(deltaPos);
        return;
    }

    camera.rotateAroundY(xOffset * 0.2f);
    camera.rotateAroundX(-yOffset * 0.2f);

    update();
}

void OpenGLWidget::wheelEvent(QWheelEvent *event) {
    float delta = event->angleDelta().y() / 120.0f;

    if (delta > 0)
        camera.zoomIn(delta);
    else
        camera.zoomOut(-delta);
    update();
}

void OpenGLWidget::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
        case Qt::Key_R:
            camera.resetRotation();
            update();
            break;

        case Qt::Key_Z:
            camera.resetZoom();
            update();
            break;

        case Qt::Key_Y:
            camera.resetYRotation();
            update();
            break;

        case Qt::Key_X:
            camera.resetXRotation();
            update();
            break;

        case Qt::Key_Shift:
            enableGeometricOperations = true;
            break;

        case Qt::Key_B:
            sphereMeshShader = new Shader(":/shaders/impostor.vert", ":/shaders/impostor.frag");
            sphereMeshShader->bindAttribute("aPos", 0);
            sphereMeshShader->linkProgram();

            smRenderer = new SphereMeshRenderer(sphereMesh);
            smRenderer->useShader(sphereMeshShader);
            break;

        case Qt::Key_U:
            resetSelection();
            update();
            break;

        default:
            QOpenGLWidget::keyPressEvent(event);
            break;
    }
}

void OpenGLWidget::keyReleaseEvent(QKeyEvent* event) {
    switch (event->key()) {
        case Qt::Key_Shift:
            enableGeometricOperations = false;
            break;

        default:
            QOpenGLWidget::keyReleaseEvent(event);
            break;
    }
}

Ray OpenGLWidget::rayOfPixel(int _x, int _y) const
{
    GLint viewport[4] = {0, 0, width(), height()};

    float x = static_cast<float>(_x);
    float y = static_cast<float>(viewport[3] - _y);

    glm::vec4 viewportVec(viewport[0], viewport[1], viewport[2], viewport[3]);

    glm::mat4 projection = camera.getOrthographicMatrix(width(), height(), 0.0001f, 100000.0f);
    glm::mat4 view = camera.getViewMatrix();

    glm::vec3 screenPosNear(x, y, 0.0f);
    glm::vec3 screenPosFar(x, y, 1.0f);

    glm::vec3 rayOrigin = glm::unProject(screenPosNear, view, projection, viewportVec);
    glm::vec3 rayFar = glm::unProject(screenPosFar, view, projection, viewportVec);

    glm::vec3 rayDirection = glm::normalize(rayFar - rayOrigin);
    rayDirection = glm::normalize(rayDirection);

    return {rayOrigin, rayDirection};
}

void OpenGLWidget::scaleSelectedSphere(const QPoint& currentPos) {
    int deltaY = currentPos.y() - initialMousePos.y();
    float scaleAmount = 1.0f + deltaY * 0.01f;

    scaleAmount = glm::max(0.1f, scaleAmount);
    sphereMesh->spheres[selectedSphere].radius *= scaleAmount;
    sphereMesh->spheres[selectedSphere].radius = glm::max(0.1f, sphereMesh->spheres[selectedSphere].radius);

    initialMousePos = currentPos;
    update();
}

void OpenGLWidget::resetSelection()
{
    if (selectedSphere != -1) {
        smRenderer->unselectSphere(selectedSphere);
        selectedSphere = -1;
    }

    for (int i : recentSelectedSpheres)
        smRenderer->unselectRecentSphere(i);

    recentSelectedSpheres.clear();
}

void OpenGLWidget::duplicateSelectedSphere()
{
    if (selectedSphere == -1) return;

    sphereMesh->duplicateSphere(selectedSphere);
    smRenderer->updateSphereMesh(sphereMesh);
}

void OpenGLWidget::setMesh(Mesh* newMesh) {
    if (mesh != nullptr)
        delete mesh;

    mesh = newMesh;
    update();
}

void OpenGLWidget::setSphereMesh(SphereMesh* newSphereMesh) {
    if (sphereMesh != nullptr)
        delete sphereMesh;

    sphereMesh = newSphereMesh;

    if (smRenderer != nullptr)
        smRenderer->updateSphereMesh(sphereMesh);
    else
    {
        smRenderer = new SphereMeshRenderer(sphereMesh);
        smRenderer->useShader(sphereMeshShader);
    }

    update();
}

void OpenGLWidget::translateSelectedSphere(const QPoint& currentPos)
{
    GLint viewport[4] = {0, 0, width(), height()};

    glm::vec4 viewportVec(viewport[0], viewport[1], viewport[2], viewport[3]);
    glm::mat4 projection = camera.getOrthographicMatrix(width(), height(), 0.0001f, 100000.0f);
    glm::mat4 view = camera.getViewMatrix();

    glm::vec3 clickPos = glm::project(posUnderMouse, view, projection, viewportVec);
    glm::vec3 currMousePos = glm::vec3(currentPos.x(), height() - currentPos.y(), clickPos.z);
    glm::vec3 deltaWorld = glm::unProject(glm::vec3(currMousePos.x, currMousePos.y, currMousePos.z), view, projection, viewportVec)
                           - glm::unProject(glm::vec3(clickPos.x, clickPos.y, clickPos.z), view, projection, viewportVec);

    sphereMesh->spheres[selectedSphere].center += deltaWorld;
    posUnderMouse += deltaWorld;

    update();
}

void OpenGLWidget::panCamera(const QPoint& deltaPos) {
    float deltaX = static_cast<float>(deltaPos.x());
    float deltaY = static_cast<float>(-deltaPos.y());

    float movementScale = camera.getViewDim() * 0.005f;

    float horizontalDelta = deltaX * movementScale;
    if (deltaX > 0)
        camera.moveRight(-horizontalDelta);
    else if (deltaX < 0)
        camera.moveLeft(horizontalDelta);

    float verticalDelta = deltaY * movementScale;
    if (deltaY > 0)
        camera.moveUp(-verticalDelta);
    else if (deltaY < 0)
        camera.moveDown(verticalDelta);

    update();
}