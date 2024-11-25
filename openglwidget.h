#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QOpenGLFramebufferObject>

#include <deque>

#include "camera.h"
#include "shader.h"
#include "mesh.h"
#include "sphere_mesh.h"
#include "sphere_mesh_renderer.h"
#include "ray.h"

using namespace SM;

class OpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    Mesh* mesh;
    Mesh* projectOnMesh;
    SphereMesh* sphereMesh;
    SphereMeshRenderer* smRenderer;

    explicit OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget() override;

    void setMesh(Mesh* newMesh);
    void setSphereMesh(SphereMesh* newSphereMesh);

    void duplicateSelectedSphere();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private:
    Camera camera;
    Shader* shader;
    Shader* sphereMeshShader;

    bool enableGeometricOperations = false;
    bool dragging = false;

    int selectedSphere = -1;
    glm::vec3 posUnderMouse;

    std::deque<int> recentSelectedSpheres;
    static constexpr int MAX_RECENT_SELECTIONS = 5;

    void panCamera(const QPoint& deltaPos);

    Ray rayOfPixel(int x, int y) const;

    void translateSelectedSphere(const QPoint& currentPos);
    void scaleSelectedSphere(const QPoint& currentPos);

    void loadSphereMesh(const std::string& filename);
    void saveSphereMesh(const std::string& filename);

    void resetSelection();

    QPoint initialMousePos;
    QPoint lastMousePos;

signals:
    void saveSphereMeshRequested();
    void loadSphereMeshRequested();
};
