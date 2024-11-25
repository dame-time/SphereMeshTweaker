#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>

class Camera {
public:
    Camera();

    void setTarget(const glm::vec3& target);
    void rotateAroundY(float delta);
    void rotateAroundX(float delta);
    void resetYRotation();
    void resetXRotation();
    void resetRotation();
    void translate(float delta);
    void resetTranslation();
    void zoomIn(float delta);
    void zoomOut(float delta);
    void resetZoom();
    void moveUp(float delta);
    void moveDown(float delta);
    void moveLeft(float delta);
    void moveRight(float delta);
    void resetScale();

    float getViewDim() { return viewDim; }
    glm::vec3 getForwardVector() const;
    glm::vec3 getRightVector() const;
    glm::vec3 getUpVector() const;
    void moveTarget(const glm::vec3& delta);

    glm::mat4 getViewMatrix() const;
    glm::mat4 getPerspectiveMatrix(float fov, float aspect, float near, float far);
    glm::mat4 getOrthographicMatrix(float width, float height, float near, float far) const;

    glm::vec3 getPosition() const;

private:
    float ro;
    float phi;
    float theta;
    float viewDim;
    float upTranslation = 0.0f;

    glm::vec3 target;
};
