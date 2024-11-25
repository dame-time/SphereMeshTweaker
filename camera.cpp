#include "camera.h"

Camera::Camera() {
    ro = 50.0f;
    phi = 0.0f;
    theta = 0.0f;
    viewDim = 20.0f;
    target = glm::vec3(0.0f);
    upTranslation = 0.0f;
}

void Camera::setTarget(const glm::vec3& target) {
    this->target = target;
}

void Camera::rotateAroundY(float delta) {
    phi += delta;
}

void Camera::rotateAroundX(float delta) {
    theta += delta;
    theta = glm::clamp(theta, -90.0f, 90.0f);
}

void Camera::resetYRotation() {
    phi = 0.0f;
}

void Camera::resetXRotation() {
    theta = 0.0f;
}

void Camera::resetRotation() {
    phi = 0.0f;
    theta = 0.0f;
}

void Camera::translate(float delta) {
    ro += delta;
}

void Camera::resetTranslation() {
    ro = 0.0f;
}

void Camera::moveUp(float delta) {
    upTranslation -= delta;
}

void Camera::moveDown(float delta) {
    upTranslation += delta;
}

void Camera::moveLeft(float delta) {
    float phiRadians = glm::radians(phi);
    glm::vec3 left = glm::vec3(-cos(phiRadians), 0.0f, sin(phiRadians));
    target += left * delta;
}

void Camera::moveRight(float delta) {
    float phiRadians = glm::radians(phi);
    glm::vec3 right = glm::vec3(cos(phiRadians), 0.0f, -sin(phiRadians));
    target += right * delta;
}

void Camera::zoomIn(float delta) {
    viewDim /= (1.0f + delta * 0.05f);
    viewDim = glm::clamp(viewDim, 0.01f, 100000.0f);
}

void Camera::zoomOut(float delta) {
    viewDim *= (1.0f + delta * 0.05f);
    viewDim = glm::clamp(viewDim, 0.01f, 100000.0f);
}

void Camera::resetZoom() {
    viewDim = 20.0f;
}

void Camera::resetScale() {
    viewDim = 1.0f;
}

glm::mat4 Camera::getViewMatrix() const
{
    float phiRadians = glm::radians(phi);
    float thetaRadians = glm::radians(theta);

    glm::mat4 originTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, upTranslation, -ro));
    glm::mat4 xRotation = glm::rotate(glm::mat4(1.0f), thetaRadians, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat4 yRotation = glm::rotate(glm::mat4(1.0f), phiRadians, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 targetTranslation = glm::translate(glm::mat4(1.0f), -target);

    return targetTranslation * originTranslation * xRotation * yRotation;
}

glm::mat4 Camera::getPerspectiveMatrix(float fov, float aspect, float near, float far) {
    return glm::perspective(glm::radians(fov), aspect, near, far);
}

glm::mat4 Camera::getOrthographicMatrix(float width, float height, float near, float far) const {
    float a = std::sqrt(height / width);

    return glm::ortho(-viewDim / a, +viewDim / a, -viewDim * a, +viewDim * a, near, far);
}

glm::vec3 Camera::getForwardVector() const {
    float phiRadians = glm::radians(phi);
    float thetaRadians = glm::radians(theta);

    glm::vec3 forward;
    forward.x = sin(phiRadians) * cos(thetaRadians);
    forward.y = sin(thetaRadians);
    forward.z = cos(phiRadians) * cos(thetaRadians);

    return glm::normalize(forward);
}

glm::vec3 Camera::getRightVector() const {
    glm::vec3 forward = getForwardVector();
    glm::vec3 worldUp = glm::vec3(0.0f, 1.0f, 0.0f);
    return glm::normalize(glm::cross(forward, worldUp));
}

glm::vec3 Camera::getUpVector() const {
    glm::vec3 right = getRightVector();
    glm::vec3 forward = getForwardVector();
    return glm::normalize(glm::cross(right, forward));
}

void Camera::moveTarget(const glm::vec3& delta) {
    target += delta;
}

glm::vec3 Camera::getPosition() const {
    float phiRadians = glm::radians(phi);
    float thetaRadians = glm::radians(theta);

    glm::vec3 position;
    position.x = ro * sin(phiRadians) * cos(thetaRadians);
    position.y = ro * sin(thetaRadians);
    position.z = ro * cos(phiRadians) * cos(thetaRadians);

    position += glm::vec3(0.0f, upTranslation, 0.0f) + target;

    return position;
}

