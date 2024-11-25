#pragma once
#include <QString>
#include <QOpenGLShaderProgram>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>

class Shader
{
public:
    QString vertexShaderPath, fragmentShaderPath;

    QOpenGLShaderProgram* program;

    Shader();
    Shader(const QString& vertexPath, const QString& fragmentPath);

    void initializeFromPaths(const QString& vertexPath, const QString& fragmentPath);
    Shader* newShaderAtPath(const QString& vertexPath, const QString& fragmentPath);

    void use() const;

    void bindAttribute(const std::string& name, unsigned int location);
    void linkProgram();

    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec2(const std::string &name, const glm::vec2& value) const;
    void setVec2(const std::string &name, float x, float y) const;
    void setVec3(const std::string &name, const glm::vec3& value) const;
    void setVec3(const std::string &name, float x, float y, float z) const;
    void setVec4(const std::string &name, const glm::vec4& value) const;
    void setVec4(const std::string &name, float x, float y, float z, float w) const;
    void setMat2(const std::string &name, const glm::mat2& mat) const;
    void setMat3(const std::string &name, const glm::mat3& mat) const;
    void setMat4(const std::string &name, const glm::mat4& mat) const;

private:
    void checkCompileErrors(QOpenGLShaderProgram* program, const QString& type);
};
