#include "shader.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <QMatrix2x2>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <glm/gtc/type_ptr.hpp>

Shader::Shader() : program(nullptr) {}

Shader::Shader(const QString& vertexPath, const QString& fragmentPath)
{
    initializeFromPaths(vertexPath, fragmentPath);
}

void Shader::initializeFromPaths(const QString& vertexPath, const QString& fragmentPath)
{
    vertexShaderPath = vertexPath;
    fragmentShaderPath = fragmentPath;

    program = new QOpenGLShaderProgram();

    program->addShaderFromSourceFile(QOpenGLShader::Vertex, vertexShaderPath);
    checkCompileErrors(program, "VERTEX");

    program->addShaderFromSourceFile(QOpenGLShader::Fragment, fragmentShaderPath);
    checkCompileErrors(program, "FRAGMENT");
}

void Shader::bindAttribute(const std::string& name, unsigned int location)
{
    if (program)
        program->bindAttributeLocation(name.c_str(), location);
}

void Shader::linkProgram()
{
    if (!program->link())
        checkCompileErrors(program, "PROGRAM");
}

Shader* Shader::newShaderAtPath(const QString& vertexPath, const QString& fragmentPath)
{
    return new Shader(vertexPath, fragmentPath);
}

void Shader::use() const
{
    if (program)
        program->bind();
}

void Shader::setBool(const std::string &name, bool value) const
{
    if (program)
        program->setUniformValue(name.c_str(), static_cast<int>(value));
}

void Shader::setInt(const std::string &name, int value) const
{
    if (program)
        program->setUniformValue(name.c_str(), value);
}

void Shader::setFloat(const std::string &name, float value) const
{
    if (program)
        program->setUniformValue(name.c_str(), value);
}

void Shader::setVec2(const std::string &name, const glm::vec2& value) const
{
    if (program)
        program->setUniformValue(name.c_str(), QVector2D(value.x, value.y));
}

void Shader::setVec2(const std::string &name, float x, float y) const
{
    if (program)
        program->setUniformValue(name.c_str(), QVector2D(x, y));
}

void Shader::setVec3(const std::string &name, const glm::vec3& value) const
{
    if (program)
        program->setUniformValue(name.c_str(), QVector3D(value.x, value.y, value.z));
}

void Shader::setVec3(const std::string &name, float x, float y, float z) const
{
    if (program)
        program->setUniformValue(name.c_str(), QVector3D(x, y, z));
}

void Shader::setVec4(const std::string &name, const glm::vec4& value) const
{
    if (program)
        program->setUniformValue(name.c_str(), QVector4D(value.x, value.y, value.z, value.w));
}

void Shader::setVec4(const std::string &name, float x, float y, float z, float w) const
{
    if (program)
        program->setUniformValue(name.c_str(), QVector4D(x, y, z, w));
}

void Shader::setMat2(const std::string &name, const glm::mat2& mat) const
{
    if (!program)
        return;

    QMatrix2x2 qmat;
    const float* matPtr = glm::value_ptr(mat);
    qmat(0, 0) = matPtr[0]; qmat(0, 1) = matPtr[1];
    qmat(1, 0) = matPtr[2]; qmat(1, 1) = matPtr[3];
    program->setUniformValue(name.c_str(), qmat.transposed());
}

void Shader::setMat3(const std::string &name, const glm::mat3& mat) const
{
    if (!program)
        return;

    QMatrix3x3 qmat;
    const float* matPtr = glm::value_ptr(mat);
    qmat(0, 0) = matPtr[0]; qmat(0, 1) = matPtr[1]; qmat(0, 2) = matPtr[2];
    qmat(1, 0) = matPtr[3]; qmat(1, 1) = matPtr[4]; qmat(1, 2) = matPtr[5];
    qmat(2, 0) = matPtr[6]; qmat(2, 1) = matPtr[7]; qmat(2, 2) = matPtr[8];
    program->setUniformValue(name.c_str(), qmat.transposed());
}

void Shader::setMat4(const std::string &name, const glm::mat4& mat) const
{
    if (!program)
        return;

    QMatrix4x4 qmat;
    const float* matPtr = glm::value_ptr(mat);
    qmat(0, 0) = matPtr[0];  qmat(0, 1) = matPtr[4];  qmat(0, 2) = matPtr[8];   qmat(0, 3) = matPtr[12];
    qmat(1, 0) = matPtr[1];  qmat(1, 1) = matPtr[5];  qmat(1, 2) = matPtr[9];   qmat(1, 3) = matPtr[13];
    qmat(2, 0) = matPtr[2];  qmat(2, 1) = matPtr[6];  qmat(2, 2) = matPtr[10];  qmat(2, 3) = matPtr[14];
    qmat(3, 0) = matPtr[3];  qmat(3, 1) = matPtr[7];  qmat(3, 2) = matPtr[11];  qmat(3, 3) = matPtr[15];
    program->setUniformValue(name.c_str(), qmat);
}

void Shader::checkCompileErrors(QOpenGLShaderProgram* program, const QString& type)
{
    if (type != "PROGRAM")
    {
        QList<QOpenGLShader *> shaders = program->shaders();
        QOpenGLShader *shader = nullptr;

        if (type == "VERTEX" && !shaders.isEmpty())
            shader = shaders.first();
        else if (type == "FRAGMENT" && shaders.size() > 1)
            shader = shaders.last();

        if (shader && !shader->isCompiled())
        {
            qDebug() << "ERROR::SHADER_COMPILATION_ERROR of type:" << type;
            qDebug() << shader->log();
        }
        else if (shader)
        {
            qDebug() << type << " shader compiled successfully.";
        }
    }
    else
    {
        if (!program->isLinked())
        {
            qDebug() << "ERROR::PROGRAM_LINKING_ERROR:";
            qDebug() << program->log();
        }
        else
        {
            qDebug() << "Shader program linked successfully.";
        }
    }
}
