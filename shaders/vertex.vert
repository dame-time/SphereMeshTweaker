#version 120

attribute vec3 aPos;
attribute vec3 aNormal;

varying vec3 Normal;
varying vec3 ViewDir;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    vec4 worldPosition = model * vec4(aPos, 1.0);

    ViewDir = normalize(vec3(view[0][2], view[1][2], view[2][2]));
    Normal = normalize(mat3(model) * aNormal);

    gl_Position = projection * view * worldPosition;
}
