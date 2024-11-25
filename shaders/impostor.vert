#version 120

attribute vec2 aPos;

uniform mat4 view;
uniform mat4 projection;
uniform vec3 center;
uniform float radius;

varying vec2 TexCoords;
varying vec4 worldPos;
varying vec3 ViewDir;
varying float radiusClip;

void main()
{
    TexCoords = aPos;

    worldPos = view * vec4(center, 1.0) + vec4(aPos * radius, 0.0, 1.0);

    radiusClip = radius * projection[2][2] * 0.5;

    ViewDir = normalize(vec3(view[0][2], view[1][2], view[2][2]));

    gl_Position = projection * vec4(worldPos.xyz, 1.0);
    gl_Position.w = 1.0;
}
