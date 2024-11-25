#version 120

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

struct Light {
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

varying vec2 TexCoords;
varying vec4 worldPos;
varying vec3 ViewDir;
varying float radiusClip;

uniform Material material;
uniform Light light;
uniform vec3 sphereCenter;

uniform mat4 view;
uniform mat4 projection;
uniform float radius;

void main()
{
    vec2 pos = TexCoords;

    // Calculate the squared distance from the center of the sphere
    float distSqr = dot(pos, pos);

    if (distSqr > 1.0)
        discard;  // Discard fragments outside the sphere

    float normalZ = sqrt(1.0 - distSqr);
    vec3 normal = normalize(vec3(pos, normalZ));

    // Calculate light direction (constant light direction can also be used here)
    vec3 lightDir = normalize(light.position - sphereCenter);

    // Ambient component
    vec3 ambient = light.ambient * material.ambient;

    // Diffuse component
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * material.diffuse;

    // Specular component
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(ViewDir, reflectDir), 0.0), 1);
    vec3 specular = light.specular * spec * material.specular;

    vec3 result = ambient + diffuse + specular;

    gl_FragColor = vec4(result, 1.0);
    gl_FragDepth = gl_FragCoord.z + normalZ * radiusClip;
}
