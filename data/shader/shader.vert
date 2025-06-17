#version 330 core
layout(location=0) in vec3 inPos;
layout(location=1) in vec2 inUV;
out vec2 fragUV;
uniform mat4 model, view, projection;
void main() {
    fragUV = inUV;
    gl_Position = projection * view * model * vec4(inPos,1.0);
}
