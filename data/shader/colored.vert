#version 330 core
layout(location=0) in vec3 in_pos;
layout(location=1) in vec3 in_color;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out vec3 frag_color;

void main() {
    gl_Position = projection_matrix * view_matrix * model_matrix * vec4(in_pos, 1.0);
    frag_color = in_color;
}
