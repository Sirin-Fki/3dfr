#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
layout (location = 3) in vec4 aColor;

out VS_OUT {
    vec4 color;
	vec2 texCoords;
} vs_out;
out vec3 FragPos;  

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    vs_out.color = aColor;
	vs_out.texCoords = aTexCoords;
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    FragPos = vec3(model * vec4(aPos, 1.0));
}