#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in VS_OUT {
    vec4 color;
    vec2 texCoords;
} gs_in[];
in vec3 inFragPos[];

uniform sampler2D texture1;

out vec4 fColor;
out vec3 outFragPos;
out vec3 Normal;


vec3 GetNormal()
{
   vec3 a = vec3(gl_in[0].gl_Position) - vec3(gl_in[1].gl_Position);
   vec3 b = vec3(gl_in[2].gl_Position) - vec3(gl_in[1].gl_Position);
   return normalize(cross(a, b));
}  

void main() {  
    Normal = GetNormal();

    fColor = texture(texture1, gs_in[0].texCoords);
    outFragPos = inFragPos[0];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

    fColor = texture(texture1, gs_in[1].texCoords);
    outFragPos = inFragPos[1];
    gl_Position = gl_in[1].gl_Position; 
    EmitVertex();

    fColor = texture(texture1, gs_in[2].texCoords);
    outFragPos = inFragPos[2];
    gl_Position = gl_in[2].gl_Position; 
    EmitVertex();
    
    EndPrimitive();
} 