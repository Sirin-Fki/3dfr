#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in VS_OUT {
    vec4 color;
} gs_in[];
in vec3 inFragPos[];

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

    fColor = gs_in[0].color;
    outFragPos = inFragPos[0];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

    fColor = gs_in[1].color;
    outFragPos = inFragPos[1];
    gl_Position = gl_in[1].gl_Position; 
    EmitVertex();

    fColor = gs_in[2].color;    
    outFragPos = inFragPos[2];
    gl_Position = gl_in[2].gl_Position; 
    EmitVertex();
    
    EndPrimitive();
} 