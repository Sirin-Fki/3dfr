#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;
//layout (points) out;

in int gl_PrimitiveIDIn;
//out vec2 TexCoords;

uniform mat3 dep_intr;
uniform mat3 color_intr;
uniform mat4 dep_col_extr;
uniform int width;
uniform int height;
uniform sampler2D texture1;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

vec4 computeVertex(int u, int v) {
    float d = texelFetch(texture1, ivec2(u,v), 0).r;
    float depth_x = (u - dep_intr[0][2]) * d / dep_intr[0][0];
    float depth_y = (v - dep_intr[1][2]) * d / dep_intr[1][1];
    vec4 posDCam = vec4(depth_x, depth_y, d, 1.0f);
    vec4 posCCam;
    posCCam.x = dep_col_extr[0][0] * posDCam.x + dep_col_extr[0][1] * posDCam.y + dep_col_extr[0][2] * posDCam.z + dep_col_extr[0][3] * posDCam.w;
    posCCam.y = dep_col_extr[1][0] * posDCam.x + dep_col_extr[1][1] * posDCam.y + dep_col_extr[1][2] * posDCam.z + dep_col_extr[1][3] * posDCam.w;
    posCCam.z = dep_col_extr[2][0] * posDCam.x + dep_col_extr[2][1] * posDCam.y + dep_col_extr[2][2] * posDCam.z + dep_col_extr[2][3] * posDCam.w;
    posCCam.w = dep_col_extr[3][0] * posDCam.x + dep_col_extr[3][1] * posDCam.y + dep_col_extr[3][2] * posDCam.z + dep_col_extr[3][3] * posDCam.w;
    
    //posCCam /= posCCam.w;
    vec4 posClip;
    float xclip = (color_intr[0][0] * posCCam.x) / posCCam.z + color_intr[0][2];
    float yclip = (color_intr[1][1] * posCCam.y) / posCCam.z + color_intr[1][2];
    posClip.x = xclip * 2. / (width - 1.)- 1.;
    posClip.y = yclip * 2. / (height - 1.)-1.;
    posClip.z = -1 + (posCCam.z) * 2 ;
    posClip.w = 1.0f;

    return posClip;
}

void main()
{  
    int u = gl_PrimitiveIDIn % width;
    int v = gl_PrimitiveIDIn / width;
    if (u >= width - 1) return;
    if (v >= height - 1) return;
    float d0 = texelFetch(texture1, ivec2(u,v+1), 0).r;
    float d1 = texelFetch(texture1, ivec2(u,v), 0).r;
    float d2 = texelFetch(texture1, ivec2(u+1,v+1), 0).r;
    float d3 = texelFetch(texture1, ivec2(u+1,v), 0).r;
    if (d0 == 0 || d1 == 0 || d2 == 0 || d3 == 0 || d0 > 1.f || d1 > 1.f || d2 > 1.f || d3 > 1.f) return;
    
    float dmax = max(max(d0, d1), max(d2, d3));
	float dmin = min(min(d0, d1), min(d2, d3));

	float d = 0.5f*(dmax + dmin);
	if (dmax - dmin > 0.01 + 0.05f*d)	return;

    gl_Position =  computeVertex(u, v+1);
    EmitVertex();
    gl_Position =  computeVertex(u, v);
    EmitVertex();
    gl_Position =  computeVertex(u+1, v+1);
    EmitVertex();
    gl_Position =  computeVertex(u+1, v);
    EmitVertex();
    
    EndPrimitive();
    
}