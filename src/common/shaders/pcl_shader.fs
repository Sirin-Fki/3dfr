#version 330 core
in vec4 fColor;

out vec4 FragColor;

void main()
{
    FragColor = vec4(fColor.b, fColor.g, fColor.r, 1.f);// set alle 4 vector values to 1.0

    //FragColor = vec4(1.0f,0.0f,0.0f,1.0f);
}