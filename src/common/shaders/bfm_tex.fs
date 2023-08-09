#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
in vec4 fColor;
in vec3 FragPos;
in vec3 Normal;

uniform vec4 lightColor;
uniform vec3 lightPos2;
uniform vec3 lightPos;
uniform vec3 viewPos;

void main()
{    
    // ambient
    float ambientStrength = 0.2f;
    vec3 ambient = ambientStrength * vec3(lightColor);

    // diffuse 
    vec3 norm = normalize(Normal);

    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * vec3(lightColor);

    vec3 lightDir2 = normalize(lightPos2 - FragPos);
    float diff2 = max(dot(norm, lightDir2), 0.0);
    vec3 diffuse2 = diff2 * vec3(lightColor);
            
    //specular 
    float specularStrength = 0.3;
    vec3 viewDir = normalize(viewPos - FragPos);

    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 60.0f);
    vec3 specular = specularStrength * spec * vec3(lightColor);

    vec3 reflectDir2 = reflect(-lightDir2, norm);  
    float spec2 = pow(max(dot(viewDir, reflectDir2), 0.0), 32.0f);
    vec3 specular2 = specularStrength * spec2 * vec3(lightColor);

    vec4 result = vec4((ambient + diffuse + specular), 1.0f) * fColor;
    result += vec4((ambient + diffuse2 + specular2), 1.0f) * fColor;

    FragColor = vec4(result.r, result.g, result.b, 1.f);
}