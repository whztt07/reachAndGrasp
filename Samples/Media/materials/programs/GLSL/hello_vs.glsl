attribute vec4 vertex;
attribute vec3 normal;

varying  vec4 colorVarying;


uniform mat4 world;
uniform mat4 worldIT;
uniform mat4 worldViewProj;
uniform vec4 lightPosition;
uniform vec4 lightColour;
uniform vec4 eyePosition;

//uniform vec3 kd;

//lambert 漫反射光照模型 + phong 镜面反射光照模型
void main()
{
    gl_Position = worldViewProj * vertex;//gl_Vertex;

    //temp parameters
    vec3 kd = vec3(1.0, 0.0, 0.0);
    vec3 ks = vec3(20.0, 0.1, 0.2);
    float shininess = 50.0;
    vec3 globalAmbient = vec3(0.1, 0.1, 0.1);
    
    //world pos and norm
    vec4 worldPos = world * vertex;
    vec3 worldNorm = (worldIT * vec4(normal, 1.0)).xyz;
         worldNorm = normalize(worldNorm);
    
    //diffuse color
    //directional_light diffuse color
    vec3 lightDir = normalize(lightPosition.xyz - (worldPos.xyz * lightPosition.w));
    float nDotVP = max(dot(worldNorm, lightDir), 0.0);
    vec4 direct_diffuseColor = vec4(kd,1.0) * lightColour * nDotVP;
    //ambient_light  diffuse color
    vec4 ambient_diffuseColor = vec4(kd * globalAmbient, 1.0);
    
    vec4 diffuseColor = direct_diffuseColor + ambient_diffuseColor;
    
    //specular light
    vec3 V = normalize(eyePosition.xyz - (worldPos.xyz * eyePosition.w));
    vec3 R = 2.0 * max(dot(worldNorm, lightDir), 0.0) * worldNorm - lightDir;
         R = normalize(R);
    vec4 specularColor = vec4(ks, 1.0) * lightColour * pow(max(dot( V, R), 0.0), shininess);
    
    //final color
    colorVarying = diffuseColor + specularColor;
}

//
//uniform float4x4 modelViewProj
//void main()
//{
//    posOut.oPosition = mul(modelViewProj, posIn.position);
//    posOut.objectPos = posIn.position;
//    posOut.objectNormal = posIn.normal;
//}

//attribute vec4 position;
//attribute vec3 normal;
//
//varying vec4 colorVarying;
//
//uniform mat4 worldViewProj;
//uniform mat3 normalMatrix;
//
//void main()
//{
//    vec3 eyeNormal = normalize(normalMatrix * normal);
//    vec3 lightPosition = vec3(0.0, 0.0, 1.0);
//    vec4 diffuseColor = vec4(0.4, 0.4, 1.0, 1.0);
//    
//    float nDotVP = max(0.0, dot(eyeNormal, normalize(lightPosition)));
//    
//    colorVarying = diffuseColor * nDotVP;
//    
//    gl_Position = worldViewProj * position;
//}