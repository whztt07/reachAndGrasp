#version 120
#extension GL_ARB_shader_texture_lod : require

attribute vec4 vertex;
attribute vec3 normal;
attribute vec3 tangent;


//varying vec4 v_vertex;
// varying vec3 v_norm;
varying vec2 v_texCoord;
// varying vec3 v_tangent;
// varying vec3 v_WS_tangent;
// varying vec3 v_WS_normal;
// varying vec4 v_WS_vertex;

varying vec3 v_TSLightDir;
varying vec3 v_TSViewDir;
varying vec3 v_TSHalfAngle;
// varying vec3 v_TSPos;
 varying vec3 v_TSNormal;

uniform mat4 worldViewProj;
//uniform samplerCube prefilteredSampleEnvMap;
// uniform mat4 world;
// uniform mat4 worldIT;
//uniform mat4 worldViewProj;
uniform vec4 lightPosition;
//uniform vec4 lightColour;
uniform vec4 eyePosition;
//uniform samplerCube sampleEnvMap;


mat3 transpose(mat3 m){
    return mat3(
            m[0][0],m[1][0],m[2][0],
            m[0][1],m[1][1],m[2][1],
            m[0][2],m[1][2],m[2][2]
            );
}

//#define SPHERE_TESTING

void main()
{
   gl_Position = worldViewProj * vertex;//gl_Vertex;
   v_texCoord = gl_MultiTexCoord0.xy;
   //v_vertex = vertex;

   vec3 lightDir = normalize(lightPosition.xyz - (vertex * lightPosition.w).xyz);
   vec3 cameraDir = normalize(eyePosition.xyz - vertex.xyz);
   vec3 halfAngle = normalize(cameraDir + lightDir);

   vec3 binormal = cross(normal, tangent);

   mat3 rotation = mat3(vec3(tangent[0], binormal[0], normal[0]),
                        vec3(tangent[1], binormal[1], normal[1]),
                        vec3(tangent[2], binormal[2], normal[2]));
   
   //tangent space
   v_TSLightDir = rotation * lightDir;
   v_TSViewDir = rotation * cameraDir;
   v_TSHalfAngle = rotation * halfAngle;

#ifdef SPHERE_TESTING 
   v_TSLightDir = lightDir;
   v_TSViewDir =  cameraDir;
   v_TSHalfAngle =  halfAngle;
   v_TSNormal = normal;
#endif
   


}
