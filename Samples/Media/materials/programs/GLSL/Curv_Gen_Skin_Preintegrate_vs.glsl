#version 120
#extension GL_ARB_shader_texture_lod : require

//可以把颜色计算放在fragment shader里进行，计算量会大些，效果会好。
// vertex shader 算出的顶点值传入到fragment shader时会进行光栅化（再顶点间的像素的颜色根据顶点颜色进行线性插值），效果肯定不如在fragment shader里每个像素都算一遍来得好。
attribute vec4 vertex;
attribute vec3 normal;
attribute vec3 tangent;


varying vec4 colorVarying;
varying vec4 v_vertex;
varying vec3 v_norm;
varying vec2 v_texCoord;
varying vec3 v_tangent;
varying vec3 v_WS_tangent;
varying vec3 v_WS_normal;
varying vec4 v_WS_vertex;

varying vec3 v_TSLightDir;
varying vec3 v_TSViewDir;
varying vec3 v_TSHalfAngle;
varying vec3 v_TSPos;
varying vec3 v_TSNormal;

uniform mat4 worldViewProj;
uniform sampler2D normalMap;


//uniform samplerCube prefilteredSampleEnvMap;

uniform mat4 world;
uniform mat4 worldIT;
//uniform mat4 worldViewProj;
uniform vec4 lightPosition;
//uniform vec4 lightColour;
uniform vec4 eyePosition;
//uniform samplerCube sampleEnvMap;

//uniform vec3 kd;

mat3 transpose(mat3 m){
    return mat3(
            m[0][0],m[1][0],m[2][0],
            m[0][1],m[1][1],m[2][1],
            m[0][2],m[1][2],m[2][2]
            
            );
}

#define NORMAL_MAP

void main()
{
#ifdef NORMAL_MAP
   //  gl_Position = worldViewProj * vertex;//gl_Vertex;
   //  v_texCoord = gl_MultiTexCoord0.xy;

    
   //  mat3 worldRotMatrix = mat3(world[0].xyz, world[1].xyz, world[2].xyz);
   //  vec3 N = normalize(normal * worldRotMatrix );
   //  vec3 T = normalize(tangent.xyz * worldRotMatrix );
   //  vec3 binormal = normalize(cross(N, T)*tangent.w);  
   //  mat3 rotation = transpose(mat3(T, binormal, N));


   //  vec4 P = vertex * world;
   //  vec3 lightDir = normalize(lightPosition.xyz - (P * lightPosition.w).xyz);
   //  vec3 viewDir = normalize(eyePosition.xyz - (P * eyePosition.w).xyz);

   //  v_TSLightDir = normalize(rotation * lightDir);
   //  v_TSViewDir = normalize(rotation * viewDir);

   // colorVarying = vec4(v_texCoord, 0.0, 1.0);
   // colorVarying = vec4(v_TSViewDir, 1.0);

   gl_Position = worldViewProj * vertex;//gl_Vertex;
   v_texCoord = gl_MultiTexCoord0.xy;
   v_vertex = vertex;


   v_WS_vertex = world * vertex;
   v_WS_normal = (worldIT * vec4(normal, 1.0)).xyz;
   v_WS_tangent = (world * vec4(tangent, 1.0)).xyz; //?

   vec3 lightDir = normalize(lightPosition.xyz - (vertex * lightPosition.w).xyz);
   vec3 cameraDir = normalize(eyePosition.xyz - vertex.xyz);

   vec3 binormal = cross(normal, tangent);

   mat3 rotation = mat3(vec3(tangent[0], binormal[0], normal[0]),
                        vec3(tangent[1], binormal[1], normal[1]),
                        vec3(tangent[2], binormal[2], normal[2]));
   
   //tangent space
   v_TSLightDir = rotation * lightDir;
   v_TSViewDir = rotation * cameraDir;

   vec3 eyeDir = normalize(eyePosition.xyz - vertex.xyz);
   vec3 halfAngle = normalize(eyeDir + lightDir);
   v_TSHalfAngle = rotation * halfAngle;
   v_TSPos = rotation * vertex.xyz;  

   v_TSNormal = rotation * normal;



   //gl_Position = gl_MultiTexCoord0.xy;



#else
    gl_Position = worldViewProj * vertex;//gl_Vertex;
    v_norm = normal;
    v_vertex = vertex;
    v_texCoord = gl_MultiTexCoord0.xy;
    v_WS_vertex = world * vertex;
    v_WS_normal = (worldIT * vec4(normal, 1.0)).xyz;
    v_WS_tangent = (worldIT * vec4(tangent, 1.0)).xyz;
#endif
    /*
     //temp parameters
     vec3 kd = vec3(1.0, 0.0, 0.0);
     vec3 ks = vec3(1.0, 0.1, 0.2);
     float shininess = 30.0;
     vec3 globalAmbient = vec3(0.1, 0.1, 0.1);
     float m = 0.2; // C-T, 表面的粗糙程度
     float f = 1.0; // C-T, 入射角度接近 0(入射方向靠近法向量)时的 Fresnel 反射系数
     
     //world pos and norm
     vec4 worldPos = world * vertex;
     vec3 worldNorm = (worldIT * vec4(normal, 1.0)).xyz;
     worldNorm = normalize(worldNorm);
     
     //diffuse color
     ////directional_light diffuse color
     vec3 lightDir = normalize(lightPosition.xyz - (worldPos.xyz * lightPosition.w));
     float nl = max(dot(worldNorm, lightDir), 0.0);
     vec3 direct_diffuseColor = kd * lightColour.rgb * nl;
     ////ambient_light  diffuse color
     vec3 ambient_diffuseColor = kd * globalAmbient;
     
     vec3 diffuseColor = direct_diffuseColor + ambient_diffuseColor;
     
     
     //specular light
     
     vec3 V = normalize(eyePosition.xyz - (worldPos.xyz * eyePosition.w));
     vec3 N = worldNorm; vec3 L = lightDir;
     vec3 H = normalize(L + V);
     vec3 specularColor = vec3(0.0,0.0,0.0);
     vec3 R = vec3(0.0,0.0,0.0);
     
     ////C-T
     if(true)
     {
     float nv = dot(N,V);
     bool back = (nv>0.0) && (nl>0.0);
     if(back)
     {
     float nh = dot(N,H);
     float temp = (nh*nh-1.0)/(m*m*nh*nh);
     float roughness = (exp(temp))/(pow(m,2.0)*pow(nh,4.0)); //粗糙度, 根据 beckmann 函数
     float vh = dot(V,H);
     float a = (2.0*nh*nv)/vh;
     float b = (2.0*nh*nl)/vh;
     float geometric = min(a,b);
     geometric = min(1.0,geometric); //几何衰减系数
     float fresnelCoe=f+(1.0-f)*pow(1.0-vh,5.0); //fresnel反射系数
     float rs = (fresnelCoe*geometric*roughness)/(nv*nl);
     specularColor = rs * lightColour.rgb * nl*ks; // 计算镜面反射光分量(这是重点)
     }
     }
     ////phong
     else
     {
     R = 2.0 * max(dot(N, L), 0.0) * N - L;
     R = normalize(R);
     specularColor = ks * lightColour.rgb * pow(max(dot( V, R), 0.0), shininess);
     }
     
     //environment mapping //环境贴图
     //R = reflect(V, N);
     
     // Ogre conversion for cube map lookup
     //R.z = -R.z;
     
     //vec4 envColor = textureCube(sampleEnvMap, R);
     
     
     //final color
     //colorVarying = envColor;// + 0.25 * vec4(diffuseColor + specularColor, 1.0);
     */



// curvature map

  v_TSNormal  = texture2D(normalMap, v_texCoord).rgb;

  gl_Position = vec4(v_texCoord.x*10, v_texCoord.y*10, 0.0, 1.0);//worldViewProj * vertex;//gl_Vertex;


}
