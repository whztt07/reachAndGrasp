#version 150
//可以把颜色计算放在fragment shader里进行，计算量会大些，效果会好。
// vertex shader 算出的顶点值传入到fragment shader时会进行光栅化（再顶点间的像素的颜色根据顶点颜色进行线性插值），效果肯定不如在fragment shader里每个像素都算一遍来得好。
attribute vec4 vertex;
attribute vec3 normal;

varying vec4 colorVarying;
varying vec4 v_vertex;
varying vec3 v_norm;

uniform mat4 worldViewProj;


//uniform mat4 world;
//uniform mat4 worldIT;
//uniform mat4 worldViewProj;
//uniform vec4 lightPosition;
//uniform vec4 lightColour;
//uniform vec4 eyePosition;
//uniform samplerCube sampleEnvMap;

//uniform vec3 kd;

//lambert 漫反射光照模型 + C-T BDRF 镜面反射光照模型
void main()
{
    gl_Position = worldViewProj * vertex;//gl_Vertex;
    v_norm = normal;
    v_vertex = vertex;
    
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
}
