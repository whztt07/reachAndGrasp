#version 120
#extension GL_ARB_shader_texture_lod : require

//varying  vec4 colorVarying;
varying vec4 v_vertex;
varying vec3 v_norm;
//varying vec3 prefilteredEnvFun;

uniform mat4 world;
uniform mat4 worldIT;
uniform vec4 lightPosition;
uniform vec4 lightColour;
uniform vec4 eyePosition;
uniform sampler2D hammersleyMap;
uniform samplerCube sampleEnvMap;
uniform samplerCube prefilteredSampleEnvMap;
uniform float u_roughness;
uniform float u_MipUpBound;

const int NumSamplesConst = 1024;//1024;


vec3 PrefilteredDiffuseIBL( vec3 DiffuseColor, vec3 N )
{    
    vec3 PrefilteredColor = textureCubeLod( prefilteredSampleEnvMap, N, u_MipUpBound).rgb;
    return DiffuseColor * PrefilteredColor;
}


vec3 EnvBRDFApprox( vec3 SpecularColor, float Roughness, float NoV )
{
    const vec4 c0 = vec4( -1.0, -0.0275, -0.572, 0.022 );
    const vec4 c1 = vec4(  1.0,  0.0425,  1.04,  -0.04 );
    vec4 r = Roughness * c0 + c1;
    float a004 = min( r.x * r.x, exp2( -9.28 * NoV ) ) * r.x + r.y;
    vec2 AB = vec2( -1.04, 1.04 ) * a004 + r.zw;
    return SpecularColor * AB.x + AB.y;
}


vec3 PrefilteredMobileApproximateSpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
{
    float NoV =  dot( N, V ) ;
    vec3 R = 2.0 * dot( V, N ) * N - V;
    R= normalize(R);
    
    float Mip = Roughness * ( u_MipUpBound - 1 );
    vec3 PrefilteredColor = //prefilteredEnvFun;
    textureCubeLod( prefilteredSampleEnvMap, R, Mip ).rgb;
    vec3 EnvBRDF =  EnvBRDFApprox( SpecularColor, Roughness, NoV );
    return PrefilteredColor * EnvBRDF;
}


// //-----------------------------------------------------------------------
// // Point Light Cook-Torrence Shading Model (Mobile Version)
// //-----------------------------------------------------------------------

// float D_GGX(float alfa, float NoH)
// {
//     //Specular D: GGX 函数
//     float temp = NoH * NoH * (alfa * alfa - 1.0) + 1.0;
//     return alfa * alfa / (temp * temp);
// }

// vec3 SpecularDirectional( vec3 ks, vec3 SpecularColor, float Roughness, vec3 N, vec3 V , vec3 L, vec4 lightColour)
// {
//     vec3 specularColor = vec3(0.0, 0.0, 0.0);
//     vec3 R = normalize(reflect(V, N));
//     vec3 H = normalize(L + V);
    
//     float alfa = Roughness * Roughness;
//     float NoL = dot(N,L);
//     float NoV = dot(N,V);
//     float NoH = dot(N,H);
//     float VoH = dot(V,H);
    
//     bool back = (NoV>0.0) && (NoL>0.0);
//     if(back)
//     {
//         //Specular D: GGX 函数
//         float temp = NoH * NoH * (alfa * alfa - 1.0) + 1.0;
//         float D = D_GGX(alfa, NoH);//alfa * alfa / (temp * temp);
//         //Specular D: Blinn 函数 approximated with a radially symmetric Phong lobe
//         //float RoL = dot(R,L);  float D = D_Approx(Roughness, RoL);
        
//         //Specular G: Schlick model with Disney hotness remapping
//         float G  = G_Smith_Remapping(Roughness, NoV, NoL);
        
//         //Specular F: Schlick Approximation modified with Spherical Gaussian approximation
//         //float fresnelCoe = f + (1.0 - f) * pow(1.0-vh, 5.0);                       //commonly used
//         //float fresnelCoe = f + (1.0 - f) * pow(2.0, (-5.55473 * VoH - 6.98316) * VoH); // Spherical Gaussian approximation
//         float Fc = pow(2.0, (-5.55473 * VoH - 6.98316) * VoH);
//         vec3  F = (1.0 - Fc) * SpecularColor + Fc;
        
//         // 计算镜面反射光分量
//         vec3 rs = (F * G * D) / (4.0 * NoV * NoL);
//         specularColor = ks * lightColour.rgb * rs * NoL;
//     }
//     return specularColor;
// }

///////////////////-------------------------------

void main()
{
    //temp parameters
    vec3 ka = vec3(0.4);
    vec3 kd = vec3(0.5);
    //vec3 ks = vec3(0.5);    //note: ks = SpecularColor
    vec3 globalAmbient = vec3(0.5);
    float f = 1.0; // 入射角度接近 0(入射方向靠近法向量)时的 Fresnel 反射系数
    vec3 SpecularColor = vec3(f);
    
    float Roughness = u_roughness;
    float alfa = Roughness * Roughness;
    
    //world pos and norm
    vec4 P = world * v_vertex;
    vec3 N = (worldIT * vec4(v_norm, 1.0)).xyz;
    N = normalize(N);
    vec3 V = normalize(eyePosition.xyz - (P.xyz * eyePosition.w));
    
    vec3 L = normalize(lightPosition.xyz - (P.xyz * lightPosition.w));
    
    vec3 diffuseColor = PrefilteredDiffuseIBL(kd, N);
    vec3 specularColorIBL = PrefilteredMobileApproximateSpecularIBL(SpecularColor, Roughness, N, V);
    
    //vec3 specularColorDirectional = SpecularDirectional(ks, SpecularColor, Roughness, N, V, L, lightColour);  //ks?
    vec3 specularColor = specularColorIBL; //+ specularColorDirectional;
    
    //final color
    gl_FragColor = vec4(diffuseColor + specularColor, 1.0);
}


