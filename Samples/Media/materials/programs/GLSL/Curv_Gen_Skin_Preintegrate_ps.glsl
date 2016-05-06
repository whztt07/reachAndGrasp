#version 120
#extension GL_ARB_shader_texture_lod : require

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


uniform mat4 world;
uniform mat4 worldIT;
uniform vec4 lightPosition;
uniform vec4 lightColour;
uniform vec4 eyePosition;
uniform sampler2D hammersleyMap;
uniform samplerCube sampleEnvMap;
uniform samplerCube prefilteredSampleEnvMap;
uniform sampler2D normalMap;
uniform sampler2D preIntegDiffuseMap;
uniform sampler2D colorMap;
uniform sampler2D preIntegShadowMap;
uniform float u_roughness;
uniform float u_MipUpBound;
uniform float u_CurvatureScale;

uniform float u_lowNormalBias;
uniform float u_tuneNormalNlurRed;
uniform float u_tuneNormalNlurGreen;
uniform float u_tuneNormalNlurBlue;

uniform float u_skin_diffuse;
uniform float u_specular_power;
uniform float u_rendering_mode;
uniform float u_rendering_mode_number;
uniform float u_normal_map_blur_level;
uniform float u_light_diffuse_intensity;

const int NumSamplesConst = 1024;
const float PI = 3.1415926;
 float curv_max_bake_texture = 4.0;//0.5;
//const float PROFILE_WIDTH = 2.0;

//float CurvatureScale = 0.0005; // 0.02~0.002 //0.0005 for uv_sphere, curvature test


//// http://filmicgames.com/archives/75
// float4 ps_main( float2 texCoord  : TEXCOORD0 ) : COLOR
// {
//    float3 texColor = tex2D(Texture0, texCoord );
//    texColor *= 16;  // Hardcoded Exposure Adjustment
//    float3 x = max(0,texColor-0.004);
//    float3 retColor = (x*(6.2*x+.5))/(x*(6.2*x+1.7)+0.06);
//    return float4(retColor,1);
// }

vec3 max(float a, vec3 b)
{
    if(a>b.x)   b.x = a;
    if(a>b.y)   b.y = a;
    if(a>b.z)   b.z = a;
    return b;
}

vec4 filmic(vec4 color, float exposureParam)
{
   color.xyz *= exposureParam;  // Hardcoded Exposure Adjustment
   vec3 x = max(0.0,color.xyz-0.004);
   vec3 retColor = (x*(6.2*x+.5))/(x*(6.2*x+1.7)+0.06);
   return vec4(retColor,1.0);
}



// Expand a range-compressed vector for normal map
vec3 expand(vec3 v)
{
    return (v - 0.5) * 2.0; 
}

float saturate( float v)
{
    if( v < 0.0 ) v = 0.0;
    if( v > 1.0 ) v = 1.0;
    return v;
}

//Calculated Gaussian Blur, performs the blur based on V and R
float Gaussian(float v, float r)
{
    return 1.0f / sqrt(2.0f * PI * v) * exp(-(r*r) / (2.0*v));
}

//Scattering Algorithm, Hard-coded for Ideal skin rendering though it still looks good on other stuff!
vec3 Scatter( float r )
{
    //Coefficients from GPU Gems 3 - "Advanced Skin Rendering"
    return Gaussian(.0064f * 1.414, r) * vec3(.233f,.455f,.649f) +
           Gaussian(.0484f * 1.414, r) * vec3(.1f,.336f,.344f) +
           Gaussian(.187f * 1.414, r) * vec3(.118f,.198f,0.0f) +
           Gaussian(.567f * 1.414, r) * vec3(.113f,.007f,.007f) +
           Gaussian(1.99f * 1.414, r) * vec3(.358f,.004f,0.0f) +
           Gaussian(7.41f * 1.414, r) * vec3(.078f,0.0f,0.0f) ;
}


vec3 IntegrateDiffuseScatteringOnRing(float cosTheta , float skinRadius) 
{
        // Angle from lighting direction
        float theta = acos(cosTheta);
        vec3 totalWeights = vec3(0.0);
        vec3 totalLight = vec3(0.0);
 
        float a = -(PI/2.0f);
        const float inc = 0.05f;
 
        while (a <= (PI/2.0f)) {
            float sampleAngle = theta + a;
            float diffuse = saturate( cos(sampleAngle) );
 
            // Distance
            float sampleDist = abs( 2.0f * skinRadius * sin(a * 0.5f) );
 
            // Profile Weight
            vec3 weights = Scatter(sampleDist);
 
            totalWeights += weights;
            totalLight += diffuse * weights;
            a+=inc;
        }
 
        vec3 result = vec3(totalLight.x / totalWeights.x, totalLight.y / totalWeights.y, totalLight.z / totalWeights.z);
        return result;
}

////http://c0de517e.blogspot.jp/2011/09/mathematica-and-skin-rendering.html
// vec3 IntegrateDiffuseScatteringOnRingALU(float NdotL, float curv)
// {
//     float cos_theta = NdotL;
//     float r = 1.0/curv;
//     float a0 = 0.0605281;
//     float a1 = 0.0903942;
//     float a2 = 0.0210583;
//     float a3 = 0.689657;
//     float a4 = 0.11103;
//     float a5 = 0.817767;
//     float a6 = 7.15776;
//     float a7 = 5.89292;
//     max(max(cos_theta * min((a0*r+a1),1) + max((a2*r+a3),0),0), 
//         max(cos_theta * min((a4*r+a5),1) + max((a6*r+a7),0),0));
// }

vec3 SkinDiffuse(float curv, vec3 NdotL3)
{
    // //skin_lookup_diffspec.png //not done....
    // vec3 lookup = NdotL3 * 0.5 + 0.5;
    // curv = 1.0 - curv;  //remap since different image reading approach
    // vec3 diffuse;
    // diffuse.r =  2.0 * texture2D( preIntegDiffuseMap, vec2(lookup.r, curv) ).r; 
    // diffuse.g =  2.0 * texture2D( preIntegDiffuseMap, vec2(lookup.g, curv) ).g;
    // diffuse.b =  2.0 * texture2D( preIntegDiffuseMap, vec2(lookup.b, curv) ).b;
    
    // //pre_int_skin_diffuse.png //not so real
    // vec3 lookup = NdotL3 * 0.5 + 0.5;
    // curv = 1.0 - curv;  //remap since different image reading approach
    // vec3 diffuse;
    // diffuse.r =  texture2D( preIntegDiffuseMap, vec2(lookup.r, curv) ).r; 
    // diffuse.g =  texture2D( preIntegDiffuseMap, vec2(lookup.g, curv) ).g;
    // diffuse.b =  texture2D( preIntegDiffuseMap, vec2(lookup.b, curv) ).b;

    //DiffuseScatteringOnRing.png //good one
    vec3 lookup = NdotL3 * 0.5 + 0.5;
    curv = curv * dot( lightColour.rgb, vec3(0.22, 0.707, 0.071 ) ); //Multiplied by light's luminosity so that brighter light leads to more scattering.
    curv = 1.0 - curv/curv_max_bake_texture;  //remap
    vec3 diffuse;
    diffuse.r =  texture2D( preIntegDiffuseMap, vec2(lookup.r, curv) ).r; 
    diffuse.g =  texture2D( preIntegDiffuseMap, vec2(lookup.g, curv) ).g;
    diffuse.b =  texture2D( preIntegDiffuseMap, vec2(lookup.b, curv) ).b;

    // //ground truth // effected by curvature badly
    // vec3 diffuse;
    // diffuse.r = IntegrateDiffuseScatteringOnRing(NdotL3.x, 1.0/curv).r;
    // diffuse.g = IntegrateDiffuseScatteringOnRing(NdotL3.y, 1.0/curv).g;
    // diffuse.b = IntegrateDiffuseScatteringOnRing(NdotL3.z, 1.0/curv).b;

    return diffuse;
}

//// shadow functions in preintegrated skin render,  not used
// float newPenumbra(float x)
// {
//    //TODO
//    return x;
// }
//
// vec3 integrateShadowScattering ( float penumbraLocation , float penumbraWidth )
// {
//     vec3 totalWeights = vec3(0.0); 
//     vec3 totalLight = vec3(0.0); 
//     float a = (-1.0)*PROFILE_WIDTH;
//     const float inc=0.05f;
//     while(a<=PROFILE_WIDTH)
//         {
//             float light=newPenumbra(penumbraLocation + a/penumbraWidth );
//             float sampleDist=abs(a);
//             vec3 weights=Scatter(sampleDist);
//             totalWeights+=weights;
//             totalLight+=light*weights;
//             a+=inc;
//         }

//     return totalLight / totalWeights ; 
// }


//---------------------------------------------------------------------------------


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


//#define C_K
#define SKIN_DIFF
#define NORMAL_MAP

void main()
{
//temp
//CurvatureScale = u_CurvatureScale;
float Roughness = u_roughness;
float alfa = Roughness * Roughness;

#ifdef NORMAL_MAP
    vec3 N = expand(texture2D(normalMap, v_texCoord).xyz);

    //sphere testing
    //N = normalize(v_TSNormal);

    vec3 V = normalize(v_TSViewDir);
    vec3 L = normalize(v_TSLightDir);

    vec3 H = normalize(v_TSHalfAngle);
    //vec3 diffuse_color = max(dot(N, L), 0.0) * lightColour.rgb;
    //gl_FragColor = vec4(diffuse_color, 1.0);
    
    // vec3 halfAngle = normalize(v_TSHalfAngle);
    // float specFactor = pow(clamp(dot(N, halfAngle), 0.0, 1.0), 1.0);
    // gl_FragColor = vec4(specFactor*lightColour.rgb, 1.0);

    //gl_FragColor = colorVarying;
    //gl_FragColor = vec4(texture2D(normalMap, lightPosition.xy).xyz, 1.0) ;
    vec3 P = v_TSPos;
#else
    //world pos and norm
    vec3 P = (world * v_vertex).xyz;
    vec3 N = (worldIT * vec4(v_norm, 1.0)).xyz;
    N = normalize(N);
    vec3 V = normalize(eyePosition.xyz - (P * eyePosition.w));
    vec3 L = normalize(lightPosition.xyz - (P * lightPosition.w));
#endif


#ifdef C_K 
    vec3 diffuseColor = PrefilteredDiffuseIBL(kd, N);
    vec3 specularColorIBL = PrefilteredMobileApproximateSpecularIBL(SpecularColor, Roughness, N, V);
    
    //vec3 specularColorDirectional = SpecularDirectional(ks, SpecularColor, Roughness, N, V, L, lightColour);  //ks?
    vec3 specularColor = specularColorIBL; //+ specularColorDirectional;
    
    //final color
    gl_FragColor = vec4(diffuseColor + specularColor, 1.0);
#endif


#ifdef SKIN_DIFF
    //////skin diffuse color using preintegration skin shading

    //---- normal 
    #ifdef NORMAL_MAP
        vec3 tuneNormalNlur = vec3 (u_tuneNormalNlurRed, u_tuneNormalNlurGreen, u_tuneNormalNlurBlue);//vec3(0.1, 0.6, 0.7);
        vec3 N_high = N; //expand(texture2D(normalMap, v_texCoord).xyz);
        vec3 N_low = expand(texture2D(normalMap, v_texCoord, u_lowNormalBias).xyz);
        vec3 N_red = normalize(mix(N_high, N_low, tuneNormalNlur.r));
        vec3 N_green = normalize(mix(N_high, N_low, tuneNormalNlur.g));
        vec3 N_blue = normalize(mix(N_high, N_low, tuneNormalNlur.b));
        vec3 NoL3 = vec3(dot(N_red, L), dot(N_green, L), dot(N_blue, L));  //for R G B 
         //NoL3 = vec3(dot(N, L));  //sphere testing
    #else
        vec3 NoL3 = vec3(dot(N, L));
    #endif
    //---- normal END

    //----Curvature  // can be optimized by reading curvatures from texture or ogre
    vec3 v_WS_normal_normalized = normalize(v_WS_normal);
    vec3 v_WS_tangent_normalized = normalize(v_WS_tangent);
    v_WS_tangent_normalized = normalize(v_WS_tangent_normalized - v_WS_normal_normalized * v_WS_tangent_normalized * v_WS_normal_normalized);
    //WS->TS matrix rotation
    vec3 binormal = (cross(v_WS_normal_normalized, v_WS_tangent_normalized));
    mat3 rotation = mat3(vec3(v_WS_tangent_normalized[0], binormal[0], v_WS_normal_normalized[0]),
                         vec3(v_WS_tangent_normalized[1], binormal[1], v_WS_normal_normalized[1]),
                         vec3(v_WS_tangent_normalized[2], binormal[2], v_WS_normal_normalized[2]));
    //TS->WS matrix inv rotation
    mat3 inv_rotation = transpose(rotation);
    vec3 WS_N = inv_rotation * texture2D(normalMap, v_texCoord, u_normal_map_blur_level).xyz;        //TS --> WS
    vec3 WS_P = v_WS_vertex.xyz;//(world*v_vertex).xyz;    //v_WS_vertex.xyz;         //TS --> WS

    float deltaWSNormal = saturate(length( fwidth(normalize(WS_N)) )); 
    float deltaWSPosition =        length( fwidth ( ((WS_P) ) ));
    float curvature = deltaWSNormal / deltaWSPosition * u_CurvatureScale;
    //sphere testing
    //curvature = u_CurvatureScale;

    //curvature = deltaWSNormal * u_CurvatureScale;
    //if(curvature < 1e-1) curvature = 1e-1;
    //curvature = 1.0;//u_CurvatureScale;//0.01;
    //----Curvature END

    //----diffuse
    vec3 skinDiffuseColor = SkinDiffuse(curvature, NoL3) * lightColour.rgb * u_light_diffuse_intensity;  //*SkinShadow( SampleShadowMap(ShadowUV) ) 
    vec3 diffuse = vec3(0.0);
    vec3 lightDiffuse = vec3(1.0);
    float NoL = dot(N, L);
    if (u_skin_diffuse >0.5)
        diffuse = skinDiffuseColor;
    else 
        diffuse = lightDiffuse * NoL * u_light_diffuse_intensity;
    //----diffuse END

    //---- specular
    float NoH = dot(N, H);
    float po = pow( max( 0.0, NoH ), u_specular_power);
    vec3 specularLight = vec3(1.0);
    vec3 specularColor =  vec3(1.0); //texture2D(colorMap, v_texCoord).rgb; //vec3(0.5);//
    float specularLevel =  texture2D(normalMap, v_texCoord).a; //小偶的数据
    vec3 skinSpecularColor = specularLight * specularColor * specularLevel * po;

    //---- specular END

    // //----skin shadow
    // float shad = 0.5;
    // float width =  0.5;
    // vec3 skinShadow = texture2D(preIntegShadowMap, vec2(shad, width)).rgb;

    //----add everything
    //diffuse = filmic(vec4(diffuse, 1.0)).rgb;  //testing filmic
    vec3 ambient = vec3 (0.2);
    vec4 ambint_diffuse = vec4 (ambient + diffuse, 1.0);
    vec4 base = texture2D(colorMap, v_texCoord);
    vec4 outcolor = base * ambint_diffuse ;//+ vec4(skinSpecularColor, 1.0);
    outcolor.a = base.a;

    if(u_rendering_mode == 0.0){ //color
        gl_FragColor = vec4(1.0) * outcolor;
        //gl_FragColor = vec4(diffuse,1.0); //sphere testing
        //gl_FragColor = vec4(skinDiffuseColor + skinSpecularColor, 1.0);
        //u_CurvatureScale = 0.1;
        //gl_FragColor = vec4(N, 1.0);
        //gl_FragColor = filmic(gl_FragColor, 1.0);//testing filmic
    }
    else if(u_rendering_mode == 1.0) { //curvature
        gl_FragColor = vec4(1.0) * curvature;
        //gl_FragColor = filmic(vec4(NoL3,1.0)); //sphere testing
    }

    //gl_FragCoord = v_texCoord;
    gl_FragColor = vec4(v_TSNormal,1.0);//texture2D(normalMap, gl_FragCoord.xy/1024.0);//textureSize(normalMap, 0));

#endif
//gl_FragColor = vec4(N, 1.0);
//gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0) * dot(N, L);
//gl_FragColor = colorVarying;

}

