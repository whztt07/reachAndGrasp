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

//---------------------------------------------
// original Cook-Torrence Shading Model
//---------------------------------------------

vec2 Hammersley(int i, int N)
{
    vec4 temp = texture2D( hammersleyMap, vec2( float(i)/float(N), 0 ) );
    return vec2(temp.x, temp.y);
}

float G_Smith( float Roughness, float NoV, float NoL )
{
    //Specular G: Schlick model with Disney hotness remapping for point light
    //float k = (Roughness + 1.0) * (Roughness + 1.0) / 8.0;
    float k = Roughness * Roughness / 2.0;
    float g1_v = NoV / (NoV * (1.0 - k) + k);
    float g1_l = NoL / (NoL * (1.0 - k) + k);
    return g1_l * g1_v;
}

float G_Smith_Remapping( float Roughness, float NoV, float NoL )
{
    //Specular G: Schlick model with Disney hotness remapping for point light
    float k = (Roughness + 1.0) * (Roughness + 1.0) / 8.0;
    //float k = Roughness * Roughness / 2.0;
    float g1_v = NoV / (NoV * (1.0 - k) + k);
    float g1_l = NoL / (NoL * (1.0 - k) + k);
    return g1_l * g1_v;
}


vec3 ImportanceSampleGGX( vec2 Xi, float Roughness, vec3 N )
{
    float PI = 3.141592653;
    float a = Roughness * Roughness;
    float Phi = 2.0 * PI * Xi.x;
    float CosTheta = sqrt( (1.0 - Xi.y) / ( 1.0 + (a*a - 1.0) * Xi.y ) );
    float SinTheta = sqrt( 1.0 - CosTheta * CosTheta );
    vec3 H;
    H.x = SinTheta * cos( Phi );
    H.y = SinTheta * sin( Phi );
    H.z = CosTheta;
    
    vec3 UpVector = abs(N.z) < 0.999 ? vec3(0,0,1) : vec3(1,0,0);
    vec3 TangentX = normalize( cross( UpVector, N ) );
    vec3 TangentY = cross( N, TangentX );
    // Tangent to world space
    return TangentX * H.x + TangentY * H.y + N * H.z;
}

//
//vec3 SpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
//{
//    vec3 SpecularLighting = vec3(0.0, 0.0, 0.0);
//    int NumSamples = 30;//NumSamplesConst;//24;
//    for( int i = 0; i < NumSamples; i++ )
//    {
//        for( int j = 0; j < NumSamples; j++ )
//        {
//            //vec2 Xi = Hammersley( i, NumSamples );
//            
//            vec2 Xi =  vec2 ( float(i) / float( NumSamples), float(j) / float( NumSamples) ) ;
//            vec3 H = ImportanceSampleGGX( Xi, Roughness, N );
//            vec3 L = 2.0 * dot( V, H ) * H - V;
//            float NoV = dot( N, V );
//            float NoL = dot( N, L );
//            float NoH = dot( N, H );
//            float VoH = dot( V, H );
//            if( NoL > 0.0 )
//            {
//                //L.y = -L.y;
//                vec3 SampleColor = textureCubeLod(sampleEnvMap, L, 0.0).rgb;
//                //vec3 SampleColor = textureCube(sampleEnvMap, L).rgb;     //vec3 SampleColor = EnvMap.SampleLevel( EnvMapSampler, L, 0 ).rgb;
//                float G = G_Smith( Roughness, NoV, NoL );
//                //float G = G_Smith_Remapping( Roughness, NoV, NoL );
//                float Fc = pow( 1.0 - VoH, 5.0 );
//                vec3 F = (1.0 - Fc) * SpecularColor + Fc;
//                // Incident light = SampleColor * NoL
//                // Microfacet specular = D*G*F / (4*NoL*NoV)
//                // pdf = D * NoH / (4 * VoH)
//                SpecularLighting += SampleColor * F * G * VoH / (NoH * NoV);
//            }
//        }
//    }
//    return SpecularLighting / float(NumSamples * NumSamples);//float(NumSamples);
//}




vec3 SpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
{
    vec3 SpecularLighting = vec3(0.0, 0.0, 0.0);
    int NumSamples = NumSamplesConst;//24;
    for( int i = 0; i < NumSamples; i++ )
    {
            vec2 Xi = Hammersley( i, NumSamples );
            vec3 H = ImportanceSampleGGX( Xi, Roughness, N );
            vec3 L = 2.0 * dot( V, H ) * H - V;
            float NoV = dot( N, V );
            float NoL = dot( N, L );
            float NoH = dot( N, H );
            float VoH = dot( V, H );
            if( NoL > 0.0 )
            {
                //L.y = -L.y;
                vec3 SampleColor = textureCubeLod(sampleEnvMap, L, 0.0).rgb;     //vec3 SampleColor = EnvMap.SampleLevel( EnvMapSampler, L, 0 ).rgb;
                //float G = G_Smith( Roughness, NoV, NoL );
                float G = G_Smith_Remapping( Roughness, NoV, NoL );
                float Fc = pow( 1.0 - VoH, 5.0 );
                vec3 F = (1.0 - Fc) * SpecularColor + Fc;
                // Incident light = SampleColor * NoL
                // Microfacet specular = D*G*F / (4*NoL*NoV)
                // pdf = D * NoH / (4 * VoH)
                SpecularLighting += SampleColor * F * G * VoH / (NoH * NoV);
            }
    }
    return SpecularLighting / float(NumSamples);
}


vec3 ImportanceSampleLambert(vec2 xi)
{
    const float PI = 3.1415926f;
    
    float phi = 2 * PI * xi.x;
    float cos_theta = sqrt(1 - xi.y);
    float sin_theta = sqrt(1 - cos_theta * cos_theta);
    return vec3(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);
}

vec3 ImportanceSampleLambert(vec2 xi, vec3 normal)
{
    vec3 h = ImportanceSampleLambert(xi);
    
    vec3 up_vec = abs(normal.z) < 0.999f ? vec3(0, 0, 1) : vec3(1, 0, 0);
    vec3 tangent = normalize(cross(up_vec, normal));
    vec3 binormal = cross(normal, tangent);
    return tangent * h.x + binormal * h.y + normal * h.z;
}


vec3 DiffuseIBL( vec3 DiffuseColor, vec3 N )
{
    vec3 DiffuseLighting = vec3(0.0);
    int NumSamples = NumSamplesConst;;
    for( int i = 0; i < NumSamples; i++ )
    {
        vec2 Xi = Hammersley( i, NumSamples );
        vec3 L = ImportanceSampleLambert( Xi, N );
        //L.y = -L.y;
        vec3 SampleColor = textureCubeLod(sampleEnvMap, L, 0.0).rgb;
        // Incident light = SampleColor * NoL
        // Lambert = c_diff / PI
        // pdf = cos sin /PI
        DiffuseLighting += SampleColor;
    }
    return DiffuseColor * DiffuseLighting / float(NumSamples);
}



vec3 PrefilteredDiffuseIBL( vec3 DiffuseColor, vec3 N )
{
//    vec3 DiffuseLighting = vec3(0.0);
//    int NumSamples = NumSamplesConst;;
//    for( int i = 0; i < NumSamples; i++ )
//    {
//        vec2 Xi = Hammersley( i, NumSamples );
//        vec3 L = ImportanceSampleLambert( Xi, N );
//        //L.y = -L.y;
//        vec3 SampleColor = textureCubeLod(sampleEnvMap, L, 0.0).rgb;
//        // Incident light = SampleColor * NoL
//        // Lambert = c_diff / PI
//        // pdf = cos sin /PI
//        DiffuseLighting += SampleColor;
//    }
    
    vec3 PrefilteredColor = textureCubeLod( prefilteredSampleEnvMap, N, u_MipUpBound).rgb;
    return DiffuseColor * PrefilteredColor;
}



//-------------------------------------------------------------------
// split-sum-approximation Cook-Torrence Shading Model (PC version)
//-------------------------------------------------------------------

vec3 PrefilterEnvMap( float Roughness, vec3 R )
{
    vec3 N = R;
    vec3 V = R;
    vec3 PrefilteredColor = vec3(0.0);
    int NumSamples = NumSamplesConst;//1024;
    float TotalWeight = 0.0;
    for( int i = 0; i < NumSamples; i++ )
    {
            vec2 Xi = Hammersley( i, NumSamples );
            vec3 H = ImportanceSampleGGX( Xi, Roughness, N );
            vec3 L = 2.0 * dot( V, H ) * H - V;
            float NoL = dot( N, L );
            float NoV = dot( N, V );
            if( NoL > 0.0 )
            {
                PrefilteredColor += textureCubeLod(sampleEnvMap, L, 0.0).rgb * NoL;
                TotalWeight += NoL;
            }
    }
    return PrefilteredColor / TotalWeight;
}


vec2 IntegrateBRDF( float Roughness, float NoV)
{
    vec3 V;
    V.x = sqrt( 1.0 - NoV * NoV ); // sin
    V.y = 0.0;
    V.z = NoV;                      // cos
    vec3 N = V;   //not quite sure
    float A = 0.0;
    float B = 0.0;
    const int NumSamples = NumSamplesConst;
    for( int i = 0; i < NumSamples; i++ )
    {
            vec2 Xi = Hammersley( i, NumSamples );
            vec3 H = ImportanceSampleGGX( Xi, Roughness, N );
            vec3 L = 2.0 * dot( V, H ) * H - V;
            float NoL = L.z;
            float NoH = H.z;
            float VoH = dot( V, H );
            if( NoL > 0.0 )
            {
                float G = G_Smith( Roughness, NoV, NoL );
                float G_Vis = G * VoH / (NoH * NoV);
                float Fc = pow( 1.0 - VoH, 5.0 );
                A += (1.0 - Fc) * G_Vis;
                B += Fc * G_Vis;
            }
    }
    return vec2( A, B ) / float(NumSamples);
}

vec3 ApproximateSpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
{
    float NoV =  dot( N, V ) ;
    vec3 R = 2.0 * dot( V, N ) * N - V;
    
    vec3 PrefilteredColor = PrefilterEnvMap( Roughness, R );
    vec2 EnvBRDF = IntegrateBRDF( Roughness, NoV);
    return PrefilteredColor * ( SpecularColor * EnvBRDF.x + EnvBRDF.y );
}

//-----------------------------------------------------------------------
// split-sum-approximation Cook-Torrence Shading Model (Mobile Version)
//-----------------------------------------------------------------------

vec3 EnvBRDFApprox( vec3 SpecularColor, float Roughness, float NoV )
{
    const vec4 c0 = vec4( -1.0, -0.0275, -0.572, 0.022 );
    const vec4 c1 = vec4(  1.0,  0.0425,  1.04,  -0.04 );
    vec4 r = Roughness * c0 + c1;
    float a004 = min( r.x * r.x, exp2( -9.28 * NoV ) ) * r.x + r.y;
    vec2 AB = vec2( -1.04, 1.04 ) * a004 + r.zw;
    return SpecularColor * AB.x + AB.y;
}

vec3 MobileApproximateSpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
{
    float NoV =  dot( N, V ) ;
    vec3 R = 2.0 * dot( V, N ) * N - V;
    
    vec3 PrefilteredColor = PrefilterEnvMap( Roughness, R );
    vec3 EnvBRDF =  EnvBRDFApprox( SpecularColor, Roughness, NoV );
    return PrefilteredColor * EnvBRDF;
}





//// @param MipCount e.g. 10 for x 512x512
//float ComputeCubemapMipFromRoughness( float Roughness, float MipCount )
//{
//    // Level starting from 1x1 mip
//    float Level = 3.0 - 1.15 * log2( Roughness ); // Roughness (0,1) Level (9,3)
//    return MipCount - 1.0 - Level;                //
//}

vec3 PrefilteredMobileApproximateSpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
{
    float NoV =  dot( N, V ) ;
    vec3 R = 2.0 * dot( V, N ) * N - V;
    R= normalize(R);

//// testing
//    float Mip = ComputeCubemapMipFromRoughness( Roughness, 8.0 );
//    Mip = Roughness * 5.0;
//#ifdef GL_ARB_texture_query_lod
//    Mip = textureQueryLod(myTexture, textureCoord).x; // NOTE CAPITALIZATION
//#endif
    
    float Mip = Roughness * ( u_MipUpBound - 1 );
    vec3 PrefilteredColor = //prefilteredEnvFun;
                            textureCubeLod( prefilteredSampleEnvMap, R, Mip ).rgb;
                            //TextureCubeSampleLevel( AmbientCubemap, AmbientCubemapSampler, R, Mip).rgb;//PrefilterEnvMap( Roughness, R );
    vec3 EnvBRDF =  EnvBRDFApprox( SpecularColor, Roughness, NoV );
    return PrefilteredColor * EnvBRDF;
}



//--------------------------------------------------------------------------------
// split-sum-approximation Cook-Torrence Shading Model (LUT Version) not finished
//--------------------------------------------------------------------------------

/*
Texture2D		PreIntegratedGF;
SamplerState	PreIntegratedGFSampler;
TextureCube     AmbientCubemap;
SamplerState    AmbientCubemapSampler;
// .x:mul, .y:add for mip adjustement, z:DiffuseMip=MipCount-DiffuseConvolveMipLevel, w:MipCount
vec4 AmbientCubemapMipAdjust;

// @param MipCount e.g. 10 for x 512x512
float ComputeCubemapMipFromRoughness( float Roughness, float MipCount )
{
    // Level starting from 1x1 mip
    float Level = 3 - 1.15 * log2( Roughness );
    return MipCount - 1 - Level;
}

vec3 EnvBRDF( vec3 SpecularColor, float Roughness, float NoV )
{
    // Importance sampled preintegrated G * F
    vec2 AB = Texture2DSampleLevel( PreIntegratedGF, PreIntegratedGFSampler, vec2( NoV, Roughness ), 0 ).rg;
    // Anything less than 2% is physically impossible and is instead considered to be shadowing
    vec3 GF = SpecularColor * AB.x + saturate( 50.0 * SpecularColor.g ) * AB.y;
    return GF;
}

vec3 LutApproximateSpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
{
    float NoV =  dot( N, V ) ;
    vec3 R = 2.0 * dot( V, N ) * N - V;
    float Mip = ComputeCubemapMipFromRoughness( Roughness, AmbientCubemapMipAdjust.w );
    
    vec3 PrefilteredColor = TextureCubeSampleLevel( AmbientCubemap, AmbientCubemapSampler, R, Mip).rgb;//PrefilterEnvMap( Roughness, R );
    vec3 EnvBRDF =  EnvBRDF( SpecularColor, Roughness, NoV);//EnvBRDFApprox(Roughness, NoV );
    //vec3 EnvBRDF =  EnvBRDFApprox( SpecularColor, Roughness, NoV );
    return PrefilteredColor * EnvBRDF;
}
*/
 
///////////////////-------------------------------


//-----------------------------------------------------------------------
// Point Light Cook-Torrence Shading Model (Mobile Version)
//-----------------------------------------------------------------------


float D_Approx( float Roughness, float RoL )
{
    float a = Roughness * Roughness;
    float a2 = a * a;
    float rcp_a2 = 1.0/a2;
    // 0.5 / ln(2), 0.275 / ln(2)
    float c = 0.72134752 * rcp_a2 + 0.39674113;
    return rcp_a2 * exp2( c * RoL - c );
}

float D_GGX(float alfa, float NoH)
{
    //Specular D: GGX 函数
    float temp = NoH * NoH * (alfa * alfa - 1.0) + 1.0;
    return alfa * alfa / (temp * temp);
}


vec3 SpecularDirectional( vec3 ks, vec3 SpecularColor, float Roughness, vec3 N, vec3 V , vec3 L, vec4 lightColour)
{
    vec3 specularColor = vec3(0.0, 0.0, 0.0);
    vec3 R = normalize(reflect(V, N));
    vec3 H = normalize(L + V);
    
    float alfa = Roughness * Roughness;
    float NoL = dot(N,L);
    float NoV = dot(N,V);
    float NoH = dot(N,H);
    float VoH = dot(V,H);

    bool back = (NoV>0.0) && (NoL>0.0);
    if(back)
    {
        //Specular D: GGX 函数
        float temp = NoH * NoH * (alfa * alfa - 1.0) + 1.0;
        float D = D_GGX(alfa, NoH);//alfa * alfa / (temp * temp);
        //Specular D: Blinn 函数 approximated with a radially symmetric Phong lobe
        //float RoL = dot(R,L);  float D = D_Approx(Roughness, RoL);
        
        //Specular G: Schlick model with Disney hotness remapping
        float G  = G_Smith_Remapping(Roughness, NoV, NoL);

        //Specular F: Schlick Approximation modified with Spherical Gaussian approximation
        //float fresnelCoe = f + (1.0 - f) * pow(1.0-vh, 5.0);                       //commonly used
        //float fresnelCoe = f + (1.0 - f) * pow(2.0, (-5.55473 * VoH - 6.98316) * VoH); // Spherical Gaussian approximation
        float Fc = pow(2.0, (-5.55473 * VoH - 6.98316) * VoH);
        vec3  F = (1.0 - Fc) * SpecularColor + Fc;
        
        // 计算镜面反射光分量
        vec3 rs = (F * G * D) / (4.0 * NoV * NoL);
        specularColor = ks * lightColour.rgb * rs * NoL;
    }
    return specularColor;
}

///////////////////-------------------------------

void main()
{
    //temp parameters
    vec3 ka = vec3(0.4);
    vec3 kd = vec3(0.5);
    //vec3 ks = vec3(0.5);  //note: ks = SpecularColor
    vec3 globalAmbient = vec3(0.5);
    float f = 1.0; // 入射角度接近 0(入射方向靠近法向量)时的 Fresnel 反射系数
    vec3 SpecularColor = vec3(f);
  
    //float alfa = 0.05; // C-T, 表面的粗糙程度
    //float Roughness = sqrt(alfa);
    float Roughness = u_roughness;
    float alfa = Roughness * Roughness;
    
    //world pos and norm
    vec4 P = world * v_vertex;
    vec3 N = (worldIT * vec4(v_norm, 1.0)).xyz;
    N = normalize(N);
    vec3 V = normalize(eyePosition.xyz - (P.xyz * eyePosition.w));

    vec3 L = normalize(lightPosition.xyz - (P.xyz * lightPosition.w));
    
    //ambient color, already in BRDF
    //vec3 ambientColor = ka * globalAmbient;
    
    //diffuse color
    //float NoL = max(dot(N, L), 0.0);
    //vec3 diffuseColor = kd * lightColour.rgb * NoL;
    vec3 diffuseColor = DiffuseIBL(kd, N);
    //vec3 diffuseColor = PrefilteredDiffuseIBL(kd, N);
    
    
    
    //specular color
    vec3 specularColorIBL = SpecularIBL(SpecularColor, Roughness, N, V);
    //vec3 specularColorIBL = ApproximateSpecularIBL(SpecularColor, Roughness, N, V);       //not so right
    //vec3 specularColorIBL = MobileApproximateSpecularIBL(SpecularColor, Roughness, N, V);
    //vec3 specularColorIBL = PrefilteredMobileApproximateSpecularIBL(SpecularColor, Roughness, N, V);

    //vec3 specularColorDirectional = SpecularDirectional(ks, SpecularColor, Roughness, N, V, L, lightColour);  //ks?
    vec3 specularColor = specularColorIBL; //+ specularColorDirectional;
    
    //final color
    gl_FragColor = vec4(diffuseColor + specularColor, 1.0);
    //gl_FragColor = vec4(diffuseColor, 1.0);//vec4(specularColor, 1.0);
 }


