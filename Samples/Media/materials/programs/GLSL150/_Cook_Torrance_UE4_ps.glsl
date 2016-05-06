
varying  vec4 colorVarying;
varying vec4 v_vertex;
varying vec3 v_norm;

uniform mat4 world;
uniform mat4 worldIT;
//uniform mat4 worldViewProj;
uniform vec4 lightPosition;
uniform vec4 lightColour;
uniform vec4 eyePosition;
uniform samplerCube sampleEnvMap;



float radicalInverse_VdC(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10; // / 0x100000000
}

//Hammersley
// http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
//int ReverseBits(int bits)
//{
//    bits = (bits << 16) | (bits >> 16);
//    bits = ((bits & 0x55555555) << 1) | ((bits & 0xAAAAAAAA) >> 1);
//    bits = ((bits & 0x33333333) << 2) | ((bits & 0xCCCCCCCC) >> 2);
//    bits = ((bits & 0x0F0F0F0F) << 4) | ((bits & 0xF0F0F0F0) >> 4);
//    bits = ((bits & 0x00FF00FF) << 8) | ((bits & 0xFF00FF00) >> 8);
//    return bits;
//}
//
//float RadicalInverseVdC(int bits)
//{
//    return ReverseBits(bits) * 2.3283064365386963e-10; // / 0x100000000
//}
//
//vec2 Hammersley2D(int i, int N)
//{
//    return vec2( float（i） / float（N）, RadicalInverseVdC(i) );
//}
//Hammersley END


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



void SpecularIBL(vec3 SpecularColor, float Roughness, vec3 N, vec3 V)
{
        vec3 SpecularLighting = vec3(0.0, 0.0, 0.0);
        int NumSamples = 1024;
        for( int i = 0; i < NumSamples; i++ )
        {
            vec2 Xi = Hammersley2D( i, NumSamples );
            vec3 H = ImportanceSampleGGX( Xi, Roughness, N );
//            vec3 L = 2.0 * dot( V, H ) * H - V;
//            float NoV = dot( N, V );
//            float NoL = dot( N, L );
//            float NoH = dot( N, H );
//            float VoH = dot( V, H );
//            if( NoL > 0.0 )
//            {
//                //￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼vec3 SampleColor = textureCube(sampleEnvMap, L).rgb;     //￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼vec3 SampleColor = EnvMap.SampleLevel( EnvMapSampler, L, 0 ).rgb;
//                float G = G_Smith( Roughness, NoV, NoL );
//                float Fc = pow( 1.0 - VoH, 5.0 );
//                vec3 F = (1 - Fc) * SpecularColor + Fc;
//                // Incident light = SampleColor * NoL
//                // Microfacet specular = D*G*F / (4*NoL*NoV)
//                // pdf = D * NoH / (4 * VoH)
//                SpecularLighting += SampleColor * F * G * VoH / (NoH * NoV);
//            }
        }

    
}

//
//
//void SpecularIBL( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
//{
////    vec3 SpecularLighting = vec3(0.0, 0.0, 0.0);
////    int NumSamples = 1024;
////    for( int i = 0; i < NumSamples; i++ )
////    {
////        vec2 Xi = Hammersley( i, NumSamples );
////        vec3 H = ImportanceSampleGGX( Xi, Roughness, N );
////        vec3 L = 2.0 * dot( V, H ) * H - V;
////        float NoV = dot( N, V );
////        float NoL = dot( N, L );
////        float NoH = dot( N, H );
////        float VoH = dot( V, H );
////        if( NoL > 0.0 )
////        {
////            //￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼vec3 SampleColor = textureCube(sampleEnvMap, L).rgb;     //￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼￼vec3 SampleColor = EnvMap.SampleLevel( EnvMapSampler, L, 0 ).rgb;
////            float G = G_Smith( Roughness, NoV, NoL );
////            float Fc = pow( 1.0 - VoH, 5.0 );
////            vec3 F = (1 - Fc) * SpecularColor + Fc;
////            // Incident light = SampleColor * NoL
////            // Microfacet specular = D*G*F / (4*NoL*NoV)
////            // pdf = D * NoH / (4 * VoH)
////            SpecularLighting += SampleColor * F * G * VoH / (NoH * NoV);
////        }
////    }
//    ￼￼￼￼￼￼//return vec3(0.0, 0.0, 0.0);//SpecularLighting / (float)NumSamples;
//}
//


void main()
{
    //gl_FragColor = colorVarying;//vec4(1.0, 0.0, 0.0, 1.0);

    
    //temp parameters
    vec3 kd = vec3(0.4, 0.2, 0.0);
    vec3 ks = vec3(1.0, 0.1, 0.2);
    float shininess = 30.0;
    vec3 globalAmbient = vec3(1.0, 1.0, 1.0);
    float m = 0.9; // C-T, 表面的粗糙程度
    float f = 0.9; // C-T, 入射角度接近 0(入射方向靠近法向量)时的 Fresnel 反射系数
    float alfa = m; float s = sqrt(alfa);
    
    //world pos and norm
    vec4 worldPos = world * v_vertex;
    vec3 worldNorm = (worldIT * vec4(v_norm, 1.0)).xyz;
    worldNorm = normalize(worldNorm);
    
    vec3 V = normalize(eyePosition.xyz - (worldPos.xyz * eyePosition.w));
    vec3 N = worldNorm;
     vec3 R = vec3(0.0,0.0,0.0);
    
    //environment color
    ////environment mapping //环境贴图
    R = reflect(V, N);
    //// Ogre conversion for cube map lookup
    R.y = -R.y;
    vec4 envColor = textureCube(sampleEnvMap, R);
    
    //diffuse color
    ////directional_light diffuse color
    vec3 lightDir = normalize(lightPosition.xyz - (worldPos.xyz * lightPosition.w));
    float nl = max(dot(worldNorm, lightDir), 0.0);
    vec3 direct_diffuseColor = kd * lightColour.rgb * nl;
    ////ambient_light  diffuse color
    vec3 ambient_diffuseColor = kd * globalAmbient;
    
    vec3 diffuseColor = direct_diffuseColor + ambient_diffuseColor;
    
    
    //specular light
    vec3 specularColor = vec3(0.0,0.0,0.0);
    vec3 L = lightDir;
    vec3 H = normalize(L + V);

    
    ////C-T
    {
        float nv = dot(N,V);
        bool back = (nv>0.0) && (nl>0.0);
        if(back)
        {
            //Specular D: GGX 函数
            float nh = dot(N, H);
            float temp = nh * nh * (alfa * alfa - 1.0) + 1.0;
            float roughness = alfa * alfa / (temp * temp);
        
            //Specular G: Schlick model with Disney hotness remapping
            float k = (s + 1.0) * (s + 1.0) / 8.0;
            float g1_v = dot(N, V) / (dot(N, V) * (1.0 - k) + k);
            float g1_l = dot(N, L) / (dot(N, L) * (1.0 - k) + k);
            float geometric = g1_l * g1_v;
            
            //Specular F: Schlick Approximation modified with Spherical Gaussian approximation
            float vh = dot(V,H);
            //float fresnelCoe = f + (1.0 - f) * pow(1.0-vh, 5.0);                       //commonly used
            float fresnelCoe = f + (1.0 - f) * pow(2.0, (-5.55473 * vh - 6.98316) * vh); // Spherical Gaussian approximation

            // 计算镜面反射光分量(这是重点)
            float rs = (fresnelCoe * geometric * roughness) / (nv * nl);
            specularColor = ks  * envColor.rgb /*lightColour.rgb*/ * rs * nl;
        }
    }
  
//    //environment color
//    ////environment mapping //环境贴图
//    R = reflect(V, N);
//    //// Ogre conversion for cube map lookup
//    R.y = -R.y;
//    vec4 envColor = textureCube(sampleEnvMap, R);

    //final color
    //gl_FragColor = envColor+ 0.25 * vec4(diffuseColor + specularColor, 1.0);
    //gl_FragColor = vec4(diffuseColor + specularColor, 1.0);
    //gl_FragColor = envColor * vec4(diffuseColor + specularColor, 1.0);
    gl_FragColor = mix(envColor, vec4(diffuseColor + specularColor, 1.0), 0.8);
}