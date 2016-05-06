// #version 100
// #extension GL_EXT_shader_texture_lod : require

// precision highp int;
// precision highp float;

#version 120
#extension GL_ARB_shader_texture_lod : require

uniform mat4 world;
uniform mat4 worldIT;
uniform vec4 lightPosition;
uniform vec4 eyePosition;
uniform	sampler2D	colorMap;
uniform	sampler2D	normalMap;
uniform	samplerCube	sampleEnvMap;
uniform sampler2D   specularLevelMap; 
uniform sampler2D   normalDiffMap; 
uniform sampler2D  lowFreqNormalMap;
uniform float  u_wetness;
uniform float  lowFreqDownBound;
uniform float  lowFreqUpBound;
uniform float  highFreqNormalLod;
uniform float  lowFreqNormalLod;
uniform float u_cube_specular_light_intensity;
uniform float u_cube_diffuse_light_intensity;
uniform float u_cube_roughness;


varying	vec2 v_tex_coord;
varying vec4 v_WS_vertex;
varying vec3 v_WS_normal;
varying vec3 v_WS_tangent;
varying vec3 v_WS_binormal;

float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}
vec3 lerp3(vec3 a, vec3 b, float t)
{
    return a + t * (b - a);
}


vec3 expand(vec3 v)
{
    return (v - 0.5) * 2.0; 
}
mat3 transpose(mat3 m){
    return mat3(
            m[0][0],m[1][0],m[2][0],
            m[0][1],m[1][1],m[2][1],
            m[0][2],m[1][2],m[2][2]
            );
}

vec3 fresnel (vec3 N, vec3 L, vec3 F0)
{
    // return max( 0, pow( abs(1.0-dot(N,V)) ,fresnelExp ) );
    
    return F0 +(vec3(1.0) - F0) * pow( abs(1.0-dot(N,L)) ,5.0 );
}

//pbr
// uniform samplerCube sPrefilteredSampleEnvMap0;
// uniform sampler2D sDiffuse1;
// uniform float Roughness;
uniform float MipUpBound;
vec3 PrefilteredDiffuseIBL( vec3 DiffuseColor, vec3 N )
{    
    //vec3 PrefilteredColor = textureCubeLod( sPrefilteredSampleEnvMap, N, c_MipUpBound).rgb;
    vec3 PrefilteredColor = textureCubeLod( sampleEnvMap, N, MipUpBound).rgb;//textureCube( sPrefilteredSampleEnvMap0, N, MipUpBound).rgb;
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
    
    float Mip = Roughness * ( MipUpBound - 1.0 );
    vec3 PrefilteredColor = textureCubeLod( sampleEnvMap, R, Mip ).rgb;
    vec3 EnvBRDF =  EnvBRDFApprox( SpecularColor, Roughness, NoV );
    return PrefilteredColor * EnvBRDF;
}
//pbr end







float G_Smith( float Roughness, float NoV, float NoL )
{
    //Specular G: Schlick model with Disney hotness remapping for point light
    //float k = (Roughness + 1.0) * (Roughness + 1.0) / 8.0;
    float k = Roughness * Roughness / 2.0;
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


vec3 SpecularIBL_GroundTruth( vec3 SpecularColor, float Roughness, vec3 N, vec3 V )
{
    vec3 SpecularLighting = vec3(0.0, 0.0, 0.0);
    int NumSamples = 8;//1024;
    for( int i = 0; i < NumSamples; i++ )
    {
        for( int j = 0; j < NumSamples; j++ )
        {
            //vec2 Xi = Hammersley( i, NumSamples );
            float temp_a = float(i) / float(NumSamples);
            float temp_b = float(j) / float(NumSamples);
            vec2 Xi = vec2(temp_a, temp_b);
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
                float G = G_Smith( Roughness, NoV, NoL );
                float Fc = pow( 1.0 - VoH, 5.0 );
                vec3 F = (1.0 - Fc) * SpecularColor + Fc;
                // Incident light = SampleColor * NoL
                // Microfacet specular = D*G*F / (4*NoL*NoV)
                // pdf = D * NoH / (4 * VoH)
                SpecularLighting += SampleColor * F * G * VoH / (NoH * NoV);
            }
        }
    }
    return SpecularLighting / float(NumSamples * NumSamples);
}


//http://blog.selfshadow.com/publications/blending-in-detail/
vec3 blend_rnm(vec3 n1, vec3 n2) //normal before expand//normal from normal map
{
    vec3 t = n1.xyz*vec3( 2.0,  2.0, 2.0) + vec3(-1.0, -1.0,  0.0);
    vec3 u = n2.xyz*vec3(-2.0, -2.0, 2.0) + vec3( 1.0,  1.0, -1.0);
    vec3 r = t*dot(t, u) - u*t.z;
    return normalize(r);
}

vec3 blend_linear(vec3 n1, vec3 n2) //n1, n2 from tex2D(tex,   uv).xyz
{
    // vec3 r = (n1 + n2) * 2.0 - 2.0;
    // return normalize(r);    //already packed and normalized, ready for rendering

    // return normalize(r)*0.5 + 0.5;
    // float3 n1 = tex2D(texBase,   uv).xyz*2 - 1;
    // float3 n2 = tex2D(texDetail, uv).xyz*2 - 1;
    // float3 r  = normalize(n1 + n2);
    
    n1 =  lerp3(vec3(0.0, 0.0, 1.0), expand(n1), u_wetness);
    n2 = expand(n2);
    vec3 r  = normalize(n1 + n2);
    return r;
}


// vec3 blend_udn(vec3 n1, vec3 n2)
// {
//     vec3 c = vec3(2.0, 1.0, 0.0);
//     vec3 r;
//     r = n2*c.yyz + n1.xyz;
//     r =  r*c.xxx -  c.xxy;
//     return normalize(r);
// }
vec3 blend_udn(vec3 N1, vec3 N2) //normal after expand
{
    return normalize(vec3(N1.xy + N2.xy, N1.z));
}


vec3 wetness_normal(float wetness) //parameters inside this func
{
    // vec3 highFreqNormal = highFreqTex.Sample(Sampler, texcoord).rgb;
    // vec3 lowFreqNormal = lowFreqTex.Sample(Sampler, lerp(0.4, 0.2, wetness) * texcoord).rgb;
    // vec3 normal = RNMBlend(lerp(vec3(0.0, 0.0, 1.0), lowFreqNormal, wetness), highFreqNormal);

    vec3 highFreqNormal = (texture2DLod(normalMap, v_tex_coord, highFreqNormalLod).rgb);//texture2DLod(highFreqTexMap, v_tex_coord, 0.0).rgb;
    vec3 lowFreqNormal =  (texture2DLod(lowFreqNormalMap, lerp(lowFreqUpBound, lowFreqDownBound , wetness) * v_tex_coord, lowFreqNormalLod).rgb);//texture2DLod(lowFreqTexMap, lerp(0.4, 0.2, wetness) * v_tex_coord, 0.0).rgb;
    // lowFreqNormal = lerp3(vec3(0.0, 0.0, 1.0), lowFreqNormal, wetness);
    // lowFreqNormal = lerp3(vec3(0.0, 0.0, 1.0), lowFreqNormal, wetness);

    vec3 normal = blend_linear(lowFreqNormal, highFreqNormal); //blend normalmap

    //vec3 normal = blend_udn(expand(lerp3(vec3(0.0, 0.0, 1.0), lowFreqNormal, wetness)), expand(highFreqNormal));
    return normal;//normal;
}


float halfLambert (vec3 N, vec3 L) //Valve's Wrapped Diffuse Function
{
    return pow(dot(N,L) * 0.5 + 0.5, 2.0) ;
}

// #define NO_USING_NORMAL_MAP
void main()                    
{
    //-------------read normal from texture
    vec3 TS_N = normalize(expand(texture2DLod(normalMap, v_tex_coord, 0.0).xyz));
    
    float method = 0.0;
    if(method == 0.0){
        if(texture2D(specularLevelMap, v_tex_coord).x < 0.95)  //眼白
            TS_N = wetness_normal(u_wetness);  //parameter
    }
    else if (method == 1.0)
    {
       if(texture2D(specularLevelMap, v_tex_coord).x < 0.95)  //眼白
            TS_N *= 2.0;
    }

    // TS_N = normalize(expand(vec3(0.0, 0.0, 1.0)));


    /////////////---------- transfer N from tangent space to world space  ----------/////////////
    // vec3 v_WS_normal_normalized =  normalize((worldIT * vec4(normalize(v_WS_normal), 0.0)).xyz);
    // vec3 v_WS_tangent_normalized = normalize((world *   vec4(normalize(v_WS_tangent),0.0)).xyz);
    vec3 v_WS_normal_normalized = normalize(v_WS_normal);
    vec3 v_WS_tangent_normalized = normalize(v_WS_tangent);

    // v_WS_tangent_normalized = normalize(v_WS_tangent_normalized - v_WS_normal_normalized * v_WS_tangent_normalized * v_WS_normal_normalized);
    //WS->TS rotation
    vec3 binormal = normalize(cross(v_WS_normal_normalized, v_WS_tangent_normalized));
    mat3 rotation = mat3(vec3(v_WS_tangent_normalized[0], binormal[0], v_WS_normal_normalized[0]),  //first coloum
                         vec3(v_WS_tangent_normalized[1], binormal[1], v_WS_normal_normalized[1]),  //second coloum
                         vec3(v_WS_tangent_normalized[2], binormal[2], v_WS_normal_normalized[2])); //third coloum
    //TS->WS inv_rotation
    mat3 inv_rotation = transpose(rotation);
    //vec3 WS_N = normalize(inv_rotation * (expand(texture2D(normalMap, v_tex_coord).xyz)));        //TS --> WS
    vec3 WS_N = normalize(inv_rotation * (TS_N));        //TS --> WS ?
    // vec3 WS_N = normalize(TS_N * inv_rotation);        //TS --> WS ?  //http://www.ogre3d.org/tikiwiki/Accurate+per-pixel+cube+mapping+with+normal+map+influence
    // if(v_tex_coord.y > 0.5) WS_N = normalize(inv_rotation * (expand(texture2DLod(normalMap, v_tex_coord, 5.0).xyz)));        //TS --> WS

    vec3 WS_Diff_N = normalize(inv_rotation * normalize(expand(texture2DLod(normalDiffMap, v_tex_coord, 0.0).xyz)));        //TS --> WS
    

    //web wrong
    // mat3 TM = mat3( normalize( v_WS_tangent ),
    //                 normalize( v_WS_binormal ),
    //                 normalize( v_WS_normal ));
    // mat3 RM = mat3( world[0].xyz,
    //                 world[1].xyz,
    //                 world[2].xyz );
    // WS_N = normalize( RM * ( TS_N * TM ) );
    // vec3 TS_Diff_N =normalize(expand(texture2DLod(normalDiffMap, v_tex_coord, 0.0).xyz));
    // WS_Diff_N = normalize( RM * ( TS_Diff_N * TM ) );
    //web end


    /////////////---------- transfer N from tangent space to world space END  ----------/////////////
#ifdef NO_USING_NORMAL_MAP
    WS_N = v_WS_normal_normalized;
    WS_Diff_N = v_WS_normal_normalized;
#endif
    vec3 WS_P = v_WS_vertex.xyz; 
    vec3 WS_V = normalize(eyePosition.xyz - WS_P * eyePosition.w);


    //--specular
	// vec3 WS_R = normalize(reflect(WS_V, WS_N)); //wrong
    vec3 WS_R = normalize(2.0 * dot( WS_V, WS_N ) * WS_N - WS_V);
	// WS_R.y = - WS_R.y;

    //web
    // WS_R = normalize( reflect( normalize( WS_P  * eyePosition.w - eyePosition.xyz ), WS_N));
    //web end

    float specular_level  = 1.0; //眼白
    float roughness = u_cube_roughness * pow(u_wetness - 1.0, 6.0);
    float cube_map_lod = 8.0 * roughness;
    if(texture2D(specularLevelMap, v_tex_coord).x > 0.95){
        specular_level = 1.5; 
        cube_map_lod = 0.0;
    }
	// vec3 spcColor =  vec3(u_cube_specular_light_intensity) * specular_level * textureCubeLod(sampleEnvMap,  WS_R, 0.0).rgb * fresnel(WS_N, WS_R, vec3(0.31));   //F0  water:0.15 glass 0.21
    vec3 spcColor =  vec3(u_cube_specular_light_intensity) * specular_level * textureCubeLod(sampleEnvMap,  WS_R, cube_map_lod).rgb * fresnel(WS_N, WS_R, vec3(0.31));   //F0  water:0.15 glass 0.21
    vec3 EnvBRDF =  EnvBRDFApprox( vec3(1.0), roughness, dot(WS_N, WS_V) );
    spcColor *= EnvBRDF;

    //pbr
    // vec3 ks = vec3(1.0);
    // float Roughness = 0.0;
    //vec3 specularColorIBL = specular_level * PrefilteredMobileApproximateSpecularIBL(ks, Roughness, WS_N, WS_V);
    //vec3 specularColorIBL = SpecularIBL_GroundTruth(ks, Roughness, WS_N, WS_V);

    //spcColor = specularColorIBL;
    //pbr end


    //--diffuse
    vec3 light_dir = normalize(lightPosition.xyz - WS_P * lightPosition.w);
	vec3 diffColor = vec3(u_cube_diffuse_light_intensity) * clamp(dot(WS_Diff_N, light_dir), 0.0, 1.0);
    // if(texture2D(specularLevelMap, v_tex_coord).x > 0.95)
    //         diffColor = vec3(1.0);
    
    vec3 ambientColor = vec3(0.3);
    vec3 baseColor = texture2DLod(colorMap, v_tex_coord, 0.0).rgb;
    gl_FragColor.rgb = (ambientColor +  diffColor) * baseColor + spcColor;
    // gl_FragColor.rgb =  TS_N;
    // gl_FragColor.rgb =  spcColor;
    gl_FragColor.a = 1.0;
    // gl_FragColor =  1.5 * vec4(envColor, 1.0);

	//gl_FragColor = texture2D(normalMap, v_tex_coord);//vec4(WS_N, 1.0);
}
