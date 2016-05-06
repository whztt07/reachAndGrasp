#version 120
#extension GL_ARB_shader_texture_lod : require
//#define SPHERE_TESTING

// varying vec4 v_vertex;
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



// uniform mat4 world;
// uniform mat4 worldIT;
// uniform vec4 lightPosition;
uniform vec4 lightColour;
// uniform vec4 eyePosition;
uniform float u_light_diffuse_intensity;
//uniform vec3 u_light_diffuse_color;
uniform float u_light_specular_intensity;
//uniform vec3 u_specularLightColor;
uniform float u_light_ambient_intensity;

//uniform float u_roughness;
//uniform float u_MipUpBound;
uniform float u_CurvatureScale;
uniform float u_lowNormalBias;
uniform float u_tuneNormalBlurRed;
uniform float u_tuneNormalBlurGreen;
uniform float u_tuneNormalBlurBlue;
uniform float u_skin_diffuse;
uniform float u_specular_power;
uniform float u_rendering_mode;
uniform float u_rendering_mode_number;
uniform float u_normal_map_blur_level;
uniform float alpha;

uniform float final_exposure;

//uniform sampler2D hammersleyMap;
//uniform samplerCube sampleEnvMap;
//uniform samplerCube prefilteredSampleEnvMap;
uniform sampler2D normalMap;
uniform sampler2D preIntegDiffuseMap;
uniform sampler2D colorMap;
//uniform sampler2D preIntegShadowMap;
uniform sampler2D curvatureMap;
uniform sampler2D specularMap;


float curv_max_bake_texture = 4.0;//0.5;




vec3 pow(vec3 a, float b)
{
    vec3 result = vec3(pow(a.x, b), pow(a.y, b), pow(a.z, b));
    return result;
}

vec3 gamma1over22 (vec3 in_value)
{
  //if (any(in_value)&&linearLight) return pow(in_value,1/2.2); //seems pow() returns 1 if it evaluates to zero ??? - use any() to see if any component of float3 is non-zero
  //return in_value;
  return pow(in_value,1/2.2);
}
vec3 gamma22 (vec3 in_value)
{
  // if (any(input)&&linearLight) return pow(input,2.2); //seems pow() returns 1 if it evaluates to zero ??? - use any() to see if any component of float3 is non-zero
  // return input;
  return pow(in_value, 2.2);

}

vec3 filmicTonemap(vec3 in_value)  //John Hable's filmic tonemap function with fixed values
{
  float A = .22;
  float B = 0.3;
  float C = 0.1;
  float D = 0.2;
  float E = 0.01;
  float F = 0.3;
  float linearLightWhite = 11.2;
  vec3 Fcolor = ((in_value*(A*in_value+C*B)+D*E)/(in_value*(A*in_value+B)+D*F)) - E/F;
  float  Fwhite = ((linearLightWhite*(A*linearLightWhite+C*B)+D*E)/(linearLightWhite*(A*linearLightWhite+B)+D*F)) - E/F;
  return Fcolor/Fwhite;
}


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
   vec3 x = max(0.0, color.xyz-0.004);
   vec3 retColor = (x*(6.2*x+.5))/(x*(6.2*x+1.7)+0.06);
   return vec4(retColor,1.0);
}

// Expand a range-compressed vector for normal map
vec3 expand(vec3 v)
{
    return (v - 0.5) * 2.0; 
}

vec3 SkinDiffuse(float curv, vec3 NdotL3)
{
    //DiffuseScatteringOnRing.png 
    vec3 lookup = NdotL3 * 0.5 + 0.5;
    //curv = curv * dot( lightColour.rgb, vec3(0.22, 0.707, 0.071 ) ); //Multiplied by light's luminosity so that brighter light leads to more scattering.
    // curv = 1.0 - curv/curv_max_bake_texture;  //remap
    vec3 diffuse;
    // diffuse.r =  texture2D( preIntegDiffuseMap, vec2(lookup.r, curv) ).r; 
    // diffuse.g =  texture2D( preIntegDiffuseMap, vec2(lookup.g, curv) ).g;
    // diffuse.b =  texture2D( preIntegDiffuseMap, vec2(lookup.b, curv) ).b;
    
    diffuse.r =  filmic(texture2D( preIntegDiffuseMap, vec2(lookup.r, curv) ), alpha).r; 
    diffuse.g =  filmic(texture2D( preIntegDiffuseMap, vec2(lookup.g, curv) ), alpha).g;
    diffuse.b =  filmic(texture2D( preIntegDiffuseMap, vec2(lookup.b, curv) ), alpha).b;
    return diffuse;
}

//	skin diffuse color using preintegration skin shading
vec3 skinDiffuseColor(vec3 N, vec3 L)
{
    //---- normal 
    vec3 tuneNormalBlur = vec3 (u_tuneNormalBlurRed, u_tuneNormalBlurGreen, u_tuneNormalBlurBlue);//vec3(0.1, 0.6, 0.7);
    vec3 N_high 	= 	N; //expand(texture2D(normalMap, v_texCoord).xyz);
    vec3 N_low 		= 	expand(texture2D(normalMap, v_texCoord, u_lowNormalBias).xyz);
    vec3 N_red		=   normalize(mix(N_high, N_low, tuneNormalBlur.r));
    vec3 N_green	= 	normalize(mix(N_high, N_low, tuneNormalBlur.g));
    vec3 N_blue 	=  	normalize(mix(N_high, N_low, tuneNormalBlur.b));
    vec3 NoL3 		= 	vec3(dot(N_red, L), dot(N_green, L), dot(N_blue, L));  //for R G B 
    //---- normal END

    // float curvature = abs(2.0*(texture2D(curvatureMap, v_texCoord).x - 0.5)) * u_CurvatureScale;
    float curvature = texture2D(curvatureMap, v_texCoord).x * u_CurvatureScale;

    curvature = u_CurvatureScale;

	#ifdef SPHERE_TESTING 
		curvature = u_CurvatureScale;
	#endif
    vec3 skinDiffuseColor = SkinDiffuse(curvature, NoL3) * lightColour.rgb ;

    return skinDiffuseColor;
}



void main()
{
	vec3 N = (expand(texture2D(normalMap, v_texCoord).xyz));
	#ifdef SPHERE_TESTING 
	     N = v_TSNormal;
	#endif
    vec3 V = normalize(v_TSViewDir);
    vec3 L = normalize(v_TSLightDir);
    vec3 H = normalize(v_TSHalfAngle);
   
    //---- diffuse
    vec3 diffuse = vec3(0.0);
    vec3 lightDiffuse = vec3(1.0);
    if (u_skin_diffuse >0.5)
      diffuse = skinDiffuseColor(N, L) * u_light_diffuse_intensity;
    else
      diffuse = lightDiffuse * clamp(dot(N, L), 0.0, 1.0) * u_light_diffuse_intensity;
    //---- diffuse END

    //diffuse = filmic(vec4(diffuse, 1.0), alpha).rgb;

    //---- specular
    float NoH = dot(N, H);
    float po = pow( clamp(NoH , 0.0, 1.0), u_specular_power);
    vec3 specularLightColor =  vec3(1.0); //texture2D(colorMap, v_texCoord).rgb; //vec3(0.5);//
    // float specularLevel =  texture2D(normalMap, v_texCoord).a; //小偶的数据
    // vec3 specular = u_light_specular_intensity * specularLightColor * specularLevel * po;
    vec3 specular = u_light_specular_intensity * specularLightColor * po;
    vec3 ks = texture2D( specularMap, v_texCoord).rgb;
    specular = ks * specular;
    //---- specular END


    //----add everything
    vec3 ambient = vec3 (u_light_ambient_intensity);
    vec4 ambint_diffuse = vec4 (ambient + diffuse, 1.0);
    vec4 _base = texture2D(colorMap, v_texCoord);
	vec4 base =  (_base);
	base.rgb = (gamma22(base.rgb));
    //base.rgb =  gamma1over22(base.rgb);

	// if(v_texCoord.x > 0.4 && v_texCoord.x < 0.5) 	base.rgb = gamma22(gamma22(_base.rgb));
    // if(v_texCoord.x > 0.6) 							base.rgb = gamma1over22(_base.rgb); 
	
	
	
    vec4 outcolor = base * ambint_diffuse + vec4(specular, 1.0);
    //vec4 outcolor = alpha*base + (1.0-alpha)*ambint_diffuse + vec4(specular, 1.0);
    //vec4 outcolor = vec4(diffuse, 1.0) + vec4(base.rgb * ambient, 1.0) + vec4(specular, 1.0);
	
    outcolor.rgb = filmicTonemap(outcolor.rgb * final_exposure);

	
    outcolor.rgb = gamma1over22(outcolor.rgb);
    outcolor.a = base.a;

	
    if(u_rendering_mode == 0.0){ //color
        gl_FragColor = outcolor;
        //gl_FragColor = vec4( diffuse, 1.0);
    }
    else if(u_rendering_mode == 1.0) { //curvature
  	    float curvature = texture2D(curvatureMap, v_texCoord).x * u_CurvatureScale;
    // float curvature = abs(2.0*(texture2D(curvatureMap, v_texCoord).x - 0.5)) * u_CurvatureScale;
        gl_FragColor = vec4(1.0) * curvature;
    }
     else if(u_rendering_mode == 2.0) { 
         gl_FragColor =  filmic(texture2D( preIntegDiffuseMap,  v_texCoord), alpha); 
     }
     else if(u_rendering_mode == 3.0) { 
        gl_FragColor = vec4( diffuse, 1.0);
     }
    else{
    	gl_FragColor = vec4(1.0);
    }
}



