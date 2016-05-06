uniform float inverseViewportWidth;
uniform float inverseViewportHeight;
uniform sampler2D diffuseMap;
varying vec2 oUv0;

vec3 cFXAAParams = vec3(2.0,5.0,1.0);

float FXAA_SPAN_MAX = 8.0;
float FXAA_REDUCE_MUL = 1.0/8.0;
float FXAA_REDUCE_MIN = 1.0/128.0;
vec3 luma = vec3(0.299, 0.587, 0.114);
float p0 = 0.0/3.0 - 0.5;
float p1 = 1.0/3.0 - 0.5;
float p2 = 2.0/3.0 - 0.5;
float p3 = 3.0/3.0 - 0.5;

void main()
{
    vec2 cGBufferInvSize = vec2(inverseViewportWidth,inverseViewportHeight);
    
    vec2 posOffset = cGBufferInvSize.xy * cFXAAParams.x;
    
    vec4 rgbaM  = texture2D(diffuseMap, oUv0);
    vec3 rgbM  = rgbaM.rgb;
    vec3 rgbNW = texture2D(diffuseMap, oUv0 + vec2(-posOffset.x, -posOffset.y)).rgb;
    vec3 rgbNE = texture2D(diffuseMap, oUv0 + vec2(posOffset.x, -posOffset.y)).rgb;
    vec3 rgbSW = texture2D(diffuseMap, oUv0 + vec2(-posOffset.x, posOffset.y)).rgb;
    vec3 rgbSE = texture2D(diffuseMap, oUv0 + vec2(posOffset.x, posOffset.y)).rgb;
    
    float lumaNW = dot(rgbNW, luma);
    float lumaNE = dot(rgbNE, luma);
    float lumaSW = dot(rgbSW, luma);
    float lumaSE = dot(rgbSE, luma);
    float lumaM  = dot(rgbM,  luma);
    
    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    
    if (((lumaMax - lumaMin) / lumaMin) >= cFXAAParams.y)
    {
        vec2 dir;
        dir.x =  ((lumaNW + lumaNE) - (lumaSW + lumaSE));
        dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));
        
        float dirReduce = max(FXAA_REDUCE_MIN,
                              (lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * FXAA_REDUCE_MUL) );
        float rcpDirMin = 1.0/(min(abs(dir.x), abs(dir.y)) + dirReduce);
        dir = min(vec2( FXAA_SPAN_MAX,  FXAA_SPAN_MAX),
                  max(vec2(-FXAA_SPAN_MAX, -FXAA_SPAN_MAX), dir * rcpDirMin))
        * cGBufferInvSize.xy;
        
        dir *= cFXAAParams.z;
        
        gl_FragColor = 0.25 * (
                               texture2D(diffuseMap, oUv0 + dir * p0) +
                               texture2D(diffuseMap, oUv0 + dir * p1) +
                               texture2D(diffuseMap, oUv0 + dir * p2) +
                               texture2D(diffuseMap, oUv0 + dir * p3));
    }
    else
        gl_FragColor = rgbaM;
}
