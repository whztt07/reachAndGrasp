#version 120

uniform sampler2D RT;
uniform float NumTiles;
uniform float Threshold;
uniform vec2 front_uv;
uniform float u_odd;


// in vec2 oUv0;
varying vec2 oUv0;
// out vec4 fragColour;


// float mod_(float a, float b)
// {
//     return a - b * floor (a/b);
// }

// vec2 mod_v2(vec2 a, vec2 b)
// {
//     vec2 result;
//     result.x = mod_(a.x, b.x);
//     result.y = mod_(a.y, b.y);
//     return result;
// }



void tilling()
{
    vec3 EdgeColor = vec3(0.7, 0.7, 0.7);
    float size = 1.0/NumTiles;
    vec2 Pbase = oUv0 - mod(oUv0, vec2(size));
    vec2 PCenter = vec2(Pbase + (size/2.0));
    vec2 st = (oUv0 - Pbase)/size;
    vec4 c1 = vec4(0.0);
    vec4 c2 = vec4(0.0);
    vec4 invOff = vec4((1.0-EdgeColor),1.0);
    if (st.x > st.y) { c1 = invOff; }
    float threshholdB =  1.0 - Threshold;
    if (st.x > threshholdB) { c2 = c1; }
    if (st.y > threshholdB) { c2 = c1; }
    vec4 cBottom = c2;
    c1 = vec4(0.0);
    c2 = vec4(0.0);
    if (st.x > st.y) { c1 = invOff; }
    if (st.x < Threshold) { c2 = c1; }
    if (st.y < Threshold) { c2 = c1; }
    vec4 cTop = c2;
    // vec4 tileColor = vec4(texture(RT, PCenter));
    vec4 tileColor = vec4(texture2D(RT, PCenter));
    vec4 result = tileColor + cTop - cBottom;
    // fragColour = result;
    gl_FragColor = result;
}
void greyscale()
{
//        gl_FragColor = vec4(texture2D(RT, oUv0));
    vec3 greyscale = vec3(dot(texture2D(RT, oUv0).rgb, vec3(0.3, 0.59, 0.11)));
    gl_FragColor = vec4(greyscale, 1.0);
}

void main()
{
    // if(oUv0.x > front_uv.x  && oUv0.y > front_uv.y)
    // float odd =1.0;
    if(u_odd ==1.0)
    {
        if(oUv0.x + oUv0.y - front_uv.y > 0.0)
        {
        	tilling();
        }
        else
        {
            greyscale();
        }
    }
    else if(u_odd ==0.0)
    {
        if(oUv0.x + oUv0.y - front_uv.y < 0.0)
        {
            tilling();
        }
        else
        {
            greyscale();
        }
    }
    

}
