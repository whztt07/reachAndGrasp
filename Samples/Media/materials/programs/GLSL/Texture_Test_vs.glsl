attribute vec4 vertex;
attribute vec3 normal;
//attribute vec2 TexCoordIn;


varying vec4 v_vertex;
varying vec3 v_norm;
varying vec2 TexCoordOut; // New


uniform mat4 worldViewProj;

void main()
{
    gl_Position = worldViewProj * vertex;
    v_norm = normal;
    v_vertex = vertex;
    gl_TexCoord[0] = gl_MultiTexCoord0;
    TexCoordOut = gl_TexCoord[0].xy;
}
