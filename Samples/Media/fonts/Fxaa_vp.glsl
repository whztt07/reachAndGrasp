uniform mat4 worldViewProj;
attribute vec4 vertex;
attribute vec2 uv0;
varying vec2 oUv0;

void main()
{
    gl_Position = worldViewProj * vertex;
    oUv0 = uv0;
}

