
attribute vec4 vertex;
// attribute vec2 uv0;
varying vec2 oUv0;
varying vec2 oUv1;
varying vec4 pos;

uniform mat4 worldViewProj;

void main()
{
    // Use standardise transform, so work accord with render system
    // specific (RS depth, requires texture flipping, etc).
    gl_Position = worldViewProj * vertex;

    // The input positions adjusted by texel offsets, so clean up inaccuracies
    vec2 inPos = sign(vertex.xy);

    // Convert to image-space
    oUv0 = (vec2(inPos.x, -inPos.y) + 1.0) * 0.5;

    // oUv0 = uv0;
    oUv1 = -vertex.xy;
    pos = gl_Position;
}
