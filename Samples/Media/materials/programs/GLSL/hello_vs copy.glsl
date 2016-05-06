uniform mat4 worldViewProj;

void main()
{
    gl_Position = worldViewProj * gl_Vertex;
}
