varying vec4 v_vertex;
varying vec3 v_norm;
varying vec2 TexCoordOut; // New


uniform mat4 world;
uniform mat4 worldIT;
//uniform mat4 worldViewProj;
uniform vec4 lightPosition;
uniform vec4 lightColour;
uniform vec4 eyePosition;
uniform sampler1D sample_1D_A;
uniform sampler2D sample_2D_A;
uniform sampler2D sample_2D_B;
uniform samplerCube sampleEnvMap;

uniform sampler2D hammersleyMap;



vec2 Hammersley(int i, int N)
{
    vec4 temp = texture2D( hammersleyMap, vec2( float(i) / float(N),0 ));
    return vec2(temp.x, temp.y);
}

void main()
{
    //final color
    //gl_FragColor =vec4( Hammersley(10,128).x, 0.0, 0.0, 1.0);
    //vec4 texel = vec4( 1.0, 0.0, 0.0, 0.0);
    //vec4 texel  = texture2D(sample_2D_A, TexCoordOut) + texture2D(sample_2D_B, TexCoordOut);
    vec4 one_d = texture1D(sample_1D_A, 0.6);
    //vec4 aa = texture2D(sample_2D_A, TexCoordOut);
    //vec4 texel  = textureCube(sampleEnvMap, vec3(TexCoordOut, 0.0))
                  //+texture2D(sample_2D_A, TexCoordOut)
                  //  + texture2D(sample_2D_B, TexCoordOut)
                    //;
    
    //vec2 bb = Hammersley(95,100);
    
    gl_FragColor = one_d;//vec4(0, bb.g,  0, 1.0);
 }
