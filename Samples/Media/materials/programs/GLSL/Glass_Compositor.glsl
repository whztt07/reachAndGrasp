uniform sampler2D RT;
varying vec2 uv;

void main()
{
    gl_FragColor = texture(RT, uv);
    gl_FragColor.r = 1.0;
}
