#version 120

uniform sampler2D RT;
varying vec2 oUv0;
uniform float u_intensity;

void main()
{
    // vec3 greyscale = vec3(dot(texture(RT, oUv0).rgb, vec3(0.3, 0.59, 0.11)));
    // fragColour = vec4(greyscale, 1.0);
    // fragColour = vec4(1.0);

	vec3 RT_rgb = texture2D(RT, oUv0).rgb;
    vec3 greyscale = vec3(dot(RT_rgb, vec3(0.3, 0.59, 0.11)));
    gl_FragColor = 	u_intensity * vec4(RT_rgb, 1.0)
    			+ (1.0 - u_intensity) * vec4(greyscale, 1.0);

    // gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);

}
