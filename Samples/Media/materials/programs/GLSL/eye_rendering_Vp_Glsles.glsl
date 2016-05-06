// #version 100
// precision highp int;
// precision highp float;

#version 120
#extension GL_ARB_shader_texture_lod : require

attribute	vec4	vertex;
attribute	vec3	normal;
attribute	vec3	tangent;
// attribute	vec4	uv0;

uniform	mat4 worldViewProj;
uniform mat4 world;
uniform mat4 worldIT;


varying	vec2 v_tex_coord;
varying vec4 v_WS_vertex;
varying vec3 v_WS_normal;
varying vec3 v_WS_tangent;
varying vec3 v_WS_binormal;

void main()
{
	gl_Position = worldViewProj * vertex;
	
	v_tex_coord = gl_MultiTexCoord0.xy;//uv0.xy;
    v_WS_vertex = world * vertex;

    v_WS_normal = normalize((worldIT * vec4(normal, 0.0)).xyz);
    v_WS_tangent = normalize((world * vec4(tangent, 0.0)).xyz);

      // v_WS_normal = normalize(normal);//normalize((worldIT * vec4(normal, 1.0)).xyz);
   //  v_WS_tangent = normalize(tangent);//normalize((world * vec4(tangent, 1.0)).xyz);
    // v_WS_binormal = cross(v_WS_normal, v_WS_tangent);
}

