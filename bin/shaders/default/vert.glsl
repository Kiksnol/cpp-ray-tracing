/* */

#version 460                 

layout(location = 0) in vec3 InPosition;
layout(location = 1) in vec2 InTexCoord;
layout(location = 2) in vec3 InNormal;
layout(location = 3) in vec4 InColor;

uniform mat4 MatrWVP;

out vec4 DrawColor;
out vec3 DrawPos;
out vec2 DrawTexCoord;
out vec3 DrawNormal;


void main( void )
{
  gl_Position = vec4(0, 0, 0, 1);
//  DrawColor = vec4(InPosition, 1);
  //gl_Position = vec4(0, 0, 0, 1);
}
