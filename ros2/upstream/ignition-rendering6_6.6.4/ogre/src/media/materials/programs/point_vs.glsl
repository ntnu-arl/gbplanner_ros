#version 120

// Generic vertex shader for point sprites
// sets position and point size.
// Works for perspective and orthographic projection.

uniform mat4 worldviewproj_matrix;
uniform float size;

void main()
{
  gl_Position = worldviewproj_matrix * gl_Vertex;
  gl_FrontColor = gl_Color;
  gl_PointSize = size;
}
