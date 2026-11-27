varying float depth;

void main()
{
  gl_Position = ftransform();
  vec4 point = gl_ModelViewMatrix * gl_Vertex;
  depth = -point.z;
}

