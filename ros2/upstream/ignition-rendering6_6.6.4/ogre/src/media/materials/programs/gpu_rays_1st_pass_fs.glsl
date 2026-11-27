uniform float retro;

uniform float near;
uniform float far;
uniform float max;
uniform float min;

varying vec4 point;

void main()
{
  float l = length(point.xyz);

  if (l > far)
    l = max;
  if (l < near)
    l = min;

  gl_FragColor = vec4(l, retro, 0, 1.0);
}
