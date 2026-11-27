varying float depth;

void main()
{
  // arbitrary camera far and near clip plane values for normalizing depth data
  // to produce a nice depth camera visualization effect
  float far = 10.0;
  float near = 0.1;
  gl_FragColor = vec4(vec3(depth / (far - near)), 1.0);

}
