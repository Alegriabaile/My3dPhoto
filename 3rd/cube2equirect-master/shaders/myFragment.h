#version 330 core
uniform sampler2D colorTexture0;//front
uniform sampler2D colorTexture1;//right
uniform sampler2D colorTexture2;//back
uniform sampler2D colorTexture3;//left                               
uniform sampler2D colorTexture4;//top
uniform sampler2D colorTexture5;//bottom

uniform sampler2D depthTexture0;
uniform sampler2D depthTexture1;
uniform sampler2D depthTexture2;
uniform sampler2D depthTexture3;
uniform sampler2D depthTexture4;
uniform sampler2D depthTexture5;

in vec2 UV;

layout(location = 0) out vec4 colorAttachment;
layout(location = 1) out vec4 depthAttachment;

void main()
{
  float theta = UV.x * M_PI;
  float phi = (UV.y * M_PI) / 2.0;

  float x = cos(phi) * sin(theta);
  float y = sin(phi);
  float z = cos(phi) * cos(theta);

  float scale;
  vec2 px;
  vec4 src;
  vec4 src_depth;

  if (abs(x) >= abs(y) && abs(x) >= abs(z)) {
    if (x < 0.0) {
      scale = -1.0 / x;
      px.x = ( z*scale + 1.0) / 2.0;
      px.y = ( y*scale + 1.0) / 2.0;
      src = texture(colorTexture3, px);
      src_depth = texture(depthTexture3, px);
    }
    else {
      scale = 1.0 / x;
      px.x = (-z*scale + 1.0) / 2.0;
      px.y = ( y*scale + 1.0) / 2.0;
      src = texture(colorTexture1, px);
      src_depth = texture(depthTexture1, px);
    }
  }
  else if (abs(y) >= abs(z)) {
    if (y < 0.0) {
      scale = -1.0 / y;
      px.x = ( x*scale + 1.0) / 2.0;
      px.y = ( z*scale + 1.0) / 2.0;
      src = texture(colorTexture4, px);
      src_depth = texture(depthTexture4, px);
    }
    else {
      scale = 1.0 / y;
      px.x = ( x*scale + 1.0) / 2.0;
      px.y = (-z*scale + 1.0) / 2.0;
      src = texture(colorTexture5, px);
      src_depth = texture(depthTexture4, px);
    }
  }
  else {
    if (z < 0.0) {
      scale = -1.0 / z;
      px.x = (-x*scale + 1.0) / 2.0;
      px.y = ( y*scale + 1.0) / 2.0;
      src = texture(colorTexture2, px);
      src_depth = texture(depthTexture4, px);
    }
    else {
      scale = 1.0 / z;
      px.x = ( x*scale + 1.0) / 2.0;
      px.y = ( y*scale + 1.0) / 2.0;
      src = texture(colorTexture0, px);
      src_depth = texture(depthTexture4, px);
    }
  }

  colorAttachment = src;
  depthAttachment = src_depth;
}
