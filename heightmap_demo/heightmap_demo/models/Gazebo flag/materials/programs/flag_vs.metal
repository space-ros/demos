/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <metal_stdlib>
using namespace metal;

struct VS_INPUT
{
  float4 position [[attribute(VES_POSITION)]];
  float2 uv0 [[attribute(VES_TEXTURE_COORDINATES0)]];
};

struct Params
{
  // time
  float t;
  float4x4 worldview_matrix;
  float4x4 worldviewproj_matrix;
  float4x4 inverse_transpose_world_matrix;
  float amplitude;
  float speed;
  float frequency;
  float2 size;
  // the flag is composed of two meshes. One with flipped normals.
  // Here we need to change the shaders logic for deforming vertices and
  // computing deformed normals
  // 1 to flip vertex deformation and normal
  int flip;
};

struct PS_INPUT
{
  float4 gl_Position [[position]];
  float3 pos;
  float3 norm;
  float2 texUv;
  float flip;
};

float4 deform(float4 v, float2 uv, float speed, float frequency, float amplitude, float t)
{
  // requires flag mesh to be facing z direction
  v.z += sin((uv.x - t * speed) * frequency) * amplitude * uv.x;
  v.x += cos((uv.y - t * speed) * frequency) * amplitude*0.3 * uv.x;
  return v;
}

vertex PS_INPUT main_metal
(
  VS_INPUT input [[stage_in]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  PS_INPUT outVs;

  // flip vertex deformation and normal calculation if needed
  float2 uv = input.uv0.xy;
  float incr = 1.0;
  if (p.flip > 0)
  {
    uv = float2(1 - uv.x, uv.y);
    incr = -1.0;
  }
  float4 v = deform(input.position, uv,
      p.speed, p.frequency, p.amplitude, p.t);

  // compute deformed normal
  float4 vertexX = float4((uv.x + (0.01*incr) - 0.5) * p.size.x, input.position.yzw);
  float4 vertexY = float4(input.position.x, (uv.y + 0.01 - 0.5) * -p.size.y, input.position.zw);
  float4 va = deform(vertexX, float2(uv.x + (0.01 * incr), uv.y),
      p.speed, p.frequency, p.amplitude, p.t);
  float4 vb = deform(vertexY, float2(uv.x, uv.y + 0.01),
      p.speed, p.frequency, p.amplitude, p.t);
  float3 norm = cross(normalize(vb.xyz - v.xyz), normalize(va.xyz - v.xyz));
  norm = normalize(norm);

  outVs.gl_Position = p.worldviewproj_matrix * v;

  float4 pos = p.worldview_matrix * v;
  outVs.pos = pos.xyz / pos.w;
  outVs.norm = float3((p.inverse_transpose_world_matrix) * float4(norm, 1.0));
  outVs.texUv = input.uv0.xy;
  outVs.flip = p.flip;

  return outVs;
}

