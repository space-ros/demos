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

#version 330

in vec4 vertex;
in vec4 uv0;

// time
uniform float t;

// constants
uniform mat4 worldview_matrix;
uniform mat4 worldviewproj_matrix;
uniform mat4 inverse_transpose_world_matrix;

uniform float amplitude;
uniform float speed;
uniform float frequency;
uniform vec2 size;

// the flag is composed of two meshes. One with flipped normals.
// Here we need to change the shaders logic for deforming vertices and
// computing deformed normals
// 1 to flip vertex deformation and normal
uniform int flip;

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec3 pos;
  vec3 norm;
  vec2 texUv;
  float flip;
} outVs;

vec4 deform(vec4 v, vec2 uv)
{
  // requires flag mesh to be facing z direction
  v.z += sin((uv.x - t * speed) * frequency) * amplitude * uv.x;
  v.x += cos((uv.y - t * speed) * frequency) * amplitude*0.3 * uv.x;
  return v;
}

void main()
{
  // flip vertex deformation and normal calculation if needed
  vec2 uv = uv0.xy;
  float incr = 1.0;
  if (flip > 0)
  {
    uv = vec2(1 - uv.x, uv.y);
    incr = -1.0;
  }
  vec4 v = deform(vertex, uv);

  // compute deformed normal
  vec4 vertexX = vec4((uv.x + (0.01*incr) - 0.5) * size.x, vertex.yzw);
  vec4 vertexY = vec4(vertex.x, (uv.y + 0.01 - 0.5) * -size.y, vertex.zw);
  vec4 va = deform(vertexX, vec2(uv.x + (0.01 * incr), uv.y));
  vec4 vb = deform(vertexY, vec2(uv.x, uv.y + 0.01));
  vec3 norm = cross(normalize(vb.xyz - v.xyz), normalize(va.xyz - v.xyz));
  norm = normalize(norm);

  gl_Position = worldviewproj_matrix * v;

  vec4 p = worldview_matrix * v;
  outVs.pos = p.xyz / p.w;
  outVs.norm = vec3((inverse_transpose_world_matrix) * vec4(norm, 1.0));
  outVs.texUv = uv0.xy;
  outVs.flip = flip;
}

