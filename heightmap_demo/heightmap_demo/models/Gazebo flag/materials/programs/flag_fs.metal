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

struct Params
{
  float3 light_dir;
};

struct PS_INPUT
{
  float3 pos;
  float3 norm;
  float2 texUv;
  float flip;
};

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float> texMap [[texture(0)]],
  sampler texMapSampler [[sampler(0)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  float3 lightAmbient = float3(0.1f, 0.1f, 0.1f);
  float3 lightDiffuse = float3(0.7f, 0.7f, 0.7f);
  float3 lightSpecular = float3(0.1f, 0.1f, 0.1f);

  float3 color = float3(0.8, 0.8, 0.8);
  float3 ambientColor = color * 0.5;
  float3 diffuseColor = texMap.sample(texMapSampler, inPs.texUv.xy).xyz;
  float3 specularColor = color;

  float3 lightDir = normalize(p.light_dir);
  if (inPs.flip > 0)
  {
    // not ideal but make sure the other side is always lit
    lightDir = -lightDir;
  }

  // normalize both input floattors
  float3 n = normalize(inPs.norm);
  float3 e = normalize(-inPs.pos);

  float specular = 0.0f;
  float NdotL = max(dot(lightDir, inPs.norm), 0.0);
  // if the vertex is lit compute the specular color
  if (NdotL> 0.0)
  {
    // compute the half floattor
    float3 halfVector = normalize(lightDir + e);

    // add specular
    float NdotH = max(dot(halfVector, n), 0.0);

    float shininess = 10.0;
    specular = pow(NdotH, shininess);
  }

  float3 finalColor = lightAmbient * ambientColor;
  finalColor += lightDiffuse * diffuseColor * NdotL;
  finalColor += lightSpecular * specularColor * specular;

  // account for gamma correction
  finalColor = finalColor * finalColor;

  return float4(finalColor, 1.0);
}
