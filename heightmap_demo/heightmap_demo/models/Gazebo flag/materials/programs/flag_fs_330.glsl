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

uniform sampler2D texMap;
uniform vec3 light_dir;
out vec4 fragColor;

in block
{
  vec3 pos;
  vec3 norm;
  vec2 texUv;
  float flip;
} inPs;

void main()
{
  vec3 lightAmbient = vec3(0.1f, 0.1f, 0.1f);
  vec3 lightDiffuse = vec3(0.7f, 0.7f, 0.7f);
  vec3 lightSpecular = vec3(0.1f, 0.1f, 0.1f);

  vec3 color = vec3(0.8, 0.8, 0.8);
  vec3 ambientColor = color * 0.5;
  vec3 diffuseColor = texture(texMap, inPs.texUv.xy).xyz;
  vec3 specularColor = color;

  vec3 lightDir = normalize(light_dir);
  if (inPs.flip > 0)
  {
    // not ideal but make sure the other side is always lit
    lightDir = -lightDir;
  }

  // normalize both input vectors
  vec3 n = normalize(inPs.norm);
  vec3 e = normalize(-inPs.pos);

  float specular = 0.0f;
  float NdotL = max(dot(lightDir, inPs.norm), 0.0);
  // if the vertex is lit compute the specular color
  if (NdotL> 0.0)
  {
    // compute the half vector
    vec3 halfVector = normalize(lightDir + e);

    // add specular
    float NdotH = max(dot(halfVector, n), 0.0);

    float shininess = 10.0;
    specular = pow(NdotH, shininess);
  }

  vec3 finalColor = lightAmbient * ambientColor;
  finalColor += lightDiffuse * diffuseColor * NdotL;
  finalColor += lightSpecular * specularColor * specular;

  // account for gamma correction
  finalColor = finalColor * finalColor;

  fragColor = vec4(finalColor, 1.0);
}
