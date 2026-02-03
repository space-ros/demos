#  Copyright 2024 Element Robotics Pty Ltd
# 
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
# 
#      http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
import numpy as np
from PIL import Image
import OpenEXR
import Imath
import sys

def load_exr_normal_map(path):
    # Open the EXR file
    file = OpenEXR.InputFile(path)

    # Get the header and data window to determine the image size
    header = file.header()
    dw = header['dataWindow']
    size = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)

    # Define the channels to extract (R, G, B)
    channels = ['R', 'G', 'B']

    # Extract and convert each channel
    pt = Imath.PixelType(Imath.PixelType.FLOAT)
    data = [np.frombuffer(file.channel(c, pt), dtype=np.float32).reshape(size[1], size[0]) for c in channels]

    # Stack the channels into an (H, W, 3) numpy array and normalize to [-1, 1]
    normal_map = np.stack(data, axis=-1)
    normal_map = normal_map * 2.0 - 1.0

    return normal_map

def load_normal_map(path):
    img = Image.open(path).convert('RGB')
    normals = np.array(img).astype(np.float32) / 255.0
    normals = normals * 2 - 1  # Map from [0, 1] to [-1, 1]
    return normals

def save_normal_map(normals, path):
    normals = (normals + 1) / 2 * 255  # Map from [-1, 1] to [0, 1]
    normals = normals.astype(np.uint8)
    img = Image.fromarray(normals)
    img.save(path)
    print(f"Normal map saved as {path}")

def blend_normals(low_res_normals, high_res_normals):
    # Perform per-pixel normal blending
    combined_normals = np.zeros_like(low_res_normals)

    # Blend the x and y components
    combined_normals[..., 0] = low_res_normals[..., 0] * high_res_normals[..., 2] + high_res_normals[..., 0] * low_res_normals[..., 2]
    combined_normals[..., 1] = low_res_normals[..., 1] * high_res_normals[..., 2] + high_res_normals[..., 1] * low_res_normals[..., 2]
    
    # Blend the z component
    combined_normals[..., 2] = low_res_normals[..., 2] * high_res_normals[..., 2] - (low_res_normals[..., 0] * high_res_normals[..., 0] + low_res_normals[..., 1] * high_res_normals[..., 1])

    # Normalize the result to ensure unit length
    norm_magnitudes = np.sqrt(np.sum(np.square(combined_normals), axis=-1, keepdims=True))
    combined_normals /= norm_magnitudes
    return combined_normals

def resample_normal_map(normal_map, target_size):
    img = Image.fromarray(((normal_map + 1) / 2 * 255).astype(np.uint8))
    img_resized = img.resize(target_size, Image.BICUBIC)
    resized_normal_map = np.array(img_resized).astype(np.float32) / 255.0
    return resized_normal_map * 2 - 1  # Convert back to [-1, 1]

def tile_normal_map(normal_map, target_shape):
    tile_x = target_shape[1] // normal_map.shape[1] + 1
    tile_y = target_shape[0] // normal_map.shape[0] + 1
    tiled_map = np.tile(normal_map, (tile_y, tile_x, 1))
    return tiled_map[:target_shape[0], :target_shape[1]]

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: python blend_normals.py <low_res_path> <high_res_path> <output_path>")
        sys.exit(1)
    low_res_path = sys.argv[1]
    high_res_path = sys.argv[2]
    output_path = sys.argv[3]
    # Load your low-resolution and high-resolution EXR normal maps
    low_res_normals = load_normal_map(low_res_path)
    high_res_normals = load_exr_normal_map(high_res_path)

    # Resample or tile the high-resolution normal map to match the low-res map size
    low_res_normals_resized = resample_normal_map(low_res_normals, high_res_normals.shape[:2])

    # Blend the normal maps
    combined_normals = blend_normals(low_res_normals_resized, high_res_normals)

    # Save the resulting combined normal map
    save_normal_map(combined_normals, output_path)
