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
import rasterio
from rasterio.plot import show
from PIL import Image
import sys

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python surface_normals.py <input_tiff> <output_png>")
        sys.exit(1)
    # Step 1: Load the DEM from the TIFF file
    dem_path = sys.argv[1]
    output_image_path = sys.argv[2]
    with rasterio.open(dem_path) as src:
        dem = src.read(1)  # Read the first band
        transform = src.transform

    # Step 2: Calculate the gradients in x and y directions
    x, y = np.gradient(dem, transform[0], transform[4])

    # Step 3: Calculate the surface normals
    normals = np.zeros((dem.shape[0], dem.shape[1], 3))
    normals[..., 0] = -x  # -dz/dx
    normals[..., 1] = -y  # -dz/dy
    normals[..., 2] = 1   # dz/dz = 1

    # Step 4: Normalize the normals
    norm_magnitudes = np.sqrt(normals[..., 0]**2 + normals[..., 1]**2 + normals[..., 2]**2)
    normals[..., 0] /= norm_magnitudes
    normals[..., 1] /= norm_magnitudes
    normals[..., 2] /= norm_magnitudes

    # Step 5: Map normals to RGB values
    # Normalize to range [0, 255] for PNG saving
    normals_rgb = (normals + 1) / 2 * 255
    normals_rgb = normals_rgb.astype(np.uint8)

    # Step 6: Save as a PNG image
    img = Image.fromarray(normals_rgb)
    img.save(output_image_path)

    print(f"Surface normals saved as {output_image_path}")
