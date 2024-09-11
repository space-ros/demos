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
import rasterio
from rasterio.windows import Window
import numpy as np
import sys

def crop_dem(input_tif, output_tif, crop_window):
    """
    Crop a DEM TIFF file and save the cropped result.

    Parameters:
    - input_tif: Path to the input TIFF file.
    - output_tif: Path to save the cropped TIFF file.
    - crop_window: Tuple of (row_start, row_stop, col_start, col_stop) defining the crop area.
    """

    with rasterio.open(input_tif) as src:
        # Define the cropping window
        window = Window(crop_window[2], crop_window[0], crop_window[3] - crop_window[2], crop_window[1] - crop_window[0])

        # Read the data from the defined window
        data = src.read(window=window)

        # Define metadata for the cropped file
        meta = src.meta.copy()
        meta.update({
            'height': window.height,
            'width': window.width,
            'transform': src.window_transform(window)
        })

        # Write the cropped data to a new TIFF file
        with rasterio.open(output_tif, 'w', **meta) as dst:
            dst.write(data)

        # Find the maximum height in the cropped region
        max_height = np.nanmax(data)  
        min_height = np.nanmin(data)
        print(f"Height diff in cropped region: {max_height - min_height}")


if __name__ == '__main__':
    if len(sys.argv) != 6:
        print("Usage: python crop_tiff.py <input_tiff> <output_tiff> <top_left_x> <top_left_y> <length>")
        sys.exit(1)
    input_tif = sys.argv[1]
    output_tif = sys.argv[2]
    row_start = int(sys.argv[3])
    length = int(sys.argv[5])
    row_stop = int(row_start + length)
    col_start = int(sys.argv[4])
    col_stop = int(col_start + length)
    crop_window = (row_start, row_stop, col_start, col_stop)
    crop_dem(input_tif, output_tif, crop_window)