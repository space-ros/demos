# Heightmap demo


This demo shows how to use custom DEM data from LRO & MRO to create performant terrain with Gazebo **heightmaps**.



## Build container:

```
cd heightmap_demo
./build.sh
```

## Run container:

```
export GZ_SIM_RESOURCE_PATH=`pwd`/models:$GZ_SIM_RESOURCE_PATH

gz sim -v 4 "apollo_16.sdf"
```


# Workflow

Digital elevation model (DEM) can be used in Gazebo as heightmaps. Example of such DEMs is data from [Lunar Reconaisance Orbiter's Apollo missions](https://wms.lroc.asu.edu/lroc/view_rdr/NAC_DTM_APOLLO16_4). DEM or DTM(Digital Terrain Model) is represented in float geotiff **.tiff** format.

Gazebo accepts data in  **.png** format, therefore we need to do a few data processing steps.
First, we need to crop image to $(2^k,2^k)$ size, for example $(4096,4096)$.

For this we use free and open source tool [QGIS](https://qgis.org/).

We load downloaded geotiff and use **layer->clip by extent**.

Then we save squar tiff image.

Values of tiff images are **32-bit float** representing **elevation** and since Gazebo heightmap accepts png, we need to convert it, to **16-bit** unsigned integer.

```
gdal_translate -outsize 4096 4096 -ot UInt16 -scale <min> <max>  0 65535 <input>.tif <output>.tif
```

You can gen min,max of extent in QGIS or with following command:
```
gdalinfo <input>.tif | grep Min
```

`gdal_translate` can also output .png format, but for some reason only 8bit version works, while 16-bit is not recognized by Gazebo and throws [out of bound error](https://github.com/gazebosim/garden_demo/pull/22#issuecomment-2322324064). This is probably due to wrong .png chunk encodings.

We can circumvent this by importing .tif into FOSS image editor [Gimp](https://www.gimp.org/) which outputs correct .png file, by using **export->png->16bpc**.



## References

The idea for this demo comes from beautiful [Garden demo](https://github.com/gazebosim/garden_demo).
This demo descibes how the heightmap can be generated from publicly available data and using open source software. 


