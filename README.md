# hector_ground_truth_gazebo_plugins
Obtain a ground truth from your gazebo world

## Elevation Map
To obtain a ground truth elevation (alt. height) map, you can add the `ElevationMapGroundTruthPlugin` plugin to your world:

```
<sdf version='1.6'>
  <world name='default'>
    <!-- ... -->
    <plugin name="ElevationMapGroundTruthPlugin" filename="libhector_ground_truth_gazebo_plugins.so"/>
  </world>
</sdf>
```

This adds a service available at `/ground_truth/generate_elevation_map` which can be called to generate the ground truth at a given location.
The service has the following options:

* **origin:** A pose marking the center of the map. The orientation marks the direction of the rays used to determine the elevation. If the orientation is the identity, the map plane will be parallel to the X-Y-plane.
* **max_z_height:** This is the height (relative to the plane) in meters from where the raycast starts and therefore also the maximum representable height.
* **truncation_distance:** This is the maximum distance the ray may travel.
* **resolution:** The resolution of the resolution elevation map, i.e., the length of a cell in meter.
* **cells_x:** How many cells in x direction the resulting map should have.
* **cells_y:** How many cells in y direction the resulting map should have.
* **samples:** How many rays are sampled (in an evenly distributed grid) per dimension. E.g., 3 samples essentially mean a grid of 3x3 rays parallel to the transformed -z-axis is used to determine the elevation. The value of a cell is the highest elevation obtained by the samples.
* **quiet:** If false, the resulting elevation map is also published on the topic `ground_truth/elevation_map`. If true, it is not.

The service returns an elevation map in form of a `grid_map_msgs/GridMap` with the origin at (0, 0, 0).

#### Example:
```
rosservice call /ground_truth/generate_elevation_map "origin:
  position: {x: -1.4, y: -0.7, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
max_z_height: 2.0
truncation_distance: 4.0
resolution: 0.05
cells_x: 100
cells_y: 120
samples: 3
quiet: false" 
```
