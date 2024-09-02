# EM_simulator
Currently using ign gazebo to test EdyMobile path planning algorithms

## Gazebo World Creation
To re-create the track in the lab, or any other track consisting of a union of rectangular roads, go to `track_maker.xacro` and define the roads according to the following macro:

```xml
<xacro:road_segment name="road_name" x_min="2.87" y_min="1.67" x_max="3.52" y_max="13.67" height="0.1" z_position="0.05" />
```
changing the name and the minimum and maximum x and y values, which defines the AABB bounding boxes or the rectangular roads exactly. The finally, provide more user permissions if necessary, as docker creates the files as root.
```bash
xacro track_maker.xacro > road_network.sdf
```

Note that when including the `road_network.sdf` file in the main world file, the relatie path defined by `file://` is based on the working directory of the shell. Hence, an absolute path is provided. This should work if the ros2 workspace has been fully built, as instructed in the dockerfile. 

