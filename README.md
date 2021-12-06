# voxl-mapper/planner

3d Mapping and Path Planning

| :warning: WARNING          |
|:---------------------------|
| This project is only authorized for beta usage |

MAKE SURE YOU ARE FOLLOWING ALL PROPER SAFETY PROTOCOLS BEFORE ATTEMPTING AUTONOMOUS FLIGHT.

ModalAI is NOT responsible for any damages caused by flight path.

## Build Instructions

1. Start voxl-cross docker image
```
you@home-pc:~$ voxl-docker -i voxl-cross
```
2. Install build dependencies into docker environment
```
./install_build_deps.sh dev
```

3. Run build script from the root of project
```
./build.sh
```

4. Create an ipk
```
./make_package.sh
```

5. Push to voxl
```
./install_on_voxl.sh
```

## Requirements
voxl-mapper requires input from various MPA services in order to run.
1. voxl-vision-px4
   * Must be running, used for localization as well as trajectory following
2. voxl-qvio-server
    * Must be running, used by voxl-vision-px4 for localization
3. voxl-camera-server [TOF]
    * voxl-mapper requires some form of depth sensor input. As of now, the application is setup to take this depth input from a TOF sensor.

<br>
voxl-vision-px4 must be setup to receive trajectories from voxl-mapper if you would like to follow generated paths. To do this, edit the config file under etc/modalai/voxl-vision-px4.conf, and change the offboard_mode field to "trajectory":

```
...
	"offboard_mode":	"trajectory",
	"follow_tag_id":	0,
	"en_tag_fixed_frame":	false,
	"fixed_frame_filter_len":	5,
	"en_transform_mavlink_pos_setpoints_from_fixed_frame":	false
}
```

## Running

voxl-mapper is a command line application and can be started on the command line:
```
voxl:/$ voxl-mapper
```
Use the -h option to view command line options
```
voxl:/$ voxl-mapper -h

voxl-mapper usually runs as a systemd background service. However, for debug purposes it can be started from the command line manually with any of the following debug options. When started from the command line, voxl-mapper will automatically stop the background service so you don't have to stop it manually.

-c, --config          load the config file only, for use by the config wizard
-d, --debug           run in debug mode.
-t, --timing          runs in timing mode. Prints how long each step takes.
-h, --help            print this help message
```

## Configuration
Configurable parameters live in config file under /etc/modalai/voxl-mapper.conf
```
voxl:/$ cat /etc/modalai/voxl-mapper.conf
/**
 * This file contains configuration that's specific to voxl-mapper.
 */
{
	"robot_radius":	0.2,
	"point_skip":	7,
	"voxel_size":	0.20000000298023224,
	"voxels_per_side":	16,
	"esdf_save_path":	"/data/voxl_mapper/esdf_map",
	"tsdf_save_path":	"/data/voxl_mapper/tsdf_map",
	"mesh_save_path":	"/data/voxl_mapper/mesh.ply",
	"esdf_max_distance":	4,
	"esdf_min_distance":	0.20000000298023224,
	"esdf_default_distance":	2,
	"esdf_inner_sphere_radius":	0.20000000298023224,
	"esdf_outer_sphere_radius":	0.600000023841858,
	"rrt_min_distance":	0.2,
	"rrt_max_runtime_nanoseconds":	-1,
	"rrt_use_first_solution":	true,
	"rrt_treat_unknown_as_occupied":	true,
	"rrt_send_tree":	false,
	"loco_num_segments":	12,
	"loco_derivative_to_optimize":	3,
	"loco_poly_degree":	12,
	"loco_smoothness_cost_weight":	0.3,
	"loco_collision_cost_weight":	12,
	"loco_waypoint_cost_weight":	0.025,
	"loco_min_collision_sampling_dist":	0.2,
	"loco_add_waypoints":	true,
	"loco_scale_time":	true,
	"loco_split_at_collisions":	true,
	"loco_resample_trajectory":	true,
	"loco_verbose":	false
}
```
The default options are setup for a ModalAi Seeker Drone equipped with a VOXL Time of Flight (TOF) Depth Sensor but can be tuned for your specific use case.

## Viewing
voxl-portal is equipped with viewing capabilities for all outputs of voxl-mapper. To read voxl-portal docs, visit [here](https://docs.modalai.com/voxl-portal/).
| :exclamation:  NOTE   |
|:---------------------------|
| voxl-portal must be installed directly from the  latest mapping_v*.\*.* tag on [gitlab](https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-portal/-/tags) or via the [plot branch](https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-portal/-/tree/plot) for the latest development updates     |

In order to view in voxl-portal, select the Map tab from the options in the top right.
![map_menu.png](/images/map_menu.png)

Within the "Map" page, there are two  main views: 2D or 3D.<br>
![views.png](/images/views.png)<br>

### 2D
The 2D view will show a costmap generated from voxl-mapper at the current hight of your drone. You are able to do most of the planning functions within this view, but it is not recommended since you cannot visually validate the paths.

![2d_view.png](/images/2d_view.png)

### 3D
The 3D view will show the current map in the form of a mesh. This view ahoqa your current position within the map, as well as the ability to toggle on/off the pointcloud you are actively inserting into the map.

![3d_view.png](/images/3d_view.png)

### Options
For either view, once all connections to your drone are established you will see the options on the right hand side of the portal load in.

* Plan Home
  * Plan from your current position to your starting position, i.e. (0,0,-1.5) [1.5m above starting ground takeoff]
* Plan to a Point
  * Click plan: after clicking this button, click a point in the map that you would like to plan to. Then, hit "GO!" and and the planning will begin.
    * Note: this will set your current height for the goal selected
    * Note: the clicked point location accuracy will increase as you zoom into the map
* Load Map
  * load up a map you have previously saved to disk. If you would like to use the default path specified [here](##Configuration), simply click Load Map again with no form input. Otherwise, specify the path you would like to load your map from in the form (no relative paths).
* Save Map
  * save your current map to disk. If you would like to use the default path specified [here](##Configuration), simply click Save Map again with no form input. Otherwise, specify the path you would like to save your map to in the form (no relative paths).
* Clear Map
    * Clear the current map. Does not touch saved maps.
* Reset VIO
    * resets vio to (0,0,0). Please do not reset in flight, only do this when it is safe and necessary.
* Clear Paths
    * clears any previously generated paths or remnants of them from view.
* Position (toggle)
    * enable or disable the position marker. Default is on
* Debug Pointcloud (toggle)
    * enable or disable the pointcloud overlay view. Default is off.

### Planning
With everything running correctly, you will be able to generate paths to follow through your map. This can be done via the Plan Home or the Plan to a Point options. <br>
Once a path is generated, you will immediately see it within the 3D view:

![path_plan.png](/images/path_plan.png)

The red path is the rough RRT* generated waypoints, and the blue path is the smoothed trajectory that you would be following. Before doing anything, please validate that the path you see is indeed obstacle free, and heed the warning that is displayed. If you are ready to follow the path, and your drone is in offboard mode with the correct trajectory mode set pre-launch(in voxl-vision-px4), click the Go/Follow Path button and your drone will begin following the path!

### Other Features
* Image overlay stream
  * click the tv icon in the bottom left, and select a camera from the pop-up menu. The stream can be resized and moved via the bottom right corner of the video, and can be closed via the red x next to the tv icon.
* Dark Mode
  * click the toggle in the right hand corner of voxl-portal in order to switch between light and dark mode
