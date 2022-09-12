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
For voxl-cam:
```
./install_build_deps.sh qrb5165 sdk-0.8
```
For voxl2:
```
./install_build_deps.sh qrb5165 sdk-0.8
```

3. Run build script from the root of project
For voxl-cam:
```
./build.sh qrb5165 sdk-0.8
```
For voxl2:
```
./build.sh qrb5165 sdk-0.8
```

4. Create an ipk
```
./make_package.sh
```

5. Push to voxl
```
./deploy_to_voxl.sh
```


## Install Instructions (if using opkg, not building from source)

1. Connect voxl to internet
```
voxl:/$ voxl-wifi station NET_NAME NET_PSWD
```
2. Configure opkg with dev repo
```
voxl:/$ voxl-configure-opkg dev
voxl:/$ opkg update
```
3. Install Latest Versions of Portal and Mapper
```
voxl:/$ opkg install voxl-portal voxl-mapper
```
* Note: with the recent package name convention change (_ to -), you may have some issues installing these new packages with the old dependencies already present. To bypass this, we can remove the old dependencies since the new packages will auto install the correct ones. If using a custom version of any of these packages, save your changes and merge up to the latest dev, then reinstall.
```
voxl:/$ opkg remove libmodal_json libmodal_pipe libvoxl_cutils libjpeg_turbo
voxl:/$ opkg install voxl-portal voxl-mapper
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

### voxl-qvio-server
If not installed, please run:
```
voxl:/$ opkg update
voxl:/$ opkg install voxl-qvio-server
```
If you are experiencing poor data quality from voxl-qvio-server, it may be worth recalibrating your tracking camera. see https://docs.modalai.com/calibrate-cameras/ for more details

### voxl-vision-px4
If not installed, please run:
```
voxl:/$ opkg update
voxl:/$ opkg install voxl-vision-px4
```

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
After setting the offboard mode, we must now enable and restart the voxl-vision-px4 service. We will do the same for voxl-qvio-server.
```
voxl:/$ systemctl enable voxl-vision-px4 voxl-qvio-server
voxl:/$ systemctl restart voxl-vision-px4 voxl-qvio-server
```

### voxl-camera-server
If not installed, please run:
```
voxl:/$ opkg update
voxl:/$ opkg install voxl-camera-server
```

Configure cameras, if not already working. Select correct config that corresponds to your setup.
```
voxl:/$ voxl-configure-cameras
```

Now, we can enable and restart the camera server to make sure these changes go into effect
```
voxl:/$ systemctl enable voxl-camera-server
voxl:/$ systemctl restart voxl-camera-server
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

It does require that both voxl-vision-px4 and voxl-qvio-server are running and functioning correctly. If there are any errors mentioning fetching from a tf buffer, this means that you are not receiving localization data. Make sure these services are enabled and running when checking via:
```
voxl:/$ voxl-inspect-services
```
Also, make sure the tof pipe is present under /run/mpa/, and you can view the pointcloud in voxl-portal.

Ideal startup process is as follows:
1. Place the drone in takeoff location
2. make sure voxl-vision-px4 is started correctly as explained above, and reset vio
3. start voxl-mapper via ssh terminal
4. take off, and start mapping -> can view live in voxl-portal as explained [here](##Viewing)

## Configuration
Configurable parameters live in config file under /etc/modalai/voxl-mapper.conf
```
voxl:/$ cat /etc/modalai/voxl-mapper.conf
/**
 * This file contains configuration that's specific to voxl-mapper.
 */
{
	"tof_pipe":	"/run/mpa/tof",
	"tof_enable":	true,
	"tof_rate":	10,
	"depth_pipe_0":	"/run/mpa/dfs_point_cloud",
	"depth_pipe_0_enable":	false,
	"extrinsics0_name":	"stereo_l",
	"depth0_rate":	10,
	"depth_pipe_1":	"/run/mpa/stereo_front_pc",
	"depth_pipe_1_enable":	false,
	"extrinsics1_name":	"stereo_front_l",
	"depth1_rate":	10,
	"depth_pipe_2":	"/run/mpa/stereo_rear_pc",
	"depth_pipe_2_enable":	false,
	"extrinsics2_name":	"stereo_rear_l",
	"depth2_rate":	10,
	"depth_pipe_3":	"/run/mpa/dfs_point_cloud",
	"depth_pipe_3_enable":	false,
	"extrinsics3_name":	"stereo_l",
	"depth3_rate":	10,
	"robot_radius":	0.3,
	"voxel_size":	0.20000000298023224,
	"voxels_per_side":	16,
	"esdf_save_path":	"/data/voxl_mapper/esdf_map",
	"tsdf_save_path":	"/data/voxl_mapper/tsdf_map",
	"mesh_save_path":	"/data/voxl_mapper/mesh.ply",
	"esdf_max_distance":	4,
	"esdf_min_distance":	0.05,
	"esdf_default_distance":	2,
	"esdf_inner_sphere_radius":	0.20000000298023224,
	"esdf_outer_sphere_radius":	0.600000023841858,
	"rrt_min_distance":	0.2,
	"rrt_max_runtime_nanoseconds":	1000000000,
	"rrt_use_first_solution":	false,
	"rrt_treat_unknown_as_occupied":	true,
	"rrt_send_tree":	false,
	"loco_num_segments":	5,
	"loco_derivative_to_optimize":	3,
	"loco_poly_degree":	4,
	"loco_smoothness_cost_weight":	2.5,
	"loco_collision_cost_weight":	18.0,
	"loco_waypoint_cost_weight":	0.0,
	"loco_min_collision_sampling_dist":	0.01,
	"loco_add_waypoints":	true,
	"loco_scale_time":	true,
	"loco_split_at_collisions":	true,
	"loco_resample_trajectory":	true,
	"loco_verbose":	false
}
```
* depth_pipe* is the name of any generic depth pointcloud topic that will be integrated into the map
* depth_pipe*_enable turns on or off the specific input source
* extrisics*_name is the name "child" relation to the body frame for this specific sensor. Default sensor    extrinsics are defined in /etc/modalai/extrinsics.conf
	* example -> "stereo_l" for a dfs input
	```
	// from /etc/modalai/extrinsics.conf //
	}, {
			"parent":	"body",
			"child":	"stereo_l",
			"T_child_wrt_parent":	[0.1, -0.04, 0],
			"RPY_parent_to_child":	[0, 90, 90]
		}, {
	...
	```
* depth*_rate is the fixed rate (in Hz) that a sensor will be sampled at. This can be useful when combining multiple inputs or high incoming data rates

The default options are setup for a ModalAi Seeker Drone equipped with a VOXL Time of Flight (TOF) Depth Sensor but can be tuned for your specific use case.

The parameters listed above are slightly different than stock, and are tuned for the ModalAi Seeker Drone in a specific setting: short, quick paths around a room. If looking to plan longer distances, it will be helpful to increase the "rrt_max_runtime_nanoseconds", "loco_num_segments", and "loco_poly_degree" params in small increments.

## Viewing
voxl-portal is equipped with viewing capabilities for all outputs of voxl-mapper. To read voxl-portal docs, visit [here](https://docs.modalai.com/voxl-portal/).

In order to view in voxl-portal, select the Map tab from the options in the top right.
![map_menu.png](/images/map_menu.png)

Within the "Map" page, there are two  main views: 2D or 3D.<br>
![views.png](/images/views.png)<br>

### 2D
The 2D view will show a costmap generated from voxl-mapper at the current hight of your drone.

![2d_view.png](/images/2d_view.png)

### 3D
The 3D view will show the current map in the form of a mesh. This view shows your current position within the map, as well as the ability to toggle on/off the pointcloud you are actively inserting into the map. This is also the only view you can perform any path planning within.

![3d_view.png](/images/3d_view.png)

### Options
For either view, once all connections to your drone are established you will see the options on the right hand side of the portal load in.

* Plan Home
  * Plan from your current position to your starting position, i.e. (0,0,-1.5) [1.5m above starting ground takeoff]
* Plan to a Point
  * Click plan: after clicking this button, click a point in the map that you would like to plan to. Then, hit "GO!" and and the planning will begin.
    * Note: this will set your current height for the goal selected
    * Note: the clicked point location accuracy will increase as you zoom into the map
* Map IO
	* if looking to save in a different location that the /data/voxl_mapper/ directory, make sure you the directory exists before starting the application. the program will create the files
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
