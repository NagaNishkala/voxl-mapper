# voxl-mapper/planner

3d Mapping and Path Planning

**NOTE**
To install all dependenices on VOXL, run this command:
```
yocto:/$ sudo opkg install libmodal_json openmp libvoxl_cutils ceres-solver nlopt voxl-mpa-tools libmodal_pipe voxblox
```

## Build Instructions

1. Start voxl-cross docker image
```
you@home-pc:~$ voxl-docker -i voxl-cross
```
2. Install build dependencies into docker environment
(will need these on target as well)
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

## Running
Run on target
```
yocto:/$ voxl-mapper
```
Configurable parameters live in config file under /etc/modalai/voxl-mapper.conf
Run with -h flag to see valid runtime flags
