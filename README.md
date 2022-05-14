# tropicana_ws

### After cloning this project,

1. Build realsense 2 package individually.
```bash 
cd src/realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
catkin_make clean --only-pkg-with-deps realsense2_camera
catkin_make --only-pkg-with-deps realsense2_camera -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install --only-pkg-with-deps realsense2_camera 
```

2. Config realsense camera

```
rosrun rqt_reconfigure rqt_reconfigure
```
- motion_module -> reduce `frames_queue_size` to 2 
- pointcloud -> reduce `frames_queue_size` to 2
- pointcloud -> change `stream_filter` to `ANY`
- rgb_camera -> reduce `frames_queue_size` to 2
- stereo_module -> reduce `frames_queue_size` to 2

** Checkout `rs_point_dynamic_cfg.yaml` **