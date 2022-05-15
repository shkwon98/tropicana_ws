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