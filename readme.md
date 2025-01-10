# 1 Dependency

1. qpOASES
2. ocs2_ros2
3. mujoco

# 2 Installation

## 2.1 Install Dependency

### 2.1.1 qpOASES

```
git clone https://github.com/coin-or/qpOASES.git
mkdir build
cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make
sudo make install
```
option `-DCMAKE_POSITION_INDEPENDENT_CODE=ON` is necessary. If you already installed qpOASES without this option, you need to reinstall it


### 2.1.2 ocs2_ros2

[Installation](https://github.com/Zionshang/ocs2_ros2)

### 2.1.3 mujoco

mujoco should be installed by building from source.

1. Clone the mujoco repository: 
```
git clone https://github.com/deepmind/mujoco.git`
```
2. Cd in mujoco path, reate a new build directory and cd into it: `
```
cd mujoco
mkdir build && cd build
```
3. Configure the build, build and install
```
cmake ..
make .
sudo make install
```

## 2.2 Install Project

1. Clone repository: 
```
mkdir -p mujoco_ocs2_ros2_ws/src
cd src
git clone https://github.com/Zionshang/mujoco_ocs2_quadruped_controller.git
```
2. Cd in ros workspace and build
```
cd mujoco_ocs2_ros2_ws
colcon build
```

## 3 Run Example
1. Open a terminal to launch the mujoco emulator
```
ros2 launch mujoco_simulator mujoco.launch.py
```
2. Open a terminal to run controller
```
ros2 launch ocs2_quadruped_controller controller.launch.py
```
3. Enter the control command in the terminal that receives the keyboard command
```
r,f : control body height
1   : stance gait
2   : trot gait
3   : walk trot gait
4   : fly trot gait
w,a,b,d : linear velocity control
j,l : yaw velocity control
```