<div align = "center">
  <h1>
    Aerial Exploration on Point Cloud Maps via Coverage Path Guidance
  </h1>
</div>
<div align = "center">
  <h2>
    A LiDAR-based Aerial Exploration Package for Avoiding Revisitations
  </h2>
</div>
<div align="center">
  <strong>
        Anonymous Authors
  </strong>
</div>

## 🛠️ Installation

### Test Environment
* Ubuntu 20.04
* ROS Noetic
* C++17

### 🚀 Quick Start

#### Clone our repository and build
```bash
mkdir -p ~/LAPIC/src
cd LAPIC/src
git clone https://github.com/ZikangYuan/lapic.git
cd..
catkin make
```
#### Download dataset 
Download simulation maps from *Google cloud address* provided by [EPIC](https://github.com/Robotics-STAR-Lab/EPIC) and [EDEN](https://github.com/NKU-MobFly-Robotics/EDEN), create the folder `MARSIM/map_generator/resource` if it doesn't exist, and move the downloaded maps to this folder.

```bash
mkdir -p MARSIM/map_generator/resource
mv /path/to/downloaded/maps/*.pcd MARSIM/map_generator/resource/
```

#### Run program 
```bash
source /devel/setup.bash && roslaunch epic_planner garage.launch
```
You can replace `garage` with other maps. Three test scenarios are evaluated in our paper: `cave`, `garage` and `city`.

Our simulation environment is developed based on the GPU version of MARSIM. So if you don't have a GPU, you may need to make some necessary modifications to the simulator.


## 🤓 Acknowledgments

We would like to express our gratitude to the following projects, which have provided significant support and inspiration for our work:
- [EPIC](https://github.com/Robotics-STAR-Lab/EPIC): An efficient framework for fast UAV exploration from which is our baseline.
- [MARSIM](https://github.com/hku-mars/MARSIM): A lightweight point-realistic simulator for LiDAR-based UAVs upon which our simulator is built.
- [FALCON](https://github.com/HKUST-Aerial-Robotics/FALCON): An efficient framework for fast UAV exploration, from which we evaluate SOP in large-scale scenarios.
