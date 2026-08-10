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
        Zikang Yuan,
        Yuan Ren,
        Yixue Wang, 
        Leyi Zhao, 
        Shangzhe Sun, 
        Chi Chen, 
        Lijun Zhu, 
        Xin Yang<sup>†</sup> and
        Tim Cheng
  </strong>
  <p>
    <sup>†</sup>Corresponding Author
  </p>
  <a href='https://www.youtube.com/watch?v=-VXHNZr-xjU'><img alt="Video" src="https://img.shields.io/badge/YouTube-Video-red"/></a>
</div>

## 💡 News
* **[2026.08.10]** The source code of **EPICON** is released !
* **[2026.07.31]** **EPICON** is accepted by RA-L 2026 🚀 !

## 🙏 Acknowledgments

We sincerely thank Prof. [**Boyu Zhou**](https://scholar.google.com.hk/citations?user=-fnyGY4AAAAJ&hl=en) and his team [**STAR**](https://robotics-star.com/) for open-sourcing the excellent works [**EPIC**](https://github.com/Robotics-STAR-Lab/EPIC) and [**FALCON**](https://github.com/HKUST-Aerial-Robotics/FALCON), which provided crucial insights for this work and significantly reduced the engineering implementation effort.

## 📜 Introduction

**EPICON** is a LiDAR-based aerial exploration package for avoiding revisitations on point cloud maps. Simulation and real-world experiments validate that our framework effectively eliminates revisitation redundancy and reduces exploration time against state-of-the-art baselines. (Click the image to view the video)

[![video](doc/illustration.png)](https://www.youtube.com/watch?v=-VXHNZr-xjU)

## 🛠️ Installation

### Test Environment
* Ubuntu 20.04
* ROS Noetic
* C++17

### 🚀 Quick Start

#### Clone our repository and build
```bash
mkdir -p ~/EPICON/src
cd EPICON/src
git clone https://github.com/ZikangYuan/epicon.git
cd..
catkin_make
```
#### Download dataset 
Download simulation maps from *Google cloud address* provided by [EPIC](https://github.com/Robotics-STAR-Lab/EPIC) and [EDEN](https://github.com/NKU-MobFly-Robotics/EDEN), create the folder `MARSIM/map_generator/resource` if it doesn't exist, and move the downloaded maps to this folder.

```bash
mkdir -p MARSIM/map_generator/resource
mv /path/to/downloaded/maps/*.pcd MARSIM/map_generator/resource/
```

#### Run program 
```bash
source /devel/setup.bash && roslaunch epicon_planner garage.launch
```
You can replace `garage` with other maps. Three test scenarios are evaluated in our paper: `cave`, `garage` and `city`.

Our simulation environment is developed based on the GPU version of MARSIM. So if you don't have a GPU, you may need to make some necessary modifications to the simulator.


## 🤓 Credits

We would like to express our gratitude to the following projects, which have provided significant support and inspiration for our work:
- [EPIC](https://github.com/Robotics-STAR-Lab/EPIC): An efficient framework for fast UAV exploration from which is our baseline.
- [MARSIM](https://github.com/hku-mars/MARSIM): A lightweight point-realistic simulator for LiDAR-based UAVs upon which our simulator is built.
- [FALCON](https://github.com/HKUST-Aerial-Robotics/FALCON): An efficient framework for fast UAV exploration, from which we evaluate SOP in large-scale scenarios.
