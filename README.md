# Minimum Acceleration Path Follower for ASVs

This path follower controller was designed to improve multibeam sonar data captured by an Autonomous Surface Vessel. The complete analysis can be found in "K. Baxevani, G. E. Otto, A. C. Trembanis, H. G. Tanner, Optimal ASV Path-Following for Improved Marine Survey Data Quality, *OCEANS Conference*, 2023 (in print)".

Installation Guide

To clone and install CCOM Project11 (ROS Noetic). For more info [CCOM Project11 official page](https://github.com/CCOMJHC/project11)
  
  ```
  mkdir -p project11/catkin_ws/src
  cd project11/catkin_ws/src
  git clone https://github.com/CCOMJHC/project11.git

  sudo apt-get install python3-rosdep python3-vcstool
  sudo rosdep init
  rosdep update

  vcs import < project11/config/repos/simulator.repos

  rosdep install --from-paths . --ignore-src -r -y

  cd ..
  catkin_make

  source devel/setup.bash
  roslaunch project11_simulation sim_local.launch
```
Clone and install UD Echoboat plugin for Project11. For more info [UD Echoboat-Project11 official page](https://github.com/grant-otto/echoboat_ud_project11)

```
cd ~/project11/catkin_ws/src
git clone https://github.com/grant-otto/echoboat_ud_project11.git
```
Clone and install the UD Path Follower

```
cd ~/project11/catkin_ws/src
git clone https://github.com/kleiobaxevani/ud_path_follower.git
```

To test the controller on the simulation 
```
  source devel/setup.bash
  roslaunch echoboat_ud_project11 sim_local.launch
```



