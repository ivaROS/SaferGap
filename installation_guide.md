### Collect and build all packages required

__Note: you must have your ssh keys set up for this to work properly!__ Follow [this tutorial](https://help.github.com/articles/connecting-to-github-with-ssh/) to setup your ssh keys with github. 

Following is a general series of steps to accomplish the most common tasks with `wstool`.

1. Install the `wstool`:
```
sudo apt-get install python3-wstool
```

Note: All following commands should be run from the root of your catkin workspace!

2. Navigate to catkin workspace and initialize the `wstool` workspace:
```
cd ~/catkin_ws/src && git clone https://github.com/ivaROS/SaferGap.git
```

3. Initialize the `wstool` workspace:
```
cd ~/catkin_ws && wstool init src
```

4. Add packages from `safer_gap_install.rosinstall` to your catkin workspace:
```
wstool merge -t src src/SaferGap/safer_gap_install.rosinstall
```

5. Download/update all packages managed by wstool:
```
wstool update -t src -j20
rosdep install --from-paths src -i -y
```

Ignore the complaint for any `Cannot locate rosdep definition for [common_rosdeps]`

6. Go to `CMakeLists.txt`. Adjust the line 68 and 93 to be compatible with your paths to google or-tools `lib` and `include` directories:

7. Build all packages:
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

