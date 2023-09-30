# liosam_liauto
Achieving relocalization and global real-time localization  based on LIO-SAM in point cloud maps\
sample video preview:[无人驾驶课题](https://www.bilibili.com/video/BV11V4y1C7Hp/?spm_id_from=333.999.0.0) 视频中“projection::UtmProjector projector(Origin({37.528444, 122.0780557}));“是当时建立地图时GPS获取的初始坐标。
# Install
System Require:\
**Ubuntu(tested with 18.04)**\
**ROS(tesed with melodic)**
## 1. git clone this repository to your **ros workspace**,Run in terminal:
```
cd path\to\your\ros_worksapce
git clone https://github.com/1475015695/liosam_liauto.git
```
## 2. Install dependency 


ROS packages:
```
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-robot-state-publisher
```
GTSAM packages：
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## 3. compile this repository
```
cd ..
catkin_make
```
**Congratulations**！ if no error messages comeout :), After compile, you may need execute
```
source devel/setup.bash
```
to let system found our executable porgram.
# Usage
## 1. Prepare map
Same as LIO-SAM, to build map we need Lidar、IMU、GPS( optinal ), dont be warry my bro, I provided an [example ros bag](https://pan.baidu.com/s/1Ie9cqvAPQS5xkZcQsWSOOQ?pwd=auto) to test our program. Moreover, I also provided built map file( incase you meet some problem or test in stages :)\
Edit save map dir in config/params.yaml at line 45
```
 savePCDDirectory: "mapDir" 
```
Then run 
```
#in terminal
roslaunch lio_sam run_ori.launch
#in another terminal
rosbag play path/to/your/rosbag.bag
```
After bag play end. ctrl + c to stop roslaunch process, check /home/mapDir, you will see there are many pcd files, and folder be like:
```
-mapDir
--CornerMap.pcd                        #whole Corner points
--GlobalMap.pcd                        #whole Surface and Corner points
--SurfMap.pcd                          #whole surface points
--trajectory.pcd                       #move trajectoies
--transformations.pcd                  #tranform information(current/ori axis to global axis)
--keyMap                               #keyMaps, help to achieve relocation
---subCorner0.pcd                      #keyCornelFrame
---......
---subSurface0.pcd                     #keySurfaceFrame
---......
```
**Congratulations**！bro, its hard for most people to get above steps done :)   \
I highly recommended make a copy of final map( every time you runlaunch, map will be overwrite, may lost our final map)                                                  
## 2. Drow and load road map
This [video](https://www.bilibili.com/video/BV1Ku411f7xi/?spm_id_from=333.999.0.0&vd_source=ce887b89ae7b1d3754fb211f66301532) is all we need, we can imitate to genrate our road map, here I name it "new_lanelet2_maps.osm" and put it in "/roscode/map_copy/", edit laneletFilePath in config/params.yaml at line 105. \
```
laneletFilePath: "/roscode/map_copy/new_lanelet2_maps.osm"  
```
OK! let check wether everything is fine:
```
#in terminal
roslaunch lio_sam liauto.launch
```
If you see white points cloud map and red lane map, **great**! you have done above steps correctly!\
Come bro, we almost done :o\
## 3. Relocatlization
After connect your sensor to ros, run:
```
#in terminal
roslaunch lio_sam liauto.launch
```
liauto will do relocatlization use current GPS message, but for convince popurse, if you known where you are, you can click 2d pose estimated in Rcviz and mark your probably posetion in map.
## 4. Realtime locatlization
After relocalization, liauto will keep track our postion in map, and output coordinate in terminal, you can use coordinate in "/currPose" message. Have Fun!
## 5. Material
1. [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init) # calib Lidar and IMU. Thanks Contributors: Fangcheng Zhu 朱方程， Yunfan Ren 任云帆， Wei Xu 徐威， Yixi Cai 蔡逸熙 :)
2. [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) # base project. Thanks TixiaoShan :)
3. [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) # load and use Lanelet map. Thanks Poggenhans, Fabian and Pauls, Jan-Hendrik and Janosovits, Johannes and Orf, Stefan and Naumann, Maximilian and Kuhnt, Florian and Mayr, Matthias :)
4. [LIO-SAM-DetailedNote](https://github.com/smilefacehh/LIO-SAM-DetailedNote) # DetailedNote for lio-sam. Thanks smilefacehh Tao Lu :)
5. [My paper]() #我的毕业论文，里面有详细的原理介绍 :) 也谢谢我自己 :)
