# liosam_liauto

基于点云地图的重定位与全局实时定位方法（基于 LIO-SAM 实现）

视频预览：
- [无人驾驶课题](https://www.bilibili.com/video/BV11V4y1C7Hp/?spm_id_from=333.999.0.0)
- [视频教程](https://www.bilibili.com/video/BV1LC4y1o7L6/#reply188159845568)

说明：
- `projection::UtmProjector projector(Origin({37.528444, 122.0780557}));` 是建立地图时 GPS 获取的初始坐标。
- 如果你希望使用自己的地图进行全局定位，请确保在 `params.yaml` 中根据实际情况修改相应参数。

# 安装

系统要求：
- Ubuntu（已在 18.04 上测试）
- ROS（已在 melodic 上测试）

## 1. 将本仓库 clone 到你的 ROS 工作空间

在终端中运行：

```bash
cd path/to/your/ros_workspace
git clone https://github.com/1475015695/liosam_liauto.git
```

## 2. 安装依赖

ROS 包：

```bash
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-robot-state-publisher
sudo apt install ros-melodic-lanelet2
```

GTSAM 包：

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## 3. 编译本仓库

```bash
cd ..
catkin_make
```

如果没有报错，编译完成后，通常还需要执行：

```bash
source devel/setup.bash
```

这样系统才能找到我们生成的可执行程序。

# 使用方法

## 1. 准备地图

和 LIO-SAM 一样，为了构建地图，需要激光雷达、IMU 和 GPS（可选）。

我提供了一个示例 ros bag，可用于测试程序：
- [示例 ros bag（百度网盘）](https://pan.baidu.com/s/1Ie9cqvAPQS5xkZcQsWSOOQ?pwd=auto)

在 `config/params.yaml` 中修改地图保存目录（第 45 行附近）：

```yaml
savePCDDirectory: "mapDir"
```

然后运行：

```bash
# 终端 1
roslaunch lio_sam run_ori.launch

# 终端 2
rosbag play path/to/your/rosbag.bag
```

当 rosbag 播放结束后，按 `Ctrl + C` 停止 `roslaunch` 进程，然后检查 `/home/mapDir`，你会看到大量 `.pcd` 文件，目录结构类似如下：

```text
-mapDir
--CornerMap.pcd                        # 全部角点
--GlobalMap.pcd                        # 全部表面点和角点
--SurfMap.pcd                          # 全部表面点
--trajectory.pcd                       # 运动轨迹
--transformations.pcd                  # 转换信息（当前/原始坐标系转全局坐标系）
--cornerVector.pcd                     # 用于重定位的关键角点帧
--surfVector.pcd                       # 用于重定位的关键表面帧
--gpsPoints                            # 用于重定位的 GPS 点
```

恭喜你，完成上述步骤并不容易。强烈建议每次生成最终地图后都复制一份备份，因为每次 `roslaunch` 都可能覆盖原地图，导致最终地图丢失。

## 2. 绘制并加载道路地图

这个 [视频](https://www.bilibili.com/video/BV1Ku411f7xi/?spm_id_from=333.999.0.0&vd_source=ce887b89ae7b1d3754fb211f66301532) 基本涵盖了所需内容。你可以仿照操作生成自己的道路地图，并命名为，例如：

```yaml
laneletFilePath: "/roscode/map_copy/new_lanelet2_maps.osm"
```

确认一切正常后，运行：

```bash
# 终端
roslaunch lio_sam liauto.launch
```

如果你看到白色点云地图和红色车道地图，说明以上步骤都配置正确！

## 3. 重定位（Relocalization）

如果你使用自己的地图，需要在 `params.yaml` 中编辑 `initial_GPS_lat` / `initial_GPS_lon`。

连接传感器后运行：

```bash
# 终端
roslaunch lio_sam liauto.launch
```

`liauto` 会使用当前 GPS 信息进行重定位。为了方便起见，如果你知道自己大致在地图中的位置，也可以在 RViz 中点击 “2D Pose Estimate” 并在地图上标记大致位置。

## 4. 实时定位（Realtime localization）

完成重定位后，`liauto` 会持续跟踪当前位置，并在终端输出坐标。你也可以在 `/currPose` 话题中获取当前位姿消息。

## 5. 资料与参考

1. [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)  # 激光雷达和 IMU 标定。感谢贡献者：Fangcheng Zhu 朱方程，Yunfan Ren 任云帆，Wei Xu 徐威，Yixi Cai 蔡逸熙 等。
2. [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)  # 基础项目。感谢 TixiaoShan。
3. [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)  # 加载和使用 Lanelet 地图。感谢 Poggenhans, Fabian and Pauls, Jan-Hendrik and Janosovits, Johannes and Orf, Stefan and others。
4. [LIO-SAM-DetailedNote](https://github.com/smilefacehh/LIO-SAM-DetailedNote)  # 对 LIO-SAM 的详细说明。感谢 smilefacehh Tao Lu。
5. [My paper](https://pan.baidu.com/s/1N4BHll2LxmXUIf_olci_WA)  # 提取码：1234，我的毕业论文中包含了详细的原理介绍，也感谢我自己 :)

---

如果你愿意，我也可以进一步把英文 README 完整整理成更标准的中文说明，并顺手补一版更适合 GitHub 展示的项目首页结构。