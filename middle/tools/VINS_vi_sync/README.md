# BW_multi_sensor_fusion
# VINS_vi_sync
# 简介 Introduction
本系统是基于VINS-Mono改进的视觉-惯性时空一体优化系统，用于解决多传感融合中的硬件时间同步问题。相比于传统的时间标定与空间标定分离，此方案有三大优势：
1、时间、空间参数在同一目标函数中优化，避免误差在分布标定中累积；
2、视觉重投影误差与IMU预积分相互约束校正，标定结果不依赖运动激励的充分性；
3、实时调整参数，适应环境变化，避免无法处理运行时的时间漂移。
与此同时，本系统还增加了新的图像预处理模块，可以处理更多类型的图像，如RGB等，不再局限于VINS-Mono本身只能处理灰度图像的特点。

---

# 功能 Functions
1、在线估计相机与IMU之间的硬件时间延迟，实时补偿时间偏差无需预先标定时间参数，系统在线自适应。
2、无需提前知道外参与时间延迟，系统可以进行在线估计并融入到每一次优化过程中，只需要提前输入相机内参即可，而内参通常会由厂商给出。
3、发布时间戳对齐后的传感器数据流，包含/aligned_grayscale_image, /inter_frame, /current_td三个新话题。其中：
/aligned_grayscale_image是时间戳对齐后的图像灰度信息，不同于VINS-Mono原本发布的图像特征信息，完整的灰度信息能提供更多图像原本的信息，便于更多可能的后续操作；
/inter_frame提供对齐后两帧之间的所有IMU数据；
/current_td是最新的时间延迟。
4、兼容EuRoC标准数据格式，支持快速部署。如需其他标准数据格式，可在config中进行自定义，或移植VINS-Mono官方的config相关代码。

---

# 环境 Environment
- Ubuntu 18.04
- ROS1 Melodic
- eigen3.3.4
- ceres1.14
- OpenCV3.4.1
- least C++11

---

# 使用 Usage
- 打开虚拟机中配置的Ubuntu 18.04后，首先对环境进行配置安装：（注意，如果遇到虚拟机内下载速度慢等问题时，可以考虑在虚拟机左上角的选项“虚拟机（M）”中，先安装好VMware Tools，这样就在自己的Windows系统下下载好需要的文件，再从Windows系统中拖进虚拟机中已经打开的指定文件夹了）
1. 首先对ROS1 Melodic进行安装。避免后面会出现无法定位软件包的错误，这里ROS提供了国内中科大的安装源，添加国内安装源命令：
先使用Ctrl+Alt+T打开终端，输入
`sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'`
接下来使用如下指令设置密钥：
`sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`
及时更新可用软件包列表：
`sudo apt update`
此时若出现如下报错：
W: GPG error: http://mirrors.ustc.edu.cn/ros/ubuntu bionic InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY F42ED6FBAB17C654
E: The repository 'http://mirrors.ustc.edu.cn/ros/ubuntu bionic InRelease' is not signed.
N: Updating from such a repository can't be done securely, and is therefore disabled by default.
N: See apt-secure(8) manpage for repository creation and user configuration details.
E: Problem executing scripts APT::Update::Post-Invoke-Success 'if /usr/bin/test -w /var/cache/app-info -a -e /usr/bin/appstreamcli; then appstreamcli refresh-cache > /dev/null; fi'
E: Sub-process returned an error code
可使用如下指令解决：
`sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654`
注意再对可用软件包列表进行更新：
`sudo apt update`
此时就可以开始ROS1 Melodic的正式安装：
`sudo apt-get install ros-melodic-desktop-full`
避免后面出现sudo rosdep:找不到命令提示，我们再执行：
`sudo apt install python-rosdep`
开始使用ROS之前，先初始化rosdep，rosdep能够轻松地安装要编译的源代码的系统依赖关系，rosdep是ROS核心组件运行的基础:
`sudo rosdep init`
对rosdep进行更新：
`rosdep update`
如果出现如下报错：
ERROR: error loading sources list:
	('The read operation timed out',)
一般是网络问题，我们反复执行更新指令即可
添加ROS环境变量：
`echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
`
为了构建和管理开发者自己的ROS工作空间，还需安装rosinstall:
`sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
`
为了确保我们配置好了ROS中VINS需要的相关文件，执行如下指令：
`sudo apt-get install ros-melodic-cv-bridge ros-melodic-tf ros-melodic-message-filters ros-melodic-image-transport`

2. 安装其他库：
eigen3.3.4版本：
`sudo apt-get install libeigen3-dev`
ceres1.14版本：
```bash
#1、 安装依赖库：
sudo apt-get install liblapack-dev libsuitesparse-dev libgflags-dev 
sudo apt-get install libgoogle-glog-dev libgtest-dev
sudo apt-get install libcxsparse3
 
#2、下载ceres-solver-1.14.0
wget ceres-solver.org/ceres-solver-1.14.0.tar.gz
 
#3、解压
tar -zxvf ceres-solver-1.14.0.tar.gz
 
#4、进入安装包
cd ceres-solver-1.14.0
 
#5、建立编译安装包
sudo mkdir build
 
#6、进入build
cd build
 
#7、预编译
sudo cmake ..
 
#8、编译
sudo make
 
#9、安装
sudo make install
```
OpenCV3.4.1以及组件OpenCV_contrib3.4.1版本：
在浏览器中进入：
`https://opencv.org/releases/page/6/`
在OpenCV-3.4.1的github选项中下载zip 文件
再进入：
`https://github.com/opencv/opencv_contrib/tree/3.4.1`
将对应组件完整code下来，将两个文件下载好使用unzip解压后，将opencv_contrib3.4.1放在opencv3.4.1文件夹里，为了方便后续操作，可将这两个文件夹分别命名为opencv、opencv_contrib,再打开终端，依次执行：

```bash
sudo apt-get install build-essential 
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
sudo apt install python3-numpy
cd  opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ..
sudo make

```
此时若遇到报错：
/home/test/opencv/opencv_contrib/modules/xfeatures2d/src/boostdesc.cpp:653:20: fatal error: boostdesc_bgm.i: No such file or directory
           #include "boostdesc_bgm.i"
                    ^~~~~~~~~~~~~~~~~
compilation terminated.
modules/xfeatures2d/CMakeFiles/opencv_xfeatures2d.dir/build.make:91: recipe for target 'modules/xfeatures2d/CMakeFiles/opencv_xfeatures2d.dir/src/boostdesc.cpp.o' failed
make[2]: *** [modules/xfeatures2d/CMakeFiles/opencv_xfeatures2d.dir/src/boostdesc.cpp.o] Error 1
CMakeFiles/Makefile2:13338: recipe for target 'modules/xfeatures2d/CMakeFiles/opencv_xfeatures2d.dir/all' failed
make[1]: *** [modules/xfeatures2d/CMakeFiles/opencv_xfeatures2d.dir/all] Error 2
Makefile:162: recipe for target 'all' failed
make: *** [all] Error 2

此时将缺失的文件下载解压后将每一个小文件均拷贝至opencv/opencv_contrib/modules/xfeatures2d/src目录下再进行`sudo make`(你可以在config文件夹中找到它)
此时若有报错：
/home/test/opencv/opencv_contrib/modules/sfm/src/simple_pipeline.cpp:41:10: fatal error: opencv2/xfeatures2d.hpp: No such file or directory
 #include <opencv2/xfeatures2d.hpp>
          ^~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
modules/sfm/CMakeFiles/opencv_sfm.dir/build.make:230: recipe for target 'modules/sfm/CMakeFiles/opencv_sfm.dir/src/simple_pipeline.cpp.o' failed
make[2]: *** [modules/sfm/CMakeFiles/opencv_sfm.dir/src/simple_pipeline.cpp.o] Error 1
CMakeFiles/Makefile2:15240: recipe for target 'modules/sfm/CMakeFiles/opencv_sfm.dir/all' failed
make[1]: *** [modules/sfm/CMakeFiles/opencv_sfm.dir/all] Error 2
Makefile:162: recipe for target 'all' failed
make: *** [all] Error 2
实际是CMake配置时未正确关联opencv_contrib的模块路径，或xfeatures2d模块未被成功编译。我们需要进行如下操作：

```bash
cd ~/opencv/build  # 替换为你的build目录路径
rm -rf *  # 清空旧的编译文件（确保在build目录下执行，避免误删其他文件）
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib/modules \  # 关键：指向opencv_contrib的modules目录
      -D BUILD_EXAMPLES=ON ..
sudo make
sudo make install
```

3. 创建自己的工作空间catkin_ws，在catkin_ws下创建src文件夹，并将本项目的功能包放入src中进行编译：

```bash
cd catkin_ws
catkin_make
```

4. 进行初始内参，外参估计模式与时间延迟估计模式的配置，我们打开catkin_ws/src/VINS_vi_sync/config/euroc，会看到两个配置文件，无论选择哪个，我们都需要在文件内写入具体的内参，文件的选择方式如下：

如果有已知的准确外参或外参的大致估计值，我们就打开euroc_config.yaml，找到参数estimate_extrinsic，如果有已知的准确外参，则将参数设置为0，同时在下方写入准确外参，如果已知大致外参，则将参数设置为1，同时在下方写入大致外参。对于时间延迟估计模式的选择，我们找到参数estimate_td，需要进行在线估计则选择将参数设置为1，否则设置为0，并填写初始td；

如果外参完全未知，我们则打开euroc_config_no_extrinsic.yaml，只需对时间延迟估计模式进行选择即可。

5. 完成初始配置后，准备EuRoC数据集，如MH_01_easy.bag，或使用自己设备录制的EuRoC数据集标准格式的bag文件。打开一个终端启动ROS：`roscore`
再打开一个新终端，进入工作空间并加载环境变量后启动launch文件：

```bash
cd catkin_ws
source devel/setup.bash
roslaunch vins_estimator euroc.launch
```
再打开一个新终端运行准备好的bag文件：
`rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag #play后为你的bag文件的路径`
此时可以通过如下指令验证相关话题是否正确输出：(某一话题确认正常输出后可使用Ctrl+C终止，再对其他话题进行检验)

```bash
cd catkin_ws
source devel/setup.bash
rostopic echo /aligned_grayscale_image
rostopic echo /interframe_imu
rostopic echo /current_td
```

---

# 目录结构 Project Structure
VINS_vi_sync
├── camera_model
│   ├── cmake
│   ├── CMakeLists.txt
│   ├── include
│   ├── instruction
│   ├── package.xml
│   └── src
├── config
│   └── euroc
├── feature_tracker
│   ├── cmake
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
├── LICENCE
├── README.md
├── support_files
│   ├── brief_k10L6.bin
│   └── brief_pattern.yml
└── vins_estimator
    ├── cmake
    ├── CMakeLists.txt
    ├── launch
    ├── package.xml
    └── src

---

# 文件说明 File Description

## 顶层构建文件

### **CMakeLists.txt**
- **功能**：ROS1 catkin构建配置，声明依赖、编译消息、生成可执行文件。

### **package.xml**
- **功能**记录ROS包的元信息，定义包名、版本、维护者和依赖。

---

## feature_tracker

### **image_preprocess/image_format.h**
- **功能**：该头文件 image_format.h 提供了一组工具函数，用于判断图像的编码格式类型，主要服务于图像预处理环节。具体功能包括：
判断图像编码是否属于 RGB 家族（含带 alpha 通道的格式）；
判断图像编码是否属于灰度图格式；
判断图像编码是否为 16 位深度格式。
- **实现步骤**：
isRGBFamily：通过直接比较输入的编码字符串与预定义的 RGB 家族编码，判断是否属于 RGB 相关格式。
isGrayFamily：通过比较编码字符串与灰度图编码，判断是否为灰度图。
is16Bit：通过检查编码字符串中是否包含子串 "16"，判断图像是否为 16 位深度。

### **image_preprocess/image_converter.h**
- **功能**：该头文件 image_converter.h 主要声明了一个用于图像格式转换的函数convertToGray。其核心功能是接收原始图像（cv::Mat 类型）和对应的 ROS 图像编码格式字符串，将输入图像转换为 8 位单通道灰度图（OpenCV 中的 CV_8UC1 类型）。用于视觉特征跟踪流程中的图像预处理阶段，为后续特征提取提供统一格式的灰度图输入。

### **image_preprocess/image_converter.cpp**
- **功能**：该文件中的convertToGray函数是一个图像预处理工具，主要功能是将不同编码格式的输入图像（可能是 RGB、BGR、灰度等格式，也可能是 8 位或 16 位深度）统一转换为 8 位单通道的灰度图像，以便后续特征跟踪等处理流程使用。
- **实现步骤**：
1、16 位图像转 8 位处理：
首先判断输入图像的编码是否为 16 位格式。如果是，通过线性缩放（将 0-65535 范围缩放到 0-255）转换为 8 位图像；如果是 8 位图像，则直接复制使用。
2、根据编码转换为灰度图像：
若图像属于 RGB 家族，根据具体编码调用对应的 OpenCV 颜色空间转换函数生成灰度图。
若图像属于灰度家族，直接使用单通道图像；若通道数异常（多通道），则强制转换为单通道。
若编码未知，根据图像通道数推测格式（3 通道假设为 BGR，4 通道假设为 BGRA，其他通道数取第一个通道作为灰度）。
3、最终格式校验：
确保输出图像为 8 位单通道（CV_8UC1），若不是则强制转换。

### **feature_tracker.h、feature_tracker.cpp**
- **功能**：两个文件共同实现了一个FeatureTracker类，主要用于在连续图像帧中检测、跟踪特征点，并进行特征点的筛选、去畸变和速度计算等处理，核心目标是稳定地跟踪图像中的关键特征，为后续的位姿计算提供可靠输入。主要功能为：
1、特征点跟踪：使用 LK 光流法在连续帧之间跟踪已检测到的特征点。
2、特征点筛选：通过基础矩阵的 RANSAC 算法剔除误匹配的特征点，保留鲁棒的匹配结果。
3、新特征点检测：当跟踪的特征点数量不足时，在图像中未被现有特征点覆盖的区域检测新的特征点（使用goodFeaturesToTrack）。
4、特征点去畸变：根据相机内参对特征点进行去畸变处理，消除镜头畸变影响。
5、特征点管理：为特征点分配唯一 ID，记录其被跟踪的时长，优先保留跟踪稳定（时间长）的特征点。
- **实现步骤**：
1、初始化与参数设置：
初始化成员变量（如特征点容器、ID 计数器、相机内参等）。
读取相机内参文件，用于后续去畸变处理。
2、图像预处理：
将输入图像转换为灰度图，可选使用 CLAHE（对比度受限自适应直方图均衡化）增强图像对比度，提升特征检测稳定性。
3、特征点跟踪：
对当前帧与前一帧使用 LK 光流法跟踪特征点，得到特征点在当前帧的位置。
过滤出边界外的特征点（通过inBorder函数），并根据跟踪状态精简特征点列表（通过reduceVector函数）。
4、特征点筛选：
使用基础矩阵的 RANSAC 算法（rejectWithF函数）剔除误匹配的特征点，提高跟踪精度。
5、新特征点补充：
生成掩码（setMask函数），确保新检测的特征点与已有特征点保持最小距离（避免密集分布）。
当特征点数量不足时，在掩码允许的区域检测新特征点，并添加到跟踪列表中（addPoints函数）。
6、去畸变与速度计算：
对当前帧的特征点进行去畸变处理（undistortedPoints函数），转换为归一化平面坐标。
根据前后帧特征点的位置变化计算其运动速度。
7、状态更新：
更新帧缓存（前一帧、当前帧、下一帧），更新特征点 ID 和跟踪计数，为下一帧处理做准备。

### **feature_tracker_node.cpp**
- **功能**：该文件是一个 ROS 节点，主要功能是接收相机图像数据，进行特征点检测与跟踪，并将跟踪结果（特征点信息、可视化图像、时间对齐后的图像）发布出去，常用于视觉 SLAM系统中提供前端特征跟踪支持。具体包括：
1、订阅相机图像消息和时间延迟（td）消息。
2、对图像进行预处理（如格式转换、均衡化）和特征点跟踪（维护特征 ID、跟踪计数等）。
3、控制发布频率，筛选有效特征点（跟踪次数大于 1 的点）并发布其坐标、ID、速度等信息。
4、发布特征跟踪的可视化图像，便于调试。
5、根据时间延迟（td）对图像时间戳进行补偿，发布对齐后的灰度图像，也是本系统重点改进之一。
- **实现步骤**：
1、初始化与参数配置：
初始化 ROS 节点，读取相机内参、是否使用鱼眼镜头等参数。
创建订阅者（订阅图像和时间延迟消息）和发布者（发布特征点、可视化图像、重启信号、时间对齐图像）。
2、图像回调处理：
首次接收图像时初始化时间戳，后续检查图像时间连续性（若中断则重置跟踪器）。
控制发布频率（根据预设频率FREQ决定是否发布当前帧）。
转换图像格式（如将8UC1转为mono8），并分相机处理图像（如鱼眼镜头的畸变校正、图像均衡化）。
3、特征跟踪与 ID 更新：
对每个相机的图像进行特征点跟踪，更新特征点 ID 和跟踪计数。
筛选跟踪次数大于 1 的有效特征点，整理其坐标、ID、速度等信息。
4、数据发布：
发布特征点的点云消息（包含坐标、ID、像素位置、速度）。
发布特征跟踪的可视化图像（标记特征点及跟踪轨迹）。
根据时间延迟td调整图像时间戳，发布时间对齐后的灰度图像。
5、时间延迟处理：
订阅时间延迟消息，实时更新当前时间延迟current_td，用于图像时间戳补偿。

### **parameters.h、parameters.cpp**
- **功能**:这两个文件是一个参数管理模块，主要功能是：
1、定义系统运行所需的各类配置参数（如相机参数、ROS 话题名、特征跟踪参数等）；
2、实现参数读取逻辑，从 ROS 参数服务器和外部配置文件（如 YAML）中加载参数，并为未配置的参数设置默认值；
3、通过全局变量的方式，让系统其他模块可以方便地访问这些参数。
- **实现步骤**：
1、参数声明（parameters.h）
声明外部（extern）全局变量，包括相机内参（ROW、COL、FOCAL_LENGTH）、ROS 话题名（IMAGE_TOPIC、IMU_TOPIC）、特征跟踪参数（MAX_CNT、MIN_DIST等）；
声明参数读取函数 readParameters，供外部调用以加载参数；
包含必要的头文件（ROS 和 OpenCV），并使用 #pragma once 防止头文件重复包含。
2、参数定义与读取实现（parameters.cpp）
定义 parameters.h 中声明的全局变量，分配内存；
实现模板函数 readParam，用于从 ROS 节点句柄（ros::NodeHandle）中读取参数，并处理读取失败的情况（报错并关闭节点）；
实现 readParameters 函数：
先从 ROS 参数服务器读取配置文件路径；
使用 OpenCV 的 cv::FileStorage 打开配置文件，读取图像话题、IMU 话题、特征数量等参数；
为部分参数设置默认值（如 WINDOW_SIZE=20、FOCAL_LENGTH=460）；
处理特殊情况（如鱼眼相机的掩码路径拼接、频率参数默认值）。

### **tic_toc.h**
- **功能**：该文件定义了一个名为TicToc的类，用于便捷地测量程序中代码段的运行时间（精确到毫秒）。其核心功能类似秒表：通过tic()方法记录开始时间，通过toc()方法计算并返回从tic()到toc()之间的时间间隔（单位：毫秒）。

---

## camera_model

- **功能**：camera相关文件主要用于定义和实现相机模型，处理相机的内参管理、畸变矫正、3D 点与图像点的投影 / 反投影变换、相机标定等核心功能，是视觉前端处理图像信息的基础。其核心作用是：
1、统一管理不同类型相机的内参（通过基类接口）。
2、实现 3D 点与图像点的精准转换（含畸变矫正），为视觉前端的特征匹配、三角化提供几何基础。
3、支持相机标定，从标定板数据中自动估计内参和外参，是系统初始化的关键步骤。

---

## config/euroc

### **euroc_config.yaml、euroc_config_no_extrinsic.yaml**
- **功能**：两个 YAML 文件是视觉惯性里程计系统的配置文件，用于定义系统运行的关键参数，其中euroc_config_no_extrinsic.yaml是完全不知道外参时对应的配置文件。它们主要作用是：
1、配置传感器（IMU 和相机）的话题名称、输出路径；
2、提供相机内参（校准参数）用于图像畸变矫正和空间点投影；
3、定义 IMU 与相机之间的外参（相对位姿）处理方式（已知、初始猜测或需要校准）；
4、设定特征跟踪、后端优化、IMU 噪声模型、回环检测等模块的运行参数，确保系统能稳定融合视觉与惯性数据，实现精准的位姿估计。
- **实现步骤**：  VIO 系统加载配置文件后的核心运行步骤如下：
1、参数读取：系统启动时解析 YAML 文件，加载所有参数到内存，作为各模块的初始化配置。
2、传感器数据订阅：根据 imu_topic 和 image_topic 订阅 IMU 和相机图像数据。
3、相机预处理：使用相机内参（model_type、distortion_parameters、projection_parameters）对图像进行畸变矫正，将像素坐标转换为相机坐标系下的三维点。
4、外参处理：
若 estimate_extrinsic=0：直接使用配置的 extrinsicRotation 和 extrinsicTranslation 作为 IMU 与相机的精确外参；
若 estimate_extrinsic=1：在初始猜测的位姿基础上进行优化；
若 estimate_extrinsic=2：不依赖初始外参，系统通过初始旋转运动自动校准外参。
特征跟踪：根据 max_cnt（最大特征数）、min_dist（特征最小间距）等参数，在连续图像中跟踪特征点，计算视觉约束。
状态估计与优化：结合 IMU 数据（使用 acc_n、gyr_n 等噪声参数建模）和视觉特征，通过后端优化（受 max_solver_time、max_num_iterations 限制）估计系统位姿。
回环检测与时间同步：若 loop_closure=1 则启用回环检测修正漂移；若 estimate_td=1 则在线估计相机与 IMU 的时间偏移。
结果输出：将位姿估计结果、跟踪图像等保存到 output_path 或 pose_graph_save_path。

---

## supportt_files

### **brief_pattern.yml**
- **功能**：提供BRIEF描述子，用于特征点的匹配和计算。

---

## vins_estimator

### **factor**
- **功能**：vins_estimator/src/factor目录下的文件主要用于定义非线性优化问题中的代价函数（残差因子），是系统状态估计的核心组件。这些因子通过描述传感器测量（视觉、IMU）与状态变量（位姿、速度、外参等）之间的约束关系，为优化器提供残差计算和雅可比矩阵，最终实现状态的最优估计。

### **initial**
- **功能**：vins_estimator/src/initial目录下的文件核心功能是实现系统的初始化过程。由于 VINS-Mono 需要融合视觉和 IMU数据，而系统启动时没有先验的状态信息（如相机位姿、三维地图点、IMU 与相机外参、重力向量、尺度因子等），该文件夹下的代码通过处理初始帧的视觉和 IMU 数据，完成这些关键参数的初始化，为后续的非线性优化提供可靠的初始值。

### **utility**
- **功能**：vins_estimator/src/utility目录下的文件夹包含了一系列通用工具类和函数，主要为 VINS的核心算法提供辅助支持，包括坐标转换、数据可视化、时间统计、文件操作等基础功能。

### **feature_manager.h、feature_manager.cpp**
- **功能**：它们是视觉 SLAM系统中用于特征点管理的核心模块。其主要功能是跟踪图像中的特征点在多帧间的运动，计算特征点的视差（用于判断关键帧）、估计特征点的深度（三维位置），并维护特征点在滑动窗口中的生命周期（添加、移除、更新等）。它们的核心类与功能为：
1、FeaturePerFrame：存储单帧图像中一个特征点的信息，包括：
归一化平面坐标（point）、像素坐标（uv）、运动速度（velocity）；
时间偏移（cur_td）、是否被使用（is_used）等辅助信息。
2、FeaturePerId：管理同一个特征点在多帧图像中的观测数据，包括：
特征点唯一 ID（feature_id）、首次出现的帧（start_frame）；
跨帧观测数据列表（feature_per_frame，存储每帧的FeaturePerFrame）；
深度估计结果（estimated_depth）、求解状态（solve_flag：未求解 / 成功 / 失败）等。
3、FeatureManager：特征管理的核心类，负责整体逻辑，包括：
添加新特征并检查视差（判断是否生成关键帧）；
计算特征点的三维深度（通过三角化）；
维护滑动窗口中的特征（移除过期、异常特征，更新边缘化后的特征状态）。
- **实现步骤**：
1、初始化：
FeatureManager构造函数接收旋转矩阵数组Rs（外参），并初始化相机外参旋转矩阵ric。
2、添加特征与视差检查：
通过addFeatureCheckParallax函数添加新帧的特征点，跟踪同一特征在多帧中的出现。当跟踪帧数足够时，计算特征点在相邻帧的视差（compensatedParallax2），若平均视差超过阈值，则判定为关键帧。
3、深度估计：
对未估计深度的特征点，通过triangulate函数利用多视图几何（SVD 分解）计算三维深度。
从优化结果中更新特征深度（setDepth），或清除深度数据（clearDepth）。
4、特征维护：
移除深度估计失败的特征（removeFailures）、异常值（removeOutlier）。
滑动窗口更新时，移除窗口外的过期特征（removeBack、removeFront），或在边缘化后调整特征的深度和起始帧（removeBackShiftDepth）。
5、辅助功能：
提供调试信息打印（debugShow）、获取两帧间对应特征（getCorresponding）、输出深度向量（getDepthVector）等工具函数。

## **parameters.h、parameters.cpp**
- **功能**：这两个文件是vins_estimator中负责参数配置与读取的核心模块。它们的主要作用是：
定义系统运行所需的所有关键参数并声明为全局变量，供系统其他模块使用。同时实现参数读取逻辑，从 ROS 参数服务器和配置文件中加载参数，并初始化上述全局变量。
- **实现步骤**：
1、parameters.h 头文件
声明全局变量：定义系统所需的参数（如 IMU 噪声、相机 - IMU 外参、求解器迭代次数等），使用extern关键字使其在其他文件中可访问；
定义常量与枚举：包含固定常量（如焦距FOCAL_LENGTH、滑动窗口大小WINDOW_SIZE）和枚举类型（如状态参数化维度SIZE_PARAMETERIZATION、状态顺序StateOrder），用于规范系统中的参数维度和索引；
声明函数：声明参数读取函数readParameters，供parameters.cpp实现。
2、parameters.cpp 源文件
实现工具函数readParam：一个模板函数，通过 ROS 的NodeHandle从参数服务器读取指定名称的参数，并处理读取失败的情况（如打印错误并关闭节点）；
实现核心函数readParameters，包括：
从 ROS 参数服务器读取配置文件路径；
使用 OpenCV 的FileStorage打开配置文件，依次读取并初始化参数：
传感器相关参数（IMU 话题名称、图像尺寸、卷帘快门参数）；
求解器参数（最大求解时间、迭代次数、关键帧选择的最小视差）；
输出路径配置（创建结果目录、初始化结果文件）；
IMU 噪声参数（加速度计 / 陀螺仪的高斯噪声和随机游走噪声）、重力加速度；
相机 - IMU 外参（根据estimate_extrinsic参数决定是否固定、优化初始值或完全校准）；
时间偏移参数（是否在线估计传感器时间差TD）；
释放配置文件资源。

### **estimator.h、estimator.cpp**
- **功能**：它们共同实现了一个核心估计器，即Estimator类。该类通过融合 IMU数据和相机图像特征，实现实时状态估计（包括位置、姿态、速度、传感器偏差等），并处理初始化、滑动窗口优化、边缘化、故障检测等关键功能，最终输出高精度的位姿信息。
- **实现步骤**:
（1）初始化
状态清零：通过clearState()重置所有状态变量，为新的估计过程做准备。
外参标定：若需要估计相机与 IMU 的外参（ESTIMATE_EXTRINSIC=2），通过initial_ex_rotation.CalibrationExRotation()利用特征匹配计算旋转外参。
视觉初始化：当滑动窗口满（frame_count == WINDOW_SIZE）时，调用initialStructure()进行运动恢复结构（SFM），通过多视图几何估计相机位姿和三维特征点。
视觉 - IMU 对齐：visualInitialAlign()将 SFM 得到的视觉结构与 IMU 数据对齐，求解尺度、重力方向和速度，完成初始化并切换到非线性优化模式。
（2）数据处理
IMU 数据处理：processIMU()对 IMU 数据进行预积分（IntegrationBase），实时更新当前帧的位姿、速度，并缓存数据用于后续优化。
图像数据处理：processImage()接收图像特征，通过FeatureManager管理特征点的匹配与跟踪，判断是否为关键帧，并触发初始化或非线性优化。
（3）非线性优化
构建优化问题：optimization()使用 Ceres 求解器，构建包含 IMU 预积分因子、视觉投影因子和边缘化因子的残差方程。
参数转换：vector2double()和double2vector()实现状态变量（位姿、速度、偏差等）在 Eigen 向量与 double 数组之间的转换，适配 Ceres 的优化接口。
求解与更新：通过 Ceres 求解优化问题，更新状态变量，并处理边缘化（保留滑动窗口中旧帧的约束信息）。
（4）滑动窗口管理
窗口滑动：slideWindow()根据关键帧判断结果（marginalization_flag），选择边缘化旧帧（slideWindowOld()）或次新帧（slideWindowNew()），维持窗口大小固定。
边缘化：移除窗口中某一帧时，通过MarginalizationInfo保留该帧与其他帧的约束，转化为边缘化因子，避免信息丢失。
（5）故障检测与重定位
故障检测：failureDetection()通过检查特征数量、IMU 偏差、位姿突变等条件，判断系统是否失效，若失效则重置状态。
重定位：setReloFrame()接收重定位信息（匹配点、参考位姿），在优化中添加重定位约束，修正漂移。

### **estimator_node.cpp**
- **功能**:该文件为核心节点文件，主要功能是实现视觉 - 惯性里程计的前端处理与状态估计。它通过订阅 IMU数据、视觉特征点数据、重定位信息等，完成传感器数据的同步、预处理、状态预测与更新，并最终发布位姿、点云等估计结果，为后续的导航与定位提供基础。
- **实现步骤**:
1、初始化与参数设置
初始化 ROS 节点，读取配置参数，设置估计器（Estimator）的参数。
创建 ROS 发布者（如发布位姿、IMU 中间数据、时间偏差等）和订阅者（订阅 IMU、视觉特征、重定位信息、重启命令等）。
2、数据接收与缓冲
通过回调函数（imu_callback、feature_callback等）接收外部传感器数据。
将接收的 IMU 数据、视觉特征数据分别存入线程安全的缓冲区（imu_buf、feature_buf），使用互斥锁（mutex）保证数据操作的安全性。
3、数据同步与处理
启动专门的处理线程（process函数），通过条件变量（condition_variable）等待缓冲区中有可用数据。
调用getMeasurements函数从缓冲区中提取时间同步的 IMU 数据与视觉特征数据（确保 IMU 数据覆盖视觉帧的时间范围）。
4、状态估计与更新
对同步后的 IMU 数据进行预处理（如积分计算），并调用估计器的processIMU方法更新状态（位置、姿态、速度等）。
对视觉特征数据进行解析，提取特征点的坐标、速度等信息，调用估计器的processImage方法进行视觉惯性融合，优化状态估计结果。
5、重定位与重启处理
接收重定位信息（relocalization_callback），并设置估计器的重定位帧，辅助状态修正。
接收重启命令（restart_callback）时，清空缓冲区并重置估计器状态，重新开始估计过程。
6、结果发布
处理完成后，发布位姿（pubOdometry）、关键帧（pubKeyframe）、点云（pubPointCloud）、TF 变换（pubTF）等结果，以及中间数据（如两帧间 IMU 数据、时间偏差）。

---
# BW_multi_sensor_fusion
