# Multi-sensor-fusion
# sensor_time_align
# 简介 Introduction
是一个基于ROS1 Noetic版的单目相机与IMU时间同步与数据融合工具，能够实现相机与imu数据的粗对齐与融合，为kalibr标定过程的自动时间偏移优化环节提供更合理的初始值，辅助优化过程。起到以下三个作用：
1. 缩小优化搜索范围，加速标定收敛
2. 降低陷入局部最优的风险
3. 兼容非严格同步的传感器硬件

---

# 功能 Features
- 实现三轴视觉角速度及角速度模长计算、imu角速度模长计算、时间偏移估计、时间戳校正
- 读取相机与IMU数据并发布话题 `/cam_image` 与 `/imu_data`
- 将相机信息与所对应区间的IMU信息融合为自定义消息 `/fused_topic`并发布

---

# 环境 Environment
- Ubuntu 20.04 
- ROS1 Noetic 
- OpenCV 4.2.0
- At least C++11

# 使用 Usage
- 准备相机与IMU原始数据。此处采用的是euroc数据集的MH01的数据，也可以接自己的设备。
- 编译运行cam_w_norm_calculate.cpp和imu_w_norm_calculate.cpp得到三轴视觉角速度及角速度模长、三轴imu角速度及角速度模长
- 编译运行time_offset_calculate_portion.cpp进行时间偏移估计
- 编译运行timestamp_correct.cpp实现时间戳校正
- 启动launch文件launch_cpp.launch实现消息的融合以及融合消息的发送

---

# 目录结构 Project Structure

catkin_ws/
└── src/
    └── sensor_time_align/
        ├── CMakeLists.txt
        ├── include
        │   └── sensor_time_align
        ├── launch
        │   └── launch_cpp.launch
        ├── msg
        │   └── FusedState.msg
        ├── package.xml
        ├── scripts
        └── src
            ├── cam_w_norm_calculate.cpp
            ├── imu_w_norm_calculate.cpp
            ├── time_offset_calculate_portion.cpp
            ├── timestamp_correct.cpp
            ├── fusion_node.cpp
            └── publish.cpp

---

#  文件说明 File Description

##  顶层构建文件

### **CMakeLists.txt**
- **功能**：ROS1 catkin构建配置，声明依赖、编译消息、生成可执行文件。

### **package.xml**
- **功能**：记录ROS包的元信息，定义包名、版本、维护者和依赖。

---

## src

### **cam_w_norm_calculate.cpp**
- **功能**：计算三轴视觉角速度以及角速度模长并保存。 
- **实现步骤**： 
  1. 遍历相邻的两帧图像数据记为若干组前后帧匹配对； 
  2. ORB算法提取相邻两帧图像的特征点和特征描述符； 
  3. 用BFMatcher进行特征匹配； 
  4. 提取匹配点的像素坐标；
  5. 估计本质矩阵，用RANSAC算法剔除外点；
  6. 解算得到旋转矩阵；
  7. Rodrigues变换将旋转矩阵转为旋转向量；
  8. 计算三轴视觉角速度以及角速度模长并输出结果。

### **imu_w_norm_calculate.cpp**
- **功能**：计算imu角速度模长并保存。
- **实现步骤**： 
  1. 由imu三轴角速度原始数据计算得到imu角速度模长并输出结果。

### **time_offset_calculate_portion.cpp**
- **功能**：对比相机和IMU各轴角速度分量序列，计算互相关从而估计时间偏移量。
- **实现步骤**： 
  1. 线性插值视觉数据到IMU数据序列。
  2. 设置时间偏移的搜索范围和搜索步长，对时间偏移逐个取值，分别对各轴分量进行归一化互相关的计算。
  3. 求得各轴归一化互相关最大时的时间偏移估计值，将相关性最强的一轴数据的时间偏移估计值记为时间偏移量并输出。
  
### **timestamp_correct.cpp**
- **功能**：修正相机时间戳并输出。
- **实现步骤**： 
  1. 由估算出的偏移量对相机时间戳进行修正，实现粗对齐。

### **publish.cpp**
- **功能**：ROS发布节点，读取数据发布话题 /cam_image 与 /imu_data。
- **实现步骤**： 
  1. 读取相机和imu信息。
  2. 等待相机和IMU都有订阅者连接后开始发布话题。 
  3. 按照时间顺序发布数据，先发布时间戳在前的数据，时间相同的话先发布相机的再发布imu的，同时在发布过程中检测数据是不是全部发送出去了，只要还有任意一种数据没发完，就继续循环发布。
  4. 发布结束后等待2秒，确保数据全部发布不遗漏。 

### **fusion_node.cpp**
- **功能**：ROS融合节点，订阅相机帧与区间内IMU数据，生成并发布融合数据话题 /fused_topic。
- **实现步骤**： 
  1. 订阅相机和imu话题。
  2. 发布融合后的信息。每一帧相机信息到来时按照FusedState.msg格式发布一次融合消息，将当前到达的图像的时间戳作为融合消息的时间戳，融合消息中的图像是当前到达的图像，imu数据是[上一帧图像时间，本帧图像时间）内缓存的各个imu的三轴加速度、三轴角速度、原始时间戳。第一帧相机数据特殊处理不包含imu信息。

---

## msg

### **FusedState.msg**
- **功能**：自定义融合消息，包括：时间戳、图像、IMU角速度、加速度、IMU时间戳。

---

## launch

### **launch_cpp.launch**
- **功能**：一键启动各个node，完成发布与融合。

---

# 消息定义 Message
## FusedState.msg:
time stamp
string image_filename                           #相机图片文件名
sensor_msgs/Image image                         #图像数据
geometry_msgs/Vector3[] angular_velocity        #存储多个IMU的角速度数据
geometry_msgs/Vector3[] linear_acceleration     #存储多个IMU的加速度数据
float64[] imu_timestamp                         #存储多个IMU数据的时间戳

---

# 实施步骤 Implementation steps
1. 安装ros1的noetic版，推荐使用包含opencv 4.2.0和rosdep 0.26.0的小鱼安装
wget http://fishros.com/install -O fishros && bash fishros
2. 如果你在vscode里面第一次调试运行使用opencv的库的cpp文件的话vscode找不到opencv会报错，此时需要修改.vscode里的tasks.json与c_cpp_properties.json让opencv可以被找到。本项目opencv4.2.0的opencv头文件路径为/usr/include/opencv4库文件路径为/usr/lib/x86_64-linux-gnu因此本项目tasks.json修改如下：
{
    "tasks": [
        {
            "type": "cppbuild",
            "label": "C/C++: g++ 生成带 OpenCV 的可执行文件",
            "command": "/usr/bin/g++",
            "args": [
                "-fdiagnostics-color=always",
                "-g",
                "-I/usr/include/opencv4",  //OpenCV头文件路径
                "${file}",  //待编译的源文件
                "-o",
                "${fileDirname}/${fileBasenameNoExtension}",  //输出可执行文件
                "-L/usr/lib/x86_64-linux-gnu",  //OpenCV库文件路径
                "-lopencv_core",
                "-lopencv_imgproc",
                "-lopencv_imgcodecs", 
                "-lopencv_highgui",
                "-lopencv_features2d", 
                "-lopencv_calib3d",     
                "-lopencv_video"       
            ],
            "options": {
                "cwd": "${fileDirname}"
            },
            "problemMatcher": [
                "$gcc"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "detail": "适配 OpenCV 的 C++ 编译任务，包含图像读写、特征提取、位姿估计等常用库"
        }
    ],
    "version": "2.0.0"
}
本项目c_cpp_properties.json修改如下：
{
  "configurations": [
    {
      "name": "linux-gcc-x64",
      "includePath": [
        "${workspaceFolder}/**",  //保留工作区自身的头文件路径
        "/usr/include/opencv4"    //OpenCV头文件路径
      ],
      "compilerPath": "/usr/bin/g++",
      "cStandard": "c11",
      "cppStandard": "c++11", 
      "intelliSenseMode": "linux-gcc-x64",
      "compilerArgs": [] 
    }
  ],
  "version": 4
}
3. 在vscode调试运行cam_w_norm_calculate.cpp文件，确保正确输入你的设备所对应的地址，即可输出包含结果的csv文件，该文件里包含时间戳以及对应的三轴视觉角速度和角速度模值。
4. 在vscode调试运行imu_w_norm_calculate.cpp文件，确保正确输入你的设备所对应的地址，即可输出包含结果的csv文件，该文件里包含时间戳以及对应的三轴imu角速度、三轴imu加速度和角速度模值。
5. 在vscode调试运行time_offset_calculate_portion.cpp文件，确保正确输入你的设备所对应的地址，可得三轴各自的时间偏移以及判断出的最佳偏移。
6. 在vscode调试运行timestamp_correct.cpp文件，确保正确输入你的设备所对应的地址，可得修正之后的相机时间戳。
7. 在catkin_ws里面去实现目录结构。没有的文件夹用mkdir命令创建，没有的文件用nano命令创建。
   在catkin_ws的src目录下创建sensor_time_align包：
   catkin_create_pkg sensor_time_align roscpp sensor_msgs cv_bridge opencv2 geometry_msgs message_generation message_runtime
   更改sensor_time_align的CMakeLists.txt如下：
cmake_minimum_required(VERSION 3.0.2)
project(sensor_time_align)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  image_transport
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  FusedState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(publish_node src/publish.cpp)
add_executable(fusion_node src/fusion_node.cpp)

add_dependencies(publish_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(publish_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(fusion_node
  ${catkin_LIBRARIES}
)

install(TARGETS publish_node fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
   
   更改sensor_time_align的package.xml如下：
<?xml version="1.0"?>
<package format="2">
  <name>sensor_time_align</name>
  <version>0.0.0</version>
  <description>The sensor_time_align package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="slam@todo.todo">slam</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/sensor_time_align</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>cv_bridge</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>cv_bridge</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package> 
   
8. 修改cpp文件或者msg文件之后需要编译，命令如下：
cd ~/catkin_ws
catkin_make
编译后、修改launch文件、打开新终端后最好重新source工作空间的环境变量确保系统识别新编译的文件,命令如下：
source devel/setup.bash 

9. 运行launch文件，命令如下：
roslaunch sensor_time_align launch_cpp.launch

---

# 示例日志 Example
## 融合消息发布成功
[INFO] [1759214737.277746161]: Published fused data: 1403636763.613555.png with 10 IMU samples
[INFO] [1759214737.291120011]: Published fused data: 1403636763.663555.png with 10 IMU samples
[INFO] [1759214737.305273432]: Published fused data: 1403636763.713556.png with 10 IMU samples
[INFO] [1759214737.319367720]: Published fused data: 1403636763.763556.png with 10 IMU samples
[INFO] [1759214737.332512779]: Published fused data: 1403636763.813555.png with 10 IMU samples

## 发布节点发布结束

[INFO] [1759214737.340724093]: Data publish finished. Total cam: 3682, imu: 36820

## 融合节点发布结束
[INFO] [1759214749.469983085]: Fusion node finished. Total camera frames: 3682, total IMU samples fused: 36810

---
# Multi-sensor-fusion
