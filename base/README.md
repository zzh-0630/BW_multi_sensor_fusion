<!-- TODO -->
base层：放一些底层传感器驱动代码

- bw_ros_driver为北微传感产品的ros驱动,目前支持ah系列，mins系列

- usb_cam为V4L类型相机的ros驱动程序
  - 相机驱动程序包如果出现以下报错：
    ```
      CMake Error at /usr/share/cmake-3.16/Modules/FindPkgConfig.cmake:463 (message):
      A required package was not found
    ```
    则说明需要安装v4l依赖，执行如下命令：
   `sudo apt-get install libv4l-dev`