//声明头文件和依赖
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

//把相机和imu的csv文件映射成对应的结构体
struct CamData {
    double timestamp;
    std::string filename;
};

struct ImuData {
    double timestamp;
    double acc_x, acc_y, acc_z;
    double gyro_x, gyro_y, gyro_z;
};

//读取相机
//定义一个函数loadCamCsv，输入参数是CSV文件路径path，返回值是相机数据的数组
std::vector<CamData> loadCamCsv(const std::string& path) {
    //定义一个空的vector，用于存放解析出来的所有CamData
    std::vector<CamData> data;
    //打开path路径的csv
    std::ifstream file(path);
    //逐行读取
    std::string line;
    //一开始对表头的验证
    bool headerSkipped = false;
    //循环读取文件的每一行，存入line
    while (std::getline(file, line)) {
        //如果headerSkipped == false，就把它标记为true并跳过，相当于是丢掉第一行的表头不作为数据去处理。
        if (!headerSkipped) { headerSkipped = true; continue; }
        //变行成一个可逐个读取的流
        std::stringstream ss(line);
        //先取时间戳再取文件名
        std::string ts, fname;
        std::getline(ss, ts, ',');
        std::getline(ss, fname, ',');
        //验证数据有效性，检查时间戳和文件名都不是空字符串
        if (!ts.empty() && !fname.empty()) {
            //创建CamData结构体实例d
            CamData d;
            //赋值
            d.timestamp = std::stod(ts);
            d.filename = fname;
            //将数据添加到存储器保存
            data.push_back(d);
        }
    }
    //循环结束数据返回结构体
    return data;
}

//读取imu
//定义一个函数loadImuCsv，输入参数是CSV文件路径path，返回值是imu数据的数组
std::vector<ImuData> loadImuCsv(const std::string& path) {
    //定义一个空的vector，用于存放解析出来的所有CamData
    std::vector<ImuData> data;
    //打开path路径的csv
    std::ifstream file(path);
    //逐行读取
    std::string line;
    //一开始对表头的验证
    bool headerSkipped = false;
    //循环读取文件的每一行，存入line
    while (std::getline(file, line)) {
        //如果headerSkipped == false，就把它标记为true并跳过，相当于是丢掉第一行的表头不作为数据去处理。
        if (!headerSkipped) { headerSkipped = true; continue; }
        //变行成一个可逐个读取的流
        std::stringstream ss(line);
        //按顺序处理ts, ax, ay, az, gx, gy, gz
        std::string ts, ax, ay, az, gx, gy, gz;
        std::getline(ss, ts, ',');
        std::getline(ss, ax, ',');
        std::getline(ss, ay, ',');
        std::getline(ss, az, ',');
        std::getline(ss, gx, ',');
        std::getline(ss, gy, ',');
        std::getline(ss, gz, ',');
        //验证数据有效性，检查时间戳和文件名都不是空字符串
        if (!ts.empty()) {
            //创建实例
            ImuData d;
            //赋值
            d.timestamp = std::stod(ts);
            d.acc_x = std::stod(ax);
            d.acc_y = std::stod(ay);
            d.acc_z = std::stod(az);
            d.gyro_x = std::stod(gx);
            d.gyro_y = std::stod(gy);
            d.gyro_z = std::stod(gz);
            //数据保存
            data.push_back(d);
        }
    }
    //循环结束数据返回结构体
    return data;
}

int main(int argc, char** argv) {
    //节点名为pub_node
    ros::init(argc, argv, "pub_node");
    //使用私有命名空间读取参数
    ros::NodeHandle nh("~");

    //读取路径,从上到下分别是修正后的相机时间戳地址、相机图像数据文件夹地址、imu数据csv文件地址
    std::string cam_csv_path, image_dir, imu_csv_path;
    nh.param<std::string>("cam_csv", cam_csv_path, "/home/fusion/fusion_data/cam0_aligned.csv");
    nh.param<std::string>("image_dir", image_dir, "/home/fusion/data/cam0/");
    nh.param<std::string>("imu_csv", imu_csv_path, "/home/fusion/data/imu0_data.csv");

    //两个publisher发布topic，队列大小设定的大，避免丢包
    ros::Publisher pub_cam = nh.advertise<sensor_msgs::Image>("/cam_image", 100);
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/imu_data", 1000);
    
    //用于OpenCV和ROS Image转换
    cv_bridge::CvImage bridge;

    //读取CSV数据到内存向量cam_data、imu_data
    auto cam_data = loadCamCsv(cam_csv_path);
    auto imu_data = loadImuCsv(imu_csv_path);

    //等待订阅者连接，必须相机和IMU都有订阅者才开始，防止publish先发fusion尚未订阅丢数据
    ROS_INFO("Waiting for subscribers to connect...");
    while (ros::ok() && (pub_cam.getNumSubscribers() == 0 || pub_imu.getNumSubscribers() == 0)) {
        //循环每执行一次就暂停0.1秒防止等待过程中因为检查而卡死
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Subscribers detected on both topics, start publishing!");

    size_t i = 0, j = 0;
    //快速发送数据，因为kalibr标定只需传输数据不需要模拟真实的发布速率，所以快点发完
    ros::Rate loop_rate(1000); 

    //检测数据是不是全部发送出去了，只要还有任意一种数据没发完，就继续循环，发布时间戳在前的数据，时间戳相同先发布相机的
    while (ros::ok() && (i < cam_data.size() || j < imu_data.size())) {
        //发布相机
        if (i < cam_data.size() && (j >= imu_data.size() || cam_data[i].timestamp <= imu_data[j].timestamp)) {
            //通过目录+文件名拼成图片完整路径
            std::string img_path = image_dir + cam_data[i].filename;
            //读图
            cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
            //有图片就发送图片
            if (!img.empty()) {
                std_msgs::Header header;
                header.stamp = ros::Time().fromSec(cam_data[i].timestamp);
                header.frame_id = cam_data[i].filename; 
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
                pub_cam.publish(msg);
            }
            i++;
        //发送imu
        } else if (j < imu_data.size()) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time().fromSec(imu_data[j].timestamp);
            imu_msg.header.frame_id = "imu";
            imu_msg.linear_acceleration.x = imu_data[j].acc_x;
            imu_msg.linear_acceleration.y = imu_data[j].acc_y;
            imu_msg.linear_acceleration.z = imu_data[j].acc_z;
            imu_msg.angular_velocity.x = imu_data[j].gyro_x;
            imu_msg.angular_velocity.y = imu_data[j].gyro_y;
            imu_msg.angular_velocity.z = imu_data[j].gyro_z;
            pub_imu.publish(imu_msg);
            j++;
        }
        //保证循环过程中能处理订阅回调
        ros::spinOnce();
        //前后呼应控制速度
        loop_rate.sleep();
    }

    ROS_INFO("Data publish finished. Total cam: %zu, imu: %zu", cam_data.size(), imu_data.size());
    ROS_INFO("Waiting 2.0 s to let messages flush...");
    //等待2秒保证全部发布
    ros::Duration(2.0).sleep();
    ROS_INFO("Publisher exiting now.");
    return 0;
}
