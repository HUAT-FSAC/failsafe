#include "main.hpp"

#include <exception>
#include <stdio.h>

#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR
#include <string>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

void init(ros::NodeHandle nh)
{
    nh.param<bool>("keep_running", keep_running, false);
    nh.param<std::string>("camera_eth_interface", cam_eth, "noexist");
    nh.param<std::string>("lidar_eth_interface", lidar_eth, "noexist");

    cam_sub = nh.subscribe("/pylon_camera_node/image_raw", 1, cam_callback);
    lidar_sub = nh.subscribe("/velodyne_points", 2, lidar_callback);
    imu_sub = nh.subscribe("/INS/ASENSING", 1, imu_callback);

    control_pub =
        nh.advertise<common_msgs::vehicle_cmd>("/failsafe_control", 1);
}

void alert(int type)
{
    if (type == FAILURE_CAM)
    {
        ROS_ERROR("Camera Failure");
        // TODO disable camera related modules
        // TODO send stop cmd
    }
    else if (type == FAILURE_IMU)
    {
        ROS_ERROR("IMU Failure");
    }
    else if (type == FAILURE_LIDAR)
    {
        ROS_ERROR("Lidar Failure");
    }
    else
    {
        ROS_WARN("No failure but alert was activated.");
    }
    control_pub.publish(error_cmd);
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr)
{
    _cloud.data = cloud_ptr->data;
}

void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msg)
{
    // if the msg is corrupted
    // _pos = msg;
}

// sensor_msgs/Image
void cam_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    _image.data = msg->data;
}

void contentCheck() {}

void hardwareCheck()
{
    // usb connection
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port < 0)
    {
        alert(FAILURE_IMU);
    }

    // ethnernet connection
    try
    {
        // camera
        file.open("/sys/class/net/" + cam_eth + "/operstate");
        file >> buffer;
        if (buffer != "up")
        {
            alert(FAILURE_CAM);
        }
        file.close();

        // lidar
        file.open("/sys/class/net/" + lidar_eth + "/operstate");
        file >> buffer;
        if (buffer != "up")
        {
            alert(FAILURE_LIDAR);
        }
        file.close();
    }
    catch (const std::exception &e)
    {
        // ROS_WARNING(e.what().d);
    }

    // icmp detection
}

// TODO make eth detect a function

void checkOnce()
{
    ROS_INFO("AS Sensor Check Monitor initiated");
    hardwareCheck();
    ros::shutdown();
}

void checkRuntime()
{
    ROS_INFO("AS Sensor Runtime Monitor initiated...");
    // TODO init a new thread to run simultaneously
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "failsafe");

    ros::NodeHandle nh;
    init(nh);

    if (!keep_running) // running mode flag
    {
        checkOnce();
    }
    else
    {
        checkRuntime();
    }

    ros::spin();
}