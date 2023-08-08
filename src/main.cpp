#include "main.hpp"

void init(ros::NodeHandle nh)
{
    ros::Subscriber cam_sub =
        nh.subscribe("/pylon_camera_node/image_raw", 1, cam_callback);
    ros::Subscriber lidar_sub =
        nh.subscribe("/velodyne_points", 2, lidar_callback);
    ros::Subscriber imu_sub = nh.subscribe("/INS/ASENSING", 1, imu_callback);

    ros::Publisher control_pub =
        nh.advertise<common_msgs::vehicle_cmd>("failsafe_control", 100);
}

void alert(int type)
{
    if (type == FAILURE_CAM)
    {
        ROS_INFO("Camera Failure");
        // TODO disable camera related modules
        // TODO send stop cmd
    }
    else if (type == FAILURE_IMU)
    {
        ROS_INFO("IMU Failure");
    }
    else if (type == FAILURE_LIDAR)
    {
        ROS_INFO("Lidar Failure");
    }
    else
    {
        ROS_INFO("No failure but alert was activated.");
    }
    // control_pub.publish();
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &original_cloud_ptr)
{
}

void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs)
{
    // if the msg is corrupted
}

// sensor_msgs/Image
void cam_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    int _width = msg->width;
    int _height = msg->height;

    // size check
    if (_width < 1080 && _height < 1920)
    {
        alert(0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "failsafe");

    ros::NodeHandle nh;
    ros::Subscriber cam_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber imu_sub;
    ros::Publisher control_pub;

    init(nh);
    ros::spin();
}