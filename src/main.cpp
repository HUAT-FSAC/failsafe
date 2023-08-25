#include "main.hpp"
#include "ros/init.h"
#include "ros/param.h"
#include "ros/rate.h"
#include <unistd.h>

void init(ros::NodeHandle nh) {

    
    // ROS_WARN_STREAM(nh.param("keep_running", keep_running, true));

    ros::param::get("keep_running", keep_running);
    ROS_WARN_STREAM(keep_running);
    ros::param::get("camera_eth_interface", cam_eth);
    ros::param::get("lidar_eth_interface", lidar_eth);

    cam_sub = nh.subscribe("/pylon_camera_node/image_raw", 1, cam_callback);
    lidar_sub = nh.subscribe("/velodyne_points", 2, lidar_callback);
    imu_sub = nh.subscribe("/INS/ASENSING", 1, imu_callback);

    control_pub =
        nh.advertise<common_msgs::vehicle_cmd>("/failsafe_control", 1);

    // init predefined value for vehicle cmd
    // values here are refered from pure_pursuit/PP_car
    cmd.head1 = 0xAA;
    cmd.head2 = 0x55;
    cmd.length = 10;
    cmd.steering = 0;
    cmd.brake_force = 0;
    cmd.pedal_ratio = 0;
    cmd.gear_position = 0;
    cmd.working_mode = 1;
    cmd.racing_num = 1;
}

void alert(int type) {
    if (type == FAILURE_NO) {
        cmd.racing_status = 1; // stands for no problem
    } else {
        if (type == FAILURE_CAM) {
            ROS_ERROR("Camera Failure");
            // TODO disable camera related modules
        } else if (type == FAILURE_IMU) {
            ROS_ERROR("IMU Failure");
        } else if (type == FAILURE_LIDAR) {
            ROS_ERROR("Lidar Failure");
        }
        cmd.racing_status = 3; // stands for a problem occured
    }
    cmd.checksum = cmd.steering + cmd.brake_force + cmd.pedal_ratio +
                   cmd.gear_position + cmd.working_mode + cmd.racing_num +
                   cmd.racing_status;
    control_pub.publish(cmd);
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
    _cloud.data = cloud_ptr->data;
}

void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msg) {
    // TODO check if the msg is corrupted
    _pos.ins_status = msg->ins_status;
}

// sensor_msgs/Image
void cam_callback(const sensor_msgs::Image::ConstPtr &msg) {
    _image.data = msg->data;
}

void contentCheck() {}

void hardwareCheck() {
    // usb connection
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port < 0) {
        alert(FAILURE_IMU);
        _ok = false;
    }

    // ethnernet connection
    try {
        // camera
        file.open("/sys/class/net/" + cam_eth + "/operstate");
        file >> buffer;
        if (buffer != "up") {
            alert(FAILURE_CAM);
            _ok = false;
        }
        file.close();

        // lidar
        file.open("/sys/class/net/" + lidar_eth + "/operstate");
        file >> buffer;
        if (buffer != "up") {
            alert(FAILURE_LIDAR);
            _ok = false;
        }
        file.close();
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM(e.what());
    }

    // icmp detection
}

void checkOnce() {
    ROS_INFO("AS Sensor Once Check initiated");
    hardwareCheck();
    ros::shutdown();
}

void checkRuntime() {
    ros::Rate rate(2); // rate in herz
    ROS_INFO("AS Sensor Runtime Monitor initiated...");

    while (ros::ok()) {
        hardwareCheck();

        if (!runtime_init_flag) {
            if (_ok) {
                ROS_INFO("AS Sensor INITIAL Check Successed");
                alert(FAILURE_NO); // vehicle initial check successed
                runtime_init_flag = true;
            } else {
                ROS_INFO("AS Sensor INITIAL Check Failed");
            }
        }

        rate.sleep();
    }
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "failsafe");

    ros::NodeHandle nh;
    init(nh);

    // TODO cannot retrive value from param label
    if (!keep_running) // running mode flag
    {
        checkOnce();
    } else {
        checkRuntime();
        // unavailable now
    }

    ros::spin();
}