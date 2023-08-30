#include "main.hpp"
#include <string>

void init(ros::NodeHandle nh) {
    ros::param::get("keep_running", keep_running);
    ros::param::get("/failsafe/camera_eth_interface", cam_eth);
    ros::param::get("/failsafe/lidar_eth_interface", lidar_eth);
    ros::param::get("/failsafe/accept_only", accept_only);

    cam_sub = nh.subscribe("/pylon_camera_node/image_raw", 1, cam_callback);
    lidar_sub = nh.subscribe("/velodyne_points", 2, lidar_callback);
    imu_sub = nh.subscribe("/INS/ASENSING", 1, imu_callback);

    control_pub = nh.advertise<common_msgs::vehicle_cmd>("/vehcileCMDMsg", 1); // wasn't my fault, just keep it.

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
        cmd.racing_status = 3; // stands for a problem occured
        _ok = false;
    }
    cmd.checksum = cmd.steering + cmd.brake_force + cmd.pedal_ratio +
                   cmd.gear_position + cmd.working_mode + cmd.racing_num +
                   cmd.racing_status;

    if (type == FAILURE_NO || runtime_init_flag) {
        ROS_DEBUG_STREAM("cmd sent, racing_status: " +
                         std::to_string(cmd.racing_status));
        control_pub.publish(cmd);
    } else {
        ROS_DEBUG("Out of some reason, cmd was not sent.");
        ROS_DEBUG_STREAM("type: " + std::to_string(type) +
                         "\truntimeflag: " + std::to_string(runtime_init_flag));
    }
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
    _ok = true;
    // usb connection
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port < 0) {
        ROS_ERROR("IMU Failure");
        alert(FAILURE_IMU);
    } else {
        ROS_DEBUG("IMU Connected");
    }

    // ethnernet connection
    try {
        // lidar
        file.open("/sys/class/net/" + lidar_eth + "/operstate");
        ROS_INFO_STREAM("lidar: " + lidar_eth);
        file >> buffer;
        if (buffer != "up") {
            ROS_ERROR("Lidar Failure");
            alert(FAILURE_LIDAR);
        } else {
            ROS_DEBUG("Lidar Connected");
        }
        file.close();

        // camera
        file.open("/sys/class/net/" + cam_eth + "/operstate");
        ROS_INFO_STREAM("cam: " + cam_eth);
        file >> buffer;
        if (buffer != "up") {
            ROS_ERROR("Camera Failure");
            alert(FAILURE_CAM);
        } else {
            ROS_DEBUG("Camera Connected");
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
    ROS_INFO("AS Sensor Runtime Monitor started");

    while (ros::ok()) {
        if (accept_only) {
            ROS_WARN("Only sending check passed msgs!");
            ROS_INFO_STREAM("Checking.." + std::to_string(_num));
            alert(FAILURE_NO);
            _num++;
        } else {
            ROS_DEBUG("Checking begin..");
            hardwareCheck();

            // runtime initial check
            if (!runtime_init_flag) {
                if (_ok) {
                    if (_num < 40) {
                        alert(FAILURE_NO); // vehicle initial check successed
                        ROS_DEBUG_STREAM("pre-initial check successed, current num: " +
                                  std::to_string(_num));
                        _num++;
                    } else {
                        runtime_init_flag = true;
                        ROS_INFO("AS Sensor initial self-check Successed, "
                                 "final check started...");
                        alert(FAILURE_NO); // last time sent
                    }
                } else {
                    ROS_INFO("Initial Check Failed, resetting internal check "
                             "counter...");
                    _num = 0;
                }
            }
        }
        rate.sleep();
    }

    ROS_WARN("Failsafe has reached its end, stopping now");
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