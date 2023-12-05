#include "ros/ros.h"
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <fcntl.h>
#include <fstream>
#include <string>

// TODO: add a debug option, when turned on ignore all sensor status and return
// true

class Failsafe {
public:
  bool ok = false; // update after every check()
  bool initial_check_done = false;
  int imu_status =
      -1; // initial state, indicating it's not updated by topic msg
  int imu_curr_times = 0;

  bool run_once(void);
  // bool run_for(double time); // not implemented
  void runtime_thread();
  void stop_runtime(void);
  bool is_time_lapse_init = false;

private:
  const int IMU_RETRY_TIMES = 15;

  bool _stop = true;
  bool _runtime_freq = 10; // default is 10 Hz
  // TODO: update freq
  bool check(void);
  bool check_lidar_conn(void);
  bool check_camera_conn(void);
  bool check_usb_connection(void);
  bool check_imu_topic(void);
  std::chrono::time_point<std::chrono::steady_clock> cstartTime = std::chrono::steady_clock::now();
  std::chrono::time_point<std::chrono::steady_clock> cendTime = std::chrono::steady_clock::now();
  std::chrono::time_point<std::chrono::steady_clock> checkTime = std::chrono::steady_clock::now();
};

bool Failsafe::check_lidar_conn(void) {
  std::ifstream file("/sys/class/net/enp2s0f1/operstate");
  if (!file) {
    ROS_ERROR("Failed to open lidar operstate file");
    return false;
  }
  std::string buffer;
  file >> buffer;
  file.close();
  if (buffer != "up") {
    ROS_ERROR("Lidar Failure");
    return false;
  }
  ROS_DEBUG("Lidar Connected");
  return true;
}

bool Failsafe::check_camera_conn(void) {
  std::ifstream file("/sys/class/net/enp2s0f0/operstate");
  if (!file) {
    ROS_ERROR("Failed to open camera operstate file");
    return false;
  }
  std::string buffer;
  file >> buffer;
  file.close();

  if (buffer != "up") {
    ROS_ERROR("Camera Failure");
    return false;
  }
  ROS_DEBUG("Camera Connected");
  return true;
}

bool Failsafe::check_usb_connection(void) {
  int _imu = access("/dev/ttyUSB0", F_OK);
  if (_imu == 0) {
    ROS_DEBUG("IMU Physcially Connected");
    return true;
  } else {
    ROS_ERROR("IMU is not connected");
    return false;
  }
}

bool Failsafe::check_imu_topic(void) {
  if (imu_status == 0 && Failsafe::imu_curr_times < Failsafe::IMU_RETRY_TIMES) {
    // TODO: setup a correct retry times
    // limiting retrying time below 1 second
    ROS_WARN_STREAM(
        "IMU Status is not refreshed yet, wait for another "
        "call...\nAvailable Retries: "
        << boost::lexical_cast<std::string>(Failsafe::IMU_RETRY_TIMES -
                                            Failsafe::imu_curr_times));
    // maybe try std::to_string
    Failsafe::imu_curr_times++;
    return true;
  } else if (imu_status == 0 &&
             Failsafe::imu_curr_times >= Failsafe::IMU_RETRY_TIMES) {
    ROS_ERROR_STREAM("IMU Status cannot update after "
                     << Failsafe::IMU_RETRY_TIMES
                     << " times, seek for another solution...\nCurrent times: "
                     << Failsafe::imu_curr_times);
    Failsafe::imu_curr_times++;
    return false;
  } else if (imu_status == 15 || imu_status == 4) {
    imu_status = 0;
    return true;
  } else {
    // condition of imu_status != 0/15/4
    ROS_ERROR_STREAM("IMU Status is invalid. Current status code: " +
                     std::to_string(imu_status));
    return false;
  }
}

bool Failsafe::check(void) {
  if (check_camera_conn() && check_lidar_conn() && check_usb_connection() &&
      check_imu_topic()) {
    // if (check_imu_topic()) {
    Failsafe::ok = true;
  } else {
    Failsafe::ok = false;
  }
  return Failsafe::ok;
}

bool Failsafe::run_once(void) { return Failsafe::check(); }

void Failsafe::stop_runtime(void) {
  ROS_INFO("Stopping Failsafe runtime...");
  Failsafe::_stop = false;
}

void Failsafe::runtime_thread(void) {
  // 依然存在子线程必要性，因为 runtime 是持续性检测，不能在 ros
  // 主线程之上否则会阻塞 topic msgs 的接受

  ROS_INFO("Runtime checking thread started!");

  while (Failsafe::_stop) {
    boost::this_thread::sleep_for(
        boost::chrono::milliseconds{1000 / Failsafe::_runtime_freq});
    // this delay is the best when compatible with topic update speed

    bool _result = Failsafe::check();

    if (_result) {
      if (!Failsafe::initial_check_done) {
        if (!Failsafe::is_time_lapse_init) {
          ROS_WARN("Time lapse check init!!!");
          // start time lapse check for initial
          is_time_lapse_init = true;
          cstartTime = std::chrono::steady_clock::now();
        } else if (Failsafe::is_time_lapse_init) {
          cendTime = std::chrono::steady_clock::now();
					int64_t _checkTime = ((std::chrono::duration_cast<std::chrono::milliseconds>(cendTime - cstartTime)).count()) / 1000;

          if(_checkTime >= 5) {
            ROS_INFO("Finihsed on Initial Check");
            Failsafe::initial_check_done = true;
            Failsafe::is_time_lapse_init = false;
            // v_cmd.racing_status = 1;
            // failsafe don't control cmd anymore
            Failsafe::ok = true;
          } else {
            ROS_INFO_THROTTLE(0.5, "Check ok, Time lapsing...");
          }
        }
      } else {
        // initial check is done
      }
    } else {
      // when sensor failure occurs
      if (Failsafe::is_time_lapse_init) {
        ROS_WARN("Time lapse Check failed, time lapse will be restarted");
        is_time_lapse_init = false;
      }

      if (Failsafe::initial_check_done) {
        ROS_ERROR_THROTTLE(0.2, "Runtime Check Failed");
        // v_cmd.racing_status = 3;
        Failsafe::ok = false;
      }
    }
  }
  return;
}