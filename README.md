# failsafe

## 原理

实时检测各传感器的信息的发送情况，并在无响应/无效响应时控制车辆行为。

## 启动

由于现行的设计模式为：工控机接受来自 VCU 的 `common_msgs/vehicle_status/racing_num=2`（暂定为2） 的信号后，failsafe 开始启动。在确认各传感器物理连接后发送 `common_msgs/vehicle_cmd/racing_status=1` 来示意 VCU 车辆可以启动，若在运行时出现问题就发送 `common_msgs/vehicle_cmd/racing_status=3` 以启动紧急制动。

因此在启动时应直接使用

```bash
roslaunch failsafe runtime_check.launch
```

直接启动运行时检测模式。

## 实现

failsafe 的实现基于硬件连接检测和话题内容检测（软件）。
并通过 ROS Topic 向 VCU 发送控制信号。

### 硬件连接

对于 USB 串口连接，通过检测 `/dev/` 下是否存在 `ttyUSB0` 这类文件即可实现对于 USB 串口设备的连接检测。而对于通过以太网连接的激光雷达、相机、VCU等，则通过查看 `/sys/class/ent/{eth name}/operstate` 的内容即可。

### 内容检测

针对话题的内容特征进行识别，如输出图片的像素大小，IMU 输出的设备状态等实现检测。

## 注意

由于IPC -> VCU 的消息传递（vehicle_cmd.msg）没有设计来自于无人系统的故障信息，故采用 “8字环绕” 的模式作为无人系统故障信号。