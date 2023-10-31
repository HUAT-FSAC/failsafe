# failsafe

## 原理

实时检测各传感器的信息的发送情况，并在无响应/无效响应时控制车辆行为。

## 启动

由于现行的设计模式为：工控机接受来自 VCU 的 `common_msgs/vehicle_status/racing_num=2`（暂定为2） 的信号后，failsafe 开始启动（由 `start.sh` 提供）。在确认各传感器物理连接后发送 `common_msgs/vehicle_cmd/racing_status=1` 来示意 VCU 车辆可以启动，若在运行时出现问题就发送 `common_msgs/vehicle_cmd/racing_status=3` 以启动紧急制动。

因此在启动时应直接使用

```bash
roslaunch failsafe runtime_check.launch
```

直接启动运行时检测模式。

## 行为

如果各传感器物理连接稳定，则发送

```plain
rostopic echo vehcileCMDMsg

head1: 170
head2: 85
length: 10
steering: 0
brake_force: 0
pedal_ratio: 0
gear_position: 0
working_mode: 1
racing_num: 1
racing_status: 1
checksum: 3
```
如果在完成 `sensor initial check` 后出现传感器掉线情况，failsafe 将会发送 `racing_status = 3` 的 msg，其余部分与上面部分相同。（checksum 除外）  
而如果在没有完成的情况下， `_num` 变量将会重置，从而重新进行 `sensor initial check`。


## 实现

failsafe 的实现基于硬件连接检测和话题内容检测（软件）。
并通过 ROS Topic 向 VCU 发送控制信号。

### 硬件连接

对于 USB 串口连接，通过检测 `/dev/` 下是否存在 `ttyUSB0` 这类文件即可实现对于 USB 串口设备的连接检测。  

> 由于 IMU 与 工控机的连接还要通过 RS434 转 USB 的转接器，因此仅检测物理 USB 连接来判断惯导是否在线并不现实。（IMU 被断电后 `/dev` 下仍存在相关 USB 接口文件）  
> 故另外检测 IMU 所发布的 topic 信息，`common_msgs/HUAT_ASENSING/ins_status` 的值即可以说明当前惯导状态（15 代表信号正常，可以正常工作；4 代表有连接但是没有有效信号）。  
> 另外在断电之后，topic 的值不会主动更新，因此还需要手动判断最后的 topic 信息是否有实质性的更新。
 
而对于通过以太网连接的激光雷达、相机、VCU等，则通过查看 `/sys/class/ent/{eth name}/operstate` 的内容即可。

### 内容检测

针对话题的内容特征进行识别，如输出图片的像素大小，IMU 输出的设备状态等实现检测。

目前已经实现：

- 针对 IMU 的基于 topic 的状态检测

## 注意

~~由于IPC -> VCU 的消息传递（vehicle_cmd.msg）没有设计来自于无人系统的故障信息，故采用 “8字环绕” 的模式作为无人系统故障信号。~~
