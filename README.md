# failsafe-class

基于原 failsafe 整合的单文件 failsafe 类版本。

## 使用

通过实例化 Failsafe 对象并调用 `check()` 或者创建新线程运行 `runtime_thread()`来检测。同时使用 topic 实时更新 `imu_status`。  
然后查看实例化对象成员变量 `Failsafe::ok` 的值即可。

## 原理

实时检测各传感器的信息的发送情况，并在无响应/无效响应做出相应反应。

## 实现

failsafe 的实现基于硬件连接检测和话题内容检测（软件）。

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
