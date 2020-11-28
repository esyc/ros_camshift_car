# ros_camshift_car
基于ros，opencv的视觉跟踪小车（python）
主从机配置
主机：树莓派3B+
系统:ubuntu mate 16.04
ros版本：ros-kinect

从机：虚拟机ubuntu 18.04
ros版本：ros-melodic
python : py2.7
opencv: 4.1.0.25

主机（树莓派小车）：
启动地盘运动控制节点接收"cmd_vel"控制移动
启动usb_cam节点启动连接在小车上的usb摄像头
从机（pc虚拟机）：
启动视觉跟踪节点


usb_cam节点：
主机将usb摄像头接收数据发送到ros master

视觉跟踪节点：
从机接收ros master的图像数据，使用ros opencv bridge转换为opencv的图像格式, 采用camshift算法跟踪图像，根据中心偏移判断小车方向，发送cmd_vel给ros master

地盘节点：
接收cmd_vel,控制小车运动

usb_cam 可以直接使用 apt-get install安装

这边给出的是视觉跟踪节点的代码

注意需要配置ros的主从机通信主从机都要配置/etc/hosts文件和bashrc的文件配置 使得主从机可以相互接收到topic
