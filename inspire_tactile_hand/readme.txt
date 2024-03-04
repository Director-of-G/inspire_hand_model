------
（这部分内容由因时提供）

一、模型简介：本模型为Inspire Robots触觉手产品模型，在Solidworks中进行建模，使用sw2urdf插件将模型导出为urdf格式，再通过手工修改，产生并联结构，并使之可以在ros-gazebo环境中进行仿真实验。

二、运行环境：Win10 | Solidworks2021 | Ubuntu20.04.1 | ROS1-noetic | gazebo11.11.0

三、运行方法：roslaunch display.launch来在Rviz中查看模型；roslaunch mylaunch.launch来在gazebo中查看模型；若要使模型运动，可安装并使用ros-noetic-gazebo-ros-control插件，相关代码已准备好，也可自行编写插件。

四、注意事项
	1.本urdf文件仅能被gazebo正确读取，在其他平台打开将丢失并联结构（包括Rviz）。
	2.进行gazebo仿真时，建议在左侧Physics菜单中关闭重力，重力会严重影响模型运动。
	3.除大拇指外，其余四指仅有一个主动关节（驱动器），其他均为从动关节；大拇指包括两个驱动器，对应两个主动关节。


------
（这部分内容由姜永鹏给出）

一、请将inspire_tactile_hand文件夹放在ROS工作空间的src目录下

二、控制因时手的代码位于inspire_tactile_hand/scripts

三、请先运行mylaunch_hand.launch，再运行hand_ros_control.py，该脚本：(1)读取joint states，(2)控制因时手到指定位置。Gazebo仿真已禁用灵巧手的重力。
