
# README

底盘启动程序，用于同ROS平台与底盘通讯。

## 启动

底盘启动命令：

```sh
roslaunch chassis_driver chassis_driver_node.launch
```

输入主题：`/cmd_vel`
输出主题：`/odom`（尙未实现）

~~rivz查看里程计：~~

```sh
roslaunch chassis_driver odom_viewer.launch
```

## 脚本

`controller.sh`：用于启动控制程序。
`run.sh`：nuc上的启动程序。
`viewer.sh`：主机上远程查看窗口。