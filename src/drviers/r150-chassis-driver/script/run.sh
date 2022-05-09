echo "同步系统时间..."
sudo ntpdate 10.42.0.1
echo "启动驱动程序..."
cd /home/xag/r150_driver
source devel/setup.bash
echo "赋予接口权限..."
sudo chmod 777 /dev/ttyUSB0
export ROS_MASTER_URI=http://10.42.0.212:11311
roslaunch chassis_driver chassis_driver_node.launch
