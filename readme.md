# DashToCones

> Coursework project for DIP class. The goal is to use vision to guide the Dashgo robot through two traffic cones in bright color.

## Environment Setting

1. Thinkpad L480 laptop
    * ROS kinetic and Ubuntu 16
    * preconfigured for HITSZ K322
2. Dashgo robot hardware
    * one USB cable to communicate with laptop
    * press power button to turn on or off
    * remember to open the emergency cap to allow motion

## Usage

1. In whatever folder

   ```
   git clone https://gitee.com/yueqianliu/dash-to-cones.git
   cd dash-to-cones
   catkin_make
   ```

2. plug the USB cable to the laptop

3. power up the robot and unscrew the emergency cap

4. launch nodes with

   ```
   roslaunch dashgo_driver driver_imu.launch
   rosrun vision_nav vision_nav_node
   # or use lazy_button.sh
   ```

   