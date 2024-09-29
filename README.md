# SnowyOwl3.0
a quadrotor hardware flight platform  

## Hardware
Jetson Xavier NX 8GB  
RTSO-6002E_V1.2  
512AN_MMW  

## System
L4t Version: R35.1.0  
Jetpack: 5.0.2  
OS: Ubuntu20.04  
TensorRT: 8.4.1  
cuDNN: 8.4.1  
CUDA: 11.4.14  
OpenCV: 4.5.4  
### input system
sudo ./flash.sh rtso-6002e-v1.2 mmcblk0p1  
### to EMMC
PARTUUID  
### install Jetpack
sdkmanager  

## Tools
Tsinghua Mirror: Ubuntu Ports & pypi  
terminator  
vscode1.65.2  
git  

## Enviroment
ROS: Noetic  
TensorFlow: jp/v502 tensorflow=1.15.5+nv22.12  

## SnowyOwl installation
cd Desktop  
mkdir -p catkin_ws/src  
cd catkin_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release  
cd src
git clone https://github.com/lixiaoxin97/SnowyOwl.git    
(dependencies.yaml)  
rm .config/ros.org/rqt_gui.ini  
sudo apt install python-is-python3  
catkin build  
cd  
echo "~/Desktop/catkin_ws/devel/setup.bash" >> .bashrc  
source .bashrc  

## Code Management
cd ~/Desktop/catkin_ws/src/SnowyOwl  
git remote set-url origin https://Token@github.com/lixiaoxin97/SnowyOwl.git  
git push  