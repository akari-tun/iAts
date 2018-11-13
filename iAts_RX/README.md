# T_Auto_Tracker

#### 项目介绍
Arduino自动跟踪云台
蓝牙版AAT

#### 软件架构
大部分代码基本来源于开源项目 amv-open360tracker_RX_v1_user 项目地址 https://github.com/raul-ortega/amv-open360tracker/  

修改了以下功能：
1. 重写了loop()函数
1. 蓝牙调参，蓝牙上传数 
2. 蓝牙传输GPS数据，在手机上显示飞机位置  
3. 修改了一点点发射端与接收端的通讯协议，传输了更多的数据  
4. 优化PID算法，加入变积分过程  
5. 加入国产QMC5883芯片的支持  
6. 加入MPU6050并使用卡尔曼滤波，消除指南针漂移
  
针对此固件的APP正在开发中。。。


#### 安装教程

1. xxxx
2. xxxx
3. xxxx

#### 使用说明

1. xxxx
2. xxxx
3. xxxx