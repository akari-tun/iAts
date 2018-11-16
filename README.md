# iAts
A antenna tracker for UAV.

#### 项目介绍
Arduino自动跟踪云台
蓝牙版AAT

#### 软件架构
大部分代码基本来源于开源项目 amv-open360tracker_RX_v1_user 项目地址 https://github.com/raul-ortega/amv-open360tracker/  

修改了以下功能：
1. 重写了loop()函数
1. APP蓝牙调参 
2. 蓝牙传输GPS数据到APP，在手机APP上显示飞机位置  
3. 优化PID算法，加入变积分过程  
4. 加入国产QMC5883芯片的支持  
5. 加入MPU6050并使用卡尔曼滤波，消除指南针漂移
  
APP正在完善，代码暂时未上传。。。

#### 成品展示

<iframe src="//player.bilibili.com/player.html?aid=31897946&cid=55789384&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

#### 安装教程


#### 使用说明
