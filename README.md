# iAts
A antenna tracker for UAV.

## 项目介绍
Arduino自动跟踪云台
蓝牙版AAT

用于装载定向高增益图传天线，通过图传的音频通道传送飞机的坐标位置与高度，再根据本地位置与高度计算出需要指向的方向与俯仰角度，从而将自动将天线指向飞机，不需要人肉调整天线角度，更加专注于飞行与享受FPV的过程。

大部分代码基本来源于开源项目 amv-open360tracker_RX_v1_user 项目地址 https://github.com/raul-ortega/amv-open360tracker/  
修改了以下功能：
1. 修改了程序流程
2. 做了手机APP进行调参与监控（通过蓝牙连接）
3. 增加了蓝牙调参的功能，通过HC05蓝牙模块与手机连接，通过手机APP进行调参
4. 与手机APP通讯，将飞机位置数据通过APP传到手机（使用自定义的TLV协议）
5. 手机APP上显示飞机的位置

## 特点

- 低成本，基于328p芯片运行，可以直接使用[MWC](http://www.multiwii.com)飞控板，或是使用[Arduino Pro Mini](https://store.arduino.cc/usa/arduino-pro-mini)+[HMC5883模块](https://s.taobao.com/search?q=HMC5883L&commend=all&ssid=s5-e&search_type=mall&sourceId=tb.index&area=c2c&spm=a1z02.1.6856637.d4910789)/[QMC5883模块](https://s.taobao.com/search?q=QMC5883&imgfile=&js=1&stats_click=search_radio_all%3A1&initiative_id=staobaoz_20181116&ie=utf8)+[MPU6050模块](https://s.taobao.com/search?q=MPU6050&imgfile=&js=1&stats_click=search_radio_all%3A1&initiative_id=staobaoz_20181116&ie=utf8)进行构建。
- 云台的运动使用舵机驱动，水平方向使用360°舵机，俯仰方向使用180°或90°舵机，比如使用[MG995](https://item.taobao.com/item.htm?spm=a230r.1.14.20.24c52706K8ff73&id=45136514387&ns=1&abbucket=11#detail)。
- 开发了手机APP进行调参与监控，可以再APP的地图中显示飞机的位置。
- 发射端可以支持NMEA协议与MAVLink协议，输出标准的LTM协议与地面端通信。
- 使用图传的音频通道传数数据到地面端。
- 云台旋转部分与底座需要使用滑环，防止绕线。
- 使用卡尔曼滤波算法融合加速度计与指南针数据。
- 地面端使用了MOZ8上djzoom大佬设计的PCB，原帖：[让每个人都能用起的舵机版AAT](http://www.moz8.com/forum.php?mod=viewthread&tid=83868&highlight=%E8%88%B5%E6%9C%BA%2BAAT&_dsign=e2235060)
- 发射端让同事帮忙画了新的PCB。
- AAT通过蓝牙连接手机，通过手机获取本地GPS位置，省略了“设置家”的步骤或是“本地GPS模块”。
  

## 成品展示

#### 图片：<Br/>
打样回来的PCB，发射端跟接收端。
![PCB](docs/images/PCB.png?raw=true "PCB")

焊好的发射端，由于使用5V供电，所以电源部分可以不需要焊了
![RX](docs/images/RX_Image1.png?raw=true "RX")

正在测试的成品，用的是枫叶大平板，由于比较重，有点抖。
![成品](docs/images/AAT_Image1.png?raw=true "成品")

APP跟踪界面截图，可以看到飞机位置，经纬度，高度，距离，速度，AAT的电压，指向角度，俯仰角度
![APP](docs/images/APP_Image1.jpg?raw=true "APP")

#### 视频：<Br/>
- [模拟测试](https://www.bilibili.com/video/av31897946/) <Br/>
- [上飞机实测](https://www.bilibili.com/video/av33431415/)

## 安装教程


## 使用说明
