# iAts
A antenna tracker for UAV.

## 项目介绍
Arduino自动跟踪云台
蓝牙版AAT

用于装载定向高增益图传天线，通过图传的音频通道传送飞机的坐标位置与高度，再根据本地位置与高度计算出需要指向的方向与俯仰角度，从而将自动将天线指向飞机，不需要人肉调整天线角度，更加专注于飞行与享受FPV的过程。

大部分代码基本来源于开源项目 amv-open360tracker_RX_v1_user 项目地址 https://github.com/raul-ortega/amv-open360tracker/  
修改了以下功能：
1. 修改了程序流程
2. 开发了手机APP进行调参与监控（通过蓝牙连接）
3. 增加了HC05蓝牙模块与手机连接，通过手机APP进行调参
4. 与手机APP通讯，将飞机位置数据通过APP传到手机（使用自定义TLV格式的协议）
5. 手机APP上显示飞机的位置

## 特点

- 低成本，基于328p芯片运行，可以直接使用[MWC MultiWii SE v2.6](https://s.taobao.com/search?q=MWC+MultiWii+SE+v2.6&imgfile=&js=1&stats_click=search_radio_all%3A1&initiative_id=staobaoz_20181117&ie=utf8)飞控板，或是使用[Arduino Pro Mini](https://store.arduino.cc/usa/arduino-pro-mini)+[HMC5883模块](https://s.taobao.com/search?q=HMC5883L&commend=all&ssid=s5-e&search_type=mall&sourceId=tb.index&area=c2c&spm=a1z02.1.6856637.d4910789)/[QMC5883模块](https://s.taobao.com/search?q=QMC5883&imgfile=&js=1&stats_click=search_radio_all%3A1&initiative_id=staobaoz_20181116&ie=utf8)+[MPU6050模块](https://s.taobao.com/search?q=MPU6050&imgfile=&js=1&stats_click=search_radio_all%3A1&initiative_id=staobaoz_20181116&ie=utf8)进行构建。
- 云台的运动使用舵机驱动，水平方向使用360°舵机，俯仰方向使用180°或90°舵机，低成本的舵机可以使用[MG995](https://item.taobao.com/item.htm?spm=a230r.1.14.20.24c52706K8ff73&id=45136514387&ns=1&abbucket=11#detail)，它有360°版本与180°版本。
- [OLED](https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15846751795.28.57594e5cFUP7UZ&id=524527857468)屏幕显示数据，可以显示飞机坐标，速度，GPS星树，本地位置，值向等数据。
- 开发了手机APP进行调参与监控，可以在APP的地图中显示飞机的位置。
- 发射端可以支持NMEA协议与MAVLink协议，输出标准的LTM协议与地面端通信。
- 使用图传的音频通道传输数据到地面端，地面端与发射端都使用[TCM3105芯片](https://s.taobao.com/search?q=TCM3105&imgfile=&js=1&stats_click=search_radio_all%3A1&initiative_id=staobaoz_20181117&ie=utf8)进行数模转换。
- 云台旋转部分与底座需要使用[微型滑环](https://s.taobao.com/search?q=%E5%BE%AE%E5%9E%8B%E6%BB%91%E7%8E%AF&imgfile=&js=1&stats_click=search_radio_all%3A1&initiative_id=staobaoz_20181117&ie=utf8)，防止绕线。
- 使用卡尔曼滤波算法融合加速度计与指南针数据，减少指南针漂移引起的抖动。
- 地面端使用了MOZ8上djzoom大佬设计的PCB，原帖：[让每个人都能用起的舵机版AAT](http://www.moz8.com/forum.php?mod=viewthread&tid=83868&highlight=%E8%88%B5%E6%9C%BA%2BAAT&_dsign=e2235060)
- 发射端让同事帮忙画了新的PCB。
- AAT通过蓝牙连接手机，通过手机获取本地GPS位置，省略了“设置家”的步骤或是“本地GPS模块”。
  
## 成品展示

#### 发射机

焊好的发射端，由于使用5V供电，所以电源部分可以不需要焊了

<img src="docs/images/TX_Image1.png?raw=true" width="640" height="480" alt="TX">

#### 前面板

前面板包含以下几个部分：
- 3个LED灯
  红色接收到图传信号并解析成功数据开始闪烁，蓝色在手机蓝牙连接成功并向手机发送数据闪烁，绿色在接收到手机发送的数据闪烁。
- 12V电源插口
  用于给图传接收机供电。
- AUX音视频插口 
  用于连接图传接收机，传输视频与音频信号，视频信号通过滑环穿过底座接显示屏，音频信号通过TCM3105芯片转换为数字信号。
- OLED屏幕
  显示GPS坐标，速度，方向，云台指向，云台电压，本地GPS坐标，高度等数据。
  
 <img src="docs/images/AAT_Image2.png?raw=true" width="640" height="480" alt="RX">

#### 内部细节

##### MPU6050模块
MPU6050用于测量水平旋转的加速度，使用卡尔曼滤波算法融合测量的指南针数据进行计算，消除指南针数据的漂移。

<img src="docs/images/AAT_Image3.png?raw=true" width="640" height="480" alt="RX">

##### HMC5883L模块
HMC5883L磁力计用于测量指向的角度，通过融合MPU6050的加速度数据进行卡尔曼滤波来消除漂移。

<img src="docs/images/AAT_Image4.png?raw=true" width="640" height="480" alt="RX">

##### 微型滑环
平台与底座使用滑环导电，通过滑环可以防止360°旋转的时候将导线缠绕。

<img src="docs/images/AAT_Image5.png?raw=true" width="640" height="480" alt="RX">

##### HC05蓝牙模块
蓝牙模块用于与手机蓝牙连接

<img src="docs/images/AAT_Image6.png?raw=true" width="640" height="480" alt="RX">

##### 接收板
接收板上焊接了Arduino Pro Mini模块，开关电源模块，TCM3105芯片，供电分做量部分，通过滑环进来的12V电源，一部分通过开关电源模块降压到5V给两个舵机供电，好处是支持电流大，效率高，但是纹波比较大，所以后部有滤波电容，另一部分通过LM7805电源芯片降压到5V给Pro Mini供电。

<img src="docs/images/AAT_Image7.png?raw=true" width="640" height="480" alt="RX">

##### 水平360°舵机
水平舵机提供360°的旋转支持。

<img src="docs/images/AAT_Image8.png?raw=true" width="640" height="480" alt="RX">

##### 内部整体图

<img src="docs/images/AAT_Image9.png?raw=true" width="640" height="480" alt="RX">

#### 外部细节

##### 底座
底座机械部分由djzoom群里的大佬设计加工，我购买的成品，底座上的齿轮固定，通过水平舵机上的齿轮旋转带动云台水平转动，底座内有两个轴承支撑平台，滑环的线通过轴承中心穿出底座，由于底座使用了CNC加工，虽然坚固漂亮，但是成本较高。

<img src="docs/images/AAT_Image10.png?raw=true" width="640" height="480" alt="RX">

##### 1/4英寸螺孔
底部提供了标准的1/4英寸螺孔，可以直接安装到普通的相机三脚架上。

<img src="docs/images/AAT_Image11.png?raw=true" width="640" height="480" alt="RX">

##### 外部整体图
外壳使用了5mm黑色亚克力加工了上盖板与下盖板，上下盖板间使用了40mm铝柱支撑固定,俯仰固定安装在上盖板上，使用了舵机配送的摇臂来安装天线支架，天线支架使用1/4英寸手拧螺丝来安装[枫叶23db大平板](https://item.taobao.com/item.htm?spm=a1z10.5-c.w4002-10110629054.23.3cf95b54f4w6pb&id=543939433629)，图传接收机使用了开源的双接收，用3D打印的外壳，直接用双面胶粘在盖板上，另外把水平尺也装在了盖板上，用于使用时调平。

<img src="docs/images/AAT_Image12.png?raw=true" width="640" height="480" alt="RX">

#### 外场使用

##### 整体图
正在测试的成品，用的是枫叶大平板，由于比较重，有点抖。

<img src="docs/images/AAT_Image1.png?raw=true" width="400" height="640" alt="成品">

##### APP截图
APP跟踪界面截图，可以看到飞机位置，经纬度，高度，距离，速度，AAT的电压，指向角度，俯仰角度

![APP](docs/images/APP_Image1.jpg?raw=true "APP")

#### 视频：<Br/>
- [模拟测试](https://www.bilibili.com/video/av31897946/) <Br/>
- [上飞机实测](https://www.bilibili.com/video/av33431415/)

## 安装教程

  #### 接收端原理图
  ![Schematic_diagram_RX](PCB/Schematic_diagram_RX.png?raw=true "Schematic_diagram_RX")
  
  #### 发射端原理图
  ![Schematic_diagram_TX](PCB/Schematic_diagram_TX.png?raw=true "Schematic_diagram_TX")
  
   #### 打样回来的PCB
  ![PCB](docs/images/PCB.png?raw=true "PCB")
  

## 使用说明
