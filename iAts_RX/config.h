#ifndef CONFIG_H
#define CONFIG_H

#define CRIUS_SE //Default

//#define DEBUG
//#define TELEMETRY_DEBUG

#define BLUETOOTH_SERIAL_BAUDRATE 9600         //蓝牙信号串口波特率
#define DIGITAL_SERIAL_BAUDRATE 1200           //数字信号串口波特率

/* #### 罗盘类型 ####
 *
 *  GY-271模块有分 国产QMC5883L（IC丝印D5883）进口HMC5883L(IC丝印L883)
 *  淘宝上很多商家将QMC5883L标识为HMC5883L出售，一不小心就买错了（比如我。。）
 *  网上QMC5883L的资料很少，所幸找到了芯片手册跟QMC5883L与HMC5883L的寄存器对比参考，按照手册与对比参考，撸了一遍QMC5883L读取地磁数据的代码
 *  
 */
#define COMPASS_HMC5983L
//#define COMPASS_QMC5883L

/* 
	#### 打开使用kalman滤波 ####
*/
#define USEKLM

/* #### 电压测量 ####
 *
 *  Arduino针脚供电最大5V，电流最大40毫安左右，所以测量12V电压需要进行分压。
 *  用电阻串联可以将电压分散，两个阻值一样的电阻串联，则每个电阻两端的电压为总电压的一半，那么一大一小两个电阻串联，小电阻两端电压 = （V*(R2/R1))
 *  
 */
#define BATTERYMONITORING_RESISTOR_1 4000   //分压电阻1
#define BATTERYMONITORING_RESISTOR_2 1000   //分压电阻2
#define BATTERYMONITORING_CORRECTION 1.03    //修正值

 /* ### Easing effect for tilt movements (EXPERIMENTAL)
 *
 * TILT_EASING_STEPS is the maximun number of steps to reach tilt position using easing functions
 *
 * 0 or 1:   normal use, no easing effect applied, servo will reach tilt position in only 1 step.
 * >1:  servo will move to tilt position in with easing effect.
 * 60 steps ~ 1 second
 * 30 steps ~ 1/2 seconds
 * 15 stips ~ 1/4 seconds
 *
 * TILT_EASING_MIN_ANGLE: Easing is applied if the difference between last and new tilt position is greater than TILT_EASING_MIN_ANGLE grade.
 * TILT_EASING_MILIS: Is the time in miliseconds spend by echa step.
 */
#define TILT_EASING_STEPS 20    // default 10
#define TILT_EASING_MIN_ANGLE 2 // default 4
#define TILT_EASING_MILIS 15    //default 15

 /*
 * ### Easing ecuations
 *  EASE_OUT_QRT ->  The easing effect is only applied at the end of the movement.
 *  EASE_INOUT_QRT -> The easing effect is applied at the begining and at the end of the movement.
 *  EASE_OUT_CIRC -> The easing out circular effect.
 *  Read more: Easing ecuations by Robert Penner, http://www.gizma.com/easing/
 */
 //#define EASE_OUT_QRT    // Easing Out Quart function
 //#define EASE_INOUT_QRT  // Easing In Out Quart function
#define EASE_OUT_CIRC     // Easing Out Circular function (Default)

/* #### Compass declination ####
 *
 * http://magnetic-declination.com/
 * Enter your city and then get the value for Magnetic declination
 * for example [Magnetic declination: 3° 2' EAST]
 *
 * now enter the value in the format DEGREE.MINUTE * 10 -> 3.2 * 10 = 32
 *
 * set to 0 if you cannot find your declination!
 */
 
#define DECLINATION -10 //default 32 //磁偏角
#define COMPASS_OFFSET 90

#endif