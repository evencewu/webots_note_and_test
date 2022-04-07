#ifndef _TYPE_DEF_
#define _TYPE_DEF_

#define l1 0.15
#define l2 0.31819805153394638598037996294718
#define d 0.15

//序列定义
#define roll  0
#define pitch 1
#define yaw   2

#define RB_MOTOR  0
#define RF_MOTOR  1
#define LB_MOTOR  2
#define LF_MOTOR  3
#define R_MOTOR   4
#define L_MOTOR   5

#define RB_POS  0
#define RF_POS  1
#define LB_POS  2
#define LF_POS  3
#define R_POS   4
#define L_POS   5

//物理&特征常数
#define MASS_BODY 30
#define MASS_COMPONENT 0.5
#define MOTO 6

#define MOTOR_NUM 6

#include <include_webots/motor.h>
#include <include_webots/position_sensor.h>
#include <include_webots/inertial_unit.h>
#include <include_webots/accelerometer.h>
#include <include_webots/gyro.h>
#include <include_webots/keyboard.h>
#include <include_webots/mouse.h>
#include <include_webots/types.h>
#include <include_webots/camera.h>
#include <include_webots/robot.h>

//电机参数
typedef struct motor_feature{
  WbDeviceTag ID;
  const char *name;
  double MAX_TORQUE;
  double torque;
  double torque_fb;
  double torque_tgt;
  double omg;
  double angle;
  double angle_last; //
}MOTOR;

//编码器参数
typedef struct position_sensor_feature{
  WbDeviceTag ID;
  const char *name;
  double resolusion;
  double position;//rad
  double position_last;
  double w;//rad/s
  double w_last;
}POSITION_SENSOR;

//imu参数
typedef struct imu_feature{
  WbDeviceTag ID;
  const char *name;
  double angle_value[3]; //rad
}IMU;

//accelerometer参数
typedef struct accelerometer_feature
{
  WbDeviceTag accelerometer_ID;
  const char *accelerometer_name;
  double accelerometer_value[3];
}ACCE;

//gyro参数
typedef struct gyro_feature
{
  WbDeviceTag gyro_ID;
  const char *gyro_name;
  double gyro_value[3];
}GYRO;

typedef struct LEG
{
    //足端坐标
    double Px;
    double Py;
    //足端速度
    double Vx;
    double Vy;
    //足端力
    double Fx;
    double Fy;
} LEG;

typedef struct robot
{
    double radius_of_wheel; //m
    double height_of_center; //m
    double mass_body;  //kg
    double mass_wheel; //kg
    double velocity;  //m/s

    LEG leg1;
    LEG leg2;

    MOTOR motor[6];
    POSITION_SENSOR position_sensor[6];
    IMU imu;
    ACCE acce;
    GYRO gyro;

} robot;


void robot_init();
    void motor_init(double angle_set);


void flash_sensor_data();
    void MOTOR_data();



#endif
