#ifndef _TYPE_DEF_
#define _TYPE_DEF_

#define l1 0.15
#define l2 0.31819805153394638598037996294718
#define d 0.15

//序列定义
#define roll  0
#define pitch 1
#define yaw   2

#define L 1
#define R 0

#define RB_MOTOR  0
#define RF_MOTOR  1
#define LB_MOTOR  2
#define LF_MOTOR  3
#define R_MOTOR   4
#define L_MOTOR   5

//物理&特征常数
#define MASS_BODY 30
#define MASS_COMPONENT 0.5

#define MOTOR_NUM 6

#define e 2.71828

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

  double torque;//力矩
  double torque_fb;//读取
  double torque_last;//上一个时间戳

  //控制参数
  double I;
  
  double want_P;
  double want_W;

  double angle;
  double angle_last; 
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
    //足端力（实时）
    double Fx;
    double Fy;
    //目标足端坐标
    double want_Px;
    double want_Py;
} LEG;

typedef struct robot
{
    double radius_of_wheel; //m
    double height_of_center; //m
    double mass_body;  //kg
    double mass_wheel; //kg
    double velocity;  //m/s

    LEG leg[2];
    

    MOTOR motor[6];
    POSITION_SENSOR position_sensor[6];
    IMU imu;
    ACCE acce;
    GYRO gyro;

} robot;


void robot_init();
    void motor_init(double angle_set);
    void imu_init();
    void acce_init();
    void gyro_init();
    void key_mouse_init();

void flash_sensor_data();
    void MOTOR_data();
    void LEG_data();
        void LEG_solution_pos(int RL,double A,double B);
        void LEG_solution_pos_opposite(int RL,double X,double Y);
        void LEG_solution_speed(int RL,double A ,double B ,double v1,double v2);
        void LEG_solution_speed_opposite(int RL,double A,double B,double vx_1,double vy_1);

void control_master();
    void stable_leg(int RL,double x,double y,double w_x ,double w_y,
                    double a,double b);
void print_data();        
void perform_motor();         

#endif
