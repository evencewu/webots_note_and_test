#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <webots/position_sensor.h>

#ifndef _INITIALIZE_
#define _INITIALIZE_

#define l1 0.15
#define l2 0.31819805153394638598037996294718
#define d 0.15

//全局姿态参数
double want_x_R = 0; 
double want_y_R = -0.225;
double want_x_L = 0; 
double want_y_L = -0.225;
//底盘电机初始化函数
char motor_device[6];
double before_pos[6] = {0,0,0,0,0,0};//辅助变量
double velocity[6];                  //电机角速度单位rand/s
void Initialize_chassis(char *motor_device){
  char motor_names[6][10] = {"RF_MOTOR","RB_MOTOR",
                            "LF_MOTOR","LB_MOTOR",
                            "L_MOTOR","R_MOTOR"};
  for (int i = 0; i < 6; i++){
    *(motor_device+i) = wb_robot_get_device(motor_names[i]);
    wb_motor_set_position(*(motor_device+i), INFINITY);
    wb_motor_set_velocity(*(motor_device+i), 0);
  }
}

//编码器初始化函数
double pos[6];
char pos_device[6];
void Initialize_pos(char *pos_device){
  char pos_names[6][8] = {"RF_POS"     ,"RB_POS",
                          "LF_POS"     ,"LB_POS",
                          "L_POS","R_POS"};
  for (int i = 0; i < 6; i++){
    *(pos_device+i) = wb_robot_get_device(pos_names[i]);
    wb_position_sensor_enable(*(pos_device+i),TIME_STEP);
  }
}

int key;
void Initialize_key(){
  wb_keyboard_enable(TIME_STEP); 
}


#endif