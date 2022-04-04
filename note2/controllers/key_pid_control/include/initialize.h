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

//底盘电机初始化函数
void Initialize_chassis(char *wheels){
  char wheels_names[2][8] = {"MOTOR1","MOTOR2"};
  for (int i = 0; i < 2; i++){
    *(wheels+i) = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(*(wheels+i), INFINITY);
    wb_motor_set_velocity(*(wheels+i), 0);
  }
}

//角度传感器初始化函数
void Initialize_imu(char *imu_device){
  *imu_device = wb_robot_get_device("IMU");
  wb_inertial_unit_enable(*imu_device, TIME_STEP);
}

//角速度传感器初始化函数
void Initialize_gyro(char *gyro_device){
  *gyro_device = wb_robot_get_device("GYRO");
  wb_gyro_enable(*gyro_device, TIME_STEP);
}

//加速度传感器初始化函数
void Initialize_accelerometer(char *accelerometer_device){
  *accelerometer_device = wb_robot_get_device("ACCELEROMETER");
  wb_accelerometer_enable(*accelerometer_device ,TIME_STEP);
}

//磁力传感器初始化函数
void Initialize_compass(char *compass_device){
  *compass_device = wb_robot_get_device("COMPASS");
  wb_compass_enable(*compass_device ,TIME_STEP);
}

//编码器初始化函数
void Initialize_pos(char *pos_device){
  *pos_device     = wb_robot_get_device("POS1");
  *(pos_device+1) = wb_robot_get_device("POS2");
  wb_position_sensor_enable(*pos_device,TIME_STEP);
  wb_position_sensor_enable(*(pos_device+1),TIME_STEP);
}

#endif