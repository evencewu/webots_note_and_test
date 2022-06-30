#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <webots/position_sensor.h>

#ifndef _SENSOR_
#define _SENSOR_

//imu的刷新
void imu_get_data(double *imu,char device){
  *imu     = *  wb_inertial_unit_get_roll_pitch_yaw(device);
  *(imu+1) = * (wb_inertial_unit_get_roll_pitch_yaw(device)+1);
  *(imu+2) = * (wb_inertial_unit_get_roll_pitch_yaw(device)+2);
}
//gyro的刷新
void gyro_get_data(double *gyro,char device){
  *gyro     = *  wb_gyro_get_values(device);
  *(gyro+1) = * (wb_gyro_get_values(device)+1); 
  *(gyro+2) = * (wb_gyro_get_values(device)+2);
}
//accelerometer的刷新
void accelerometer_get_data(double *accelerometer,char device){
  *accelerometer     = *  wb_accelerometer_get_values(device);
  *(accelerometer+1) = * (wb_accelerometer_get_values(device)+1);
  *(accelerometer+2) = * (wb_accelerometer_get_values(device)+2);
}

//compass的刷新
void compass_get_data(double *compass,char device){
  *compass     = *  wb_compass_get_values(device);
  *(compass+1) = * (wb_compass_get_values(device)+1);
  *(compass+2) = * (wb_compass_get_values(device)+2);
}

//pos的刷新
void pos_get_data(double *pos,char *device){
  *pos     =  wb_position_sensor_get_value(*device);
  *(pos+1) =  wb_position_sensor_get_value(*(device + 1));
}

//motor_sensor的刷新
void motor_sensor_get_data(double *velocity,char *wheels){
  *velocity = wb_motor_get_velocity(*wheels);
}

#endif