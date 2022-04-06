#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>

#ifndef _SENSOR_
#define _SENSOR_

//pos的刷新
void pos_get_data(double *pos,char *device){
  for (int i = 0; i < 6; i++){
    *(pos+i) =  wb_position_sensor_get_value(*(device + i));
  }
}

//velocity刷新（基于pos）单位rand/s
void velocity_get_data(){
  for (int i = 0; i < 6; i++){
    velocity[i] =  (pos[i] - before_pos[i])/TIME_STEP;
  }
  for (int i = 0; i < 6; i++){
    before_pos[i] =  pos[i];
  }
}

#endif