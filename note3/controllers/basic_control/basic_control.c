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

#define TIME_STEP 10

#include "include\initialize.h"
#include "include\sensor.h"
#include "include\basic_pack.h"

int main(int argc, char **argv) {
  wb_robot_init();

  Initialize_chassis(&motor_device[0]);
  Initialize_pos(&pos_device[0]);
  Initialize_key();
  
  while (wb_robot_step(TIME_STEP) != -1) {

    pos_get_data(&pos[0],&pos_device[0]);
    
    velocity_get_data();
    master();

    printf("---------------------------------\n");
    printf("pos: %f %f %f %f\n",pos[0],pos[1],pos[2],pos[3]);
    printf("--------------------------------\n");
  };
  wb_robot_cleanup();
  return 0;
}