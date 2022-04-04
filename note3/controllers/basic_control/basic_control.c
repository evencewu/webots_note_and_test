//机器参数质心离轴0.1
//  总重mass = 10.02
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

int main(int argc, char **argv) {
  wb_robot_init();

  char RF_motor;
  RF_motor = wb_robot_get_device("RF_MOTOR");
  wb_motor_set_position(RF_motor, INFINITY);
  wb_motor_set_velocity(RF_motor, 0);
  
  char RB_motor;
  RB_motor = wb_robot_get_device("RB_MOTOR");
  wb_motor_set_position(RB_motor, INFINITY);
  wb_motor_set_velocity(RB_motor, 0);

  while (wb_robot_step(TIME_STEP) != -1) {
    wb_motor_set_velocity(RF_motor,-0.1);
    wb_motor_set_velocity(RB_motor,0.1);
     
  };
  wb_robot_cleanup();

  return 0;
}