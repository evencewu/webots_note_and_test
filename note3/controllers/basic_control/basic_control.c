//**定义
/**
 * ！！！注：代码定义部分大量借（c）鉴（v）ROBOT4代码！！！
 * 机器定义
 *          
 *           ||
 *           || 
 *       LB  ||  LF
 * ==========||===========> X轴      向外y轴
 *       RB  ||  RF
 *           ||
 *           ||
 *           \/ 
 *           Z轴
 * 
 * 质量：3 + 0.1*6 =3.6 kg
 * 参数：短臂  l1 = 0.15 m
 *      长臂  l2 = 0.31819805153394638598037996294718 m
 *      轴距  l1 = 0.15 m
 * 
 * 
 * 
**/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <include_webots\accelerometer.h>
#include <include_webots\brake.h>
#include <include_webots\camera_recognition_object.h>
#include <include_webots\camera.h>
#include <include_webots\compass.h>
#include <include_webots\connector.h>
#include <include_webots\console.h>
#include <include_webots\device.h>
#include <include_webots\differential_wheels.h>
#include <include_webots\display.h>
#include <include_webots\distance_sensor.h>
#include <include_webots\emitter.h>
#include <include_webots\gps.h>
#include <include_webots\gyro.h>
#include <include_webots\inertial_unit.h>
#include <include_webots\joystick.h>
#include <include_webots\keyboard.h>
#include <include_webots\led.h>
#include <include_webots\lidar_point.h>
#include <include_webots\lidar.h>
#include <include_webots\light_sensor.h>
#include <include_webots\microphone.h>
#include <include_webots\motor.h>
#include <include_webots\mouse_state.h>
#include <include_webots\mouse.h>
#include <include_webots\nodes.h>
#include <include_webots\position_sensor.h>
#include <include_webots\pen.h>
#include <include_webots\radar_target.h>
#include <include_webots\radar.h>
#include <include_webots\radio.h>
#include <include_webots\range_finder.h>
#include <include_webots\receiver.h>
#include <include_webots\remote_control.h>
#include <include_webots\robot_window.h>
#include <include_webots\robot_wwi.h>
#include <include_webots\robot.h>
#include <include_webots\skin.h>
#include <include_webots\speaker.h>
#include <include_webots\supervisor.h>
#include <include_webots\touch_sensor.h>
#include <include_webots\types.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 10

#include "include\type_def.h"

robot ROBOT;

#include "include\initialize.h"
#include "include\sensor.h"

int main(int argc, char **argv) {
  wb_robot_init();//系统初始化
  
  robot_init();

  wb_motor_set_available_torque(ROBOT.motor[0].ID, 1 * ROBOT.motor[0].torque);
  wb_motor_set_available_torque(ROBOT.motor[1].ID, 1 * ROBOT.motor[1].torque);
  wb_motor_set_available_torque(ROBOT.motor[2].ID, 1 * ROBOT.motor[2].torque);
  wb_motor_set_available_torque(ROBOT.motor[3].ID, 1 * ROBOT.motor[3].torque);

  while (wb_robot_step(TIME_STEP) != -1) {
    flash_sensor_data();//刷新全局传感器数据和预测数据
  };
  wb_robot_cleanup();
  return 0;
}

void robot_init(){
  ROBOT.height_of_center = 0.225;
  ROBOT.radius_of_wheel = 0.06;
  ROBOT.mass_body = MASS_BODY;
  //标定
  motor_init(0);
  position_sensor_init();
  //imu_init();
  //acce_init();
  //gyro_init();
}

void flash_sensor_data(){
  MOTOR_data();
}


