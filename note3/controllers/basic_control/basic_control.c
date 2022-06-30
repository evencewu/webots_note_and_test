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
 * 输入读取端RF/LB_pos/motor 方向将置于默认状态，但在部分算法输入输出端将被反转以适应不同机构的算法同构
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


#define TIME_STEP 10

#include "include\type_def.h"

robot ROBOT;

#include "include\initialize.h"
#include "include\sensor.h"
#include "include\control.h"

int main(int argc, char **argv) {
  wb_robot_init();//系统初始化
  
  robot_init();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    flash_sensor_data();//刷新全局传感器数据  
    control_master();
    print_data();
    perform_motor();

  };
  wb_robot_cleanup();
  return 0;
};
//=====================================
void robot_init(){
  ROBOT.height_of_center = 0.225;
  ROBOT.radius_of_wheel = 0.06;
  ROBOT.mass_body = MASS_BODY;

  ROBOT.leg[R].want_Px = 0;
  ROBOT.leg[R].want_Py = -0.225;
  ROBOT.leg[L].want_Px = 0;
  ROBOT.leg[L].want_Py = -0.225;

  //标定
  motor_init(0);
  position_sensor_init();
  imu_init();
  acce_init();
  gyro_init();
  key_mouse_init();
};
//=====================================
void flash_sensor_data(){
  MOTOR_data();
  LEG_data();
  key_data();
};
//=====================================
void control_master(){
  stable_leg(R, ROBOT.leg[R].Px, ROBOT.leg[R].Py, ROBOT.leg[R].want_Px, ROBOT.leg[R].want_Py, ROBOT.position_sensor[RB_MOTOR].position, -ROBOT.position_sensor[RF_MOTOR].position);
  stable_leg(L, ROBOT.leg[L].Px, ROBOT.leg[L].Py, ROBOT.leg[L].want_Px, ROBOT.leg[L].want_Py, ROBOT.position_sensor[LB_MOTOR].position, -ROBOT.position_sensor[LF_MOTOR].position);
}
//=====================================
void print_data(){
  printf("----------------------------------------------------------------\n");
  printf("RW_P:[%f,%f]|||LW_P:[%f,%f]\n",ROBOT.motor[RB_MOTOR].want_P, ROBOT.motor[RF_MOTOR].want_P,
                                           ROBOT.motor[LB_MOTOR].want_P, ROBOT.motor[LF_MOTOR].want_P);
  printf("RW_XY:[%f,%f]|||LW_XY:[%f,%f]\n",ROBOT.leg[R].want_Px, ROBOT.leg[R].want_Py,
                                           ROBOT.leg[L].want_Px, ROBOT.leg[L].want_Py);
  printf("R_POS:[%f,%f]|||L_POS:[%f,%f]\n",ROBOT.position_sensor[RB_MOTOR].position,ROBOT.position_sensor[RF_MOTOR].position,
                                           ROBOT.position_sensor[LB_MOTOR].position,ROBOT.position_sensor[LF_MOTOR].position);
  printf("R足端位置:[%f,%f]|||L足端位置:[%f,%f]\n",ROBOT.leg[R].Px,ROBOT.leg[R].Py,ROBOT.leg[L].Px,ROBOT.leg[L].Py);
  printf("R足端速度:[%f,%f]|||L足端速度:[%f,%f]\n",ROBOT.leg[R].Vx,ROBOT.leg[R].Vy,ROBOT.leg[L].Vx,ROBOT.leg[L].Vy);
  printf("R电机力矩:[%f,%f]|||L电机力矩:[%f,%f]\n",ROBOT.motor[RB_MOTOR].torque, ROBOT.motor[RF_MOTOR].torque,
                                                  ROBOT.motor[LB_MOTOR].torque, ROBOT.motor[LF_MOTOR].torque);
  printf("----------------------------------------------------------------\n");
}
//=====================================
void perform_motor(){

  for(int i = 0; i < 4;i++){
    if(ROBOT.motor[i].torque > 35)
      ROBOT.motor[i].torque = 35;
    else if(ROBOT.motor[i].torque < -35)
      ROBOT.motor[i].torque = -35;
  }

  wb_motor_set_torque(ROBOT.motor[RB_MOTOR].ID,   1 * ROBOT.motor[RB_MOTOR].torque);
  wb_motor_set_torque(ROBOT.motor[RF_MOTOR].ID,   -1 * ROBOT.motor[RF_MOTOR].torque);
  wb_motor_set_torque(ROBOT.motor[LB_MOTOR].ID,   1 * ROBOT.motor[LB_MOTOR].torque);
  wb_motor_set_torque(ROBOT.motor[LF_MOTOR].ID,   -1 * ROBOT.motor[LF_MOTOR].torque);

  //wb_motor_set_available_torque(ROBOT.motor[R_MOTOR ].ID,  1 * ROBOT.motor[R_MOTOR ].torque);
  //wb_motor_set_available_torque(ROBOT.motor[L_MOTOR ].ID,  1 * ROBOT.motor[L_MOTOR ].torque);
}
//=====================================