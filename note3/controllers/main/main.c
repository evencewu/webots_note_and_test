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

robot_feature ROBOT;

#include "include\init.h"
#include "include\sensor.h"
#include "include\control.h"



int main(int argc, char **argv) {
  wb_robot_init();//系统初始化
  
  robot_init();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    control_master();  
    flash_sensor_data();//刷新全局传感器数据  
    print_data();
    perform_motor();

  };
  wb_robot_cleanup();
  return 0;
};

void print_data(){
  printf("----------------------------------------------------------------\n");
  printf("电机角度||RF:%f||RB:%f||LF:%f||LB:%f\n", ROBOT.position_sensor[RF].position, ROBOT.position_sensor[RB].position, ROBOT.position_sensor[LF].position, ROBOT.position_sensor[LB].position);
  printf("电机速度||RF:%f||RB:%f||LF:%f||LB:%f\n", ROBOT.position_sensor[RF].w       , ROBOT.position_sensor[RB].w       , ROBOT.position_sensor[LF].w       , ROBOT.position_sensor[LB].w       );
  printf("足端位置||RX:%f||RY:%f||LX:%f||LY:%f\n", ROBOT.leg[R].Px                   , ROBOT.leg[R].Py                   , ROBOT.leg[L].Px                   , ROBOT.leg[L].Py                   );

  printf("电机目标||RF:%f||RB:%f||LF:%f||LB:%f\n", ROBOT.motor[RF].want_P            , ROBOT.motor[RB].want_P            , ROBOT.motor[LF].want_P            , ROBOT.motor[LB].want_P            );
  printf("足端目标||RX:%f||RY:%f||LX:%f||LY:%f\n", ROBOT.leg[R].want_Px              , ROBOT.leg[R].want_Py              , ROBOT.leg[L].want_Px              , ROBOT.leg[L].want_Py              );

  printf("电机力矩||RF:%f||RB:%f||LF:%f||LB:%f\n", ROBOT.motor[RF].torque            , ROBOT.motor[RB].torque            , ROBOT.motor[LF].torque            , ROBOT.motor[LB].torque            );
  printf("反馈力矩||RF:%f||RB:%f||LF:%f||LB:%f\n", ROBOT.motor[RF].torque_fb         , ROBOT.motor[RB].torque_fb         , ROBOT.motor[LF].torque_fb         , ROBOT.motor[LB].torque_fb         );
}

void perform_motor(){
  for(int i = 2; i < 6;i++){
    if(ROBOT.motor[i].torque > 35)
      ROBOT.motor[i].torque = 35;
    else if(ROBOT.motor[i].torque < -35)
      ROBOT.motor[i].torque = -35;
  }

  wb_motor_set_torque(ROBOT.motor[RB].ID,   1 * ROBOT.motor[RB].torque);
  wb_motor_set_torque(ROBOT.motor[RF].ID,  -1 * ROBOT.motor[RF].torque);
  wb_motor_set_torque(ROBOT.motor[LB].ID,   1 * ROBOT.motor[LB].torque);
  wb_motor_set_torque(ROBOT.motor[LF].ID,  -1 * ROBOT.motor[LF].torque);
}