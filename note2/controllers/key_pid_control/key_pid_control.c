//机器参数质心离轴0.1
//  总重mass = 10.02

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
#define PI 3.1415926 
#define MASS 1.02   //单位 kg
#define R 0.1        //单位 m
#define g 9.8        //单位 m/s^2

#include "include\initialize.h"
#include "include\sensor.h"


int main(int argc, char **argv) {
  wb_robot_init();

  //底盘电机初始化
  char wheels[2];
  double velocity;
  Initialize_chassis(&wheels[0]);

  //角度传感器初始化
  char imu_device;
  double imu[3];
  Initialize_imu(&imu_device);

  //角速度传感器初始化
  char gyro_device;
  double gyro[3];
  Initialize_gyro(&gyro_device);
  
  //加速度传感器初始化
  char accelerometer_device;
  double accelerometer[3];
  Initialize_accelerometer(&accelerometer_device);
  
  //磁力传感器初始化
  char compass_device;
  double compass[3];
  Initialize_compass(&compass_device);

  //编码器初始化
  char pos_device[2];
  double pos[2];
  Initialize_pos(&pos_device[0]);
  
  //PID 平衡环
  double balance_velocity = 0;
  double balance_k_p = 50;
  double balance_k_i = 2;
  double balance_k_d = 1000;
  double balance_I = 0;
  
  //PID 位置环
  double pos_velocity = 0;
  double pos_k_p = 1;
  double pos_k_i = 0.013;
  double pos_k_d = 0.75;
  double pos_I = 0;
  
  //键盘控制初始化
  int ch;
  wb_keyboard_enable(TIME_STEP); 
  double want_pos = 0;
  
  //-----------------------------------------------------------------------
  while (wb_robot_step(TIME_STEP) != -1) {
    //传感器状态读取
    imu_get_data(&imu[0],imu_device);
    gyro_get_data(&gyro[0],gyro_device);
    compass_get_data(&compass[0],compass_device);
    pos_get_data(&pos[0],&pos_device[0]);
    motor_sensor_get_data(&velocity,&wheels[0]);

    ch = wb_keyboard_get_key();
    if(ch == 68){
      want_pos = want_pos + 0.2;
    }
    else if(ch == 65){
      want_pos = want_pos - 0.2;
    }
    else if(ch == 83){
      want_pos = pos[0];
    }else{
      want_pos = want_pos;
    }
    
    
    if(imu[1] > 0){
      balance_k_d = -fabs(balance_k_d);
      pos_k_d = -fabs(pos_k_d);
    }  
    else if(imu[1] > 0){ 
      balance_k_d = fabs(balance_k_d); 
      pos_k_d = fabs(pos_k_d);
    }
    else{ 
      balance_k_d = balance_k_d; 
      pos_k_d = pos_k_d;
    }
    
    //倒立摆环
    balance_I = balance_I + imu[1] * balance_k_i;
    balance_velocity = balance_k_p * imu[1] - balance_k_d * gyro[1] + balance_I;
    
    //位置环
    pos_I = pos_I + (pos[0] - want_pos) * pos_k_i;
    pos_velocity = pos_k_p * (pos[0] - want_pos) - pos_k_d * velocity + pos_I;

    velocity = pos_velocity + balance_velocity;
    
    printf("---------------------------------\n");
    printf("imu:             %f %f %f\n",imu[0],imu[1],imu[2]);
    printf("compass:         %f %f %f\n",compass[0],compass[1],compass[2]);
    printf("gyro:            %f %f %f\n",gyro[0],gyro[1],gyro[2]);
    printf("pos:             %f %f\n"   ,pos[0],pos[1]);
    printf("pos_velocity:    %f\n"      ,pos_velocity);
    printf("balance_velocity:%f\n"      ,balance_velocity);
    printf("velocity:        %f\n"      ,velocity); 
    printf("want_pos:        %f\n"      ,want_pos);
    printf("---------------------------------\n");
    // want_velocity = 10;
    wb_motor_set_velocity(wheels[0], velocity); 
    wb_motor_set_velocity(wheels[1], velocity); 
     
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
