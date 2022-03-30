#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <stdio.h>


#define TIME_STEP 64



//初始化----------------------------------------------------------
//底盘电机初始化函数
void Initialize_chassis(char *wheels){
  int i;
  char wheels_names[4][8] = {"MOTOR1","MOTOR2","MOTOR3","MOTOR4"};
  for (i = 0; i < 4; i++){
    *(wheels+i) = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(*(wheels+i),INFINITY);
  }
}
//角度传感器初始化函数
void Initialize_imu(char *imu_device){
  *imu_device = wb_robot_get_device("imu");
  wb_inertial_unit_enable(*imu_device, TIME_STEP);
}
//角速度传感器初始化函数
void Initialize_gyro(char *gyro_device){
  *gyro_device = wb_robot_get_device("gyro");
  wb_gyro_enable(*gyro_device, TIME_STEP);
}
//---------------------------------------------------------------


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

int main(int argc, char **argv) {
  wb_robot_init();

  double Speed =10;
  

  //底盘电机初始化
  char wheels[4];
  Initialize_chassis(&wheels[0]);

  //键盘控制初始化
  int ch;
  wb_keyboard_enable(TIME_STEP); 

  //角度传感器初始化
  char imu_device;
  double imu[3];
  Initialize_imu(&imu_device);

  //角速度传感器初始化
  char gyro_device;
  double gyro[3];
  Initialize_gyro(&gyro_device);

  
  
  //进入循环
  while (wb_robot_step(TIME_STEP) != -1) {  
    imu_get_data(&imu[0],imu_device);
    gyro_get_data(&gyro[0],gyro_device);
    
    printf("roll:%f\npitch:%f\nyaw:%f\n",imu[0],imu[1],imu[2]);
    printf("x:%f\ny:%f\nz:%f\n",gyro[0],gyro[1],gyro[2]);

    
    ch = wb_keyboard_get_key();
    switch (ch) {
      case 87://W
        wb_motor_set_velocity(wheels[0],Speed);
        wb_motor_set_velocity(wheels[1],Speed);
        wb_motor_set_velocity(wheels[2],Speed);
        wb_motor_set_velocity(wheels[3],Speed);
        break; 
      case 83://S
        wb_motor_set_velocity(wheels[0],-Speed);
        wb_motor_set_velocity(wheels[1],-Speed);
        wb_motor_set_velocity(wheels[2],-Speed);
        wb_motor_set_velocity(wheels[3],-Speed);
        break; 
      case 65://A
        wb_motor_set_velocity(wheels[0],-Speed);
        wb_motor_set_velocity(wheels[1],Speed);
        wb_motor_set_velocity(wheels[2],-Speed);
        wb_motor_set_velocity(wheels[3],Speed);
        break; 
      case 68://D
        wb_motor_set_velocity(wheels[0],Speed);
        wb_motor_set_velocity(wheels[1],-Speed);
        wb_motor_set_velocity(wheels[2],Speed);
        wb_motor_set_velocity(wheels[3],-Speed);
        break; 
      default:
        wb_motor_set_velocity(wheels[0],0);
        wb_motor_set_velocity(wheels[1],0);
        wb_motor_set_velocity(wheels[2],0);
        wb_motor_set_velocity(wheels[3],0);
        break;   
    }   
      ch = -1;
  };

  wb_robot_cleanup();

  return 0;
}
