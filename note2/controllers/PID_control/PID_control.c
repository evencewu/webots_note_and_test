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
#define PI 3.1415926 
#define MASS 1.02   //单位 kg
#define R 0.1        //单位 m
#define g 9.8        //单位 m/s^2

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


//-----------------------------------------------------------------------
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

//-----------------------------------------------------------------------
int main(int argc, char **argv) {
  wb_robot_init();

  //底盘电机初始化
  char wheels[2];
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
  double pos_k_i = 0.01;
  double pos_k_d = 0.57;
  double pos_I = 0;
  
  double velocity = 0;
  
  //-----------------------------------------------------------------------
  while (wb_robot_step(TIME_STEP) != -1) {
    //传感器状态读取
    imu_get_data(&imu[0],imu_device);
    gyro_get_data(&gyro[0],gyro_device);
    compass_get_data(&compass[0],compass_device);
    pos_get_data(&pos[0],&pos_device[0]);
    
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
    pos_I = pos_I + pos[0] * pos_k_i;
    pos_velocity = pos_k_p * pos[0] - pos_k_d * wb_motor_get_velocity(wheels[0]) + pos_I;

    velocity = pos_velocity + balance_velocity;
    
    printf("---------------------------------\n");
    printf("imu:             %f %f %f\n",imu[0],imu[1],imu[2]);
    printf("compass:         %f %f %f\n",compass[0],compass[1],compass[2]);
    printf("gyro:            %f %f %f\n",gyro[0],gyro[1],gyro[2]);
    printf("pos:             %f %f\n"   ,pos[0],pos[1]);
    printf("pos_velocity:    %f\n"      ,pos_velocity);
    printf("balance_velocity:%f\n"      ,balance_velocity);
    printf("velocity:        %f\n"      ,velocity); 
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
