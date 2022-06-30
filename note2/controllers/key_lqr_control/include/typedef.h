#ifndef _TYPEDEF_
#define _TYPEDEF_

#define PI 3.1415926 
#define g 9.8    

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <webots/position_sensor.h>

#define L_WHEEL 0
#define R_WHEEL 1 

#define MOTOR_NUM 2

#define yaw 0
#define pitch 1 
#define roll 2

typedef struct motor_feature {
    WbDeviceTag ID;
    const char* name;
}MOTOR;

typedef struct position_sensor_feature {
    WbDeviceTag ID;
    const char* name;
    double resolusion;

    double position;//rad
    double position_last;

    double w;//rad/s
}POSITION_SENSOR;

typedef struct imu_feature{
    WbDeviceTag ID;
    const char* name;
    double angle_value[3]; //rad
}IMU;

typedef struct accelerometer_feature{
    WbDeviceTag accelerometer_ID;
    const char* accelerometer_name;
    double accelerometer_value[3];
}ACCE;

typedef struct gyro_feature{
    WbDeviceTag gyro_ID;
    const char* gyro_name;
    double gyro_value[3];
}GYRO;

typedef struct robot{
    double radius_of_wheel; //m  0.1 
    double height_of_center; //m 0.1
    double mass_body;  //kg 1.00
    double mass_wheel; //kg 0.01

    double velocity;  //m/s

    MOTOR motor[2];
    POSITION_SENSOR pos[2];

    IMU imu;
    ACCE acce;
    GYRO gyro;

} robot;

void Initialize_robot();
void init_pos();
void init_motor();
void update();
#endif