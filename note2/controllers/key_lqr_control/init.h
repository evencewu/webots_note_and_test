#ifndef _INIT_
#define _INIT_

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <webots/position_sensor.h>


void init_motor() {
    ROBOT.motor[R_WHEEL].name = "MOTOR1"; //右
    ROBOT.motor[L_WHEEL].name = "MOTOR2"; //左

    int i;
    for (i = 0; i < MOTOR_NUM; i++) {
        //获取电机ID
        ROBOT.motor[i].ID = wb_robot_get_device(ROBOT.motor[i].name);
        assert(ROBOT.motor[i].ID);
        //归零
        wb_motor_set_position(ROBOT.motor[i].ID, INFINITY);
        wb_motor_set_velocity(ROBOT.motor[i].ID, 0);
        printf("get motor %s succeed: %d\n", ROBOT.motor[i].name, ROBOT.motor[i].ID);
    }
}

//编码器初始化函数
void init_pos() {
    ROBOT.pos[R_WHEEL].name = "POS1";
    ROBOT.pos[L_WHEEL].name = "POS2";

    int i;
    for (i = 0; i < MOTOR_NUM; i++) {
        ROBOT.pos[i].ID = wb_robot_get_device(ROBOT.pos[i].name);
        assert(ROBOT.pos[i].ID);
        wb_position_sensor_enable(ROBOT.pos[i].ID, (int)TIME_STEP);
        ROBOT.pos[i].position = 0;
    }
}

void imu_init() {
    ROBOT.imu.name = "IMU";
    ROBOT.imu.ID = wb_robot_get_device(ROBOT.imu.name);
    wb_inertial_unit_enable(ROBOT.imu.ID, (int)TIME_STEP);
    ROBOT.imu.angle_value[yaw] = 0;
    ROBOT.imu.angle_value[pitch] = 0;
    ROBOT.imu.angle_value[roll] = 0;
};

void acce_init() {
    ROBOT.acce.accelerometer_name = "ACCE";
    ROBOT.acce.accelerometer_ID = wb_robot_get_device(ROBOT.acce.accelerometer_name);
    wb_accelerometer_enable(ROBOT.acce.accelerometer_ID, (int)TIME_STEP);
};

void gyro_init() {
    ROBOT.gyro.gyro_ID = wb_robot_get_device("GYRO");
    wb_gyro_enable(ROBOT.gyro.gyro_ID, (int)TIME_STEP);
    ROBOT.gyro.gyro_value[yaw] = 0;
    ROBOT.gyro.gyro_value[pitch] = 0;
    ROBOT.gyro.gyro_value[roll] = 0;
};

int ky;
void key_mouse_init() {
    wb_keyboard_enable(TIME_STEP);
};

#endif