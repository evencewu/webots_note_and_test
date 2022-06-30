
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <webots/position_sensor.h>

#define TIME_STEP 10

#include "typedef.h"
robot ROBOT;

#include "init.h"
#include "control.h"
int main(int argc, char** argv) {
    wb_robot_init();

    Initialize_robot();

    while (wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(ROBOT.motor[R_WHEEL].ID, 10);
        wb_motor_set_velocity(ROBOT.motor[L_WHEEL].ID, 10);

    };

    wb_robot_cleanup();

    return 0;
}

void Initialize_robot() {
    init_motor();
    init_pos();
}

