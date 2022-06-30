#ifndef _control_
#define _control_

void update() {
    int i;
    for (i = 0; i < MOTOR_NUM; i++) {
        assert(ROBOT.pos[i].ID);//´íÎó¼ì²â
        ROBOT.pos[i].position = wb_position_sensor_get_value(ROBOT.pos[i].ID);
        ROBOT.pos[i].w = ROBOT.pos[i].position - ROBOT.pos[i].position_last;
        ROBOT.pos[i].position_last = ROBOT.pos[i].position;
    }      

    for (i = 0; i < MOTOR_NUM; i++) {
        assert(ROBOT.imu.ID);//´íÎó¼ì²â
        ROBOT.imu.angle_value[yaw]   = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID));
        ROBOT.imu.angle_value[pitch] = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 1);
        ROBOT.imu.angle_value[roll]  = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 2);
    }
}

#endif