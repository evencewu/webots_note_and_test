#ifndef _INITIALIZE_
#define _INITIALIZE_



//电机初始化
void motor_init(double angle_set){
  ROBOT.motor[RB_MOTOR].name = "RB_MOTOR";
  ROBOT.motor[RF_MOTOR].name = "RF_MOTOR";
  ROBOT.motor[LB_MOTOR].name = "LB_MOTOR";
  ROBOT.motor[LF_MOTOR].name = "LF_MOTOR";
  ROBOT.motor[R_MOTOR ].name = "R_MOTOR"; //右
  ROBOT.motor[L_MOTOR ].name = "L_MOTOR"; //左

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    //获取电机ID
    ROBOT.motor[i].ID = wb_robot_get_device(ROBOT.motor[i].name);
    assert(ROBOT.motor[i].ID);
    //获取最大扭矩
    ROBOT.motor[i].MAX_TORQUE = wb_motor_get_max_torque(ROBOT.motor[i].ID);
    //printf("max_t%f\n",ROBOT.motor[i].MAX_TORQUE);
    //使能扭矩反馈
    int sampling_period;
    sampling_period = TIME_STEP;// wb_motor_get_torque_feedback_sampling_period(ROBOT.motor[i].ID);
    wb_motor_enable_torque_feedback(ROBOT.motor[i].ID, sampling_period);
    //归零
    ROBOT.motor[i].torque = 0;
    ROBOT.motor[i].angle = angle_set;
    ROBOT.motor[i].torque_last = 0;
    ROBOT.motor[i].I = 0;
    wb_motor_set_torque(ROBOT.motor[i].ID,0);
    printf("get motor %s succeed: %d\n", ROBOT.motor[i].name, ROBOT.motor[i].ID);
  }

};

void position_sensor_init(){
  ROBOT.position_sensor[RB_MOTOR].name = "RB_POS";
  ROBOT.position_sensor[RF_MOTOR].name = "RF_POS";
  ROBOT.position_sensor[LB_MOTOR].name = "LB_POS";
  ROBOT.position_sensor[LF_MOTOR].name = "LF_POS";
  ROBOT.position_sensor[R_MOTOR].name = "R_POS";
  ROBOT.position_sensor[L_MOTOR].name = "L_POS"; 

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
  ROBOT.position_sensor[i].ID = wb_robot_get_device(ROBOT.position_sensor[i].name);
  assert(ROBOT.position_sensor[i].ID);
  wb_position_sensor_enable(ROBOT.position_sensor[i].ID, (int)TIME_STEP);

  ROBOT.position_sensor[i].position = 0;
  ROBOT.position_sensor[i].position_last = 0;

  ROBOT.position_sensor[i].w = 0;
  ROBOT.position_sensor[i].w_last = 0;
  }
};

void imu_init(){
  ROBOT.imu.name = "IMU";
  ROBOT.imu.ID = wb_robot_get_device(ROBOT.imu.name);
  wb_inertial_unit_enable(ROBOT.imu.ID, (int)TIME_STEP);
  ROBOT.imu.angle_value[yaw  ] = 0;
  ROBOT.imu.angle_value[pitch] = 0;
  ROBOT.imu.angle_value[roll ] = 0;
};

void acce_init(){
  ROBOT.acce.accelerometer_name = "ACCE";
  ROBOT.acce.accelerometer_ID = wb_robot_get_device(ROBOT.acce.accelerometer_name);
  wb_accelerometer_enable(ROBOT.acce.accelerometer_ID, (int)TIME_STEP);
};

void gyro_init(){
  ROBOT.gyro.gyro_ID = wb_robot_get_device("GYRO");
  wb_gyro_enable(ROBOT.gyro.gyro_ID, (int)TIME_STEP);
  ROBOT.gyro.gyro_value[yaw  ] = 0;
  ROBOT.gyro.gyro_value[pitch] = 0;
  ROBOT.gyro.gyro_value[roll ] = 0;
};

int ky;
void key_mouse_init(){
  wb_keyboard_enable(TIME_STEP); 
};

#endif